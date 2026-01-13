/**
 * @file lead.cpp
 * @brief Leader Truck - Platoon controller
 * 
 * Functions:
 *   - TCP server: Accept join requests from followers
 *   - UDP broadcast: Send control state to followers
 *   - UDP receiver: Receive state from followers
 */

#include "lead.h"
#include "message.h"

#include <iostream>
#include <iomanip>
#include <thread>
#include <vector>
#include <mutex>
#include <chrono>
#include <sstream>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>

// ============== LOGGING ==============
std::mutex log_mtx;

// ANSI escape codes
#define CLEAR_SCREEN "\033[2J\033[H"
#define MOVE_TO(row, col) "\033[" << row << ";" << col << "H"
#define CLEAR_LINE "\033[K"
#define COLOR_GREEN "\033[32m"
#define COLOR_YELLOW "\033[33m"
#define COLOR_RED "\033[31m"
#define COLOR_CYAN "\033[36m"
#define COLOR_RESET "\033[0m"
#define COLOR_BOLD "\033[1m"

// Dashboard mode flag
bool dashboard_mode = false;
int event_line = 15;  // Line for scrolling events

std::string getTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
    
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time), "%H:%M:%S");
    ss << "." << std::setfill('0') << std::setw(3) << ms.count();
    return ss.str();
}

void log(const std::string& component, const std::string& msg) {
    std::lock_guard<std::mutex> lock(log_mtx);
    std::cout << "[" << getTimestamp() << "] [" << component << "] " << msg << std::endl;
}

void logEvent(const std::string& msg) {
    std::lock_guard<std::mutex> lock(log_mtx);
    if (dashboard_mode) {
        std::cout << MOVE_TO(event_line, 1) << CLEAR_LINE;
        std::cout << COLOR_YELLOW << "[" << getTimestamp() << "] " << msg << COLOR_RESET << std::flush;
    } else {
        std::cout << "[" << getTimestamp() << "] [EVENT] " << msg << std::endl;
    }
}

void logSeparator(const std::string& title) {
    std::lock_guard<std::mutex> lock(log_mtx);
    if (title.empty()) {
        std::cout << "────────────────────────────────────────────────" << std::endl;
    } else {
        std::cout << "══════════ " << title << " ══════════" << std::endl;
    }
}

// ============== GLOBAL STATE ==============
std::mutex platoon_mtx;

struct FollowerInfo {
    int32_t index;
    std::string vehicle_id;
    double speed;
    double position;
    double distance_to_leader;
    uint64_t last_seen_ms;
};

std::vector<FollowerInfo> followers;
int next_index = 1;

// Leader state
double leader_speed = 80.0;       // km/h
double leader_position = 0.0;     // m
double leader_acceleration = 0.0; // m/s²
bool brake_active = false;
bool emergency = false;
uint32_t broadcast_seq = 0;

// ============== UTILITY ==============
uint64_t nowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

// Dashboard display (must be after global state)
void updateDashboard() {
    std::lock_guard<std::mutex> lock(log_mtx);
    std::cout << MOVE_TO(1, 1);
    
    // Header
    std::cout << COLOR_BOLD << "╔══════════════════════════════════════════════════════════════╗" << COLOR_RESET << "\n";
    std::cout << COLOR_BOLD << "║            🚛  LEADER TRUCK DASHBOARD  🚛                    ║" << COLOR_RESET << "\n";
    std::cout << COLOR_BOLD << "╠══════════════════════════════════════════════════════════════╣" << COLOR_RESET << "\n";
    
    // Leader state
    std::cout << "║  " << COLOR_CYAN << "Speed:" << COLOR_RESET << " ";
    std::cout << std::fixed << std::setprecision(1) << std::setw(6) << leader_speed << " km/h";
    std::cout << "  │  " << COLOR_CYAN << "Position:" << COLOR_RESET << " ";
    std::cout << std::setw(8) << leader_position << " m";
    std::cout << "           ║\n";
    
    std::cout << "║  " << COLOR_CYAN << "Brake:" << COLOR_RESET << " ";
    if (brake_active) {
        std::cout << COLOR_RED << "  ON  " << COLOR_RESET;
    } else {
        std::cout << COLOR_GREEN << " OFF  " << COLOR_RESET;
    }
    std::cout << "       │  " << COLOR_CYAN << "Emergency:" << COLOR_RESET << " ";
    if (emergency) {
        std::cout << COLOR_RED << "ACTIVE" << COLOR_RESET;
    } else {
        std::cout << COLOR_GREEN << "  NO  " << COLOR_RESET;
    }
    std::cout << "              ║\n";
    
    std::cout << "╠══════════════════════════════════════════════════════════════╣\n";
    std::cout << "║  " << COLOR_BOLD << "FOLLOWERS (" << followers.size() << ")" << COLOR_RESET << "                                              ║\n";
    std::cout << "╠══════════════════════════════════════════════════════════════╣\n";
    
    // Follower list
    if (followers.empty()) {
        std::cout << "║  (No followers yet)                                          ║\n";
    } else {
        for (const auto& f : followers) {
            std::cout << "║  #" << f.index << " [" << std::setw(12) << std::left << f.vehicle_id << std::right << "] ";
            std::cout << "Spd:" << std::setw(5) << f.speed << " km/h  ";
            std::cout << "Gap:" << std::setw(5) << f.distance_to_leader << " m    ║\n";
        }
    }
    
    std::cout << "╚══════════════════════════════════════════════════════════════╝\n";
    std::cout << std::flush;
}

// ============== TCP: MESSAGE HANDLERS ==============

/**
 * @brief Handle JOIN_REQUEST from a follower
 * @param client_socket Socket to send response
 * @param buffer Raw message buffer
 * @param len Length of received data
 * @return true if handled successfully
 */
bool handleJoinRequest(int client_socket, const uint8_t* buffer, ssize_t len) {
    if (len != sizeof(JoinRequest)) {
        return false;
    }
    
    const JoinRequest* req = reinterpret_cast<const JoinRequest*>(buffer);
    
    std::lock_guard<std::mutex> lock(platoon_mtx);
    
    // Register follower
    FollowerInfo info;
    info.index = next_index++;
    info.vehicle_id = std::string(req->vehicle_id);
    info.speed = req->speed;
    info.position = req->position;
    info.distance_to_leader = leader_position - req->position;
    info.last_seen_ms = nowMs();
    followers.push_back(info);
    
    // Send response
    JoinResponse resp{};
    resp.type = static_cast<uint8_t>(MsgType::JOIN_RESPONSE);
    resp.accepted = 1;
    resp.assigned_index = info.index;
    snprintf(resp.message, sizeof(resp.message), "Welcome to platoon, position #%d", info.index);
    
    send(client_socket, &resp, sizeof(resp), 0);
    
    logEvent("✅ JOIN: " + info.vehicle_id + " → Position #" + std::to_string(info.index));
    return true;
}

/**
 * @brief Handle LEAVE_REQUEST from a follower
 * @param client_socket Socket to send response
 * @param buffer Raw message buffer
 * @param len Length of received data
 * @return true if handled successfully
 */
bool handleLeaveRequest(int client_socket, const uint8_t* buffer, ssize_t len) {
    if (len != sizeof(LeaveRequest)) {
        return false;
    }
    
    const LeaveRequest* req = reinterpret_cast<const LeaveRequest*>(buffer);
    
    std::lock_guard<std::mutex> lock(platoon_mtx);
    
    // Find and remove follower
    bool found = false;
    std::string leaving_id;
    int leaving_index = req->vehicle_index;
    
    for (auto it = followers.begin(); it != followers.end(); ++it) {
        if (it->index == req->vehicle_index) {
            leaving_id = it->vehicle_id;
            followers.erase(it);
            found = true;
            break;
        }
    }
    
    // Send response
    LeaveResponse resp{};
    resp.type = static_cast<uint8_t>(MsgType::LEAVE_RESPONSE);
    
    if (found) {
        resp.success = 1;
        snprintf(resp.message, sizeof(resp.message), "Goodbye, removed from platoon");
        
        const char* reason_str = "normal";
        if (req->reason == 1) reason_str = "emergency";
        else if (req->reason == 2) reason_str = "maintenance";
        
        logEvent("🚪 LEAVE: " + leaving_id + " (Position #" + std::to_string(leaving_index) + 
                 ", reason: " + reason_str + ")");
    } else {
        resp.success = 0;
        snprintf(resp.message, sizeof(resp.message), "Vehicle not found in platoon");
        logEvent("⚠️ LEAVE failed: Vehicle #" + std::to_string(req->vehicle_index) + " not found");
    }
    
    send(client_socket, &resp, sizeof(resp), 0);
    return found;
}

// ============== TCP: MANAGEMENT SERVER ==============
void tcp_management_server() {
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
        log("TCP", "❌ Failed to create socket");
        return;
    }
    
    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(TCP_PORT);

    if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) < 0) {
        log("TCP", "❌ Bind failed on port " + std::to_string(TCP_PORT));
        close(server_fd);
        return;
    }
    
    listen(server_fd, 5);
    log("TCP", "✅ Listening on port " + std::to_string(TCP_PORT) + " for management requests");

    while (true) {
        sockaddr_in client_addr{};
        socklen_t client_len = sizeof(client_addr);
        int client_socket = accept(server_fd, (struct sockaddr*)&client_addr, &client_len);
        
        if (client_socket < 0) {
            log("TCP", "⚠️  Accept failed");
            continue;
        }
        
        char client_ip[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &client_addr.sin_addr, client_ip, INET_ADDRSTRLEN);
        
        // Receive message (use max size of all possible messages)
        uint8_t buffer[256];
        ssize_t n = read(client_socket, buffer, sizeof(buffer));
        
        if (n > 0) {
            // First byte is always message type
            MsgType msg_type = static_cast<MsgType>(buffer[0]);
            
            switch (msg_type) {
                case MsgType::JOIN_REQUEST:
                    if (!handleJoinRequest(client_socket, buffer, n)) {
                        log("TCP", "⚠️ Invalid JOIN_REQUEST from " + std::string(client_ip));
                    }
                    break;
                    
                case MsgType::LEAVE_REQUEST:
                    if (!handleLeaveRequest(client_socket, buffer, n)) {
                        log("TCP", "⚠️ Invalid LEAVE_REQUEST from " + std::string(client_ip));
                    }
                    break;
                    
                default:
                    log("TCP", "⚠️ Unknown message type " + std::to_string(buffer[0]) + 
                        " from " + std::string(client_ip));
                    break;
            }
        }
        
        close(client_socket);
    }
}

// ============== UDP: BROADCAST LEADER STATE ==============
void udp_broadcast_server() {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        log("UDP-TX", "❌ Failed to create socket");
        return;
    }
    
    // Enable broadcast
    int broadcast = 1;
    setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));
    
    sockaddr_in broadcast_addr{};
    broadcast_addr.sin_family = AF_INET;
    broadcast_addr.sin_port = htons(UDP_LEADER_PORT);
    broadcast_addr.sin_addr.s_addr = inet_addr("127.255.255.255"); // Broadcast local
    
    log("UDP-TX", "✅ Broadcasting on port " + std::to_string(UDP_LEADER_PORT));

    while (true) {
        // Update leader position (simulation)
        {
            std::lock_guard<std::mutex> lock(platoon_mtx);
            leader_position += (leader_speed / 3.6) * 0.1; // 100ms interval
        }
        
        // Build message
        LeaderState msg{};
        msg.type = static_cast<uint8_t>(MsgType::LEADER_STATE);
        msg.sequence = ++broadcast_seq;
        msg.timestamp_ms = nowMs();
        msg.speed = leader_speed;
        msg.acceleration = leader_acceleration;
        msg.position = leader_position;
        msg.brake_active = brake_active ? 1 : 0;
        msg.emergency = emergency ? 1 : 0;
        
        sendto(sock, &msg, sizeof(msg), 0, 
               (struct sockaddr*)&broadcast_addr, sizeof(broadcast_addr));
        
        // Update dashboard every 500ms (5 messages)
        if (dashboard_mode && broadcast_seq % 5 == 0) {
            updateDashboard();
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// ============== UDP: RECEIVE FOLLOWER STATE ==============
void udp_receive_follower_state() {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        log("UDP-RX", "❌ Failed to create socket");
        return;
    }
    
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(UDP_FOLLOWER_PORT);
    
    if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        log("UDP-RX", "❌ Bind failed on port " + std::to_string(UDP_FOLLOWER_PORT));
        close(sock);
        return;
    }
    
    log("UDP-RX", "✅ Listening on port " + std::to_string(UDP_FOLLOWER_PORT) + " for follower states");

    while (true) {
        FollowerState msg{};
        sockaddr_in from{};
        socklen_t from_len = sizeof(from);
        
        ssize_t n = recvfrom(sock, &msg, sizeof(msg), 0, 
                            (struct sockaddr*)&from, &from_len);
        
        // Handle more messages here, for example a follower truck faces an ostacle
        if (n == sizeof(msg) && msg.type == static_cast<uint8_t>(MsgType::FOLLOWER_STATE)) {
            std::lock_guard<std::mutex> lock(platoon_mtx);
            
            // Update follower info
            bool found = false;
            for (auto& f : followers) {
                if (f.index == msg.vehicle_index) {
                    f.speed = msg.speed;
                    f.position = msg.position;
                    f.distance_to_leader = msg.distance_to_leader;
                    f.last_seen_ms = nowMs();
                    found = true;
                    break;
                }
            }
            
            // Log only on status change or warning
            if (msg.status != 0) {
                logEvent("⚠️ [" + std::string(msg.vehicle_id) + "] Status warning!");
            }
        }
    }
}

// ============== MAIN ==============
int main() {
    logSeparator("LEADER TRUCK STARTING");
    log("MAIN", "Initializing platooning leader...");
    log("MAIN", "TCP Port: " + std::to_string(TCP_PORT) + " (Join/Leave)");
    log("MAIN", "UDP Port: " + std::to_string(UDP_LEADER_PORT) + " (Broadcast)");
    log("MAIN", "UDP Port: " + std::to_string(UDP_FOLLOWER_PORT) + " (Receive)");
    log("MAIN", "Starting dashboard mode in 2 seconds...");
    logSeparator("");
    
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    // Clear screen and enable dashboard mode
    std::cout << CLEAR_SCREEN << std::flush;
    dashboard_mode = true;
    
    // Launch threads
    std::thread tcp_thread(tcp_management_server);
    std::thread udp_tx_thread(udp_broadcast_server);
    std::thread udp_rx_thread(udp_receive_follower_state);

    tcp_thread.join();
    udp_tx_thread.join();
    udp_rx_thread.join();
    
    return 0;
}