/**
 * @file follow.cpp
 * @brief Follower Truck - Platoon member
 * 
 * Functions:
 *   - TCP client: Send join request to leader
 *   - UDP receiver: Receive state from leader
 *   - UDP sender: Send state to leader
 */

#include "follow.h"
#include "message.h"

#include <iostream>
#include <iomanip>
#include <thread>
#include <mutex>
#include <atomic>
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
int event_line = 14;

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
std::mutex state_mtx;

// Follower info (assigned after join)
int32_t my_index = -1;
char my_vehicle_id[32] = "Follower_01";

// Follower state
double my_speed = 0.0;          // km/h
double my_position = 0.0;       // m
double distance_to_leader = 50.0; // m (initial gap)
uint8_t my_status = 0;          // 0 = OK

// Leader state (received)
double leader_speed = 0.0;
double leader_position = 0.0;
bool leader_brake = false;
bool leader_emergency = false;
uint32_t last_leader_seq = 0;

std::atomic<bool> joined{false};
std::atomic<bool> running{true};
uint32_t send_seq = 0;

// Leader IP (set from command line)
const char* g_leader_ip = "127.0.0.1";

// ============== UTILITY ==============
uint64_t nowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

// Dashboard display for follower (must be after global state)
void updateFollowerDashboard() {
    std::lock_guard<std::mutex> lock(log_mtx);
    std::cout << MOVE_TO(1, 1);
    
    // Header
    std::cout << COLOR_BOLD << "╔══════════════════════════════════════════════════════════════╗" << COLOR_RESET << "\n";
    std::cout << COLOR_BOLD << "║            🚚  FOLLOWER TRUCK DASHBOARD  🚚                  ║" << COLOR_RESET << "\n";
    std::cout << COLOR_BOLD << "║  ID: " << std::setw(12) << std::left << my_vehicle_id << std::right << "    Index: #" << my_index << "                             ║" << COLOR_RESET << "\n";
    std::cout << COLOR_BOLD << "╠══════════════════════════════════════════════════════════════╣" << COLOR_RESET << "\n";
    
    // My state
    std::cout << "║  " << COLOR_CYAN << "My Speed:" << COLOR_RESET << " ";
    std::cout << std::fixed << std::setprecision(1) << std::setw(6) << my_speed << " km/h";
    std::cout << "  │  " << COLOR_CYAN << "Position:" << COLOR_RESET << " ";
    std::cout << std::setw(8) << my_position << " m";
    std::cout << "         ║\n";
    
    // Gap visualization
    std::cout << "║  " << COLOR_CYAN << "Gap to Leader:" << COLOR_RESET << " ";
    double gap = distance_to_leader;
    if (gap < 15) {
        std::cout << COLOR_RED;
    } else if (gap < 25) {
        std::cout << COLOR_GREEN;
    } else {
        std::cout << COLOR_YELLOW;
    }
    std::cout << std::setw(6) << gap << " m " << COLOR_RESET;
    
    // Gap bar [====|====]
    std::cout << "[";
    int bar_pos = std::min(20, std::max(0, (int)(gap)));
    for (int i = 0; i < 20; i++) {
        if (i == 10) std::cout << "|";
        else if (i == bar_pos) std::cout << "█";
        else std::cout << "─";
    }
    std::cout << "]               ║\n";
    
    std::cout << "╠══════════════════════════════════════════════════════════════╣\n";
    std::cout << "║  " << COLOR_BOLD << "LEADER STATE" << COLOR_RESET << "                                                ║\n";
    std::cout << "╠══════════════════════════════════════════════════════════════╣\n";
    
    // Leader state
    std::cout << "║  " << COLOR_CYAN << "Leader Speed:" << COLOR_RESET << " ";
    std::cout << std::setw(6) << leader_speed << " km/h";
    std::cout << "  │  " << COLOR_CYAN << "Brake:" << COLOR_RESET << " ";
    if (leader_brake) {
        std::cout << COLOR_RED << " ON " << COLOR_RESET;
    } else {
        std::cout << COLOR_GREEN << "OFF " << COLOR_RESET;
    }
    std::cout << "  │  " << COLOR_CYAN << "Emerg:" << COLOR_RESET << " ";
    if (leader_emergency) {
        std::cout << COLOR_RED << "YES" << COLOR_RESET;
    } else {
        std::cout << COLOR_GREEN << " NO" << COLOR_RESET;
    }
    std::cout << "      ║\n";
    
    std::cout << "║  " << COLOR_CYAN << "Seq #:" << COLOR_RESET << " " << std::setw(8) << last_leader_seq;
    std::cout << "                                           ║\n";
    
    std::cout << "╠══════════════════════════════════════════════════════════════╣\n";
    std::cout << "║  " << COLOR_CYAN << "LAST EVENT:" << COLOR_RESET << "                                                 ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════════╝\n";
    std::cout << std::flush;
}

// ============== TCP: JOIN REQUEST ==============
bool request_join(const char* leader_ip, const char* vehicle_id) {
    log("TCP", "Connecting to leader at " + std::string(leader_ip) + ":" + std::to_string(TCP_PORT));
    
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        log("TCP", "❌ Failed to create socket");
        return false;
    }
    
    sockaddr_in serv_addr{};
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(TCP_PORT);
    
    if (inet_pton(AF_INET, leader_ip, &serv_addr.sin_addr) <= 0) {
        log("TCP", "❌ Invalid IP address");
        close(sock);
        return false;
    }

    if (connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
        log("TCP", "❌ Connection failed - Is leader running?");
        close(sock);
        return false;
    }
    
    log("TCP", "✅ Connected to leader");
    
    // Send JOIN request
    JoinRequest req{};
    req.type = static_cast<uint8_t>(MsgType::JOIN_REQUEST);
    strncpy(req.vehicle_id, vehicle_id, sizeof(req.vehicle_id) - 1);
    req.position = my_position;
    req.speed = my_speed;
    
    send(sock, &req, sizeof(req), 0);
    log("TCP", "📤 Sent JOIN_REQUEST");
    
    // Receive response
    JoinResponse resp{};
    ssize_t n = read(sock, &resp, sizeof(resp));
    
    close(sock);
    
    if (n == sizeof(resp) && resp.type == static_cast<uint8_t>(MsgType::JOIN_RESPONSE)) {
        if (resp.accepted) {
            my_index = resp.assigned_index;
            strncpy(my_vehicle_id, vehicle_id, sizeof(my_vehicle_id) - 1);
            
            // Initialize gap based on index (20m per position)
            distance_to_leader = my_index * 20.0;
            
            log("TCP", "✅ JOIN ACCEPTED");
            log("TCP", "   ├─ Assigned Index: #" + std::to_string(my_index));
            log("TCP", "   ├─ Target Gap: " + std::to_string((int)distance_to_leader) + " m");
            log("TCP", "   └─ Message: " + std::string(resp.message));
            return true;
        } else {
            log("TCP", "❌ JOIN REJECTED: " + std::string(resp.message));
            return false;
        }
    }
    
    log("TCP", "❌ Invalid response from leader");
    return false;
}

// ============== UDP: RECEIVE LEADER STATE ==============
void udp_receive_leader_state() {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        log("UDP-RX", "❌ Failed to create socket");
        return;
    }
    
    // Allow multiple processes to bind to the same port (for multiple followers)
    int opt = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    setsockopt(sock, SOL_SOCKET, SO_REUSEPORT, &opt, sizeof(opt));
    
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(UDP_LEADER_PORT);
    
    if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        log("UDP-RX", "❌ Bind failed on port " + std::to_string(UDP_LEADER_PORT));
        close(sock);
        return;
    }
    
    log("UDP-RX", "✅ Listening on port " + std::to_string(UDP_LEADER_PORT) + " for leader broadcasts");

    while (running.load()) {
        LeaderState msg{};
        ssize_t n = recvfrom(sock, &msg, sizeof(msg), 0, nullptr, nullptr);
        
        if (n == sizeof(msg) && msg.type == static_cast<uint8_t>(MsgType::LEADER_STATE)) {
            std::lock_guard<std::mutex> lock(state_mtx);
            
            // Check sequence for packet loss
            if (last_leader_seq > 0 && msg.sequence > last_leader_seq + 1) {
                uint32_t lost = msg.sequence - last_leader_seq - 1;
                log("UDP-RX", "⚠️  Lost " + std::to_string(lost) + " packets");
            }
            last_leader_seq = msg.sequence;
            
            // Update leader state
            leader_speed = msg.speed;
            leader_position = msg.position;
            leader_brake = msg.brake_active != 0;
            leader_emergency = msg.emergency != 0;
            
            // Simple following logic: adjust own speed
            distance_to_leader = leader_position - my_position;
            
            // Target gap based on position in platoon (20m per index)
            // Follower #1 = 20m, #2 = 40m, #3 = 60m, etc.
            double target_gap = my_index * 20.0;
            double gap_error = distance_to_leader - target_gap;
            
            // Simple proportional control
            my_speed = leader_speed + (gap_error * 0.5);
            if (my_speed < 0) my_speed = 0;
            if (my_speed > 120) my_speed = 120;
            
            // Update position
            my_position += (my_speed / 3.6) * 0.1; // 100ms interval
            
            // Handle brake/emergency
            if (leader_brake) {
                my_speed *= 0.9; // Slow down
                logEvent("🛑 Leader braking!");
            }
            if (leader_emergency) {
                my_speed = 0;
                my_status = 1; // Warning
                logEvent("🚨 EMERGENCY from leader!");
            }
            
            // Update dashboard every 5 messages (500ms)
            if (dashboard_mode && msg.sequence % 5 == 0) {
                updateFollowerDashboard();
            }
            
            // Log packet loss
            if (last_leader_seq > 0 && msg.sequence > last_leader_seq + 1) {
                uint32_t lost = msg.sequence - last_leader_seq - 1;
                logEvent("⚠️ Lost " + std::to_string(lost) + " packets");
            }
        }
    }
    
    close(sock);
}

// ============== UDP: SEND FOLLOWER STATE ==============
void udp_send_follower_state() {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        log("UDP-TX", "❌ Failed to create socket");
        return;
    }
    
    sockaddr_in leader_addr{};
    leader_addr.sin_family = AF_INET;
    leader_addr.sin_port = htons(UDP_FOLLOWER_PORT);
    leader_addr.sin_addr.s_addr = inet_addr(g_leader_ip);
    
    log("UDP-TX", "✅ Sending state to leader (" + std::string(g_leader_ip) + ":" + std::to_string(UDP_FOLLOWER_PORT) + ")");

    while (running.load()) {
        if (!joined.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            continue;
        }
        
        FollowerState msg{};
        {
            std::lock_guard<std::mutex> lock(state_mtx);
            msg.type = static_cast<uint8_t>(MsgType::FOLLOWER_STATE);
            msg.sequence = ++send_seq;
            msg.timestamp_ms = nowMs();
            msg.vehicle_index = my_index;
            strncpy(msg.vehicle_id, my_vehicle_id, sizeof(msg.vehicle_id) - 1);
            msg.speed = my_speed;
            msg.position = my_position;
            msg.distance_to_leader = distance_to_leader;
            msg.status = my_status;
        }
        
        sendto(sock, &msg, sizeof(msg), 0, 
               (struct sockaddr*)&leader_addr, sizeof(leader_addr));
        
        std::this_thread::sleep_for(std::chrono::milliseconds(500)); // 2Hz
    }
    
    close(sock);
}

// ============== MAIN ==============
int main(int argc, char* argv[]) {
    // Parse arguments
    const char* leader_ip = "127.0.0.1";
    const char* vehicle_id = "Follower_01";
    
    if (argc >= 2) vehicle_id = argv[1];
    if (argc >= 3) leader_ip = argv[2];
    
    // Set global leader IP for UDP thread
    g_leader_ip = leader_ip;
    
    logSeparator("FOLLOWER TRUCK STARTING");
    log("MAIN", "Vehicle ID: " + std::string(vehicle_id));
    log("MAIN", "Leader IP: " + std::string(leader_ip));
    logSeparator("");
    
    // Step 1: Join platoon via TCP
    if (!request_join(leader_ip, vehicle_id)) {
        log("MAIN", "❌ Failed to join platoon. Exiting.");
        return 1;
    }
    
    joined.store(true);
    logSeparator("JOINED PLATOON");
    log("MAIN", "Starting dashboard mode in 2 seconds...");
    
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    // Clear screen and enable dashboard mode  
    std::cout << CLEAR_SCREEN << std::flush;
    dashboard_mode = true;
    
    // Step 2: Start UDP threads
    std::thread rx_thread(udp_receive_leader_state);
    std::thread tx_thread(udp_send_follower_state);
    
    rx_thread.join();
    tx_thread.join();
    
    return 0;
}