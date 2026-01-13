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
#include <string>
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
int input_line = 16;  // Line for command input

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
std::atomic<bool> position_initialized{false};  // Flag to init position once
uint32_t send_seq = 0;

// Leader IP (set from command line)
std::string g_leader_ip = "127.0.0.1";

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
    double target_gap = my_index * 20.0;  // Target: 20m per index
    double gap_error = std::abs(gap - target_gap);
    
    // Color based on how close we are to target gap
    // Green: within 10m of target, Yellow: within 20m, Red: too far or too close
    if (gap_error < 10) {
        std::cout << COLOR_GREEN;   // Good - close to target
    } else if (gap_error < 20) {
        std::cout << COLOR_YELLOW;  // Warning - drifting from target
    } else {
        std::cout << COLOR_RED;     // Danger - too far from target
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
            
            // Keep user's initial gap (don't overwrite)
            // my_position will be initialized when first leader state is received
            
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

// ============== TCP: LEAVE REQUEST ==============
bool request_leave(uint8_t reason = 0) {
    if (!joined.load()) {
        log("TCP", "❌ Not in platoon, cannot leave");
        return false;
    }
    
    const char* reason_str = "normal";
    if (reason == 1) reason_str = "emergency";
    else if (reason == 2) reason_str = "maintenance";
    
    log("TCP", "Sending LEAVE_REQUEST (reason: " + std::string(reason_str) + ")");
    
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        log("TCP", "❌ Failed to create socket");
        return false;
    }
    
    sockaddr_in serv_addr{};
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(TCP_PORT);
    
    if (inet_pton(AF_INET, g_leader_ip.c_str(), &serv_addr.sin_addr) <= 0) {
        log("TCP", "❌ Invalid IP address");
        close(sock);
        return false;
    }
    
    if (connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
        log("TCP", "❌ Connection failed - Is leader running?");
        close(sock);
        return false;
    }
    
    // Send LEAVE request
    LeaveRequest req{};
    req.type = static_cast<uint8_t>(MsgType::LEAVE_REQUEST);
    req.vehicle_index = my_index;
    strncpy(req.vehicle_id, my_vehicle_id, sizeof(req.vehicle_id) - 1);
    req.reason = reason;
    
    send(sock, &req, sizeof(req), 0);
    log("TCP", "📤 Sent LEAVE_REQUEST");
    
    // Receive response
    LeaveResponse resp{};
    ssize_t n = read(sock, &resp, sizeof(resp));
    
    close(sock);
    
    if (n == sizeof(resp) && resp.type == static_cast<uint8_t>(MsgType::LEAVE_RESPONSE)) {
        if (resp.success) {
            log("TCP", "✅ LEAVE ACCEPTED: " + std::string(resp.message));
            joined.store(false);
            running.store(false);  // Stop all threads
            return true;
        } else {
            log("TCP", "❌ LEAVE FAILED: " + std::string(resp.message));
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
            
            // Initialize my_position on first leader state received
            // This places us at the correct distance behind the leader
            if (!position_initialized.load()) {
                my_position = leader_position - distance_to_leader;
                position_initialized.store(true);
            }
            
            // Calculate actual gap
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
    leader_addr.sin_addr.s_addr = inet_addr(g_leader_ip.c_str());
    
    log("UDP-TX", "✅ Sending state to leader (" + g_leader_ip + ":" + std::to_string(UDP_FOLLOWER_PORT) + ")");

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

// ============== TERMINAL INPUT HANDLER ==============
void printHelp() {
    std::cout << "\n" << COLOR_CYAN << "Available commands:" << COLOR_RESET << "\n";
    std::cout << "  leave [reason]  - Leave platoon (reason: 0=normal, 1=emergency, 2=maintenance)\n";
    std::cout << "  status          - Show current status\n";
    std::cout << "  help            - Show this help\n";
    std::cout << "  quit            - Exit program\n";
    std::cout << std::endl;
}

void terminal_input_handler() {
    // Wait for join to complete
    while (!joined.load() && running.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // Wait for dashboard to initialize
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    std::string line;
    while (running.load()) {
        // Show prompt
        {
            std::lock_guard<std::mutex> lock(log_mtx);
            if (dashboard_mode) {
                std::cout << MOVE_TO(input_line, 1) << CLEAR_LINE;
                std::cout << COLOR_GREEN << "Command> " << COLOR_RESET << std::flush;
            }
        }
        
        // Read command (blocking)
        if (!std::getline(std::cin, line)) {
            break;  // EOF or error
        }
        
        // Skip empty lines
        if (line.empty()) continue;
        
        // Parse command
        std::istringstream iss(line);
        std::string cmd;
        iss >> cmd;
        
        if (cmd == "leave" || cmd == "l") {
            int reason = 0;
            iss >> reason;  // Optional reason
            
            logEvent("🚪 Leaving platoon...");
            
            // Disable dashboard for clean output
            dashboard_mode = false;
            std::cout << CLEAR_SCREEN << std::flush;
            
            if (request_leave(static_cast<uint8_t>(reason))) {
                log("MAIN", "👋 Left platoon successfully. Exiting...");
                std::this_thread::sleep_for(std::chrono::seconds(1));
                break;
            } else {
                log("MAIN", "Failed to leave platoon");
                // Re-enable dashboard
                std::cout << CLEAR_SCREEN << std::flush;
                dashboard_mode = true;
            }
            
        } else if (cmd == "status" || cmd == "s") {
            std::lock_guard<std::mutex> lock(log_mtx);
            std::cout << MOVE_TO(input_line + 1, 1) << CLEAR_LINE;
            std::cout << "Index: #" << my_index 
                      << ", Speed: " << my_speed << " km/h"
                      << ", Gap: " << distance_to_leader << " m\n";
                      
        } else if (cmd == "help" || cmd == "h" || cmd == "?") {
            dashboard_mode = false;
            std::cout << CLEAR_SCREEN;
            printHelp();
            std::cout << "Press Enter to continue...";
            std::getline(std::cin, line);
            std::cout << CLEAR_SCREEN << std::flush;
            dashboard_mode = true;
            
        } else if (cmd == "quit" || cmd == "q" || cmd == "exit") {
            logEvent("Exiting without leaving platoon...");
            running.store(false);
            break;
            
        } else {
            logEvent("Unknown command: " + cmd + " (type 'help')");
        }
    }
}

// ============== MAIN ==============

std::string generateFollowerId() {
    // Use process ID to generate unique follower ID
    // This ensures each follower process gets a unique ID
    pid_t pid = getpid();
    int id_num = pid % 100;  // Last 2 digits of PID
    std::ostringstream oss;
    oss << "Follower_" << std::setfill('0') << std::setw(2) << id_num;
    return oss.str();
}

void printUsage(const char* prog) {
    std::cout << "Usage: " << prog << " [options]\n";
    std::cout << "Options:\n";
    std::cout << "  --name <id>       Vehicle ID (default: auto-generated)\n";
    std::cout << "  --leadip <ip>     Leader IP address (default: 127.0.0.1)\n";
    std::cout << "  --speed <km/h>    Initial speed (default: 60)\n";
    std::cout << "  --gap <m>         Initial distance to leader (default: 50)\n";
    std::cout << "  --help            Show this help\n";
    std::cout << "\nExample:\n";
    std::cout << "  " << prog << " --name TRUCK_01 --speed 80 --gap 30\n";
}

int main(int argc, char* argv[]) {
    // Default values
    std::string leader_ip = "127.0.0.1";
    std::string vehicle_id_str;
    double initial_speed = 60.0;      // km/h
    double initial_gap = 50.0;        // m
    
    // Parse named arguments
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        
        if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            return 0;
        } else if (arg == "--name" && i + 1 < argc) {
            vehicle_id_str = argv[++i];
        } else if (arg == "--leadip" && i + 1 < argc) {
            leader_ip = argv[++i];
        } else if (arg == "--speed" && i + 1 < argc) {
            initial_speed = std::atof(argv[++i]);
        } else if (arg == "--gap" && i + 1 < argc) {
            initial_gap = std::atof(argv[++i]);
        } else {
            std::cerr << "Unknown option: " << arg << "\n";
            printUsage(argv[0]);
            return 1;
        }
    }
    
    // Generate vehicle ID if not provided
    if (vehicle_id_str.empty()) {
        vehicle_id_str = generateFollowerId();
    }
    
    // Set initial values to global state
    my_speed = initial_speed;
    distance_to_leader = initial_gap;
    
    const char* vehicle_id = vehicle_id_str.c_str();
    
    // Set global leader IP for UDP thread
    g_leader_ip = leader_ip;
    
    logSeparator("FOLLOWER TRUCK STARTING");
    log("MAIN", "Vehicle ID: " + vehicle_id_str);
    log("MAIN", "Leader IP: " + g_leader_ip);
    log("MAIN", "Initial Speed: " + std::to_string((int)initial_speed) + " km/h");
    log("MAIN", "Initial Gap: " + std::to_string((int)initial_gap) + " m");
    logSeparator("");
    
    // Step 1: Join platoon via TCP
    if (!request_join(g_leader_ip.c_str(), vehicle_id)) {
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
    std::thread input_thread(terminal_input_handler);
    
    // Wait for input handler to finish (user quit or leave)
    input_thread.join();
    
    // Signal other threads to stop
    running.store(false);
    
    // Note: rx_thread may block on recvfrom, will exit when socket closes
    // For clean shutdown, we'd need to use non-blocking sockets or select()
    rx_thread.detach();
    tx_thread.detach();
    
    log("MAIN", "Goodbye! 👋");
    
    return 0;
}