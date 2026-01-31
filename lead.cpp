#include "lead.h"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <atomic>
#include <chrono>
#include <thread>
#include "system_config.h"
#include "message.h"
#include <algorithm>
#include <fcntl.h>
#include <sys/stat.h>
#include <string>

// ANSI colors for terminal output
static const char* COLOR_RED="[31m";
static const char* COLOR_YELLOW="[33m";
static const char* COLOR_GREEN="[32m";
static const char* COLOR_CYAN="[36m";
static const char* COLOR_RESET="[0m";

#define EVENT_FIFO "/tmp/leader_event_fifo"

// For OpenCL Lidar processing
// Safety check kernel:
// Input: Array of distances
// Output: Array of risk flags: 1 means danger, 0 means safe
const char *lidarKernelSource = R"(
__kernel void check_lidar_safety(__global const float *distances, 
                                 __global int *risks, 
                                 float safety_threshold) {
    // Get current index of the ray
    int i = get_global_id(0);
    
    // Actual Lidar rays count
    // If distance < safety threshold AND > 0 (to exclude noise = 0), mark as danger
    if (distances[i] < safety_threshold && distances[i] > 0.1f) {
        risks[i] = 1; // DANGER!
    } else {
        risks[i] = 0; // SAFE!
    }
}
)";

// Constructor
LeadingVehicle::LeadingVehicle(int id, double initialPosition, double initialSpeed)
    : state_(NORMAL), serverRunning_(false), originalSpeed_(initialSpeed), energyAlertSent_(false), stopTimeMs_(0), gasStationStop_(false) {
    info_.id = id;
    info_.position = initialPosition;
    info_.speed = initialSpeed;
    info_.mode = LeaderMode;
    info_.ipAddress = inet_addr("127.0.0.1");
    info_.port = htons(SERVER_PORT);
    platoonState_.leaderId = id;

    // Create event queue
    pthread_mutex_init(&eventMutex_, nullptr);
    pthread_cond_init(&eventCv_, nullptr);

    // Add leader to platoon state
    platoonState_.vehicles.push_back(info_);
    platoonState_.followerCount = 0;
    pthread_mutex_init(&mutex_, nullptr);
    initOpenCL_AEB();
}

// Destructor
LeadingVehicle::~LeadingVehicle() {
    stopServer();
    pthread_mutex_destroy(&mutex_);
    pthread_mutex_destroy(&eventMutex_);
    pthread_cond_destroy(&eventCv_);
    cleanOpenCL();
}

// Start UDP server to communicate with followers
void LeadingVehicle::startServer() {
    // Create UDP socket, bind, etc.
    createServerSocket();
    serverRunning_ = true;
}

// Stop the UDP server
void LeadingVehicle::stopServer() {
    serverRunning_ = false;
    pthread_join(recvThread_, nullptr);
    pthread_join(displayThread_, nullptr);
    close(serverSocket_);
}

// Set leader state
void LeadingVehicle::setState(LeaderState newState) {
    state_ = newState;
    if (newState == STOPPED) {
        stopTimeMs_ = nowMs();
    }
}

// Get leader state
LeaderState LeadingVehicle::getState() const {
    return state_;
}

// Helpers
void LeadingVehicle::createServerSocket() {
    // Implementation of socket creation
    serverSocket_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (serverSocket_ < 0) {
        throw std::runtime_error("Failed to create socket");
    }

    // Allow broadcasting so we can broadcast platoon state
    int broadcastEnable = 1;
    if (setsockopt(serverSocket_, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable)) < 0) {
        // Non-fatal: print warning but continue
        std::cerr << "Warning: failed to enable SO_BROADCAST\n";
    }

    // Set SO_REUSEADDR and SO_REUSEPORT for quick restart
    int opt = 1;
    setsockopt(serverSocket_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    setsockopt(serverSocket_, SOL_SOCKET, SO_REUSEPORT, &opt, sizeof(opt));
    
    // Set SO_LINGER to force immediate close without TIME_WAIT
    struct linger sl;
    sl.l_onoff = 1;
    sl.l_linger = 0;
    setsockopt(serverSocket_, SOL_SOCKET, SO_LINGER, &sl, sizeof(sl));

    // Bind the socket to the server address
    struct sockaddr_in serverAddr{};
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = INADDR_ANY;
    serverAddr.sin_port = htons(SERVER_PORT);
    if (bind(serverSocket_, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
        close(serverSocket_);
        throw std::runtime_error("Failed to bind socket");
    }
}

// Send structured platoon state to all followers
void LeadingVehicle::sendPlatoonState() {
    PlatoonStateMessage psMsg{};
    psMsg.type = MessageType::PLATOON_STATE;
    psMsg.timestamp = nowMs();

    pthread_mutex_lock(&mutex_);
    psMsg.leaderId = platoonState_.leaderId;

    // Sort vehicles by position descending (leader first, then followers in order)
    std::vector<VehicleInfo> sorted = platoonState_.vehicles;
    std::sort(sorted.begin(), sorted.end(),
              [](const VehicleInfo& a, const VehicleInfo& b) { return a.position > b.position; });

    psMsg.vehicleCount = std::min(static_cast<int>(sorted.size()), MAX_PLATOON_VEHICLES);
    for (int i = 0; i < psMsg.vehicleCount; ++i) {
        psMsg.vehicles[i] = sorted[i];
    }

    // Send to each follower using their stored address
    for (const auto& v : platoonState_.vehicles) {
        if (v.id == info_.id) continue; // skip leader
        if (v.ipAddress == 0 || v.port == 0) continue;

        struct sockaddr_in dest{};
        dest.sin_family = AF_INET;
        dest.sin_addr.s_addr = v.ipAddress;
        dest.sin_port = v.port;

        ssize_t sent = sendto(serverSocket_, &psMsg, sizeof(psMsg), 0,
                              (struct sockaddr*)&dest, sizeof(dest));
        if (sent < 0) {
            std::cerr << "sendPlatoonState to " << v.id << " failed: " << strerror(errno) << "\n";
        }
    }
    pthread_mutex_unlock(&mutex_);
}

// Static method: Thread entry for receiving messages
void* LeadingVehicle::recvThreadEntry(void* arg) {
    LeadingVehicle* leader = static_cast<LeadingVehicle*>(arg);
    const size_t BUFFER_SIZE = 4096;
    char* buffer = new char[BUFFER_SIZE];

    while (leader->serverRunning_) {
        // Receive messages from followers
        struct sockaddr_in followerAddr{};
        socklen_t addrLen = sizeof(followerAddr);
        ssize_t recvLen = recvfrom(leader->serverSocket_, buffer, BUFFER_SIZE, 0,
                                   (struct sockaddr*)&followerAddr, &addrLen);

        if (recvLen >= 1) {
            // Process received message
            MessageType msgType = static_cast<MessageType>(buffer[0]);
            switch (msgType)
            {
                case STATUS_UPDATE: {
                    // Update follower info in platoon state
                    if (static_cast<size_t>(recvLen) < sizeof(StatusUpdateMessage)) {
                        std::cerr << "Received short STATUS_UPDATE (" << recvLen << " bytes)\n";
                        break;
                    }

                    // Safe copy from network buffer into local struct
                    StatusUpdateMessage status;
                    std::memcpy(&status, buffer, sizeof(status));

                    // Protect platoonState_ when modifying
                    pthread_mutex_lock(&leader->mutex_);

                    // Update or add vehicle info
                    auto it = std::find_if(
                        leader->platoonState_.vehicles.begin(),
                        leader->platoonState_.vehicles.end(),
                        [&](const VehicleInfo& v){ return v.id == status.info.id; });
                    if (it != leader->platoonState_.vehicles.end()) {
                        it->position = status.info.position;
                        it->speed = status.info.speed;
                    } else {
                        VehicleInfo fi;
                        fi.id = status.info.id;
                        fi.position = status.info.position;
                        fi.speed = status.info.speed;
                        fi.mode = FollowerMode;
                        leader->platoonState_.vehicles.push_back(fi);
                        std::cout << "Added new vehicle " << fi.id << " to platoon\n";
                        it = leader->platoonState_.vehicles.end() - 1; // point to newly added
                    }

                    // Store follower network address and heartbeat timestamp directly in VehicleInfo
                    it->ipAddress = followerAddr.sin_addr.s_addr;
                    it->port = followerAddr.sin_port;
                    it->lastHeartbeatMs = nowMs();

                    pthread_mutex_unlock(&leader->mutex_);
                    break;
                }

                case COUPLE_COMMAND: {
                    // Validate length
                    if (static_cast<size_t>(recvLen) < sizeof(CoupleCommandMessage)) {
                        std::cerr << "Received short COUPLE_COMMAND (" << recvLen << " bytes)\n";
                        break;
                    }

                    // Safe copy from network buffer into local struct
                    CoupleCommandMessage cmd;
                    std::memcpy(&cmd, buffer, sizeof(cmd));

                    std::cout << "Received couple command from vehicle "
                            << cmd.info.id
                            << " to " << (cmd.couple ? "couple" : "decouple")
                            << " (ts=" << cmd.timestamp << ")\n";

                    // Protect platoonState_ when modifying
                    pthread_mutex_lock(&leader->mutex_);
                    if (cmd.couple) {
                        // Add or update existing
                        auto it = std::find_if(
                            leader->platoonState_.vehicles.begin(),
                            leader->platoonState_.vehicles.end(),
                            [&](const VehicleInfo& v){ return v.id == cmd.info.id; });
                        if (it == leader->platoonState_.vehicles.end()) {
                            VehicleInfo fi;
                            fi.id = cmd.info.id;
                            fi.position = cmd.info.position;
                            fi.speed = cmd.info.speed;
                            fi.ipAddress = cmd.info.ipAddress;
                            fi.port = cmd.info.port;
                            fi.mode = FollowerMode;
                            leader->platoonState_.vehicles.push_back(fi);
                            std::cout << "Added vehicle " << fi.id << " to platoon\n";
                        } else {
                            it->position = cmd.info.position;
                            it->speed = cmd.info.speed;
                            it->ipAddress = cmd.info.ipAddress;
                            it->port = cmd.info.port;
                            std::cout << "Updated vehicle " << it->id << " info\n";
                        }
                    } else {
                        leader->platoonState_.vehicles.erase(
                            std::remove_if(
                                leader->platoonState_.vehicles.begin(),
                                leader->platoonState_.vehicles.end(),
                                [&](const VehicleInfo& v){ return v.id == cmd.info.id; }),
                            leader->platoonState_.vehicles.end());
                        std::cout << "Removed vehicle " << cmd.info.id << " from platoon\n";
                    }

                    // Update ip/port/lastHeartbeat for the vehicle (if still present)
                    {
                        auto itr = std::find_if(
                            leader->platoonState_.vehicles.begin(),
                            leader->platoonState_.vehicles.end(),
                            [&](const VehicleInfo& v){ return v.id == cmd.info.id; });
                        if (itr != leader->platoonState_.vehicles.end()) {
                            itr->ipAddress = followerAddr.sin_addr.s_addr;
                            itr->port = followerAddr.sin_port;
                            itr->lastHeartbeatMs = nowMs();
                        }
                    }

                    pthread_mutex_unlock(&leader->mutex_);
                    // Broadcast updated platoon membership
                    leader->sendPlatoonState();
                    break;
                }
                case TRAFFIC_LIGHT_ALERT: {
                    if (static_cast<size_t>(recvLen) < sizeof(TrafficLightMessage)) {
                        std::cerr << "Received short TRAFFIC_LIGHT_ALERT (" << recvLen << " bytes)\n";
                        break;
                    }
                    TrafficLightMessage tlMsg;
                    std::memcpy(&tlMsg, buffer, sizeof(tlMsg));
                    TrafficLightStatus alert = static_cast<TrafficLightStatus>(tlMsg.status);
                    std::cout << "Received traffic light alert from vehicle "
                              << followerAddr.sin_addr.s_addr << ":"
                              << ntohs(followerAddr.sin_port) << " - "
                              << (alert == LIGHT_RED ? "RED" : "GREEN") << std::endl;
                    // For leader, we just log it. Further processing can be added here.
                    if (alert == LIGHT_RED) {
                        leader->setState(STOPPING);
                    } else if (alert == LIGHT_GREEN) {
                        leader->setState(STARTING);
                    }
                    break;
                }
                case ENERGY_DEPLETION_ALERT: {
                    if (static_cast<size_t>(recvLen) < sizeof(EnergyDepletionMessage)) {
                        std::cerr << "Received short ENERGY_DEPLETION_ALERT (" << recvLen << " bytes)\n";
                        break;
                    }
                    EnergyDepletionMessage energyMsg;
                    std::memcpy(&energyMsg, buffer, sizeof(energyMsg));
                    std::cout << "Received energy low alert from vehicle " << energyMsg.vehicleId << std::endl;

                    // Gradually reduce leader's speed by 40% to allow followers to conserve energy
                    leader->setState(LOW_ENERGY);
                    leader->targetSpeed_ = leader->originalSpeed_ * 0.6;
                    std::cout << "Leader will gradually reduce speed to " << leader->targetSpeed_ << " m/s due to low energy alert\n";

                    // Followers will adjust their speed in next status update
                    break;
                }
                case ENERGY_RESTORED: {
                    if (static_cast<size_t>(recvLen) < sizeof(EnergyRestoredMessage)) {
                        std::cerr << "Received short ENERGY_RESTORED (" << recvLen << " bytes)\n";
                        break;
                    }
                    EnergyRestoredMessage erMsg;
                    std::memcpy(&erMsg, buffer, sizeof(erMsg));
                    std::cout << "Received energy restored from vehicle " << erMsg.vehicleId << std::endl;

                    // Stop at gas station
                    leader->setState(STOPPING);
                    std::cout << "Leader stopping at gas station\n";
                    break;
                }
                case GAS_STATION_ALERT: {
                    if (static_cast<size_t>(recvLen) < sizeof(GasStationMessage)) {
                        std::cerr << "Received short GAS_STATION_ALERT (" << recvLen << " bytes)\n";
                        break;
                    }
                    GasStationMessage gsMsg;
                    std::memcpy(&gsMsg, buffer, sizeof(gsMsg));
                    std::cout << "Received gas station alert from vehicle " << gsMsg.vehicleId << std::endl;

                    // Stop at gas station
                    leader->setState(STOPPING);
                    std::cout << "Leader stopping at gas station\n";
                    break;
                }
                default:
                    std::cout << "Unknown message type received: " << static_cast<int>(msgType) << std::endl;
                    break;
            }
        }
    }
    delete[] buffer;
    return nullptr;
}

void* LeadingVehicle::eventSimulationThreadEntry(void* arg) {
    LeadingVehicle* leader = static_cast<LeadingVehicle*>(arg);
    
    // Open FIFO for reading
    int fd = open(EVENT_FIFO, O_RDONLY);
    if (fd == -1) return nullptr;

    int eventChoice;
    while (leader->serverRunning_) {
        // Wait for event input
        if (read(fd, &eventChoice, sizeof(int)) > 0) {
            std::cout << "\n[SYSTEM] Received Event: " << eventChoice << std::endl;
            
            // Handle event
            switch (eventChoice) {
                case 1: {
                    // Obstacle detected toggle
                    leader->force_obstacle_ = !leader->force_obstacle_;
                    std::cout << "[SYSTEM] Obstacle detection forced to " << (leader->force_obstacle_ ? "ON" : "OFF") << std::endl;
                    ObstacleMessage obsMsg{};
                    obsMsg.type = MessageType::OBSTACLE_DETECTED_ALERT;
                    obsMsg.obstacleDetected = leader->force_obstacle_;
                    obsMsg.timestamp = nowMs();

                    if (!leader->force_obstacle_ && leader->getState() == STOPPED) {
                        leader->setState(NORMAL); 
                    }
                    // send event to followers
                    EventMessage eventMsg{};
                    eventMsg.type = MessageType::OBSTACLE_DETECTED_ALERT;
                    eventMsg.eventData = &obsMsg.obstacleDetected;
                    eventMsg.timestamp = nowMs();
                    pthread_mutex_lock(&leader->eventMutex_);
                    leader->eventQueue_.push(eventMsg);
                    pthread_cond_signal(&leader->eventCv_);
                    pthread_mutex_unlock(&leader->eventMutex_);
                    break;
                } // Obstacle 

                case 2: {
                    // Red Light

                    // send event to followers
                    EventMessage eventMsg{};
                    TrafficLightStatus status = LIGHT_RED;
                    eventMsg.type = MessageType::TRAFFIC_LIGHT_ALERT;
                    eventMsg.eventData = &status;
                    eventMsg.timestamp = nowMs();
                    pthread_mutex_lock(&leader->eventMutex_);
                    leader->eventQueue_.push(eventMsg);
                    pthread_cond_signal(&leader->eventCv_);
                    pthread_mutex_unlock(&leader->eventMutex_);

                    // change leader state
                    leader->setState(STOPPING);
                    break;
                }
                
                case 3: { 
                    // Green Light
                    leader->setState(STARTING);

                    // send event to followers
                    EventMessage eventMsg{};
                    TrafficLightStatus status = LIGHT_GREEN;
                    eventMsg.type = MessageType::TRAFFIC_LIGHT_ALERT;
                    eventMsg.eventData = &status;
                    eventMsg.timestamp = nowMs();
                    pthread_mutex_lock(&leader->eventMutex_);
                    leader->eventQueue_.push(eventMsg);
                    pthread_cond_signal(&leader->eventCv_);
                    pthread_mutex_unlock(&leader->eventMutex_);

                    break;
                }
                case 4: {
                    // Return to normal speed
                    leader->setState(NORMAL);
                    leader->targetSpeed_ = leader->originalSpeed_;
                    break;
                }
                case 5: {
                    // Run out of energy
                    leader->targetSpeed_ = leader->info_.speed * 0.6;
                    leader->setState(LOW_ENERGY);
                    leader->energyAlertSent_ = false; // Allow sending alert in status thread
                    break;
                }
                case 6: {
                    // Restore energy
                    leader->gasStationStop_ = true; // Mark as gas station stop
                    leader->setState(STOPPING); // Stop at gas station
                    // Send gas station alert to followers
                    EventMessage eventMsg{};
                    eventMsg.type = MessageType::GAS_STATION_ALERT;
                    eventMsg.timestamp = nowMs();
                    pthread_mutex_lock(&leader->eventMutex_);
                    leader->eventQueue_.push(eventMsg);
                    pthread_cond_signal(&leader->eventCv_);
                    pthread_mutex_unlock(&leader->eventMutex_);
                    break;
                }
                case 7: {
                    // Leave Platoon
                    std::cout << "\n========================================\n";
                    std::cout << "LEADER DEPARTURE INITIATED\n";
                    std::cout << "Leader ID " << leader->info_.id << " is leaving the platoon\n";
                    std::cout << "Next vehicle will become new leader\n";
                    std::cout << "========================================\n\n";

                    LeavePlatoonMessage lpMsg{};
                    lpMsg.type = MessageType::LEAVE_PLATOON;
                    lpMsg.vehicleId = leader->info_.id;
                    lpMsg.timestamp = nowMs();

                    // send event to followers
                    EventMessage eventMsg{};
                    eventMsg.type = MessageType::LEAVE_PLATOON;
                    eventMsg.eventData = &lpMsg.vehicleId;
                    eventMsg.timestamp = nowMs();

                    pthread_mutex_lock(&leader->eventMutex_);
                    leader->eventQueue_.push(eventMsg);
                    pthread_cond_signal(&leader->eventCv_);
                    pthread_mutex_unlock(&leader->eventMutex_);
                    break;
                }
                case 0: { // Exit
                    leader->serverRunning_ = false;
                    break;
                }
                default: {
                    break;
                }
            }
        }
    }
    close(fd);
    return nullptr;
}

void* LeadingVehicle::displayThreadEntry(void* arg) {
    const int REFRESH_MS = 300; // refresh interval
    LeadingVehicle* leader = static_cast<LeadingVehicle*>(arg);
    while (leader->serverRunning_) {
        // Build display under mutex to get consistent snapshot
        pthread_mutex_lock(&leader->mutex_);
        LeaderState state = leader->getState();
        std::string stateStr;
        switch (state) {
            case NORMAL: stateStr = "NORMAL"; break;
            case ERROR: stateStr = "ERROR"; break;
            case STOPPING: stateStr = "STOPPING"; break;
            case STOPPED: stateStr = "STOPPED"; break;
            case STARTING: stateStr = "STARTING"; break;
            case LOW_ENERGY: stateStr = "LOW_ENERGY"; break;
            case DEGRADED: stateStr = "DEGRADED"; break;
            default: stateStr = "UNKNOWN"; break;
        }
        // Clear and print header + platoon list
        std::cout << "\033[2J\033[H"; // clear screen + home
        std::cout << "\n\033[1;36m================== LEADER ==================\033[0m\n";
        std::cout << "Position: " << leader->info_.position
                  << "  Speed: " << leader->info_.speed
                  << "  State: " << stateStr << "\n";
        std::cout << "Platoon vehicles:\n";
        for (const auto &v : leader->platoonState_.vehicles) {
            std::cout << "Vehicle ID: " << v.id
                      << ", Position: " << v.position
                      << ", Speed: " << v.speed << " m/s"
                      << ", Mode: " << (v.mode == LeaderMode ? "Leader" : "Follower") << "\n";
        }
        pthread_mutex_unlock(&leader->mutex_);

        std::this_thread::sleep_for(std::chrono::milliseconds(REFRESH_MS));
    }
    return nullptr;
}

void* LeadingVehicle::runThreadEntry(void* arg) {
    LeadingVehicle* leader = static_cast<LeadingVehicle*>(arg);

    const int TIMESTEP_MS = 100;
    const double dt = TIMESTEP_MS / 1000.0;
    const double accel = 8;    // m/s^2 when starting
    const double decel = 12;   // m/s^2 when stopping

    while (leader->serverRunning_) {
        // For OpenCL Lidar processing******************************************************************************
        // Update sensor data
        leader->updateSimulatedLidar();
        // Scan environment for obstacles
        bool obstacleDetected = leader->scanEnvironmentWithGPU();
        if (obstacleDetected) {
            if (leader->getState() != STOPPED && leader->getState() != STOPPING) {
                std::cout << "\033[1;31m[AEB ACTIVATED] Obstacle detected by OpenCL! Emergency Braking!\033[0m\n";
                leader->setState(STOPPING);
            }
        }
        // End OpenCL Lidar processing******************************************************************************

        // Update kinematics according to the leader state machine
        LeaderState st = leader->getState();
        double targetSpeed = (st == NORMAL) ? leader->originalSpeed_ : leader->info_.speed;
        switch (st) {
            case NORMAL:
                if (leader->info_.speed < targetSpeed) {
                    leader->info_.speed = std::min(targetSpeed, leader->info_.speed + accel * dt);
                } else if (leader->info_.speed > targetSpeed) {
                    leader->info_.speed = std::max(targetSpeed, leader->info_.speed - decel * dt);
                }
                break;
            case STOPPING:
                leader->info_.speed = std::max(0.0, leader->info_.speed - decel * dt * 1.2); // slightly more aggressive decel
                if (leader->info_.speed <= 0.001) {
                    leader->info_.speed = 0.0;
                    leader->setState(STOPPED);
                }
                break;
            case STOPPED:
                leader->info_.speed = 0.0;
                if (leader->gasStationStop_ && leader->nowMs() - leader->stopTimeMs_ > 3000) { // 3 seconds for gas station
                    std::cout << "Energy restored.\n";
                    leader->gasStationStop_ = false; // Reset flag
                    leader->setState(STARTING);
                }
                break;
            case STARTING:
                leader->info_.speed = std::min(targetSpeed, leader->info_.speed + accel * dt);
                if (leader->info_.speed >= targetSpeed - 1e-6) {
                    leader->setState(NORMAL);
                }
                break;
            case LOW_ENERGY:
                targetSpeed = leader->targetSpeed_;
                if (leader->info_.speed > targetSpeed) {
                    leader->info_.speed = std::max(targetSpeed, leader->info_.speed - decel * dt);
                } else if (leader->info_.speed < targetSpeed) {
                    leader->info_.speed = std::min(targetSpeed, leader->info_.speed + accel * dt);
                }
                break;
            case DEGRADED:
                // Lower speed to 48 m/s for safety
                targetSpeed = 48.0;
                if (leader->info_.speed > targetSpeed) {
                    leader->info_.speed = std::max(targetSpeed, leader->info_.speed - decel * dt);
                } else if (leader->info_.speed < targetSpeed) {
                    leader->info_.speed = std::min(targetSpeed, leader->info_.speed + accel * dt);
                }
                break;
            default:
                break;
        }

        // Integrate position
        leader->info_.position += leader->info_.speed * dt;

        // Update leader entry in the shared platoon snapshot
        pthread_mutex_lock(&leader->mutex_);
        for (auto &v : leader->platoonState_.vehicles) {
            if (v.id == leader->info_.id) {
                v.position = leader->info_.position;
                v.speed = leader->info_.speed;
                break;
            }
        }
        pthread_mutex_unlock(&leader->mutex_);

        std::this_thread::sleep_for(std::chrono::milliseconds(TIMESTEP_MS));
    }

    return nullptr;
}

void* LeadingVehicle::heartbeatThreadEntry(void* arg) {
    LeadingVehicle* leader = static_cast<LeadingVehicle*>(arg);
    const int HEARTBEAT_INTERVAL_MS = 1000; // Check every second
    const int TIMEOUT_MS = 5000; // Timeout for follower heartbeat - remove after 5 seconds
    const int DEGRADED_MS = 1000; // Enter degraded mode after 1 second of heartbeat timeout 
    
    while (leader->serverRunning_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(HEARTBEAT_INTERVAL_MS));
        std::int64_t currentTime = nowMs();

        std::vector<int> toRemove;
        bool allFollowersOk = true;
        pthread_mutex_lock(&leader->mutex_);
        for (auto it = leader->platoonState_.vehicles.begin(); it != leader->platoonState_.vehicles.end(); ) {
            if (it->id == leader->info_.id) { ++it; continue; } // skip leader
            const std::int64_t age = (it->lastHeartbeatMs == 0) ? (INT64_MAX) : (currentTime - it->lastHeartbeatMs);
            if (age > DEGRADED_MS) {
                allFollowersOk = false;
            }
            if ((age > DEGRADED_MS) && (age < TIMEOUT_MS) && leader->getState() != DEGRADED) {
                std::cout << COLOR_YELLOW << "[HEARTBEAT] Follower " << it->id << " heartbeat delayed. Entering DEGRADED mode.\n" << COLOR_RESET;
                leader->setState(DEGRADED);
            } else if (it->lastHeartbeatMs == 0 || age > TIMEOUT_MS) {
                std::cout << COLOR_RED << "[HEARTBEAT] Follower " << it->id << " timed out. Removing from platoon.\n" << COLOR_RESET;

                toRemove.push_back(it->id);
                it = leader->platoonState_.vehicles.erase(it);

                // Send remove notification to the follower
                RemoveVehicleMessage removeMsg{};
                removeMsg.type = MessageType::REMOVE_VEHICLE;
                removeMsg.vehicleId = it->id;
                removeMsg.timestamp = nowMs();
                struct sockaddr_in followerAddr{};
                followerAddr.sin_family = AF_INET;
                followerAddr.sin_addr.s_addr = it->ipAddress;
                followerAddr.sin_port = it->port;
                ssize_t sentBytes = sendto(leader->serverSocket_, &removeMsg, sizeof(removeMsg), 0,
                                           (const struct sockaddr*)&followerAddr, sizeof(followerAddr));
                if (sentBytes < 0) {
                    std::cerr << "Failed to send remove notification to follower " << it->id << std::endl;
                }
            } else {
                ++it;
            }
        }
        // If all followers are ok and leader is in degraded, exit degraded mode
        if (allFollowersOk && leader->getState() == DEGRADED) {
            std::cout << COLOR_GREEN << "[HEARTBEAT] All followers recovered. Exiting DEGRADED mode.\n" << COLOR_RESET;
            leader->setState(NORMAL);
        }
        // (legacy maps removed; VehicleInfo entries already erased above)
        pthread_mutex_unlock(&leader->mutex_);

        if (!toRemove.empty()) leader->sendPlatoonState();
    }
    return nullptr;
}

void* LeadingVehicle::sendStatusThreadEntry(void* arg) {
    LeadingVehicle* leader = static_cast<LeadingVehicle*>(arg);

    const int STATUS_INTERVAL_MS = 100; // Send status every 100 ms
    while (leader->serverRunning_) {
        // Build status message
        StatusUpdateMessage status;
        status.type = MessageType::STATUS_UPDATE;
        status.info.id = leader->info_.id;
        status.info.position = leader->info_.position;
        status.info.speed = leader->info_.speed;
        status.info.mode = leader->info_.mode;
        status.timestamp = nowMs();

        // Send to all known follower addresses
        pthread_mutex_lock(&leader->mutex_);
        for (const auto &v : leader->platoonState_.vehicles) {
            if (v.id == leader->info_.id) continue; // skip leader
            if (v.ipAddress == 0 || v.port == 0) continue; // no address known
            struct sockaddr_in dest{};
            dest.sin_family = AF_INET;
            dest.sin_addr.s_addr = v.ipAddress;
            dest.sin_port = v.port;
            ssize_t sentBytes = sendto(leader->serverSocket_, &status, sizeof(status), 0,
                                       (const struct sockaddr*)&dest, sizeof(dest));
            if (sentBytes < 0) {
                std::cerr << "Failed to send status to follower " << v.id << ": " << strerror(errno) << std::endl;
            }
        }
        pthread_mutex_unlock(&leader->mutex_);
        std::this_thread::sleep_for(std::chrono::milliseconds(STATUS_INTERVAL_MS));
    }
    return nullptr;
}

// Send event messages to followers
void* LeadingVehicle::eventSenderThreadEntry(void* arg) {
    LeadingVehicle* leader = static_cast<LeadingVehicle*>(arg);

    while(leader->serverRunning_) {
        // Waiting for event queue
        pthread_mutex_lock(&leader->eventMutex_);
        while (leader->eventQueue_.empty() && leader->serverRunning_) {
             pthread_cond_wait(&leader->eventCv_, &leader->eventMutex_);
        }
        if (!leader->serverRunning_) {
            pthread_mutex_unlock(&leader->eventMutex_);
            break;
        }
        EventMessage eventMsg = leader->eventQueue_.front();
        leader->eventQueue_.pop();
        pthread_mutex_unlock(&leader->eventMutex_);

        // Send event message only to the vehicle directly behind leader (index 1 in sorted list)
        pthread_mutex_lock(&leader->mutex_);
        
        // Handle self-exit even if no followers
        if (eventMsg.type == MessageType::LEAVE_PLATOON) {
             if (leader->platoonState_.vehicles.size() > 1) {
                 const VehicleInfo& rear = leader->platoonState_.vehicles[1];
                 if (rear.ipAddress != 0 && rear.port != 0) {
                      struct sockaddr_in dest{};
                        dest.sin_family = AF_INET;
                        dest.sin_addr.s_addr = rear.ipAddress;
                        dest.sin_port = rear.port;
                         LeavePlatoonMessage leaveMsg{};
                        leaveMsg.type = MessageType::LEAVE_PLATOON;
                        leaveMsg.vehicleId = leader->info_.id;
                        leaveMsg.timestamp = eventMsg.timestamp;
                        sendto(leader->serverSocket_, &leaveMsg, sizeof(leaveMsg), 0,
                                           (const struct sockaddr*)&dest, sizeof(dest));
                        std::cout << "[SYSTEM] Notified vehicle " << rear.id << " to become new leader.\n";
                 }
             }
             std::cout << "[SYSTEM] Stopping leader server...\n";
             leader->serverRunning_ = false;
             pthread_cond_broadcast(&leader->eventCv_);
             ::shutdown(leader->serverSocket_, SHUT_RDWR);
             ::close(leader->serverSocket_);
             exit(0);
        }

        if (leader->platoonState_.vehicles.size() > 1) {
            const VehicleInfo& rear = leader->platoonState_.vehicles[1]; // index 0 is leader, index 1 is rear
            if (rear.ipAddress != 0 && rear.port != 0) {
                struct sockaddr_in dest{};
                dest.sin_family = AF_INET;
                dest.sin_addr.s_addr = rear.ipAddress;
                dest.sin_port = rear.port;

                ssize_t sentBytes = 0;
                switch (eventMsg.type) {
                    case TRAFFIC_LIGHT_ALERT: {
                        TrafficLightMessage tlMsg{};
                        tlMsg.type = MessageType::TRAFFIC_LIGHT_ALERT;
                        tlMsg.timestamp = eventMsg.timestamp;
                        tlMsg.status = eventMsg.eventData ? *(static_cast<TrafficLightStatus*>(eventMsg.eventData)) : LIGHT_GREEN;
                        sentBytes = sendto(leader->serverSocket_, &tlMsg, sizeof(tlMsg), 0,
                                           (const struct sockaddr*)&dest, sizeof(dest));
                        break;
                    }
                    case GAS_STATION_ALERT: {
                        GasStationMessage gsMsg{};
                        gsMsg.type = MessageType::GAS_STATION_ALERT;
                        gsMsg.vehicleId = leader->info_.id;
                        gsMsg.timestamp = eventMsg.timestamp;
                        sentBytes = sendto(leader->serverSocket_, &gsMsg, sizeof(gsMsg), 0,
                                           (const struct sockaddr*)&dest, sizeof(dest));
                        break;
                    }
                    case OBSTACLE_DETECTED_ALERT: {
                        ObstacleMessage obsMsg{};
                        obsMsg.type = MessageType::OBSTACLE_DETECTED_ALERT;
                        obsMsg.timestamp = eventMsg.timestamp;
                        obsMsg.obstacleDetected = eventMsg.eventData ? *(static_cast<bool*>(eventMsg.eventData)) : false;
                        sentBytes = sendto(leader->serverSocket_, &obsMsg, sizeof(obsMsg), 0,
                                           (const struct sockaddr*)&dest, sizeof(dest));
                        break;
                    }
                    case LEAVE_PLATOON: {
                        // Send leave platoon message to rear vehicle (next leader)
                        LeavePlatoonMessage leaveMsg{};
                        leaveMsg.type = MessageType::LEAVE_PLATOON;
                        leaveMsg.vehicleId = leader->info_.id;
                        leaveMsg.timestamp = eventMsg.timestamp;
                        sentBytes = sendto(leader->serverSocket_, &leaveMsg, sizeof(leaveMsg), 0,
                                           (const struct sockaddr*)&dest, sizeof(dest));
                        
                        if (sentBytes > 0) {
                            std::cout << "[SYSTEM] Notified vehicle " << rear.id << " to become new leader.\n";
                        }

                        // Give time for message to be delivered
                        std::this_thread::sleep_for(std::chrono::milliseconds(500));

                        // Update platoon state to remove self
                        leader->platoonState_.vehicles.erase(
                            std::remove_if(
                                leader->platoonState_.vehicles.begin(),
                                leader->platoonState_.vehicles.end(),
                                [&](const VehicleInfo& v){ return v.id == leader->info_.id; }),
                                leader->platoonState_.vehicles.end());
                        
                        // Send final platoon state update
                        leader->sendPlatoonState();

                        std::cout << "[SYSTEM] Closing socket and shutting down...\n";

                        // Close socket immediately to release port
                        ::shutdown(leader->serverSocket_, SHUT_RDWR);
                        ::close(leader->serverSocket_);
                        leader->serverSocket_ = -1;
                        
                        std::cout << "[SYSTEM] Leader departure complete.\n";
                        
                        // Force exit to ensure socket is released and process stops
                        exit(0);
                        break;
                    }
                    default:
                        break;
                }

                if (sentBytes < 0) {
                    std::cerr << "Failed to send event to rear vehicle " << rear.id << ": " << strerror(errno) << std::endl;
                } else {
                    switch (eventMsg.type)
                    {
                    case LEAVE_PLATOON: {
                        // After sending leave platoon, stop the leader server
                        std::cout << "[SYSTEM] Stopping leader server...\n";
                        leader->serverRunning_ = false;
                        pthread_cond_broadcast(&leader->eventCv_);
                        exit(0); // Ensure exit here too if fall through
                        break;
                    }

                    default:
                        break;
                    }                    
                }
            }
        }
        pthread_mutex_unlock(&leader->mutex_);
    }
    return nullptr;
}

// Start internal threads
void LeadingVehicle::startThreads() {
    // Start receive message thread
    pthread_create(&recvThread_, nullptr, recvThreadEntry, this);

    // Start user input event simulation thread
    pthread_create(&eventThread_, nullptr, eventSimulationThreadEntry, this);

    // Display thread
    pthread_create(&displayThread_, nullptr, displayThreadEntry, this);

    // Run thread
    pthread_create(&runThread_, nullptr, runThreadEntry, this);

    // Heartbeat monitoring thread
    pthread_create(&heartbeatThread_, nullptr, heartbeatThreadEntry, this);

    // Status sending thread
    pthread_create(&sendStatusThread_, nullptr, sendStatusThreadEntry, this);

    // Event sending thread
    pthread_create(&eventSenderThread_, nullptr, eventSenderThreadEntry, this);
}

// Timing
std::int64_t LeadingVehicle::nowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch()).count();
}

// OpenCL helper functions would go here (omitted for brevity)
void LeadingVehicle::initOpenCL_AEB() {
    cl_int ret;

    // 1. Basic OpenCL setup
    clGetPlatformIDs(1, &platform_id, NULL);
    clGetDeviceIDs(platform_id, CL_DEVICE_TYPE_DEFAULT, 1, &device_id, NULL);
    context = clCreateContext(NULL, 1, &device_id, NULL, NULL, &ret);
    command_queue = clCreateCommandQueue(context, device_id, 0, &ret);

    // 2. Memory allocation
    lidar_data_.resize(LIDAR_RAYS, 100.0f); // Default clear road (100m)
    risk_map_.resize(LIDAR_RAYS, 0);

    input_distance_buffer = clCreateBuffer(context, CL_MEM_READ_ONLY, 
                                           LIDAR_RAYS * sizeof(float), NULL, &ret);
    output_risk_buffer = clCreateBuffer(context, CL_MEM_WRITE_ONLY, 
                                        LIDAR_RAYS * sizeof(int), NULL, &ret);

    // 3. Build Kernel
    program = clCreateProgramWithSource(context, 1, (const char **)&lidarKernelSource, NULL, &ret);
    ret = clBuildProgram(program, 1, &device_id, NULL, NULL, NULL);
    
    if (ret != CL_SUCCESS) {
        // Debug build errors (important when kernel code is wrong)
        size_t len;
        char buffer[2048];
        clGetProgramBuildInfo(program, device_id, CL_PROGRAM_BUILD_LOG, sizeof(buffer), buffer, &len);
        std::cerr << "[OPENCL ERROR] Build Log: " << buffer << std::endl;
        exit(1);
    }

    lidar_kernel = clCreateKernel(program, "check_lidar_safety", &ret);
    std::cout << "[SYSTEM] AEB System (OpenCL) Initialized. Monitoring " << LIDAR_RAYS << " Lidar rays.\n";
}

void LeadingVehicle::cleanOpenCL() {
    clReleaseMemObject(input_distance_buffer);
    clReleaseMemObject(output_risk_buffer);
    clReleaseKernel(lidar_kernel);
    clReleaseProgram(program);
    clReleaseCommandQueue(command_queue);
    clReleaseContext(context);
}

void LeadingVehicle::updateSimulatedLidar() {
    // Reset all to 100m (clear road)
    std::fill(lidar_data_.begin(), lidar_data_.end(), 100.0f);

    // [SIMULATION] Randomly simulate obstacle appearance
    // 1% chance each call to have an obstacle appear in front (middle of array)
    if (force_obstacle_) {  // Forced obstacle for testing
        // Assume obstacle is 15m away (below safe distance 20m)
        // Obstacle blocks rays from 350 to 370
        for (int i = 350; i < 370; ++i) {
            lidar_data_[i] = 15.0f; 
        }
        // std::cout << "[SIM] Obstacle detected by sensors!\n"; 
    }
}

bool LeadingVehicle::scanEnvironmentWithGPU() {
    cl_int ret;

    // 1. Copy latest Lidar data from RAM to VRAM
    ret = clEnqueueWriteBuffer(command_queue, input_distance_buffer, CL_TRUE, 0,
                               LIDAR_RAYS * sizeof(float), lidar_data_.data(), 
                               0, NULL, NULL);

    // 2. Set Kernel arguments
    float threshold = (float)SAFETY_DIST;
    clSetKernelArg(lidar_kernel, 0, sizeof(cl_mem), (void *)&input_distance_buffer);
    clSetKernelArg(lidar_kernel, 1, sizeof(cl_mem), (void *)&output_risk_buffer);
    clSetKernelArg(lidar_kernel, 2, sizeof(float), (void *)&threshold);

    // 3. Launch GPU with 720 parallel threads
    size_t global_size = LIDAR_RAYS;
    ret = clEnqueueNDRangeKernel(command_queue, lidar_kernel, 1, NULL, 
                                 &global_size, NULL, 0, NULL, NULL);

    // 4. Read analysis results back to RAM
    ret = clEnqueueReadBuffer(command_queue, output_risk_buffer, CL_TRUE, 0,
                              LIDAR_RAYS * sizeof(int), risk_map_.data(), 
                              0, NULL, NULL);

    // 5. CPU aggregate results (Simple reduction)
    // If any ray reports danger (value 1), return true
    for (int risk : risk_map_) {
        if (risk == 1) return true; // Obstacle detected!
    }
    return false;
}

// Program start entry
int main(int argc, char** argv) {
    // Terminal for event input
    if (argc >= 2 && std::string(argv[1]) == "input_mode") {
        int fd = open(EVENT_FIFO, O_WRONLY);
        if (fd == -1) exit(1);

        int choice;
        while (true) {
            // Display menu
            std::cout << "\033[2J\033[H"; // clear screen + home
            std::cout << "\n[LEADER EVENT INPUT]\n";
            std::cout << "1: Obstacle detected (ON/OFF)\n";
            std::cout << "2: Traffic light RED\n";
            std::cout << "3: Traffic light GREEN\n";
            std::cout << "4: Cut-in vehicle alert\n";
            std::cout << "5: Run out of energy\n";
            std::cout << "6: Restore energy\n";
            std::cout << "7: Leave platoon\n";
            std::cout << "0: Exit\n";
            std::cout << "Enter choice: ";
            std::cin >> choice;

            write(fd, &choice, sizeof(int));
            if (choice == 0) break;
        }
        close(fd);
        return 0;
    }

    // Terminal Leader mode
    int id = LEADER_INITIAL_ID;
    double baseSpeed = LEADER_INITIAL_SPEED;
    double initialPos = LEADER_INITIAL_POSITION;

    try {
        // Create FIFO
        unlink(EVENT_FIFO);
        mkfifo(EVENT_FIFO, 0666);

        // Open new terminal for input. Try several common terminal emulators;
        // if none can be launched (e.g., WSL/no GUI), fall back to running
        // `input_mode` directly in the child process so input still works.
        pid_t child = fork();
        if (child == 0) {
            const char* terms[] = {"gnome-terminal", "x-terminal-emulator", "xfce4-terminal", "lxterminal", "mate-terminal", "konsole", "xterm"};
            for (const char* term : terms) {
                execlp(term, term, "--", argv[0], "input_mode", NULL);
                execlp(term, term, "-e", argv[0], "input_mode", NULL);
            }

            // Couldn't launch a separate GUI terminal — run input_mode directly here.
            execlp(argv[0], argv[0], "input_mode", NULL);

            // If everything failed, exit child quietly.
            exit(0);
        }

        LeadingVehicle leader(id, initialPos, baseSpeed);
        leader.startServer();
        leader.startThreads(); // Ensure event handling thread is started here
        while (true) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    } catch (const std::exception& ex) {
        std::cerr << "Leader error: " << ex.what() << std::endl;
        return 1;
    }
    return 0;
}
