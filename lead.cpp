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

#define EVENT_FIFO "/tmp/leader_event_fifo"

// Constructor
LeadingVehicle::LeadingVehicle(int id, double initialPosition, double initialSpeed)
    : state_(NORMAL), serverRunning_(false) {
    info_.id = id;
    info_.position = initialPosition;
    info_.speed = initialSpeed;
    info_.mode = LeaderMode;
    platoonState_.leaderId = id;

    // Create event queue
    pthread_mutex_init(&eventMutex_, nullptr);
    pthread_cond_init(&eventCv_, nullptr);

    // Add leader to platoon state
    platoonState_.vehicles.push_back(info_);
    platoonState_.followerCount = 0;
    pthread_mutex_init(&mutex_, nullptr);
}

// Destructor
LeadingVehicle::~LeadingVehicle() {
    stopServer();
    pthread_mutex_destroy(&mutex_);
    pthread_mutex_destroy(&eventMutex_);
    pthread_cond_destroy(&eventCv_);
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

// Send a textual platoon state via UDP broadcast (simple implementation)
void LeadingVehicle::sendPlatoonState() {
    // Build a simple textual representation under mutex
    std::string payload;
    pthread_mutex_lock(&mutex_);
    payload += "PLATOON_STATE ";
    payload += "leader=" + std::to_string(platoonState_.leaderId) + " ";
    payload += "count=" + std::to_string((int)platoonState_.vehicles.size()) + " ";
    payload += "members:";
    for (const auto &v : platoonState_.vehicles) {
        payload += std::to_string(v.id);
        payload += (v.mode == LeaderMode) ? "(L)" : "(F)";
        payload += ",";
    }
    pthread_mutex_unlock(&mutex_);

    // Send via UDP broadcast on SERVER_PORT
    struct sockaddr_in destAddr{};
    destAddr.sin_family = AF_INET;
    destAddr.sin_port = htons(SERVER_PORT);
    destAddr.sin_addr.s_addr = htonl(INADDR_BROADCAST);

    ssize_t sent = sendto(serverSocket_, payload.c_str(), payload.size(), 0,
                          (struct sockaddr*)&destAddr, sizeof(destAddr));
    if (sent < 0) {
        std::cerr << "sendPlatoonState: sendto failed\n";
    }
}

// Static method: Thread entry for receiving messages
void* LeadingVehicle::recvThreadEntry(void* arg) {
    LeadingVehicle* leader = static_cast<LeadingVehicle*>(arg);
    char buffer[1024];

    while (leader->serverRunning_) {
        // Receive messages from followers
        struct sockaddr_in followerAddr{};
        socklen_t addrLen = sizeof(followerAddr);
        ssize_t recvLen = recvfrom(leader->serverSocket_, buffer, sizeof(buffer), 0,
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
                    }

                    // Store follower network address and heartbeat timestamp
                    leader->followerAddrs_[status.info.id] = followerAddr;
                    leader->lastHeartbeatTimes_[status.info.id] = nowMs();

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
                            fi.mode = FollowerMode;
                            leader->platoonState_.vehicles.push_back(fi);
                            std::cout << "Added vehicle " << fi.id << " to platoon\n";
                        } else {
                            it->position = cmd.info.position;
                            it->speed = cmd.info.speed;
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

                    // store follower address + heartbeat timestamp for joins/updates
                    leader->followerAddrs_[cmd.info.id] = followerAddr;
                    leader->lastHeartbeatTimes_[cmd.info.id] = nowMs();

                    pthread_mutex_unlock(&leader->mutex_);
                    // Broadcast updated platoon membership
                    leader->sendPlatoonState();
                    break;
                }
                default:
                    std::cout << "Unknown message type received: " << static_cast<int>(msgType) << std::endl;
                    break;
            }
        }
    }
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
        // Clear and print header + platoon list
        std::cout << "\033[2J\033[H"; // clear screen + home
        std::cout << "\n\033[1;36m================== LEADER ==================\033[0m\n";
        std::cout << "ID: " << leader->info_.id
                  << "  Position: " << leader->info_.position
                  << "  Speed: " << leader->info_.speed << "\n";
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

    // snapshot desired cruising speed (can be changed elsewhere)
    double targetSpeed = leader->info_.speed;

    while (leader->serverRunning_) {
        // Update kinematics according to the leader state machine
        LeaderState st = leader->getState();
        switch (st) {
            case NORMAL:
                if (leader->info_.speed < targetSpeed) {
                    leader->info_.speed = std::min(targetSpeed, leader->info_.speed + accel * dt);
                } else if (leader->info_.speed > targetSpeed) {
                    leader->info_.speed = std::max(targetSpeed, leader->info_.speed - decel * dt);
                }
                break;
            case STOPPING:
                leader->info_.speed = std::max(0.0, leader->info_.speed - decel * dt);
                if (leader->info_.speed <= 0.001) {
                    leader->info_.speed = 0.0;
                    leader->setState(STOPPED);
                }
                break;
            case STOPPED:
                leader->info_.speed = 0.0;
                break;
            case STARTING:
                leader->info_.speed = std::min(targetSpeed, leader->info_.speed + accel * dt);
                if (leader->info_.speed >= targetSpeed - 1e-6) {
                    leader->setState(NORMAL);
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
    const int TIMEOUT_MS = 3000; // Timeout for follower heartbeat
    while (leader->serverRunning_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(HEARTBEAT_INTERVAL_MS));
        std::int64_t currentTime = nowMs();

        std::vector<int> toRemove;
        pthread_mutex_lock(&leader->mutex_);
        for (auto it = leader->platoonState_.vehicles.begin(); it != leader->platoonState_.vehicles.end(); ) {
            if (it->id == leader->info_.id) { ++it; continue; } // skip leader

            auto lastIt = leader->lastHeartbeatTimes_.find(it->id);
            if (lastIt == leader->lastHeartbeatTimes_.end() ||
                (currentTime - lastIt->second) > TIMEOUT_MS) {
                std::cout << "[HEARTBEAT] Follower " << it->id << " timed out. Removing from platoon.\n";
                toRemove.push_back(it->id);
                it = leader->platoonState_.vehicles.erase(it);
            } else {
                ++it;
            }
        }
        // Cleanup address + timestamp entries
        for (int id : toRemove) {
            leader->followerAddrs_.erase(id);
            leader->lastHeartbeatTimes_.erase(id);
        }
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
        for (const auto &p : leader->followerAddrs_) {
            int fid = p.first;
            if (fid == leader->info_.id) continue;
            const struct sockaddr_in dest = p.second;
            ssize_t sentBytes = sendto(leader->serverSocket_, &status, sizeof(status), 0,
                                       (const struct sockaddr*)&dest, sizeof(dest));
            if (sentBytes < 0) {
                std::cerr << "Failed to send status to follower " << fid << ": " << strerror(errno) << std::endl;
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

        // Send event message to all followers
        pthread_mutex_lock(&leader->mutex_);
        for (const auto &p : leader->followerAddrs_) {
            int fid = p.first;
            if (fid == leader->info_.id) continue;
            const struct sockaddr_in dest = p.second;

            ssize_t sentBytes = 0;
            switch (eventMsg.type)
            {
                case TRAFFIC_LIGHT_ALERT: {
                    TrafficLightMessage tlMsg{};
                    tlMsg.type = MessageType::TRAFFIC_LIGHT_ALERT;
                    tlMsg.timestamp = eventMsg.timestamp;
                    tlMsg.status = *(static_cast<TrafficLightStatus*>(eventMsg.eventData));
                    std::cout << tlMsg.status << std::endl;
                    // std::memcpy(&eventMsg, &tlMsg, sizeof(tlMsg));
                    sentBytes = sendto(leader->serverSocket_, &tlMsg, sizeof(tlMsg), 0,
                                            (const struct sockaddr*)&dest, sizeof(dest));
                    break;
                }
                default:
                    break;
            }

            if (sentBytes < 0) {
                std::cerr << "Failed to send event to follower " << fid << ": " << strerror(errno) << std::endl;
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
            std::cout << "\n[EVENT INPUT TERMINAL]\n";
            std::cout << "1: Obstacle detected\n";
            std::cout << "2: Traffic light RED\n";
            std::cout << "3: Traffic light GREEN\n";
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

        // Open new terminal for input
        if (fork() == 0) {
            // Run gnome-terminal with this file and input_mode argument
            execlp("gnome-terminal", "gnome-terminal", "--", argv[0], "input_mode", NULL);
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
