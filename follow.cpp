#include "follow.h"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <atomic>
#include <chrono>
#include "system_config.h"
#include "message.h"
#include <thread>
#include <cmath>
#include <algorithm>
#include <fcntl.h>
#include <sys/stat.h>

// ANSI colors for terminal output
static const char* COLOR_RED="[31m";
static const char* COLOR_RESET="[0m";

// Constructor
FollowingVehicle::FollowingVehicle(int id,
                                 double initialSpeed,
                                 double initialPosition)
    : state_(NORMAL), clientRunning_(false), energyAlertSent_(false), clientSocket_(-1),
      isLeader_(false), leaderRunning_(false) {
    info_.id = id;
    info_.position = initialPosition;
    info_.speed = std::round(initialSpeed);
    info_.mode = FollowerMode;

    // Initialize front/rear vehicle info as invalid (id = -1 means unknown)
    frontVehicleInfo_.id = -1;
    frontVehicleInfo_.ipAddress = 0;
    frontVehicleInfo_.port = 0;
    rearVehicleInfo_.id = -1;
    rearVehicleInfo_.ipAddress = 0;
    rearVehicleInfo_.port = 0;

    pthread_mutex_init(&leaderMutex_, nullptr);
    pthread_mutex_init(&eventMutex_, nullptr);
    pthread_cond_init(&eventCv_, nullptr);
}

// Destructor
FollowingVehicle::~FollowingVehicle() {
    clientRunning_ = false;
    if (clientSocket_ >= 0) {
        ::shutdown(clientSocket_, SHUT_RDWR);
        ::close(clientSocket_);
        clientSocket_ = -1;
    }
    pthread_mutex_destroy(&leaderMutex_);
    pthread_mutex_destroy(&eventMutex_);
    pthread_cond_destroy(&eventCv_);
}

// Helpers
void FollowingVehicle::createClientSocket() {
    clientSocket_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (clientSocket_ < 0) {
        throw std::runtime_error(std::string("Failed to create UDP socket: ") + strerror(errno));
    }

    int opt = 1;
    if (setsockopt(clientSocket_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
        ::close(clientSocket_);
        clientSocket_ = -1;
        throw std::runtime_error(std::string("Failed to set SO_REUSEADDR: ") + strerror(errno));
    }

    // Bind to a unique port for this follower (base port + id)
    struct sockaddr_in myAddr{};
    myAddr.sin_family = AF_INET;
    myAddr.sin_addr.s_addr = INADDR_ANY;
    myAddr.sin_port = htons(SERVER_PORT + info_.id); // unique port per follower

    if (bind(clientSocket_, (struct sockaddr*)&myAddr, sizeof(myAddr)) < 0) {
        ::close(clientSocket_);
        clientSocket_ = -1;
        throw std::runtime_error(std::string("Failed to bind socket: ") + strerror(errno));
    }

    // Store our own address info for sending to leader
    info_.ipAddress = htonl(INADDR_LOOPBACK); // our IP (localhost for testing)
    info_.port = htons(SERVER_PORT + info_.id); // our port

    // Setup leader address for sending
    memset(&leaderAddr_, 0, sizeof(leaderAddr_));
    leaderAddr_.sin_family = AF_INET;
    leaderAddr_.sin_port = htons(SERVER_PORT);
    leaderAddr_.sin_addr.s_addr = htonl(INADDR_LOOPBACK); // assume leader on localhost
}

// Main entry point for connecting to the leader
void FollowingVehicle::connectToLeader() {
    createClientSocket();
    clientRunning_ = true;
    // Send couple command once on connect
    sendCoupleCommandToLeader();
}

// Send couple command to leader
void FollowingVehicle::sendCoupleCommandToLeader(bool couple) {
    CoupleCommandMessage cmd{};
    cmd.type = MessageType::COUPLE_COMMAND;
    cmd.info.id = info_.id;
    cmd.info.position = info_.position;
    cmd.info.speed = info_.speed;
    cmd.info.mode = info_.mode;
    cmd.info.ipAddress = info_.ipAddress; // include our address so leader knows where to send
    cmd.info.port = info_.port;
    cmd.couple = couple;
    cmd.timestamp = nowMs();

    ssize_t sentBytes = sendto(clientSocket_, &cmd, sizeof(cmd), 0,
                               (const struct sockaddr*)&leaderAddr_, sizeof(leaderAddr_));
    if (sentBytes < 0) {
        std::cerr << "Failed to send couple command: " << strerror(errno) << std::endl;
    } else {
        std::cout << "Sent couple command (id=" << cmd.info.id
                  << " pos=" << cmd.info.position
                  << " speed=" << cmd.info.speed << ")" << std::endl;
    }
}

// Timing helper
std::int64_t FollowingVehicle::nowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch()).count();
}

// Thread start
void FollowingVehicle::startThreads() {
    pthread_create(&recvThread_, nullptr, recvThreadEntry, this);
    pthread_create(&runThread_, nullptr, runThreadEntry, this);
    pthread_create(&sendStatusThread_, nullptr, sendStatusThreadEntry, this);
    pthread_create(&displayThread_, nullptr, displayThreadEntry, this);
    pthread_create(&eventThread_, nullptr, eventSimulationThreadEntry, this);
    pthread_create(&eventSenderThread_, nullptr, eventSenderThreadEntry, this);
}

// Set follower state
void FollowingVehicle::setState(FollowerState newState) {
    state_ = newState;
}

// Get follower state
FollowerState FollowingVehicle::getState() const {
    return state_;
}

// Receive messages (unconnected UDP - can receive from leader and other vehicles)
void* FollowingVehicle::recvThreadEntry(void* arg) {
    FollowingVehicle* follower = static_cast<FollowingVehicle*>(arg);
    const size_t BUFFER_SIZE = 4096;
    char* buffer = new char[BUFFER_SIZE];
    
    std::cout << "[RECV] Thread started." << std::endl;

    while (follower->clientRunning_) {
        struct sockaddr_in senderAddr{};
        socklen_t addrLen = sizeof(senderAddr);
        ssize_t recvLen = recvfrom(follower->clientSocket_, buffer, BUFFER_SIZE, 0,
                                   (struct sockaddr*)&senderAddr, &addrLen);
        if (recvLen < 0) {
             if (errno != EAGAIN && errno != EINTR) {
                 std::cerr << "[RECV] Error: " << strerror(errno) << std::endl;
             }
             continue;
        }
        if (recvLen >= 1) {
            MessageType msgType = static_cast<MessageType>(buffer[0]);
            
            switch (msgType) {
                case STATUS_UPDATE: {
                    if (static_cast<size_t>(recvLen) < sizeof(StatusUpdateMessage)) break;
                    StatusUpdateMessage status;
                    std::memcpy(&status, buffer, sizeof(status));

                    pthread_mutex_lock(&follower->leaderMutex_);
                    // Always update leader info if this is from the leader
                    if (status.info.mode == LeaderMode) {
                        follower->leaderPosition_ = status.info.position;
                        follower->leaderSpeed_ = status.info.speed;
                    }
                    // Update front vehicle info if this status is from our front vehicle
                    if (status.info.id == follower->frontVehicleInfo_.id) {
                        follower->frontVehicleInfo_.position = status.info.position;
                        follower->frontVehicleInfo_.speed = status.info.speed;
                    }
                    pthread_mutex_unlock(&follower->leaderMutex_);
                    break;
                }
                case TRAFFIC_LIGHT_ALERT: {
                    if (static_cast<size_t>(recvLen) < sizeof(TrafficLightMessage)) break;
                    TrafficLightMessage trafficMsg;
                    std::memcpy(&trafficMsg, buffer, sizeof(trafficMsg));
                    TrafficLightStatus alert = static_cast<TrafficLightStatus>(trafficMsg.status);
                    std::cout << "Received traffic light alert: "
                              << (alert == LIGHT_RED ? "RED" : "GREEN") << std::endl;
                    
                                        // Store latest light state
                    pthread_mutex_lock(&follower->leaderMutex_);
                    follower->trafficLight_ = alert;
                    pthread_mutex_unlock(&follower->leaderMutex_);

                    // Process the alert (delay + decoupled logic)
                    if (alert == LIGHT_RED) {
                        follower->setState(STOPPING_FOR_RED_LIGHT);
                    } else if (alert == LIGHT_GREEN) {
                        bool isDecoupled = false;
                        bool hasDelay = false;
                        int delaySec = 0;
                        pthread_mutex_lock(&follower->leaderMutex_);
                        isDecoupled = follower->decoupled_;
                        hasDelay = follower->delayAfterNextGreenArmed_;
                        delaySec = follower->delayAfterNextGreenSec_;
                        if (hasDelay) {
                            follower->delayedUntilMs_ = nowMs() + static_cast<std::int64_t>(delaySec) * 1000;
                            follower->delayAfterNextGreenArmed_ = false; // consume
                        }
                        pthread_mutex_unlock(&follower->leaderMutex_);

                        if (isDecoupled) {
                            follower->setState(DECOUPLED);
                        } else if (hasDelay && delaySec > 0) {
                            // Stay stopped until delayedUntilMs_ expires (handled in run loop)
                            follower->setState(STOPPED);
                        } else {
                            follower->setState(STARTING);
                        }
                    }

                    // Forward to rear vehicle
                    pthread_mutex_lock(&follower->leaderMutex_);
                    VehicleInfo rear = follower->rearVehicleInfo_;
                    pthread_mutex_unlock(&follower->leaderMutex_);

                    if (rear.id != -1 && rear.ipAddress != 0 && rear.port != 0) {
                        struct sockaddr_in rearAddr{};
                        rearAddr.sin_family = AF_INET;
                        rearAddr.sin_addr.s_addr = rear.ipAddress;
                        rearAddr.sin_port = rear.port;
                        sendto(follower->clientSocket_, &trafficMsg, sizeof(trafficMsg), 0,
                               (const struct sockaddr*)&rearAddr, sizeof(rearAddr));
                    }
                    break;
                }
                case ENERGY_DEPLETION_ALERT: {
                    if (static_cast<size_t>(recvLen) < sizeof(EnergyDepletionMessage)) break;
                    EnergyDepletionMessage energyMsg;
                    std::memcpy(&energyMsg, buffer, sizeof(energyMsg));
                    std::cout << "Received energy depletion alert from vehicle " << energyMsg.vehicleId << std::endl;

                    // If it's from the front vehicle, perhaps adjust behavior, but for now just log
                    // Forward to rear vehicle
                    pthread_mutex_lock(&follower->leaderMutex_);
                    VehicleInfo rear = follower->rearVehicleInfo_;
                    pthread_mutex_unlock(&follower->leaderMutex_);

                    if (rear.id != -1 && rear.ipAddress != 0 && rear.port != 0) {
                        struct sockaddr_in rearAddr{};
                        rearAddr.sin_family = AF_INET;
                        rearAddr.sin_addr.s_addr = rear.ipAddress;
                        rearAddr.sin_port = rear.port;
                        sendto(follower->clientSocket_, &energyMsg, sizeof(energyMsg), 0,
                               (const struct sockaddr*)&rearAddr, sizeof(rearAddr));
                    }
                    break;
                }
                case PLATOON_STATE: {
                    if (static_cast<size_t>(recvLen) < sizeof(PlatoonStateMessage)) break;
                    PlatoonStateMessage psMsg;
                    std::memcpy(&psMsg, buffer, sizeof(psMsg));

                    // DEBUG RECV
                    /* 
                    if (psMsg.vehicleCount > 1) {
                         for (int i=0; i<psMsg.vehicleCount; ++i) {
                             if (psMsg.vehicles[i].id < 0 && psMsg.vehicles[i].id != -1) {
                                 std::cerr << "[CRITICAL_RECV] Received CORRUPT ID " << psMsg.vehicles[i].id << " at index " << i << std::endl;
                             }
                         }
                    }
                    */

                    pthread_mutex_lock(&follower->leaderMutex_);
                    // Update platoon state snapshot
                    int myIndex = -1;
                    for (int i = 0; i < psMsg.vehicleCount && i < MAX_PLATOON_VEHICLES; ++i) {
                        if (psMsg.vehicles[i].id == follower->info_.id) {
                            myIndex = i;
                            break;
                        }
                    }

                    // Auto-rejoin if dropped from platoon
                    if (myIndex == -1 && !follower->isLeader_) {
                         std::int64_t now = nowMs();
                         if (now - follower->lastReconnectAttemptMs_ > 2000) {
                             follower->lastReconnectAttemptMs_ = now;
                             std::cout << "[FOLLOWER " << follower->info_.id << "] Not in platoon list (Leader " << psMsg.leaderId << "). Attempting to rejoin...\n";
                             
                             // Unlock to send
                             pthread_mutex_unlock(&follower->leaderMutex_);
                             follower->sendCoupleCommandToLeader();
                             pthread_mutex_lock(&follower->leaderMutex_);
                         }
                    }

                    // Update platoon state snapshot
                    follower->platoonState_.leaderId = psMsg.leaderId;
                    follower->platoonState_.vehicles.clear();
                    for (int i = 0; i < psMsg.vehicleCount && i < MAX_PLATOON_VEHICLES; ++i) {
                        follower->platoonState_.vehicles.push_back(psMsg.vehicles[i]);
                    }

                    // Front vehicle is at myIndex - 1 (closer to leader)
                    if (myIndex > 0) {
                        follower->frontVehicleInfo_ = psMsg.vehicles[myIndex - 1];
                    } else {
                        follower->frontVehicleInfo_.id = -1; // no front (I'm right behind leader or not found)
                    }

                    // Rear vehicle is at myIndex + 1
                    if (myIndex >= 0 && myIndex + 1 < psMsg.vehicleCount) {
                        follower->rearVehicleInfo_ = psMsg.vehicles[myIndex + 1];
                    } else {
                        follower->rearVehicleInfo_.id = -1; // no rear vehicle
                    }
                    // If we find ourselves in the platoon list, we successfully rejoined
                    if (myIndex != -1) {
                        follower->tryingRejoin_ = false;
                        // Also ensure we are not marked decoupled
                        follower->decoupled_ = false;
                    }
                    pthread_mutex_unlock(&follower->leaderMutex_);
                    break;
                }                
                case OBSTACLE_DETECTED_ALERT: {
                    if (static_cast<size_t>(recvLen) < sizeof(ObstacleMessage)) break;
                    ObstacleMessage obsMsg;
                    std::memcpy(&obsMsg, buffer, sizeof(obsMsg));
                    bool detected = obsMsg.obstacleDetected;
                    std::cout << "Received obstacle detected alert: "
                              << (detected ? "DETECTED" : "CLEARED") << std::endl;

                    if (detected) {
                        follower->setState(STOPPING_FOR_OBSTACLE);
                    } else {
                        if (follower->getState() == STOPPED || follower->getState() == STOPPING_FOR_OBSTACLE) {
                            follower->setState(NORMAL);
                        }
                    }
                    // Process obstacle detection (for now just log)
                    // Forward to rear vehicle
                    pthread_mutex_lock(&follower->leaderMutex_);
                    VehicleInfo rear = follower->rearVehicleInfo_;
                    pthread_mutex_unlock(&follower->leaderMutex_);

                    if (rear.id != -1 && rear.ipAddress != 0 && rear.port != 0) {
                        struct sockaddr_in rearAddr{};
                        rearAddr.sin_family = AF_INET;
                        rearAddr.sin_addr.s_addr = rear.ipAddress;
                        rearAddr.sin_port = rear.port;
                        sendto(follower->clientSocket_, &obsMsg, sizeof(obsMsg), 0,
                               (const struct sockaddr*)&rearAddr, sizeof(rearAddr));
                    }
                        }
                    break;
                case REMOVE_VEHICLE: {
                    if (static_cast<size_t>(recvLen) < sizeof(RemoveVehicleMessage)) break;
                    // Pause 5 seconds before attempting to rejoin
                    follower->setState(STOPPED);
                    std::this_thread::sleep_for(std::chrono::seconds(5));
                    follower->setState(NORMAL);
                    RemoveVehicleMessage removeMsg;
                    std::memcpy(&removeMsg, buffer, sizeof(removeMsg));
                    if (removeMsg.vehicleId == follower->info_.id) {
                        std::cout << COLOR_RED << "[FOLLOWER " << follower->info_.id << "] Received remove notification. Will request rejoin.\n" << COLOR_RESET;
                        // Reset local platoon snapshot and mark for periodic rejoin attempts
                        pthread_mutex_lock(&follower->leaderMutex_);
                        follower->leaderPosition_ = 0.0;
                        follower->leaderSpeed_ = 0.0;
                        follower->lastLeaderUpdateMs_ = 0;
                        follower->frontVehicleInfo_.id = -1;
                        follower->rearVehicleInfo_.id = -1;
                        follower->platoonState_.vehicles.clear();
                        follower->trafficLight_ = LIGHT_GREEN;
                        // Ensure we are not marked decoupled so run loop treats us as NORMAL
                        follower->decoupled_ = false;
                        follower->delayedUntilMs_ = 0;
                        follower->setState(NORMAL);
                        pthread_mutex_unlock(&follower->leaderMutex_);

                        // Ensure we resume sending status so leader updates heartbeat timestamp
                        follower->stopSendingStatus_ = false;
                        // Send an immediate status and a couple command to rejoin
                        StatusUpdateMessage status{};
                        status.type = MessageType::STATUS_UPDATE;
                        status.info = follower->info_;
                        status.timestamp = nowMs();
                        sendto(follower->clientSocket_, &status, sizeof(status), 0,
                               (const struct sockaddr*)&follower->leaderAddr_, sizeof(follower->leaderAddr_));
                        follower->tryingRejoin_ = true;
                        follower->lastReconnectAttemptMs_ = nowMs();
                        follower->sendCoupleCommandToLeader(true);
                    }
                    break;
                }
                case LEAVE_PLATOON: {
                    // If receive from leader, it means this vehicle will be new leader
                    if (static_cast<size_t>(recvLen) < sizeof(LeavePlatoonMessage)) break;
                    LeavePlatoonMessage leaveMsg;
                    std::memcpy(&leaveMsg, buffer, sizeof(leaveMsg));
                    std::cout << "Received LEAVE_PLATOON command for vehicle "
                              << leaveMsg.vehicleId << std::endl;
                    // If the vehicle leaving is the current leader, we become the new leader
                    if (leaveMsg.vehicleId == follower->platoonState_.leaderId) {
                        std::cout << "\n========================================\n";
                        std::cout << "TRANSITION: Becoming new leader as vehicle " << leaveMsg.vehicleId << " left platoon.\n";
                        std::cout << "========================================\n\n";
                        
                        // // Transition to leader mode within same process
                        follower->transitionToLeader();
                        return nullptr;
                    } else if (leaveMsg.vehicleId == follower->rearVehicleInfo_.id) {
                        // Leave command from rear vehicle, send it to our front vehicle
                        // and update our rear vehicle info to the vehicle behind the one that left
                        pthread_mutex_lock(&follower->leaderMutex_);
                        VehicleInfo front = follower->frontVehicleInfo_;
                        pthread_mutex_unlock(&follower->leaderMutex_);
                        if (front.id != -1 && front.ipAddress != 0 && front.port != 0) {
                            struct sockaddr_in frontAddr{};
                            frontAddr.sin_family = AF_INET;
                            frontAddr.sin_addr.s_addr = front.ipAddress;
                            frontAddr.sin_port = front.port;
                            sendto(follower->clientSocket_, &leaveMsg, sizeof(leaveMsg), 0,
                                   (const struct sockaddr*)&frontAddr, sizeof(frontAddr));
                        }
                        // Update rear vehicle info to the one behind the leaving vehicle
                        pthread_mutex_lock(&follower->leaderMutex_);
                        auto it = std::find_if(
                            follower->platoonState_.vehicles.begin(),
                            follower->platoonState_.vehicles.end(),
                            [&](const VehicleInfo& v){ return v.id == leaveMsg.vehicleId; });
                        if (it != follower->platoonState_.vehicles.end() && (it + 1) != follower->platoonState_.vehicles.end()) {
                            follower->rearVehicleInfo_ = *(it + 1);
                        } else {
                            follower->rearVehicleInfo_.id = -1; // no rear vehicle
                        }
                        pthread_mutex_unlock(&follower->leaderMutex_);
                    } else if (leaveMsg.vehicleId == follower->frontVehicleInfo_.id) {
                        // Leave command from front vehicle, update front vehicle info
                        pthread_mutex_lock(&follower->leaderMutex_);
                        
                        // Check if the leaving vehicle is the leader
                        if (leaveMsg.vehicleId == follower->platoonState_.leaderId) {
                            // Leader is leaving, find the new leader (first vehicle after old leader)
                            if (follower->platoonState_.vehicles.size() > 1) {
                                // New leader is the second vehicle in the list
                                follower->frontVehicleInfo_ = follower->platoonState_.vehicles[1];
                                follower->platoonState_.leaderId = follower->platoonState_.vehicles[1].id;
                                std::cout << "[UPDATE] Front vehicle updated to new leader ID " 
                                         << follower->frontVehicleInfo_.id << "\n";
                            } else {
                                follower->frontVehicleInfo_.id = -1; // no front vehicle
                            }
                        } else {
                            // Regular follower leaving, find the vehicle before it
                            auto it = std::find_if(
                                follower->platoonState_.vehicles.begin(),
                                follower->platoonState_.vehicles.end(),
                                [&](const VehicleInfo& v){ return v.id == leaveMsg.vehicleId; });
                            if (it != follower->platoonState_.vehicles.end() && it != follower->platoonState_.vehicles.begin()) {
                                follower->frontVehicleInfo_ = *(it - 1);
                            } else {
                                follower->frontVehicleInfo_.id = -1; // no front vehicle    
                            }
                        }
                        pthread_mutex_unlock(&follower->leaderMutex_);
                    }
                    break;
                }
                default:
                    // ignore other message types for now
                    break;
            }
        }
    }
    delete[] buffer;
    return nullptr;
}

void* FollowingVehicle::runThreadEntry(void* arg) {
    FollowingVehicle* follower = static_cast<FollowingVehicle*>(arg);

    const int TIMESTEP_MS = 100;
    const double dt = TIMESTEP_MS / 1000.0;
    const double accel = 8;   // m/s^2
    const double decel = 12;  // m/s^2
    double targetSpeed = follower->info_.speed;

    while (follower->clientRunning_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(TIMESTEP_MS));

        // Read front vehicle and leader snapshot (thread-safe)
        double frontPos = 0.0;
        double frontSp = 0.0;
        double leaderPos = 0.0;
        double leaderSp = 0.0;
        int frontId = -1;
        std::uint8_t frontMode = FollowerMode;

        pthread_mutex_lock(&follower->leaderMutex_);
        leaderPos = follower->leaderPosition_;
        leaderSp = follower->leaderSpeed_;
        frontId = follower->frontVehicleInfo_.id;
        frontMode = follower->frontVehicleInfo_.mode;
        frontPos = follower->frontVehicleInfo_.position;
        frontSp = follower->frontVehicleInfo_.speed;
        pthread_mutex_unlock(&follower->leaderMutex_);

        // Snapshot traffic light + decouple/delay state (protected by leaderMutex_)
        TrafficLightStatus light = LIGHT_GREEN;
        std::int64_t delayedUntil = 0;
        bool isDecoupled = false;
        bool tryingRejoin = false;
        pthread_mutex_lock(&follower->leaderMutex_);
        light = follower->trafficLight_;
        delayedUntil = follower->delayedUntilMs_;
        isDecoupled = follower->decoupled_;
        tryingRejoin = follower->tryingRejoin_;
        pthread_mutex_unlock(&follower->leaderMutex_);

        // Determine which vehicle to follow:
        // - If front vehicle is unknown (id == -1) or IS the leader, follow leader
        // - Otherwise follow the front vehicle
        double refPos, refSp;
        if (isDecoupled || frontId == -1 || frontMode == LeaderMode) {
            refPos = leaderPos;
            refSp = leaderSp;
        } else {
            refPos = frontPos;
            refSp = frontSp;
        }

        double gap = refPos - follower->info_.position;

        // If we are delaying start after GREEN, hold still until the timer expires
        const std::int64_t now = nowMs();
        if (!isDecoupled && light == LIGHT_GREEN && delayedUntil > now) {
            follower->info_.speed = 0.0;
            follower->setState(STOPPED);
        } else if (!isDecoupled && light == LIGHT_GREEN && delayedUntil != 0 && delayedUntil <= now && follower->getState() == STOPPED) {
            pthread_mutex_lock(&follower->leaderMutex_);
            follower->delayedUntilMs_ = 0;
            pthread_mutex_unlock(&follower->leaderMutex_);
            follower->setState(STARTING);
        }

        // Detect "left-behind" on GREEN when leader is moving and this follower is still stopped
        // Don't mark decoupled if we're currently attempting to rejoin after removal
        if (!isDecoupled && !tryingRejoin && light == LIGHT_GREEN) {
            const bool leaderMoving = (leaderSp > 0.5);
            const bool thisStopped = (follower->info_.speed < LEFT_BEHIND_STOPPED_EPS);
            if (leaderMoving && thisStopped && gap > LEFT_BEHIND_GAP_THRESHOLD) {
                pthread_mutex_lock(&follower->leaderMutex_);
                follower->decoupled_ = true;
                follower->delayedUntilMs_ = 0;
                pthread_mutex_unlock(&follower->leaderMutex_);

                follower->sendCoupleCommandToLeader(false); // temporarily leave platoon
                follower->setState(DECOUPLED);
                isDecoupled = true;
            }
        }

        // Simple state machine for follower behavior
        FollowerState currentState = follower->getState();
        switch (currentState)
        {
            case NORMAL: 
                if (gap < SAFE_DISTANCE) {
                    follower->setState(STOPPING);
                } else if (gap > (SAFE_DISTANCE + 20.0)) {
                    follower->setState(CATCHING_UP);
                } else {
                    // Maintain speed - match the vehicle in front
                    follower->info_.speed = refSp;
                }
                break;
            case ERROR: 
                break;
            case STARTING:
                follower->info_.speed = std::round(std::min(targetSpeed, follower->info_.speed + accel * dt));
                if (follower->info_.speed >= targetSpeed - 1e-6) {
                    follower->setState(NORMAL);
                }
                break;
            case CATCHING_UP:
                if ((gap >= SAFE_DISTANCE) && (gap <= (SAFE_DISTANCE + 20.0))) {
                    follower->setState(NORMAL);
                } else if (gap > (SAFE_DISTANCE + 100.0)) {
                    // Far behind - speed up aggressively
                    follower->info_.speed = std::round(std::min(refSp + 30.0, follower->info_.speed + accel * dt));
                } else if (gap > (SAFE_DISTANCE + 20.0)) {
                    // Moderately behind - speed up gently
                    follower->info_.speed = std::round(std::min(refSp + 5.0, follower->info_.speed + accel * dt));
                }
                break;
            case STOPPING:
                if (gap >= (SAFE_DISTANCE + 5.0)) {
                    follower->setState(NORMAL);
                } else if (follower->info_.speed > 0.0) {
                    follower->info_.speed = std::round(std::max(0.0, follower->info_.speed - decel * dt));
                    if (follower->info_.speed <= 0.001) {
                        follower->info_.speed = 0.0;
                        // follower->setState(STOPPED);
                    }
                }
                break;
            case STOPPING_FOR_RED_LIGHT:
            case STOPPING_FOR_OBSTACLE:
                if (follower->info_.speed > 0.0) {
                    follower->info_.speed = std::round(std::max(0.0, follower->info_.speed - decel * dt * 1.32));
                    if (follower->info_.speed <= 0.001) {
                        follower->info_.speed = 0.0;
                        follower->setState(STOPPED);
                    }
                }
                break;
            case STOPPED:
                follower->info_.speed = 0.0;
                break;
            case DECOUPLED:
                // On RED, remain stopped. On GREEN, catch up to the leader until the target gap is reached.
                if (light == LIGHT_RED) {
                    follower->info_.speed = 0.0;
                    break;
                }

                // Rejoin when close to desired following distance
                if (gap >= (SAFE_DISTANCE - REJOIN_TOLERANCE) && gap <= (SAFE_DISTANCE + REJOIN_TOLERANCE)) {
                    pthread_mutex_lock(&follower->leaderMutex_);
                    follower->decoupled_ = false;
                    pthread_mutex_unlock(&follower->leaderMutex_);

                    follower->sendCoupleCommandToLeader(true);
                    follower->setState(NORMAL);
                    follower->info_.speed = refSp;
                    break;
                }

                // Otherwise, accelerate above leader speed (bounded)
                follower->info_.speed = std::round(std::min(leaderSp + CATCH_UP_SPEED_BONUS,
                                                           follower->info_.speed + accel * dt));
                break;
            case LOW_ENERGY:
                // Continue following but cap speed at target speed
                if (gap < SAFE_DISTANCE) {
                    follower->setState(STOPPING);
                } else if (gap > (SAFE_DISTANCE + 20.0)) {
                    follower->setState(CATCHING_UP);
                } else {
                    // Maintain speed - match the vehicle in front, but don't exceed target speed
                    double desiredSpeed = std::min(refSp, follower->targetSpeed_);
                    follower->info_.speed = desiredSpeed;
                }
                break;
            default:
                break;
        }
        // Integrate position
        follower->info_.position += follower->info_.speed * dt;
    }
    return nullptr;
}

void* FollowingVehicle::sendStatusThreadEntry(void* arg) {
    FollowingVehicle* follower = static_cast<FollowingVehicle*>(arg);
    const int TIMESTEP_MS = 200; // send status at 5 Hz

    while (follower->clientRunning_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(TIMESTEP_MS));

        // Check if we should stop sending status
        std::int64_t now = nowMs();
        if (follower->stopSendingStatus_ && now > follower->resumeSendingMs_) {
            follower->stopSendingStatus_ = false;
        }
        if (follower->stopSendingStatus_) {
            continue;
        }

        StatusUpdateMessage status{};
        status.type = MessageType::STATUS_UPDATE;
        // Copy current info (no deep locks needed for simple struct copy)
        status.info = follower->info_;
        status.timestamp = now;

        // Send to leader (using sendto since socket is not connected)
        ssize_t sentBytes = sendto(follower->clientSocket_, &status, sizeof(status), 0,
                                   (const struct sockaddr*)&follower->leaderAddr_, sizeof(follower->leaderAddr_));
        if (sentBytes < 0) {
            std::cerr << "Failed to send status to leader: " << strerror(errno) << std::endl;
        }

        // Also send to rear vehicle if known
        pthread_mutex_lock(&follower->leaderMutex_);
        VehicleInfo rear = follower->rearVehicleInfo_;
        pthread_mutex_unlock(&follower->leaderMutex_);

        if (rear.id != -1 && rear.ipAddress != 0 && rear.port != 0) {
            struct sockaddr_in rearAddr{};
            rearAddr.sin_family = AF_INET;
            rearAddr.sin_addr.s_addr = rear.ipAddress;
            rearAddr.sin_port = rear.port;
            ssize_t rearSent = sendto(follower->clientSocket_, &status, sizeof(status), 0,
                                      (const struct sockaddr*)&rearAddr, sizeof(rearAddr));
            if (rearSent < 0) {
                std::cerr << "Failed to send status to rear vehicle " << rear.id << ": " << strerror(errno) << std::endl;
            }
        }
    }
    return nullptr;
}

void* FollowingVehicle::displayThreadEntry(void* arg) {
    FollowingVehicle* follower = static_cast<FollowingVehicle*>(arg);
    while (follower->clientRunning_) {
        std::this_thread::sleep_for(std::chrono::seconds(1));

        pthread_mutex_lock(&follower->leaderMutex_);
        double leaderPos = follower->leaderPosition_;
        double leaderSp = follower->leaderSpeed_;
        int frontId = follower->frontVehicleInfo_.id;
        double frontPos = follower->frontVehicleInfo_.position;
        double frontSp = follower->frontVehicleInfo_.speed;
        std::uint8_t frontMode = follower->frontVehicleInfo_.mode;
        int rearId = follower->rearVehicleInfo_.id;
        int leaderId = follower->platoonState_.leaderId;
        pthread_mutex_unlock(&follower->leaderMutex_);

        // Calculate gap to front vehicle (or leader if front is leader/unknown)
        double refPos = (frontId == -1 || frontMode == LeaderMode) ? leaderPos : frontPos;
        double gap = refPos - follower->info_.position;

        FollowerState state = follower->getState();
        std::string stateStr;
        switch (state) {
            case NORMAL: stateStr = "NORMAL"; break;
            case ERROR: stateStr = "ERROR"; break;
            case STOPPING: stateStr = "STOPPING"; break;
            case STOPPED: stateStr = "STOPPED"; break;
            case STARTING: stateStr = "STARTING"; break;
            case CATCHING_UP: stateStr = "CATCHING_UP"; break;
            case STOPPING_FOR_RED_LIGHT: stateStr = "STOPPING_FOR_RED_LIGHT"; break;
            case STOPPING_FOR_OBSTACLE: stateStr = "STOPPING_FOR_OBSTACLE"; break;
            case DECOUPLED: stateStr = "DECOUPLED"; break;
            case LOW_ENERGY: stateStr = "LOW_ENERGY"; break;
            default: stateStr = "UNKNOWN"; break;
        }
        std::cout << "\033[2J\033[H"; // clear screen + home
        std::cout << "\n================== FOLLOWER " << follower->info_.id << " ==================\n";
        std::cout << "Position: " << follower->info_.position
                  << "  Speed: " << follower->info_.speed
                  << "  State: " << stateStr << "\n";
        std::cout << "Gap to front: " << gap << " m\n";
        std::cout << "Front vehicle: " << ((frontId == -1 || frontMode == LeaderMode) ? ("Leader (ID " + std::to_string(frontId == -1 ? leaderId : frontId) + ")") : ("ID " + std::to_string(frontId)))
                  << " (pos=" << refPos << ", spd=" << (frontId == -1 || frontMode == LeaderMode ? leaderSp : frontSp) << ")\n";
        std::cout << "Rear vehicle: " << (rearId == -1 ? "None" : ("ID " + std::to_string(rearId))) << "\n";
        std::cout << std::flush;
    }
    return nullptr;
}

// Event simulation thread - reads user input from FIFO
void* FollowingVehicle::eventSimulationThreadEntry(void* arg) {
    FollowingVehicle* follower = static_cast<FollowingVehicle*>(arg);

    // FIFO already created in main()
    std::string fifoPath = "/tmp/follower_" + std::to_string(follower->info_.id) + "_event_fifo";

    int fd = open(fifoPath.c_str(), O_RDONLY);
    if (fd == -1) return nullptr;

    int eventChoice;
    while (follower->clientRunning_) {
        if (read(fd, &eventChoice, sizeof(int)) > 0) {
            std::cout << "\n[FOLLOWER " << follower->info_.id << "] Received Event: " << eventChoice << std::endl;

            if (eventChoice == 0) {
                follower->clientRunning_ = false;
                break;
            }

            // Push event to queue for sender thread
            pthread_mutex_lock(&follower->eventMutex_);
            follower->eventQueue_.push(eventChoice);
            pthread_cond_signal(&follower->eventCv_);
            pthread_mutex_unlock(&follower->eventMutex_);
        }
    }
    close(fd);
    unlink(fifoPath.c_str());
    return nullptr;
}

// Event sender thread - sends events to front and rear vehicles
void* FollowingVehicle::eventSenderThreadEntry(void* arg) {
    FollowingVehicle* follower = static_cast<FollowingVehicle*>(arg);

    while (follower->clientRunning_) {
        // Wait for event
        pthread_mutex_lock(&follower->eventMutex_);
        while (follower->eventQueue_.empty() && follower->clientRunning_) {
            pthread_cond_wait(&follower->eventCv_, &follower->eventMutex_);
        }
        if (!follower->clientRunning_) {
            pthread_mutex_unlock(&follower->eventMutex_);
            break;
        }
        int eventCode = follower->eventQueue_.front();
        follower->eventQueue_.pop();
        pthread_mutex_unlock(&follower->eventMutex_);

        // Special input: arm a start-delay after the next GREEN
        // We encode delay as 1000 + seconds via FIFO (still only one int)
        if (eventCode >= 1000) {
            int sec = eventCode - 1000;
            pthread_mutex_lock(&follower->leaderMutex_);
            follower->delayAfterNextGreenArmed_ = true;
            follower->delayAfterNextGreenSec_ = sec;
            pthread_mutex_unlock(&follower->leaderMutex_);
            std::cout << "[FOLLOWER " << follower->info_.id << "] Armed start-delay after next GREEN: " << sec << "s" << std::endl;
            continue;
        }

        // Build event message
        EventMessage eventMsg{};
        eventMsg.timestamp = nowMs();

        switch (eventCode) {
            case 1: // Red light
                eventMsg.type = MessageType::TRAFFIC_LIGHT_ALERT;
                eventMsg.eventData = (void*)LIGHT_RED;
                follower->setState(STOPPING_FOR_RED_LIGHT);
                break;
            case 2: // Green light
                eventMsg.type = MessageType::TRAFFIC_LIGHT_ALERT;
                eventMsg.eventData = (void*)LIGHT_GREEN;
                follower->setState(STARTING);
                break;
            case 3: // Run out of energy
                eventMsg.type = MessageType::ENERGY_DEPLETION_ALERT;
                // Set energy to 0, reduce speed to 60%, send alert immediately
                follower->targetSpeed_ = follower->info_.speed * 0.6;
                follower->setState(LOW_ENERGY);
                break;
            case 5: // Simulate communication lost 3 seconds
                std::cout << "[FOLLOWER " << follower->info_.id << "] Simulating communication loss for 3 seconds...\n";
                follower->stopSendingStatus_ = true;
                follower->resumeSendingMs_ = nowMs() + 3000;
                break;
            case 6: // Simulate communication lost 5 seconds
                std::cout << "[FOLLOWER " << follower->info_.id << "] Simulating communication loss for 5 seconds...\n";
                follower->stopSendingStatus_ = true;
                follower->resumeSendingMs_ = nowMs() + 5000 + 15000; // 5s + 15s delay
                break;
            default:
                continue; // Skip unknown events
        }

        // Get front and rear vehicle info
        pthread_mutex_lock(&follower->leaderMutex_);
        VehicleInfo front = follower->frontVehicleInfo_;
        VehicleInfo rear = follower->rearVehicleInfo_;
        pthread_mutex_unlock(&follower->leaderMutex_);

        // Send event message to front and rear vehicles
        // Send to front vehicle
        if (front.id != -1 && front.ipAddress != 0 && front.port != 0) {
            struct sockaddr_in frontAddr{};
            frontAddr.sin_family = AF_INET;
            frontAddr.sin_addr.s_addr = front.ipAddress;
            frontAddr.sin_port = front.port;

            ssize_t sentBytes = 0;
            switch (eventMsg.type) {
                case TRAFFIC_LIGHT_ALERT: {
                    TrafficLightMessage tlMsg{};
                    tlMsg.type = MessageType::TRAFFIC_LIGHT_ALERT;
                    tlMsg.timestamp = eventMsg.timestamp;
                    tlMsg.status = (TrafficLightStatus)(uintptr_t)eventMsg.eventData;
                    sentBytes = sendto(follower->clientSocket_, &tlMsg, sizeof(tlMsg), 0,
                                       (const struct sockaddr*)&frontAddr, sizeof(frontAddr));
                    break;
                }
                case ENERGY_DEPLETION_ALERT: {
                    EnergyDepletionMessage energyMsg{};
                    energyMsg.type = MessageType::ENERGY_DEPLETION_ALERT;
                    energyMsg.vehicleId = follower->info_.id;
                    energyMsg.timestamp = eventMsg.timestamp;
                    sentBytes = sendto(follower->clientSocket_, &energyMsg, sizeof(energyMsg), 0,
                                       (const struct sockaddr*)&frontAddr, sizeof(frontAddr));
                    break;
                }
                default:
                    break;
            }

            if (sentBytes < 0) {
                std::cerr << "Failed to send event to front vehicle " << front.id << ": " << strerror(errno) << std::endl;
            }
        }

        // Send to rear vehicle
        if (rear.id != -1 && rear.ipAddress != 0 && rear.port != 0) {
            struct sockaddr_in rearAddr{};
            rearAddr.sin_family = AF_INET;
            rearAddr.sin_addr.s_addr = rear.ipAddress;
            rearAddr.sin_port = rear.port;

            ssize_t sentBytes = 0;
            switch (eventMsg.type) {
                case TRAFFIC_LIGHT_ALERT: {
                    TrafficLightMessage tlMsg{};
                    tlMsg.type = MessageType::TRAFFIC_LIGHT_ALERT;
                    tlMsg.timestamp = eventMsg.timestamp;
                    tlMsg.status = (TrafficLightStatus)(uintptr_t)eventMsg.eventData;
                    sentBytes = sendto(follower->clientSocket_, &tlMsg, sizeof(tlMsg), 0,
                                       (const struct sockaddr*)&rearAddr, sizeof(rearAddr));
                    break;
                }
                case ENERGY_DEPLETION_ALERT: {
                    EnergyDepletionMessage energyMsg{};
                    energyMsg.type = MessageType::ENERGY_DEPLETION_ALERT;
                    energyMsg.vehicleId = follower->info_.id;
                    energyMsg.timestamp = eventMsg.timestamp;
                    sentBytes = sendto(follower->clientSocket_, &energyMsg, sizeof(energyMsg), 0,
                                       (const struct sockaddr*)&rearAddr, sizeof(rearAddr));
                    break;
                }
                default:
                    break;
            }

            if (sentBytes < 0) {
                std::cerr << "Failed to send event to rear vehicle " << rear.id << ": " << strerror(errno) << std::endl;
            }
        }

        // For energy depletion, also send directly to leader
        if (eventMsg.type == ENERGY_DEPLETION_ALERT) {
            EnergyDepletionMessage energyMsg{};
            energyMsg.type = MessageType::ENERGY_DEPLETION_ALERT;
            energyMsg.vehicleId = follower->info_.id;
            energyMsg.timestamp = eventMsg.timestamp;
            ssize_t sentBytes = sendto(follower->clientSocket_, &energyMsg, sizeof(energyMsg), 0,
                                       (const struct sockaddr*)&follower->leaderAddr_, sizeof(follower->leaderAddr_));
            if (sentBytes < 0) {
                std::cerr << "Failed to send energy depletion alert to leader: " << strerror(errno) << std::endl;
            } else {
                std::cout << "Sent energy low alert (id=" << energyMsg.vehicleId << ")\n";
                follower->energyAlertSent_ = true; // Prevent sending again in status thread
            }
        }

        std::cout << "[FOLLOWER " << follower->info_.id << "] Sent event to neighbors" << std::endl;
    }
    return nullptr;
}

int main(int argc, char* argv[]) {
    // Check if running in input_mode (separate terminal for event input)
    if (argc >= 3 && std::string(argv[1]) == "input_mode") {
        int followerId = std::stoi(argv[2]);
        std::string fifoPath = "/tmp/follower_" + std::to_string(followerId) + "_event_fifo";
        
        int fd = open(fifoPath.c_str(), O_WRONLY);
        if (fd == -1) {
            std::cerr << "Failed to open FIFO: " << fifoPath << std::endl;
            exit(1);
        }

        int choice;
        while (true) {
            std::cout << "\033[2J\033[H"; // clear screen + home
            std::cout << "\n[FOLLOWER " << followerId << " EVENT INPUT]\n";
            std::cout << "1: Traffic light RED\n";
            std::cout << "2: Traffic light GREEN\n";
            std::cout << "3: Run out of energy\n";
            std::cout << "4: Set delay after next GREEN (seconds)\n";
            std::cout << "5: Simulate comm loss (3s)\n";
            std::cout << "6: Simulate comm loss (5s)\n";
            std::cout << "0: Exit\n";
            std::cout << "Enter choice: ";
            std::cin >> choice;

            if (choice == 4) {
                int sec = 0;
                std::cout << "Delay seconds: ";
                std::cin >> sec;
                int code = 1000 + sec;
                write(fd, &code, sizeof(int));
                continue;
            }

            write(fd, &choice, sizeof(int));
            if (choice == 0) break;
        }
        close(fd);
        return 0;
    }

    // Normal follower mode
    int id = FOLLOWER_INITIAL_ID;
    setbuf(stdout, NULL);
    double initialSpeed = FOLLOWER_INITIAL_SPEED;
    double initialPosition = FOLLOWER_INITIAL_POSITION;
    if (argc >= 2) id = std::stoi(argv[1]);

    // New argument form: ./follow id initialPosition
    // If provided, argv[2] is treated as the initial position (meters).
    if (argc >= 3) initialPosition = std::stod(argv[2]);

    // Create unique FIFO for this follower
    std::string fifoPath = "/tmp/follower_" + std::to_string(id) + "_event_fifo";
    unlink(fifoPath.c_str());
    mkfifo(fifoPath.c_str(), 0666);

    // Fork a new terminal for event input.
    // Try several common terminal emulators; if none can be launched (e.g., in WSL/no GUI),
    // fall back to running `input_mode` in the child process so the user can still input events.
    pid_t child = fork();
    if (child == 0) {
        std::string idStr = std::to_string(id);
        const char* terms[] = {"gnome-terminal", "x-terminal-emulator", "xfce4-terminal", "lxterminal", "mate-terminal", "konsole", "xterm"};
        for (const char* term : terms) {
            // Try common argument forms; successful execlp will not return.
            execlp(term, term, "--", argv[0], "input_mode", idStr.c_str(), NULL);
            execlp(term, term, "-e", argv[0], "input_mode", idStr.c_str(), NULL);
        }

        // Couldn't launch a separate GUI terminal — run input_mode directly in this child process.
        execlp(argv[0], argv[0], "input_mode", idStr.c_str(), NULL);

        // If we reach here, everything failed; exit child quietly.
        exit(0);
    }

    FollowingVehicle follower(id, initialSpeed, initialPosition);
    try {
        follower.connectToLeader();
        follower.startThreads();
        while (true) std::this_thread::sleep_for(std::chrono::seconds(1));
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}

// ============== LEADER MODE FUNCTIONS ==============

// Transition from follower to leader mode
void FollowingVehicle::transitionToLeader() {
    std::cout << "[TRANSITION] Stopping follower mode...\n";
    
    // Stop follower threads
    clientRunning_ = false;
    
    // Signal event thread to wake up
    pthread_mutex_lock(&eventMutex_);
    pthread_cond_broadcast(&eventCv_);
    pthread_mutex_unlock(&eventMutex_);
    
    // Wait for all follower threads to finish
    std::cout << "[TRANSITION] Waiting for follower threads to finish...\n";
    if (!pthread_equal(pthread_self(), recvThread_)) {
        pthread_join(recvThread_, nullptr);
    } else {
        std::cout << "[TRANSITION] Skipping join of recvThread (self)\n";
    }
    pthread_join(runThread_, nullptr);
    pthread_join(sendStatusThread_, nullptr);
    pthread_join(displayThread_, nullptr);
    pthread_join(eventSenderThread_, nullptr);
    
    std::cout << "[TRANSITION] All follower threads stopped.\n";
    std::cout << "[TRANSITION] Waiting for old leader to release port...\n";
    std::this_thread::sleep_for(std::chrono::seconds(3));
    
    // Update to leader mode
    info_.mode = LeaderMode;
    isLeader_ = true;
    
    // Update platoon state - this vehicle is now the leader
    pthread_mutex_lock(&leaderMutex_);
    platoonState_.leaderId = info_.id;
    
    // Remove old leader from platoon vehicles
    platoonState_.vehicles.erase(
        std::remove_if(
            platoonState_.vehicles.begin(),
            platoonState_.vehicles.end(),
            [this](const VehicleInfo& v){ return v.mode == LeaderMode && v.id != info_.id; }),
        platoonState_.vehicles.end());
    
    // Update this vehicle to leader in platoon state
    for (auto& v : platoonState_.vehicles) {
        if (v.id == info_.id) {
            v.mode = LeaderMode;
            v.ipAddress = htonl(INADDR_LOOPBACK);
            v.port = htons(SERVER_PORT);
        } else {
             // Reset heartbeat to avoid immediate timeout during transition.
             // We add a generous 20-second buffer relative to 'now' to allow
             // the new leader socket to bind and followers to reconnect.
             v.lastHeartbeatMs = nowMs() + 20000;
        }
    }
    pthread_mutex_unlock(&leaderMutex_);
    
    std::cout << "[TRANSITION] Starting leader mode...\\n";
    std::cout << "New Leader ID: " << info_.id << "\\n";
    std::cout << "Position: " << info_.position << " m\\n";
    std::cout << "Speed: " << info_.speed << " m/s\\n";

    // Set original speed to current speed or target speed
    originalSpeed_ = info_.speed;
    setState(NORMAL); // Reset state to NORMAL
    
    // Start leader mode
    startLeaderMode();
}


void FollowingVehicle::startLeaderMode() {
    // Close old follower socket
    if (clientSocket_ >= 0) {
        ::close(clientSocket_);
    }
    
    // Create server socket for leader
    clientSocket_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (clientSocket_ < 0) {
        std::cerr << "Failed to create leader socket: " << strerror(errno) << std::endl;
        return;
    }
    
    int opt = 1;
    setsockopt(clientSocket_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    // setsockopt(clientSocket_, SOL_SOCKET, SO_REUSEPORT, &opt, sizeof(opt)); // DISABLED to prevent Zombie leaders
    
    // Set SO_LINGER to force immediate close
    struct linger sl;
    sl.l_onoff = 1;
    sl.l_linger = 0;
    setsockopt(clientSocket_, SOL_SOCKET, SO_LINGER, &sl, sizeof(sl));
    
    // Bind to leader port
    struct sockaddr_in serverAddr{};
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = INADDR_ANY;
    serverAddr.sin_port = htons(SERVER_PORT);
    
    // Try binding with more retries and longer delays
    int retries = 15;
    std::cout << "[LEADER] Attempting to bind to port " << SERVER_PORT << "...\\n";
    while (retries-- > 0) {
        if (bind(clientSocket_, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) >= 0) {
            std::cout << "\\n[LEADER] ✓ Successfully bound to port " << SERVER_PORT << "\\n";
            break;
        }
        if (retries > 0) {
            std::cout << "[LEADER] Port still in use, waiting... (" << retries << " retries left)\\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(1500));
        }
    }
    
    if (retries < 0) {
        std::cerr << "[LEADER] Failed to bind leader socket after retries: " << strerror(errno) << std::endl;
        return;
    }
    
    // Update info
    info_.ipAddress = htonl(INADDR_LOOPBACK);
    info_.port = htons(SERVER_PORT);
    
    // Start leader threads
    leaderRunning_ = true;
    pthread_create(&leaderRecvThread_, nullptr, leaderRecvThreadEntry, this);
    pthread_create(&leaderRunThread_, nullptr, leaderRunThreadEntry, this);
    pthread_create(&leaderDisplayThread_, nullptr, leaderDisplayThreadEntry, this);
    pthread_create(&leaderStatusThread_, nullptr, leaderStatusThreadEntry, this);
    pthread_create(&leaderHeartbeatThread_, nullptr, leaderHeartbeatThreadEntry, this);
    
    // Restart event thread to continue reading from FIFO
    // Note: The FIFO is already open by the child process, so we just need to keep reading
    clientRunning_ = true;  // Re-enable for event thread
    pthread_create(&eventThread_, nullptr, eventSimulationThreadEntry, this);
    pthread_create(&eventSenderThread_, nullptr, leaderEventSenderThreadEntry, this);
    
    std::cout << "[LEADER] All leader threads started (including event handling)\\n";
}

void FollowingVehicle::sendPlatoonStateAsLeader() {
    PlatoonStateMessage psMsg{};
    psMsg.type = MessageType::PLATOON_STATE;
    psMsg.timestamp = nowMs();
    
    pthread_mutex_lock(&leaderMutex_);
    psMsg.leaderId = platoonState_.leaderId;
    
    // Sort vehicles by position descending
    std::vector<VehicleInfo> sorted = platoonState_.vehicles;
    std::sort(sorted.begin(), sorted.end(),
              [](const VehicleInfo& a, const VehicleInfo& b) { return a.position > b.position; });
    
    psMsg.vehicleCount = std::min(static_cast<int>(sorted.size()), MAX_PLATOON_VEHICLES);
    for (int i = 0; i < psMsg.vehicleCount; ++i) {
        psMsg.vehicles[i] = sorted[i];
    }
    
    // DEBUG: Print what we are sending
    if (psMsg.vehicleCount > 1) {
         // std::cerr << "[DEBUG_SEND] Sending PLATOON_STATE (" << psMsg.vehicleCount << " vehicles):" << std::endl;
         for (int i=0; i<psMsg.vehicleCount; ++i) {
             // std::cerr << "  [" << i << "] ID=" << psMsg.vehicles[i].id 
             //           << " Mode=" << (int)psMsg.vehicles[i].mode 
             //           << " Pos=" << psMsg.vehicles[i].position << std::endl;
             if (psMsg.vehicles[i].id < 0 && psMsg.vehicles[i].id != -1) {
                 std::cerr << "[CRITICAL_WARNING] Sending CORRUPT ID " << psMsg.vehicles[i].id << " at index " << i << std::endl;
             }
         }
    }

        // Send to each follower
    for (const auto& v : platoonState_.vehicles) {
        if (v.id == info_.id) continue;
        if (v.ipAddress == 0 || v.port == 0) continue;
        
        struct sockaddr_in dest{};
        dest.sin_family = AF_INET;
        dest.sin_addr.s_addr = v.ipAddress;
        dest.sin_port = v.port;
        // DEBUG:
        // std::cout << "[LEADER_SEND] Sending PLATOON_STATE to " << v.id << " (" << ntohs(v.port) << ")\n";
        
        sendto(clientSocket_, &psMsg, sizeof(psMsg), 0,
               (struct sockaddr*)&dest, sizeof(dest));
    }
    pthread_mutex_unlock(&leaderMutex_);
}

// Leader receive thread
void* FollowingVehicle::leaderRecvThreadEntry(void* arg) {
    FollowingVehicle* leader = static_cast<FollowingVehicle*>(arg);
    const size_t BUFFER_SIZE = 4096;
    char* buffer = new char[BUFFER_SIZE];
    
    std::cout << "[LEADER_RECV] Thread started on socket " << leader->clientSocket_ << std::endl;

    while (leader->leaderRunning_) {
        struct sockaddr_in senderAddr{};
        socklen_t addrLen = sizeof(senderAddr);
        ssize_t recvLen = recvfrom(leader->clientSocket_, buffer, BUFFER_SIZE, 0,
                                   (struct sockaddr*)&senderAddr, &addrLen);
        if (recvLen < 0) {
             // Only print if not EAGAIN/EINTR
             if (errno != EAGAIN && errno != EINTR) {
                 std::cerr << "[LEADER_RECV] Error: " << strerror(errno) << std::endl;
             }
             continue;
        }

        if (recvLen >= 1) {
            MessageType msgType = static_cast<MessageType>(buffer[0]);
            
            switch (msgType) {
                case STATUS_UPDATE: {
                    if (static_cast<size_t>(recvLen) < sizeof(StatusUpdateMessage)) break;
                    StatusUpdateMessage status;
                    std::memcpy(&status, buffer, sizeof(status));
                    
                    if (status.info.id <= 0) {
                        std::cerr << "[LEADER_RECV] Ignored STATUS_UPDATE with invalid ID " << status.info.id << "\n";
                        break;
                    }

                    pthread_mutex_lock(&leader->leaderMutex_);
                    auto it = std::find_if(
                        leader->platoonState_.vehicles.begin(),
                        leader->platoonState_.vehicles.end(),
                        [&](const VehicleInfo& v){ return v.id == status.info.id; });
                    if (it != leader->platoonState_.vehicles.end()) {
                        it->position = status.info.position;
                        it->speed = status.info.speed;
                        it->ipAddress = senderAddr.sin_addr.s_addr;
                        it->port = senderAddr.sin_port;
                        it->lastHeartbeatMs = nowMs();
                    } else {
                        // Add new follower
                        VehicleInfo fi = status.info;
                        fi.ipAddress = senderAddr.sin_addr.s_addr;
                        fi.port = senderAddr.sin_port;
                        fi.lastHeartbeatMs = nowMs();
                        leader->platoonState_.vehicles.push_back(fi);
                        std::cout << "[LEADER] Added new vehicle " << fi.id << " to platoon\n";
                    }
                    pthread_mutex_unlock(&leader->leaderMutex_);
                    break;
                }
                case COUPLE_COMMAND: {
                    if (static_cast<size_t>(recvLen) < sizeof(CoupleCommandMessage)) break;
                    CoupleCommandMessage cmd;
                    std::memcpy(&cmd, buffer, sizeof(cmd));
                    
                    if (cmd.info.id <= 0) {
                         std::cerr << "[LEADER_RECV] Ignored COUPLE_COMMAND with invalid ID " << cmd.info.id << "\n";
                         break;
                    }
                    
                    std::cout << "[LEADER] Received " << (cmd.couple ? "COUPLE" : "DECOUPLE") 
                              << " from " << cmd.info.id << std::endl;
                    
                    pthread_mutex_lock(&leader->leaderMutex_);
                    if (cmd.couple) {
                         auto it = std::find_if(
                            leader->platoonState_.vehicles.begin(),
                            leader->platoonState_.vehicles.end(),
                            [&](const VehicleInfo& v){ return v.id == cmd.info.id; });
                        if (it == leader->platoonState_.vehicles.end()) {
                            VehicleInfo fi = cmd.info;
                            fi.ipAddress = senderAddr.sin_addr.s_addr;
                            fi.port = senderAddr.sin_port;
                            fi.lastHeartbeatMs = nowMs();
                            leader->platoonState_.vehicles.push_back(fi);
                            std::cout << "[LEADER] Added new vehicle " << fi.id << " to platoon" << std::endl;
                        } else {
                            it->position = cmd.info.position;
                            it->speed = cmd.info.speed;
                            it->ipAddress = senderAddr.sin_addr.s_addr;
                            it->port = senderAddr.sin_port;
                            it->lastHeartbeatMs = nowMs();
                        }
                    } else {
                        leader->platoonState_.vehicles.erase(
                            std::remove_if(
                                leader->platoonState_.vehicles.begin(),
                                leader->platoonState_.vehicles.end(),
                                [&](const VehicleInfo& v){ return v.id == cmd.info.id; }),
                            leader->platoonState_.vehicles.end());
                    }
                    pthread_mutex_unlock(&leader->leaderMutex_);
                    leader->sendPlatoonStateAsLeader();
                    break;
                }
                case TRAFFIC_LIGHT_ALERT: {
                    if (static_cast<size_t>(recvLen) < sizeof(TrafficLightMessage)) break;
                    TrafficLightMessage tlMsg;
                    std::memcpy(&tlMsg, buffer, sizeof(tlMsg));
                    TrafficLightStatus alert = static_cast<TrafficLightStatus>(tlMsg.status);
                     if (alert == LIGHT_RED) {
                        leader->setState(STOPPING);
                    } else if (alert == LIGHT_GREEN) {
                        leader->setState(STARTING);
                    }
                    break;
                }
                case ENERGY_DEPLETION_ALERT: {
                    if (static_cast<size_t>(recvLen) < sizeof(EnergyDepletionMessage)) break;
                    std::cout << "[LEADER] Low energy alert. Reducing speed.\n";
                    leader->setState(LOW_ENERGY);
                    leader->targetSpeed_ = leader->originalSpeed_ * 0.6;
                    break;
                }
                 case ENERGY_RESTORED: {
                    std::cout << "[LEADER] Energy restored. Stopping at gas station.\n";
                    leader->setState(STOPPING);
                    leader->gasStationStop_ = true;
                    break;
                }
                case GAS_STATION_ALERT: {
                     std::cout << "[LEADER] Gas station alert. Stopping.\n";
                     leader->setState(STOPPING);
                     leader->gasStationStop_ = false;
                     break;
                }
                default: break;
            }
        }
    }
    delete[] buffer;
    return nullptr;
}

// Leader run thread
void* FollowingVehicle::leaderRunThreadEntry(void* arg) {
    FollowingVehicle* leader = static_cast<FollowingVehicle*>(arg);
    const int TIMESTEP_MS = 100;
    const double dt = TIMESTEP_MS / 1000.0;
    const double accel = 8;
    const double decel = 12;

    while (leader->leaderRunning_) {
        // Simple state machine for promoted leader
        FollowerState st = leader->getState();
        double target = (st == NORMAL) ? leader->originalSpeed_ : leader->info_.speed;
        
        switch (st) {
             case NORMAL:
                if (leader->info_.speed < target) {
                    leader->info_.speed = std::min(target, leader->info_.speed + accel * dt);
                } else if (leader->info_.speed > target) {
                    leader->info_.speed = std::max(target, leader->info_.speed - decel * dt);
                }
                break;
            case STOPPING:
                leader->info_.speed = std::max(0.0, leader->info_.speed - decel * dt * 1.2);
                if (leader->info_.speed <= 0.001) {
                    leader->info_.speed = 0.0;
                    leader->setState(STOPPED);
                    leader->stopTimeMs_ = nowMs();
                }
                break;
            case STOPPED:
                leader->info_.speed = 0.0;
                if (leader->gasStationStop_ && (nowMs() - leader->stopTimeMs_ > 3000)) {
                     std::cout << "[LEADER] Energy restored. Resuming.\n";
                     leader->gasStationStop_ = false;
                     leader->setState(STARTING);
                }
                break;
            case STARTING:
                leader->info_.speed = std::min(target, leader->info_.speed + accel * dt);
                if (leader->info_.speed >= target - 1e-6) {
                    leader->setState(NORMAL);
                }
                break;
            case LOW_ENERGY:
                 target = leader->targetSpeed_;
                 if (leader->info_.speed > target) {
                    leader->info_.speed = std::max(target, leader->info_.speed - decel * dt);
                } else if (leader->info_.speed < target) {
                    leader->info_.speed = std::min(target, leader->info_.speed + accel * dt);
                }
                break;
            default: break;
        }

        leader->info_.position += leader->info_.speed * dt;
        
        pthread_mutex_lock(&leader->leaderMutex_);
        for (auto& v : leader->platoonState_.vehicles) {
            if (v.id == leader->info_.id) {
                v.position = leader->info_.position;
                v.speed = leader->info_.speed;
                v.mode = LeaderMode; // Ensure mode is correct
                break;
            }
        }
        pthread_mutex_unlock(&leader->leaderMutex_);
        
        std::this_thread::sleep_for(std::chrono::milliseconds(TIMESTEP_MS));
    }
    return nullptr;
}


// Leader display thread
void* FollowingVehicle::leaderDisplayThreadEntry(void* arg) {
    FollowingVehicle* leader = static_cast<FollowingVehicle*>(arg);
    
    while (leader->leaderRunning_) {
        pthread_mutex_lock(&leader->leaderMutex_);
         std::string stateStr;
        switch (leader->getState()) {
            case NORMAL: stateStr = "NORMAL"; break;
            case STOPPING: stateStr = "STOPPING"; break;
            case STOPPED: stateStr = "STOPPED"; break;
            case STARTING: stateStr = "STARTING"; break;
            case LOW_ENERGY: stateStr = "LOW_ENERGY"; break;
            default: stateStr = "UNKNOWN"; break;
        }
        
        std::cout << "\033[2J\033[H"; // Clear screen
        std::cout << "\n\033[1;36m================== PROMOTED LEADER (ID=" << leader->info_.id << ") ==================\033[0m\n";
        std::cout << "Position: " << leader->info_.position
                  << "  Speed: " << leader->info_.speed 
                  << "  State: " << stateStr << "\n";
        std::cout << "Platoon vehicles:\n";
        
        auto vehicles = leader->platoonState_.vehicles;
        std::sort(vehicles.begin(), vehicles.end(), [](const VehicleInfo& a, const VehicleInfo& b){
            return a.position > b.position;
        });
        
        for (const auto& v : vehicles) {
            std::cout << "  Vehicle ID: " << v.id
                      << ", Pos: " << v.position
                      << ", Spd: " << v.speed
                      << ", Mode: " << (v.mode == LeaderMode ? "Leader" : "Follower") << "\n";
        }
        std::cout << std::flush;
        pthread_mutex_unlock(&leader->leaderMutex_);
        
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
    return nullptr;
}

// Leader status thread
void* FollowingVehicle::leaderStatusThreadEntry(void* arg) {
    FollowingVehicle* leader = static_cast<FollowingVehicle*>(arg);
    while (leader->leaderRunning_) {
        StatusUpdateMessage status{};
        status.type = MessageType::STATUS_UPDATE;
        status.info = leader->info_;
        status.timestamp = nowMs();

        pthread_mutex_lock(&leader->leaderMutex_);
        for (const auto& v : leader->platoonState_.vehicles) {
            if (v.id == leader->info_.id) continue;
            if (v.ipAddress == 0 || v.port == 0) continue;
            
            struct sockaddr_in dest{};
            dest.sin_family = AF_INET;
            dest.sin_addr.s_addr = v.ipAddress;
            dest.sin_port = v.port;
            sendto(leader->clientSocket_, &status, sizeof(status), 0,
                   (const struct sockaddr*)&dest, sizeof(dest));
        }
        pthread_mutex_unlock(&leader->leaderMutex_);
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return nullptr;
}

// Leader heartbeat thread
void* FollowingVehicle::leaderHeartbeatThreadEntry(void* arg) {
    FollowingVehicle* leader = static_cast<FollowingVehicle*>(arg);
    const int TIMEOUT_MS = 5000;
    
    while (leader->leaderRunning_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        std::int64_t now = nowMs();
        std::vector<int> toRemove;
        
        pthread_mutex_lock(&leader->leaderMutex_);
        for (auto it = leader->platoonState_.vehicles.begin(); it != leader->platoonState_.vehicles.end(); ) {
            if (it->id == leader->info_.id) { ++it; continue; }
            
            std::int64_t age = (it->lastHeartbeatMs == 0) ? (TIMEOUT_MS + 1) : (now - it->lastHeartbeatMs);
            if (age > TIMEOUT_MS) {
                std::cerr << "[LEADER] Follower " << it->id << " TIMED OUT (Age: " << age << "ms). Removing.\n";
                toRemove.push_back(it->id);
                
                RemoveVehicleMessage removeMsg{};
                removeMsg.type = MessageType::REMOVE_VEHICLE;
                removeMsg.vehicleId = it->id;
                removeMsg.timestamp = nowMs();
                
                struct sockaddr_in dest{};
                dest.sin_family = AF_INET;
                dest.sin_addr.s_addr = it->ipAddress;
                dest.sin_port = it->port;
                sendto(leader->clientSocket_, &removeMsg, sizeof(removeMsg), 0,
                       (const struct sockaddr*)&dest, sizeof(dest));
                       
                it = leader->platoonState_.vehicles.erase(it);
            } else {
                ++it;
            }
        }
        pthread_mutex_unlock(&leader->leaderMutex_);
        
        if (!toRemove.empty()) {
            leader->sendPlatoonStateAsLeader();
        }
    }
    return nullptr;
}

// Leader event sender thread (handles traffic light events)
void* FollowingVehicle::leaderEventSenderThreadEntry(void* arg) {
    FollowingVehicle* leader = static_cast<FollowingVehicle*>(arg);
    while (leader->leaderRunning_) {
        pthread_mutex_lock(&leader->eventMutex_);
        while (leader->eventQueue_.empty() && leader->leaderRunning_) {
            pthread_cond_wait(&leader->eventCv_, &leader->eventMutex_);
        }
        if (!leader->leaderRunning_) {
            pthread_mutex_unlock(&leader->eventMutex_);
            break;
        }
        int eventCode = leader->eventQueue_.front();
        leader->eventQueue_.pop();
        pthread_mutex_unlock(&leader->eventMutex_);
        
        // Handle Leader Event Codes (1-7)
        EventMessage eventMsg{};
        eventMsg.timestamp = nowMs();
        bool broadcast = false;
        
        switch (eventCode) {
            case 1: // Obstacle
                leader->force_obstacle_ = !leader->force_obstacle_;
                eventMsg.type = MessageType::OBSTACLE_DETECTED_ALERT;
                eventMsg.eventData = (void*)(uintptr_t)leader->force_obstacle_; 
                broadcast = true;
                break;
            case 2: // Red
                eventMsg.type = MessageType::TRAFFIC_LIGHT_ALERT;
                eventMsg.eventData = (void*)(uintptr_t)LIGHT_RED;
                leader->setState(STOPPING);
                broadcast = true;
                break;
            case 3: // Green
                eventMsg.type = MessageType::TRAFFIC_LIGHT_ALERT;
                eventMsg.eventData = (void*)(uintptr_t)LIGHT_GREEN;
                leader->setState(STARTING);
                broadcast = true;
                break;
            case 4: // Normal
                leader->setState(NORMAL);
                leader->targetSpeed_ = leader->originalSpeed_;
                break;
            case 5: // Low energy
                leader->targetSpeed_ = leader->originalSpeed_ * 0.6;
                leader->setState(LOW_ENERGY);
                break;
             case 6: // Restore energy
                leader->gasStationStop_ = true;
                leader->setState(STOPPING);
                eventMsg.type = MessageType::GAS_STATION_ALERT;
                broadcast = true;
                break;
             case 7: { // LEAVE
                std::cout << "[LEADER] Leaving Platoon...\n";
                 pthread_mutex_lock(&leader->leaderMutex_);
                 auto vehicles = leader->platoonState_.vehicles;
                 std::sort(vehicles.begin(), vehicles.end(), [](const VehicleInfo& a, const VehicleInfo& b){
                     return a.position > b.position;
                 });
                 int myIndex = -1;
                 for(int i=0; i<(int)vehicles.size(); ++i) { if(vehicles[i].id == leader->info_.id) { myIndex=i; break; } }
                 
                 if (myIndex != -1 && (myIndex + 1) < (int)vehicles.size()) {
                     VehicleInfo rear = vehicles[myIndex+1];
                     LeavePlatoonMessage leaveMsg{};
                     leaveMsg.type = MessageType::LEAVE_PLATOON;
                     leaveMsg.vehicleId = leader->info_.id;
                     leaveMsg.timestamp = nowMs();
                     
                    struct sockaddr_in dest{};
                    dest.sin_family = AF_INET;
                    dest.sin_addr.s_addr = rear.ipAddress;
                    dest.sin_port = rear.port;
                    sendto(leader->clientSocket_, &leaveMsg, sizeof(leaveMsg), 0,
                           (const struct sockaddr*)&dest, sizeof(dest));
                    
                    std::cout << "[LEADER] Sent LEAVE command to next leader " << rear.id << "\n";
                 }
                 pthread_mutex_unlock(&leader->leaderMutex_);
                 std::this_thread::sleep_for(std::chrono::milliseconds(500));
                 leader->leaderRunning_ = false; 
                 // We don't exit process, just stop threads. Ideally we should clean up but this matches basic request.
                 break;
             }
            default: break;
        }
        
        if (broadcast && leader->leaderRunning_) {
            pthread_mutex_lock(&leader->leaderMutex_);
            auto vehicles = leader->platoonState_.vehicles;
             std::sort(vehicles.begin(), vehicles.end(), [](const VehicleInfo& a, const VehicleInfo& b){
                 return a.position > b.position;
             });
             int myIndex = -1;
             for(int i=0; i<(int)vehicles.size(); ++i) { if(vehicles[i].id == leader->info_.id) { myIndex=i; break; } }
             
             if (myIndex != -1 && (myIndex + 1) < (int)vehicles.size()) {
                  VehicleInfo rear = vehicles[myIndex+1];
                  struct sockaddr_in dest{};
                    dest.sin_family = AF_INET;
                    dest.sin_addr.s_addr = rear.ipAddress;
                    dest.sin_port = rear.port;
                    
                    if (eventMsg.type == MessageType::TRAFFIC_LIGHT_ALERT) {
                        TrafficLightMessage msg{};
                        msg.type = MessageType::TRAFFIC_LIGHT_ALERT;
                        msg.status = (TrafficLightStatus)(uintptr_t)eventMsg.eventData; 
                        sendto(leader->clientSocket_, &msg, sizeof(msg), 0, (struct sockaddr*)&dest, sizeof(dest));
                    } else if (eventMsg.type == MessageType::OBSTACLE_DETECTED_ALERT) {
                        ObstacleMessage msg{};
                        msg.type = MessageType::OBSTACLE_DETECTED_ALERT;
                        msg.obstacleDetected = (bool)(uintptr_t)eventMsg.eventData;
                        sendto(leader->clientSocket_, &msg, sizeof(msg), 0, (struct sockaddr*)&dest, sizeof(dest));
                    } else if (eventMsg.type == MessageType::GAS_STATION_ALERT) {
                         GasStationMessage msg{};
                         msg.type = MessageType::GAS_STATION_ALERT;
                         msg.vehicleId = leader->info_.id;
                         sendto(leader->clientSocket_, &msg, sizeof(msg), 0, (struct sockaddr*)&dest, sizeof(dest));
                    }
             }
             pthread_mutex_unlock(&leader->leaderMutex_);
        }
    }
    return nullptr;
}