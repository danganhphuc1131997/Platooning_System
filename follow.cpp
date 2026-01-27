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

// Constructor
FollowingVehicle::FollowingVehicle(int id,
                                 double initialSpeed,
                                 double initialPosition)
    : state_(NORMAL), clientRunning_(false), energyAlertSent_(false), clientSocket_(-1) {
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
                  << " speed=" << cmd.info.speed << ")\n";
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

    while (follower->clientRunning_) {
        struct sockaddr_in senderAddr{};
        socklen_t addrLen = sizeof(senderAddr);
        ssize_t recvLen = recvfrom(follower->clientSocket_, buffer, sizeof(buffer), 0,
                                   (struct sockaddr*)&senderAddr, &addrLen);
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

                    // Find my index in the platoon list (sorted by position descending: leader first)
                    int myIndex = -1;
                    for (int i = 0; i < psMsg.vehicleCount && i < MAX_PLATOON_VEHICLES; ++i) {
                        if (psMsg.vehicles[i].id == follower->info_.id) {
                            myIndex = i;
                            break;
                        }
                    }

                    pthread_mutex_lock(&follower->leaderMutex_);
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
                    pthread_mutex_unlock(&follower->leaderMutex_);
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
        pthread_mutex_lock(&follower->leaderMutex_);
        light = follower->trafficLight_;
        delayedUntil = follower->delayedUntilMs_;
        isDecoupled = follower->decoupled_;
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
        if (!isDecoupled && light == LIGHT_GREEN) {
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
                        follower->setState(STOPPED);
                    }
                }
                break;
            case STOPPING_FOR_RED_LIGHT:
                if (follower->info_.speed > 0.0) {
                    follower->info_.speed = std::round(std::max(0.0, follower->info_.speed - decel * dt));
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

        StatusUpdateMessage status{};
        status.type = MessageType::STATUS_UPDATE;
        // Copy current info (no deep locks needed for simple struct copy)
        status.info = follower->info_;
        status.timestamp = nowMs();

        // Send to leader (using sendto since socket is not connected)
        ssize_t sentBytes = sendto(follower->clientSocket_, &status, sizeof(status), 0,
                                   (const struct sockaddr*)&follower->leaderAddr_, sizeof(follower->leaderAddr_));
        if (sentBytes < 0) {
            std::cerr << "Failed to send status to leader: " << strerror(errno) << std::endl;
        }

        // Send energy alert if needed
        if (!follower->energyAlertSent_ && follower->getState() == LOW_ENERGY && follower->info_.speed <= follower->targetSpeed_ + 0.1) {
            EnergyDepletionMessage energyMsg{};
            energyMsg.type = MessageType::ENERGY_DEPLETION_ALERT;
            energyMsg.vehicleId = follower->info_.id;
            energyMsg.timestamp = nowMs();

            ssize_t energySent = sendto(follower->clientSocket_, &energyMsg, sizeof(energyMsg), 0,
                                        (const struct sockaddr*)&follower->leaderAddr_, sizeof(follower->leaderAddr_));
            if (energySent < 0) {
                std::cerr << "Failed to send energy depletion alert: " << strerror(errno) << std::endl;
            } else {
                std::cout << "Sent energy low alert (id=" << energyMsg.vehicleId << ", energy=" << follower->info_.energy << ")\n";
                follower->energyAlertSent_ = true;
            }
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
            case DECOUPLED: stateStr = "DECOUPLED"; break;
            case LOW_ENERGY: stateStr = "LOW_ENERGY"; break;
            default: stateStr = "UNKNOWN"; break;
        }
        std::cout << "\033[2J\033[H"; // clear screen + home
        std::cout << "============ FOLLOWER " << follower->info_.id << " ============\n";
        std::cout << "Position: " << follower->info_.position
                  << "  Speed: " << follower->info_.speed
                  << "  State: " << stateStr << "\n";
        std::cout << "Gap to front: " << gap << " m\n";
        std::cout << "Front vehicle: " << (frontId == -1 ? "Leader" : ("ID " + std::to_string(frontId)))
                  << " (pos=" << refPos << ", spd=" << (frontId == -1 || frontMode == LeaderMode ? leaderSp : frontSp) << ")\n";
        std::cout << "Rear vehicle: " << (rearId == -1 ? "None" : ("ID " + std::to_string(rearId))) << "\n";
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

        // Build traffic light message (similar to leader)
        TrafficLightMessage tlMsg{};
        tlMsg.type = MessageType::TRAFFIC_LIGHT_ALERT;
        tlMsg.timestamp = nowMs();

        switch (eventCode) {
            case 1: // Red light
                tlMsg.status = LIGHT_RED;
                follower->setState(STOPPING_FOR_RED_LIGHT);
                break;
            case 2: // Green light
                tlMsg.status = LIGHT_GREEN;
                follower->setState(STARTING);
                break;
            case 3: // Run out of energy
                // Set energy to 0, reduce speed to 60%, send alert after reaching target
                follower->info_.energy = 0.0;
                follower->targetSpeed_ = follower->info_.speed * 0.6;
                follower->setState(LOW_ENERGY);
                follower->energyAlertSent_ = false; // Allow sending alert in status thread
                // Skip sending tlMsg for energy event
                break;
            default:
                continue; // Skip unknown events
        }

        // Get front and rear vehicle info
        pthread_mutex_lock(&follower->leaderMutex_);
        VehicleInfo front = follower->frontVehicleInfo_;
        VehicleInfo rear = follower->rearVehicleInfo_;
        pthread_mutex_unlock(&follower->leaderMutex_);

        // Send to front vehicle
        if (front.id != -1 && front.ipAddress != 0 && front.port != 0) {
            struct sockaddr_in frontAddr{};
            frontAddr.sin_family = AF_INET;
            frontAddr.sin_addr.s_addr = front.ipAddress;
            frontAddr.sin_port = front.port;
            sendto(follower->clientSocket_, &tlMsg, sizeof(tlMsg), 0,
                   (const struct sockaddr*)&frontAddr, sizeof(frontAddr));
        }

        // Send to rear vehicle
        if (rear.id != -1 && rear.ipAddress != 0 && rear.port != 0) {
            struct sockaddr_in rearAddr{};
            rearAddr.sin_family = AF_INET;
            rearAddr.sin_addr.s_addr = rear.ipAddress;
            rearAddr.sin_port = rear.port;
            sendto(follower->clientSocket_, &tlMsg, sizeof(tlMsg), 0,
                   (const struct sockaddr*)&rearAddr, sizeof(rearAddr));
        }

        std::cout << "[FOLLOWER " << follower->info_.id << "] Sent traffic light event to neighbors" << std::endl;
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

