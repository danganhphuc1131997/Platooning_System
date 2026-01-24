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

// Constructor
FollowingVehicle::FollowingVehicle(int id,
                                 double initialSpeed,
                                 double initialPosition)
    : state_(NORMAL), clientRunning_(false), clientSocket_(-1) {
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
void FollowingVehicle::sendCoupleCommandToLeader() {
    CoupleCommandMessage cmd{};
    cmd.type = MessageType::COUPLE_COMMAND;
    cmd.info.id = info_.id;
    cmd.info.position = info_.position;
    cmd.info.speed = info_.speed;
    cmd.info.mode = info_.mode;
    cmd.info.ipAddress = info_.ipAddress; // include our address so leader knows where to send
    cmd.info.port = info_.port;
    cmd.couple = true;
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
    char buffer[1024];

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
                    switch (alert) {
                        case LIGHT_RED:
                            follower->setState(STOPPING_FOR_RED_LIGHT);
                            break;
                        case LIGHT_GREEN:
                            follower->setState(STARTING);
                            break;
                        default:
                            break;
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

        // Determine which vehicle to follow:
        // - If front vehicle is unknown (id == -1) or IS the leader, follow leader
        // - Otherwise follow the front vehicle
        double refPos, refSp;
        if (frontId == -1 || frontMode == LeaderMode) {
            refPos = leaderPos;
            refSp = leaderSp;
        } else {
            refPos = frontPos;
            refSp = frontSp;
        }

        double gap = refPos - follower->info_.position;

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

int main(int argc, char* argv[]) {
    int id = FOLLOWER_INITIAL_ID;
    double initialSpeed = FOLLOWER_INITIAL_SPEED;
    double initialPosition = FOLLOWER_INITIAL_POSITION;
    if (argc >= 2) id = std::stoi(argv[1]);
    if (argc >= 3) initialSpeed = std::stod(argv[2]);
    if (argc >= 4) initialPosition = std::stod(argv[3]);

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

