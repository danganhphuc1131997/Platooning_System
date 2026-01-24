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

    memset(&leaderAddr_, 0, sizeof(leaderAddr_));
    leaderAddr_.sin_family = AF_INET;
    leaderAddr_.sin_port = htons(SERVER_PORT);
    leaderAddr_.sin_addr.s_addr = htonl(INADDR_LOOPBACK); // assume leader on localhost

    if (connect(clientSocket_, (struct sockaddr*)&leaderAddr_, sizeof(leaderAddr_)) < 0) {
        ::close(clientSocket_);
        clientSocket_ = -1;
        throw std::runtime_error(std::string("Failed to connect to leader: ") + strerror(errno));
    }
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
    cmd.couple = true;
    cmd.timestamp = nowMs();

    ssize_t sentBytes = send(clientSocket_, &cmd, sizeof(cmd), 0);
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

// Receive messages (connected UDP)
void* FollowingVehicle::recvThreadEntry(void* arg) {
    FollowingVehicle* follower = static_cast<FollowingVehicle*>(arg);
    char buffer[1024];

    while (follower->clientRunning_) {
        ssize_t recvLen = recv(follower->clientSocket_, buffer, sizeof(buffer), 0);
        if (recvLen >= 1) {
            MessageType msgType = static_cast<MessageType>(buffer[0]);
            switch (msgType) {
                case STATUS_UPDATE: {
                    if (static_cast<size_t>(recvLen) < sizeof(StatusUpdateMessage)) break;
                    StatusUpdateMessage status;
                    std::memcpy(&status, buffer, sizeof(status));
                    if (status.info.mode == LeaderMode) {
                        pthread_mutex_lock(&follower->leaderMutex_);
                        follower->leaderPosition_ = status.info.position;
                        follower->leaderSpeed_ = status.info.speed;
                        pthread_mutex_unlock(&follower->leaderMutex_);
                    }
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

        // Read leader snapshot (thread-safe)
        double leaderPos = 0.0;
        double leaderSp = 0.0;
        pthread_mutex_lock(&follower->leaderMutex_);
        leaderPos = follower->leaderPosition_;
        leaderSp = follower->leaderSpeed_;
        pthread_mutex_unlock(&follower->leaderMutex_);
        double gap = leaderPos - follower->info_.position;

        // Simple state machine for follower behavior
        FollowerState currentState = follower->getState();
        switch (currentState)
        {
            case NORMAL: 
                if (gap < SAFE_DISTANCE) {
                    follower->setState(STOPPING);
                } else if (gap > (SAFE_DISTANCE + 20.0)) {
                    follower->setState(CATCHING_UP);
                } else if ( (gap >= SAFE_DISTANCE) && (gap <= (SAFE_DISTANCE + 20.0)) ) {
                    // Maintain speed
                    // if (follower->info_.speed < targetSpeed) {
                    //     follower->info_.speed = std::round(std::min(targetSpeed, follower->info_.speed + accel * dt));
                    // } else if (follower->info_.speed > targetSpeed) {
                    //     follower->info_.speed = std::round(std::max(targetSpeed, follower->info_.speed - decel * dt));
                    // }
                    follower->info_.speed = leaderSp;
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
                if ((gap >= SAFE_DISTANCE) && (gap <= (SAFE_DISTANCE + 20.0)) ) {
                    follower->setState(NORMAL);
                } else if (gap > (SAFE_DISTANCE + 100.0)) {
                    // Speed up to catch leader (rounded to integer)
                    follower->info_.speed = std::round(std::min(leaderSp + 30.0, follower->info_.speed + accel * dt));
                } else if ((gap <= (SAFE_DISTANCE + 100.0)) && (gap > (SAFE_DISTANCE + 20.0))) {
                    // Speed up to catch leader (rounded to integer)
                    follower->info_.speed = std::round(std::min(leaderSp + 5.0, follower->info_.speed + accel * dt));
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
                // if (gap < (SAFE_DISTANCE + 25.0)) {
                //     follower->setState(STARTING);
                // }
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

        ssize_t sentBytes = send(follower->clientSocket_, &status, sizeof(status), 0);
        if (sentBytes < 0) {
            std::cerr << "Failed to send status update: " << strerror(errno) << std::endl;
        }
    }
    return nullptr;
}

void* FollowingVehicle::displayThreadEntry(void* arg) {
    FollowingVehicle* follower = static_cast<FollowingVehicle*>(arg);
    while (follower->clientRunning_) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        double leaderPos = 0.0;
        double leaderSp = 0.0;
        pthread_mutex_lock(&follower->leaderMutex_);
        leaderPos = follower->leaderPosition_;
        leaderSp = follower->leaderSpeed_;
        pthread_mutex_unlock(&follower->leaderMutex_);

        double gap = leaderPos - follower->info_.position;
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
        std::cout << "Follower id=" << follower->info_.id
                  << " pos=" << follower->info_.position
                  << " speed=" << follower->info_.speed
                  << " state=" << stateStr
                  << " gap=" << gap << std::endl;
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

