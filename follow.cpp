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
    : state_(NORMAL), clientRunning_(false), clientSocket_(-1), 
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
                    
                    // Process the alert
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
                        
                        // Transition to leader mode within same process
                        follower->transitionToLeader();
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
                        // Leave command from front vehicle, just update front vehicle info
                        pthread_mutex_lock(&follower->leaderMutex_);
                        auto it = std::find_if(
                            follower->platoonState_.vehicles.begin(),
                            follower->platoonState_.vehicles.end(),
                            [&](const VehicleInfo& v){ return v.id == leaveMsg.vehicleId; });
                        if (it != follower->platoonState_.vehicles.end() && it != follower->platoonState_.vehicles.begin()) {
                            follower->frontVehicleInfo_ = *(it - 1);
                        } else {
                            follower->frontVehicleInfo_.id = -1; // no front vehicle    
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
            std::cout << "0: Exit\n";
            std::cout << "Enter choice: ";
            std::cin >> choice;

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
    pthread_join(recvThread_, nullptr);
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
            break;
        }
    }
    pthread_mutex_unlock(&leaderMutex_);
    
    std::cout << "[TRANSITION] Starting leader mode...\\n";
    std::cout << "New Leader ID: " << info_.id << "\\n";
    std::cout << "Position: " << info_.position << " m\\n";
    std::cout << "Speed: " << info_.speed << " m/s\\n";
    
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
    setsockopt(clientSocket_, SOL_SOCKET, SO_REUSEPORT, &opt, sizeof(opt));
    
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
    
    // Send to each follower
    for (const auto& v : platoonState_.vehicles) {
        if (v.id == info_.id) continue;
        if (v.ipAddress == 0 || v.port == 0) continue;
        
        struct sockaddr_in dest{};
        dest.sin_family = AF_INET;
        dest.sin_addr.s_addr = v.ipAddress;
        dest.sin_port = v.port;
        
        sendto(clientSocket_, &psMsg, sizeof(psMsg), 0,
               (struct sockaddr*)&dest, sizeof(dest));
    }
    pthread_mutex_unlock(&leaderMutex_);
}

// Leader receive thread
void* FollowingVehicle::leaderRecvThreadEntry(void* arg) {
    FollowingVehicle* leader = static_cast<FollowingVehicle*>(arg);
    char buffer[1024];
    
    while (leader->leaderRunning_) {
        struct sockaddr_in followerAddr{};
        socklen_t addrLen = sizeof(followerAddr);
        ssize_t recvLen = recvfrom(leader->clientSocket_, buffer, sizeof(buffer), 0,
                                   (struct sockaddr*)&followerAddr, &addrLen);
        
        if (recvLen >= 1) {
            MessageType msgType = static_cast<MessageType>(buffer[0]);
            
            if (msgType == STATUS_UPDATE) {
                if (static_cast<size_t>(recvLen) < sizeof(StatusUpdateMessage)) continue;
                
                StatusUpdateMessage status;
                std::memcpy(&status, buffer, sizeof(status));
                
                pthread_mutex_lock(&leader->leaderMutex_);
                auto it = std::find_if(
                    leader->platoonState_.vehicles.begin(),
                    leader->platoonState_.vehicles.end(),
                    [&](const VehicleInfo& v){ return v.id == status.info.id; });
                
                if (it != leader->platoonState_.vehicles.end()) {
                    it->position = status.info.position;
                    it->speed = status.info.speed;
                    it->ipAddress = followerAddr.sin_addr.s_addr;
                    it->port = followerAddr.sin_port;
                    it->lastHeartbeatMs = nowMs();
                } else {
                    VehicleInfo fi = status.info;
                    fi.ipAddress = followerAddr.sin_addr.s_addr;
                    fi.port = followerAddr.sin_port;
                    fi.lastHeartbeatMs = nowMs();
                    leader->platoonState_.vehicles.push_back(fi);
                }
                pthread_mutex_unlock(&leader->leaderMutex_);
            }
        }
    }
    return nullptr;
}

// Leader run thread
void* FollowingVehicle::leaderRunThreadEntry(void* arg) {
    FollowingVehicle* leader = static_cast<FollowingVehicle*>(arg);
    const int TIMESTEP_MS = 100;
    const double dt = TIMESTEP_MS / 1000.0;
    const double accel = 8;
    const double decel = 12;
    double targetSpeed = leader->info_.speed;
    
    while (leader->leaderRunning_) {
        // Simple leader movement
        FollowerState st = leader->getState();
        switch (st) {
            case NORMAL:
                if (leader->info_.speed < targetSpeed) {
                    leader->info_.speed = std::min(targetSpeed, leader->info_.speed + accel * dt);
                }
                break;
            case STOPPING:
            case STOPPING_FOR_RED_LIGHT:
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
        
        leader->info_.position += leader->info_.speed * dt;
        
        // Update in platoon state
        pthread_mutex_lock(&leader->leaderMutex_);
        for (auto& v : leader->platoonState_.vehicles) {
            if (v.id == leader->info_.id) {
                v.position = leader->info_.position;
                v.speed = leader->info_.speed;
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
        std::cout << "\033[2J\033[H"; // Clear screen
        std::cout << "\n\033[1;36m================== LEADER (ID=" << leader->info_.id << ") ==================\033[0m\n";
        std::cout << "Position: " << leader->info_.position
                  << "  Speed: " << leader->info_.speed << "\n";
        std::cout << "Platoon vehicles:\n";
        for (const auto& v : leader->platoonState_.vehicles) {
            std::cout << "  Vehicle ID: " << v.id
                      << ", Pos: " << v.position
                      << ", Spd: " << v.speed
                      << ", Mode: " << (v.mode == LeaderMode ? "Leader" : "Follower") << "\n";
        }
        std::cout << std::flush; // Force flush output buffer
        pthread_mutex_unlock(&leader->leaderMutex_);
        
        // Sleep at the end to avoid rapid initial updates
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    return nullptr;
}

// Leader status thread
void* FollowingVehicle::leaderStatusThreadEntry(void* arg) {
    FollowingVehicle* leader = static_cast<FollowingVehicle*>(arg);
    const int STATUS_INTERVAL_MS = 100;
    
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
        
        // Send platoon state periodically
        leader->sendPlatoonStateAsLeader();
        
        std::this_thread::sleep_for(std::chrono::milliseconds(STATUS_INTERVAL_MS));
    }
    return nullptr;
}

// Leader heartbeat thread
void* FollowingVehicle::leaderHeartbeatThreadEntry(void* arg) {
    FollowingVehicle* leader = static_cast<FollowingVehicle*>(arg);
    const int HEARTBEAT_INTERVAL_MS = 1000;
    const int TIMEOUT_MS = 3000;
    
    while (leader->leaderRunning_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(HEARTBEAT_INTERVAL_MS));
        std::int64_t currentTime = nowMs();
        
        pthread_mutex_lock(&leader->leaderMutex_);
        for (auto it = leader->platoonState_.vehicles.begin(); 
             it != leader->platoonState_.vehicles.end(); ) {
            if (it->id == leader->info_.id) {
                ++it;
                continue;
            }
            
            if (it->lastHeartbeatMs == 0 || 
                (currentTime - it->lastHeartbeatMs) > TIMEOUT_MS) {
                std::cout << "[HEARTBEAT] Follower " << it->id << " timed out\\n";
                it = leader->platoonState_.vehicles.erase(it);
            } else {
                ++it;
            }
        }
        pthread_mutex_unlock(&leader->leaderMutex_);
    }
    return nullptr;
}

// Leader event sender thread (handles traffic light events)
void* FollowingVehicle::leaderEventSenderThreadEntry(void* arg) {
    FollowingVehicle* leader = static_cast<FollowingVehicle*>(arg);
    
    while (leader->clientRunning_ && leader->leaderRunning_) {
        // Wait for event
        pthread_mutex_lock(&leader->eventMutex_);
        while (leader->eventQueue_.empty() && leader->clientRunning_ && leader->leaderRunning_) {
            pthread_cond_wait(&leader->eventCv_, &leader->eventMutex_);
        }
        if (!leader->clientRunning_ || !leader->leaderRunning_) {
            pthread_mutex_unlock(&leader->eventMutex_);
            break;
        }
        int eventCode = leader->eventQueue_.front();
        leader->eventQueue_.pop();
        pthread_mutex_unlock(&leader->eventMutex_);
        
        std::cout << "[LEADER EVENT] Processing event code: " << eventCode << "\\n";
        
        // Process event as leader
        switch (eventCode) {
            case 1: // Red light
                leader->setState(STOPPING_FOR_RED_LIGHT);
                break;
            case 2: // Green light
                leader->setState(STARTING);
                break;
            default:
                continue;
        }
        
        // Build and send traffic light message to rear vehicle (first follower)
        TrafficLightMessage tlMsg{};
        tlMsg.type = MessageType::TRAFFIC_LIGHT_ALERT;
        tlMsg.timestamp = nowMs();
        tlMsg.status = (eventCode == 1) ? LIGHT_RED : LIGHT_GREEN;
        
        // Get rear vehicle (first follower after leader in sorted list)
        pthread_mutex_lock(&leader->leaderMutex_);
        if (leader->platoonState_.vehicles.size() > 1) {
            // Find first non-leader vehicle
            for (const auto& v : leader->platoonState_.vehicles) {
                if (v.id == leader->info_.id) continue;
                if (v.ipAddress == 0 || v.port == 0) continue;
                
                struct sockaddr_in dest{};
                dest.sin_family = AF_INET;
                dest.sin_addr.s_addr = v.ipAddress;
                dest.sin_port = v.port;
                
                ssize_t sent = sendto(leader->clientSocket_, &tlMsg, sizeof(tlMsg), 0,
                                     (const struct sockaddr*)&dest, sizeof(dest));
                if (sent > 0) {
                    std::cout << "[LEADER EVENT] Sent traffic light event to vehicle " << v.id << "\\n";
                }
                break; // Only send to first follower
            }
        }
        pthread_mutex_unlock(&leader->leaderMutex_);
    }
    return nullptr;
}

