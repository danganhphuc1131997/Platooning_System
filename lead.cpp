#include "lead.h"

#include <chrono>
#include <cstring>
#include <iostream>
#include <thread>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

namespace {
// Timing / behaviour tuning
constexpr std::int64_t BROADCAST_PERIOD_MS = 300;
constexpr std::int64_t MONITOR_PERIOD_MS   = 200;

// Use-case: cut-in can cause temporary message loss.
// We treat short loss as DEGRADED mode, long loss as node failure.
constexpr std::int64_t DEGRADED_THRESHOLD_MS = 1000; // 1s
constexpr std::int64_t TIMEOUT_MS            = 3000; // 3s

constexpr double DEGRADED_SPEED_FACTOR = 0.8; // leader slows down slightly
}

LeadingVehicle::LeadingVehicle(int id, double initialPosition, double initialSpeed)
    : id_(id),
      position_(initialPosition),
      baseSpeed_(initialSpeed),
      currentSpeed_(initialSpeed),
      obstacleDetected_(false),
      serverSocket_(-1),
      running_(false) {
    pthread_mutex_init(&mutex_, nullptr);

    // Fill free clock indices 1..MAX_NODES-1
    for (int i = 1; i < MAX_NODES; ++i) {
        freeClockIndices_.push_back(i);
    }
    initClock();
}

LeadingVehicle::~LeadingVehicle() {
    stopServer();
    pthread_mutex_destroy(&mutex_);
}

void LeadingVehicle::startServer() {
    createServerSocket();
    running_.store(true);

    // Start accept / broadcast / monitor threads
    pthread_create(&acceptThread_, nullptr, &LeadingVehicle::acceptThreadEntry, this);
    pthread_create(&broadcastThread_, nullptr, &LeadingVehicle::broadcastThreadEntry, this);
    pthread_create(&monitorThread_, nullptr, &LeadingVehicle::monitorThreadEntry, this);

    std::cout << "Leader listening on port " << PORT << "..." << std::endl;

    // Block main thread until stopped (Ctrl+C)
    // You can keep printing state here if you want.
    while (running_.load()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

void LeadingVehicle::stopServer() {
    bool expected = true;
    if (!running_.compare_exchange_strong(expected, false)) {
        // already stopped
        return;
    }

    // Close server socket to break accept
    if (serverSocket_ >= 0) {
        ::shutdown(serverSocket_, SHUT_RDWR);
        ::close(serverSocket_);
        serverSocket_ = -1;
    }

    // Join threads
    if (acceptThread_) pthread_join(acceptThread_, nullptr);
    if (broadcastThread_) pthread_join(broadcastThread_, nullptr);
    if (monitorThread_) pthread_join(monitorThread_, nullptr);

    // Close all followers
    pthread_mutex_lock(&mutex_);
    for (auto &kv : followers_) {
        ::shutdown(kv.second.socketFd, SHUT_RDWR);
        ::close(kv.second.socketFd);
    }
    followers_.clear();
    idToClockIndex_.clear();
    pthread_mutex_unlock(&mutex_);
}

void LeadingVehicle::createServerSocket() {
    serverSocket_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (serverSocket_ < 0) {
        throw std::runtime_error("Failed to create server socket");
    }

    int opt = 1;
    ::setsockopt(serverSocket_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);

    if (::bind(serverSocket_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0) {
        ::close(serverSocket_);
        serverSocket_ = -1;
        throw std::runtime_error("Bind failed (port in use?)");
    }

    if (::listen(serverSocket_, 10) < 0) {
        ::close(serverSocket_);
        serverSocket_ = -1;
        throw std::runtime_error("Listen failed");
    }
}

void LeadingVehicle::initClock() {
    std::memset(clock_, 0, sizeof(clock_));
    // Leader is index 0
    clock_[0][0] = 1;
}

void LeadingVehicle::printClock() {
    std::cout << "Clock Matrix:" << std::endl;
    for (int i = 0; i < MAX_NODES; ++i) {
        for (int j = 0; j < MAX_NODES; ++j) {
            std::cout << clock_[i][j] << (j + 1 == MAX_NODES ? "" : " ");
        }
        std::cout << std::endl;
    }
}

void LeadingVehicle::mergeClockElementwiseMax(const std::int32_t other[MAX_NODES][MAX_NODES]) {
    for (int i = 0; i < MAX_NODES; ++i) {
        for (int j = 0; j < MAX_NODES; ++j) {
            if (other[i][j] > clock_[i][j]) clock_[i][j] = other[i][j];
        }
    }
}

// Matrix-clock receive rule (simplified):
//  1) Element-wise max merge
//  2) Update our row with sender's knowledge: clock_[self][j] = max(clock_[self][j], other[sender][j])
//  3) Advance our local logical time
void LeadingVehicle::onClockReceive(int selfIdx, int senderIdx,
                                   const std::int32_t other[MAX_NODES][MAX_NODES]) {
    if (selfIdx < 0 || selfIdx >= MAX_NODES) return;
    if (senderIdx < 0 || senderIdx >= MAX_NODES) {
        mergeClockElementwiseMax(other);
        clock_[selfIdx][selfIdx] += 1;
        return;
    }

    mergeClockElementwiseMax(other);

    for (int j = 0; j < MAX_NODES; ++j) {
        if (other[senderIdx][j] > clock_[selfIdx][j]) clock_[selfIdx][j] = other[senderIdx][j];
    }

    // local event after receive
    const int senderTime = other[senderIdx][senderIdx];
    if (senderTime + 1 > clock_[selfIdx][selfIdx]) clock_[selfIdx][selfIdx] = senderTime + 1;
    else clock_[selfIdx][selfIdx] += 1;
}

void LeadingVehicle::onClockLocalEvent(int selfIdx) {
    if (selfIdx < 0 || selfIdx >= MAX_NODES) return;
    clock_[selfIdx][selfIdx] += 1;
}

std::int64_t LeadingVehicle::nowMs() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

void* LeadingVehicle::acceptThreadEntry(void* arg) {
    static_cast<LeadingVehicle*>(arg)->acceptLoop();
    return nullptr;
}

void* LeadingVehicle::broadcastThreadEntry(void* arg) {
    static_cast<LeadingVehicle*>(arg)->broadcastLoop();
    return nullptr;
}

void* LeadingVehicle::monitorThreadEntry(void* arg) {
    static_cast<LeadingVehicle*>(arg)->monitorLoop();
    return nullptr;
}

void LeadingVehicle::acceptLoop() {
    while (running_.load()) {
        sockaddr_in clientAddr{};
        socklen_t clientLen = sizeof(clientAddr);
        int clientSocket = ::accept(serverSocket_, reinterpret_cast<sockaddr*>(&clientAddr), &clientLen);
        if (clientSocket < 0) {
            if (running_.load()) {
                // Accept can fail when socket is closed during shutdown
                continue;
            }
            break;
        }

        // Expect JOIN as the very first message so we can bind this socket to the real follower id.
        WireMessage join{};
        if (!recvAll(clientSocket, &join, sizeof(join)) ||
            static_cast<MsgType>(join.type) != MsgType::JOIN) {
            std::cerr << "Leader: first message was not JOIN. Closing connection." << std::endl;
            ::close(clientSocket);
            continue;
        }

        const int followerId = join.senderId;

        // Allocate clock index for this follower
        pthread_mutex_lock(&mutex_);
        if (freeClockIndices_.empty()) {
            pthread_mutex_unlock(&mutex_);
            std::cerr << "Max followers reached. Closing connection." << std::endl;
            ::close(clientSocket);
            continue;
        }

        const int assignedIndex = freeClockIndices_.back();
        freeClockIndices_.pop_back();

        idToClockIndex_[followerId] = assignedIndex;

        // Matrix-clock: treat JOIN as a receive event (senderIndex unknown / -1).
        onClockReceive(0, join.senderIndex, join.clock);

        FollowerInfo info;
        info.socketFd = clientSocket;
        info.followerId = followerId;
        info.clockIndex = assignedIndex;
        info.lastSeenMs = nowMs();
        followers_[clientSocket] = info;
        pthread_mutex_unlock(&mutex_);

        // Send JOIN_ACK with assigned index so follower can use consistent matrix-clock indexing.
        WireMessage ack{};
        ack.type = static_cast<std::uint8_t>(MsgType::JOIN_ACK);
        ack.senderId = id_;
        ack.senderIndex = 0;
        ack.assignedIndex = assignedIndex;
        ack.position = position_;
        ack.speed = currentSpeed_;
        ack.obstacle = 0;
        ack.flags = FLAG_CONNECTED;
        std::memcpy(ack.clock, clock_, sizeof(clock_));
        sendAll(clientSocket, &ack, sizeof(ack));

        std::cout << "Follower joined. ID: " << followerId
                  << " (clockIndex=" << assignedIndex << ")" << std::endl;

// Start handler thread for this follower
        auto* args = new HandlerArgs{this, clientSocket};
        pthread_t t;
        pthread_create(&t, nullptr, &LeadingVehicle::followerHandlerEntry, args);
        pthread_detach(t);
    }
}

void* LeadingVehicle::followerHandlerEntry(void* arg) {
    HandlerArgs* args = static_cast<HandlerArgs*>(arg);
    args->self->handleFollower(args->clientSocket);
    delete args;
    return nullptr;
}

void LeadingVehicle::handleFollower(int clientSocket) {
    while (running_.load()) {
        WireMessage msg{};
        if (!recvAll(clientSocket, &msg, sizeof(msg))) {
            pthread_mutex_lock(&mutex_);
            removeFollowerLocked(clientSocket, "socket closed");
            pthread_mutex_unlock(&mutex_);
            return;
        }

        pthread_mutex_lock(&mutex_);
        auto it = followers_.find(clientSocket);
        if (it == followers_.end()) {
            pthread_mutex_unlock(&mutex_);
            return;
        }

        // Merge clock & update last seen
        onClockReceive(0, msg.senderIndex, msg.clock); // leader increments its own logical clock on receive
        it->second.lastSeenMs = nowMs();
        it->second.position = msg.position;
        it->second.speed = msg.speed;

        if (msg.obstacle) {
            obstacleDetected_ = true;
            std::cout << "Follower ID " << it->second.followerId << " is facing an obstacle" << std::endl;
        }

        if (static_cast<MsgType>(msg.type) == MsgType::LEAVE) {
            removeFollowerLocked(clientSocket, "graceful leave");
            pthread_mutex_unlock(&mutex_);
            return;
        }

        pthread_mutex_unlock(&mutex_);
    }
}

void LeadingVehicle::broadcastLoop() {
    auto last = nowMs();
    while (running_.load()) {
        auto now = nowMs();
        auto dtMs = now - last;
        last = now;

        pthread_mutex_lock(&mutex_);
        // Update leader position based on currentSpeed_
        double dtSec = static_cast<double>(dtMs) / 1000.0;
        position_ += currentSpeed_ * dtSec;

        // Increment logical clock on send
        onClockLocalEvent(0);

        // Print state occasionally (kept simple)
        std::cout << "\nLeading Vehicle ID: " << id_
                  << "  Position: " << position_
                  << "  Speed: " << currentSpeed_ << std::endl;
        printClock();

        // Prepare message
        std::uint8_t flags = FLAG_CONNECTED;
        if (!obstacleDetected_ && currentSpeed_ < baseSpeed_) {
            flags |= FLAG_DEGRADED;
        }
        WireMessage out = makeLeaderStateMessage(flags);

        // Copy followers socket list to avoid iterator invalidation during send
        std::vector<int> sockets;
        sockets.reserve(followers_.size());
        for (const auto& kv : followers_) sockets.push_back(kv.first);
        pthread_mutex_unlock(&mutex_);

        for (int fd : sockets) {
            if (!sendAll(fd, &out, sizeof(out))) {
                pthread_mutex_lock(&mutex_);
                removeFollowerLocked(fd, "send failed");
                pthread_mutex_unlock(&mutex_);
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(BROADCAST_PERIOD_MS));
    }
}

void LeadingVehicle::monitorLoop() {
    while (running_.load()) {
        pthread_mutex_lock(&mutex_);

        const auto now = nowMs();
        bool anyDegraded = false;

        // If obstacle: immediate stop.
        if (obstacleDetected_) {
            currentSpeed_ = 0.0;
        } else {
            // Check followers timeouts
            std::vector<int> toRemove;
            for (const auto& kv : followers_) {
                const auto& fi = kv.second;
                const auto delta = now - fi.lastSeenMs;
                if (delta > TIMEOUT_MS) {
                    toRemove.push_back(kv.first);
                } else if (delta > DEGRADED_THRESHOLD_MS) {
                    anyDegraded = true;
                }
            }

            for (int fd : toRemove) {
                removeFollowerLocked(fd, "timeout (node failure)");
            }

            // Two-phase strategy: slightly slow down if temporary loss
            currentSpeed_ = anyDegraded ? (baseSpeed_ * DEGRADED_SPEED_FACTOR) : baseSpeed_;
        }

        pthread_mutex_unlock(&mutex_);
        std::this_thread::sleep_for(std::chrono::milliseconds(MONITOR_PERIOD_MS));
    }
}

WireMessage LeadingVehicle::makeLeaderStateMessage(std::uint8_t flags) {
    WireMessage m{};
    m.type = static_cast<std::uint8_t>(MsgType::LEADER_STATE);
    m.senderId = id_;
    m.senderIndex = 0;
    m.assignedIndex = -1;
    m.position = position_;
    m.speed = currentSpeed_;
    m.obstacle = obstacleDetected_ ? 1 : 0;
    m.flags = flags;
    std::memcpy(m.clock, clock_, sizeof(clock_));
    return m;
}

bool LeadingVehicle::sendAll(int fd, const void* data, size_t len) {
    const std::uint8_t* p = static_cast<const std::uint8_t*>(data);
    size_t sent = 0;
    while (sent < len) {
        ssize_t n = ::send(fd, p + sent, len - sent, 0);
        if (n <= 0) return false;
        sent += static_cast<size_t>(n);
    }
    return true;
}

bool LeadingVehicle::recvAll(int fd, void* data, size_t len) {
    std::uint8_t* p = static_cast<std::uint8_t*>(data);
    size_t recvd = 0;
    while (recvd < len) {
        ssize_t n = ::recv(fd, p + recvd, len - recvd, 0);
        if (n <= 0) return false;
        recvd += static_cast<size_t>(n);
    }
    return true;
}

void LeadingVehicle::removeFollowerLocked(int clientSocket, const char* reason) {
    auto it = followers_.find(clientSocket);
    if (it == followers_.end()) return;

    const int fid = it->second.followerId;
    const int idx = it->second.clockIndex;

    std::cout << "Follower " << fid << " removed (" << reason << ")" << std::endl;

    // recycle clock index
    idToClockIndex_.erase(fid);
    freeClockIndices_.push_back(idx);

    ::shutdown(clientSocket, SHUT_RDWR);
    ::close(clientSocket);
    followers_.erase(it);
}

// ---------------------------
// Program entry point
// ---------------------------
int main(int argc, char** argv) {
    // Optional CLI: ./lead [baseSpeed] [initialPosition]
    double baseSpeed = 60.0;
    double initialPos = 50.0;
    if (argc >= 2) baseSpeed = std::stod(argv[1]);
    if (argc >= 3) initialPos = std::stod(argv[2]);

    try {
        LeadingVehicle leader(/*id=*/1, initialPos, baseSpeed);
        leader.startServer();
    } catch (const std::exception& ex) {
        std::cerr << "Leader error: " << ex.what() << std::endl;
        return 1;
    }
    return 0;
}
