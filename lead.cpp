#include "lead.h"

#include <chrono>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

// ANSI color codes for terminal output
#define RESET   "\033[0m"
#define BOLD    "\033[1m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"
#define CYAN    "\033[36m"
#define CLEAR   "\033[2J\033[H"

namespace {
// Timing / behaviour tuning
constexpr std::int64_t BROADCAST_PERIOD_MS = 300;
constexpr std::int64_t MONITOR_PERIOD_MS   = 200;

// Use-case: cut-in can cause temporary message loss.
// We treat short loss as DEGRADED mode, long loss as node failure.
constexpr std::int64_t DEGRADED_THRESHOLD_MS = 1000; // 1s
constexpr std::int64_t TIMEOUT_MS            = 3000; // 3s

constexpr double DEGRADED_SPEED_FACTOR = 0.8; // leader slows down slightly
constexpr int DISPLAY_UPDATE_INTERVAL = 5; // update dashboard every N broadcasts
}

std::string LeadingVehicle::addrKey(const sockaddr_in& a) {
    char buf[64]{};
    ::inet_ntop(AF_INET, &a.sin_addr, buf, sizeof(buf));
    const int port = ntohs(a.sin_port);
    return std::string(buf) + ":" + std::to_string(port);
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

    // Start recv / broadcast / monitor threads
    pthread_create(&recvThread_, nullptr, &LeadingVehicle::recvThreadEntry, this);
    pthread_create(&broadcastThread_, nullptr, &LeadingVehicle::broadcastThreadEntry, this);
    pthread_create(&monitorThread_, nullptr, &LeadingVehicle::monitorThreadEntry, this);

    std::cout << "Leader listening (UDP) on port " << PORT << "..." << std::endl;

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

    // Close server socket to break recv
    if (serverSocket_ >= 0) {
        ::close(serverSocket_);
        serverSocket_ = -1;
    }

    // Join threads
    if (recvThread_) pthread_join(recvThread_, nullptr);
    if (broadcastThread_) pthread_join(broadcastThread_, nullptr);
    if (monitorThread_) pthread_join(monitorThread_, nullptr);

    // Clear all followers
    pthread_mutex_lock(&mutex_);
    followers_.clear();
    idToClockIndex_.clear();
    pthread_mutex_unlock(&mutex_);
}

void LeadingVehicle::createServerSocket() {
    serverSocket_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (serverSocket_ < 0) {
        throw std::runtime_error("Failed to create UDP server socket");
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
}

void LeadingVehicle::initClock() {
    std::memset(clock_, 0, sizeof(clock_));
    // Leader is index 0
    clock_[0][0] = 1;
}

void LeadingVehicle::printClock() {
    // Compact clock display
    std::cout << CYAN << "Clock[0]: [";
    for (int j = 0; j < MAX_NODES; ++j) {
        std::cout << clock_[0][j] << (j + 1 == MAX_NODES ? "" : ",");
    }
    std::cout << "]" << RESET << std::endl;
}

void LeadingVehicle::printDashboard() {
    std::cout << CLEAR;  // Clear screen
    std::cout << BOLD << "╔════════════════════════════════════════════════════════╗" << RESET << std::endl;
    std::cout << BOLD << "║" << GREEN << "           🚗 LEADER VEHICLE DASHBOARD                " << RESET << BOLD << "║" << RESET << std::endl;
    std::cout << BOLD << "╠════════════════════════════════════════════════════════╣" << RESET << std::endl;
    
    // Status
    std::ostringstream status;
    if (obstacleDetected_) {
        status << RED << "⛔ STOPPED (obstacle)" << RESET;
    } else if (currentSpeed_ < baseSpeed_) {
        status << YELLOW << "⚠️  DEGRADED MODE" << RESET;
    } else {
        status << GREEN << "✓ NORMAL" << RESET;
    }
    std::cout << BOLD << "║" << RESET << " Status: " << std::left << std::setw(45) << status.str() << BOLD << "║" << RESET << std::endl;
    
    // Position & Speed
    std::cout << BOLD << "║" << RESET << " Position: " << CYAN << std::fixed << std::setprecision(1) << std::setw(10) << position_ << RESET 
              << " m    Speed: " << CYAN << std::setw(6) << currentSpeed_ << RESET << " m/s       " << BOLD << "║" << RESET << std::endl;
    
    std::cout << BOLD << "╠════════════════════════════════════════════════════════╣" << RESET << std::endl;
    std::cout << BOLD << "║" << RESET << " Followers: " << YELLOW << followers_.size() << RESET << "                                           " << BOLD << "║" << RESET << std::endl;
    
    for (const auto& kv : followers_) {
        const auto& f = kv.second;
        auto age = nowMs() - f.lastSeenMs;
        std::string ageColor = (age < 500) ? GREEN : (age < 1000) ? YELLOW : RED;
        std::cout << BOLD << "║" << RESET << "   ID " << f.followerId 
                  << ": pos=" << std::fixed << std::setprecision(1) << f.position 
                  << " spd=" << std::setprecision(1) << f.speed
                  << " " << ageColor << "(" << age << "ms ago)" << RESET;
        // Padding
        std::cout << std::string(10, ' ') << BOLD << "║" << RESET << std::endl;
    }
    
    std::cout << BOLD << "╠════════════════════════════════════════════════════════╣" << RESET << std::endl;
    printClock();
    std::cout << BOLD << "╚════════════════════════════════════════════════════════╝" << RESET << std::endl;
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

void* LeadingVehicle::recvThreadEntry(void* arg) {
    static_cast<LeadingVehicle*>(arg)->recvLoop();
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

void LeadingVehicle::recvLoop() {
    while (running_.load()) {
        WireMessage msg{};
        sockaddr_in clientAddr{};
        socklen_t clientLen = sizeof(clientAddr);
        ssize_t n = ::recvfrom(serverSocket_, &msg, sizeof(msg), 0,
                               reinterpret_cast<sockaddr*>(&clientAddr), &clientLen);
        if (n <= 0) {
            if (running_.load()) continue;
            break;
        }
        if (static_cast<size_t>(n) != sizeof(msg)) continue;

        const std::string key = addrKey(clientAddr);

        if (static_cast<MsgType>(msg.type) == MsgType::JOIN) {
            pthread_mutex_lock(&mutex_);
            if (freeClockIndices_.empty()) {
                pthread_mutex_unlock(&mutex_);
                std::cerr << "Max followers reached. Ignoring JOIN from " << key << std::endl;
                continue;
            }

            const int assignedIndex = freeClockIndices_.back();
            freeClockIndices_.pop_back();
            idToClockIndex_[msg.senderId] = assignedIndex;

            onClockReceive(0, msg.senderIndex, msg.clock);

            FollowerInfo info;
            info.addr = clientAddr;
            info.followerId = msg.senderId;
            info.clockIndex = assignedIndex;
            info.lastSeenMs = nowMs();
            followers_[key] = info;

            // Send JOIN_ACK
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
            sendToFollower(clientAddr, &ack, sizeof(ack));

            std::cout << "Follower joined. ID: " << msg.senderId
                      << " (" << key << ") clockIndex=" << assignedIndex << std::endl;
            pthread_mutex_unlock(&mutex_);
            continue;
        }

        // For other messages, lookup follower
        pthread_mutex_lock(&mutex_);
        auto it = followers_.find(key);
        if (it == followers_.end()) {
            pthread_mutex_unlock(&mutex_);
            continue;
        }

        onClockReceive(0, msg.senderIndex, msg.clock);
        it->second.lastSeenMs = nowMs();
        it->second.position = msg.position;
        it->second.speed = msg.speed;

        if (msg.obstacle) {
            obstacleDetected_ = true;
            std::cout << "Follower ID " << it->second.followerId << " is facing an obstacle" << std::endl;
        }

        if (static_cast<MsgType>(msg.type) == MsgType::LEAVE) {
            removeFollowerLocked(key, "graceful leave");
            pthread_mutex_unlock(&mutex_);
            continue;
        }

        pthread_mutex_unlock(&mutex_);
    }
}

bool LeadingVehicle::sendToFollower(const sockaddr_in& addr, const void* data, size_t len) {
    ssize_t n = ::sendto(serverSocket_, data, len, 0,
                         reinterpret_cast<const sockaddr*>(&addr), sizeof(addr));
    return (n == static_cast<ssize_t>(len));
}

void LeadingVehicle::broadcastLoop() {
    auto last = nowMs();
    int displayCounter = 0;
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

        // Print dashboard every N iterations
        if (++displayCounter >= DISPLAY_UPDATE_INTERVAL) {
            displayCounter = 0;
            printDashboard();
        }

        // Prepare message
        std::uint8_t flags = FLAG_CONNECTED;
        if (!obstacleDetected_ && currentSpeed_ < baseSpeed_) {
            flags |= FLAG_DEGRADED;
        }
        WireMessage out = makeLeaderStateMessage(flags);

        // Copy followers list to avoid iterator invalidation during send
        std::vector<FollowerInfo> followerList;
        followerList.reserve(followers_.size());
        for (const auto& kv : followers_) followerList.push_back(kv.second);
        pthread_mutex_unlock(&mutex_);

        for (const auto& fi : followerList) {
            if (!sendToFollower(fi.addr, &out, sizeof(out))) {
                pthread_mutex_lock(&mutex_);
                removeFollowerLocked(addrKey(fi.addr), "send failed");
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
            std::vector<std::string> toRemove;
            for (const auto& kv : followers_) {
                const auto& fi = kv.second;
                const auto delta = now - fi.lastSeenMs;
                if (delta > TIMEOUT_MS) {
                    toRemove.push_back(kv.first);
                } else if (delta > DEGRADED_THRESHOLD_MS) {
                    anyDegraded = true;
                }
            }

            for (const auto& key : toRemove) {
                removeFollowerLocked(key, "timeout (node failure)");
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

void LeadingVehicle::removeFollowerLocked(const std::string& key, const char* reason) {
    auto it = followers_.find(key);
    if (it == followers_.end()) return;

    const int fid = it->second.followerId;
    const int idx = it->second.clockIndex;

    std::cout << "Follower " << fid << " removed (" << reason << ")" << std::endl;

    // recycle clock index
    idToClockIndex_.erase(fid);
    freeClockIndices_.push_back(idx);

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
