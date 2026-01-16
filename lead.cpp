#include "lead.h"

#include <chrono>
#include <cstring>
#include <iostream>
#include <thread>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

// ---------------------------- config ----------------------------
namespace {
// Timing / behaviour tuning
constexpr std::int64_t BROADCAST_PERIOD_MS = 300;
constexpr std::int64_t MONITOR_PERIOD_MS   = 200;

// Use-case: cut-in can cause temporary message loss.
// We treat short loss as DEGRADED mode, long loss as node failure.
constexpr std::int64_t DEGRADED_THRESHOLD_MS = 1000; // 1s
constexpr std::int64_t TIMEOUT_MS            = 3000; // 3s

constexpr double DEGRADED_SPEED_FACTOR = 0.8; // leader slows down slightly

// Traffic light use-case
constexpr double TRAFFIC_LIGHT_STOPLINE_POS = 200.0;
constexpr double LIGHT_DETECTION_RANGE      = 80.0;   // detect light when within 80m
constexpr std::int64_t GREEN_DURATION_MS    = 6000;   // 6s green
constexpr std::int64_t RED_DURATION_MS      = 4000;   // 4s red
constexpr double STOP_EPS                   = 0.5;    // close enough to stop line
}

// --------------------- small terminal helpers ---------------------
static const char* colorLight(std::uint8_t tl) {
    if (tl == static_cast<std::uint8_t>(TrafficLight::RED))   return "\033[1;31mRED\033[0m";
    if (tl == static_cast<std::uint8_t>(TrafficLight::GREEN)) return "\033[1;32mGREEN\033[0m";
    return "NONE";
}

// ------------------------------ class ------------------------------

LeadingVehicle::LeadingVehicle(int id, double initialPosition, double initialSpeed)
    : id_(id),
      position_(initialPosition),
      baseSpeed_(initialSpeed),
      currentSpeed_(initialSpeed),
      obstacleDetected_(false),
      serverSocket_(-1),
      trafficLight_(static_cast<std::uint8_t>(TrafficLight::NONE)),
      stopLinePos_(TRAFFIC_LIGHT_STOPLINE_POS),
      lightPhaseStartMs_(0),
      lightActive_(false),
      running_(false) {
    pthread_mutex_init(&mutex_, nullptr);

    // Fill vehicle index for leader
    vehicleIndex[id_] = 0;
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

    std::cout << "\033[1;36mLeader listening on port " << PORT << "...\033[0m\n";

    // Block main thread until stopped
    while (running_.load()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

void LeadingVehicle::stopServer() {
    bool expected = true;
    if (!running_.compare_exchange_strong(expected, false)) {
        return; // already stopped
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
    vehicleIndex.clear();
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
    vehicleIndex[id_] = 0;
    clock_[0][0] = 1;
}

void LeadingVehicle::printClock() {
    int n = 0;
    for (auto& p : vehicleIndex) n = std::max(n, p.second + 1);

    std::cout << "Clock Matrix (" << n << " vehicles):\n";
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            std::cout << clock_[i][j] << (j + 1 == n ? "" : " ");
        }
        std::cout << "\n";
    }
}

void LeadingVehicle::mergeClockElementwiseMax(const std::int32_t other[MAX_VEHICLES][MAX_VEHICLES]) {
    for (int i = 0; i < MAX_VEHICLES; ++i) {
        for (int j = 0; j < MAX_VEHICLES; ++j) {
            if (other[i][j] > clock_[i][j]) clock_[i][j] = other[i][j];
        }
    }
}

void LeadingVehicle::onClockReceive(int selfIdx, int senderIdx,
                                   const std::int32_t other[MAX_VEHICLES][MAX_VEHICLES]) {
    if (selfIdx < 0 || selfIdx >= MAX_VEHICLES) return;

    if (senderIdx < 0 || senderIdx >= MAX_VEHICLES) {
        mergeClockElementwiseMax(other);
        clock_[selfIdx][selfIdx] += 1;
        return;
    }

    mergeClockElementwiseMax(other);

    for (int j = 0; j < MAX_VEHICLES; ++j) {
        if (other[senderIdx][j] > clock_[selfIdx][j]) clock_[selfIdx][j] = other[senderIdx][j];
    }

    // local event after receive
    const int senderTime = other[senderIdx][senderIdx];
    if (senderTime + 1 > clock_[selfIdx][selfIdx]) clock_[selfIdx][selfIdx] = senderTime + 1;
    else clock_[selfIdx][selfIdx] += 1;
}

void LeadingVehicle::onClockLocalEvent(int selfIdx) {
    if (selfIdx < 0 || selfIdx >= MAX_VEHICLES) return;
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
            if (running_.load()) continue;
            break;
        }

        // Expect JOIN as the very first message
        WireMessage join{};
        if (!recvAll(clientSocket, &join, sizeof(join)) ||
            static_cast<MsgType>(join.type) != MsgType::JOIN) {
            std::cerr << "Leader: first message was not JOIN. Closing connection.\n";
            ::close(clientSocket);
            continue;
        }

        const int followerId = join.senderId;

        pthread_mutex_lock(&mutex_);

        if (followers_.size() >= MAX_VEHICLES - 1) {
            pthread_mutex_unlock(&mutex_);
            std::cerr << "Max followers reached. Closing connection.\n";
            ::close(clientSocket);
            continue;
        }

        // Assign matrix index: 1..MAX_VEHICLES-1
        const int assignedIndex = static_cast<int>(vehicleIndex.size());
        vehicleIndex[followerId] = assignedIndex;

        // JOIN counts as receive event
        onClockReceive(0, join.senderIndex, join.clockMatrix);

        FollowerInfo info;
        info.socketFd = clientSocket;
        info.followerId = followerId;
        info.clockIndex = assignedIndex;
        info.lastSeenMs = nowMs();
        followers_[clientSocket] = info;

        pthread_mutex_unlock(&mutex_);

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

        // Traffic-light info for follower
        ack.trafficLight = trafficLight_;
        ack.stopLinePos = stopLinePos_;
        ack.followerMode = 0;

        std::memcpy(ack.clockMatrix, clock_, sizeof(clock_));

        // Fill mapping arrays (nice for robustness / demo)
        int k = 0;
        for (auto& p : vehicleIndex) {
            if (k >= MAX_VEHICLES) break;
            ack.vehicleIds[k] = p.first;
            ack.vehicleIndices[k] = p.second;
            ++k;
        }
        ack.numVehicles = static_cast<int>(vehicleIndex.size());

        sendAll(clientSocket, &ack, sizeof(ack));

        std::cout << "\033[1;32mFollower joined\033[0m  ID=" << followerId
                  << "  clockIndex=" << assignedIndex << "\n";

        // Start handler thread
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
        onClockReceive(0, msg.senderIndex, msg.clockMatrix);
        it->second.lastSeenMs = nowMs();
        it->second.position = msg.position;
        it->second.speed = msg.speed;

        if (msg.obstacle) {
            obstacleDetected_ = true;
            std::cout << "\n\033[1;31m[LEADER] Obstacle reported by follower "
                      << it->second.followerId << " -> STOP\033[0m\n";
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

        // Update leader position
        double dtSec = static_cast<double>(dtMs) / 1000.0;
        position_ += currentSpeed_ * dtSec;

        // Local event on send
        onClockLocalEvent(0);

        // Header print
        std::cout << "\n\033[1;36m================== LEADER ==================\033[0m\n";
        std::cout << "ID: " << id_
                  << "  Position: " << position_
                  << "  Speed: " << currentSpeed_ << "\n";

        if (lightActive_) {
            std::cout << "TrafficLight: " << colorLight(trafficLight_)
                      << "  StopLine: " << stopLinePos_ << "m\n";
        }

        printClock();
        std::cout << std::flush;

        // Prepare message
        std::uint8_t flags = FLAG_CONNECTED;
        if (!obstacleDetected_ && currentSpeed_ < baseSpeed_) {
            flags |= FLAG_DEGRADED;
        }

        WireMessage out = makeLeaderStateMessage(flags);

        // Copy socket list (avoid invalidation)
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

        // ---------- Traffic light simulation ----------
        const double distToStopLine = stopLinePos_ - position_;

        if (!lightActive_) {
            if (distToStopLine <= LIGHT_DETECTION_RANGE) {
                lightActive_ = true;
                trafficLight_ = static_cast<std::uint8_t>(TrafficLight::GREEN);
                lightPhaseStartMs_ = now;

                std::cout << "\n\033[1;36m[LEADER] Traffic light detected!\033[0m"
                          << " StopLine=" << stopLinePos_ << "m\n";
            }
        } else {
            const auto phaseElapsed = now - lightPhaseStartMs_;
            if (trafficLight_ == static_cast<std::uint8_t>(TrafficLight::GREEN) &&
                phaseElapsed > GREEN_DURATION_MS) {
                trafficLight_ = static_cast<std::uint8_t>(TrafficLight::RED);
                lightPhaseStartMs_ = now;
                std::cout << "\n\033[1;31m[LIGHT] turned RED\033[0m\n";
            } else if (trafficLight_ == static_cast<std::uint8_t>(TrafficLight::RED) &&
                       phaseElapsed > RED_DURATION_MS) {
                trafficLight_ = static_cast<std::uint8_t>(TrafficLight::GREEN);
                lightPhaseStartMs_ = now;
                std::cout << "\n\033[1;32m[LIGHT] turned GREEN\033[0m\n";
            }
        }

        // ---------- Failure detection ----------
        if (obstacleDetected_) {
            currentSpeed_ = 0.0;
        } else {
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

            double desired = anyDegraded ? (baseSpeed_ * DEGRADED_SPEED_FACTOR) : baseSpeed_;

            // ---------- Traffic light braking ----------
            if (lightActive_ && trafficLight_ == static_cast<std::uint8_t>(TrafficLight::RED)) {
                // If not past stop line, brake to stop line.
                if (position_ < stopLinePos_ - STOP_EPS) {
                    desired = std::min(desired, 15.0);
                    if (distToStopLine < 15.0) desired = std::min(desired, 5.0);
                    if (distToStopLine < 5.0) desired = 0.0;
                }
            }

            currentSpeed_ = desired;
        }

        pthread_mutex_unlock(&mutex_);
        std::this_thread::sleep_for(std::chrono::milliseconds(MONITOR_PERIOD_MS));
    }
}

WireMessage LeadingVehicle::makeLeaderStateMessage(std::uint8_t flags) {
    int n = 0;
    for (auto& p : vehicleIndex) n = std::max(n, p.second + 1);

    WireMessage m{};
    m.type = static_cast<std::uint8_t>(MsgType::LEADER_STATE);
    m.senderId = id_;
    m.senderIndex = 0;
    m.assignedIndex = -1;

    m.position = position_;
    m.speed = currentSpeed_;
    m.obstacle = obstacleDetected_ ? 1 : 0;
    m.flags = flags;

    // Traffic light information
    m.trafficLight = trafficLight_;
    m.stopLinePos = stopLinePos_;
    m.followerMode = 0;

    std::memcpy(m.clockMatrix, clock_, sizeof(clock_));

    // fill vehicle mapping arrays (for debug/robustness)
    int k = 0;
    for (auto& p : vehicleIndex) {
        if (k >= MAX_VEHICLES) break;
        m.vehicleIds[k] = p.first;
        m.vehicleIndices[k] = p.second;
        ++k;
    }

    m.numVehicles = n;
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

    std::cout << "\033[1;33mFollower " << fid << " removed\033[0m"
              << " (" << reason << ")\n";

    vehicleIndex.erase(fid);

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
