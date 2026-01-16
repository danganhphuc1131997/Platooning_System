#include "follow.h"

#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <netinet/in.h>
#include <sys/socket.h>
#include <termios.h>
#include <thread>
#include <unistd.h>

#ifdef USE_OPENCL
#include <CL/cl.h>
#include <vector>
#endif

namespace {
constexpr std::int64_t HEARTBEAT_PERIOD_MS = 500;

#ifdef USE_OPENCL
// Very small GPU demo (OpenCL):
// Compute the minimum absolute distance between the ego position and a set of obstacle positions.
static bool g_clInitOk = false;
static cl_context g_ctx = nullptr;
static cl_command_queue g_q = nullptr;
static cl_program g_prog = nullptr;
static cl_kernel g_kernel = nullptr;

static bool initOpenCLOnce() {
    if (g_clInitOk) return true;

    cl_int err = CL_SUCCESS;
    cl_uint numPlatforms = 0;
    err = clGetPlatformIDs(0, nullptr, &numPlatforms);
    if (err != CL_SUCCESS || numPlatforms == 0) return false;

    std::vector<cl_platform_id> platforms(numPlatforms);
    if (clGetPlatformIDs(numPlatforms, platforms.data(), nullptr) != CL_SUCCESS) return false;

    cl_device_id dev = nullptr;
    for (auto p : platforms) {
        cl_uint numDevices = 0;
        if (clGetDeviceIDs(p, CL_DEVICE_TYPE_GPU, 1, &dev, &numDevices) == CL_SUCCESS && numDevices > 0) break;
        dev = nullptr;
    }
    if (!dev) {
        for (auto p : platforms) {
            cl_uint numDevices = 0;
            if (clGetDeviceIDs(p, CL_DEVICE_TYPE_CPU, 1, &dev, &numDevices) == CL_SUCCESS && numDevices > 0) break;
            dev = nullptr;
        }
    }
    if (!dev) return false;

    g_ctx = clCreateContext(nullptr, 1, &dev, nullptr, nullptr, &err);
    if (!g_ctx || err != CL_SUCCESS) return false;

    g_q = clCreateCommandQueue(g_ctx, dev, 0, &err);
    if (!g_q || err != CL_SUCCESS) return false;

    const char* src = R"CLC(
        __kernel void absdiff(__global const float* obs, float ego, __global float* out) {
            int i = get_global_id(0);
            float d = obs[i] - ego;
            out[i] = d < 0 ? -d : d;
        }
    )CLC";

    g_prog = clCreateProgramWithSource(g_ctx, 1, &src, nullptr, &err);
    if (!g_prog || err != CL_SUCCESS) return false;

    err = clBuildProgram(g_prog, 0, nullptr, nullptr, nullptr, nullptr);
    if (err != CL_SUCCESS) return false;

    g_kernel = clCreateKernel(g_prog, "absdiff", &err);
    if (!g_kernel || err != CL_SUCCESS) return false;

    g_clInitOk = true;
    return true;
}

static float gpuMinAbsDistance(const std::vector<float>& obstacles, float egoPos) {
    if (obstacles.empty()) return 1e9f;
    if (!initOpenCLOnce()) return 1e9f;

    cl_int err = CL_SUCCESS;
    const size_t n = obstacles.size();
    const size_t bytes = n * sizeof(float);

    cl_mem bufObs = clCreateBuffer(g_ctx, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, bytes,
                                   const_cast<float*>(obstacles.data()), &err);
    if (!bufObs || err != CL_SUCCESS) return 1e9f;

    cl_mem bufOut = clCreateBuffer(g_ctx, CL_MEM_WRITE_ONLY, bytes, nullptr, &err);
    if (!bufOut || err != CL_SUCCESS) { clReleaseMemObject(bufObs); return 1e9f; }

    err  = clSetKernelArg(g_kernel, 0, sizeof(cl_mem), &bufObs);
    err |= clSetKernelArg(g_kernel, 1, sizeof(float), &egoPos);
    err |= clSetKernelArg(g_kernel, 2, sizeof(cl_mem), &bufOut);
    if (err != CL_SUCCESS) { clReleaseMemObject(bufObs); clReleaseMemObject(bufOut); return 1e9f; }

    size_t global = n;
    err = clEnqueueNDRangeKernel(g_q, g_kernel, 1, nullptr, &global, nullptr, 0, nullptr, nullptr);
    if (err != CL_SUCCESS) { clReleaseMemObject(bufObs); clReleaseMemObject(bufOut); return 1e9f; }

    std::vector<float> out(n);
    err = clEnqueueReadBuffer(g_q, bufOut, CL_TRUE, 0, bytes, out.data(), 0, nullptr, nullptr);
    clReleaseMemObject(bufObs);
    clReleaseMemObject(bufOut);
    if (err != CL_SUCCESS) return 1e9f;

    float mn = out[0];
    for (size_t i = 1; i < n; ++i) if (out[i] < mn) mn = out[i];
    return mn;
}
#endif

// Cut-in simulation: pause heartbeat for this long
constexpr std::int64_t CUTIN_PAUSE_MS = 3000;

// Terminal strings
static const char* colorMode(FollowerMode m) {
    switch (m) {
        case FollowerMode::COUPLED:            return "\033[1;32mCOUPLED\033[0m";
        case FollowerMode::STOPPING_FOR_LIGHT: return "\033[1;34mSTOPPING\033[0m";
        case FollowerMode::DECOUPLED:          return "\033[1;35mDECOUPLED\033[0m";
        case FollowerMode::CATCH_UP:           return "\033[1;33mCATCH_UP\033[0m";
        default:                               return "UNKNOWN";
    }
}
static const char* colorLight(TrafficLight l) {
    switch (l) {
        case TrafficLight::RED:   return "\033[1;31mRED\033[0m";
        case TrafficLight::GREEN: return "\033[1;32mGREEN\033[0m";
        default:                  return "NONE";
    }
}
}

// -------- terminal helpers (non-canonical single-key input) ----------
static struct termios g_oldTermios;

static void setNonCanonicalMode() {
    struct termios newTermios{};
    tcgetattr(STDIN_FILENO, &g_oldTermios);
    newTermios = g_oldTermios;
    newTermios.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newTermios);
}
static void restoreTerminalSettings() {
    tcsetattr(STDIN_FILENO, TCSANOW, &g_oldTermios);
}
static bool isKeyPressed() {
    fd_set set;
    struct timeval timeout{};
    FD_ZERO(&set);
    FD_SET(STDIN_FILENO, &set);
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;
    return select(STDIN_FILENO + 1, &set, nullptr, nullptr, &timeout) > 0;
}

// ------------------------------ class --------------------------------

FollowingVehicle::FollowingVehicle(int id,
                                   double initialSpeed,
                                   double initialPosition,
                                   double targetDistance,
                                   double Kp,
                                   double Ki,
                                   double Kd)
    : id_(id),
      position_(initialPosition),
      speed_(initialSpeed),
      socketFd_(-1),
      targetDistance_(targetDistance),
      Kp_(Kp),
      Ki_(Ki),
      Kd_(Kd),
      integralError_(0.0),
      previousError_(0.0),
      running_(false),
      connected_(false),
      obstacleDetected_(false),
      cutInActive_(false),
      mode_(static_cast<std::uint8_t>(FollowerMode::COUPLED)) {
    pthread_mutex_init(&stateMutex_, nullptr);
    initClock();
}

FollowingVehicle::~FollowingVehicle() {
    stop();
    pthread_mutex_destroy(&stateMutex_);
}

std::int64_t FollowingVehicle::nowMs() {
    using namespace std::chrono;
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

void FollowingVehicle::initClock() {
    std::memset(clock_, 0, sizeof(clock_));
    if (selfIndex_ >= 0 && selfIndex_ < MAX_VEHICLES) {
        clock_[selfIndex_][selfIndex_] = 1;
    }
}

void FollowingVehicle::mergeClockElementwiseMax(const std::int32_t other[MAX_VEHICLES][MAX_VEHICLES]) {
    for (int i = 0; i < MAX_VEHICLES; ++i) {
        for (int j = 0; j < MAX_VEHICLES; ++j) {
            if (other[i][j] > clock_[i][j]) clock_[i][j] = other[i][j];
        }
    }
}

void FollowingVehicle::onClockReceive(int senderIdx, const std::int32_t other[MAX_VEHICLES][MAX_VEHICLES]) {
    const int selfIdx = selfIndex_;
    if (selfIdx < 0 || selfIdx >= MAX_VEHICLES) {
        mergeClockElementwiseMax(other);
        return;
    }

    mergeClockElementwiseMax(other);

    if (senderIdx >= 0 && senderIdx < MAX_VEHICLES) {
        for (int j = 0; j < MAX_VEHICLES; ++j) {
            if (other[senderIdx][j] > clock_[selfIdx][j]) clock_[selfIdx][j] = other[senderIdx][j];
        }

        const int senderTime = other[senderIdx][senderIdx];
        if (senderTime + 1 > clock_[selfIdx][selfIdx]) clock_[selfIdx][selfIdx] = senderTime + 1;
        else clock_[selfIdx][selfIdx] += 1;
    } else {
        clock_[selfIdx][selfIdx] += 1;
    }
}

void FollowingVehicle::onClockLocalEvent() {
    const int selfIdx = selfIndex_;
    if (selfIdx < 0 || selfIdx >= MAX_VEHICLES) return;
    clock_[selfIdx][selfIdx] += 1;
}

void FollowingVehicle::printClock(int numVehicles) {
    std::cout << "Clock Matrix (" << numVehicles << " vehicles):\n";
    for (int i = 0; i < numVehicles; ++i) {
        for (int j = 0; j < numVehicles; ++j) {
            std::cout << clock_[i][j] << (j + 1 == numVehicles ? "" : " ");
        }
        std::cout << "\n";
    }
}

void FollowingVehicle::connectToLeader(const std::string& ipAddress) {
    leaderIp_ = ipAddress;
    socketFd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (socketFd_ < 0) {
        throw std::runtime_error("Follower: failed to create socket");
    }

    sockaddr_in serverAddr{};
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(PORT);
    if (::inet_pton(AF_INET, ipAddress.c_str(), &serverAddr.sin_addr) <= 0) {
        throw std::runtime_error("Follower: invalid IP address");
    }

    if (::connect(socketFd_, reinterpret_cast<sockaddr*>(&serverAddr), sizeof(serverAddr)) < 0) {
        ::close(socketFd_);
        socketFd_ = -1;
        throw std::runtime_error("Follower: communication failure (not connected to leader)");
    }

    connected_.store(true);
    std::cout << "\033[1;32mConnected to the leading vehicle!\033[0m\n";

    // Send JOIN
    WireMessage join{};
    join.type = static_cast<std::uint8_t>(MsgType::JOIN);
    join.senderId = id_;
    join.senderIndex = -1;
    join.assignedIndex = -1;
    join.position = position_;
    join.speed = speed_;
    join.obstacle = 0;
    join.flags = FLAG_CONNECTED;

    join.trafficLight = static_cast<std::uint8_t>(TrafficLight::NONE);
    join.stopLinePos = 0.0;
    join.followerMode = static_cast<std::uint8_t>(FollowerMode::COUPLED);

    std::memcpy(join.clockMatrix, clock_, sizeof(clock_));
    sendAll(socketFd_, &join, sizeof(join));

    // Receive JOIN_ACK
    WireMessage ack{};
    if (!recvAll(socketFd_, &ack, sizeof(ack)) ||
        static_cast<MsgType>(ack.type) != MsgType::JOIN_ACK) {
        throw std::runtime_error("Follower: did not receive JOIN_ACK from leader");
    }

    selfIndex_ = ack.assignedIndex;
    initClock();

    std::cout << "\033[1;36mFollower " << id_
              << " joined platoon. Assigned clockIndex=" << selfIndex_
              << "\033[0m\n";
}

void FollowingVehicle::run() {
    running_.store(true);
    setNonCanonicalMode();

    pthread_create(&recvThread_, nullptr, &FollowingVehicle::recvThreadEntry, this);
    pthread_create(&sendThread_, nullptr, &FollowingVehicle::sendThreadEntry, this);
    pthread_create(&inputThread_, nullptr, &FollowingVehicle::inputThreadEntry, this);

    pthread_join(inputThread_, nullptr);
    pthread_join(recvThread_, nullptr);
    pthread_join(sendThread_, nullptr);

    restoreTerminalSettings();
}

void FollowingVehicle::stop() {
    if (!running_.exchange(false)) return;

    connected_.store(false);
    if (socketFd_ >= 0) {
        ::shutdown(socketFd_, SHUT_RDWR);
        ::close(socketFd_);
        socketFd_ = -1;
    }
}

void* FollowingVehicle::recvThreadEntry(void* arg) {
    static_cast<FollowingVehicle*>(arg)->recvLoop();
    return nullptr;
}
void* FollowingVehicle::sendThreadEntry(void* arg) {
    static_cast<FollowingVehicle*>(arg)->sendLoop();
    return nullptr;
}
void* FollowingVehicle::inputThreadEntry(void* arg) {
    static_cast<FollowingVehicle*>(arg)->inputLoop();
    return nullptr;
}

void FollowingVehicle::recvLoop() {
    while (running_.load()) {
        if (!connected_.load()) {
            if (!tryReconnect()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                continue;
            }
        }

        WireMessage msg{};
        if (!recvAll(socketFd_, &msg, sizeof(msg))) {
            std::cerr << "\033[1;31mFollower: lost connection to leader.\033[0m\n";
            connected_.store(false);
            continue;
        }

        if (static_cast<MsgType>(msg.type) != MsgType::LEADER_STATE) continue;

        pthread_mutex_lock(&stateMutex_);
        onClockReceive(msg.senderIndex, msg.clockMatrix);

        const bool leaderObstacle = (msg.obstacle != 0);
        const bool degraded = (msg.flags & FLAG_DEGRADED) != 0;

        const auto light = static_cast<TrafficLight>(msg.trafficLight);
        const double stopLine = msg.stopLinePos;

        updateControl(msg.position, msg.speed, leaderObstacle, degraded, light, stopLine);

        auto m = static_cast<FollowerMode>(mode_.load());

        std::cout << "\n\033[1;36m---------------- FOLLOWER ----------------\033[0m\n";
        std::cout << "ID: " << id_
                  << "  Mode: " << colorMode(m)
                  << "  Light: " << colorLight(light)
                  << "  StopLine: " << stopLine << "m\n";
        std::cout << "Position: " << position_
                  << "  Speed: " << speed_ << "\n";

        printClock(msg.numVehicles);
        std::cout << std::flush;

        pthread_mutex_unlock(&stateMutex_);
    }
}

void FollowingVehicle::sendLoop() {
    auto cutInStart = std::int64_t{0};

    while (running_.load()) {
        if (!connected_.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(HEARTBEAT_PERIOD_MS));
            continue;
        }

        // Cut-in simulation: pause heartbeat
        if (cutInActive_.load()) {
            if (cutInStart == 0) {
                cutInStart = nowMs();
                std::cout << "\n\033[1;33m[CUT-IN] Pausing heartbeat for "
                          << (CUTIN_PAUSE_MS / 1000) << "s...\033[0m\n";
            }
            if (nowMs() - cutInStart < CUTIN_PAUSE_MS) {
                std::this_thread::sleep_for(std::chrono::milliseconds(HEARTBEAT_PERIOD_MS));
                continue;
            }
            cutInActive_.store(false);
            cutInStart = 0;
            std::cout << "\033[1;33m[CUT-IN] Communication resumed.\033[0m\n";
        }

#ifdef USE_OPENCL
        if (!obstacleDetected_.load()) {
            std::vector<float> obs(2048);
            float base = static_cast<float>(position_);
            for (size_t i = 0; i < obs.size(); ++i) {
                obs[i] = base + static_cast<float>((i * 37u) % 200u);
            }
            const float mn = gpuMinAbsDistance(obs, static_cast<float>(position_));
            if (mn < 8.0f) {
                obstacleDetected_.store(true);
                std::cout << "\n\033[1;31m[FOLLOWER] GPU obstacle trigger (minDist="
                          << mn << ").\033[0m\n";
            }
        }
#endif

        pthread_mutex_lock(&stateMutex_);
        onClockLocalEvent();

        WireMessage out{};
        out.type = static_cast<std::uint8_t>(MsgType::FOLLOWER_STATE);
        out.senderId = id_;
        out.senderIndex = selfIndex_;
        out.assignedIndex = -1;
        out.position = position_;
        out.speed = speed_;
        out.obstacle = obstacleDetected_.load() ? 1 : 0;
        out.flags = FLAG_CONNECTED;

        // follower does not command traffic light
        out.trafficLight = static_cast<std::uint8_t>(TrafficLight::NONE);
        out.stopLinePos = 0.0;
        out.followerMode = mode_.load();

        std::memcpy(out.clockMatrix, clock_, sizeof(clock_));
        pthread_mutex_unlock(&stateMutex_);

        if (!sendAll(socketFd_, &out, sizeof(out))) {
            std::cerr << "\033[1;31mFollower: send failed (connection lost).\033[0m\n";
            connected_.store(false);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(HEARTBEAT_PERIOD_MS));
    }
}

void FollowingVehicle::inputLoop() {
    std::cout << "\033[1;36mControls:\033[0m "
              << "[o]=obstacle  [c]=cut-in  [q]=quit\n";

    while (running_.load()) {
        if (!isKeyPressed()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }
        char ch = static_cast<char>(::getchar());

        if (ch == 'o' || ch == 'O') {
            obstacleDetected_.store(true);
            std::cout << "\n\033[1;31m[FOLLOWER] Obstacle detected -> notifying leader.\033[0m\n";
        } else if (ch == 'c' || ch == 'C') {
            cutInActive_.store(true);
        } else if (ch == 'q' || ch == 'Q') {
            // Graceful leave
            if (connected_.load()) {
                WireMessage leave{};
                leave.type = static_cast<std::uint8_t>(MsgType::LEAVE);
                leave.senderId = id_;
                leave.senderIndex = selfIndex_;
                leave.position = position_;
                leave.speed = speed_;
                leave.obstacle = obstacleDetected_.load() ? 1 : 0;
                leave.flags = 0;

                leave.trafficLight = static_cast<std::uint8_t>(TrafficLight::NONE);
                leave.stopLinePos = 0.0;
                leave.followerMode = mode_.load();

                std::memcpy(leave.clockMatrix, clock_, sizeof(clock_));
                sendAll(socketFd_, &leave, sizeof(leave));
            }
            std::cout << "\n\033[1;33m[FOLLOWER] Leaving platoon. Bye!\033[0m\n";
            stop();
            return;
        }
    }
}

// -------------------- traffic-light + left-behind use case --------------------
void FollowingVehicle::updateControl(double leaderPos,
                                     double leaderSpeed,
                                     bool leaderObstacle,
                                     bool degraded,
                                     TrafficLight light,
                                     double stopLinePos) {
    // Safety: if leader reports obstacle -> slow to stop.
    if (leaderObstacle) {
        speed_ *= 0.7;
        if (speed_ < 0.05) speed_ = 0.0;
        return;
    }

    // If degraded: be conservative (slow a bit)
    if (degraded) leaderSpeed *= 0.9;

    auto mode = static_cast<FollowerMode>(mode_.load());

    const double PASS_MARGIN = 2.0;
    const bool leaderPassed = (leaderPos > stopLinePos + PASS_MARGIN);
    const bool iAmBeforeLine = (position_ < stopLinePos - 0.5);

    // Left-behind detection:
    // If leader passed but we are still before stop line while light is RED -> DECOUPLED.
    if (light == TrafficLight::RED && leaderPassed && iAmBeforeLine) {
        if (mode != FollowerMode::DECOUPLED) {
            mode_.store(static_cast<std::uint8_t>(FollowerMode::DECOUPLED));
            std::cout << "\n\033[1;35m[FOLLOWER " << id_
                      << "] LEFT-BEHIND -> DECOUPLED\033[0m\n";
        }
        mode = FollowerMode::DECOUPLED;
    }

    // If DECOUPLED and light becomes GREEN -> start CATCH_UP
    if (mode == FollowerMode::DECOUPLED && light == TrafficLight::GREEN) {
        mode_.store(static_cast<std::uint8_t>(FollowerMode::CATCH_UP));
        std::cout << "\n\033[1;33m[FOLLOWER " << id_
                  << "] GREEN -> CATCH_UP\033[0m\n";
        mode = FollowerMode::CATCH_UP;
    }

    // If RED and leader not yet passed: stop before stop line, keep a safe gap.
    if (light == TrafficLight::RED && !leaderPassed) {
        mode_.store(static_cast<std::uint8_t>(FollowerMode::STOPPING_FOR_LIGHT));

        // Stop target before the line so we keep a safe distance.
        const double stopTarget = stopLinePos - targetDistance_;
        const double distToStop = stopTarget - position_;

        if (distToStop <= 0.3) {
            speed_ = 0.0;
        } else {
            double desired = std::min(leaderSpeed, 12.0);
            if (distToStop < 10.0) desired = std::min(desired, 6.0);
            if (distToStop < 3.0) desired = 0.0;

            speed_ += 0.25 * (desired - speed_);
            if (speed_ < 0.05) speed_ = 0.0;
        }

        position_ += speed_ * (HEARTBEAT_PERIOD_MS / 1000.0);
        return;
    }

    // DECOUPLED: stay stopped (waiting for GREEN)
    if (mode == FollowerMode::DECOUPLED) {
        speed_ = 0.0;
        return;
    }

    // CATCH_UP: go faster than leader until target distance is recovered, then rejoin.
    if (mode == FollowerMode::CATCH_UP) {
        const double gap = leaderPos - position_;
        const double error = gap - targetDistance_;

        double targetSpeed = leaderSpeed + 25.0; // boost above leader
        if (targetSpeed > leaderSpeed + 30.0) targetSpeed = leaderSpeed + 30.0;
        if (targetSpeed < 0.0) targetSpeed = 0.0;

        speed_ += 0.3 * (targetSpeed - speed_);
        position_ += speed_ * (HEARTBEAT_PERIOD_MS / 1000.0);

        if (std::abs(error) <= 1.0) {
            mode_.store(static_cast<std::uint8_t>(FollowerMode::COUPLED));
            std::cout << "\n\033[1;32m[FOLLOWER " << id_
                      << "] Rejoined -> COUPLED\033[0m\n";
        }
        return;
    }

    // Normal COUPLED following (PID-ish control)
    mode_.store(static_cast<std::uint8_t>(FollowerMode::COUPLED));

    const double gap = leaderPos - position_;
    const double error = gap - targetDistance_;
    integralError_ += error;
    const double derivative = error - previousError_;
    previousError_ = error;

    const double control = Kp_ * error + Ki_ * integralError_ + Kd_ * derivative;

    double targetSpeed = leaderSpeed + control;
    if (targetSpeed < 0.0) targetSpeed = 0.0;
    if (targetSpeed > leaderSpeed + 20.0) targetSpeed = leaderSpeed + 20.0;

    speed_ += 0.2 * (targetSpeed - speed_);
    if (std::abs(speed_) < 1e-6) speed_ = 0.0;

    position_ += speed_ * (HEARTBEAT_PERIOD_MS / 1000.0);
}

// -------------------- socket helpers --------------------

bool FollowingVehicle::sendAll(int fd, const void* data, size_t len) {
    const std::uint8_t* p = static_cast<const std::uint8_t*>(data);
    size_t sent = 0;
    while (sent < len) {
        ssize_t n = ::send(fd, p + sent, len - sent, 0);
        if (n <= 0) return false;
        sent += static_cast<size_t>(n);
    }
    return true;
}

bool FollowingVehicle::recvAll(int fd, void* data, size_t len) {
    std::uint8_t* p = static_cast<std::uint8_t*>(data);
    size_t recvd = 0;
    while (recvd < len) {
        ssize_t n = ::recv(fd, p + recvd, len - recvd, 0);
        if (n <= 0) return false;
        recvd += static_cast<size_t>(n);
    }
    return true;
}

bool FollowingVehicle::tryReconnect() {
    if (socketFd_ >= 0) {
        ::close(socketFd_);
        socketFd_ = -1;
    }

    try {
        connectToLeader(leaderIp_.empty() ? "127.0.0.1" : leaderIp_);
        return true;
    } catch (...) {
        return false;
    }
}

// ---------------------------
// Program entry point
// ---------------------------
int main(int argc, char** argv) {
    int id = 2;
    double initialSpeed = 0.0;
    double initialPos = 10.0;
    double targetDistance = 10.0;
    std::string ip = "127.0.0.1";

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--id" && i + 1 < argc) {
            id = std::stoi(argv[++i]);
        } else if (arg == "--initspeed" && i + 1 < argc) {
            initialSpeed = std::stod(argv[++i]);
        } else if (arg == "--initposition" && i + 1 < argc) {
            initialPos = std::stod(argv[++i]);
        } else if (arg == "--distance" && i + 1 < argc) {
            targetDistance = std::stod(argv[++i]);
        } else if (arg == "--ip" && i + 1 < argc) {
            ip = argv[++i];
        } else {
            std::cerr << "Unknown argument: " << arg << "\n";
            std::cerr << "Usage: ./follow [--id <id>] [--initspeed <speed>] "
                         "[--initposition <pos>] [--distance <dist>] [--ip <ip>]\n";
            return 1;
        }
    }

    // Control params (tuned lightly for stability)
    double Kp = 0.05, Ki = 0.0005, Kd = 0.02;

    try {
        FollowingVehicle follower(id, initialSpeed, initialPos, targetDistance, Kp, Ki, Kd);
        follower.connectToLeader(ip);
        follower.run();
    } catch (const std::exception& ex) {
        std::cerr << "Follower error: " << ex.what() << "\n";
        return 1;
    }
    return 0;
}
