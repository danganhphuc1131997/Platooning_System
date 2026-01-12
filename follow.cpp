#include "follow.h"

#include <chrono>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <netinet/in.h>
#include <sys/socket.h>
#include <termios.h>
#include <thread>
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

#ifdef USE_OPENCL
#include <CL/cl.h>
#include <vector>
#endif

namespace {
constexpr std::int64_t HEARTBEAT_PERIOD_MS = 500;
constexpr std::int64_t RECV_TIMEOUT_MS     = 5000; // for logging (UDP non-blocking)
constexpr int DISPLAY_UPDATE_INTERVAL = 3; // update dashboard every N receives

#ifdef USE_OPENCL
// Very small GPU demo (OpenCL):
// Compute the minimum absolute distance between the ego position and a set of obstacle positions.
// This is intentionally simple so it can be explained in a report as "GPU offload of a vector loop".
// Build example:
//   g++ -DUSE_OPENCL follow.cpp lead.cpp -lpthread -lOpenCL -o platoon
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

    // Try GPU first, then CPU as a fallback.
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
      cutInActive_(false) {
    pthread_mutex_init(&stateMutex_, nullptr);
    initClock();
}

FollowingVehicle::~FollowingVehicle() {
    stop();
    pthread_mutex_destroy(&stateMutex_);
}

std::int64_t FollowingVehicle::nowMs() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

void FollowingVehicle::initClock() {
    std::memset(clock_, 0, sizeof(clock_));
    // Once we know our assigned matrix index, we start our local time at 1.
    if (selfIndex_ >= 0 && selfIndex_ < MAX_NODES) {
        clock_[selfIndex_][selfIndex_] = 1;
    }
}

void FollowingVehicle::mergeClockElementwiseMax(const std::int32_t other[MAX_NODES][MAX_NODES]) {
    for (int i = 0; i < MAX_NODES; ++i) {
        for (int j = 0; j < MAX_NODES; ++j) {
            if (other[i][j] > clock_[i][j]) clock_[i][j] = other[i][j];
        }
    }
}

void FollowingVehicle::onClockReceive(int senderIdx, const std::int32_t other[MAX_NODES][MAX_NODES]) {
    const int selfIdx = selfIndex_;
    if (selfIdx < 0 || selfIdx >= MAX_NODES) {
        // Not yet assigned; do a safe merge.
        mergeClockElementwiseMax(other);
        return;
    }

    mergeClockElementwiseMax(other);

    if (senderIdx >= 0 && senderIdx < MAX_NODES) {
        for (int j = 0; j < MAX_NODES; ++j) {
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
    if (selfIdx < 0 || selfIdx >= MAX_NODES) return;
    clock_[selfIdx][selfIdx] += 1;
}

void FollowingVehicle::printClock() {
    // Compact clock display - only show our row
    std::cout << CYAN << "Clock[" << selfIndex_ << "]: [";
    for (int j = 0; j < MAX_NODES; ++j) {
        std::cout << clock_[selfIndex_][j] << (j + 1 == MAX_NODES ? "" : ",");
    }
    std::cout << "]" << RESET << std::endl;
}

void FollowingVehicle::printDashboard(double leaderPos, double leaderSpeed, bool degraded) {
    std::cout << CLEAR;  // Clear screen
    std::cout << BOLD << "╔════════════════════════════════════════════════════════╗" << RESET << std::endl;
    std::cout << BOLD << "║" << BLUE << "          🚙 FOLLOWER VEHICLE #" << id_ << " DASHBOARD            " << RESET << BOLD << "║" << RESET << std::endl;
    std::cout << BOLD << "╠════════════════════════════════════════════════════════╣" << RESET << std::endl;
    
    // Status
    std::ostringstream status;
    if (obstacleDetected_.load()) {
        status << RED << "⛔ OBSTACLE DETECTED" << RESET;
    } else if (cutInActive_.load()) {
        status << YELLOW << "⚠️  CUT-IN SIMULATION (no heartbeat)" << RESET;
    } else if (!connected_.load()) {
        status << RED << "✗ DISCONNECTED" << RESET;
    } else if (degraded) {
        status << YELLOW << "⚠️  LEADER IN DEGRADED MODE" << RESET;
    } else {
        status << GREEN << "✓ FOLLOWING" << RESET;
    }
    std::cout << BOLD << "║" << RESET << " Status: " << std::left << std::setw(45) << status.str() << BOLD << "║" << RESET << std::endl;
    
    // My Position & Speed
    std::cout << BOLD << "║" << RESET << " My Position: " << CYAN << std::fixed << std::setprecision(1) << std::setw(8) << position_ << RESET 
              << " m   Speed: " << CYAN << std::setw(6) << speed_ << RESET << " m/s      " << BOLD << "║" << RESET << std::endl;
    
    // Leader info
    double gap = leaderPos - position_;
    std::string gapColor = (gap > targetDistance_ - 2 && gap < targetDistance_ + 2) ? GREEN : YELLOW;
    std::cout << BOLD << "║" << RESET << " Leader Pos:  " << CYAN << std::setprecision(1) << std::setw(8) << leaderPos << RESET 
              << " m   Speed: " << CYAN << std::setw(6) << leaderSpeed << RESET << " m/s      " << BOLD << "║" << RESET << std::endl;
    std::cout << BOLD << "║" << RESET << " Gap: " << gapColor << std::setprecision(1) << gap << " m" << RESET 
              << " (target: " << targetDistance_ << " m)                      " << BOLD << "║" << RESET << std::endl;
    
    std::cout << BOLD << "╠════════════════════════════════════════════════════════╣" << RESET << std::endl;
    printClock();
    std::cout << BOLD << "╠════════════════════════════════════════════════════════╣" << RESET << std::endl;
    std::cout << BOLD << "║" << RESET << " Controls: [o]=obstacle  [c]=cut-in  [q]=quit         " << BOLD << "║" << RESET << std::endl;
    std::cout << BOLD << "╚════════════════════════════════════════════════════════╝" << RESET << std::endl;
}

void FollowingVehicle::connectToLeader(const std::string& ipAddress) {
    leaderIp_ = ipAddress;
    socketFd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (socketFd_ < 0) {
        throw std::runtime_error("Follower: failed to create UDP socket");
    }

    // Bind to ephemeral port so leader can reply
    sockaddr_in localAddr{};
    localAddr.sin_family = AF_INET;
    localAddr.sin_addr.s_addr = INADDR_ANY;
    localAddr.sin_port = 0;
    if (::bind(socketFd_, reinterpret_cast<sockaddr*>(&localAddr), sizeof(localAddr)) < 0) {
        ::close(socketFd_);
        socketFd_ = -1;
        throw std::runtime_error("Follower: bind failed");
    }

    std::memset(&leaderAddr_, 0, sizeof(leaderAddr_));
    leaderAddr_.sin_family = AF_INET;
    leaderAddr_.sin_port = htons(PORT);
    if (::inet_pton(AF_INET, ipAddress.c_str(), &leaderAddr_.sin_addr) <= 0) {
        ::close(socketFd_);
        socketFd_ = -1;
        throw std::runtime_error("Follower: invalid IP address");
    }

    // Set recv timeout for JOIN_ACK
    struct timeval tv{};
    tv.tv_sec = 2;
    tv.tv_usec = 0;
    ::setsockopt(socketFd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    // Send JOIN message
    WireMessage join{};
    join.type = static_cast<std::uint8_t>(MsgType::JOIN);
    join.senderId = id_;
    join.senderIndex = -1;
    join.assignedIndex = -1;
    join.position = position_;
    join.speed = speed_;
    join.obstacle = 0;
    join.flags = FLAG_CONNECTED;
    std::memcpy(join.clock, clock_, sizeof(clock_));

    if (!sendMsgToLeader(&join, sizeof(join))) {
        ::close(socketFd_);
        socketFd_ = -1;
        throw std::runtime_error("Follower: failed to send JOIN");
    }

    // Wait for JOIN_ACK
    WireMessage ack{};
    sockaddr_in from{};
    if (!recvMsgFromLeader(&ack, sizeof(ack), &from) ||
        static_cast<MsgType>(ack.type) != MsgType::JOIN_ACK) {
        ::close(socketFd_);
        socketFd_ = -1;
        throw std::runtime_error("Follower: did not receive JOIN_ACK from leader");
    }

    selfIndex_ = ack.assignedIndex;
    initClock();

    connected_.store(true);
    std::cout << "Connected to the leading vehicle! Assigned clockIndex=" << selfIndex_ << std::endl;

    // Remove recv timeout for normal operation
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    ::setsockopt(socketFd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
}

void FollowingVehicle::run() {
    running_.store(true);

    setNonCanonicalMode();

    pthread_create(&recvThread_, nullptr, &FollowingVehicle::recvThreadEntry, this);
    pthread_create(&sendThread_, nullptr, &FollowingVehicle::sendThreadEntry, this);
    pthread_create(&inputThread_, nullptr, &FollowingVehicle::inputThreadEntry, this);

    // Wait for threads
    pthread_join(inputThread_, nullptr);
    // input thread triggers stop()
    pthread_join(recvThread_, nullptr);
    pthread_join(sendThread_, nullptr);

    restoreTerminalSettings();
}

void FollowingVehicle::stop() {
    if (!running_.exchange(false)) {
        // already stopped
        return;
    }

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
    int displayCounter = 0;
    while (running_.load()) {
        if (!connected_.load()) {
            // Try reconnect if we were disconnected
            if (!tryReconnect()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                continue;
            }
        }

        WireMessage msg{};
        sockaddr_in from{};
        if (!recvMsgFromLeader(&msg, sizeof(msg), &from)) {
            std::cerr << RED << "[!] Lost connection to leader (UDP recv timeout)." << RESET << std::endl;
            connected_.store(false);
            continue;
        }

        if (static_cast<MsgType>(msg.type) != MsgType::LEADER_STATE) {
            continue;
        }

        pthread_mutex_lock(&stateMutex_);
        onClockReceive(msg.senderIndex, msg.clock);

        const bool leaderObstacle = (msg.obstacle != 0);
        const bool degraded = (msg.flags & FLAG_DEGRADED) != 0;
        updateControl(msg.position, msg.speed, leaderObstacle, degraded);

        // Print dashboard every N iterations
        if (++displayCounter >= DISPLAY_UPDATE_INTERVAL) {
            displayCounter = 0;
            printDashboard(msg.position, msg.speed, degraded);
        }
        pthread_mutex_unlock(&stateMutex_);
    }
}

void FollowingVehicle::sendLoop() {
    auto last = nowMs();
    auto cutInStart = std::int64_t{0};

    while (running_.load()) {
        if (!connected_.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(HEARTBEAT_PERIOD_MS));
            continue;
        }

        // Cut-in simulation: pause heartbeat for CUTIN_PAUSE_MS
        if (cutInActive_.load()) {
            if (cutInStart == 0) {
                cutInStart = nowMs();
                std::cout << "[CUT-IN] Simulating temporary communication loss for "
                          << (CUTIN_PAUSE_MS / 1000) << "s..." << std::endl;
            }
            if (nowMs() - cutInStart < CUTIN_PAUSE_MS) {
                std::this_thread::sleep_for(std::chrono::milliseconds(HEARTBEAT_PERIOD_MS));
                continue;
            }
            // End cut-in
            cutInActive_.store(false);
            cutInStart = 0;
            std::cout << "[CUT-IN] Communication resumed." << std::endl;
        }

        // Build follower heartbeat/state message
#ifdef USE_OPENCL
        // Optional GPU path: compute minimum obstacle distance using OpenCL.
        // Only auto-sets obstacleDetected_ if it isn't already set by the user (key 'o').
        if (!obstacleDetected_.load()) {
            std::vector<float> obs(2048);
            // simple deterministic pseudo-random-ish obstacles ahead of us
            float base = static_cast<float>(position_);
            for (size_t i = 0; i < obs.size(); ++i) {
                obs[i] = base + static_cast<float>((i * 37u) % 200u); // 0..199m ahead
            }
            const float mn = gpuMinAbsDistance(obs, static_cast<float>(position_));
            if (mn < 8.0f) {
                obstacleDetected_.store(true);
                std::cout << "[FOLLOWER] GPU obstacle trigger (minDist=" << mn << ")." << std::endl;
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
        std::memcpy(out.clock, clock_, sizeof(clock_));
        pthread_mutex_unlock(&stateMutex_);

        if (!sendMsgToLeader(&out, sizeof(out))) {
            std::cerr << "Follower: send failed (leader not reachable)." << std::endl;
            connected_.store(false);
        }

        // Periodic send
        auto now = nowMs();
        auto dt = now - last;
        last = now;
        (void)dt;
        std::this_thread::sleep_for(std::chrono::milliseconds(HEARTBEAT_PERIOD_MS));
    }
}

void FollowingVehicle::inputLoop() {
    std::cout << "Controls: [o]=obstacle  [c]=cut-in (temporary comm loss)  [q]=quit" << std::endl;

    while (running_.load()) {
        if (!isKeyPressed()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }
        char ch = static_cast<char>(::getchar());
        if (ch == 'o' || ch == 'O') {
            obstacleDetected_.store(true);
            std::cout << "[FOLLOWER] Obstacle detected -> notifying leader." << std::endl;
        } else if (ch == 'c' || ch == 'C') {
            cutInActive_.store(true);
        } else if (ch == 'q' || ch == 'Q') {
            // Graceful leave
            if (connected_.load()) {
                WireMessage leave{};
                leave.type = static_cast<std::uint8_t>(MsgType::LEAVE);
                leave.senderId = id_;
                leave.senderIndex = selfIndex_;
                leave.assignedIndex = -1;
                leave.position = position_;
                leave.speed = speed_;
                leave.obstacle = obstacleDetected_.load() ? 1 : 0;
                leave.flags = 0;
                std::memcpy(leave.clock, clock_, sizeof(clock_));
                sendMsgToLeader(&leave, sizeof(leave));
            }
            std::cout << "[FOLLOWER] Leaving platoon. Bye!" << std::endl;
            stop();
            return;
        }
    }
}

void FollowingVehicle::updateControl(double leaderPos, double leaderSpeed, bool leaderObstacle, bool degraded) {
    // Calculate actual time delta
    static std::int64_t lastUpdateMs = 0;
    std::int64_t now = nowMs();
    double dtSec = (lastUpdateMs == 0) ? 0.0 : static_cast<double>(now - lastUpdateMs) / 1000.0;
    lastUpdateMs = now;
    
    // Clamp dt to avoid huge jumps
    if (dtSec > 1.0) dtSec = 0.3;
    if (dtSec < 0.01) dtSec = 0.0;

    // Safety: if leader reports obstacle -> slow to stop.
    if (leaderObstacle) {
        speed_ *= 0.7;
        if (speed_ < 0.05) speed_ = 0.0;
        position_ += speed_ * dtSec;
        return;
    }

    // If degraded: be conservative (slow down a bit)
    if (degraded) {
        leaderSpeed *= 0.9;
    }

    // PID-ish on spacing error
    const double gap = leaderPos - position_;
    const double error = gap - targetDistance_;
    integralError_ += error * dtSec;  // Scale integral by dt
    const double derivative = (dtSec > 0) ? (error - previousError_) / dtSec : 0.0;
    previousError_ = error;

    const double control = Kp_ * error + Ki_ * integralError_ + Kd_ * derivative;

    // Convert to speed adjustment
    double targetSpeed = leaderSpeed + control;
    if (targetSpeed < 0.0) targetSpeed = 0.0;
    if (targetSpeed > leaderSpeed + 20.0) targetSpeed = leaderSpeed + 20.0;

    // Smooth speed changes
    speed_ += 0.3 * (targetSpeed - speed_);
    if (std::abs(speed_) < 1e-6) speed_ = 0.0;

    // Update position using actual time delta
    position_ += speed_ * dtSec;
}

bool FollowingVehicle::sendMsgToLeader(const void* data, size_t len) {
    ssize_t n = ::sendto(socketFd_, data, len, 0,
                         reinterpret_cast<const sockaddr*>(&leaderAddr_), sizeof(leaderAddr_));
    return (n == static_cast<ssize_t>(len));
}

bool FollowingVehicle::recvMsgFromLeader(void* data, size_t len, sockaddr_in* from) {
    socklen_t fromLen = sizeof(sockaddr_in);
    ssize_t n = ::recvfrom(socketFd_, data, len, 0, reinterpret_cast<sockaddr*>(from), &fromLen);
    if (n <= 0) return false;
    return (static_cast<size_t>(n) == len);
}

bool FollowingVehicle::tryReconnect() {
    // Minimal reconnect logic: user can restart follower; we also auto-retry.
    // We assume leader is on localhost for reconnect.
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
    // Optional CLI: ./follow [id] [initialSpeed] [initialPosition]
    int id = 2;
    double initialSpeed = 0.0;
    double initialPos = 10.0;

    if (argc >= 2) id = std::stoi(argv[1]);
    if (argc >= 3) initialSpeed = std::stod(argv[2]);
    if (argc >= 4) initialPos = std::stod(argv[3]);

    // Control params (tuned lightly for stability)
    double targetDistance = 10.0;
    double Kp = 0.05, Ki = 0.0005, Kd = 0.02;

    try {
        FollowingVehicle follower(id, initialSpeed, initialPos, targetDistance, Kp, Ki, Kd);
        const std::string ip = (argc >= 5) ? std::string(argv[4]) : std::string("127.0.0.1");
        follower.connectToLeader(ip);
        follower.run();
    } catch (const std::exception& ex) {
        std::cerr << "Follower error: " << ex.what() << std::endl;
        return 1;
    }
    return 0;
}
