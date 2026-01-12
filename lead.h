/**
 * @file lead.h
 *
 * @brief Leading vehicle (platoon leader) simulation.
 *
 * Changes vs. original version:
 *  - Uses pthreads + mutex for concurrency (no OpenMP for sockets)
 *  - Uses fixed-size WireMessage (no std::vector in messages)
 *  - Uses UDP for communication
 *  - Adds heartbeat-based node failure detection and degraded-mode handling
 *  - Logical matrix clock maintained and merged on receive
 */

#ifndef LEAD_H
#define LEAD_H

#include "message.h"

#include <atomic>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include <arpa/inet.h>
#include <pthread.h>

static constexpr int PORT = 8080;

struct FollowerInfo {
    sockaddr_in addr{};       // follower IP:port for UDP
    int followerId{};
    int clockIndex{}; // 1..MAX_NODES-1
    double position{};
    double speed{};
    std::int64_t lastSeenMs{}; // monotonic ms timestamp
};

class LeadingVehicle {
public:
    LeadingVehicle(int id, double initialPosition, double initialSpeed);
    ~LeadingVehicle();

    void startServer();
    void stopServer();

private:
    // Core state
    int id_;
    double position_;
    double baseSpeed_;
    double currentSpeed_;
    bool obstacleDetected_;

    int serverSocket_;

    // Clock matrix and follower registry
    std::int32_t clock_[MAX_NODES][MAX_NODES]{};
    std::map<std::string, FollowerInfo> followers_; // key: "ip:port"
    std::map<int, int> idToClockIndex_;      // followerId -> index
    std::vector<int> freeClockIndices_;

    // Concurrency
    pthread_mutex_t mutex_;
    std::atomic<bool> running_;

    pthread_t recvThread_{};
    pthread_t broadcastThread_{};
    pthread_t monitorThread_{};

    // Helpers
    void createServerSocket();
    void initClock();
    void printClock();
    void printDashboard();
    void mergeClockElementwiseMax(const std::int32_t other[MAX_NODES][MAX_NODES]);
    void onClockReceive(int selfIdx, int senderIdx, const std::int32_t other[MAX_NODES][MAX_NODES]);
    void onClockLocalEvent(int selfIdx);

    // Threads
    static void* recvThreadEntry(void* arg);
    static void* broadcastThreadEntry(void* arg);
    static void* monitorThreadEntry(void* arg);

    void recvLoop();
    void broadcastLoop();
    void monitorLoop();

    // Messaging
    bool sendToFollower(const sockaddr_in& addr, const void* data, size_t len);
    WireMessage makeLeaderStateMessage(std::uint8_t flags);

    // Timing
    static std::int64_t nowMs();
    static std::string addrKey(const sockaddr_in& addr);

    // Cleanup
    void removeFollowerLocked(const std::string& key, const char* reason);
};

#endif // LEAD_H
