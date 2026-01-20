/**
 * @file lead.h
 *
 * @brief Leading vehicle (platoon leader) simulation.
 *
 * Changes vs. original version:
 *  - Uses pthreads + mutex for concurrency (no OpenMP for sockets)
 *  - Uses fixed-size WireMessage (no std::vector in messages)
 *  - Adds heartbeat-based node failure detection and degraded-mode handling
 *  - Logical matrix clock maintained and merged on receive
 *  - Adds traffic light use-case: leader detects light and broadcasts RED/GREEN + stop line
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
    sockaddr_in addr{};
    int followerId{};
    int clockIndex{}; // 1..MAX_VEHICLES-1
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

    // Traffic light simulation (use case)
    std::uint8_t trafficLight_;     // TrafficLight
    double stopLinePos_;            // fixed stop line position
    std::int64_t lightPhaseStartMs_;
    bool lightActive_;              // enabled once leader "detects" light

    // Clock matrix and follower registry
    std::int32_t clock_[MAX_VEHICLES][MAX_VEHICLES];
    std::map<int, FollowerInfo> followers_; // key: followerId
    std::map<int, int> vehicleIndex;        // vehicleId -> matrix index

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
    void mergeClockElementwiseMax(const std::int32_t other[MAX_VEHICLES][MAX_VEHICLES]);
    void onClockReceive(int selfIdx, int senderIdx, const std::int32_t other[MAX_VEHICLES][MAX_VEHICLES]);
    void onClockLocalEvent(int selfIdx);

    // Threads
    static void* recvThreadEntry(void* arg);
    static void* broadcastThreadEntry(void* arg);
    static void* monitorThreadEntry(void* arg);

    void recvLoop();
    void broadcastLoop();
    void monitorLoop();

    // Per-follower handler

    static void* followerHandlerEntry(void* arg);
    void handleFollower(int clientSocket);

    // Messaging
    bool sendAll(int fd, const void* data, size_t len);
    bool recvAll(int fd, void* data, size_t len);
    WireMessage makeLeaderStateMessage(std::uint8_t flags);

    // Timing
    static std::int64_t nowMs();

    // Cleanup
    void removeFollowerLocked(int clientSocket, const char* reason);
};

#endif // LEAD_H
