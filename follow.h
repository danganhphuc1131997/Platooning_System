/**
 * @file follow.h
 *
 * @brief Following vehicle (platoon follower) simulation.
 *
 * Changes vs. original version:
 *  - Uses pthreads + mutex (no OpenMP for sockets)
 *  - Uses fixed-size WireMessage for TCP communication
 *  - Adds Cut-in simulation: temporarily pause heartbeat to emulate partition
 *  - Safe mode when connection is lost; optional reconnect
 */

#ifndef FOLLOW_H
#define FOLLOW_H

#include "message.h"

#include <atomic>
#include <cstdint>
#include <string>

#include <arpa/inet.h>
#include <pthread.h>

static constexpr int PORT = 8080;

class FollowingVehicle {
public:
    FollowingVehicle(int id,
                     double initialSpeed,
                     double initialPosition,
                     double targetDistance,
                     double Kp,
                     double Ki,
                     double Kd);

    ~FollowingVehicle();

    void connectToLeader(const std::string& ipAddress);
    void run();
    void stop();

private:
    int id_;
    int selfIndex_{-1};
    std::string leaderIp_;

    double position_;
    double speed_;
    int socketFd_;

    // Controller
    double targetDistance_;
    double Kp_;
    double Ki_;
    double Kd_;
    double integralError_;
    double previousError_;

    // Flags
    std::atomic<bool> running_;
    std::atomic<bool> connected_;
    std::atomic<bool> obstacleDetected_;
    std::atomic<bool> cutInActive_;

    // Logical clock
    std::int32_t clock_[MAX_NODES][MAX_NODES]{};

    // Concurrency
    pthread_mutex_t stateMutex_;
    pthread_t recvThread_{};
    pthread_t sendThread_{};
    pthread_t inputThread_{};

    // Threads
    static void* recvThreadEntry(void* arg);
    static void* sendThreadEntry(void* arg);
    static void* inputThreadEntry(void* arg);

    void recvLoop();
    void sendLoop();
    void inputLoop();

    // Helpers
    void initClock();
    void mergeClockElementwiseMax(const std::int32_t other[MAX_NODES][MAX_NODES]);
    void onClockReceive(int senderIdx, const std::int32_t other[MAX_NODES][MAX_NODES]);
    void onClockLocalEvent();
    void printClock();

    void updateControl(double leaderPos, double leaderSpeed, bool leaderObstacle, bool degraded);

    bool sendAll(int fd, const void* data, size_t len);
    bool recvAll(int fd, void* data, size_t len);

    static std::int64_t nowMs();
    bool tryReconnect();
};

#endif // FOLLOW_H
