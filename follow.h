/**
 * @file follow.h
 *
 * @brief Following vehicle (platoon follower) simulation.
 */

#ifndef FOLLOW_H
#define FOLLOW_H

#include <atomic>
#include <cstdint>
#include <string>

#include <netinet/in.h>

#include <arpa/inet.h>
#include <pthread.h>
#include "vehicle.h"

enum FollowerState : std::uint8_t {
    NORMAL = 0,   // Normal following
    ERROR,        // Error state
    STOPPING,     // Braking to stop
    STOPPED,      // Stopped
    STARTING,     // Starting from stop,
    CATCHING_UP,   // Speeding up to catch leader
    STOPPING_FOR_RED_LIGHT // Stopping for red light
};
class FollowingVehicle {
public:
    FollowingVehicle(int id,
                     double initialSpeed,
                     double initialPosition);
    ~FollowingVehicle();

    void connectToLeader(); // Connect to the leader vehicle
    void startThreads(); // Start internal threads
    void setState(FollowerState newState);
    FollowerState getState() const;

private:
    VehicleInfo info_;
    FollowerState state_;
    std::atomic<bool> clientRunning_;

    PlatoonState platoonState_; // Current state of the platoon
    
    // Leader snapshot (updated by recv thread when leader STATUS_UPDATE received)
    double leaderPosition_{}; // latest leader position (meters)
    double leaderSpeed_{};    // latest leader speed (m/s)
    pthread_mutex_t leaderMutex_{}; // protects leaderPosition_/leaderSpeed_

    // Client socket
    int clientSocket_;
    struct sockaddr_in leaderAddr_;

    // Timing
    static std::int64_t nowMs();

    // Threads
    pthread_t recvThread_{};
    static void* recvThreadEntry(void* arg);            // Thread entry for receiving messages
    pthread_t runThread_{};
    static void* runThreadEntry(void* arg);             // Main follower logic thread
    pthread_t sendStatusThread_{};
    static void* sendStatusThreadEntry(void* arg);      // Thread entry for sending status to leader
    pthread_t displayThread_{};
    static void* displayThreadEntry(void* arg);         // Display status thread
    
    // Helpers
    void createClientSocket();
    void sendStatusToLeader();
    void sendCoupleCommandToLeader();
};

#endif // FOLLOW_H
