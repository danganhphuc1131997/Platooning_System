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
#include <queue>

#include <netinet/in.h>

#include <arpa/inet.h>
#include <pthread.h>
#include "vehicle.h"
#include "system_config.h"

enum FollowerState : std::uint8_t {
    NORMAL = 0,             // Normal following
    ERROR,                  // Error state
    STOPPING,               // Braking to stop
    STOPPED,                // Stopped
    STARTING,               // Starting from stop
    CATCHING_UP,            // Speeding up to catch leader
    STOPPING_FOR_RED_LIGHT, // Stopping for red light
    DECOUPLED,              // Temporarily decoupled from platoon (left behind)
    LOW_ENERGY,              // Low energy, reduced speed
    STOPPING_FOR_OBSTACLE   // Stopping for obstacle detected ahead
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
    VehicleInfo frontVehicleInfo_;
    VehicleInfo rearVehicleInfo_;

    FollowerState state_;
    std::atomic<bool> clientRunning_;

    bool energyAlertSent_; // Flag to send energy alert only once
    double targetSpeed_; // Target speed for gradual changes

    PlatoonState platoonState_; // Current state of the platoon
    
    // Leader snapshot (updated by recv thread when leader STATUS_UPDATE received)
    double leaderPosition_{}; // latest leader position (meters)
    double leaderSpeed_{};    // latest leader speed (m/s)
    std::int64_t lastLeaderUpdateMs_{0}; // last time leader status was received
    pthread_mutex_t leaderMutex_{}; // protects leaderPosition_/leaderSpeed_

    // Client socket
    int clientSocket_;
    struct sockaddr_in leaderAddr_;

    // Event handling
    std::queue<int> eventQueue_;      // Queue of event codes from user input
    pthread_mutex_t eventMutex_{};    // Protects eventQueue_
    pthread_cond_t eventCv_{};        // Signal when new event is available

    // --- Traffic light + left-behind simulation ---
    TrafficLightStatus trafficLight_{LIGHT_GREEN};
    bool decoupled_{false};
    bool communicationLost_{false};
    bool stopSendingStatus_{false};
    std::int64_t lastReconnectAttemptMs_{0};
    std::int64_t resumeSendingMs_{0};
    bool tryingRejoin_{false};
    bool delayAfterNextGreenArmed_{false};
    int delayAfterNextGreenSec_{0};
    std::int64_t delayedUntilMs_{0};

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
    pthread_t eventThread_{};
    static void* eventSimulationThreadEntry(void* arg); // User input event thread
    pthread_t eventSenderThread_{};
    static void* eventSenderThreadEntry(void* arg);     // Event sender thread
    
    // Helpers
    void createClientSocket();
    void sendStatusToLeader();
    void sendCoupleCommandToLeader(bool couple = true);
};

#endif // FOLLOW_H
