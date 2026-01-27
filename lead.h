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

#include <atomic>
#include <cstdint>
#include <string>
#include "vehicle.h"
#include <netinet/in.h>
#include <queue>
#include "message.h"

enum LeaderState : std::uint8_t {
    NORMAL = 0, // Normal driving
    ERROR,      // Error state
    STOPPING,   // Braking to stop
    STOPPED,    // Stopped
    STARTING,    // Starting from stop
    STOPPING_FOR_RED_LIGHT // Stopping for red traffic light
};

class LeadingVehicle {
public:
    LeadingVehicle(int id, double initialPosition, double initialSpeed);
    ~LeadingVehicle();

    void startServer(); // Start UDP server to communicate with followers
    void stopServer();  // Stop the UDP server
    void startThreads(); // Start internal threads
    void setState(LeaderState newState);
    LeaderState getState() const;

private:
    VehicleInfo info_;
    LeaderState state_;
    std::atomic<bool> serverRunning_;
    PlatoonState platoonState_; // Current state of the platoon

    // Mutex for platoon state
    pthread_mutex_t mutex_;

    // Server socket
    int serverSocket_;

    // Timing
    static std::int64_t nowMs();

    // Event sending
    // for event sending
    std::queue<EventMessage> eventQueue_;
    pthread_mutex_t eventMutex_{};
    pthread_cond_t eventCv_{};

    // Threads
    pthread_t recvThread_{};
    static void* recvThreadEntry(void* arg);            // Thread entry for receiving messages
    pthread_t eventThread_{};
    static void* eventSimulationThreadEntry(void* arg); // User input thread
    pthread_t runThread_{};
    static void* runThreadEntry(void* arg);             // Main leader logic thread
    pthread_t displayThread_{};
    static void* displayThreadEntry(void* arg);         // Display status thread
    pthread_t heartbeatThread_{};
    static void* heartbeatThreadEntry(void* arg);       // Heartbeat monitoring thread
    pthread_t sendStatusThread_{};
    static void* sendStatusThreadEntry(void* arg);      // Status sending thread
    pthread_t eventSenderThread_{};
    static void* eventSenderThreadEntry(void* arg);     // Event sending thread

    // Helpers
    void createServerSocket();
    void sendPlatoonState();
    void displayLoop();
};

#endif // LEAD_H
