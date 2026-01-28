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
#include <CL/cl.h>
#include <vector>
#include <algorithm>

enum LeaderState : std::uint8_t {
    NORMAL = 0, // Normal driving
    ERROR,      // Error state
    STOPPING,   // Braking to stop
    STOPPED,    // Stopped
    STARTING,   // Starting from stop
    LOW_ENERGY  // Low energy, reducing speed
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

    // For OpenCL testing purposes
    bool force_obstacle_{false};
private:
    VehicleInfo info_;
    LeaderState state_;
    std::atomic<bool> serverRunning_;
    double targetSpeed_; // Target speed for gradual changes
    double originalSpeed_; // Original speed before reduction
    bool energyAlertSent_; // Flag to track if energy depletion alert has been sent
    std::int64_t stopTimeMs_; // Time when stopped for auto start
    bool gasStationStop_; // Flag to indicate if current stop is for gas station
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

    // For OpenCL Lidar processing
    cl_platform_id platform_id;
    cl_device_id device_id;
    cl_context context;
    cl_command_queue command_queue;
    cl_program program;
    cl_kernel lidar_kernel;

    cl_mem input_distance_buffer; // Input buffer for distances
    cl_mem output_risk_buffer;    // Output buffer for risk levels

    std::vector<float> lidar_data_; // Lidar distance data
    std::vector<int> risk_map_;     // Risk map data

    void initOpenCL_AEB();          // Initialize OpenCL for AEB
    void cleanOpenCL();             // Clean up OpenCL resources
    bool scanEnvironmentWithGPU();  // Returns True if danger detected
    void updateSimulatedLidar();    // Simulate changing sensor data

    // Helpers
    void createServerSocket();
    void sendPlatoonState();
    void displayLoop();
};

#endif // LEAD_H
