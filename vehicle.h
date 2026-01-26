/**
 * @file vehicle.h
 *
 * @brief Vehicle base class for platooning simulation.
 */

#ifndef VEHICLE_H
#define VEHICLE_H

#include <cstdint>
#include <vector>
#include <algorithm>

// Maximum number of processes for matrix clock
constexpr int MAX_PROCESSES = 16;

// Matrix Clock structure
struct MatrixClock {
    int matrix[MAX_PROCESSES][MAX_PROCESSES] = {{0}};
    int processId = 0;
    int numProcesses = 0;

    // Initialize matrix clock for a process
    void init(int id, int n) {
        processId = id;
        numProcesses = n;
        for (int i = 0; i < MAX_PROCESSES; ++i) {
            for (int j = 0; j < MAX_PROCESSES; ++j) {
                matrix[i][j] = 0;
            }
        }
    }

    // Increment local clock on local event
    void localEvent() {
        matrix[processId][processId]++;
    }

    // Prepare clock before sending (increment and return copy)
    void onSend() {
        matrix[processId][processId]++;
    }

    // Merge received matrix clock
    void onReceive(const MatrixClock& received) {
        int senderId = received.processId;
        
        // Update our view of sender's row with their latest knowledge
        for (int j = 0; j < MAX_PROCESSES; ++j) {
            matrix[senderId][j] = std::max(matrix[senderId][j], received.matrix[senderId][j]);
        }
        
        // Take element-wise max for all other rows (what we and sender both know)
        for (int i = 0; i < MAX_PROCESSES; ++i) {
            if (i == processId) continue; // skip our own row for now
            for (int j = 0; j < MAX_PROCESSES; ++j) {
                matrix[i][j] = std::max(matrix[i][j], received.matrix[i][j]);
            }
        }
        
        // Update our own row with max of what we knew and what sender knows about others
        for (int j = 0; j < MAX_PROCESSES; ++j) {
            if (j != processId) {
                matrix[processId][j] = std::max(matrix[processId][j], received.matrix[senderId][j]);
            }
        }
        
        // Increment own clock
        matrix[processId][processId]++;
    }

    // Check if all processes know about a particular event
    bool allKnow(int eventProcess, int eventTime) const {
        for (int i = 0; i < numProcesses; ++i) {
            if (matrix[i][eventProcess] < eventTime) {
                return false;
            }
        }
        return true;
    }

    // Get own logical time
    int getLocalTime() const {
        return matrix[processId][processId];
    }
};

// Vehicle operating modes
enum VehicleMode : std::uint8_t {
    LeaderMode = 0,
    FollowerMode = 1
};

// Structure to hold vehicle state information
struct VehicleInfo {
    int id;             // Unique vehicle ID
    double position;    // in meters
    double speed;       // in m/s
    std::uint8_t mode;  // LeaderMode or FollowerMode
    uint32_t ipAddress; // IP address (network byte order)
    uint16_t port;      // UDP port number
        std::int64_t lastHeartbeatMs; // monotonic timestamp (ms) of last heartbeat
    double energy;      // Energy level (0.0 to 100.0)
    // TODO: Add more if needed: obstacleDetected, clock, etc.
};

// Struct to hold
struct PlatoonState {
    std::vector<VehicleInfo> vehicles;  // List of vehicles in the platoon
    int leaderId;  // ID of the current leader
    int followerCount; // Number of followers
    // TODO: Add metadata: timestamp, etc.
};

#endif // VEHICLE_H
