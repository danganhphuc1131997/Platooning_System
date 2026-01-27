/**
 * @file vehicle.h
 *
 * @brief Vehicle base class for platooning simulation.
 */

#ifndef VEHICLE_H
#define VEHICLE_H

#include <cstdint>
#include <vector>

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
