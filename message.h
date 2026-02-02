/**
 * @file message.h
 *
 * @brief Message definitions for platooning simulation.
 */

#ifndef MESSAGE_H
#define MESSAGE_H

#include <cstdint>
#include <vector>
#include "vehicle.h"
#include "system_config.h"
#include "matrix_clock.h"
// Message types
enum MessageType : std::uint8_t {
    STATUS_UPDATE = 0,   // Vehicle status update
    COUPLE_COMMAND = 1,  // Command to couple/decouple vehicles
    PLATOON_STATE = 2,   // Platoon state broadcast
    TRAFFIC_LIGHT_ALERT = 3,   // Traffic alert message
    ENERGY_DEPLETION_ALERT = 4,  // Energy depletion alert
    ENERGY_RESTORED = 5,  // Energy restored message
    GAS_STATION_ALERT = 6,  // Gas station alert
    OBSTACLE_DETECTED_ALERT = 7, // Obstacle detected alert
    REMOVE_VEHICLE = 8, // Remove vehicle from platoon
    LEAVE_PLATOON = 9, // Vehicle leaving platoon
    DELAY_NOTIFICATION = 10 // Notification to delay start
};

struct DelayNotificationMessage {
    MessageType type;       // Message type
    int delaySeconds;       // How many seconds to delay
    std::int64_t timestamp; // Timestamp of the message
    MatrixClock matrixClock;                  // Logical matrix clock
};

struct LeavePlatoonMessage {
    MessageType type;       // Message type
    int vehicleId;          // ID of the vehicle leaving the platoon
    std::int64_t timestamp; // Timestamp of the message
    MatrixClock matrixClock;                  // Logical matrix clock
};
struct RemoveVehicleMessage {
    MessageType type;       // Message type
    int vehicleId;          // ID of the vehicle to remove
    std::int64_t timestamp; // Timestamp of the message
    MatrixClock matrixClock;                  // Logical matrix clock
};
struct ObstacleMessage {
    MessageType type;       // Message type
    bool obstacleDetected;  // true if obstacle detected
    std::int64_t timestamp; // Timestamp of the event
    MatrixClock matrixClock;                  // Logical matrix clock
};

struct EventMessage {
    MessageType type;       // Message type
    void* eventData;        // Pointer to event-specific data
    std::int64_t timestamp; // Timestamp of the event
    MatrixClock matrixClock;                  // Logical matrix clock
};

// Traffic light status alert data
struct TrafficLightMessage {
    MessageType type;       // Message type
    std::uint8_t status;    // LIGHT_RED or LIGHT_GREEN
    std::int64_t timestamp; // Timestamp of the event
    MatrixClock matrixClock;                  // Logical matrix clock
};
// Energy depletion alert data
struct EnergyDepletionMessage {
    MessageType type;       // Message type
    int vehicleId;          // ID of the vehicle running out of energy
    std::int64_t timestamp; // Timestamp of the event
    MatrixClock matrixClock;                  // Logical matrix clock
};

// Energy restored data
struct EnergyRestoredMessage {
    MessageType type;       // Message type
    int vehicleId;          // ID of the vehicle restoring energy
    std::int64_t timestamp; // Timestamp of the event
    MatrixClock matrixClock;                  // Logical matrix clock
};

// Gas station alert data
struct GasStationMessage {
    MessageType type;       // Message type
    int vehicleId;          // ID of the vehicle at gas station
    std::int64_t timestamp; // Timestamp of the event
    MatrixClock matrixClock;                  // Logical matrix clock
};

// Structure for status update message
struct StatusUpdateMessage {
    MessageType type;       // Message type
    VehicleInfo info;       // Vehicle information
    std::int64_t timestamp; // Timestamp of the message
    MatrixClock matrixClock;                  // Logical matrix clock
};

// Structure for couple command message
struct CoupleCommandMessage {
    MessageType type;       // Message type
    VehicleInfo info;       // Vehicle information
    bool couple;            // true to couple, false to decouple
    std::int64_t timestamp; // Timestamp of the message
    MatrixClock matrixClock;// Logical matrix clock
};

// Structure for platoon state broadcast (leader -> followers)
struct PlatoonStateMessage {
    MessageType type;                         // PLATOON_STATE
    int leaderId;                             // Leader vehicle ID
    int vehicleCount;                         // Number of vehicles in platoon
    VehicleInfo vehicles[MAX_PLATOON_VEHICLES]; // Fixed-size array (sorted by position descending: leader first)
    std::int64_t timestamp;                   // Timestamp of the message
    MatrixClock matrixClock;                  // Logical matrix clock
};

#endif // MESSAGE_H
