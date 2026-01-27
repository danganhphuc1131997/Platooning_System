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

// Message types
enum MessageType : std::uint8_t {
    STATUS_UPDATE = 0,   // Vehicle status update
    COUPLE_COMMAND = 1,  // Command to couple/decouple vehicles
    PLATOON_STATE = 2,   // Platoon state broadcast
    TRAFFIC_LIGHT_ALERT = 3,   // Traffic alert message
    ENERGY_DEPLETION_ALERT = 4,  // Energy depletion alert
    ENERGY_RESTORED = 5,  // Energy restored message
    GAS_STATION_ALERT = 6  // Gas station alert
};

struct EventMessage {
    MessageType type;       // Message type
    void* eventData;        // Pointer to event-specific data
    std::int64_t timestamp; // Timestamp of the event
};

// Traffic light status alert data
struct TrafficLightMessage {
    MessageType type;       // Message type
    std::uint8_t status;    // LIGHT_RED or LIGHT_GREEN
    std::int64_t timestamp; // Timestamp of the event
};
// Energy depletion alert data
struct EnergyDepletionMessage {
    MessageType type;       // Message type
    int vehicleId;          // ID of the vehicle running out of energy
    std::int64_t timestamp; // Timestamp of the event
};

// Energy restored data
struct EnergyRestoredMessage {
    MessageType type;       // Message type
    int vehicleId;          // ID of the vehicle restoring energy
    std::int64_t timestamp; // Timestamp of the event
};

// Gas station alert data
struct GasStationMessage {
    MessageType type;       // Message type
    int vehicleId;          // ID of the vehicle at gas station
    std::int64_t timestamp; // Timestamp of the event
};

// Structure for status update message
struct StatusUpdateMessage {
    MessageType type;       // Message type
    VehicleInfo info;       // Vehicle information
    std::int64_t timestamp; // Timestamp of the message
};

// Structure for couple command message
struct CoupleCommandMessage {
    MessageType type;       // Message type
    VehicleInfo info;       // Vehicle information
    bool couple;            // true to couple, false to decouple
    std::int64_t timestamp; // Timestamp of the message
};

// Maximum vehicles in a platoon for fixed-size wire message
constexpr int MAX_PLATOON_VEHICLES = 16;

// Structure for platoon state broadcast (leader -> followers)
struct PlatoonStateMessage {
    MessageType type;                         // PLATOON_STATE
    int leaderId;                             // Leader vehicle ID
    int vehicleCount;                         // Number of vehicles in platoon
    VehicleInfo vehicles[MAX_PLATOON_VEHICLES]; // Fixed-size array (sorted by position descending: leader first)
    std::int64_t timestamp;                   // Timestamp of the message
};

#endif // MESSAGE_H
