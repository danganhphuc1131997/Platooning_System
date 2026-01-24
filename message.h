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
    TRAFFIC_LIGHT_ALERT = 3    // Traffic alert message
};

struct EventMessage {
    MessageType type;       // Message type
    void* eventData;        // Pointer to event-specific data
    std::int64_t timestamp; // Timestamp of the event
};

// Traffic light status alert data
struct TrafficLightMessage {
    MessageType type;       // Message type
    std::int64_t timestamp; // Timestamp of the event
    std::uint8_t status; // LIGHT_RED or LIGHT_GREEN
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

#endif // MESSAGE_H
