// message.h - shared wire protocol for leader/follower communication
//
// Goals:
//  - Safe over TCP: fixed-size POD struct (no std::vector)
//  - Implements a logical matrix clock pattern (fixed MAX_VEHICLES)
//  - Supports: join, leave, leader state, follower state (heartbeat), degraded mode
//  - Adds traffic light use-case: (RED/GREEN) + stop line position
//  - Adds follower mode reporting: COUPLED / STOPPING / DECOUPLED / CATCH_UP

#ifndef PLATOON_MESSAGE_H
#define PLATOON_MESSAGE_H

#include <cstdint>

// Adjust if you want to support more nodes in the same platoon.
// Leader uses index 0. Followers are assigned indices 1..MAX_VEHICLES-1 on join.
const int MAX_VEHICLES = 10;
const int MAX_NODES = MAX_VEHICLES;

enum class MsgType : std::uint8_t {
    JOIN = 1,
    JOIN_ACK = 2,
    LEAVE = 3,
    LEADER_STATE = 4,
    FOLLOWER_STATE = 5
};

// Flags bitfield
static constexpr std::uint8_t FLAG_CONNECTED = 1u << 0;
static constexpr std::uint8_t FLAG_DEGRADED  = 1u << 1;

// Traffic light states
enum class TrafficLight : std::uint8_t {
    NONE  = 0,
    GREEN = 1,
    RED   = 2
};

// Follower mode (visible in terminal + can be used for debugging)
enum class FollowerMode : std::uint8_t {
    COUPLED = 0,
    STOPPING_FOR_LIGHT = 1,
    DECOUPLED = 2,
    CATCH_UP = 3
};

// NOTE: Packed wire message. Keep it POD.
#pragma pack(push, 1)
struct WireMessage {
    std::uint8_t  type;          // MsgType
    std::int32_t  senderId;      // application-level node id (e.g., vehicle id)
    std::int32_t  senderIndex;   // matrix-clock index of sender (0..MAX_VEHICLES-1), or -1 if unknown (JOIN)
    std::int32_t  assignedIndex; // used in JOIN_ACK: index assigned by the leader; otherwise -1

    double        position;
    double        speed;
    std::uint8_t  obstacle;      // 0/1
    std::uint8_t  flags;         // FLAG_* bitfield

    // Traffic light use-case (sent by leader in LEADER_STATE and JOIN_ACK)
    std::uint8_t  trafficLight;  // TrafficLight
    double        stopLinePos;   // where to stop (meters)

    // Follower mode (sent by follower in FOLLOWER_STATE)
    std::uint8_t  followerMode;  // FollowerMode

    // Logical matrix clock
    std::int32_t  clockMatrix[MAX_VEHICLES][MAX_VEHICLES];

    // Optional mapping info (useful for debugging / robust demo output)
    std::int32_t  vehicleIds[MAX_VEHICLES];       // vehicleIds[i] = some node id
    std::int32_t  vehicleIndices[MAX_VEHICLES];   // vehicleIndices[i] = matrix index
    std::int32_t  numVehicles;                    // how many are active (<= MAX_VEHICLES)
};
#pragma pack(pop)

static_assert(sizeof(WireMessage) > 0, "WireMessage must have size");

#endif // PLATOON_MESSAGE_H
