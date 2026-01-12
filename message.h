// message.h - shared wire protocol for leader/follower communication
//
// Goals:
//  - Safe over UDP: fixed-size POD struct (no std::vector)
//  - Implements a logical matrix clock pattern (fixed MAX_NODES)
//  - Supports: join, leave, leader state, follower state (heartbeat), and degraded mode.

#ifndef PLATOON_MESSAGE_H
#define PLATOON_MESSAGE_H

#include <cstdint>

// Adjust if you want to support more nodes in the same platoon.
// Leader uses index 0. Followers are assigned indices 1..MAX_NODES-1 on join.
static constexpr int MAX_NODES = 6;

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

// NOTE: Packed wire message. Keep it POD.
#pragma pack(push, 1)
struct WireMessage {
    std::uint8_t  type;          // MsgType
    std::int32_t  senderId;      // application-level node id (e.g., vehicle id)
    std::int32_t  senderIndex;   // matrix-clock index of sender (0..MAX_NODES-1), or -1 if unknown (JOIN)
    std::int32_t  assignedIndex; // used in JOIN_ACK: the index assigned by the leader; otherwise -1

    double        position;
    double        speed;
    std::uint8_t  obstacle;      // 0/1
    std::uint8_t  flags;         // FLAG_* bitfield
    std::int32_t  clock[MAX_NODES][MAX_NODES];
};
#pragma pack(pop)

static_assert(sizeof(WireMessage) > 0, "WireMessage must have size");

#endif // PLATOON_MESSAGE_H
