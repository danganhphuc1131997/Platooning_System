/**
 * @file message.h
 * @brief Message structures for Leader-Follower communication
 * 
 * Protocol:
 *   - TCP (port 8080): JOIN request/response (reliable)
 *   - UDP (port 9000): Leader broadcasts state
 *   - UDP (port 9001): Follower sends state to Leader
 */

#ifndef MESSAGE_H
#define MESSAGE_H

#include <cstdint>
#include <cstring>

// ============== PORTS ==============
constexpr int TCP_PORT = 8080;          // TCP: Join/Leave
constexpr int UDP_LEADER_PORT = 9000;   // UDP: Leader -> Followers
constexpr int UDP_FOLLOWER_PORT = 9001; // UDP: Followers -> Leader

// ============== MESSAGE TYPES ==============
enum class MsgType : uint8_t {
    // TCP Messages (Join/Leave Phase)
    JOIN_REQUEST   = 1,
    JOIN_RESPONSE  = 2,
    LEAVE_REQUEST  = 3,
    LEAVE_RESPONSE = 4,
    
    // UDP Messages (Data Phase)
    LEADER_STATE   = 10,   // Leader broadcasts state
    FOLLOWER_STATE = 11,   // Follower sends state to Leader
};

// ============== JOIN MESSAGES (TCP) ==============
#pragma pack(push, 1)
struct JoinRequest {
    uint8_t  type;           // MsgType::JOIN_REQUEST
    char     vehicle_id[32]; // Follower vehicle ID
    double   position;       // Current position (m)
    double   speed;          // Current speed (km/h)
};

struct JoinResponse {
    uint8_t  type;           // MsgType::JOIN_RESPONSE
    uint8_t  accepted;       // 1 = accepted, 0 = rejected
    int32_t  assigned_index; // Position in platoon (1, 2, 3...)
    char     message[64];    // Response message
};

struct LeaveRequest {
    uint8_t  type;           // MsgType::LEAVE_REQUEST
    int32_t  vehicle_index;  // Position in platoon
    char     vehicle_id[32]; // Vehicle ID
    uint8_t  reason;         // 0 = normal, 1 = emergency, 2 = maintenance
};

struct LeaveResponse {
    uint8_t  type;           // MsgType::LEAVE_RESPONSE
    uint8_t  success;        // 1 = success, 0 = failed
    char     message[64];    // Response message
};

// ============== LEADER STATE (UDP) ==============
struct LeaderState {
    uint8_t  type;           // MsgType::LEADER_STATE
    uint32_t sequence;       // Message sequence number (for packet loss detection)
    uint64_t timestamp_ms;   // Send timestamp (milliseconds)
    
    // Control data
    double   speed;          // Speed (km/h)
    double   acceleration;   // Acceleration (m/s²)
    double   position;       // Position (m)
    
    // Status flags
    uint8_t  brake_active;   // 0 = off, 1 = on
    uint8_t  emergency;      // 0 = normal, 1 = emergency
};

// ============== FOLLOWER STATE (UDP) ==============
struct FollowerState {
    uint8_t  type;           // MsgType::FOLLOWER_STATE
    uint32_t sequence;       // Message sequence number
    uint64_t timestamp_ms;   // Send timestamp
    
    int32_t  vehicle_index;  // Position in platoon
    char     vehicle_id[32]; // Vehicle ID
    
    // State data
    double   speed;          // Speed (km/h)
    double   position;       // Position (m)
    double   distance_to_leader; // Gap to leader (m)
    
    // Status
    uint8_t  status;         // 0 = OK, 1 = warning, 2 = error
};
#pragma pack(pop)

// ============== HELPER FUNCTIONS ==============
inline const char* msgTypeToString(MsgType t) {
    switch (t) {
        case MsgType::JOIN_REQUEST:   return "JOIN_REQUEST";
        case MsgType::JOIN_RESPONSE:  return "JOIN_RESPONSE";
        case MsgType::LEAVE_REQUEST:  return "LEAVE_REQUEST";
        case MsgType::LEAVE_RESPONSE: return "LEAVE_RESPONSE";
        case MsgType::LEADER_STATE:   return "LEADER_STATE";
        case MsgType::FOLLOWER_STATE: return "FOLLOWER_STATE";
        default: return "UNKNOWN";
    }
}

#endif // MESSAGE_H
