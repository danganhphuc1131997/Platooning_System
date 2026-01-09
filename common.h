/**
 * common.h - Shared definitions for distributed platoon system
 * 
 * Architecture:
 * - DISTRIBUTED: Each vehicle is a separate process
 * - PARALLEL: Each vehicle has multiple threads (Comm, Control, Sensor, Decision)
 * - IPC: POSIX Message Queues for inter-vehicle communication
 */

#ifndef COMMON_H
#define COMMON_H

#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>
#include <mqueue.h>

/*============================================================================
 * SYSTEM CONSTANTS
 *============================================================================*/

#define MAX_VEHICLES        8
#define DT_MS               100     // Control loop period (ms)
#define DESIRED_GAP         10.0    // meters
#define MAX_SPEED           30.0    // m/s
#define MAX_ACCEL           3.0     // m/s²
#define HEARTBEAT_INTERVAL  10      // cycles (1 second)
#define HEARTBEAT_TIMEOUT   50      // cycles (5 seconds)

// Message Queue settings
#define MQ_MAX_MSGS         10      // System default limit
#define MQ_MSG_SIZE         256
#define MQ_NAME_PREFIX      "/platoon_v"  // e.g., /platoon_v0, /platoon_v1

/*============================================================================
 * MESSAGE TYPES (for IPC between vehicles)
 *============================================================================*/

typedef enum {
    MSG_HEARTBEAT = 0,
    MSG_COUPLE_REQUEST,
    MSG_COUPLE_ACCEPT,
    MSG_COUPLE_REJECT,
    MSG_DECOUPLE_REQUEST,
    MSG_DECOUPLE_ACK,
    MSG_EMERGENCY_BRAKE,
    MSG_LEADER_ELECTION,
    MSG_LEADER_ANNOUNCE,
    MSG_STATE_UPDATE
} MessageType;

typedef struct {
    MessageType type;
    int         sender_id;
    int         target_id;      // -1 = broadcast
    uint32_t    seq_num;
    uint32_t    timestamp;
    
    // Payload
    double      pos;
    double      speed;
    double      accel;
    int         priority;       // for election
    int         predecessor_id;
} Message;

/*============================================================================
 * VEHICLE STATE (shared within a vehicle process)
 *============================================================================*/

typedef enum {
    ROLE_LEADER = 0,
    ROLE_FOLLOWER
} VehicleRole;

typedef enum {
    STATE_IDLE = 0,
    STATE_REQUESTING,
    STATE_COUPLED,
    STATE_DECOUPLING,
    STATE_EMERGENCY
} VehicleState;

// Shared state within a vehicle (accessed by multiple threads)
typedef struct {
    // Identity
    int             id;
    VehicleRole     role;
    VehicleState    state;
    
    // Dynamics
    double          pos;
    double          speed;
    double          accel;
    
    // Platoon info
    int             leader_id;
    int             predecessor_id;
    int             num_vehicles;
    
    // Timing
    uint32_t        seq_num;
    uint32_t        last_heartbeat[MAX_VEHICLES];
    
    // Flags
    bool            running;
    bool            election_in_progress;
    
    // Sensor data (from sensor thread)
    double          sensor_gap;         // measured gap to predecessor
    double          sensor_rel_speed;   // relative speed
    bool            sensor_obstacle;    // obstacle detected
    
    // Decision output (from decision thread)
    double          cmd_accel;          // commanded acceleration
    bool            cmd_emergency;      // emergency flag
    
    // Mutex for thread safety
    pthread_mutex_t mutex;
    
    // Condition variables for thread synchronization
    pthread_cond_t  sensor_ready;
    pthread_cond_t  decision_ready;
} VehicleData;

/*============================================================================
 * UTILITY FUNCTIONS
 *============================================================================*/

static inline void util_msleep(int ms) {
    struct timespec ts = { ms / 1000, (ms % 1000) * 1000000L };
    nanosleep(&ts, NULL);
}

static inline const char* msg_type_str(MessageType t) {
    static const char* names[] = {
        "HEARTBEAT", "COUPLE_REQ", "COUPLE_ACC", "COUPLE_REJ",
        "DECOUPLE_REQ", "DECOUPLE_ACK", "EMERGENCY", 
        "ELECTION", "LEADER_ANN", "STATE_UPD"
    };
    return (t >= 0 && t <= MSG_STATE_UPDATE) ? names[t] : "UNKNOWN";
}

static inline const char* state_str(VehicleState s) {
    static const char* names[] = {
        "IDLE", "REQUESTING", "COUPLED", "DECOUPLING", "EMERGENCY"
    };
    return (s >= 0 && s <= STATE_EMERGENCY) ? names[s] : "UNKNOWN";
}

static inline const char* role_str(VehicleRole r) {
    return (r == ROLE_LEADER) ? "LEADER" : "FOLLOWER";
}

#endif /* COMMON_H */
