# Truck Platooning System - Architecture

## System Overview

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                         TRUCK PLATOONING SYSTEM                              │
│                                                                              │
│   ┌─────────────┐         TCP (8080)         ┌─────────────┐                 │
│   │   LEADER    │◄────────JOIN/ACK──────────►│  FOLLOWER   │                 │
│   │             │                            │             │                 │
│   │             │        UDP (9000)          │             │                 │
│   │             │────────BROADCAST──────────►│             │                 │
│   │             │                            │             │                 │
│   │             │        UDP (9001)          │             │                 │
│   │             │◄─────FOLLOWER_STATE────────│             │                 │
│   └─────────────┘                            └─────────────┘                 │
│         │                                           │                        │
│         │           UDP (9000)                      │                        │
│         └──────────BROADCAST──────────────►┌─────────────┐                   │
│                                            │  FOLLOWER   │                   │
│                    UDP (9001)              │             │                   │
│         ◄─────────FOLLOWER_STATE───────────│             │                   │
│                                            └─────────────┘                   │
└──────────────────────────────────────────────────────────────────────────────┘
```

---

## Communication Protocol

### Hybrid TCP/UDP

| Phase | Protocol | Port | Description |
|-------|----------|------|-------------|
| Join | TCP | 8080 | Platoon registration (reliable) |
| Data | UDP | 9000 | Leader → Followers (broadcast) |
| Data | UDP | 9001 | Followers → Leader (unicast) |

**Why hybrid approach:**
- **TCP for JOIN**: Ensures follower receives `assigned_index` before joining
- **UDP for DATA**: High frequency (10Hz), tolerates packet loss, has sequence number for detection

---

## Message Structures

### JoinRequest (TCP)
```cpp
struct JoinRequest {
    uint8_t  type;           // = 1 (JOIN_REQUEST)
    char     vehicle_id[32]; // Vehicle ID
    double   position;       // Position (m)
    double   speed;          // Speed (km/h)
};
```

### JoinResponse (TCP)
```cpp
struct JoinResponse {
    uint8_t  type;           // = 2 (JOIN_RESPONSE)
    uint8_t  accepted;       // 1 = accepted
    int32_t  assigned_index; // Position in platoon
    char     message[64];    // Response message
};
```

### LeaderState (UDP)
```cpp
struct LeaderState {
    uint8_t  type;           // = 10 (LEADER_STATE)
    uint32_t sequence;       // Sequence number (packet loss detection)
    uint64_t timestamp_ms;   // Send timestamp
    double   speed;          // Speed (km/h)
    double   acceleration;   // Acceleration (m/s²)
    double   position;       // Position (m)
    uint8_t  brake_active;   // 0/1
    uint8_t  emergency;      // 0/1
};
```

### FollowerState (UDP)
```cpp
struct FollowerState {
    uint8_t  type;           // = 11 (FOLLOWER_STATE)
    uint32_t sequence;       // Sequence number
    uint64_t timestamp_ms;   // Send timestamp
    int32_t  vehicle_index;  // Position in platoon
    char     vehicle_id[32]; // Vehicle ID
    double   speed;          // Speed (km/h)
    double   position;       // Position (m)
    double   distance_to_leader; // Gap (m)
    uint8_t  status;         // 0=OK, 1=warning, 2=error
};
```

---

## Sequence Diagram

```
┌──────────┐                    ┌──────────┐
│ Follower │                    │  Leader  │
└────┬─────┘                    └────┬─────┘
     │                               │
     │  ══════ JOIN PHASE (TCP) ═════│
     │                               │
     │──── JoinRequest ─────────────►│
     │     (vehicle_id, pos, speed)  │
     │                               │
     │◄─── JoinResponse ─────────────│
     │     (accepted, index, msg)    │
     │                               │
     │   [TCP connection closed]     │
     │                               │
     │ ══════ DATA PHASE (UDP) ══════│
     │                               │
     │◄─── LeaderState (broadcast) ──│ (10Hz)
     │     (speed, pos, brake, seq)  │
     │                               │
     │──── FollowerState ───────────►│ (2Hz)
     │     (speed, pos, gap, status) │
     │                               │
     │◄─── LeaderState ──────────────│
     │                               │
     │──── FollowerState ───────────►│
     │                               │
     ▼                               ▼
```

---

## Thread Model

### Leader (3 threads)
```
┌─────────────────────────────────────────────┐
│                  LEADER                      │
├─────────────────────────────────────────────┤
│  ┌─────────────────┐                        │
│  │ TCP Thread      │ ← Accept JOIN requests │
│  │ (tcp_mgmt)      │                        │
│  └─────────────────┘                        │
│                                             │
│  ┌─────────────────┐                        │
│  │ UDP TX Thread   │ → Broadcast state      │
│  │ (udp_broadcast) │   every 100ms          │
│  └─────────────────┘                        │
│                                             │
│  ┌─────────────────┐                        │
│  │ UDP RX Thread   │ ← Receive follower     │
│  │ (udp_receive)   │   states               │
│  └─────────────────┘                        │
│                                             │
│  ┌─────────────────┐                        │
│  │ Shared State    │ ← Protected by mutex   │
│  │ (platoon_mtx)   │                        │
│  └─────────────────┘                        │
└─────────────────────────────────────────────┘
```

### Follower (2 threads)
```
┌─────────────────────────────────────────────┐
│                 FOLLOWER                     │
├─────────────────────────────────────────────┤
│  ┌─────────────────┐                        │
│  │ main()          │ → TCP JOIN (blocking)  │
│  └─────────────────┘                        │
│                                             │
│  ┌─────────────────┐                        │
│  │ UDP RX Thread   │ ← Receive leader       │
│  │ (udp_rx)        │   broadcasts           │
│  └─────────────────┘                        │
│                                             │
│  ┌─────────────────┐                        │
│  │ UDP TX Thread   │ → Send state to        │
│  │ (udp_tx)        │   leader (2Hz)         │
│  └─────────────────┘                        │
│                                             │
│  ┌─────────────────┐                        │
│  │ Shared State    │ ← Protected by mutex   │
│  │ (state_mtx)     │                        │
│  └─────────────────┘                        │
└─────────────────────────────────────────────┘
```

---

## Logging Format

```
[HH:MM:SS.mmm] [COMPONENT] MESSAGE
```

### Components
| Component | Description |
|-----------|-------------|
| `MAIN` | Main thread initialization |
| `TCP` | TCP join/leave operations |
| `UDP-TX` | UDP sending operations |
| `UDP-RX` | UDP receiving operations |

### Icons
| Icon | Meaning |
|------|---------|
| ✅ | Success |
| ❌ | Error |
| ⚠️ | Warning |
| 📡 | Broadcast |
| 📤 | Send |
| 📥 | Receive |
| 🛑 | Brake |
| 🚨 | Emergency |

---

## Future Extensions

1. **CACC Controller**: Add real PID/MPC controller
2. **Obstacle Detection**: Handle sensor obstacle data
3. **Multi-leader**: Support platoon splitting when leader leaves
4. **Encryption**: Add TLS for TCP, DTLS for UDP
5. **File Logging**: Write logs to file for analysis
