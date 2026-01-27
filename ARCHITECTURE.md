# Platooning System Architecture

## System Overview

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                         PLATOONING SYSTEM                                    │
│                                                                              │
│   ┌─────────────┐          UDP (port 8080)           ┌─────────────┐         │
│   │   LEADER    │◄──────────────────────────────────►│  FOLLOWER   │         │
│   │  (lead.cpp) │         WireMessage                │ (follow.cpp)│         │
│   └─────────────┘                                    └─────────────┘         │
│         ▲                                                   ▲                │
│         │                                                   │                │
│         ▼                                                   ▼                │
│   ┌─────────────┐                                    ┌─────────────┐         │
│   │  FOLLOWER   │                                    │  FOLLOWER   │         │
│   │ (follow.cpp)│                                    │ (follow.cpp)│         │
│   └─────────────┘                                    └─────────────┘         │
│                                                                              │
└──────────────────────────────────────────────────────────────────────────────┘
```

---

## Component Diagram (Mermaid)

```mermaid
flowchart TB
    subgraph Leader["🚗 Leader Vehicle (lead.cpp)"]
        LS[UDP Socket<br/>Port 8080]
        LM[Mutex Lock]
        LC[Matrix Clock<br/>MAX_NODES x MAX_NODES]
        
        subgraph LThreads["Threads"]
            RT[Recv Thread]
            BT[Broadcast Thread]
            MT[Monitor Thread]
        end
        
        LState["State:<br/>• position<br/>• speed<br/>• obstacle flag<br/>• degraded mode"]
    end

    subgraph Follower["🚙 Follower Vehicle (follow.cpp)"]
        FS[UDP Socket]
        FM[Mutex Lock]
        FC[Matrix Clock<br/>MAX_NODES x MAX_NODES]
        
        subgraph FThreads["Threads"]
            RT[Recv Thread]
            ST[Send Thread]
            IT[Input Thread]
        end
        
        FState["State:<br/>• position<br/>• speed<br/>• PID controller<br/>• cut-in flag"]
    end

    Leader <-->|WireMessage| Follower
```

---

## Message Protocol

```mermaid
classDiagram
    class WireMessage {
        +uint8_t type
        +int32_t senderId
        +int32_t senderIndex
        +int32_t assignedIndex
        +double position
        +double speed
        +uint8_t obstacle
        +uint8_t flags
        +int32_t clock[MAX_NODES][MAX_NODES]
    }

    class MsgType {
        <<enumeration>>
        JOIN = 1
        JOIN_ACK = 2
        LEAVE = 3
        LEADER_STATE = 4
        FOLLOWER_STATE = 5
    }

    class Flags {
        <<bitfield>>
        FLAG_CONNECTED = 0x01
        FLAG_DEGRADED = 0x02
    }

    WireMessage --> MsgType : type
    WireMessage --> Flags : flags
```

---

## Communication Sequence

```mermaid
sequenceDiagram
    participant F as Follower
    participant L as Leader

    Note over F,L: Join Phase
    F->>L: JOIN (senderId, position, speed)
    L->>F: JOIN_ACK (assignedIndex)

    Note over F,L: Normal Operation
    loop Every broadcast interval
        L->>F: LEADER_STATE (position, speed, clock)
    end

    loop Heartbeat
        F->>L: FOLLOWER_STATE (position, speed, clock)
    end

    Note over F,L: Obstacle Detection
    F->>L: FOLLOWER_STATE (obstacle=1)
    L->>F: LEADER_STATE (speed=0, obstacle=1)

    Note over F,L: Cut-in / Partition
    F--xL: Heartbeat paused (3s)
    L->>L: Timeout → Degraded Mode
    L->>F: LEADER_STATE (FLAG_DEGRADED)
    F->>L: FOLLOWER_STATE resumed
    L->>L: Clear degraded mode

    Note over F,L: Leave Phase
    F->>L: LEAVE
    L->>L: Remove follower from registry
```

---

## State Machine

```mermaid
stateDiagram-v2
    [*] --> Disconnected

    state Leader {
        [*] --> Listening
        Listening --> ReceivingMessages : bind()
        ReceivingMessages --> Broadcasting : follower joined (JOIN)
        Broadcasting --> DegradedMode : heartbeat timeout
        DegradedMode --> Broadcasting : heartbeat resumed
        Broadcasting --> Stopped : obstacle detected
        Stopped --> Broadcasting : obstacle cleared
    }

    state Follower {
        [*] --> Connecting
        Connecting --> Connected : JOIN_ACK received
        Connected --> Following : receive LEADER_STATE
        Following --> SafeMode : connection lost
        SafeMode --> Connecting : tryReconnect()
        Following --> CutInSim : 'c' pressed
        CutInSim --> Following : 3s elapsed
        Following --> Leaving : 'q' pressed
        Leaving --> [*]
    }
```

---

## Thread Model

```mermaid
flowchart LR
    subgraph Leader
        direction TB
        Main1[main] --> Recv[recvLoop]
        Main1 --> Broadcast[broadcastLoop]
        Main1 --> Monitor[monitorLoop]
    end

    subgraph Follower
        direction TB
        Main2[main] --> Recv[recvLoop]
        Main2 --> Send[sendLoop]
        Main2 --> Input[inputLoop]
    end

    Recv -.->|mutex| State1[(Shared State)]
    Broadcast -.->|mutex| State1
    Monitor -.->|mutex| State1

    Recv -.->|mutex| State2[(Shared State)]
    Send -.->|mutex| State2
    Input -.->|mutex| State2
```

---

## Matrix Clock Synchronization

```mermaid
flowchart TD
    subgraph Node0["Leader (index 0)"]
        C0["clock[0][0]++<br/>on local event"]
    end

    subgraph Node1["Follower 1 (index 1)"]
        C1["clock[1][1]++<br/>on local event"]
    end

    subgraph Node2["Follower 2 (index 2)"]
        C2["clock[2][2]++<br/>on local event"]
    end

    Node0 -->|"send: include full clock matrix"| Node1
    Node0 -->|"send: include full clock matrix"| Node2
    Node1 -->|"send: include full clock matrix"| Node0
    Node2 -->|"send: include full clock matrix"| Node0

    Node1 -->|"merge: elementwise max"| Node1
    Node2 -->|"merge: elementwise max"| Node2
    Node0 -->|"merge: elementwise max"| Node0
```

---

## File Structure

```
Platooning_System/
├── message.h         # Shared wire protocol (WireMessage, MsgType, Flags)
├── lead.h            # Leader class declaration
├── lead.cpp          # Leader implementation
├── follow.h          # Follower class declaration
├── follow.cpp        # Follower implementation
├── README_RUN.md     # Build & run instructions
├── ARCHITECTURE.md   # This file
│
└── dsrc/             # 🆕 DSRC V2V Refactored Version
    ├── bsm.h         # Basic Safety Message (SAE J2735 inspired)
    ├── vehicle.h     # Unified vehicle node
    ├── vehicle.cpp   # Vehicle implementation
    ├── main.cpp      # Entry point
    ├── Makefile      # Build system
    └── README.md     # DSRC-specific docs
```

---

## Quick Reference

| Component | Port | Protocol | Key Features |
|-----------|------|----------|--------------|
| Leader    | 8080 | UDP      | Receive from followers, broadcast state, monitor heartbeats |
| Follower  | ephemeral | UDP | Send to leader, receive broadcasts, PID control |

| Message Type    | Direction       | Purpose |
|-----------------|-----------------|---------|
| JOIN            | Follower→Leader | Request to join platoon |
| JOIN_ACK        | Leader→Follower | Assign clock index |
| LEAVE           | Follower→Leader | Graceful departure |
| LEADER_STATE    | Leader→Follower | Broadcast position/speed |
| FOLLOWER_STATE  | Follower→Leader | Heartbeat + state update |

---

## DSRC V2V Version (Recommended for Real-World)

The `dsrc/` folder contains a refactored version using UDP broadcast, which is more suitable for real V2V/DSRC deployment.

### Key Differences

| Aspect | Main Version | DSRC Version |
|--------|-------------|--------------|
| Transport | UDP (unicast) | **UDP broadcast/multicast** |
| Topology | Star (all → leader) | **Mesh** (all hear all) |
| Binary | Separate `lead` + `follow` | **Single `platoon`** |
| Leadership | Fixed leader server | **Position-based** (auto failover) |
| Message | Custom `WireMessage` | **BSM** (SAE J2735 inspired) |
| Frequency | Variable | **10 Hz** (DSRC standard) |

### DSRC Architecture

```mermaid
flowchart TB
    subgraph DSRC["UDP Broadcast Channel (Port 5900)"]
        direction LR
        V1["🚗 Vehicle 1<br/>(Position 0 = Leader)"]
        V2["🚙 Vehicle 2<br/>(Position 1)"]
        V3["🚙 Vehicle 3<br/>(Position 2)"]
    end

    V1 <-->|BSM @ 10Hz| V2
    V2 <-->|BSM @ 10Hz| V3
    V1 <-->|BSM @ 10Hz| V3

    Note["All vehicles broadcast BSM<br/>All vehicles listen<br/>Leader = lowest alive position"]
```

### Leader Failover

```mermaid
sequenceDiagram
    participant V1 as Vehicle 1 (pos=0)
    participant V2 as Vehicle 2 (pos=1)
    participant V3 as Vehicle 3 (pos=2)

    Note over V1,V3: Normal Operation
    V1->>V2: BSM (Leader)
    V1->>V3: BSM (Leader)
    V2->>V1: BSM
    V3->>V1: BSM

    Note over V1: Leader leaves/crashes
    V1--xV2: BSM timeout (500ms)
    V1--xV3: BSM timeout (500ms)

    Note over V2: No peer with position < 1 alive
    V2->>V2: Become Leader (position = 0)
    
    Note over V2,V3: V2 is now Leader
    V2->>V3: BSM (Leader)
    V3->>V2: BSM
```

### Build & Run DSRC Version

```bash
cd dsrc
make

# Terminal 1 - Leader
./platoon 1 100 0

# Terminal 2 - Follower
./platoon 2 100 1

# Terminal 3 - Follower
./platoon 3 100 2
```

### BSM Message Format

```mermaid
classDiagram
    class PlatoonBSM {
        +uint32_t vehicleId
        +uint32_t platoonId
        +uint8_t platoonPosition
        +double latitude
        +double longitude
        +float speed
        +float acceleration
        +float heading
        +float targetGap
        +float actualGap
        +uint8_t brakeStatus
        +uint8_t intentFlags
        +uint64_t timestampMs
        +int32_t vectorClock[]
    }

    class Intent {
        <<flags>>
        JOINING
        LEAVING
        EMERGENCY
        LANE_CHANGE
    }

    class BrakeStatus {
        <<flags>>
        APPLIED
        ABS_ACTIVE
        STABILITY
        TRACTION
    }

    PlatoonBSM --> Intent
    PlatoonBSM --> BrakeStatus
```
