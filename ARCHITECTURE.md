# Platooning System Architecture

## System Overview

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                         PLATOONING SYSTEM                                    │
│                                                                              │
│   ┌─────────────┐         TCP/IP (port 8080)         ┌─────────────┐        │
│   │   LEADER    │◄──────────────────────────────────►│  FOLLOWER   │        │
│   │  (lead.cpp) │         WireMessage                │ (follow.cpp)│        │
│   └─────────────┘                                    └─────────────┘        │
│         ▲                                                   ▲               │
│         │                                                   │               │
│         ▼                                                   ▼               │
│   ┌─────────────┐                                    ┌─────────────┐        │
│   │  FOLLOWER   │                                    │  FOLLOWER   │        │
│   │ (follow.cpp)│                                    │ (follow.cpp)│        │
│   └─────────────┘                                    └─────────────┘        │
│                                                                              │
└──────────────────────────────────────────────────────────────────────────────┘
```

---

## Component Diagram (Mermaid)

```mermaid
flowchart TB
    subgraph Leader["🚗 Leader Vehicle (lead.cpp)"]
        LS[Server Socket<br/>Port 8080]
        LM[Mutex Lock]
        LC[Matrix Clock<br/>MAX_NODES x MAX_NODES]
        
        subgraph LThreads["Threads"]
            AT[Accept Thread]
            BT[Broadcast Thread]
            MT[Monitor Thread]
            HT[Handler Threads<br/>per follower]
        end
        
        LState["State:<br/>• position<br/>• speed<br/>• obstacle flag<br/>• degraded mode"]
    end

    subgraph Follower["🚙 Follower Vehicle (follow.cpp)"]
        FS[TCP Socket]
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
        Listening --> AcceptingFollowers : accept()
        AcceptingFollowers --> Broadcasting : follower joined
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
        Main1[main] --> Accept[acceptLoop]
        Main1 --> Broadcast[broadcastLoop]
        Main1 --> Monitor[monitorLoop]
        Accept --> Handler1[handleFollower 1]
        Accept --> Handler2[handleFollower 2]
        Accept --> HandlerN[handleFollower N]
    end

    subgraph Follower
        direction TB
        Main2[main] --> Recv[recvLoop]
        Main2 --> Send[sendLoop]
        Main2 --> Input[inputLoop]
    end

    Handler1 -.->|mutex| State1[(Shared State)]
    Handler2 -.->|mutex| State1
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
├── message.h       # Shared wire protocol (WireMessage, MsgType, Flags)
├── lead.h          # Leader class declaration
├── lead.cpp        # Leader implementation
├── follow.h        # Follower class declaration
├── follow.cpp      # Follower implementation
├── README_RUN.md   # Build & run instructions
└── ARCHITECTURE.md # This file
```

---

## Quick Reference

| Component | Port | Protocol | Key Features |
|-----------|------|----------|--------------|
| Leader    | 8080 | TCP      | Accept followers, broadcast state, monitor heartbeats |
| Follower  | -    | TCP      | Connect to leader, PID control, send heartbeat |

| Message Type    | Direction       | Purpose |
|-----------------|-----------------|---------|
| JOIN            | Follower→Leader | Request to join platoon |
| JOIN_ACK        | Leader→Follower | Assign clock index |
| LEAVE           | Follower→Leader | Graceful departure |
| LEADER_STATE    | Leader→Follower | Broadcast position/speed |
| FOLLOWER_STATE  | Follower→Leader | Heartbeat + state update |
