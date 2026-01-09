# Distributed Platoon System v2 - Diagrams

## 1. System Architecture Overview

```mermaid
graph TB
    subgraph "Distributed Platoon System"
        subgraph "Process: Vehicle 0 (Leader)"
            V0_COMM[Comm Thread]
            V0_SENSOR[Sensor Thread]
            V0_DECISION[Decision Thread]
            V0_CONTROL[Control Thread]
            V0_STATE[(Shared State<br/>g_vehicle)]
        end
        
        subgraph "Process: Vehicle 1 (Follower)"
            V1_COMM[Comm Thread]
            V1_SENSOR[Sensor Thread]
            V1_DECISION[Decision Thread]
            V1_CONTROL[Control Thread]
            V1_STATE[(Shared State<br/>g_vehicle)]
        end
        
        subgraph "Process: Vehicle N (Follower)"
            VN_COMM[Comm Thread]
            VN_SENSOR[Sensor Thread]
            VN_DECISION[Decision Thread]
            VN_CONTROL[Control Thread]
            VN_STATE[(Shared State<br/>g_vehicle)]
        end
        
        subgraph "POSIX Message Queues (IPC)"
            MQ0[/platoon_v0/]
            MQ1[/platoon_v1/]
            MQN[/platoon_vN/]
        end
    end
    
    V0_COMM <-->|"read/write"| MQ0
    V0_COMM -->|"send"| MQ1
    V0_COMM -->|"send"| MQN
    
    V1_COMM -->|"send"| MQ0
    V1_COMM <-->|"read/write"| MQ1
    V1_COMM -->|"send"| MQN
    
    VN_COMM -->|"send"| MQ0
    VN_COMM -->|"send"| MQ1
    VN_COMM <-->|"read/write"| MQN
    
    style V0_STATE fill:#f9f,stroke:#333
    style V1_STATE fill:#f9f,stroke:#333
    style VN_STATE fill:#f9f,stroke:#333
```

---

## 2. Thread Pipeline (Per Vehicle)

```mermaid
flowchart LR
    subgraph "Single Vehicle Process"
        direction TB
        
        subgraph SENSOR["🔍 Sensor Thread"]
            S1[Measure Gap<br/>to Predecessor]
            S2[Calculate<br/>Relative Speed]
            S3[Obstacle<br/>Detection]
        end
        
        subgraph DECISION["🧠 Decision Thread"]
            D1[Emergency<br/>Check]
            D2[Coupling<br/>Logic]
            D3[Leader<br/>Election]
            D4[PD Controller<br/>Calculate Accel]
        end
        
        subgraph CONTROL["⚙️ Control Thread"]
            C1[Apply<br/>Acceleration]
            C2[Update<br/>Position/Speed]
            C3[Speed<br/>Limiting]
        end
        
        subgraph COMM["📡 Comm Thread"]
            CM1[Receive<br/>Messages]
            CM2[Send<br/>Heartbeats]
            CM3[Process<br/>Requests]
        end
        
        STATE[(Shared State<br/>VehicleData)]
    end
    
    S1 --> STATE
    S2 --> STATE
    S3 --> STATE
    
    STATE --> D1
    STATE --> D2
    STATE --> D3
    STATE --> D4
    
    D4 --> STATE
    
    STATE --> C1
    C1 --> C2
    C2 --> STATE
    
    CM1 --> STATE
    STATE --> CM2
    CM3 --> STATE
    
    IPC[("🌐 IPC<br/>Message Queues")]
    
    CM1 <--> IPC
    CM2 --> IPC
```

---

## 3. Message Flow Sequence Diagram

```mermaid
sequenceDiagram
    participant V0 as Vehicle 0 (Leader)
    participant MQ as Message Queues
    participant V1 as Vehicle 1 (Follower)
    participant V2 as Vehicle 2 (Follower)
    
    Note over V0,V2: === Startup & Coupling ===
    
    V0->>MQ: Create /platoon_v0
    V1->>MQ: Create /platoon_v1
    V2->>MQ: Create /platoon_v2
    
    V1->>MQ: COUPLE_REQUEST to V0
    MQ->>V0: COUPLE_REQUEST from V1
    V0->>MQ: COUPLE_ACCEPT to V1
    MQ->>V1: COUPLE_ACCEPT (pred=V0)
    V1->>V1: State = COUPLED
    
    V2->>MQ: COUPLE_REQUEST to V0
    MQ->>V0: COUPLE_REQUEST from V2
    V0->>MQ: COUPLE_ACCEPT to V2
    MQ->>V2: COUPLE_ACCEPT (pred=V1)
    V2->>V2: State = COUPLED
    
    Note over V0,V2: === Normal Operation ===
    
    loop Every 1 second
        V0->>MQ: HEARTBEAT (broadcast)
        MQ->>V1: HEARTBEAT from V0
        MQ->>V2: HEARTBEAT from V0
        
        V1->>MQ: HEARTBEAT (broadcast)
        MQ->>V0: HEARTBEAT from V1
        MQ->>V2: HEARTBEAT from V1
        
        V2->>MQ: HEARTBEAT (broadcast)
        MQ->>V0: HEARTBEAT from V2
        MQ->>V1: HEARTBEAT from V2
    end
    
    Note over V0,V2: === Emergency Scenario ===
    
    V1->>V1: Obstacle Detected!
    V1->>MQ: EMERGENCY_BRAKE (broadcast)
    MQ->>V0: EMERGENCY_BRAKE
    MQ->>V2: EMERGENCY_BRAKE
    V0->>V0: Emergency Stop
    V2->>V2: Emergency Stop
```

---

## 4. Leader Election Sequence

```mermaid
sequenceDiagram
    participant V0 as Vehicle 0 (Leader)
    participant V1 as Vehicle 1
    participant V2 as Vehicle 2
    participant V3 as Vehicle 3
    
    Note over V0: Leader crashes or exits
    V0-xV0: Process Exit
    
    Note over V1,V3: Heartbeat timeout (5s)
    
    V1->>V1: No HB from V0 > 50 cycles
    V2->>V2: No HB from V0 > 50 cycles
    V3->>V3: No HB from V0 > 50 cycles
    
    V1->>V2: LEADER_ELECTION
    V1->>V3: LEADER_ELECTION
    V2->>V1: LEADER_ELECTION
    V2->>V3: LEADER_ELECTION
    V3->>V1: LEADER_ELECTION
    V3->>V2: LEADER_ELECTION
    
    Note over V1,V3: Wait 500ms for responses
    
    V1->>V1: Check: Am I lowest active ID?
    Note over V1: Yes! V0 is dead, I'm V1
    
    V1->>V1: Become LEADER
    V1->>V2: LEADER_ANNOUNCE
    V1->>V3: LEADER_ANNOUNCE
    
    V2->>V2: leader_id = V1
    V3->>V3: leader_id = V1
    
    Note over V1,V3: Platoon continues with V1 as leader
```

---

## 5. Vehicle State Machine

```mermaid
stateDiagram-v2
    [*] --> IDLE: Initialize
    
    IDLE --> REQUESTING: Send COUPLE_REQUEST
    REQUESTING --> COUPLED: Receive COUPLE_ACCEPT
    REQUESTING --> IDLE: Timeout / COUPLE_REJECT
    
    COUPLED --> DECOUPLING: Send DECOUPLE_REQUEST
    DECOUPLING --> IDLE: Receive DECOUPLE_ACK
    
    COUPLED --> EMERGENCY: Gap < 3m OR Obstacle
    IDLE --> EMERGENCY: Critical Event
    REQUESTING --> EMERGENCY: Critical Event
    
    EMERGENCY --> COUPLED: Resume (after safe)
    EMERGENCY --> IDLE: System Reset
    
    note right of IDLE: Follower not yet in platoon
    note right of COUPLED: Active platoon member
    note right of EMERGENCY: Emergency braking active
```

---

## 6. Component Diagram

```mermaid
graph TB
    subgraph "Source Files"
        LAUNCHER[launcher.c<br/>Process Manager]
        VEHICLE[vehicle.c<br/>Main Entry Point]
        THREADS[threads.c<br/>Thread Functions]
        IPC[ipc.c<br/>Message Queue]
        COMMON[common.h<br/>Shared Definitions]
    end
    
    subgraph "Build Output"
        PLATOON[platoon<br/>Launcher Binary]
        VEHICLE_BIN[vehicle<br/>Vehicle Binary]
    end
    
    subgraph "OS Resources"
        MQ[POSIX Message Queues<br/>/dev/mqueue/platoon_v*]
        PTHREADS[POSIX Threads]
    end
    
    LAUNCHER --> PLATOON
    VEHICLE --> VEHICLE_BIN
    THREADS --> VEHICLE_BIN
    IPC --> VEHICLE_BIN
    COMMON --> VEHICLE_BIN
    COMMON --> PLATOON
    
    PLATOON -->|"fork()"| VEHICLE_BIN
    VEHICLE_BIN --> MQ
    VEHICLE_BIN --> PTHREADS
    
    style COMMON fill:#ffd,stroke:#333
    style MQ fill:#dff,stroke:#333
    style PTHREADS fill:#dff,stroke:#333
```

---

## 7. Data Flow Diagram

```mermaid
flowchart TB
    subgraph "External Inputs"
        LIDAR[("📡 LiDAR<br/>(Simulated)")]
        RADAR[("📡 Radar<br/>(Simulated)")]
        CAMERA[("📷 Camera<br/>(Simulated)")]
    end
    
    subgraph "Sensor Thread"
        SENSE[Process<br/>Sensor Data]
    end
    
    subgraph "Shared State"
        GAP[sensor_gap]
        REL_SPD[sensor_rel_speed]
        OBSTACLE[sensor_obstacle]
        CMD_ACCEL[cmd_accel]
        STATE[Vehicle State]
    end
    
    subgraph "Decision Thread"
        PD[PD Controller<br/>Kp=0.5, Kd=0.3]
        EMERGENCY[Emergency<br/>Logic]
        ELECTION[Election<br/>Logic]
    end
    
    subgraph "Control Thread"
        APPLY[Apply<br/>Acceleration]
        DYNAMICS[Update<br/>Dynamics]
    end
    
    subgraph "Vehicle Dynamics"
        POS[Position]
        SPD[Speed]
        ACCEL[Acceleration]
    end
    
    LIDAR --> SENSE
    RADAR --> SENSE
    CAMERA --> SENSE
    
    SENSE --> GAP
    SENSE --> REL_SPD
    SENSE --> OBSTACLE
    
    GAP --> PD
    REL_SPD --> PD
    OBSTACLE --> EMERGENCY
    
    PD --> CMD_ACCEL
    EMERGENCY --> STATE
    ELECTION --> STATE
    
    CMD_ACCEL --> APPLY
    APPLY --> DYNAMICS
    
    DYNAMICS --> POS
    DYNAMICS --> SPD
    DYNAMICS --> ACCEL
```

---

## 8. Platoon Formation

```mermaid
graph LR
    subgraph "Physical Platoon Layout"
        direction LR
        
        V0["🚗 Vehicle 0<br/>LEADER<br/>pos: 0m"]
        V1["🚙 Vehicle 1<br/>FOLLOWER<br/>pos: -10m"]
        V2["🚙 Vehicle 2<br/>FOLLOWER<br/>pos: -20m"]
        V3["🚙 Vehicle 3<br/>FOLLOWER<br/>pos: -30m"]
        
        V0 ---|"Gap: 10m"| V1
        V1 ---|"Gap: 10m"| V2
        V2 ---|"Gap: 10m"| V3
    end
    
    subgraph "Predecessor Chain"
        P0["V0<br/>pred: -1"]
        P1["V1<br/>pred: V0"]
        P2["V2<br/>pred: V1"]
        P3["V3<br/>pred: V2"]
        
        P0 --> P1
        P1 --> P2
        P2 --> P3
    end
    
    style V0 fill:#faa,stroke:#333
    style V1 fill:#aaf,stroke:#333
    style V2 fill:#aaf,stroke:#333
    style V3 fill:#aaf,stroke:#333
```

---

## 9. Process Spawning (Launcher)

```mermaid
flowchart TB
    START([Start Launcher])
    PARSE[Parse Arguments<br/>num_vehicles]
    CLEANUP[Cleanup Old<br/>Message Queues]
    
    subgraph FORK_LOOP["Fork Loop (i = 0 to N-1)"]
        FORK[fork()]
        CHILD{Child?}
        EXEC[exec ./vehicle i N]
        SAVE_PID[Save PID]
        DELAY[Sleep 200ms]
    end
    
    MONITOR[Monitor Loop]
    CHECK_CHILD{Child<br/>Exited?}
    CHECK_SIGNAL{SIGINT<br/>Received?}
    LOG[Log Exit]
    
    STOP_ALL[Send SIGTERM<br/>to All Children]
    WAIT_ALL[waitpid() for All]
    CLEANUP_END[Cleanup Queues]
    DONE([Exit])
    
    START --> PARSE
    PARSE --> CLEANUP
    CLEANUP --> FORK
    
    FORK --> CHILD
    CHILD -->|"Yes (pid=0)"| EXEC
    CHILD -->|"No (pid>0)"| SAVE_PID
    SAVE_PID --> DELAY
    DELAY --> FORK
    
    DELAY -.->|"Loop complete"| MONITOR
    
    MONITOR --> CHECK_CHILD
    CHECK_CHILD -->|"Yes"| LOG
    LOG --> MONITOR
    CHECK_CHILD -->|"No"| CHECK_SIGNAL
    CHECK_SIGNAL -->|"No"| MONITOR
    CHECK_SIGNAL -->|"Yes"| STOP_ALL
    
    STOP_ALL --> WAIT_ALL
    WAIT_ALL --> CLEANUP_END
    CLEANUP_END --> DONE
```

---

## 10. PD Controller Block Diagram

```mermaid
flowchart LR
    subgraph "Inputs"
        DESIRED[Desired Gap<br/>10m]
        ACTUAL[Actual Gap<br/>sensor_gap]
        REL_V[Relative Speed<br/>sensor_rel_speed]
    end
    
    subgraph "PD Controller"
        ERROR["Error<br/>e = gap - desired"]
        KP["Kp × e<br/>(Kp = 0.5)"]
        KD["Kd × rel_speed<br/>(Kd = 0.3)"]
        SUM["Σ"]
        CLAMP["Clamp<br/>±3 m/s²"]
    end
    
    subgraph "Output"
        CMD[cmd_accel]
    end
    
    DESIRED --> ERROR
    ACTUAL --> ERROR
    ERROR --> KP
    REL_V --> KD
    KP --> SUM
    KD --> SUM
    SUM --> CLAMP
    CLAMP --> CMD
    
    style DESIRED fill:#dfd,stroke:#333
    style ACTUAL fill:#ffd,stroke:#333
    style CMD fill:#fdd,stroke:#333
```

---

## 11. Synchronization Primitives

```mermaid
graph TB
    subgraph "Thread Synchronization"
        MUTEX[("pthread_mutex_t<br/>g_vehicle.mutex")]
        COND_S[("pthread_cond_t<br/>sensor_ready")]
        COND_D[("pthread_cond_t<br/>decision_ready")]
    end
    
    subgraph "Threads"
        COMM[Comm Thread]
        SENSOR[Sensor Thread]
        DECISION[Decision Thread]
        CONTROL[Control Thread]
    end
    
    subgraph "Protected Data"
        DATA[("VehicleData<br/>g_vehicle")]
    end
    
    COMM -->|"lock/unlock"| MUTEX
    SENSOR -->|"lock/unlock"| MUTEX
    DECISION -->|"lock/unlock"| MUTEX
    CONTROL -->|"lock/unlock"| MUTEX
    
    MUTEX -->|"protects"| DATA
    
    SENSOR -->|"signal"| COND_S
    DECISION -->|"wait"| COND_S
    DECISION -->|"signal"| COND_D
    CONTROL -->|"wait"| COND_D
```

---

## 12. Message Types

```mermaid
graph TB
    subgraph "Message Categories"
        subgraph "Status Messages"
            HB[MSG_HEARTBEAT<br/>Periodic alive signal]
            STATE[MSG_STATE_UPDATE<br/>Position/Speed update]
        end
        
        subgraph "Coupling Messages"
            CREQ[MSG_COUPLE_REQUEST<br/>Join platoon request]
            CACC[MSG_COUPLE_ACCEPT<br/>Request approved]
            CREJ[MSG_COUPLE_REJECT<br/>Request denied]
            DREQ[MSG_DECOUPLE_REQUEST<br/>Leave platoon]
            DACK[MSG_DECOUPLE_ACK<br/>Leave confirmed]
        end
        
        subgraph "Emergency Messages"
            EMG[MSG_EMERGENCY_BRAKE<br/>Critical stop]
        end
        
        subgraph "Election Messages"
            ELEC[MSG_LEADER_ELECTION<br/>Start election]
            ANN[MSG_LEADER_ANNOUNCE<br/>New leader]
        end
    end
    
    style HB fill:#dfd,stroke:#333
    style EMG fill:#fdd,stroke:#333
    style ELEC fill:#ffd,stroke:#333
```

---

## 13. Deployment Diagram

```mermaid
graph TB
    subgraph "Development Machine"
        subgraph "Build System"
            MAKE[Makefile]
            GCC[GCC Compiler]
        end
        
        subgraph "Executables"
            PLATOON[platoon]
            VEHICLE[vehicle]
        end
        
        subgraph "Terminal Windows"
            T1["Terminal 1<br/>./platoon 4"]
            T2["Terminal 2<br/>./vehicle 0 4"]
            T3["Terminal 3<br/>./vehicle 1 4"]
            T4["Terminal 4<br/>./vehicle 2 4"]
            T5["Terminal 5<br/>./vehicle 3 4"]
        end
        
        subgraph "System Resources"
            MQ["/dev/mqueue/<br/>platoon_v0..v3"]
        end
    end
    
    MAKE --> GCC
    GCC --> PLATOON
    GCC --> VEHICLE
    
    T1 -->|"or"| PLATOON
    T2 --> VEHICLE
    T3 --> VEHICLE
    T4 --> VEHICLE
    T5 --> VEHICLE
    
    VEHICLE --> MQ
    
    style MQ fill:#dff,stroke:#333
```

---

## 14. Class Diagram (Data Structures)

```mermaid
classDiagram
    class Message {
        +MessageType type
        +int sender_id
        +int target_id
        +uint32_t seq_num
        +uint32_t timestamp
        +double pos
        +double speed
        +double accel
        +int priority
        +int predecessor_id
    }
    
    class VehicleData {
        +int id
        +VehicleRole role
        +VehicleState state
        +double pos
        +double speed
        +double accel
        +int leader_id
        +int predecessor_id
        +int num_vehicles
        +uint32_t seq_num
        +uint32_t last_heartbeat[]
        +bool running
        +bool election_in_progress
        +double sensor_gap
        +double sensor_rel_speed
        +bool sensor_obstacle
        +double cmd_accel
        +bool cmd_emergency
        +pthread_mutex_t mutex
        +pthread_cond_t sensor_ready
        +pthread_cond_t decision_ready
    }
    
    class VehicleThreads {
        +pthread_t comm
        +pthread_t sensor
        +pthread_t decision
        +pthread_t control
    }
    
    class MessageType {
        <<enumeration>>
        MSG_HEARTBEAT
        MSG_COUPLE_REQUEST
        MSG_COUPLE_ACCEPT
        MSG_COUPLE_REJECT
        MSG_DECOUPLE_REQUEST
        MSG_DECOUPLE_ACK
        MSG_EMERGENCY_BRAKE
        MSG_LEADER_ELECTION
        MSG_LEADER_ANNOUNCE
        MSG_STATE_UPDATE
    }
    
    class VehicleRole {
        <<enumeration>>
        ROLE_LEADER
        ROLE_FOLLOWER
    }
    
    class VehicleState {
        <<enumeration>>
        STATE_IDLE
        STATE_REQUESTING
        STATE_COUPLED
        STATE_DECOUPLING
        STATE_EMERGENCY
    }
    
    VehicleData --> VehicleRole
    VehicleData --> VehicleState
    Message --> MessageType
    VehicleData --> VehicleThreads : uses
```

---

## 15. Timing Diagram

```mermaid
gantt
    title Thread Execution Timeline (100ms cycle)
    dateFormat X
    axisFormat %L ms
    
    section Comm Thread
    Receive Messages    :0, 10
    Process Messages    :10, 20
    Send Heartbeat     :20, 30
    Update Counters    :30, 40
    Sleep              :40, 100
    
    section Sensor Thread
    Measure Gap        :0, 15
    Measure Rel Speed  :15, 25
    Obstacle Check     :25, 35
    Signal Ready       :35, 40
    Sleep (50ms)       :40, 90
    
    section Decision Thread
    Wait Sensor        :0, 40
    Emergency Check    :40, 50
    Election Check     :50, 60
    PD Calculate       :60, 75
    Signal Ready       :75, 80
    Sleep              :80, 100
    
    section Control Thread
    Wait Decision      :0, 80
    Apply Accel        :80, 85
    Update Dynamics    :85, 95
    Sleep              :95, 100
```

---

## Viewing These Diagrams

These diagrams use **Mermaid** syntax. To view them:

1. **VS Code**: Install "Markdown Preview Mermaid Support" extension
2. **GitHub**: Diagrams render automatically in markdown files
3. **Online**: Use [Mermaid Live Editor](https://mermaid.live/)
4. **Export**: Use `mmdc` CLI tool to export as PNG/SVG

```bash
# Install mermaid CLI
npm install -g @mermaid-js/mermaid-cli

# Export diagram
mmdc -i DIAGRAMS.md -o output.png
```
