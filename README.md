# Distributed Platoon System v2

## Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    DISTRIBUTED + PARALLEL ARCHITECTURE                       │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  ┌──────────────────┐    ┌──────────────────┐    ┌──────────────────┐       │
│  │  PROCESS: V0     │    │  PROCESS: V1     │    │  PROCESS: V2     │       │
│  │  (Leader)        │    │  (Follower)      │    │  (Follower)      │       │
│  │                  │    │                  │    │                  │       │
│  │  ┌────────────┐  │    │  ┌────────────┐  │    │  ┌────────────┐  │       │
│  │  │ Comm Thread│◄─┼────┼──│ Comm Thread│◄─┼────┼──│ Comm Thread│  │       │
│  │  └────────────┘  │    │  └────────────┘  │    │  └────────────┘  │       │
│  │        │         │    │        │         │    │        │         │       │
│  │  ┌────────────┐  │    │  ┌────────────┐  │    │  ┌────────────┐  │       │
│  │  │Sensor Thrd │  │    │  │Sensor Thrd │  │    │  │Sensor Thrd │  │       │
│  │  └────────────┘  │    │  └────────────┘  │    │  └────────────┘  │       │
│  │        │         │    │        │         │    │        │         │       │
│  │  ┌────────────┐  │    │  ┌────────────┐  │    │  ┌────────────┐  │       │
│  │  │Decision Thrd│ │    │  │Decision Thrd│ │    │  │Decision Thrd│ │       │
│  │  └────────────┘  │    │  └────────────┘  │    │  └────────────┘  │       │
│  │        │         │    │        │         │    │        │         │       │
│  │  ┌────────────┐  │    │  ┌────────────┐  │    │  ┌────────────┐  │       │
│  │  │Control Thrd│  │    │  │Control Thrd│  │    │  │Control Thrd│  │       │
│  │  └────────────┘  │    │  └────────────┘  │    │  └────────────┘  │       │
│  └────────┬─────────┘    └────────┬─────────┘    └────────┬─────────┘       │
│           │                       │                       │                  │
│           └───────────────────────┴───────────────────────┘                  │
│                                   │                                          │
│                    ┌──────────────▼──────────────┐                           │
│                    │   POSIX MESSAGE QUEUES      │                           │
│                    │   /platoon_v0               │                           │
│                    │   /platoon_v1               │                           │
│                    │   /platoon_v2               │                           │
│                    │   (Simulates DSRC/V2V)      │                           │
│                    └─────────────────────────────┘                           │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

## Distributed Properties

| Aspect | Implementation |
|--------|----------------|
| **Process Isolation** | Each vehicle is a separate process (`./vehicle 0 4`, `./vehicle 1 4`, ...) |
| **No Shared Memory** | Processes don't share memory, communicate only via IPC |
| **Message Passing** | POSIX Message Queues (`/dev/mqueue/platoon_v*`) |
| **Fault Tolerance** | If one vehicle crashes, others continue running and elect a new leader |
| **Independent Execution** | Can run vehicles on different machines (replace MQ with sockets) |

## Parallel Properties

| Aspect | Implementation |
|--------|----------------|
| **Multi-threaded** | Each vehicle has 4 threads running in parallel |
| **Thread Pipeline** | Sensor → Decision → Control (data flow) |
| **Concurrent Comm** | Comm thread handles IPC independently from control |
| **Shared State** | Threads within a process share `g_vehicle` (protected by mutex) |
| **Synchronization** | `pthread_mutex_t`, `pthread_cond_t` |

## Threads per Vehicle

```
┌─────────────────────────────────────────────────────────────────┐
│                    VEHICLE PROCESS                               │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌──────────────┐   ┌──────────────┐   ┌──────────────┐         │
│  │ COMM THREAD  │   │SENSOR THREAD │   │CONTROL THREAD│         │
│  │              │   │              │   │              │         │
│  │ • Recv msgs  │   │ • Lidar sim  │   │ • Apply accel│         │
│  │ • Send HB    │   │ • Gap measure│   │ • Update pos │         │
│  │ • Coupling   │   │ • Obstacle   │   │ • Speed limit│         │
│  └──────┬───────┘   └──────┬───────┘   └──────▲───────┘         │
│         │                  │                  │                  │
│         │                  ▼                  │                  │
│         │         ┌──────────────┐            │                  │
│         └────────►│DECISION THRD │────────────┘                  │
│                   │              │                               │
│                   │ • Coupling   │                               │
│                   │ • Emergency  │                               │
│                   │ • Election   │                               │
│                   │ • PD control │                               │
│                   └──────────────┘                               │
│                          │                                       │
│                          ▼                                       │
│                   ┌──────────────┐                               │
│                   │ Shared State │                               │
│                   │  g_vehicle   │                               │
│                   │  (mutex)     │                               │
│                   └──────────────┘                               │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

## Build & Run

```bash
# Build
cd platoon_v2
make

# Option 1: Run all vehicles via launcher
./launcher 4          # 4 vehicles, output in 1 terminal
./launcher 4 -t       # 4 vehicles, each in separate xterm window

# Option 2: Run each vehicle separately (open 4 terminals)
# Terminal 1:
./vehicle 0 4    # Leader

# Terminal 2:
./vehicle 1 4    # Follower 1

# Terminal 3:
./vehicle 2 4    # Follower 2

# Terminal 4:
./vehicle 3 4    # Follower 3
```

## Comparison with v1

| Feature | v1 (platoon/) | v2 (platoon_v2/) |
|---------|---------------|------------------|
| **Distributed** | ❌ All in 1 process | ✅ Each vehicle = 1 process |
| **IPC** | In-memory queues | POSIX Message Queues |
| **Parallel** | 1 thread/vehicle | 4 threads/vehicle |
| **Fault Tolerance** | Process crash = all die | 1 vehicle crash, others continue |
| **Scalability** | Single machine | Can extend to multi-machine |

## Files

```
platoon_v2/
├── common.h      - Shared types, constants
├── ipc.h/c       - POSIX Message Queue wrapper
├── threads.h/c   - 4 thread implementations
├── vehicle.c     - Single vehicle process entry point
├── launcher.c    - Process launcher utility
├── Makefile      - Build system
└── README.md     - This file
```
