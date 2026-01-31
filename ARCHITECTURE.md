# Platooning System Architecture (Simplified)

This document explains the high-level code flow and interaction between the components in the Platooning System.

## 1. System Components

The system consists of two main entity types:
1.  **Leading Vehicle (`lead.cpp`)**: The head of the platoon. It determines the speed, direction, and broadcasts the global state. It also performs heavy computation (AEB) using OpenCL.
2.  **Following Vehicle (`follow.cpp`)**: Vehicles that follow the leader (or the vehicle immediately in front). In case of Leader departure, a Follower can **promote** itself to become the new Leader.

## 2. Data Structures (`vehicle.h` & `message.h`)

### `VehicleInfo`
Holds the core state of a single vehicle:
- `id`: Unique identifier (e.g., 1, 2, 3).
- `mode`: `LeaderMode` or `FollowerMode`.
- `position`, `speed`: Physics state.
- `ipAddress`, `port`: Network addressing for UDP.

### `PlatoonState`
Maintained by the Leader and synchronized to all Followers:
- `vehicles`: A `std::vector` of `VehicleInfo` representing the current topology (sorted by position).
- `leaderId`: ID of the current leader.

### Messages (UDP)
- `STATUS_UPDATE`: Followers send their Position/Speed to Leader (5Hz).
- `PLATOON_STATE`: Leader broadcasts the full vehicle list to everyone.
- `COUPLE_COMMAND`: Follower requests to join.
- `ALERTS`: Critical events like `TRAFFIC_LIGHT`, `OBSTACLE`, `LEAVE_PLATOON`.

## 3. Leader Logic (`lead.cpp`)

The Leader runs multiple threads:

1.  **Receive Thread (`recvThread`)**:
    - Listens on `SERVER_PORT` (5000).
    - Accepts `COUPLE_COMMAND` -> Adds new vehicle to list.
    - Accepts `STATUS_UPDATE` -> Updates position/speed of followers in the list.
    
2.  **Run Thread (`runThread`)**:
    - Updates physics (Position = Speed * TimeInSeconds).
    - Checks for AEB (OpenCL).

3.  **Broadcast Thread (`sendStatusThread`)**:
    - Periodically sends `PLATOON_STATE` (the "Single Source of Truth") to all followers.
    
4.  **Heartbeat Thread**:
    - Checks `lastHeartbeatMs` of every follower.
    - If a follower is silent > 5s, it is removed from the list.

5.  **Event Thread**:
    - Reads user input (FIFO) for events like "Leave Platoon".

**OpenCL AEB**:
- `initOpenCL_AEB()`: Compiles a Ray Tracing kernel.
- `scanEnvironmentWithGPU()`: Executes kernel to detect obstacles in parallel (720 rays).

## 4. Follower Logic (`follow.cpp`)

The Follower is a state machine controlled by several threads plus a PID controller:

1.  **State Machine**:
    - `NORMAL`: Following using PID control to maintain `SAFE_DISTANCE` (20m).
    - `CATCHING_UP`: Speeding up to close a large gap.
    - `STOPPING`/`STOPPED`: Reacting to Red Light or Obstacles.
    - `DECOUPLED`: Temporarily lost or manually detached.

2.  **Receive Thread (`recvThread`)**:
    - Receives `PLATOON_STATE`: Knows who is Leader, who is Front Vehicle.
    - **Self-Correction**: If it sees it is missing from the list, it auto-sends a `COUPLE` request (Rejoin logic).
    - **Transition Trigger**: If it receives `LEAVE_PLATOON` from the current Leader, it checks if it is the "Next in Line" (Follower 2). If so, it calls `transitionToLeader()`.

3.  **Transition Logic (`transitionToLeader`)**:
    - Stops "Follower Threads" (to avoid packet stealing).
    - Binds to the Leader Port (5000).
    - Promotes self to `LeaderMode`.
    - Retains existing list of followers (preventing topology break).
    - Starts "Leader Threads".
    - **Crucial Fix**: Ensures old threads are fully stopped before starting new ones to prevent "Zombie Thread" issues where packets are processed by the wrong handler.

## 5. Network Flow Diagram

```text
      [Leader L1]             [Follower F2]             [Follower F3]
           |                        |                         |
           |                        |                         |
   (Normal Operation)               |                         |
           |                        |                         |
           |<--- STATUS_UPDATE -----|                         |
           |<-------------------------------- STATUS_UPDATE --|
           |                        |                         |
           |---- PLATOON_STATE ---->|                         |
           |-------------------------------- PLATOON_STATE -->|
           |                        |                         |
           |                        |                         |
 (Leader Leaves - Event 7)          |                         |
           |---- LEAVE_PLATOON ---->|                         |
           |-------------------------------- LEAVE_PLATOON -->|
      [Shutdown]                    |                         |
                                    |                         |
                          (Detects Next Leader)               |
                                    |                         |
                           [transitionToLeader]               |
                             [Bind Port 5000]                 |
                                    |                         |
                                    |                         |
                         (New Era - F2 is Leader)             |
                                    |                         |
                                    |<---- STATUS_UPDATE -----|
                                    |                         |
                                    |----- PLATOON_STATE ---->|
```

## 6. Event Processing Diagram

This diagram charts the lifecycle of a system event (e.g., "Traffic Light Red", "Leader Leave") from user input to system reaction.

```text
[User / Script]
      |
      v (writes int to FIFO)
[/tmp/leader_event_fifo]
      |
      | (read by)
      v
[Leader: eventSimulationThread]
      | checks event type (e.g., RED_LIGHT)
      | updates local state (e.g., setState(STOPPING))
      | 
      | pushes EventMessage
      v
[Leader: eventQueue_ (Mutex Protected)]
      |
      | (popped by)
      v
[Leader: eventSenderThread]
      |
      | broadcasts / unicasts (UDP)
      v
[Network Layer]
      |
      | (recvfrom)
      v
[Follower: recvThread] ----> [Leader: recvThread] (If Applicable)
      |                         |
      v                         v
(Process Message)         (Process Message)
  - TRAFFIC_LIGHT           - STATUS_UPDATE
  - LEAVE_PLATOON           - COUPLE_COMMAND
      |                         |
      v                         v
[Update Local State]      [Update PlatoonState]
(e.g., STOPPING)          (e.g., Add/Remove Vehicle)
```

## 7. Speed Adjustment Diagram (runThread)

Each vehicle runs a physics loop (`runThread`) that adjusts speed based on the current state and target.

### Leader Speed Logic
```text
   [Start Loop]
        |
   [Lidar Check (OpenCL)] --> [Obstacle?] --(Yes)--> [State = STOPPING]
        |
   [Check State]
        |
    +---+-------+-------+
    |           |       |
[NORMAL]    [STOPPING] [STARTING]
    |           |       |
Target=SetSpd   |    Spd += Accel
Spd -> Target   |       |
    |      Spd -= Decel |
    |           |       |
    +-----------+-------+
        |
   [Update Position] (x += v * dt)
```

### Follower Speed Logic (simplified)
```text
   [Start Loop]
        |
   [Read Platoon Data] (Leader Pos, Front Vehicle Pos)
        |
   [Calculate Gap] (RefPos - MyPos)
        |
   [State / PID Check]
        |
   (Gap < SafeDist?) ----(Yes)---> [State = STOPPING] (Brake)
        |
   (Gap > Large?) -------(Yes)---> [State = CATCHING_UP] (Accelerate)
        |
   (Otherwise) ------------------> [State = NORMAL] (Match Ref Speed)
        |
   [Update Speed] (Apply Accel/Decel towards Target)
        |
   [Update Position]
```
