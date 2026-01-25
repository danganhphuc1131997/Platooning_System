# Leader Departure and Transition Implementation

## Overview
This implementation handles the safe departure of a platoon leader and the automatic promotion of the next follower to become the new leader.

## Key Features

### 1. Leader Departure Process
When a leader decides to leave the platoon:

1. **Notification Phase**
   - Leader sends `LEAVE_PLATOON` message to the vehicle immediately behind it
   - Message includes the departing leader's ID and timestamp

2. **State Update**
   - Leader removes itself from the platoon state
   - Broadcasts final platoon state to all remaining vehicles
   - Adds 500ms delay to ensure message delivery

3. **Shutdown**
   - All leader threads are gracefully stopped
   - Server socket is closed
   - Leader exits the system

### 2. Follower-to-Leader Transition
When a follower receives `LEAVE_PLATOON` from the current leader:

1. **Transition Detection**
   - Follower checks if the leaving vehicle is the current platoon leader
   - If yes, initiates transition to leader mode

2. **Follower Cleanup**
   - Stops all follower threads (`clientRunning_ = false`)
   - Closes follower socket connections
   - Preserves current position and speed

3. **Leader Initialization**
   - Uses `execlp()` to replace the current process with the leader program
   - Passes vehicle ID, current position, and current speed as arguments
   - New leader process starts with inherited state

4. **Seamless Handover**
   - New leader maintains the same ID, position, and speed
   - Existing followers automatically recognize the new leader
   - Display switches from follower view to leader view
   - All leader responsibilities are assumed (status broadcasting, platoon management, etc.)

## Command Line Arguments

### Leader Program
```bash
./lead [id] [position] [speed]
```
- **id**: Vehicle ID (default: LEADER_INITIAL_ID)
- **position**: Initial position in meters (default: LEADER_INITIAL_POSITION)
- **speed**: Initial speed in m/s (default: LEADER_INITIAL_SPEED)

### Follower Program
```bash
./follow [id] [position]
```
- **id**: Vehicle ID (default: FOLLOWER_INITIAL_ID)
- **position**: Initial position in meters (default: FOLLOWER_INITIAL_POSITION)

## Message Flow

### Normal Leader Departure
```
Leader (ID=0)
    |
    | LEAVE_PLATOON message
    v
Follower (ID=1) → Becomes Leader (ID=1)
    |
    | STATUS_UPDATE, PLATOON_STATE
    v
Follower (ID=2)
    |
    v
Follower (ID=3)
```

### After Transition
```
New Leader (ID=1)
    |
    | STATUS_UPDATE, PLATOON_STATE
    v
Follower (ID=2)
    |
    v
Follower (ID=3)
```

## Code Changes

### follow.cpp
- **LEAVE_PLATOON Case**: Updated to handle leader departure
  - Detects when current leader is leaving
  - Executes transition to leader using `execlp()`
  - Passes current state (ID, position, speed) to new leader process

### lead.cpp
- **Constructor**: Enhanced with initialization message showing new leader details
- **Main Function**: Updated to accept position and speed from command line
- **Event Handler**: Improved leader departure notification with clear status messages
- **Event Sender**: Reordered operations to:
  1. Send LEAVE_PLATOON to next vehicle
  2. Wait for message delivery (500ms)
  3. Remove self from platoon state
  4. Send final platoon state update
  5. Shutdown server

## Safety Guarantees

1. **Message Delivery**: 500ms delay ensures LEAVE_PLATOON message is delivered before shutdown
2. **State Preservation**: New leader inherits exact position and speed
3. **Thread Safety**: All state updates protected by mutex locks
4. **Clean Shutdown**: All threads and sockets properly closed before transition
5. **Fallback**: If `execlp()` fails, error is logged and process exits safely

## Testing Scenario

1. Start leader: `./lead 0 100 20`
2. Start followers: 
   - `./follow 1 80`
   - `./follow 2 60`
   - `./follow 3 40`
3. In leader input terminal, select option 4 (Leave Platoon)
4. Observe:
   - Leader notifies follower ID=1
   - Leader shuts down
   - Follower ID=1 transitions to leader role
   - Followers ID=2 and ID=3 continue following new leader
   - Display updates automatically

## Notes

- The transition uses `execlp()` to replace the process, ensuring clean state
- All followers automatically adapt to the new leader through PLATOON_STATE messages
- The new leader's display shows the full leader interface with platoon management
- Event input terminals remain functional through the transition
