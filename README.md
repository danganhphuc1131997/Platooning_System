# Truck Platooning System v2

A truck platooning system using hybrid TCP/UDP protocol.

## Communication Architecture

```
┌─────────────┐    TCP (8080)     ┌─────────────┐
│   LEADER    │◄─────JOIN────────►│  FOLLOWER   │
│  (lead.cpp) │                   │ (follow.cpp)│
│             │    UDP (9000)     │             │
│             │────BROADCAST─────►│             │
│             │    UDP (9001)     │             │
│             │◄────STATE─────────│             │
└─────────────┘                   └─────────────┘
```

- **TCP (port 8080)**: Join platoon (reliable)
- **UDP (port 9000)**: Leader broadcasts state (10Hz) - realtime
- **UDP (port 9001)**: Follower sends state to leader (2Hz)

## Message Structures

| Message Type | Protocol | Description |
|-------------|----------|-------------|
| `JoinRequest` | TCP | Follower requests to join platoon |
| `JoinResponse` | TCP | Leader confirms, assigns position |
| `LeaderState` | UDP | Speed, position, brake, emergency |
| `FollowerState` | UDP | Speed, position, gap to leader |

## Build

```bash
g++ -std=c++17 lead.cpp -o lead -pthread
g++ -std=c++17 follow.cpp -o follow -pthread
```

## Run

### Terminal 1 - Leader
```bash
./lead
```

### Terminal 2 - Follower
```bash
./follow                      # Default: Follower_01, leader at 127.0.0.1
./follow Truck_02             # Set vehicle name
./follow Truck_02 192.168.1.5 # Set vehicle name and leader IP
```

## Sample Logs

### Leader
```
══════════ LEADER TRUCK STARTING ══════════
[10:30:15.123] [MAIN] Initializing platooning leader...
[10:30:15.124] [TCP] ✅ Listening on port 8080 for JOIN requests
[10:30:15.125] [UDP-TX] ✅ Broadcasting on port 9000
[10:30:15.126] [UDP-RX] ✅ Listening on port 9001 for follower states
────────────────────────────────────────────────
[10:30:20.456] [TCP] ✅ JOIN ACCEPTED
[10:30:20.456] [TCP]    ├─ Vehicle ID : Truck_02
[10:30:20.456] [TCP]    ├─ From IP    : 127.0.0.1
[10:30:20.456] [TCP]    ├─ Position # : 1
[10:30:20.456] [TCP]    └─ Total followers: 1
[10:30:21.000] [UDP-TX] 📡 BROADCAST #10 | Speed: 80.0 km/h | Pos: 22.2 m | Brake: OFF
[10:30:21.500] [UDP-RX] 📥 FROM [Truck_02] #1 | Speed: 78.5 km/h | Gap: 20.1 m | Status: OK
```

### Follower
```
══════════ FOLLOWER TRUCK STARTING ══════════
[10:30:20.100] [MAIN] Vehicle ID: Truck_02
[10:30:20.100] [MAIN] Leader IP: 127.0.0.1
────────────────────────────────────────────────
[10:30:20.450] [TCP] ✅ Connected to leader
[10:30:20.451] [TCP] 📤 Sent JOIN_REQUEST
[10:30:20.455] [TCP] ✅ JOIN ACCEPTED
[10:30:20.455] [TCP]    ├─ Assigned Index: #1
[10:30:20.455] [TCP]    └─ Message: Welcome to platoon, position #1
══════════ JOINED PLATOON ══════════
[10:30:21.000] [UDP-RX] 📥 LEADER #10 | L.Speed: 80.0 km/h | Gap: 22.2 m | My Speed: 81.1 km/h
[10:30:23.500] [UDP-TX] 📤 SENT #5 | Speed: 79.5 km/h | Pos: 2.2 m | Gap: 20.0 m
```

## Files

| File | Description |
|------|-------------|
| `message.h` | Message structure definitions |
| `lead.h` | Leader class header |
| `lead.cpp` | Leader truck implementation |
| `follow.h` | Follower class header |
| `follow.cpp` | Follower truck implementation |
| `ARCHITECTURE.md` | Detailed architecture documentation |
