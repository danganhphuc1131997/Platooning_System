# Truck Platooning System v2

A truck platooning system using hybrid TCP/UDP protocol.

## Communication Architecture

```
┌─────────────┐    TCP (8080)     ┌─────────────┐
│   LEADER    │◄───JOIN/LEAVE────►│  FOLLOWER   │
│  (lead.cpp) │                   │ (follow.cpp)│
│             │    UDP (9000)     │             │
│             │────BROADCAST─────►│             │
│             │    UDP (9001)     │             │
│             │◄────STATE─────────│             │
└─────────────┘                   └─────────────┘
```

- **TCP (port 8080)**: Join/Leave platoon (reliable)
- **UDP (port 9000)**: Leader broadcasts state (10Hz) - realtime
- **UDP (port 9001)**: Follower sends state to leader (2Hz)

## Message Structures

| Message Type | Protocol | Description |
|-------------|----------|-------------|
| `JoinRequest` | TCP | Follower requests to join platoon |
| `JoinResponse` | TCP | Leader confirms, assigns position |
| `LeaveRequest` | TCP | Follower requests to leave platoon |
| `LeaveResponse` | TCP | Leader confirms removal |
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

### Terminal 2+ - Follower
```bash
./follow [options]

Options:
  --name <id>       Vehicle ID (default: auto-generated Follower_XX)
  --leadip <ip>     Leader IP address (default: 127.0.0.1)
  --speed <km/h>    Initial speed (default: 60)
  --gap <m>         Initial distance to leader (default: 50)
  --help            Show help
```

#### Examples
```bash
# Default (auto ID, 60 km/h, 50m gap)
./follow

# Custom name only
./follow --name TRUCK_01

# Full configuration
./follow --name TRUCK_01 --leadip 192.168.1.100 --speed 80 --gap 30

# Any order works
./follow --gap 25 --speed 75 --name MyTruck
```

## Interactive Commands (Follower)

While follower is running, you can type commands:

| Command | Description |
|---------|-------------|
| `leave` or `l` | Leave platoon (normal) |
| `leave 1` | Leave platoon (emergency) |
| `leave 2` | Leave platoon (maintenance) |
| `status` or `s` | Show current status |
| `help` or `h` | Show available commands |
| `quit` or `q` | Exit program |

## Sample Session

### Leader Dashboard
```
╔══════════════════════════════════════════════════════════════╗
║            🚛  LEADER TRUCK DASHBOARD  🚛                    ║
╠══════════════════════════════════════════════════════════════╣
║  Speed:   80.0 km/h  │  Position:   1234.5 m                 ║
║  Brake:  OFF         │  Emergency:   NO                      ║
╠══════════════════════════════════════════════════════════════╣
║  FOLLOWERS (2)                                               ║
╠══════════════════════════════════════════════════════════════╣
║  #1 [TRUCK_01     ] Spd: 79.5 km/h  Gap: 20.0 m              ║
║  #2 [TRUCK_02     ] Spd: 78.8 km/h  Gap: 40.5 m              ║
╚══════════════════════════════════════════════════════════════╝
```

### Follower Dashboard
```
╔══════════════════════════════════════════════════════════════╗
║            🚚  FOLLOWER TRUCK DASHBOARD  🚚                  ║
║  ID: TRUCK_01         Index: #1                              ║
╠══════════════════════════════════════════════════════════════╣
║  My Speed:  79.5 km/h  │  Position:  1214.5 m                ║
║  Gap to Leader:  20.0 m [──────────█─────────]               ║
╠══════════════════════════════════════════════════════════════╣
║  LEADER STATE                                                ║
╠══════════════════════════════════════════════════════════════╣
║  Leader Speed:  80.0 km/h  │  Brake: OFF  │  Emerg:  NO      ║
╚══════════════════════════════════════════════════════════════╝

Command> leave
🚪 Leaving platoon...
✅ LEAVE ACCEPTED: Goodbye, removed from platoon
👋 Left platoon successfully. Exiting...
```

## Files

| File | Description |
|------|-------------|
| `message.h` | Message structure definitions (all protocols) |
| `lead.h` | Leader constants and declarations |
| `lead.cpp` | Leader truck implementation |
| `follow.h` | Follower constants and declarations |
| `follow.cpp` | Follower truck implementation |
| `ARCHITECTURE.md` | Detailed architecture documentation |
