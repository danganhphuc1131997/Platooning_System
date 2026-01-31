# Platooning System Project

A simulation of a vehicle platoon system handling leader election, vehicle following logic (PID), and collision avoidance (AEB using OpenCL).

## Prerequisites

This project relies on standard C++ libraries, POSIX Threads (pthread), and OpenCL for the AEB system.

### Install Dependencies (Ubuntu/WSL)

```bash
sudo apt update
sudo apt install build-essential ocl-icd-opencl-dev opencl-headers clinfo
```

To verify OpenCL installation:
```bash
clinfo
```

## Build

The project uses a `Makefile` for compilation.

```bash
# Clean and Build all components
make clean
make
```

This will generate executables in the `output/` directory:
- `output/lead`: The Leader vehicle process.
- `output/follow`: The Follower vehicle process.
- `output/leader_input`: Input simulator for Leader events (e.g., Leave Platoon).
- `output/follower_input`: (Legacy) Input simulator for Follower events.

## Usage

### 1. Manual Run (Multi-Terminal)

You generally need multiple terminals to run the system interactively.

**Terminal 1: Leader**
```bash
./output/lead
```

**Terminal 2+: Followers**
Start followers with a specific ID.
```bash
# Follower with ID 2
./output/follow 2

# Follower with ID 3
./output/follow 3
```

**Terminal 3: Controller**
Send commands to the leader (e.g., speed up, slow down, leave platoon).
```bash
./output/leader_input
```

### 2. Automated Test

A specialized script is available to test the "Leader Leave & Promote" scenario. This script sets up a topology (L1 -> F2 -> F3), triggers the Leader (L1) to leave, and verifies that F2 takes over as the new Leader.

```bash
chmod +x test_3_vehicles_leader_leave_join.sh
./test_3_vehicles_leader_leave_join.sh
```

## Key Features

- **Leader Election**: Dynamic promotion of the first follower to leader if the original leader departs.
- **OpenCL AEB**: Parallel processing (GPU/CPU) for calculating Autonomous Emergency Braking risk using ray-casting simulation.
- **UDP Networking**: Vehicles communicate via UDP sockets (Port 5000 is reserved for the Leader).
- **PID Control**: Followers use PID controllers to maintain safe distance and speed matching.
