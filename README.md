# Platooning System (Updated)

This updated version aligns with the professor’s requirements we discussed:

## What was fixed/added
- ✅ **pthreads + mutex** for parallel execution (no OpenMP for `send/recv`)
- ✅ **UDP communication** (connectionless, using `sendto/recvfrom`)
- ✅ **Safe messages** (fixed-size `WireMessage`, no `std::vector` in `send()`)
- ✅ **Node failure detection** via **heartbeat + timeout**
- ✅ **Your use case**: **cut-in / temporary communication loss** (`c`) → leader enters **degraded mode** (slows down slightly) → then recovers
- ✅ **Logical matrix clock pattern** (fixed `MAX_NODES` matrix, merged on receive)

## Build (WSL / Linux)
From this folder (`CPP_SRC`):

```bash
g++ -std=c++17 lead.cpp -o lead -pthread
g++ -std=c++17 follow.cpp -o follow -pthread
```

## Run (two terminals)

### Terminal 1 (Leader)
```bash
./lead
```

### Terminal 2 (Follower)
```bash
./follow
```

You can start more followers in more terminals:
```bash
./follow 3
./follow 4
```

## Controls (in follower terminal)
- `o` = obstacle detected (leader stops)
- `c` = **cut-in simulation** (pause heartbeat for 3s → leader slows down → then recovers)
- `q` = leave platoon (graceful)

## Node failure demo
While followers are running, kill one follower terminal with `Ctrl+C` (crash). Leader will remove it after the timeout.
