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

### Quick Build with Makefile (Recommended)
```bash
make
```
This builds all programs: `lead`, `follow`, `leader_input`, and `follower_input`

### Manual Build
```bash
g++ -std=c++17 lead.cpp -o lead -pthread
g++ -std=c++17 follow.cpp -o follow -pthread
g++ -std=c++17 leader_input.cpp -o leader_input -pthread
g++ -std=c++17 follower_input.cpp -o follower_input -pthread
```

## Run in WSL / Linux

### ⭐ NEW: WSL-Compatible Multi-Terminal Setup

**The old approach (auto-spawning GUI terminals) doesn't work in WSL.** Use one of these methods:

#### Option 1: Multiple Terminal Windows (Manual)
Open 4 separate WSL terminal windows:

**Terminal 1 - Leader Vehicle:**
```bash
./lead
```

**Terminal 2 - Follower Vehicle:**
```bash
./follow 2
```

**Terminal 3 - Leader Event Input:**
```bash
./leader_input
```

**Terminal 4 - Follower Event Input:**
```bash
./follower_input 2
```

#### Option 2: Using tmux (Recommended)
```bash
./start_tmux.sh
```
This automatically sets up 4 panes in tmux. Then run:
- Pane 1 (top-left): `./lead`
- Pane 2 (top-right): `./follow 2`
- Pane 3 (bottom-left): `./leader_input`
- Pane 4 (bottom-right): `./follower_input 2`

See [WSL_GUIDE.md](WSL_GUIDE.md) for detailed instructions!

---

### Running Additional Followers
You can start more followers:
```bash
./follow 3
./follow 4
```

And their corresponding input programs:
```bash
./follower_input 3
./follower_input 4
```

## Event Controls

### Leader Events (via leader_input):
- `1` = Obstacle detected (leader stops)
- `2` = Traffic light RED
- `3` = Traffic light GREEN
- `4` = Cut-in vehicle alert
- `0` = Exit

### Follower Events (via follower_input):
- `1` = Traffic light RED (follower stops)
- `2` = Traffic light GREEN (follower resumes)
- `0` = Exit

## Node failure demo
While followers are running, kill one follower terminal with `Ctrl+C` (crash). Leader will remove it after the timeout.
