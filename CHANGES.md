# NEW WSL-Compatible Programs Summary

## Problem Solved
The original code attempted to automatically spawn GUI terminal windows (gnome-terminal, xterm, etc.) for event input, which doesn't work in WSL because:
1. WSL typically runs without a GUI/X11 server
2. Even with X11, the automatic terminal spawning is unreliable
3. The fork/exec approach with GUI terminals fails silently in WSL

## Solution
Created 2 standalone input programs that can be run manually in separate terminals or tmux panes:

### 1. leader_input.cpp
- **Purpose**: Send events to the leader vehicle
- **Usage**: `./leader_input`
- **Features**:
  - Waits for leader to create FIFO (auto-connects)
  - Interactive menu with 5 options (obstacle, red light, green light, cut-in, exit)
  - Clear feedback for each action
  - Handles FIFO errors gracefully

### 2. follower_input.cpp
- **Purpose**: Send events to specific follower vehicles
- **Usage**: `./follower_input <follower_id>`
- **Features**:
  - Takes follower ID as command-line argument
  - Waits for follower to create FIFO (auto-connects)
  - Interactive menu with 3 options (red light, green light, exit)
  - Clear feedback for each action
  - Handles FIFO errors gracefully

## Additional Files Created

### Makefile
- Builds all 4 programs with one command: `make`
- Separate targets for main programs and input programs
- Clean targets to remove binaries and stale FIFOs
- Help target with usage instructions

### WSL_GUIDE.md
- Comprehensive guide for WSL users
- Step-by-step instructions for both manual and tmux setups
- Troubleshooting section
- Tmux quick reference

### start_tmux.sh
- Automated tmux session setup
- Creates 4 panes with proper layout
- Pre-configures each pane with appropriate commands
- Includes visual guide and instructions

### setup_info.sh
- Displays setup instructions in a nice formatted way
- Shows both manual and tmux options
- Quick reference after building

## How It Works

### Communication Flow
```
┌──────────┐         FIFO          ┌────────────────┐
│  leader  │◄──────────────────────┤ leader_input   │
└──────────┘  /tmp/leader_event_   └────────────────┘
              fifo
              
┌──────────┐         FIFO          ┌────────────────┐
│follow (2)│◄──────────────────────┤follower_input 2│
└──────────┘  /tmp/follower_2_     └────────────────┘
              event_fifo
```

1. Main program (lead/follow) creates FIFO
2. Input program waits for FIFO to exist
3. Input program opens FIFO for writing
4. User sends events through interactive menu
5. Main program reads events from FIFO

## Benefits Over Old Approach

✓ **WSL Compatible**: Works perfectly in WSL without GUI dependencies
✓ **Flexible**: Can use multiple terminal windows OR tmux
✓ **Reliable**: No silent failures or complex fork/exec logic
✓ **User-Friendly**: Clear menus and feedback
✓ **Easy to Use**: Simple command-line arguments
✓ **Scalable**: Easy to run multiple followers with separate inputs
✓ **No Dependencies**: No X11 or GUI terminal emulators required

## Usage Examples

### Basic Usage (2 followers)
```bash
# Terminal 1
./lead

# Terminal 2
./follow 2

# Terminal 3
./leader_input

# Terminal 4
./follower_input 2
```

### Advanced Usage (4 followers with tmux)
```bash
./start_tmux.sh
# Creates organized panes for:
# - Leader + 3 followers
# - Leader input + 3 follower inputs
```

### Multiple Followers
```bash
# Start followers
./follow 2
./follow 3
./follow 4

# In separate terminals/panes
./follower_input 2
./follower_input 3
./follower_input 4
```

## Files Modified
- ✅ README.md - Updated with new WSL instructions
- ✅ Makefile - New build system
- ✅ WSL_GUIDE.md - New comprehensive guide

## Files Created
- ✅ leader_input.cpp - Standalone leader event input
- ✅ follower_input.cpp - Standalone follower event input  
- ✅ start_tmux.sh - Automated tmux setup
- ✅ setup_info.sh - Setup instructions display
