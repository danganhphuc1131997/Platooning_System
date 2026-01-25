# Running the Platooning System in WSL

This guide shows how to run the platooning system in WSL (Windows Subsystem for Linux) using separate terminal windows or tmux panes for event input.

## Quick Start

### 1. Build All Programs
```bash
make
```

This creates 4 programs:
- `lead` - Leader vehicle
- `follow` - Follower vehicle  
- `leader_input` - Event input for leader
- `follower_input` - Event input for follower

### 2. Option A: Using Multiple WSL Terminal Windows

**Terminal 1** - Start Leader:
```bash
./lead
```

**Terminal 2** - Start Follower (ID 2):
```bash
./follow 2
```

**Terminal 3** - Leader Event Input:
```bash
./leader_input
```

**Terminal 4** - Follower Event Input:
```bash
./follower_input 2
```

### 3. Option B: Using tmux (Recommended for WSL)

Start tmux:
```bash
tmux
```

Split into 4 panes:
1. Press `Ctrl+b` then `"` (split horizontally)
2. Press `Ctrl+b` then `%` (split vertically)  
3. Navigate: `Ctrl+b` then arrow keys
4. Repeat splits until you have 4 panes

In each pane:
- **Pane 1**: `./lead`
- **Pane 2**: `./follow 2`
- **Pane 3**: `./leader_input`
- **Pane 4**: `./follower_input 2`

## Event Input Controls

### Leader Events (leader_input):
- **1**: Obstacle detected (leader stops)
- **2**: Traffic light RED
- **3**: Traffic light GREEN  
- **4**: Cut-in vehicle alert
- **0**: Exit

### Follower Events (follower_input <id>):
- **1**: Traffic light RED (follower stops)
- **2**: Traffic light GREEN (follower resumes)
- **0**: Exit

## Running Multiple Followers

Start each follower with a unique ID:
```bash
./follow 2
./follow 3
./follow 4
```

Then for each follower, open its input program:
```bash
./follower_input 2
./follower_input 3
./follower_input 4
```

## Tmux Quick Reference

- `Ctrl+b` then `"` - Split horizontally
- `Ctrl+b` then `%` - Split vertically
- `Ctrl+b` then arrow key - Navigate between panes
- `Ctrl+b` then `x` - Close current pane
- `Ctrl+b` then `d` - Detach from session
- `tmux attach` - Reattach to session
- `Ctrl+b` then `z` - Zoom current pane (toggle fullscreen)

## Troubleshooting

### "FIFO not found" error
Make sure the main program (lead/follow) is running before starting the input program.

### Stale FIFO files
Clean up:
```bash
make clean
```

Or manually:
```bash
rm -f /tmp/*_event_fifo /tmp/leader_event_fifo
```

## Build Options

```bash
make          # Build all programs
make main     # Build lead and follow only
make input    # Build input programs only  
make clean    # Remove all binaries and FIFOs
make help     # Show help
```
