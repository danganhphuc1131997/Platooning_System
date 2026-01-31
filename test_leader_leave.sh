#!/bin/bash

# Compile first
echo "Compiling..."
make clean > /dev/null
make > /dev/null

if [ $? -ne 0 ]; then
    echo "Compilation failed!"
    exit 1
fi

echo "Starting Leader..."
# Start leader in background
./lead > lead_log.txt 2>&1 &
LEAD_PID=$!
sleep 1

echo "Starting Follower 2..."
./follow 2 > follow2_log.txt 2>&1 &
FOLLOW1_PID=$!
sleep 1

echo "Starting Follower 3..."
./follow 3 > follow3_log.txt 2>&1 &
FOLLOW2_PID=$!
sleep 2

echo "Allowing platoon to stabilize..."
sleep 5

echo "Sending LEAVE command (7) to Leader..."
# Leader FIFO
LEADER_FIFO="/tmp/leader_event_fifo"
if [ ! -p "$LEADER_FIFO" ]; then
    echo "Error: Leader FIFO not found!"
    kill $LEAD_PID $FOLLOW1_PID $FOLLOW2_PID
    exit 1
fi
# Write int 7 (binary) to FIFO. Using python to write binary int
python3 -c "import sys, struct; sys.stdout.buffer.write(struct.pack('i', 7))" > "$LEADER_FIFO"

echo "Waiting for transition..."
sleep 10

echo "Checking if Follower 2 (PID $FOLLOW1_PID) is still running (as Leader)..."
if ps -p $FOLLOW1_PID > /dev/null; then
    echo "Follower 2 is running."
else
    echo "Follower 2 crashed or exited!"
fi

echo "Sending OBSTACLE command (1) to New Leader (Follower 2)..."
# Follower 2 FIFO
FOLLOW1_FIFO="/tmp/follower_2_event_fifo"
if [ ! -p "$FOLLOW1_FIFO" ]; then
    echo "Error: Follower 2 FIFO not found!"
else
    # New leader expects code 1 for obstacle
    python3 -c "import sys, struct; sys.stdout.buffer.write(struct.pack('i', 1))" > "$FOLLOW1_FIFO"
fi

sleep 5

echo "Logs available in lead_log.txt, follow2_log.txt, follow3_log.txt"
echo "Killing processes..."
kill $LEAD_PID $FOLLOW1_PID $FOLLOW2_PID 2>/dev/null
echo "Done."
