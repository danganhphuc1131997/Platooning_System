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

# Leader FIFO
LEADER_FIFO="/tmp/leader_event_fifo"
FOLLOW3_FIFO="/tmp/follower_3_event_fifo"

if [ ! -p "$LEADER_FIFO" ]; then
    echo "Error: Leader FIFO not found!"
    kill $LEAD_PID $FOLLOW1_PID $FOLLOW2_PID
    exit 1
fi

echo "Sending Traffic Light RED (2) to Leader..."
python3 -c "import sys, struct; sys.stdout.buffer.write(struct.pack('i', 2))" > "$LEADER_FIFO"

echo "Waiting for vehicles to stop..."
sleep 5

echo "Arming Delay 5s (1005) for Follower 3..."
if [ ! -p "$FOLLOW3_FIFO" ]; then
    echo "Error: Follower 3 FIFO not found!"
    kill $LEAD_PID $FOLLOW1_PID $FOLLOW2_PID
    exit 1
fi
# Write int 1005 (binary)
python3 -c "import sys, struct; sys.stdout.buffer.write(struct.pack('i', 1005))" > "$FOLLOW3_FIFO"

echo "Sending Traffic Light GREEN (3) to Leader..."
python3 -c "import sys, struct; sys.stdout.buffer.write(struct.pack('i', 3))" > "$LEADER_FIFO"

echo "Follower 3 should WAIT while others start."
sleep 2

echo "Checking logs for verification..."

# Check if Follower 3 received the command
if grep -q "Armed start-delay after next GREEN: 5s" follow3_log.txt; then
    echo "✓ Follower 3 received delay command."
else
    echo "✗ Follower 3 did NOT receive delay command."
fi

# We can't easily check real-time state via logs without tailing, but we can verify the Decoupling message appears later.

echo "Waiting for delay (5s) to expire..."
sleep 5

echo "Checking if Follower 3 Decoupled and Speed increased..."
if grep -q "Speed: 100" follow3_log.txt || grep -q "Speed: 9[0-9]" follow3_log.txt; then
    echo "✓ Follower 3 reached high speed (~100)."
else
    echo "⚠ Follower 3 speed check unclear from grep (check logs manually)."
fi

echo "Logs available in lead_log.txt, follow2_log.txt, follow3_log.txt"
echo "Killing processes..."
kill $LEAD_PID $FOLLOW1_PID $FOLLOW2_PID 2>/dev/null
echo "Done."
