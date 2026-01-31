#!/bin/bash

# Compile first
echo "Compiling..."
make clean > /dev/null
make > /dev/null

if [ $? -ne 0 ]; then
    echo "Compilation failed!"
    exit 1
fi

echo "Starting Initial Leader (ID 1)..."
./lead > lead_log.txt 2>&1 &
LEAD_PID=$!
sleep 1

echo "Starting Follower 2..."
./follow 2 > follow2_log.txt 2>&1 &
FOLLOW2_PID=$!
sleep 1

echo "Starting Follower 3..."
./follow 3 > follow3_log.txt 2>&1 &
FOLLOW3_PID=$!
sleep 1

echo "Allowing platoon to stabilize (Leader + 2 + 3)..."
sleep 5
if grep "Added new vehicle 3" lead_log.txt > /dev/null; then
    echo "  - Initial platoon stabilized (Leader 1, Follower 2, Follower 3)."
else
    echo "  - Warning: Follower 3 might not have joined yet."
fi

# Send LEAVE command to Leader
echo "Sending LEAVE command (7) to Leader (ID 1)..."
LEADER_FIFO="/tmp/leader_event_fifo"
if [ ! -p "$LEADER_FIFO" ]; then
    echo "Error: Leader FIFO not found!"
    kill $LEAD_PID $FOLLOW2_PID $FOLLOW3_PID
    exit 1
fi
python3 -c "import sys, struct; sys.stdout.buffer.write(struct.pack('i', 7))" > "$LEADER_FIFO"

echo "Waiting for transition (Leader 1 leaves, Follower 2 becomes Leader)..."
sleep 12

# Force kill old leader just in case (same fix as before)
echo "Ensuring Old Leader (PID $LEAD_PID) is dead..."
kill -9 $LEAD_PID 2>/dev/null

echo "Checking if Follower 2 became Leader..."
if grep "Successfully bound to port 5000" follow2_log.txt > /dev/null; then
    echo "  - Follower 2 successfully bound to Leader Port 5000."
else
    echo "  - Warning: Follower 2 might not have bound to port 5000 yet."
fi

echo "Starting New Follower (ID 4)..."
./follow 4 > follow4_log.txt 2>&1 &
FOLLOW4_PID=$!
sleep 5

echo "Verifying system state..."

# Check if Leader 2 added Follower 4
if grep "Added new vehicle 4" follow2_log.txt > /dev/null; then
    echo "SUCCESS: New Leader (ID 2) accepted New Follower (ID 4)."
else
    echo "FAILURE: New Leader (ID 2) did not report adding Follower 4."
    echo "--- Tail of Leader 2 Log ---"
    tail -n 10 follow2_log.txt
fi

# Check if Follower 3 recognized the change
# Follower 3 should now see ID 2 as Leader (assuming it prints "Leader (ID 2)")
# Note: we changed follow.cpp to print "Leader (ID 2)" if front is leader.
# Follower 3's front is Leader 2.
if grep "Front vehicle: Leader (ID 2)" follow3_log.txt > /dev/null; then
    echo "SUCCESS: Follower 3 recognizes Follower 2 as the new Leader."
else
    echo "FAILURE: Follower 3 does not seem to recognize Leader ID 2."
    echo "--- Tail of Follower 3 Log ---"
    tail -n 10 follow3_log.txt
fi

# Check if Follower 4 joined correctly
if grep "Front vehicle: ID 3" follow4_log.txt > /dev/null; then
    echo "SUCCESS: Follower 4 recognizes Follower 3 as its front vehicle."
else
    echo "FAILURE: Follower 4 does not seem to be behind Follower 3."
    echo "--- Tail of Follower 4 Log ---"
    tail -n 10 follow4_log.txt
fi

echo "Cleaning up..."
kill $LEAD_PID $FOLLOW2_PID $FOLLOW3_PID $FOLLOW4_PID 2>/dev/null
rm -f /tmp/leader_event_fifo /tmp/follower_*_event_fifo
echo "Done."
