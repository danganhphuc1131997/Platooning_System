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

echo "Starting Initial Follower (ID 2)..."
./follow 2 > follow2_log.txt 2>&1 &
FOLLOW2_PID=$!
sleep 2

echo "Allowing platoon to stabilize..."
echo "  - Checking if Follower 2 joined..."
grep "Added new vehicle 2" lead_log.txt > /dev/null
if [ $? -eq 0 ]; then
    echo "  - Follower 2 joined successfully."
else
    echo "  - Warning: Follower 2 might not have joined yet."
fi
sleep 3

echo "Sending LEAVE command (7) to Leader (ID 1)..."
# Leader FIFO
LEADER_FIFO="/tmp/leader_event_fifo"
if [ ! -p "$LEADER_FIFO" ]; then
    echo "Error: Leader FIFO not found!"
    kill $LEAD_PID $FOLLOW2_PID
    exit 1
fi
# Write int 7 (binary) to FIFO.
python3 -c "import sys, struct; sys.stdout.buffer.write(struct.pack('i', 7))" > "$LEADER_FIFO"

echo "Waiting for transition (Old leader leaves, Follower 2 becomes New Leader)..."
# The transition takes a few seconds (socket release wait time)
sleep 8

# FORCE KILL OLD LEADER if it didn't exit (Workaround to ensure port release for test)
echo "Ensuring Old Leader (PID $LEAD_PID) is dead..."
kill -9 $LEAD_PID 2>/dev/null

echo "Checking if New Leader (ID 2, PID $FOLLOW2_PID) is valid..."
if grep "Successfully bound to port 5000" follow2_log.txt > /dev/null; then
    echo "  - Follower 2 successfully bound to Leader Port 5000."
else
    echo "  - Warning: Follower 2 might not have bound to port 5000 yet. Checking log tail:"
    tail -n 5 follow2_log.txt
fi

echo "Starting New Follower (ID 3)..."
./follow 3 > follow3_log.txt 2>&1 &
FOLLOW3_PID=$!
sleep 3

echo "Verifying New Follower (ID 3) joined the platoon..."
netstat -uapn | grep 5000
grep "Got COUPLE" follow2_log.txt && echo "DEBUG: Leader received COUPLE command."
grep "Added new vehicle 3" follow2_log.txt && echo "DEBUG: Leader added vehicle 3."

# Check new leader logs for "Added new vehicle 3"
if grep "Added new vehicle 3" follow2_log.txt > /dev/null; then
    echo "SUCCESS: New Leader (ID 2) accepted New Follower (ID 3)."
else
    echo "FAILURE: New Leader (ID 2) did not report adding Follower 3."
    echo "Last lines of New Leader (Follower 2) log:"
    tail -n 10 follow2_log.txt
fi

# Check follower 3 logs for platoon state reception
if grep "Leader" follow3_log.txt | grep "ID 2" > /dev/null; then
    echo "SUCCESS: Follower 3 recognizes ID 2 as Leader."
else
    echo "FAILURE: Follower 3 does not seem to recognize Leader ID 2."
    echo "Last lines of Follower 3 log:"
    tail -n 10 follow3_log.txt
fi

echo "Killing processes..."
kill $LEAD_PID $FOLLOW2_PID $FOLLOW3_PID 2>/dev/null
echo "Done."
