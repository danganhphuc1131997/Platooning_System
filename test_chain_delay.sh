#!/bin/bash

# Compile
echo "Compiling..."
make clean > /dev/null
make > /dev/null

if [ $? -ne 0 ]; then
    echo "Compilation failed!"
    exit 1
fi

echo "Starting Leader..."
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

echo "Starting Follower 4..."
./follow 4 > follow4_log.txt 2>&1 &
FOLLOW4_PID=$!
sleep 2

echo "Allowing platoon to stabilize..."
sleep 5

LEADER_FIFO="/tmp/leader_event_fifo"
FOLLOW2_FIFO="/tmp/follower_2_event_fifo"

echo "Sending Traffic Light RED (2) to Leader..."
# Code 2 is RED in lead.cpp
python3 -c "import sys, struct; sys.stdout.buffer.write(struct.pack('i', 2))" > "$LEADER_FIFO"

echo "Waiting 5s for vehicles to stop..."
sleep 5

echo "Arming Delay 5s (event 4 + 5 = code 1005) for Follower 2..."
# follower_input: choice 4 -> cin sec -> writes 1000+sec.
if [ -p "$FOLLOW2_FIFO" ]; then
    python3 -c "import sys, struct; sys.stdout.buffer.write(struct.pack('i', 1005))" > "$FOLLOW2_FIFO"
else
    echo "Follower 2 FIFO missing"
fi

echo "Sending Traffic Light GREEN (3) to Leader..."
# Code 3 is GREEN in lead.cpp
python3 -c "import sys, struct; sys.stdout.buffer.write(struct.pack('i', 3))" > "$LEADER_FIFO"

echo "Waiting for delay to process..."
sleep 2

echo "Checking logs..."
grep "Sent delay notification" follow2_log.txt
grep "Received DELAY_NOTIFICATION" follow3_log.txt
grep "Sent delay notification" follow3_log.txt
grep "Received DELAY_NOTIFICATION" follow4_log.txt

sleep 5
echo "Cleaning up..."
kill $LEAD_PID $FOLLOW2_PID $FOLLOW3_PID $FOLLOW4_PID
wait $LEAD_PID $FOLLOW2_PID $FOLLOW3_PID $FOLLOW4_PID 2>/dev/null
