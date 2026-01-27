#!/bin/bash

# Simple test to verify UDP communication works
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "Testing UDP communication on port 5000..."
echo ""

# Clean up
pkill -9 -f "./lead\|./follow" 2>/dev/null
sleep 1

# Start tcpdump in background (requires sudo)
if command -v tcpdump &> /dev/null; then
    echo "Starting tcpdump to capture packets on port 5000..."
    sudo tcpdump -i lo -n 'udp port 5000' -w /tmp/udp_test.pcap &
    TCPDUMP_PID=$!
    sleep 1
else
    echo "tcpdump not available, skipping packet capture"
    TCPDUMP_PID=""
fi

# Test 1: Start leader, then follower 2
echo "Test 1: Leader + Follower 2 (should work)"
./lead > /tmp/test_lead.log 2>&1 &
LEAD_PID=$!
sleep 2

./follow 2 1000 > /tmp/test_f2.log 2>&1 &
F2_PID=$!
sleep 5

echo "Checking if Follower 2 joined..."
if grep -q "Added vehicle 2" /tmp/test_lead.log; then
    echo "✓ Follower 2 joined successfully"
else
    echo "✗ Follower 2 failed to join"
fi

# Kill them
kill -9 $LEAD_PID $F2_PID 2>/dev/null
sleep 2

# Test 2: Follower 2 transitions to leader, then follower 4 joins
echo ""
echo "Test 2: Leader transition scenario"
./lead > /tmp/test_lead2.log 2>&1 &
LEAD_PID=$!
sleep 2

./follow 2 1000 > /tmp/test_f2_2.log 2>&1 &
F2_PID=$!
sleep 3

# Send leave command
echo "4" | ./leader_input > /dev/null 2>&1
sleep 5

# Now start follower 4
./follow 4 100 > /tmp/test_f4.log 2>&1 &
F4_PID=$!
sleep 5

echo "Checking if Leader received messages from Follower 4 (port 5004)..."
if grep -q "5004" /tmp/test_f2_2.log; then
    echo "✓ Leader received messages from Follower 4"
    grep "5004" /tmp/test_f2_2.log | head -3
else
    echo "✗ Leader did NOT receive messages from Follower 4"
    echo "Messages leader received:"
    tail -10 /tmp/test_f2_2.log | grep "LEADER RECV"
fi

# Clean up
kill -9 $LEAD_PID $F2_PID $F4_PID 2>/dev/null
if [ -n "$TCPDUMP_PID" ]; then
    sudo kill -9 $TCPDUMP_PID 2>/dev/null
    echo ""
    echo "Packet capture saved to /tmp/udp_test.pcap"
    echo "Analyze with: tcpdump -r /tmp/udp_test.pcap"
fi

pkill -9 -f "./lead\|./follow" 2>/dev/null
