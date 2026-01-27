#!/bin/bash

# Test script for leader transition and new follower join
# This tests the scenario: Leader 1 -> Follower 2 -> Follower 3, then Leader leaves, Follower 4 joins

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Log directory
LOG_DIR="$SCRIPT_DIR/test_logs"
mkdir -p "$LOG_DIR"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Leader Transition Test${NC}"
echo -e "${GREEN}========================================${NC}"
echo "Log directory: $LOG_DIR"
echo "Timestamp: $TIMESTAMP"
echo ""

# Clean up any existing processes
echo -e "${YELLOW}Cleaning up existing processes...${NC}"
pkill -9 -f "./lead" 2>/dev/null
pkill -9 -f "./follow" 2>/dev/null
pkill -9 -f "leader_input" 2>/dev/null
pkill -9 -f "follower_input" 2>/dev/null
sleep 1

# Clean up FIFOs
rm -f /tmp/*_event_fifo /tmp/leader_event_fifo 2>/dev/null

echo -e "${GREEN}Step 1: Starting Leader (Vehicle 1)${NC}"
./lead > "$LOG_DIR/leader1_${TIMESTAMP}.log" 2>&1 &
LEADER_PID=$!
echo "Leader PID: $LEADER_PID"
sleep 2

echo -e "${GREEN}Step 2: Starting Follower 2${NC}"
./follow 2 1000 > "$LOG_DIR/follower2_${TIMESTAMP}.log" 2>&1 &
FOLLOWER2_PID=$!
echo "Follower 2 PID: $FOLLOWER2_PID"
sleep 2

echo -e "${GREEN}Step 3: Starting Follower 3${NC}"
./follow 3 500 > "$LOG_DIR/follower3_${TIMESTAMP}.log" 2>&1 &
FOLLOWER3_PID=$!
echo "Follower 3 PID: $FOLLOWER3_PID"
sleep 3

echo -e "${YELLOW}Waiting for platoon to stabilize (5 seconds)...${NC}"
sleep 5

echo -e "${GREEN}Step 4: Sending Leave Platoon command to Leader 1${NC}"
echo "4" | ./leader_input > /dev/null 2>&1 &
sleep 1

echo -e "${YELLOW}Waiting for leader transition (5 seconds)...${NC}"
sleep 5

echo -e "${GREEN}Step 5: Starting Follower 4 (new follower joining after transition)${NC}"
./follow 4 100 > "$LOG_DIR/follower4_${TIMESTAMP}.log" 2>&1 &
FOLLOWER4_PID=$!
echo "Follower 4 PID: $FOLLOWER4_PID"

echo -e "${YELLOW}Letting system run for 10 seconds...${NC}"
sleep 10

echo -e "${YELLOW}Collecting logs...${NC}"

# Kill all processes
echo -e "${RED}Stopping all processes...${NC}"
kill -INT $LEADER_PID 2>/dev/null
kill -INT $FOLLOWER2_PID 2>/dev/null
kill -INT $FOLLOWER3_PID 2>/dev/null
kill -INT $FOLLOWER4_PID 2>/dev/null
sleep 2

# Force kill if still running
pkill -9 -f "./lead" 2>/dev/null
pkill -9 -f "./follow" 2>/dev/null

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Test Complete!${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo "Logs saved to:"
echo "  - Leader 1: $LOG_DIR/leader1_${TIMESTAMP}.log"
echo "  - Follower 2 (new leader): $LOG_DIR/follower2_${TIMESTAMP}.log"
echo "  - Follower 3: $LOG_DIR/follower3_${TIMESTAMP}.log"
echo "  - Follower 4 (test subject): $LOG_DIR/follower4_${TIMESTAMP}.log"
echo ""

# Analyze logs
echo -e "${YELLOW}========================================${NC}"
echo -e "${YELLOW}Log Analysis${NC}"
echo -e "${YELLOW}========================================${NC}"
echo ""

echo -e "${GREEN}Follower 4 - Couple Command Attempts:${NC}"
grep "COUPLE_COMMAND\|Sent couple command" "$LOG_DIR/follower4_${TIMESTAMP}.log" | head -5

echo ""
echo -e "${GREEN}Follower 2 (New Leader) - Received Messages:${NC}"
grep "LEADER RECV" "$LOG_DIR/follower2_${TIMESTAMP}.log" | tail -20

echo ""
echo -e "${GREEN}Follower 2 (New Leader) - Socket Info:${NC}"
grep "socket FD\|Socket receive buffer" "$LOG_DIR/follower2_${TIMESTAMP}.log"

echo ""
echo -e "${GREEN}Follower 4 - Final State:${NC}"
tail -15 "$LOG_DIR/follower4_${TIMESTAMP}.log" | grep -A 5 "FOLLOWER 4"

echo ""
echo -e "${GREEN}Checking if Follower 4 joined platoon:${NC}"
if grep -q "SUCCESS.*Joined platoon" "$LOG_DIR/follower4_${TIMESTAMP}.log"; then
    echo -e "${GREEN}✓ Follower 4 successfully joined!${NC}"
else
    echo -e "${RED}✗ Follower 4 failed to join platoon${NC}"
fi

echo ""
echo -e "${GREEN}Checking if Leader received COUPLE_COMMAND from port 5004:${NC}"
if grep -q "5004" "$LOG_DIR/follower2_${TIMESTAMP}.log"; then
    echo -e "${GREEN}✓ Leader received messages from Follower 4${NC}"
    grep "5004" "$LOG_DIR/follower2_${TIMESTAMP}.log" | head -3
else
    echo -e "${RED}✗ Leader did NOT receive any messages from Follower 4 (port 5004)${NC}"
fi

echo ""
echo -e "${YELLOW}To view full logs, check files in: $LOG_DIR${NC}"
