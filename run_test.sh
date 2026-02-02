#!/bin/bash

# Configuration
# ==============================================================================
# Number of followers to spawn
NUM_FOLLOWERS=15

# Duration to run the test (in seconds)
TEST_DURATION=30

# Enable input terminals? (true/false)
# If true, it tries to launch terminals for user input. If false, runs in background.
ENABLE_INPUT=false 
# ==============================================================================

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}==================================================${NC}"
echo -e "${GREEN}      PLATOONING SYSTEM - AUTOMATED TEST RUN      ${NC}"
echo -e "${GREEN}==================================================${NC}"

# 1. Cleanup old processes
echo -e "${YELLOW}[1/4] Cleaning up old processes...${NC}"
pkill -f "./lead"
pkill -f "./follow"
pkill -f "input_mode"
rm -f /tmp/leader_event_fifo /tmp/follower_*_event_fifo
sleep 1

# 2. Build project with WCET measurement enabled
echo -e "${YELLOW}[2/4] Building project with WCET measurement...${NC}"
make clean > /dev/null 2>&1
make -j4 CXXFLAGS="-DENABLE_WCET" > /dev/null
if [ $? -ne 0 ]; then
    echo -e "${RED}Build failed! Exiting.${NC}"
    exit 1
fi
echo -e "${GREEN}Build successful (WCET enabled).${NC}"

# 3. Start Leader (Background)
echo -e "${YELLOW}[3/4] Starting Leader vehicle...${NC}"
# Use stdbuf to unbuffer output so we can see it realtime if needed
# We redirect output to a log file, but also keep running in background
./lead > leader.log 2>&1 &
LEADER_PID=$!
echo -e "Leader started (PID: $LEADER_PID). Logs: leader.log"
sleep 2 # Wait for server to bind port

# 4. Start Followers (Background)
echo -e "${YELLOW}[4/4] Starting $NUM_FOLLOWERS Follower vehicles...${NC}"
for (( i=1; i<=NUM_FOLLOWERS; i++ ))
do
    # Calculate position: Leader at 0.0, followers behind by 30m each
    # Follower 2: 30m, Follower 3: 60m... (Since ID starts at 2)
    # Actually code uses ID=2 for first follower.
    # Initial Leader(1) Pos=0. 
    # Let's put followers ahead or behind? Usually behind.
    # Leader is at 0. Followers need to spawn behind? Or Code calculates gaps.
    # Code: Follower ID start at 2.
    
    # Leader is at 0. Let's spawn followers starting at 0 and let them adjust?
    # Or spawn them properly spaced: Leader=600m, F2=570m, F3=540m...
    # Leader default is Pos=0. 
    # Let's assume circular track or just linear.
    # Let's spawn them:
    # ID: 2, 3, 4, 5
    POS=$(( (NUM_FOLLOWERS - i + 1) * 30 )) # Just some arbitrary positions
    
    ./follow $((i + 1)) 60.0 $POS > follower_$((i+1)).log 2>&1 &
    echo -e "Follower $((i+1)) started at Pos=${POS}m. Logs: follower_$((i+1)).log"
    sleep 0.5
done

echo -e "${GREEN}All systems running.${NC}"
echo -e "${YELLOW}Test will run for ${TEST_DURATION} seconds...${NC}"

# Progress bar / Wait
for (( i=1; i<=TEST_DURATION; i++ ))
do
   sleep 1
   echo -ne "."
done
echo ""

# 5. Stop and Collect Stats
echo -e "${YELLOW}Stopping system and collecting WCET stats...${NC}"

# Send SIGINT (Ctrl+C) to Leader to trigger stats printing
kill -SIGINT $LEADER_PID
wait $LEADER_PID 2>/dev/null

echo -e "${GREEN}==================================================${NC}"
echo -e "${GREEN}               WCET RESULTS (LEADER)              ${NC}"
echo -e "${GREEN}==================================================${NC}"

# Extract WCET lines from leader log and save to file
grep "\[WCET\]" leader.log | tee wcet_results.txt

echo -e "${GREEN}==================================================${NC}"
echo -e "${YELLOW}WCET results saved to: ${GREEN}wcet_results.txt${NC}"
echo -e "${YELLOW}Detailed logs are in 'leader.log' and 'follower_*.log'${NC}"
pkill -f "./follow" # Kill remaining followers
echo -e "${GREEN}Done.${NC}"
