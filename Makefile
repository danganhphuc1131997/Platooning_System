# Makefile for Platooning System
# Builds all programs with pthread support

CXX = g++
CXXFLAGS = -std=c++17 -pthread -Wall
TARGET_DIR = .

# Main programs
LEAD = $(TARGET_DIR)/lead
FOLLOW = $(TARGET_DIR)/follow

# Input programs for WSL/non-GUI environments
LEADER_INPUT = $(TARGET_DIR)/leader_input
FOLLOWER_INPUT = $(TARGET_DIR)/follower_input

# All targets
all: $(LEAD) $(FOLLOW) $(LEADER_INPUT) $(FOLLOWER_INPUT)

# Build lead vehicle program
$(LEAD): lead.cpp lead.h vehicle.h message.h system_config.h
	$(CXX) $(CXXFLAGS) lead.cpp -o $(LEAD)

# Build follow vehicle program
$(FOLLOW): follow.cpp follow.h vehicle.h message.h system_config.h
	$(CXX) $(CXXFLAGS) follow.cpp -o $(FOLLOW)

# Build leader input program
$(LEADER_INPUT): leader_input.cpp
	$(CXX) $(CXXFLAGS) leader_input.cpp -o $(LEADER_INPUT)

# Build follower input program
$(FOLLOWER_INPUT): follower_input.cpp
	$(CXX) $(CXXFLAGS) follower_input.cpp -o $(FOLLOWER_INPUT)

# Build only main programs (lead and follow)
main: $(LEAD) $(FOLLOW)

# Build only input programs
input: $(LEADER_INPUT) $(FOLLOWER_INPUT)

# Clean all binaries
clean:
	rm -f $(LEAD) $(FOLLOW) $(LEADER_INPUT) $(FOLLOWER_INPUT)
	rm -f /tmp/*_event_fifo /tmp/leader_event_fifo

# Clean only input programs
clean-input:
	rm -f $(LEADER_INPUT) $(FOLLOWER_INPUT)

# Help target
help:
	@echo "Platooning System Build Instructions"
	@echo "====================================="
	@echo "make          - Build all programs"
	@echo "make main     - Build lead and follow programs only"
	@echo "make input    - Build input programs only"
	@echo "make clean    - Remove all binaries and FIFOs"
	@echo ""
	@echo "Programs:"
	@echo "  lead            - Leader vehicle"
	@echo "  follow          - Follower vehicle"
	@echo "  leader_input    - Send events to leader"
	@echo "  follower_input  - Send events to follower"

.PHONY: all main input clean clean-input help
