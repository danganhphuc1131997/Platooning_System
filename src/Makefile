# Makefile for Distributed Platoon System v2
#
# Architecture:
# - DISTRIBUTED: Each vehicle = separate process
# - PARALLEL: Each vehicle has 4 threads (Comm, Sensor, Decision, Control)
# - IPC: POSIX Message Queues

CC = gcc
CFLAGS = -Wall -Wextra -pthread -O2
LDFLAGS = -pthread -lrt

# Source files
VEHICLE_SRCS = vehicle.c threads.c ipc.c
LAUNCHER_SRCS = launcher.c

# Object files
VEHICLE_OBJS = $(VEHICLE_SRCS:.c=.o)

# Targets
all: vehicle launcher

vehicle: $(VEHICLE_OBJS)
	$(CC) $(LDFLAGS) -o $@ $^

launcher: launcher.c
	$(CC) $(CFLAGS) -o $@ $<

%.o: %.c common.h ipc.h threads.h
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm -f *.o vehicle launcher
	rm -f /dev/mqueue/platoon_v*

# Run targets
run: vehicle launcher
	./launcher 4

run-term: vehicle launcher
	./launcher 4 -t

# Run individual vehicles manually (for debugging)
run-v0: vehicle
	./vehicle 0 4

run-v1: vehicle
	./vehicle 1 4

run-v2: vehicle
	./vehicle 2 4

run-v3: vehicle
	./vehicle 3 4

# Debug build
debug: CFLAGS += -g -DDEBUG
debug: clean all

.PHONY: all clean run run-term run-v0 run-v1 run-v2 run-v3 debug
