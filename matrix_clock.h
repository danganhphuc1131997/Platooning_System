/**
 * @file matrix_clock.h
 *
 * @brief Matrix Clock implementation for distributed clock synchronization
 * 
 * Matrix Clock extends Vector Clock to track not only each node's logical time,
 * but also what each node knows about every other node's time.
 * 
 * Matrix[i][j] = Node i's view of Node j's logical clock
 */

#ifndef MATRIX_CLOCK_H
#define MATRIX_CLOCK_H

#include <cstdint>
#include <algorithm>
#include <iostream>
#include "system_config.h"

/// Maximum number of processes for matrix clock
constexpr int MAX_PROCESSES = 16;

// Matrix Clock structure
struct MatrixClock {
    int matrix[MAX_PROCESSES][MAX_PROCESSES] = {{0}};
    int processId = 0;
    int numProcesses = 0;

    // Initialize matrix clock for a process
    void init(int id, int n) {
        processId = id;
        numProcesses = n;
        for (int i = 0; i < MAX_PROCESSES; ++i) {
            for (int j = 0; j < MAX_PROCESSES; ++j) {
                matrix[i][j] = 0;
            }
        }
    }

    // Increment local clock on local event
    void localEvent() {
        matrix[processId][processId]++;
    }

    // Prepare clock before sending (increment and return copy)
    void onSend() {
        matrix[processId][processId]++;
    }

    // Merge received matrix clock
    void onReceive(const MatrixClock& received) {
        int senderId = received.processId;
        
        // Update our view of sender's row with their latest knowledge
        for (int j = 0; j < MAX_PROCESSES; ++j) {
            matrix[senderId][j] = std::max(matrix[senderId][j], received.matrix[senderId][j]);
        }
        
        // Take element-wise max for all other rows (what we and sender both know)
        for (int i = 0; i < MAX_PROCESSES; ++i) {
            if (i == processId) continue; // skip our own row for now
            for (int j = 0; j < MAX_PROCESSES; ++j) {
                matrix[i][j] = std::max(matrix[i][j], received.matrix[i][j]);
            }
        }
        
        // Update our own row with max of what we knew and what sender knows about others
        for (int j = 0; j < MAX_PROCESSES; ++j) {
            if (j != processId) {
                matrix[processId][j] = std::max(matrix[processId][j], received.matrix[senderId][j]);
            }
        }
        
        // Increment own clock
        matrix[processId][processId]++;
    }

    // Check if all processes know about a particular event
    bool allKnow(int eventProcess, int eventTime) const {
        for (int i = 0; i < numProcesses; ++i) {
            if (matrix[i][eventProcess] < eventTime) {
                return false;
            }
        }
        return true;
    }

    // Get own logical time
    int getLocalTime() const {
        return matrix[processId][processId];
    }
};

#endif // MATRIX_CLOCK_H
