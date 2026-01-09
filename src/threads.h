/**
 * threads.h - Intra-vehicle parallel threads
 * 
 * Each vehicle process has 4 threads:
 * 1. Comm Thread    - Handle IPC (send/receive messages)
 * 2. Sensor Thread  - Simulate sensor readings (lidar, radar)
 * 3. Decision Thread - Make decisions (couple, decouple, emergency)
 * 4. Control Thread - Execute control (speed, acceleration)
 */

#ifndef THREADS_H
#define THREADS_H

#include "common.h"

/*============================================================================
 * THREAD ENTRY POINTS
 *============================================================================*/

/**
 * Communication Thread
 * - Receives messages from other vehicles via IPC
 * - Sends heartbeats and state updates
 * - Processes coupling/decoupling requests (if leader)
 */
void* comm_thread(void *arg);

/**
 * Sensor Thread
 * - Simulates lidar/radar readings
 * - Measures gap to predecessor
 * - Detects obstacles
 * - Updates sensor data in shared state
 */
void* sensor_thread(void *arg);

/**
 * Decision Thread
 * - Reads sensor data and messages
 * - Decides on coupling/decoupling
 * - Triggers emergency if needed
 * - Outputs commanded acceleration
 */
void* decision_thread(void *arg);

/**
 * Control Thread
 * - Reads commanded acceleration from decision thread
 * - Applies acceleration limits
 * - Updates vehicle dynamics (pos, speed, accel)
 */
void* control_thread(void *arg);

/*============================================================================
 * THREAD MANAGEMENT
 *============================================================================*/

typedef struct {
    pthread_t comm;
    pthread_t sensor;
    pthread_t decision;
    pthread_t control;
} VehicleThreads;

/**
 * Start all threads for a vehicle
 */
int threads_start(VehicleThreads *t, VehicleData *data);

/**
 * Stop all threads (set running = false and join)
 */
void threads_stop(VehicleThreads *t, VehicleData *data);

#endif /* THREADS_H */
