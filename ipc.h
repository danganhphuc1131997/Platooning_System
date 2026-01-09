/**
 * ipc.h - Inter-Process Communication via POSIX Message Queues
 * 
 * Simulates DSRC wireless broadcast between vehicle processes
 */

#ifndef IPC_H
#define IPC_H

#include "common.h"

/**
 * Initialize IPC for a vehicle
 * Creates/opens message queues for sending and receiving
 */
int ipc_init(int vehicle_id, int num_vehicles);

/**
 * Shutdown IPC, close and unlink queues
 */
void ipc_shutdown(int vehicle_id, bool is_leader);

/**
 * Broadcast message to all other vehicles
 */
int ipc_broadcast(int sender_id, int num_vehicles, const Message *msg);

/**
 * Send message to specific vehicle
 */
int ipc_send(int target_id, const Message *msg);

/**
 * Receive message (non-blocking)
 * Returns 0 if message received, -1 if queue empty
 */
int ipc_receive(int vehicle_id, Message *out);

/**
 * Receive message (blocking with timeout)
 * Returns 0 if message received, -1 on timeout/error
 */
int ipc_receive_timeout(int vehicle_id, Message *out, int timeout_ms);

#endif /* IPC_H */
