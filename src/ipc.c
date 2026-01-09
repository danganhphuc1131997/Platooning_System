/**
 * ipc.c - POSIX Message Queue implementation for inter-vehicle communication
 * 
 * Each vehicle has its own receive queue: /platoon_v0, /platoon_v1, etc.
 * Broadcasting = sending to all other vehicles' queues
 */

#include "ipc.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>

static mqd_t g_my_queue = (mqd_t)-1;
static mqd_t g_other_queues[MAX_VEHICLES];
static int   g_my_id = -1;
static int   g_num_vehicles = 0;

/**
 * Get queue name for a vehicle
 */
static void get_queue_name(int id, char *buf, size_t len) {
    snprintf(buf, len, "%s%d", MQ_NAME_PREFIX, id);
}

int ipc_init(int vehicle_id, int num_vehicles) {
    g_my_id = vehicle_id;
    g_num_vehicles = num_vehicles;
    
    char qname[64];
    struct mq_attr attr = {
        .mq_flags = 0,
        .mq_maxmsg = MQ_MAX_MSGS,
        .mq_msgsize = sizeof(Message),
        .mq_curmsgs = 0
    };
    
    // Initialize other queues array
    for (int i = 0; i < MAX_VEHICLES; i++) {
        g_other_queues[i] = (mqd_t)-1;
    }
    
    // Create my receive queue
    get_queue_name(vehicle_id, qname, sizeof(qname));
    
    // First try to unlink old queue (in case of previous crash)
    mq_unlink(qname);
    
    g_my_queue = mq_open(qname, O_CREAT | O_RDONLY | O_NONBLOCK, 0644, &attr);
    if (g_my_queue == (mqd_t)-1) {
        perror("mq_open (my queue)");
        return -1;
    }
    
    printf("[V%d IPC] Created receive queue: %s\n", vehicle_id, qname);
    
    // Wait a bit for other vehicles to create their queues
    util_msleep(100 + vehicle_id * 50);
    
    // Open queues to other vehicles for sending
    for (int i = 0; i < num_vehicles; i++) {
        if (i == vehicle_id) continue;
        
        get_queue_name(i, qname, sizeof(qname));
        
        // Try multiple times (other processes may not have started yet)
        for (int retry = 0; retry < 10; retry++) {
            g_other_queues[i] = mq_open(qname, O_WRONLY | O_NONBLOCK);
            if (g_other_queues[i] != (mqd_t)-1) {
                printf("[V%d IPC] Connected to V%d queue\n", vehicle_id, i);
                break;
            }
            util_msleep(100);
        }
        
        if (g_other_queues[i] == (mqd_t)-1) {
            printf("[V%d IPC] Warning: Could not connect to V%d\n", vehicle_id, i);
        }
    }
    
    return 0;
}

void ipc_shutdown(int vehicle_id, bool is_leader) {
    // Close my queue
    if (g_my_queue != (mqd_t)-1) {
        mq_close(g_my_queue);
        
        char qname[64];
        get_queue_name(vehicle_id, qname, sizeof(qname));
        mq_unlink(qname);
        
        printf("[V%d IPC] Closed and unlinked queue\n", vehicle_id);
    }
    
    // Close connections to other queues
    for (int i = 0; i < MAX_VEHICLES; i++) {
        if (g_other_queues[i] != (mqd_t)-1) {
            mq_close(g_other_queues[i]);
        }
    }
}

int ipc_broadcast(int sender_id, int num_vehicles, const Message *msg) {
    int sent = 0;
    
    for (int i = 0; i < num_vehicles; i++) {
        if (i == sender_id) continue;
        
        if (g_other_queues[i] != (mqd_t)-1) {
            if (mq_send(g_other_queues[i], (const char*)msg, sizeof(Message), 0) == 0) {
                sent++;
            }
            // Ignore EAGAIN (queue full) - like packet drop in real network
        }
    }
    
    return sent;
}

int ipc_send(int target_id, const Message *msg) {
    if (target_id < 0 || target_id >= MAX_VEHICLES) return -1;
    if (g_other_queues[target_id] == (mqd_t)-1) return -1;
    
    if (mq_send(g_other_queues[target_id], (const char*)msg, sizeof(Message), 0) == 0) {
        return 0;
    }
    
    return -1;
}

int ipc_receive(int vehicle_id, Message *out) {
    (void)vehicle_id;  // We use g_my_queue
    
    ssize_t bytes = mq_receive(g_my_queue, (char*)out, sizeof(Message), NULL);
    
    if (bytes > 0) {
        return 0;
    }
    
    return -1;  // No message or error
}

int ipc_receive_timeout(int vehicle_id, Message *out, int timeout_ms) {
    (void)vehicle_id;
    
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec += timeout_ms / 1000;
    ts.tv_nsec += (timeout_ms % 1000) * 1000000L;
    if (ts.tv_nsec >= 1000000000L) {
        ts.tv_sec++;
        ts.tv_nsec -= 1000000000L;
    }
    
    // Need to temporarily make queue blocking for timed receive
    struct mq_attr old_attr, new_attr;
    mq_getattr(g_my_queue, &old_attr);
    new_attr = old_attr;
    new_attr.mq_flags = 0;  // Remove O_NONBLOCK
    mq_setattr(g_my_queue, &new_attr, NULL);
    
    ssize_t bytes = mq_timedreceive(g_my_queue, (char*)out, sizeof(Message), NULL, &ts);
    
    // Restore non-blocking
    mq_setattr(g_my_queue, &old_attr, NULL);
    
    return (bytes > 0) ? 0 : -1;
}
