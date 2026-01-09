/**
 * threads.c - Implementation of intra-vehicle parallel threads
 * 
 * Thread Pipeline:
 *   Sensor → Decision → Control
 *      ↑         ↓
 *     Comm ←→ IPC (other vehicles)
 */

#include "threads.h"
#include "ipc.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/*============================================================================
 * GLOBAL VEHICLE STATE (extern from vehicle.c)
 *============================================================================*/

extern VehicleData g_vehicle;
extern double g_other_pos[MAX_VEHICLES];
extern double g_other_speed[MAX_VEHICLES];

/*============================================================================
 * COMMUNICATION THREAD
 * 
 * Responsible for:
 * - Receiving messages from other vehicles
 * - Sending heartbeats periodically
 * - Processing coupling requests (if leader)
 * - Forwarding messages to decision thread via shared state
 *============================================================================*/

void* comm_thread(void *arg) {
    (void)arg;
    int step = 0;
    
    printf("[V%d COMM] Thread started\n", g_vehicle.id);
    
    while (1) {
        pthread_mutex_lock(&g_vehicle.mutex);
        if (!g_vehicle.running) {
            pthread_mutex_unlock(&g_vehicle.mutex);
            break;
        }
        int my_id = g_vehicle.id;
        int num_vehicles = g_vehicle.num_vehicles;
        VehicleRole my_role = g_vehicle.role;
        pthread_mutex_unlock(&g_vehicle.mutex);
        
        // ─────────────────────────────────────────────────────────────
        // RECEIVE: Process all pending messages
        // ─────────────────────────────────────────────────────────────
        Message msg;
        while (ipc_receive(my_id, &msg) == 0) {
            pthread_mutex_lock(&g_vehicle.mutex);
            
            switch (msg.type) {
                case MSG_HEARTBEAT:
                case MSG_STATE_UPDATE:
                    // Update other vehicle's state
                    if (msg.sender_id >= 0 && msg.sender_id < MAX_VEHICLES) {
                        g_vehicle.last_heartbeat[msg.sender_id] = 0;
                        g_other_pos[msg.sender_id] = msg.pos;
                        g_other_speed[msg.sender_id] = msg.speed;
                    }
                    break;
                    
                case MSG_COUPLE_REQUEST:
                    if (my_role == ROLE_LEADER) {
                        printf("[V%d COMM] Received COUPLE_REQUEST from V%d\n", 
                               my_id, msg.sender_id);
                        
                        // Accept the request
                        Message reply = {
                            .type = MSG_COUPLE_ACCEPT,
                            .sender_id = my_id,
                            .target_id = msg.sender_id,
                            .predecessor_id = (msg.sender_id > 0) ? msg.sender_id - 1 : my_id,
                            .seq_num = g_vehicle.seq_num++
                        };
                        pthread_mutex_unlock(&g_vehicle.mutex);
                        ipc_send(msg.sender_id, &reply);
                        pthread_mutex_lock(&g_vehicle.mutex);
                        
                        printf("[V%d COMM] Sent COUPLE_ACCEPT to V%d (pred=V%d)\n",
                               my_id, msg.sender_id, reply.predecessor_id);
                    }
                    break;
                    
                case MSG_COUPLE_ACCEPT:
                    if (msg.target_id == my_id) {
                        g_vehicle.state = STATE_COUPLED;
                        g_vehicle.predecessor_id = msg.predecessor_id;
                        g_vehicle.leader_id = msg.sender_id;
                        printf("[V%d COMM] Coupling ACCEPTED, predecessor=V%d\n",
                               my_id, msg.predecessor_id);
                    }
                    break;
                    
                case MSG_EMERGENCY_BRAKE:
                    g_vehicle.state = STATE_EMERGENCY;
                    g_vehicle.cmd_emergency = true;
                    printf("[V%d COMM] EMERGENCY from V%d!\n", my_id, msg.sender_id);
                    break;
                    
                case MSG_LEADER_ANNOUNCE:
                    g_vehicle.leader_id = msg.sender_id;
                    if (msg.sender_id != my_id) {
                        g_vehicle.role = ROLE_FOLLOWER;
                    }
                    g_vehicle.election_in_progress = false;
                    printf("[V%d COMM] New leader is V%d\n", my_id, msg.sender_id);
                    break;
                    
                case MSG_LEADER_ELECTION:
                    g_vehicle.election_in_progress = true;
                    // Simple bully: lowest ID wins, check after timeout
                    break;
                    
                default:
                    break;
            }
            
            pthread_mutex_unlock(&g_vehicle.mutex);
        }
        
        // ─────────────────────────────────────────────────────────────
        // SEND: Heartbeat every HEARTBEAT_INTERVAL
        // ─────────────────────────────────────────────────────────────
        if (step % HEARTBEAT_INTERVAL == 0) {
            pthread_mutex_lock(&g_vehicle.mutex);
            Message hb = {
                .type = MSG_HEARTBEAT,
                .sender_id = my_id,
                .target_id = -1,
                .seq_num = g_vehicle.seq_num++,
                .pos = g_vehicle.pos,
                .speed = g_vehicle.speed,
                .accel = g_vehicle.accel
            };
            pthread_mutex_unlock(&g_vehicle.mutex);
            
            ipc_broadcast(my_id, num_vehicles, &hb);
        }
        
        // ─────────────────────────────────────────────────────────────
        // UPDATE: Heartbeat counters for other vehicles
        // ─────────────────────────────────────────────────────────────
        pthread_mutex_lock(&g_vehicle.mutex);
        for (int i = 0; i < num_vehicles; i++) {
            if (i != my_id) {
                g_vehicle.last_heartbeat[i]++;
            }
        }
        pthread_mutex_unlock(&g_vehicle.mutex);
        
        step++;
        util_msleep(DT_MS);
    }
    
    printf("[V%d COMM] Thread stopped\n", g_vehicle.id);
    return NULL;
}

/*============================================================================
 * SENSOR THREAD
 * 
 * Simulates:
 * - Lidar: measures distance to predecessor
 * - Radar: measures relative speed
 * - Camera: obstacle detection (random for simulation)
 *============================================================================*/

void* sensor_thread(void *arg) {
    (void)arg;
    
    printf("[V%d SENSOR] Thread started\n", g_vehicle.id);
    
    while (1) {
        pthread_mutex_lock(&g_vehicle.mutex);
        if (!g_vehicle.running) {
            pthread_mutex_unlock(&g_vehicle.mutex);
            break;
        }
        
        int pred_id = g_vehicle.predecessor_id;
        double my_pos = g_vehicle.pos;
        double my_speed = g_vehicle.speed;
        
        // ─────────────────────────────────────────────────────────────
        // LIDAR: Measure gap to predecessor
        // ─────────────────────────────────────────────────────────────
        if (pred_id >= 0 && pred_id < MAX_VEHICLES) {
            double pred_pos = g_other_pos[pred_id];
            double pred_speed = g_other_speed[pred_id];
            
            // Add some noise to simulate real sensor
            double noise = ((rand() % 100) - 50) / 500.0;  // ±0.1m
            g_vehicle.sensor_gap = pred_pos - my_pos + noise;
            
            // Relative speed
            g_vehicle.sensor_rel_speed = pred_speed - my_speed;
        } else {
            g_vehicle.sensor_gap = 0;
            g_vehicle.sensor_rel_speed = 0;
        }
        
        // ─────────────────────────────────────────────────────────────
        // OBSTACLE DETECTION: Random simulation
        // ─────────────────────────────────────────────────────────────
        // 0.1% chance of obstacle per cycle
        g_vehicle.sensor_obstacle = (rand() % 1000 == 0);
        
        // Signal that sensor data is ready
        pthread_cond_signal(&g_vehicle.sensor_ready);
        
        pthread_mutex_unlock(&g_vehicle.mutex);
        
        util_msleep(DT_MS / 2);  // Sensor runs faster than control
    }
    
    printf("[V%d SENSOR] Thread stopped\n", g_vehicle.id);
    return NULL;
}

/*============================================================================
 * DECISION THREAD
 * 
 * Makes high-level decisions:
 * - Request coupling when idle
 * - Trigger emergency brake if gap too small or obstacle
 * - Start election if leader timeout
 * - Calculate desired acceleration
 *============================================================================*/

void* decision_thread(void *arg) {
    (void)arg;
    int step = 0;
    bool couple_requested = false;
    
    printf("[V%d DECISION] Thread started\n", g_vehicle.id);
    
    // Initial delay
    util_msleep(500 + g_vehicle.id * 200);
    
    while (1) {
        pthread_mutex_lock(&g_vehicle.mutex);
        if (!g_vehicle.running) {
            pthread_mutex_unlock(&g_vehicle.mutex);
            break;
        }
        
        int my_id = g_vehicle.id;
        int num_vehicles = g_vehicle.num_vehicles;
        VehicleRole role = g_vehicle.role;
        VehicleState state = g_vehicle.state;
        int leader_id = g_vehicle.leader_id;
        
        // Read sensor data
        double gap = g_vehicle.sensor_gap;
        double rel_speed = g_vehicle.sensor_rel_speed;
        bool obstacle = g_vehicle.sensor_obstacle;
        
        // ─────────────────────────────────────────────────────────────
        // EMERGENCY CHECK
        // ─────────────────────────────────────────────────────────────
        if ((gap > 0 && gap < 3.0) || obstacle) {
            if (state != STATE_EMERGENCY) {
                g_vehicle.state = STATE_EMERGENCY;
                g_vehicle.cmd_emergency = true;
                g_vehicle.cmd_accel = -MAX_ACCEL;
                
                printf("[V%d DECISION] EMERGENCY! gap=%.1f obstacle=%d\n",
                       my_id, gap, obstacle);
                
                // Broadcast emergency
                Message emg = {
                    .type = MSG_EMERGENCY_BRAKE,
                    .sender_id = my_id,
                    .target_id = -1,
                    .seq_num = g_vehicle.seq_num++
                };
                pthread_mutex_unlock(&g_vehicle.mutex);
                ipc_broadcast(my_id, num_vehicles, &emg);
                pthread_mutex_lock(&g_vehicle.mutex);
            }
        }
        
        // ─────────────────────────────────────────────────────────────
        // LEADER TIMEOUT CHECK (for followers)
        // ─────────────────────────────────────────────────────────────
        if (role == ROLE_FOLLOWER && leader_id >= 0) {
            if (g_vehicle.last_heartbeat[leader_id] > HEARTBEAT_TIMEOUT &&
                !g_vehicle.election_in_progress) {
                
                printf("[V%d DECISION] Leader V%d timeout! Starting election\n",
                       my_id, leader_id);
                
                g_vehicle.election_in_progress = true;
                
                Message elec = {
                    .type = MSG_LEADER_ELECTION,
                    .sender_id = my_id,
                    .target_id = -1,
                    .priority = my_id,
                    .seq_num = g_vehicle.seq_num++
                };
                pthread_mutex_unlock(&g_vehicle.mutex);
                ipc_broadcast(my_id, num_vehicles, &elec);
                
                // Wait for election
                util_msleep(500);
                
                pthread_mutex_lock(&g_vehicle.mutex);
                
                // Check if I should become leader (lowest active ID)
                bool i_am_leader = true;
                for (int i = 0; i < my_id; i++) {
                    if (g_vehicle.last_heartbeat[i] < HEARTBEAT_TIMEOUT) {
                        i_am_leader = false;
                        break;
                    }
                }
                
                if (i_am_leader) {
                    g_vehicle.role = ROLE_LEADER;
                    g_vehicle.leader_id = my_id;
                    g_vehicle.election_in_progress = false;
                    
                    printf("[V%d DECISION] I am the new LEADER!\n", my_id);
                    
                    Message ann = {
                        .type = MSG_LEADER_ANNOUNCE,
                        .sender_id = my_id,
                        .target_id = -1,
                        .seq_num = g_vehicle.seq_num++
                    };
                    pthread_mutex_unlock(&g_vehicle.mutex);
                    ipc_broadcast(my_id, num_vehicles, &ann);
                    pthread_mutex_lock(&g_vehicle.mutex);
                }
            }
        }
        
        // ─────────────────────────────────────────────────────────────
        // COUPLING DECISION (for followers)
        // ─────────────────────────────────────────────────────────────
        if (role == ROLE_FOLLOWER && state == STATE_IDLE && !couple_requested) {
            printf("[V%d DECISION] Requesting coupling\n", my_id);
            
            g_vehicle.state = STATE_REQUESTING;
            
            Message req = {
                .type = MSG_COUPLE_REQUEST,
                .sender_id = my_id,
                .target_id = -1,
                .seq_num = g_vehicle.seq_num++
            };
            pthread_mutex_unlock(&g_vehicle.mutex);
            ipc_broadcast(my_id, num_vehicles, &req);
            couple_requested = true;
            pthread_mutex_lock(&g_vehicle.mutex);
        }
        
        // ─────────────────────────────────────────────────────────────
        // CONTROL DECISION: Calculate desired acceleration
        // ─────────────────────────────────────────────────────────────
        if (state == STATE_COUPLED && g_vehicle.predecessor_id >= 0) {
            double gap_error = gap - DESIRED_GAP;
            
            // PD controller
            double kp = 0.8;
            double kd = 0.3;
            double accel = kp * gap_error + kd * rel_speed;
            
            // Clamp
            if (accel > MAX_ACCEL) accel = MAX_ACCEL;
            if (accel < -MAX_ACCEL) accel = -MAX_ACCEL;
            
            g_vehicle.cmd_accel = accel;
        } else if (state == STATE_EMERGENCY) {
            g_vehicle.cmd_accel = -MAX_ACCEL;
        } else if (role == ROLE_LEADER) {
            // Leader maintains constant speed
            g_vehicle.cmd_accel = (5.0 - g_vehicle.speed) * 0.5;
        } else {
            // Coast
            g_vehicle.cmd_accel = (2.0 - g_vehicle.speed) * 0.3;
        }
        
        // Signal decision is ready
        pthread_cond_signal(&g_vehicle.decision_ready);
        
        pthread_mutex_unlock(&g_vehicle.mutex);
        
        step++;
        util_msleep(DT_MS);
    }
    
    printf("[V%d DECISION] Thread stopped\n", g_vehicle.id);
    return NULL;
}

/*============================================================================
 * CONTROL THREAD
 * 
 * Low-level control:
 * - Reads commanded acceleration from decision thread
 * - Applies physical limits
 * - Updates vehicle dynamics
 *============================================================================*/

void* control_thread(void *arg) {
    (void)arg;
    double dt = DT_MS / 1000.0;
    
    printf("[V%d CONTROL] Thread started\n", g_vehicle.id);
    
    while (1) {
        pthread_mutex_lock(&g_vehicle.mutex);
        if (!g_vehicle.running) {
            pthread_mutex_unlock(&g_vehicle.mutex);
            break;
        }
        
        // ─────────────────────────────────────────────────────────────
        // APPLY COMMANDED ACCELERATION
        // ─────────────────────────────────────────────────────────────
        double cmd_accel = g_vehicle.cmd_accel;
        
        // Apply limits
        if (cmd_accel > MAX_ACCEL) cmd_accel = MAX_ACCEL;
        if (cmd_accel < -MAX_ACCEL) cmd_accel = -MAX_ACCEL;
        
        g_vehicle.accel = cmd_accel;
        
        // ─────────────────────────────────────────────────────────────
        // UPDATE DYNAMICS
        // ─────────────────────────────────────────────────────────────
        g_vehicle.speed += g_vehicle.accel * dt;
        
        // Speed limits
        if (g_vehicle.speed < 0) g_vehicle.speed = 0;
        if (g_vehicle.speed > MAX_SPEED) g_vehicle.speed = MAX_SPEED;
        
        g_vehicle.pos += g_vehicle.speed * dt;
        
        pthread_mutex_unlock(&g_vehicle.mutex);
        
        util_msleep(DT_MS);
    }
    
    printf("[V%d CONTROL] Thread stopped\n", g_vehicle.id);
    return NULL;
}

/*============================================================================
 * THREAD MANAGEMENT
 *============================================================================*/

int threads_start(VehicleThreads *t, VehicleData *data) {
    data->running = true;
    
    if (pthread_create(&t->comm, NULL, comm_thread, NULL) != 0) {
        perror("pthread_create comm");
        return -1;
    }
    
    if (pthread_create(&t->sensor, NULL, sensor_thread, NULL) != 0) {
        perror("pthread_create sensor");
        return -1;
    }
    
    if (pthread_create(&t->decision, NULL, decision_thread, NULL) != 0) {
        perror("pthread_create decision");
        return -1;
    }
    
    if (pthread_create(&t->control, NULL, control_thread, NULL) != 0) {
        perror("pthread_create control");
        return -1;
    }
    
    return 0;
}

void threads_stop(VehicleThreads *t, VehicleData *data) {
    pthread_mutex_lock(&data->mutex);
    data->running = false;
    pthread_mutex_unlock(&data->mutex);
    
    pthread_join(t->comm, NULL);
    pthread_join(t->sensor, NULL);
    pthread_join(t->decision, NULL);
    pthread_join(t->control, NULL);
}
