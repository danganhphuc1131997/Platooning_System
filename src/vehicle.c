/**
 * vehicle.c - Single vehicle process
 * 
 * This is the main entry point for ONE vehicle.
 * Run multiple instances to create a platoon:
 *   ./vehicle 0 4   # Vehicle 0 (leader), 4 total vehicles
 *   ./vehicle 1 4   # Vehicle 1 (follower)
 *   ./vehicle 2 4   # Vehicle 2 (follower)
 *   ./vehicle 3 4   # Vehicle 3 (follower)
 */

#include "common.h"
#include "ipc.h"
#include "threads.h"
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>

/*============================================================================
 * GLOBAL STATE (for this vehicle process)
 *============================================================================*/

VehicleData g_vehicle;
double g_other_pos[MAX_VEHICLES];
double g_other_speed[MAX_VEHICLES];

static volatile bool g_shutdown = false;

/*============================================================================
 * SIGNAL HANDLER
 *============================================================================*/

static void signal_handler(int sig) {
    (void)sig;
    g_shutdown = true;
    
    pthread_mutex_lock(&g_vehicle.mutex);
    g_vehicle.running = false;
    pthread_mutex_unlock(&g_vehicle.mutex);
}

/*============================================================================
 * INITIALIZATION
 *============================================================================*/

static void init_vehicle(int id, int num_vehicles) {
    memset(&g_vehicle, 0, sizeof(g_vehicle));
    memset(g_other_pos, 0, sizeof(g_other_pos));
    memset(g_other_speed, 0, sizeof(g_other_speed));
    
    g_vehicle.id = id;
    g_vehicle.num_vehicles = num_vehicles;
    
    // First vehicle is leader
    if (id == 0) {
        g_vehicle.role = ROLE_LEADER;
        g_vehicle.state = STATE_COUPLED;
        g_vehicle.predecessor_id = -1;
        g_vehicle.leader_id = 0;
        g_vehicle.pos = 0.0;
        g_vehicle.speed = 5.0;  // Leader starts moving
    } else {
        g_vehicle.role = ROLE_FOLLOWER;
        g_vehicle.state = STATE_IDLE;
        g_vehicle.predecessor_id = id - 1;
        g_vehicle.leader_id = 0;
        g_vehicle.pos = -id * DESIRED_GAP;  // Spaced behind
        g_vehicle.speed = 0.0;
    }
    
    // Initialize other vehicles' positions (approximate)
    for (int i = 0; i < num_vehicles; i++) {
        g_other_pos[i] = -i * DESIRED_GAP;
        g_other_speed[i] = (i == 0) ? 5.0 : 0.0;
    }
    
    pthread_mutex_init(&g_vehicle.mutex, NULL);
    pthread_cond_init(&g_vehicle.sensor_ready, NULL);
    pthread_cond_init(&g_vehicle.decision_ready, NULL);
    
    g_vehicle.running = true;
}

/*============================================================================
 * STATUS DISPLAY
 *============================================================================*/

static void print_status(void) {
    pthread_mutex_lock(&g_vehicle.mutex);
    
    printf("\033[2J\033[H");  // Clear screen
    printf("╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║  VEHICLE %d - %s                                              \n",
           g_vehicle.id, role_str(g_vehicle.role));
    printf("╠═══════════════════════════════════════════════════════════════╣\n");
    printf("║  State: %-12s │ Leader: V%d │ Pred: V%d               \n",
           state_str(g_vehicle.state), g_vehicle.leader_id, g_vehicle.predecessor_id);
    printf("╠═══════════════════════════════════════════════════════════════╣\n");
    printf("║  Position: %8.2f m                                        \n", g_vehicle.pos);
    printf("║  Speed:    %8.2f m/s                                      \n", g_vehicle.speed);
    printf("║  Accel:    %8.2f m/s²                                     \n", g_vehicle.accel);
    printf("╠═══════════════════════════════════════════════════════════════╣\n");
    printf("║  Sensor Gap: %6.2f m │ Rel Speed: %6.2f m/s              \n",
           g_vehicle.sensor_gap, g_vehicle.sensor_rel_speed);
    printf("╠═══════════════════════════════════════════════════════════════╣\n");
    printf("║  Threads: COMM ✓ │ SENSOR ✓ │ DECISION ✓ │ CONTROL ✓        \n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n");
    printf("\nPress Ctrl+C to stop.\n");
    
    pthread_mutex_unlock(&g_vehicle.mutex);
}

/*============================================================================
 * MAIN
 *============================================================================*/

int main(int argc, char **argv) {
    if (argc < 3) {
        printf("Usage: %s <vehicle_id> <num_vehicles> [-e]\n", argv[0]);
        printf("  vehicle_id:   0 = leader, 1..N-1 = followers\n");
        printf("  num_vehicles: total number of vehicles in platoon\n");
        printf("  -e:           simulate leader exit after 5 seconds\n");
        printf("\nExample (run in separate terminals):\n");
        printf("  %s 0 4      # Leader\n", argv[0]);
        printf("  %s 0 4 -e   # Leader (exits after 5s to test election)\n", argv[0]);
        printf("  %s 1 4      # Follower 1\n", argv[0]);
        printf("  %s 2 4      # Follower 2\n", argv[0]);
        printf("  %s 3 4      # Follower 3\n", argv[0]);
        return 1;
    }
    
    int id = atoi(argv[1]);
    int num_vehicles = atoi(argv[2]);
    bool simulate_exit = (argc > 3 && argv[3][0] == '-' && argv[3][1] == 'e');
    
    if (id < 0 || id >= MAX_VEHICLES) {
        fprintf(stderr, "Invalid vehicle_id (0-%d)\n", MAX_VEHICLES - 1);
        return 1;
    }
    
    if (num_vehicles < 2 || num_vehicles > MAX_VEHICLES) {
        fprintf(stderr, "Invalid num_vehicles (2-%d)\n", MAX_VEHICLES);
        return 1;
    }
    
    // Setup signal handler
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    printf("╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║  DISTRIBUTED PLATOON SYSTEM v2                                ║\n");
    printf("║  Vehicle %d of %d                                              ║\n",
           id, num_vehicles);
    printf("║                                                               ║\n");
    printf("║  Architecture:                                                ║\n");
    printf("║  - DISTRIBUTED: Each vehicle = 1 process                      ║\n");
    printf("║  - PARALLEL: 4 threads (Comm, Sensor, Decision, Control)      ║\n");
    printf("║  - IPC: POSIX Message Queues                                  ║\n");
    printf("╚═══════════════════════════════════════════════════════════════╝\n\n");
    
    // Initialize
    init_vehicle(id, num_vehicles);
    
    // Initialize IPC
    if (ipc_init(id, num_vehicles) != 0) {
        fprintf(stderr, "Failed to initialize IPC\n");
        return 1;
    }
    
    // Start threads
    VehicleThreads threads;
    if (threads_start(&threads, &g_vehicle) != 0) {
        fprintf(stderr, "Failed to start threads\n");
        ipc_shutdown(id, id == 0);
        return 1;
    }
    
    printf("[V%d] All threads started. Running...\n\n", id);
    
    // Monitor loop
    int monitor_count = 0;
    while (!g_shutdown) {
        print_status();
        util_msleep(500);
        monitor_count++;
        
        // Simulate leader exit after 5 seconds (10 x 500ms)
        if (simulate_exit && id == 0 && monitor_count >= 10) {
            printf("\n[V%d] *** SIMULATING LEADER EXIT ***\n", id);
            break;
        }
    }
    
    printf("\n[V%d] Shutting down...\n", id);
    
    // Stop threads
    threads_stop(&threads, &g_vehicle);
    
    // Shutdown IPC
    ipc_shutdown(id, id == 0);
    
    // Cleanup
    pthread_mutex_destroy(&g_vehicle.mutex);
    pthread_cond_destroy(&g_vehicle.sensor_ready);
    pthread_cond_destroy(&g_vehicle.decision_ready);
    
    printf("[V%d] Stopped.\n", id);
    
    return 0;
}
