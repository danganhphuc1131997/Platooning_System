/**
 * launcher.c - Launch all vehicle processes
 * 
 * This is a convenience program that spawns all vehicle processes
 * and manages them (kill all on Ctrl+C)
 */

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <sys/wait.h>
#include <sys/types.h>

#define MAX_VEHICLES 8

static pid_t g_pids[MAX_VEHICLES];
static int g_num_vehicles = 0;
static volatile int g_shutdown = 0;

static void signal_handler(int sig) {
    (void)sig;
    g_shutdown = 1;
}

static void cleanup_mqueues(int num) {
    for (int i = 0; i < num; i++) {
        char cmd[128];
        snprintf(cmd, sizeof(cmd), "rm -f /dev/mqueue/platoon_v%d 2>/dev/null", i);
        system(cmd);
    }
}

int main(int argc, char **argv) {
    if (argc < 2) {
        printf("Usage: %s <num_vehicles> [-t terminal]\n", argv[0]);
        printf("  num_vehicles: 2-%d\n", MAX_VEHICLES);
        printf("  -t: launch each vehicle in separate terminal (requires xterm)\n");
        printf("\nExample:\n");
        printf("  %s 4        # 4 vehicles in same terminal (compact output)\n", argv[0]);
        printf("  %s 4 -t     # 4 vehicles in separate xterm windows\n", argv[0]);
        return 1;
    }
    
    g_num_vehicles = atoi(argv[1]);
    if (g_num_vehicles < 2) g_num_vehicles = 2;
    if (g_num_vehicles > MAX_VEHICLES) g_num_vehicles = MAX_VEHICLES;
    
    int use_terminal = (argc > 2 && argv[2][0] == '-' && argv[2][1] == 't');
    
    // Setup signal handler
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // Cleanup old message queues
    cleanup_mqueues(g_num_vehicles);
    
    printf("╔═══════════════════════════════════════════════════════════════════════════╗\n");
    printf("║  DISTRIBUTED PLATOON LAUNCHER                                             ║\n");
    printf("║  Spawning %d vehicle processes...                                          ║\n",
           g_num_vehicles);
    printf("╚═══════════════════════════════════════════════════════════════════════════╝\n\n");
    
    // Fork vehicle processes
    for (int i = 0; i < g_num_vehicles; i++) {
        pid_t pid = fork();
        
        if (pid < 0) {
            perror("fork");
            // Kill already started processes
            for (int j = 0; j < i; j++) {
                kill(g_pids[j], SIGTERM);
            }
            return 1;
        }
        
        if (pid == 0) {
            // Child process
            char id_str[16], num_str[16];
            snprintf(id_str, sizeof(id_str), "%d", i);
            snprintf(num_str, sizeof(num_str), "%d", g_num_vehicles);
            
            if (use_terminal) {
                // Launch in separate terminal
                char title[64];
                snprintf(title, sizeof(title), "Vehicle %d (%s)", 
                         i, (i == 0) ? "LEADER" : "FOLLOWER");
                execlp("xterm", "xterm", 
                       "-title", title,
                       "-geometry", "70x25",
                       "-e", "./vehicle", id_str, num_str, 
                       NULL);
            } else {
                // Launch directly (output will mix)
                execl("./vehicle", "./vehicle", id_str, num_str, NULL);
            }
            
            perror("exec");
            exit(1);
        }
        
        // Parent
        g_pids[i] = pid;
        printf("[Launcher] Started Vehicle %d (PID %d)\n", i, pid);
        
        // Small delay between launches
        usleep(200000);  // 200ms
    }
    
    printf("\n[Launcher] All %d vehicles launched.\n", g_num_vehicles);
    printf("[Launcher] Press Ctrl+C to stop all vehicles.\n\n");
    
    // Wait for signal or child exit
    while (!g_shutdown) {
        // Check if any child died
        int status;
        pid_t died = waitpid(-1, &status, WNOHANG);
        if (died > 0) {
            printf("[Launcher] Vehicle (PID %d) exited\n", died);
        }
        usleep(500000);  // 500ms
    }
    
    printf("\n[Launcher] Stopping all vehicles...\n");
    
    // Send SIGTERM to all children
    for (int i = 0; i < g_num_vehicles; i++) {
        if (g_pids[i] > 0) {
            printf("[Launcher] Stopping Vehicle %d (PID %d)\n", i, g_pids[i]);
            kill(g_pids[i], SIGTERM);
        }
    }
    
    // Wait for all to exit
    for (int i = 0; i < g_num_vehicles; i++) {
        if (g_pids[i] > 0) {
            waitpid(g_pids[i], NULL, 0);
        }
    }
    
    // Cleanup message queues
    cleanup_mqueues(g_num_vehicles);
    
    printf("[Launcher] All vehicles stopped.\n");
    
    return 0;
}
