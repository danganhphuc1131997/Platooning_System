/**
 * @file leader_input.cpp
 * @brief Standalone program for sending events to the leader vehicle via FIFO
 * 
 * This program can be run in a separate terminal/tmux pane to send events
 * to the leader vehicle without relying on automatic terminal launching.
 * 
 * Usage: ./leader_input
 */

#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <cstring>
#include <string>

#define EVENT_FIFO "/tmp/leader_event_fifo"

int main() {
    std::cout << "=== Leader Event Input Program ===\n";
    std::cout << "Waiting for leader to create FIFO...\n";

    // Wait for FIFO to exist (created by leader)
    int attempts = 0;
    while (access(EVENT_FIFO, F_OK) != 0 && attempts < 30) {
        sleep(1);
        attempts++;
    }

    if (attempts >= 30) {
        std::cerr << "Error: FIFO not found. Make sure leader is running first.\n";
        return 1;
    }

    std::cout << "Connected to leader!\n\n";

    // Open FIFO for writing
    int fd = open(EVENT_FIFO, O_WRONLY);
    if (fd == -1) {
        std::cerr << "Failed to open FIFO: " << strerror(errno) << std::endl;
        return 1;
    }

    int choice;
    while (true) {
        // Display menu
        std::cout << "\n========================================\n";
        std::cout << "[LEADER EVENT INPUT]\n";
        std::cout << "========================================\n";
        std::cout << "1: Obstacle detected (leader stops)\n";
        std::cout << "2: Traffic light RED\n";
        std::cout << "3: Traffic light GREEN\n";
        std::cout << "4: Cut-in vehicle alert\n";
        std::cout << "5: Run out of energy\n";
        std::cout << "6: Restore energy\n";
        std::cout << "0: Exit\n";
        std::cout << "========================================\n";
        std::cout << "Enter choice: ";
        
        if (!(std::cin >> choice)) {
            std::cin.clear();
            std::cin.ignore(10000, '\n');
            std::cout << "Invalid input. Please enter a number.\n";
            continue;
        }

        if (choice < 0 || choice > 6) {
            std::cout << "Invalid choice. Please select 0-6.\n";
            continue;
        }

        // Send event to leader
        ssize_t written = write(fd, &choice, sizeof(int));
        if (written != sizeof(int)) {
            std::cerr << "Failed to write to FIFO. Leader may have stopped.\n";
            break;
        }

        // Show confirmation
        switch (choice) {
            case 1:
                std::cout << "✓ Sent: Obstacle detected\n";
                break;
            case 2:
                std::cout << "✓ Sent: Traffic light RED\n";
                break;
            case 3:
                std::cout << "✓ Sent: Traffic light GREEN\n";
                break;
            case 4:
                std::cout << "✓ Sent: Cut-in vehicle alert\n";
                break;
            case 5:
                std::cout << "✓ Sent: Run out of energy\n";
                break;
            case 6:
                std::cout << "✓ Sent: Restore energy\n";
                break;
            case 0:
                std::cout << "✓ Exiting...\n";
                break;
        }

        if (choice == 0) break;
    }

    close(fd);
    std::cout << "\nLeader input program closed.\n";
    return 0;
}
