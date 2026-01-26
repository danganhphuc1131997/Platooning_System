/**
 * @file follower_input.cpp
 * @brief Standalone program for sending events to a follower vehicle via FIFO
 * 
 * This program can be run in a separate terminal/tmux pane to send events
 * to a specific follower vehicle without relying on automatic terminal launching.
 * 
 * Usage: ./follower_input <follower_id>
 * Example: ./follower_input 2
 */

#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <cstring>
#include <string>

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <follower_id>\n";
        std::cerr << "Example: " << argv[0] << " 2\n";
        return 1;
    }

    int followerId;
    try {
        followerId = std::stoi(argv[1]);
    } catch (...) {
        std::cerr << "Error: Invalid follower ID. Must be a number.\n";
        return 1;
    }

    std::string fifoPath = "/tmp/follower_" + std::to_string(followerId) + "_event_fifo";
    
    std::cout << "=== Follower " << followerId << " Event Input Program ===\n";
    std::cout << "Waiting for follower to create FIFO...\n";

    // Wait for FIFO to exist (created by follower)
    int attempts = 0;
    while (access(fifoPath.c_str(), F_OK) != 0 && attempts < 30) {
        sleep(1);
        attempts++;
    }

    if (attempts >= 30) {
        std::cerr << "Error: FIFO not found at " << fifoPath << "\n";
        std::cerr << "Make sure follower " << followerId << " is running first.\n";
        return 1;
    }

    std::cout << "Connected to follower " << followerId << "!\n\n";

    // Open FIFO for writing
    int fd = open(fifoPath.c_str(), O_WRONLY);
    if (fd == -1) {
        std::cerr << "Failed to open FIFO: " << strerror(errno) << std::endl;
        return 1;
    }

    int choice;
    while (true) {
        // Display menu
        std::cout << "\n========================================\n";
        std::cout << "[FOLLOWER " << followerId << " EVENT INPUT]\n";
        std::cout << "========================================\n";
        std::cout << "1: Traffic light RED (follower stops)\n";
        std::cout << "2: Traffic light GREEN (follower resumes)\n";
        std::cout << "3: Energy low (follower reduces speed)\n";
        std::cout << "0: Exit\n";
        std::cout << "========================================\n";
        std::cout << "Enter choice: ";
        
        if (!(std::cin >> choice)) {
            std::cin.clear();
            std::cin.ignore(10000, '\n');
            std::cout << "Invalid input. Please enter a number.\n";
            continue;
        }

        if (choice < 0 || choice > 3) {
            std::cout << "Invalid choice. Please select 0-3.\n";
            continue;
        }

        // Send event to follower
        ssize_t written = write(fd, &choice, sizeof(int));
        if (written != sizeof(int)) {
            std::cerr << "Failed to write to FIFO. Follower may have stopped.\n";
            break;
        }

        // Show confirmation
        switch (choice) {
            case 1:
                std::cout << "✓ Sent: Traffic light RED\n";
                break;
            case 2:
                std::cout << "✓ Sent: Traffic light GREEN\n";
                break;
            case 3:
                std::cout << "✓ Sent: Energy low\n";
                break;
            case 0:
                std::cout << "✓ Exiting...\n";
                break;
        }

        if (choice == 0) break;
    }

    close(fd);
    std::cout << "\nFollower " << followerId << " input program closed.\n";
    return 0;
}
