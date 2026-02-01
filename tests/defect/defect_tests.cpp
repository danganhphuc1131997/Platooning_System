#include "../../lead.h"
#include "../../follow.h"
#include "asserts.h"

// Defect Test 1: Initialize Leader with Negative ID
void test_leader_init_negative_id() {
    LOG_SECTION("Defect 1: Leader Negative ID Initialization");
    LOG_INFO("Attempting to initialize Leader with ID = -1");
    try {
        LeadingVehicle leader(-1, 0.0, 0.0);
        LOG_INFO("[FAIL] System accepted negative ID -1 (Validation Missed)");
    } catch (const std::invalid_argument& e) {
        LOG_PASS("System rejected negative ID (Caught exception: " + std::string(e.what()) + ")");
    } catch (...) {
        LOG_PASS("System rejected negative ID (Unknown exception)");
    }
}

// Defect Test 2: Initialize Leader with Negative Speed
void test_leader_init_negative_speed() {
    LOG_SECTION("Defect 2: Leader Negative Speed Initialization");
    LOG_INFO("Attempting to initialize Leader with Speed = -50.0");
    try {
        LeadingVehicle leader(10, 0.0, -50.0);
        LOG_INFO("[FAIL] System accepted negative speed (Validation Missed)");
    } catch (const std::invalid_argument& e) {
        LOG_PASS("System rejected negative speed (Caught exception: " + std::string(e.what()) + ")");
    }
}

// Defect Test 3: Initialize Follower with Negative Position
void test_follower_init_negative_pos() {
    LOG_SECTION("Defect 3: Follower Negative Position Initialization");
    LOG_INFO("Attempting to initialize Follower with Position = -100.0");
    try {
        FollowingVehicle follower(11, 10.0, -100.0);
        LOG_INFO("[FAIL] System accepted negative position");
    } catch (const std::invalid_argument& e) {
        LOG_PASS("System rejected negative position (Caught exception: " + std::string(e.what()) + ")");
    }
}

// Defect Test 4: Invalid State Cast
// Force an invalid integer into the State enum and see behavior
void test_leader_invalid_state_cast() {
    LOG_SECTION("Defect 4: Leader Invalid State Cast");
    LeadingVehicle leader(12, 0.0, 0.0);
    LOG_INFO("Casting 255 to LeaderState and setting.");
    LeaderState invalidState = static_cast<LeaderState>(255);
    leader.setState(invalidState);
    
    // We expect the state NOT to change to 255 if validated
    if (leader.getState() == invalidState) {
        LOG_INFO("[FAIL] System accepted invalid state 255");
    } else {
        LOG_PASS("System rejected invalid state (State remained: " + std::to_string(static_cast<int>(leader.getState())) + ")");
    }
}

// Defect Test 5: Follower Self-Coupling Scenario (Conceptual)
// We verify if follower allows connecting to itself if we were to mock sockets (here just init check)
void test_follower_zero_id() {
    LOG_SECTION("Defect 5: Follower Zero ID");
    LOG_INFO("Attempting to initialize Follower with ID = 0");
    try {
        FollowingVehicle follower(0, 0.0, 0.0);
        LOG_INFO("[FAIL] System accepted ID 0");
    } catch (const std::invalid_argument& e) {
        LOG_PASS("System rejected ID 0 (Caught exception: " + std::string(e.what()) + ")");
    }
}

int main() {
    std::cout << YELLOW << "\n>>> Running 5 Defect/Edge-Case Tests <<<\n" << RESET << std::endl;
    test_leader_init_negative_id();
    test_leader_init_negative_speed();
    test_follower_init_negative_pos();
    test_leader_invalid_state_cast();
    test_follower_zero_id();
    std::cout << YELLOW << "\n>>> All Defect Tests Completed (Robustness Check) <<<" << RESET << std::endl;
    return 0;
}
