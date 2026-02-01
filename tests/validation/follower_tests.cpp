#include "../../follow.h"
#include "asserts.h"

void test_initialization() {
    LOG_SECTION("Follower Initialization");
    LOG_INFO("Initializing FollowingVehicle(id=2, pos=20.0, speed=90.0)");
    FollowingVehicle follower(2, 20.0, 90.0);
    
    ASSERT_EQ(2, follower.getId());
    ASSERT_EQ(FollowerState::NORMAL, follower.getState());
    LOG_PASS("Follower initialized correctly");
}

void test_state_change() {
    LOG_SECTION("Follower State Change");
    FollowingVehicle follower(2, 20.0, 90.0);
    
    LOG_INFO("Changing state to FollowerState::STARTING");
    follower.setState(FollowerState::STARTING);
    
    ASSERT_EQ(FollowerState::STARTING, follower.getState());
    LOG_PASS("State changed to FollowerState::STARTING");
}

int main() {
    std::cout << YELLOW << "\n>>> Running 2 Follower Tests <<<\n" << RESET << std::endl;
    test_initialization();
    test_state_change();
    std::cout << YELLOW << "\n>>> All Follower Tests Completed Successfully <<<" << RESET << std::endl;
    return 0;
}
