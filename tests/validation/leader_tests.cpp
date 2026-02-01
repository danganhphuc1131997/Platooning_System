#include "../../lead.h"
#include "asserts.h"

void test_initialization() {
    LOG_SECTION("Leader Initialization");
    LOG_INFO("Initializing LeadingVehicle(id=1, pos=100.0, speed=20.0)");
    LeadingVehicle leader(1, 100.0, 20.0);
    
    ASSERT_EQ(1, leader.getId());
    // Assuming initial state is LeaderState::NORMAL
    ASSERT_EQ(LeaderState::NORMAL, leader.getState());
    LOG_PASS("Leader initialized correctly");
}

void test_state_transition() {
    LOG_SECTION("Leader State Transition");
    LeadingVehicle leader(1, 100.0, 20.0);
    
    LOG_INFO("Changing state to LeaderState::STOPPING");
    leader.setState(LeaderState::STOPPING);
    
    ASSERT_EQ(LeaderState::STOPPING, leader.getState());
    LOG_PASS("State transitioned to LeaderState::STOPPING");
}

void test_low_energy() {
    LOG_SECTION("Leader Low Energy");
    LeadingVehicle leader(1, 100.0, 20.0);
    
    LOG_INFO("Setting state to LeaderState::LOW_ENERGY");
    leader.setState(LeaderState::LOW_ENERGY);
    
    ASSERT_EQ(LeaderState::LOW_ENERGY, leader.getState());
    LOG_PASS("State set to LeaderState::LOW_ENERGY");
}

int main() {
    std::cout << YELLOW << "\n>>> Running 3 Leader Tests <<<\n" << RESET << std::endl;
    test_initialization();
    test_state_transition();
    test_low_energy();
    std::cout << YELLOW << "\n>>> All Leader Tests Completed Successfully <<<" << RESET << std::endl;
    return 0;
}
