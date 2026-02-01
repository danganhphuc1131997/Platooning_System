# Validation Tests Description

This folder contains unit tests meant to validate the basic functionality of the Platooning System vehicles.

## Test Files

### 1. `leader_tests.cpp`
*   **test_initialization**: Validates that a `LeadingVehicle` object is correctly initialized with the provided ID, position, and speed.
*   **test_state_transition**: Checks if the leader can transition from `NORMAL` to `STOPPING` state correctly.
*   **test_low_energy**: Verifies the logic for setting the vehicle into `LOW_ENERGY` mode.

### 2. `follower_tests.cpp`
*   **test_initialization**: Validates that a `FollowingVehicle` object is correctly initialized.
*   **test_state_change**: Checks state transition logic for the follower.

## How to Run
Execute the runner script:
```bash
./run_tests.sh
```
