# Defect Tests Description

This folder contains "Defect Tests" or "Negative Tests". These tests are designed to probe the system's handling of invalid inputs, edge cases, and potentially problematic states.

The goal isn't necessarily for all of them to pass strict logic gates (since validation logic might not exist yet), but to reveal where the system lacks validation or robustness.

## Test Files

### `defect_tests.cpp`

1.  **test_leader_init_negative_id**:
    *   **Scenario**: Initialize a `LeadingVehicle` with `id = -1`.
    *   **Goal**: check if the system validates IDs or blindly accepts negative ones.
2.  **test_leader_init_negative_speed**:
    *   **Scenario**: Initialize a `LeadingVehicle` with `speed = -50.0`.
    *   **Goal**: Check robustness against physical impossibilities (negative speed).
3.  **test_follower_init_negative_pos**:
    *   **Scenario**: Initialize a `FollowingVehicle` with `position = -100.0`.
    *   **Goal**: Check robustness against negative coordinates.
4.  **test_leader_invalid_state_cast**:
    *   **Scenario**: Cast an invalid integer (255) to `LeaderState` and set it.
    *   **Goal**: Determine if the state machine checks for valid enum values.
5.  **test_follower_zero_id**:
    *   **Scenario**: Initialize with `id = 0`.
    *   **Goal**: Check handling of ID 0 (typically reserved for uninitialized or generic).

## How to Run
Execute the runner script:
```bash
./run_tests.sh
```
