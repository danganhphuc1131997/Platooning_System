# Platooning System - Test Specification & Results

This document describes the testing strategy, test cases, and traceability to system requirements.

## 1. Validation Tests (Unit Tests)
**Location**: `tests/validation/`
**Objective**: Verify that individual functions and classes behave correctly under normal, expected conditions ("Happy Path").

| Test ID | Name | Description | Traceability / Requirement |
| :--- | :--- | :--- | :--- |
| **VAL-01** | `test_leader_initialization` | Verifies `LeadingVehicle` initializes with correct ID, Position, and Speed. | **Req-1.1**: System shall support initialization of a Leading Vehicle. |
| **VAL-02** | `test_leader_set_state` | Verifies `setState()` correctly updates the leader's internal mode. | **Req-1.2**: Leader vehicle shall maintain an internal state (Normal, Emergency, etc). |
| **VAL-03** | `test_leader_heartbeat` | Verifies logical clock/heartbeat mechanisms update correctly. | **Req-1.3**: Leader shall track time/heartbeats for safety. |
| **VAL-04** | `test_follower_initialization`| Verifies `FollowingVehicle` initializes with correct ID and Position. | **Req-2.1**: System shall support initialization of Following Vehicles. |
| **VAL-05** | `test_follower_compute_dist` | Verifies distance calculation logic between two coordinates. | **Req-2.2**: Followers shall calculate distance to the vehicle ahead. |

---

## 2. Defect Tests (Robustness/Edge Cases)
**Location**: `tests/defect/`
**Objective**: Verify that the system safely handles invalid inputs, boundary conditions, and ensures no undefined behavior ("Negative Path").

| Test ID | Name | Case/Input | Expected Behavior |
| :--- | :--- | :--- | :--- |
| **DEF-01** | `test_leader_init_negative_id` | `ID = -1` | **Reject**: Throw `std::invalid_argument`. |
| **DEF-02** | `test_leader_init_negative_speed` | `Speed = -50.0` | **Reject**: Throw `std::invalid_argument`. |
| **DEF-03** | `test_follower_init_negative_pos` | `Position = -100.0` | **Reject**: Throw `std::invalid_argument`. |
| **DEF-04** | `test_leader_invalid_state_cast` | Cast `255` to Enum | **Safety**: Method ignores invalid value or logs warning; State remains unchanged. |
| **DEF-05** | `test_follower_zero_id` | `ID = 0` | **Reject**: Throw exceptions (ID 0 is usually reserved/invalid). |

---

## 3. Component / Interface Tests
**Location**: `tests/component/`
**Objective**: Verify the interaction between different components (Leader vs. Follower) and the integrity of the data interfaces (Network Protocol).

| Test ID | Name | Component | description | Traceability |
| :--- | :--- | :--- | :--- | :--- |
| **COM-01** | **Message Serialization** | `message.h` (Network Interface) | Verifies that `StatusUpdateMessage` and `PlatoonStateMessage` structs are correctly packed/unpacked into raw bytes for UDP transmission. | **Arch-2.0**: Data Structures. Ensures binary compatibility between sender/receiver. |
| **COM-02** | **Leader State Logic** | `lead.cpp` (Leader Component) | Verifies the internal state machine that drives the behavior of the Leader component before it broadcasts commands. | **Arch-3.0**: Leader Logic. Ensures Leader transitions to STOPPING/NORMAL modes correctly. |
| **COM-03** | **Follower Reaction** | `follow.cpp` (Follower Component) | Simulates the Follower's data generation process to verify it produces valid packet structures (ID, Payload Size) for the interface. | **Arch-1.0**: System Components. Verifies Follower adheres to the communication contract. |

## 4. How to Run Tests

### Run All Validation Tests
```bash
cd tests/validation
./run_tests.sh
```

### Run All Defect Tests
```bash
cd tests/defect
./run_tests.sh
```

### Run All Component Tests
```bash
cd tests/component
./run_tests.sh
```
