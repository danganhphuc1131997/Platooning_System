#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

// System configuration parameters
const int MAX_VEHICLES = 10;          // Maximum number of vehicles in the platoon
const double SAFE_DISTANCE = 20.0;    // Safe following distance in meters
static const int SERVER_PORT = 5000;  // UDP server port for leader

// Leader initial parameters
const int LEADER_INITIAL_ID = 1;
const double LEADER_INITIAL_POSITION = 0.0; // in meters
const double LEADER_INITIAL_SPEED = 60.0;   // in m/s

// Follower initial parameters
const int FOLLOWER_INITIAL_ID = 2;
const double FOLLOWER_INITIAL_SPEED = 60.0; // in m/s
const double FOLLOWER_INITIAL_POSITION = -20.0; // in meters

// Traffic light statuses
enum TrafficLightStatus {
    LIGHT_GREEN = 0,
    LIGHT_RED = 1
};

// event simulation parameters
enum EventType {
    NO_EVENT = 0,
    OBSTACLE_DETECTED,
    TRAFFIC_LIGHT_RED,
    TRAFFIC_LIGHT_GREEN
};

#endif // SYSTEM_CONFIG_H