/**
 * @file follow.h
 * @brief Follower Truck - Platoon member header
 * 
 * Functions:
 *   - TCP client: Send join request to leader
 *   - UDP receiver: Receive state from leader
 *   - UDP sender: Send state to leader
 */

#ifndef FOLLOW_H
#define FOLLOW_H

#include "message.h"

#include <string>
#include <mutex>
#include <atomic>
#include <cstdint>

// ============== CONSTANTS ==============
constexpr int HEARTBEAT_INTERVAL_MS = 500;  // 2 Hz
constexpr double TARGET_GAP_M = 20.0;       // Target gap to leader (m)
constexpr double MAX_SPEED_KMH = 120.0;     // Maximum speed (km/h)

// ============== FUNCTION DECLARATIONS ==============

/**
 * @brief Get current timestamp in milliseconds
 */
uint64_t nowMs();

/**
 * @brief Get formatted timestamp string for logging
 */
std::string getTimestamp();

/**
 * @brief Log a message with timestamp and component tag
 */
void log(const std::string& component, const std::string& msg);

/**
 * @brief Print a separator line in the log
 */
void logSeparator(const std::string& title);

/**
 * @brief Request to join the platoon via TCP
 * @return true if join was successful
 */
bool request_join(const char* leader_ip, const char* vehicle_id);

/**
 * @brief UDP receiver thread - receives leader state broadcasts
 */
void udp_receive_leader_state();

/**
 * @brief UDP sender thread - sends follower state to leader at 2Hz
 */
void udp_send_follower_state();

#endif // FOLLOW_H
