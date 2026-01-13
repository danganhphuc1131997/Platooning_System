/**
 * @file lead.h
 * @brief Leader Truck - Platoon controller header
 * 
 * Functions:
 *   - TCP server: Accept join requests from followers
 *   - UDP broadcast: Send control state to followers
 *   - UDP receiver: Receive state from followers
 */

#ifndef LEAD_H
#define LEAD_H

#include "message.h"

#include <string>
#include <vector>
#include <mutex>
#include <cstdint>

// ============== CONSTANTS ==============
constexpr int BROADCAST_INTERVAL_MS = 100;  // 10 Hz
constexpr int LOG_INTERVAL = 10;            // Log every N broadcasts

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
 * @brief TCP management server thread - handles JOIN requests
 */
void tcp_management_server();

/**
 * @brief UDP broadcast server thread - broadcasts leader state at 10Hz
 */
void udp_broadcast_server();

/**
 * @brief UDP receiver thread - receives state updates from followers
 */
void udp_receive_follower_state();

#endif // LEAD_H
