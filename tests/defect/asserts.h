#pragma once
#include <iostream>
#include <cstdlib>
#include <string>

// ANSI Code Colors
#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define BLUE    "\033[34m"
#define CYAN    "\033[36m"
#define YELLOW  "\033[33m"

#define ASSERT_EQ(expected, actual) \
    if ((expected) != (actual)) { \
        std::cerr << RED << "[FAIL] " << RESET << __FILE__ << ":" << __LINE__ << ": Expected " << (expected) << ", got " << (actual) << std::endl; \
        std::exit(1); \
    }

#define ASSERT_TRUE(condition) \
    if (!(condition)) { \
        std::cerr << RED << "[FAIL] " << RESET << __FILE__ << ":" << __LINE__ << ": Expected true" << std::endl; \
        std::exit(1); \
    }

inline void LOG_SECTION(const std::string& name) {
    std::cout << "\n" << CYAN << "========================================" << RESET << std::endl;
    std::cout << CYAN << " TEST: " << name << RESET << std::endl;
    std::cout << CYAN << "========================================" << RESET << std::endl;
}

inline void LOG_PASS(const std::string& name) {
    std::cout << GREEN << "[PASS] " << RESET << name << std::endl;
}

inline void LOG_INFO(const std::string& msg) {
    std::cout << BLUE << "[INFO] " << RESET << msg << std::endl;
}
