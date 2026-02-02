#include "wcet_measurement.h"
#include <iostream>
#include <limits>

WCETMeasurement::WCETMeasurement(const std::string& threadName)
    : name_(threadName), 
      maxDurationNs_(0), 
      minDurationNs_(std::numeric_limits<long long>::max()),
      totalDurationNs_(0),
      count_(0) {
}

WCETMeasurement::~WCETMeasurement() {
    // printStats(); // Optional: can rely on explicit call or destructor
}

void WCETMeasurement::printStats() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (count_ > 0) {
        double avg = static_cast<double>(totalDurationNs_) / count_;
        std::cout << "[WCET] " << name_ << ": "
                  << "Max=" << maxDurationNs_ / 1000.0 << "us, "
                  << "Min=" << minDurationNs_ / 1000.0 << "us, "
                  << "Avg=" << avg / 1000.0 << "us, "
                  << "Samples=" << count_ << std::endl;
    } else {
        std::cout << "[WCET] " << name_ << ": No samples collected." << std::endl;
    }
}

void WCETMeasurement::start() {
    startTime_ = std::chrono::high_resolution_clock::now();
}

void WCETMeasurement::stop() {
    auto endTime = std::chrono::high_resolution_clock::now();
    // Calculate duration in nanoseconds
    long long duration = std::chrono::duration_cast<std::chrono::nanoseconds>(endTime - startTime_).count();
    
    std::lock_guard<std::mutex> lock(mutex_);
    if (duration > maxDurationNs_) {
        maxDurationNs_ = duration;
    }
    if (duration < minDurationNs_) {
        minDurationNs_ = duration;
    }
    totalDurationNs_ += duration;
    count_++;
}
