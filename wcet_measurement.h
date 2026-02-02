#ifndef WCET_MEASUREMENT_H
#define WCET_MEASUREMENT_H

#include <string>
#include <chrono>
#include <mutex>

class WCETMeasurement {
public:
    WCETMeasurement(const std::string& threadName);
    ~WCETMeasurement();

    void start();
    void stop();
    void printStats();
    
private:
    std::string name_;
    std::chrono::high_resolution_clock::time_point startTime_;
    long long maxDurationNs_;
    long long minDurationNs_;
    long long totalDurationNs_;
    long long count_;
    std::mutex mutex_;
};

#endif // WCET_MEASUREMENT_H
