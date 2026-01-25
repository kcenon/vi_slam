#ifndef VI_SLAM_LATENCY_MEASUREMENT_HPP
#define VI_SLAM_LATENCY_MEASUREMENT_HPP

#include <chrono>
#include <string>
#include <vector>

namespace vi_slam {
namespace testing {

// Latency statistics
struct LatencyStats {
    double minMs;
    double maxMs;
    double averageMs;
    double medianMs;
    double p95Ms;  // 95th percentile
    double p99Ms;  // 99th percentile
    size_t sampleCount;
};

// Latency measurement utility class
class LatencyMeasurement {
public:
    LatencyMeasurement();

    // Start timing a new measurement
    void start();

    // Stop timing and record the latency
    void stop();

    // Reset all measurements
    void reset();

    // Get statistics
    LatencyStats getStats() const;

    // Get all raw latency values
    const std::vector<double>& getLatencies() const { return latencies_; }

    // Check if target latency is met
    bool meetsTarget(double targetMs, double percentile = 0.95) const;

    // Export to CSV
    bool exportToCSV(const std::string& filename) const;

private:
    std::chrono::high_resolution_clock::time_point startTime_;
    std::vector<double> latencies_;  // Stored in milliseconds
    bool timing_;

    // Calculate percentile
    double calculatePercentile(double percentile) const;
};

}  // namespace testing
}  // namespace vi_slam

#endif  // VI_SLAM_LATENCY_MEASUREMENT_HPP
