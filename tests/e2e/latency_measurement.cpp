#include "latency_measurement.hpp"
#include <algorithm>
#include <fstream>
#include <iostream>
#include <numeric>

namespace vi_slam {
namespace testing {

LatencyMeasurement::LatencyMeasurement() : timing_(false) {
}

void LatencyMeasurement::start() {
    startTime_ = std::chrono::high_resolution_clock::now();
    timing_ = true;
}

void LatencyMeasurement::stop() {
    if (!timing_) {
        std::cerr << "Warning: stop() called without start()" << std::endl;
        return;
    }

    auto endTime = std::chrono::high_resolution_clock::now();
    double latencyMs = std::chrono::duration<double, std::milli>(endTime - startTime_).count();
    latencies_.push_back(latencyMs);
    timing_ = false;
}

void LatencyMeasurement::reset() {
    latencies_.clear();
    timing_ = false;
}

LatencyStats LatencyMeasurement::getStats() const {
    LatencyStats stats;

    if (latencies_.empty()) {
        stats.minMs = 0.0;
        stats.maxMs = 0.0;
        stats.averageMs = 0.0;
        stats.medianMs = 0.0;
        stats.p95Ms = 0.0;
        stats.p99Ms = 0.0;
        stats.sampleCount = 0;
        return stats;
    }

    std::vector<double> sorted = latencies_;
    std::sort(sorted.begin(), sorted.end());

    stats.minMs = sorted.front();
    stats.maxMs = sorted.back();
    stats.averageMs = std::accumulate(sorted.begin(), sorted.end(), 0.0) / sorted.size();
    stats.medianMs = calculatePercentile(0.50);
    stats.p95Ms = calculatePercentile(0.95);
    stats.p99Ms = calculatePercentile(0.99);
    stats.sampleCount = sorted.size();

    return stats;
}

bool LatencyMeasurement::meetsTarget(double targetMs, double percentile) const {
    if (latencies_.empty()) {
        return false;
    }

    double actualLatency = calculatePercentile(percentile);
    return actualLatency <= targetMs;
}

bool LatencyMeasurement::exportToCSV(const std::string& filename) const {
    std::ofstream file(filename);
    if (!file) {
        std::cerr << "Failed to open file for export: " << filename << std::endl;
        return false;
    }

    file << "Sample,Latency_ms" << std::endl;

    for (size_t i = 0; i < latencies_.size(); ++i) {
        file << i << "," << latencies_[i] << std::endl;
    }

    file.close();
    std::cout << "Latency data exported to: " << filename << std::endl;
    return true;
}

double LatencyMeasurement::calculatePercentile(double percentile) const {
    if (latencies_.empty()) {
        return 0.0;
    }

    std::vector<double> sorted = latencies_;
    std::sort(sorted.begin(), sorted.end());

    double index = percentile * (sorted.size() - 1);
    size_t lowerIndex = static_cast<size_t>(index);
    size_t upperIndex = lowerIndex + 1;

    if (upperIndex >= sorted.size()) {
        return sorted.back();
    }

    double fraction = index - lowerIndex;
    return sorted[lowerIndex] * (1.0 - fraction) + sorted[upperIndex] * fraction;
}

}  // namespace testing
}  // namespace vi_slam
