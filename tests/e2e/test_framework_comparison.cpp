#include "e2e_test_fixture.hpp"
#include "latency_measurement.hpp"
#include "slam/slam_engine.hpp"
#include <chrono>
#include <iomanip>
#include <iostream>
#include <map>

using namespace vi_slam::testing;
using namespace std::chrono;

struct FrameworkMetrics {
    std::string frameworkName;
    bool initialized;
    size_t processedFrames;
    double averageLatencyMs;
    double maxLatencyMs;
    double accuracyRMSE;
    int64_t switchTimeMs;
};

class MultiFrameworkTestFixture : public E2ETestFixture {
public:
    bool switchFramework(SLAMFramework framework) {
        auto startTime = steady_clock::now();

        // Store current framework state
        lastFramework_ = currentFramework_;
        currentFramework_ = framework;

        // Reinitialize engine with new framework
        engine_.reset();
        engine_ = std::make_unique<SLAMEngine>();

        if (!engine_->initialize(framework, configPath_, calibPath_)) {
            return false;
        }

        auto endTime = steady_clock::now();
        lastSwitchTimeMs_ = duration_cast<milliseconds>(endTime - startTime).count();

        return true;
    }

    int64_t getLastSwitchTimeMs() const { return lastSwitchTimeMs_; }
    SLAMFramework getCurrentFramework() const { return currentFramework_; }

private:
    SLAMFramework currentFramework_ = SLAMFramework::ORBSLAM3;
    SLAMFramework lastFramework_ = SLAMFramework::ORBSLAM3;
    int64_t lastSwitchTimeMs_ = 0;
};

int main() {
    std::cout << "=== Multi-Framework Comparison Test ===" << std::endl;

    // Define frameworks to test
    std::vector<SLAMFramework> frameworks = {
        SLAMFramework::ORBSLAM3,
        SLAMFramework::VINS_MONO,
        SLAMFramework::OPENVINS
        // SLAMFramework::BASALT  // Add when available
    };

    std::map<SLAMFramework, std::string> frameworkNames = {
        {SLAMFramework::ORBSLAM3, "ORB-SLAM3"},
        {SLAMFramework::VINS_MONO, "VINS-Mono"},
        {SLAMFramework::OPENVINS, "OpenVINS"}
        // {SLAMFramework::BASALT, "Basalt"}
    };

    // Create test fixture
    MultiFrameworkTestFixture fixture;

    // Setup
    std::cout << "\n[1/5] Setting up test environment..." << std::endl;
    if (!fixture.setUp()) {
        std::cerr << "FAIL: Test setup failed" << std::endl;
        return 1;
    }
    std::cout << "PASS: Test environment setup complete" << std::endl;

    // Load test data
    std::cout << "\n[2/5] Loading test data..." << std::endl;
    if (!fixture.loadTestData("/tmp/framework_test_data")) {
        std::cerr << "WARN: Could not load test data, using generated data" << std::endl;
    }

    const size_t numTestFrames = std::min(fixture.getNumTestSamples(), size_t(100));
    std::cout << "PASS: Using " << numTestFrames << " test frames" << std::endl;

    // Test each framework
    std::cout << "\n[3/5] Testing each SLAM framework..." << std::endl;
    std::vector<FrameworkMetrics> results;

    for (const auto& framework : frameworks) {
        std::cout << "\n--- Testing " << frameworkNames[framework] << " ---" << std::endl;

        FrameworkMetrics metrics;
        metrics.frameworkName = frameworkNames[framework];
        metrics.initialized = false;
        metrics.processedFrames = 0;
        metrics.averageLatencyMs = 0.0;
        metrics.maxLatencyMs = 0.0;
        metrics.accuracyRMSE = 0.0;
        metrics.switchTimeMs = 0;

        // Initialize framework
        std::cout << "  Initializing " << metrics.frameworkName << "..." << std::endl;
        if (!fixture.switchFramework(framework)) {
            std::cerr << "  WARN: Failed to initialize " << metrics.frameworkName << std::endl;
            results.push_back(metrics);
            continue;
        }

        metrics.initialized = true;
        metrics.switchTimeMs = fixture.getLastSwitchTimeMs();
        std::cout << "  Initialization time: " << metrics.switchTimeMs << " ms" << std::endl;

        // Run test
        std::cout << "  Processing " << numTestFrames << " frames..." << std::endl;
        E2ETestResult result = fixture.runStreamingTest(numTestFrames);

        if (!result.success) {
            std::cerr << "  WARN: Processing failed - " << result.errorMessage << std::endl;
            results.push_back(metrics);
            continue;
        }

        // Store metrics
        metrics.processedFrames = result.processedFrames;
        metrics.averageLatencyMs = result.averageLatencyMs;
        metrics.maxLatencyMs = result.maxLatencyMs;

        std::cout << "  Processed: " << metrics.processedFrames << " frames" << std::endl;
        std::cout << "  Avg latency: " << metrics.averageLatencyMs << " ms" << std::endl;
        std::cout << "  Max latency: " << metrics.maxLatencyMs << " ms" << std::endl;

        results.push_back(metrics);
    }

    // Test hot-switching
    std::cout << "\n[4/5] Testing framework hot-switching..." << std::endl;

    if (frameworks.size() >= 2) {
        std::cout << "  Switching from " << frameworkNames[frameworks[0]]
                  << " to " << frameworkNames[frameworks[1]] << "..." << std::endl;

        if (!fixture.switchFramework(frameworks[1])) {
            std::cerr << "  FAIL: Hot-switch failed" << std::endl;
        } else {
            int64_t switchTime = fixture.getLastSwitchTimeMs();
            std::cout << "  Switch time: " << switchTime << " ms" << std::endl;

            const int64_t maxSwitchTimeMs = 5000;  // Target: < 5 seconds
            if (switchTime > maxSwitchTimeMs) {
                std::cerr << "  WARN: Switch time (" << switchTime
                          << " ms) exceeds target (" << maxSwitchTimeMs << " ms)" << std::endl;
            } else {
                std::cout << "  PASS: Switch time within target" << std::endl;
            }
        }
    } else {
        std::cout << "  SKIP: Need at least 2 frameworks for hot-switch test" << std::endl;
    }

    // Generate comparison report
    std::cout << "\n[5/5] Generating framework comparison report..." << std::endl;

    std::cout << "\n=== Framework Comparison Summary ===" << std::endl;
    std::cout << "\n" << std::left << std::setw(15) << "Framework"
              << std::setw(12) << "Status"
              << std::setw(12) << "Init (ms)"
              << std::setw(12) << "Frames"
              << std::setw(15) << "Avg Lat (ms)"
              << std::setw(15) << "Max Lat (ms)" << std::endl;
    std::cout << std::string(80, '-') << std::endl;

    for (const auto& metrics : results) {
        std::cout << std::left << std::setw(15) << metrics.frameworkName
                  << std::setw(12) << (metrics.initialized ? "OK" : "FAILED")
                  << std::setw(12) << metrics.switchTimeMs
                  << std::setw(12) << metrics.processedFrames
                  << std::setw(15) << std::fixed << std::setprecision(2) << metrics.averageLatencyMs
                  << std::setw(15) << std::fixed << std::setprecision(2) << metrics.maxLatencyMs
                  << std::endl;
    }

    // Verify at least one framework works
    bool anyFrameworkWorked = false;
    for (const auto& metrics : results) {
        if (metrics.initialized && metrics.processedFrames > 0) {
            anyFrameworkWorked = true;
            break;
        }
    }

    // Cleanup
    fixture.tearDown();

    if (!anyFrameworkWorked) {
        std::cerr << "\nFAIL: No framework successfully processed frames" << std::endl;
        return 1;
    }

    std::cout << "\n=== Test Summary ===" << std::endl;
    std::cout << "Framework comparison test completed!" << std::endl;
    std::cout << "Frameworks tested: " << results.size() << std::endl;
    std::cout << "Frameworks working: "
              << std::count_if(results.begin(), results.end(),
                               [](const auto& m) { return m.initialized && m.processedFrames > 0; })
              << std::endl;

    return 0;
}
