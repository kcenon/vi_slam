#include "e2e_test_fixture.hpp"
#include "latency_measurement.hpp"
#include <iostream>

using namespace vi_slam::testing;

int main() {
    std::cout << "=== E2E Basic Streaming Test ===" << std::endl;

    // Create test fixture
    E2ETestFixture fixture;

    // Setup
    std::cout << "\n[1/5] Setting up test environment..." << std::endl;
    if (!fixture.setUp()) {
        std::cerr << "FAIL: Test setup failed" << std::endl;
        return 1;
    }
    std::cout << "PASS: Test environment setup complete" << std::endl;

    // Load test data
    std::cout << "\n[2/5] Loading test data..." << std::endl;
    if (!fixture.loadTestData("/tmp/test_data")) {
        std::cerr << "FAIL: Failed to load test data" << std::endl;
        fixture.tearDown();
        return 1;
    }
    std::cout << "PASS: Loaded " << fixture.getNumTestSamples() << " test samples" << std::endl;

    // Run streaming test
    std::cout << "\n[3/5] Running streaming test..." << std::endl;
    const size_t numFrames = 50;  // Test with 50 frames
    E2ETestResult result = fixture.runStreamingTest(numFrames);

    if (!result.success) {
        std::cerr << "FAIL: Streaming test failed - " << result.errorMessage << std::endl;
        fixture.tearDown();
        return 1;
    }

    std::cout << "PASS: Streaming test completed" << std::endl;
    std::cout << "  Processed frames: " << result.processedFrames << std::endl;
    std::cout << "  Dropped frames: " << result.droppedFrames << std::endl;
    std::cout << "  Average latency: " << result.averageLatencyMs << " ms" << std::endl;
    std::cout << "  Max latency: " << result.maxLatencyMs << " ms" << std::endl;

    // Verify latency requirement
    std::cout << "\n[4/5] Verifying latency requirements..." << std::endl;
    const double targetLatency = 100.0;  // Target: < 100ms

    if (result.averageLatencyMs > targetLatency) {
        std::cerr << "FAIL: Average latency (" << result.averageLatencyMs
                  << " ms) exceeds target (" << targetLatency << " ms)" << std::endl;
        fixture.tearDown();
        return 1;
    }

    std::cout << "PASS: Latency requirement met" << std::endl;

    // Verify frame processing
    std::cout << "\n[5/5] Verifying frame processing..." << std::endl;

    double dropRate = static_cast<double>(result.droppedFrames) / result.processedFrames;
    const double maxDropRate = 0.01;  // Maximum 1% drop rate

    if (dropRate > maxDropRate) {
        std::cerr << "FAIL: Frame drop rate (" << (dropRate * 100)
                  << "%) exceeds maximum (" << (maxDropRate * 100) << "%)" << std::endl;
        fixture.tearDown();
        return 1;
    }

    std::cout << "PASS: Frame processing verified (drop rate: "
              << (dropRate * 100) << "%)" << std::endl;

    // Cleanup
    fixture.tearDown();

    // Summary
    std::cout << "\n=== Test Summary ===" << std::endl;
    std::cout << "All E2E streaming tests passed!" << std::endl;
    std::cout << "\nPerformance Metrics:" << std::endl;
    std::cout << "  Total frames processed: " << result.processedFrames << std::endl;
    std::cout << "  Average latency: " << result.averageLatencyMs << " ms" << std::endl;
    std::cout << "  Max latency: " << result.maxLatencyMs << " ms" << std::endl;
    std::cout << "  Drop rate: " << (dropRate * 100) << "%" << std::endl;

    return 0;
}
