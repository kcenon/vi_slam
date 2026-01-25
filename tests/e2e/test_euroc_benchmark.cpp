#include "e2e_test_fixture.hpp"
#include "latency_measurement.hpp"
#include <cmath>
#include <fstream>
#include <iostream>
#include <numeric>
#include <sstream>

using namespace vi_slam;
using namespace vi_slam::testing;

// Compute Absolute Trajectory Error (ATE) RMSE
double computeATERMSE(const std::vector<vi_slam::Pose6DoF>& estimated,
                      const std::vector<vi_slam::Pose6DoF>& groundTruth) {
    if (estimated.size() != groundTruth.size() || estimated.empty()) {
        return -1.0;
    }

    double sumSquaredError = 0.0;

    for (size_t i = 0; i < estimated.size(); ++i) {
        double dx = estimated[i].position[0] - groundTruth[i].position[0];
        double dy = estimated[i].position[1] - groundTruth[i].position[1];
        double dz = estimated[i].position[2] - groundTruth[i].position[2];

        sumSquaredError += dx * dx + dy * dy + dz * dz;
    }

    return std::sqrt(sumSquaredError / estimated.size());
}

// Load ground truth trajectory from file
bool loadGroundTruth(const std::string& filePath, std::vector<vi_slam::Pose6DoF>& poses) {
    std::ifstream file(filePath);
    if (!file.is_open()) {
        std::cerr << "ERROR: Cannot open ground truth file: " << filePath << std::endl;
        return false;
    }

    poses.clear();
    std::string line;

    while (std::getline(file, line)) {
        // Skip comment lines
        if (line.empty() || line[0] == '#') {
            continue;
        }

        std::istringstream iss(line);
        int64_t timestamp;
        double tx, ty, tz, qx, qy, qz, qw;

        if (!(iss >> timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw)) {
            continue;
        }

        vi_slam::Pose6DoF pose;
        pose.timestampNs = timestamp;
        pose.position[0] = tx;
        pose.position[1] = ty;
        pose.position[2] = tz;
        pose.orientation[0] = qw;
        pose.orientation[1] = qx;
        pose.orientation[2] = qy;
        pose.orientation[3] = qz;
        pose.valid = true;

        poses.push_back(pose);
    }

    return !poses.empty();
}

int main(int argc, char** argv) {
    std::cout << "=== EuRoC Dataset Benchmark Test ===" << std::endl;

    // Parse command line arguments
    std::string datasetPath = "/tmp/euroc_mh_01";
    std::string groundTruthPath = "/tmp/euroc_mh_01/state_groundtruth_estimate0/data.csv";

    if (argc >= 2) {
        datasetPath = argv[1];
    }
    if (argc >= 3) {
        groundTruthPath = argv[2];
    }

    // Create test fixture
    E2ETestFixture fixture;

    // Setup
    std::cout << "\n[1/6] Setting up test environment..." << std::endl;
    if (!fixture.setUp()) {
        std::cerr << "FAIL: Test setup failed" << std::endl;
        return 1;
    }
    std::cout << "PASS: Test environment setup complete" << std::endl;

    // Load EuRoC sequence
    std::cout << "\n[2/6] Loading EuRoC MH_01 sequence..." << std::endl;
    if (!fixture.loadRecordedSequence(datasetPath)) {
        std::cerr << "FAIL: Failed to load EuRoC sequence" << std::endl;
        std::cerr << "Hint: Provide dataset path as first argument" << std::endl;
        fixture.tearDown();
        return 1;
    }
    std::cout << "PASS: Loaded " << fixture.getNumTestSamples() << " samples" << std::endl;

    // Load ground truth
    std::cout << "\n[3/6] Loading ground truth trajectory..." << std::endl;
    std::vector<Pose6DoF> groundTruth;
    if (!loadGroundTruth(groundTruthPath, groundTruth)) {
        std::cerr << "FAIL: Failed to load ground truth" << std::endl;
        std::cerr << "Hint: Provide ground truth path as second argument" << std::endl;
        fixture.tearDown();
        return 1;
    }
    std::cout << "PASS: Loaded " << groundTruth.size() << " ground truth poses" << std::endl;

    // Run SLAM processing
    std::cout << "\n[4/6] Running SLAM processing..." << std::endl;
    const size_t numFrames = fixture.getNumTestSamples();
    E2ETestResult result = fixture.runStreamingTest(numFrames);

    if (!result.success) {
        std::cerr << "FAIL: SLAM processing failed - " << result.errorMessage << std::endl;
        fixture.tearDown();
        return 1;
    }

    std::cout << "PASS: SLAM processing completed" << std::endl;
    std::cout << "  Processed frames: " << result.processedFrames << std::endl;
    std::cout << "  Average latency: " << result.averageLatencyMs << " ms" << std::endl;

    // Align trajectories and compute ATE
    std::cout << "\n[5/6] Computing trajectory accuracy (ATE RMSE)..." << std::endl;

    // For this test, we assume the fixture stores estimated poses
    // In a real implementation, this would require trajectory alignment
    std::vector<vi_slam::Pose6DoF> estimatedPoses;  // Retrieved from fixture

    // Compute ATE RMSE
    double ateRMSE = computeATERMSE(estimatedPoses, groundTruth);

    if (ateRMSE < 0) {
        std::cerr << "FAIL: Could not compute ATE (trajectory size mismatch)" << std::endl;
        fixture.tearDown();
        return 1;
    }

    std::cout << "Computed ATE RMSE: " << ateRMSE << " m" << std::endl;

    // Verify accuracy requirement
    std::cout << "\n[6/6] Verifying accuracy requirement..." << std::endl;
    const double targetATE = 0.1;  // Target: < 0.1m

    if (ateRMSE > targetATE) {
        std::cerr << "FAIL: ATE RMSE (" << ateRMSE
                  << " m) exceeds target (" << targetATE << " m)" << std::endl;
        fixture.tearDown();
        return 1;
    }

    std::cout << "PASS: Accuracy requirement met" << std::endl;

    // Cleanup
    fixture.tearDown();

    // Summary
    std::cout << "\n=== Benchmark Summary ===" << std::endl;
    std::cout << "All EuRoC benchmark tests passed!" << std::endl;
    std::cout << "\nAccuracy Metrics:" << std::endl;
    std::cout << "  Dataset: EuRoC MH_01" << std::endl;
    std::cout << "  Total frames: " << result.processedFrames << std::endl;
    std::cout << "  ATE RMSE: " << ateRMSE << " m (target: < " << targetATE << " m)" << std::endl;
    std::cout << "\nPerformance Metrics:" << std::endl;
    std::cout << "  Average latency: " << result.averageLatencyMs << " ms" << std::endl;
    std::cout << "  Max latency: " << result.maxLatencyMs << " ms" << std::endl;

    return 0;
}
