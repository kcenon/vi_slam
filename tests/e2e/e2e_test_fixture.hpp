#ifndef VI_SLAM_E2E_TEST_FIXTURE_HPP
#define VI_SLAM_E2E_TEST_FIXTURE_HPP

#include "common/types.hpp"
#include "slam/slam_engine.hpp"
#include <chrono>
#include <fstream>
#include <memory>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace vi_slam {
namespace testing {

// Test data sample containing image and IMU measurements
struct TestSample {
    int64_t timestampNs;
    cv::Mat image;
    std::vector<IMUSample> imuMeasurements;
};

// E2E test results
struct E2ETestResult {
    bool success;
    std::string errorMessage;
    double averageLatencyMs;
    double maxLatencyMs;
    size_t processedFrames;
    size_t droppedFrames;
};

// Base class for E2E test fixtures
class E2ETestFixture {
public:
    E2ETestFixture();
    virtual ~E2ETestFixture();

    // Setup and teardown
    virtual bool setUp();
    virtual void tearDown();

    // Test data loading
    bool loadTestData(const std::string& dataPath);
    bool loadRecordedSequence(const std::string& sequencePath);

    // Test execution
    E2ETestResult runStreamingTest(size_t numFrames);
    E2ETestResult runLatencyTest(size_t numFrames, double targetLatencyMs);

    // Data access
    size_t getNumTestSamples() const { return testSamples_.size(); }
    const TestSample& getTestSample(size_t index) const;

protected:
    // Test data
    std::vector<TestSample> testSamples_;
    std::unique_ptr<SLAMEngine> engine_;

    // Test configuration
    std::string configPath_;
    std::string calibPath_;

    // Results tracking
    std::vector<Pose6DoF> receivedPoses_;
    std::vector<TrackingStatus> receivedStatuses_;
    std::vector<double> frameLatencies_;

    // Helper methods
    void onPoseReceived(const Pose6DoF& pose);
    void onStatusReceived(TrackingStatus status);
    cv::Mat generateTestImage(int width, int height);
    IMUSample generateTestIMU(int64_t timestampNs);
};

}  // namespace testing
}  // namespace vi_slam

#endif  // VI_SLAM_E2E_TEST_FIXTURE_HPP
