#include "e2e_test_fixture.hpp"
#include <iostream>
#include <random>

namespace vi_slam {
namespace testing {

E2ETestFixture::E2ETestFixture()
    : engine_(nullptr),
      configPath_("/tmp/test_config.yaml"),
      calibPath_("/tmp/test_calib.yaml") {
}

E2ETestFixture::~E2ETestFixture() {
    tearDown();
}

bool E2ETestFixture::setUp() {
    // Create SLAMEngine instance
    engine_ = std::make_unique<SLAMEngine>();

    // Create dummy config files for testing
    std::ofstream configFile(configPath_);
    if (!configFile) {
        std::cerr << "Failed to create test config file" << std::endl;
        return false;
    }
    configFile << "# Test configuration file" << std::endl;
    configFile << "max_features: 200" << std::endl;
    configFile.close();

    std::ofstream calibFile(calibPath_);
    if (!calibFile) {
        std::cerr << "Failed to create test calibration file" << std::endl;
        return false;
    }
    calibFile << "# Test calibration file" << std::endl;
    calibFile << "camera_model: pinhole" << std::endl;
    calibFile.close();

    // Initialize engine
    if (!engine_->selectFramework(SLAMFrameworkType::VINS_MONO)) {
        std::cerr << "Failed to select SLAM framework" << std::endl;
        return false;
    }

    if (!engine_->initialize(configPath_)) {
        std::cerr << "Failed to initialize SLAM engine" << std::endl;
        return false;
    }

    if (!engine_->loadCalibration(calibPath_)) {
        std::cerr << "Failed to load calibration" << std::endl;
        return false;
    }

    // Set up callbacks
    engine_->setPoseCallback([this](const Pose6DoF& pose) {
        this->onPoseReceived(pose);
    });

    engine_->setStatusCallback([this](TrackingStatus status) {
        this->onStatusReceived(status);
    });

    return true;
}

void E2ETestFixture::tearDown() {
    if (engine_) {
        engine_->shutdown();
        engine_.reset();
    }

    // Clear test data
    testSamples_.clear();
    receivedPoses_.clear();
    receivedStatuses_.clear();
    frameLatencies_.clear();
}

bool E2ETestFixture::loadTestData(const std::string& dataPath) {
    std::cout << "Loading test data from: " << dataPath << std::endl;

    // For now, generate synthetic test data
    // In production, this would load from actual dataset files

    const size_t numSamples = 100;
    testSamples_.reserve(numSamples);

    for (size_t i = 0; i < numSamples; ++i) {
        TestSample sample;
        sample.timestampNs = static_cast<int64_t>(i * 100000000LL);  // 10 Hz
        sample.image = generateTestImage(640, 480);

        // Generate 10 IMU samples between image frames (100 Hz IMU)
        for (size_t j = 0; j < 10; ++j) {
            int64_t imuTimestamp = sample.timestampNs + static_cast<int64_t>(j * 10000000LL);
            sample.imuMeasurements.push_back(generateTestIMU(imuTimestamp));
        }

        testSamples_.push_back(std::move(sample));
    }

    std::cout << "Loaded " << testSamples_.size() << " test samples" << std::endl;
    return true;
}

bool E2ETestFixture::loadRecordedSequence(const std::string& sequencePath) {
    // TODO: Implement loading from recorded dataset (EuRoC, TUM-VI, etc.)
    std::cout << "Loading recorded sequence from: " << sequencePath << std::endl;
    return loadTestData(sequencePath);  // Fallback to synthetic data for now
}

E2ETestResult E2ETestFixture::runStreamingTest(size_t numFrames) {
    E2ETestResult result;
    result.success = true;
    result.processedFrames = 0;
    result.droppedFrames = 0;

    if (!engine_) {
        result.success = false;
        result.errorMessage = "Engine not initialized";
        return result;
    }

    if (testSamples_.empty()) {
        result.success = false;
        result.errorMessage = "No test data loaded";
        return result;
    }

    const size_t framesToProcess = std::min(numFrames, testSamples_.size());

    for (size_t i = 0; i < framesToProcess; ++i) {
        const auto& sample = testSamples_[i];

        auto startTime = std::chrono::high_resolution_clock::now();

        // Process IMU measurements
        for (const auto& imu : sample.imuMeasurements) {
            engine_->processIMU(imu);
        }

        // Process image
        engine_->processImage(sample.image, sample.timestampNs);

        auto endTime = std::chrono::high_resolution_clock::now();
        double latencyMs = std::chrono::duration<double, std::milli>(endTime - startTime).count();
        frameLatencies_.push_back(latencyMs);

        result.processedFrames++;
    }

    // Calculate statistics
    if (!frameLatencies_.empty()) {
        double sum = 0.0;
        result.maxLatencyMs = frameLatencies_[0];

        for (double latency : frameLatencies_) {
            sum += latency;
            if (latency > result.maxLatencyMs) {
                result.maxLatencyMs = latency;
            }
        }

        result.averageLatencyMs = sum / frameLatencies_.size();
    }

    return result;
}

E2ETestResult E2ETestFixture::runLatencyTest(size_t numFrames, double targetLatencyMs) {
    E2ETestResult result = runStreamingTest(numFrames);

    if (result.success && result.averageLatencyMs > targetLatencyMs) {
        result.success = false;
        result.errorMessage = "Average latency (" + std::to_string(result.averageLatencyMs) +
                             "ms) exceeds target (" + std::to_string(targetLatencyMs) + "ms)";
    }

    return result;
}

const TestSample& E2ETestFixture::getTestSample(size_t index) const {
    if (index >= testSamples_.size()) {
        throw std::out_of_range("Test sample index out of range");
    }
    return testSamples_[index];
}

void E2ETestFixture::onPoseReceived(const Pose6DoF& pose) {
    receivedPoses_.push_back(pose);
}

void E2ETestFixture::onStatusReceived(TrackingStatus status) {
    receivedStatuses_.push_back(status);
}

cv::Mat E2ETestFixture::generateTestImage(int width, int height) {
    // Generate a synthetic test image with some features
    cv::Mat image(height, width, CV_8UC1);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 255);

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            image.at<uint8_t>(y, x) = static_cast<uint8_t>(dis(gen));
        }
    }

    return image;
}

IMUSample E2ETestFixture::generateTestIMU(int64_t timestampNs) {
    IMUSample imu;
    imu.timestampNs = timestampNs;

    // Simulate stationary device with gravity
    imu.accX = 0.0;
    imu.accY = 0.0;
    imu.accZ = 9.81;  // Gravity in z-axis

    // No rotation
    imu.gyroX = 0.0;
    imu.gyroY = 0.0;
    imu.gyroZ = 0.0;

    return imu;
}

}  // namespace testing
}  // namespace vi_slam
