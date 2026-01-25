#include "slam/adapters/openvins_adapter.hpp"
#include <iostream>
#include <fstream>
#include <opencv2/core.hpp>

using namespace vi_slam;

int main() {
    std::cout << "Testing OpenVINSAdapter..." << std::endl;

    // Create adapter instance
    OpenVINSAdapter adapter;

    // Test 1: Check initial status
    TrackingStatus status = adapter.getStatus();
    if (status != TrackingStatus::UNINITIALIZED) {
        std::cerr << "FAIL: Initial status should be UNINITIALIZED" << std::endl;
        return 1;
    }
    std::cout << "PASS: Initial status is UNINITIALIZED" << std::endl;

    // Test 2: Initialize with dummy config
    const std::string configPath = "/tmp/dummy_openvins_config.yaml";
    const std::string calibPath = "/tmp/dummy_openvins_calib.yaml";

    // Create dummy files for testing
    {
        std::ofstream configFile(configPath);
        configFile << "# Dummy OpenVINS config file for testing" << std::endl;
        configFile << "feature_tracker: KLT" << std::endl;
        configFile << "num_features: 200" << std::endl;
    }
    {
        std::ofstream calibFile(calibPath);
        calibFile << "# Dummy OpenVINS calibration file for testing" << std::endl;
        calibFile << "cam0:" << std::endl;
        calibFile << "  intrinsics: [458.654, 457.296, 367.215, 248.375]" << std::endl;
    }

    bool initSuccess = adapter.initialize(configPath);
    if (!initSuccess) {
        std::cerr << "FAIL: Initialization failed" << std::endl;
        return 1;
    }
    std::cout << "PASS: Initialization successful" << std::endl;

    // Test 3: Load calibration
    bool calibSuccess = adapter.loadCalibration(calibPath);
    if (!calibSuccess) {
        std::cerr << "FAIL: Calibration loading failed" << std::endl;
        return 1;
    }
    std::cout << "PASS: Calibration loaded successfully" << std::endl;

    // Test 4: Process IMU sample
    IMUSample imu;
    imu.timestampNs = 1000000000LL;  // 1 second
    imu.accX = 0.0;
    imu.accY = 0.0;
    imu.accZ = 9.81;  // Gravity
    imu.gyroX = 0.0;
    imu.gyroY = 0.0;
    imu.gyroZ = 0.0;

    adapter.processIMU(imu);
    std::cout << "PASS: IMU sample processed" << std::endl;

    // Test 5: Process image
    cv::Mat testImage(480, 640, CV_8UC1, cv::Scalar(128));
    adapter.processImage(testImage, 1000000000LL);
    std::cout << "PASS: Image processed" << std::endl;

    // Test 6: Get pose
    Pose6DoF pose;
    bool poseValid = adapter.getPose(pose);
    std::cout << "Pose valid: " << (poseValid ? "true" : "false") << std::endl;

    // Test 7: Get map points
    std::vector<MapPoint> mapPoints = adapter.getMapPoints();
    std::cout << "PASS: Retrieved " << mapPoints.size() << " map points" << std::endl;

    // Test 8: Reset
    adapter.reset();
    status = adapter.getStatus();
    if (status != TrackingStatus::INITIALIZING && status != TrackingStatus::UNINITIALIZED) {
        std::cerr << "FAIL: Status after reset should be INITIALIZING or UNINITIALIZED" << std::endl;
        return 1;
    }
    std::cout << "PASS: Reset successful" << std::endl;

    // Test 9: Shutdown
    adapter.shutdown();
    status = adapter.getStatus();
    if (status != TrackingStatus::UNINITIALIZED) {
        std::cerr << "FAIL: Status after shutdown should be UNINITIALIZED" << std::endl;
        return 1;
    }
    std::cout << "PASS: Shutdown successful" << std::endl;

    std::cout << "\nAll tests passed!" << std::endl;
    return 0;
}
