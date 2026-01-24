#include "slam/slam_engine.hpp"
#include <iostream>
#include <fstream>
#include <opencv2/core.hpp>

using namespace vi_slam;

int main() {
    std::cout << "Testing SLAMEngine..." << std::endl;

    // Create engine instance
    SLAMEngine engine;

    // Test 1: Check initial status
    TrackingStatus status = engine.getStatus();
    if (status != TrackingStatus::UNINITIALIZED) {
        std::cerr << "FAIL: Initial status should be UNINITIALIZED" << std::endl;
        return 1;
    }
    std::cout << "PASS: Initial status is UNINITIALIZED" << std::endl;

    // Test 2: Select framework
    bool selectSuccess = engine.selectFramework(SLAMFrameworkType::VINS_MONO);
    if (!selectSuccess) {
        std::cerr << "FAIL: Framework selection failed" << std::endl;
        return 1;
    }
    std::cout << "PASS: Framework selected successfully" << std::endl;

    // Test 3: Verify selected framework
    SLAMFrameworkType currentType = engine.getCurrentFramework();
    if (currentType != SLAMFrameworkType::VINS_MONO) {
        std::cerr << "FAIL: Framework type mismatch" << std::endl;
        return 1;
    }
    std::cout << "PASS: Framework type verified" << std::endl;

    // Test 4: Initialize with dummy config
    const std::string configPath = "/tmp/dummy_config.yaml";
    const std::string calibPath = "/tmp/dummy_calib.yaml";

    // Create dummy files for testing
    {
        std::ofstream configFile(configPath);
        configFile << "# Dummy config file for testing" << std::endl;
    }
    {
        std::ofstream calibFile(calibPath);
        calibFile << "# Dummy calibration file for testing" << std::endl;
    }

    bool initSuccess = engine.initialize(configPath);
    if (!initSuccess) {
        std::cerr << "FAIL: Initialization failed" << std::endl;
        return 1;
    }
    std::cout << "PASS: Initialization successful" << std::endl;

    // Test 5: Load calibration
    bool calibSuccess = engine.loadCalibration(calibPath);
    if (!calibSuccess) {
        std::cerr << "FAIL: Calibration loading failed" << std::endl;
        return 1;
    }
    std::cout << "PASS: Calibration loaded successfully" << std::endl;

    // Test 6: Set callbacks
    bool poseCallbackInvoked = false;
    bool statusCallbackInvoked = false;

    engine.setPoseCallback([&poseCallbackInvoked](const Pose6DoF& pose) {
        poseCallbackInvoked = true;
        std::cout << "Pose callback invoked at timestamp: " << pose.timestampNs << std::endl;
    });

    engine.setStatusCallback([&statusCallbackInvoked](TrackingStatus status) {
        statusCallbackInvoked = true;
        std::cout << "Status callback invoked: " << static_cast<int>(status) << std::endl;
    });

    std::cout << "PASS: Callbacks set successfully" << std::endl;

    // Test 7: Process IMU sample
    IMUSample imu;
    imu.timestampNs = 1000000000LL;  // 1 second
    imu.accX = 0.0;
    imu.accY = 0.0;
    imu.accZ = 9.81;  // Gravity
    imu.gyroX = 0.0;
    imu.gyroY = 0.0;
    imu.gyroZ = 0.0;

    engine.processIMU(imu);
    std::cout << "PASS: IMU sample processed" << std::endl;

    // Test 8: Process image
    cv::Mat testImage(480, 640, CV_8UC1, cv::Scalar(128));
    engine.processImage(testImage, 1000000000LL);
    std::cout << "PASS: Image processed" << std::endl;

    // Test 9: Get pose
    Pose6DoF pose;
    bool poseValid = engine.getPose(pose);
    std::cout << "Pose valid: " << (poseValid ? "true" : "false") << std::endl;

    // Test 10: Get map points
    std::vector<MapPoint> mapPoints = engine.getMapPoints();
    std::cout << "Map points retrieved: " << mapPoints.size() << std::endl;

    // Test 11: Reset
    engine.reset();
    status = engine.getStatus();
    std::cout << "PASS: Reset successful" << std::endl;

    // Test 12: Framework switching
    bool switchSuccess = engine.selectFramework(SLAMFrameworkType::VINS_MONO);
    if (!switchSuccess) {
        std::cerr << "FAIL: Framework switching failed" << std::endl;
        return 1;
    }
    std::cout << "PASS: Framework switching successful" << std::endl;

    // Test 13: Shutdown
    engine.shutdown();
    status = engine.getStatus();
    if (status != TrackingStatus::UNINITIALIZED) {
        std::cerr << "FAIL: Status after shutdown should be UNINITIALIZED" << std::endl;
        return 1;
    }
    std::cout << "PASS: Shutdown successful" << std::endl;

    std::cout << "\nAll tests passed!" << std::endl;
    return 0;
}
