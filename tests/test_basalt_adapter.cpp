#include "slam/adapters/basalt_adapter.hpp"
#include <iostream>
#include <fstream>
#include <opencv2/core.hpp>
#include <Eigen/Core>

using namespace vi_slam;

int main() {
    std::cout << "Testing BasaltAdapter..." << std::endl;

    // Create adapter instance
    BasaltAdapter adapter;

    // Test 1: Check initial status
    TrackingStatus status = adapter.getStatus();
    if (status != TrackingStatus::UNINITIALIZED) {
        std::cerr << "FAIL: Initial status should be UNINITIALIZED" << std::endl;
        return 1;
    }
    std::cout << "PASS: Initial status is UNINITIALIZED" << std::endl;

    // Test 2: Initialize with dummy config
    const std::string configPath = "/tmp/dummy_basalt_config.json";
    const std::string calibPath = "/tmp/dummy_basalt_calib.json";

    // Create dummy files for testing
    {
        std::ofstream configFile(configPath);
        configFile << "{" << std::endl;
        configFile << "  \"optical_flow\": {" << std::endl;
        configFile << "    \"max_points\": 200," << std::endl;
        configFile << "    \"quality_threshold\": 0.7" << std::endl;
        configFile << "  }," << std::endl;
        configFile << "  \"vio\": {" << std::endl;
        configFile << "    \"backend\": \"square_root\"" << std::endl;
        configFile << "  }" << std::endl;
        configFile << "}" << std::endl;
    }
    {
        std::ofstream calibFile(calibPath);
        calibFile << "{" << std::endl;
        calibFile << "  \"intrinsics\": [" << std::endl;
        calibFile << "    {" << std::endl;
        calibFile << "      \"camera_type\": \"pinhole\"," << std::endl;
        calibFile << "      \"intrinsics\": [458.654, 457.296, 367.215, 248.375]" << std::endl;
        calibFile << "    }" << std::endl;
        calibFile << "  ]," << std::endl;
        calibFile << "  \"T_i_c\": [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]" << std::endl;
        calibFile << "}" << std::endl;
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
    imu.acceleration = Eigen::Vector3d(0.0, 0.0, 9.81);  // Gravity
    imu.angularVelocity = Eigen::Vector3d::Zero();

    adapter.processIMU(imu);
    std::cout << "PASS: IMU sample processed" << std::endl;

    // Test 5: Process image (Basalt uses optical flow)
    cv::Mat testImage(480, 640, CV_8UC1, cv::Scalar(128));
    adapter.processImage(testImage, 1000000000LL);
    std::cout << "PASS: Image processed" << std::endl;

    // Test 6: Get pose
    Pose6DoF pose;
    bool poseValid = adapter.getPose(pose);
    std::cout << "Pose valid: " << (poseValid ? "true" : "false") << std::endl;

    // Test 7: Get map points (Basalt maintains sparse feature map)
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
