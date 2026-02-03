#ifndef VI_SLAM_ORBSLAM3_ADAPTER_HPP
#define VI_SLAM_ORBSLAM3_ADAPTER_HPP

#include "slam/i_slam_framework.hpp"
#include <memory>
#include <mutex>
#include <deque>
#include <atomic>
#include <string>

namespace vi_slam {

// Forward declaration of ORB-SLAM3 System wrapper
class ORBSLAM3System;

/**
 * @brief Configuration parameters for ORB-SLAM3
 *
 * These parameters are parsed from the YAML configuration file
 * and match the ORB-SLAM3 expected format.
 */
struct ORBSLAM3Config {
    // Vocabulary and sensor settings
    std::string vocabularyPath = "";
    std::string sensorType = "Monocular-Inertial";
    std::string cameraType = "PinHole";

    // Camera intrinsics
    double fx = 458.654;
    double fy = 457.296;
    double cx = 367.215;
    double cy = 248.375;

    // Distortion coefficients (RadTan model)
    double k1 = 0.0;
    double k2 = 0.0;
    double p1 = 0.0;
    double p2 = 0.0;
    double k3 = 0.0;

    // Image settings
    int imageWidth = 752;
    int imageHeight = 480;
    double fps = 20.0;
    bool isRGB = true;

    // Stereo settings
    double bf = 0.0;
    double thDepth = 40.0;
    double depthMapFactor = 1.0;

    // IMU parameters
    double imuFrequency = 200.0;
    double noiseGyro = 0.004;
    double noiseAcc = 0.08;
    double gyroWalk = 2.0e-05;
    double accWalk = 4.0e-04;
    double imuTimeOffset = 0.0;

    // Camera-IMU extrinsics (4x4 transformation matrix, row-major)
    double Tbc[16] = {
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0
    };

    // ORB feature parameters
    int nFeatures = 1200;
    double scaleFactor = 1.2;
    int nLevels = 8;
    int iniThFAST = 20;
    int minThFAST = 7;

    // System settings
    bool useViewer = false;
    bool saveTrajectory = false;
    std::string trajectoryFile = "trajectory.txt";
    int maxMaps = 5;
    int localWindow = 10;
};

/**
 * @brief Adapter for ORB-SLAM3 visual-inertial SLAM framework
 *
 * This adapter wraps the ORB-SLAM3 library functionality, providing
 * a unified interface for visual-inertial odometry and mapping.
 *
 * ORB-SLAM3 features:
 * - Monocular, stereo, and RGB-D camera support
 * - Tight IMU integration (visual-inertial mode)
 * - Loop closing and relocalization
 * - Multi-map support (Atlas)
 *
 * Reference: https://github.com/UZ-SLAMLab/ORB_SLAM3
 */
class ORBSLAM3Adapter : public ISLAMFramework {
public:
    ORBSLAM3Adapter();
    ~ORBSLAM3Adapter() override;

    // ISLAMFramework interface implementation
    bool initialize(const std::string& configPath) override;
    bool loadCalibration(const std::string& calibPath) override;

    void processImage(const cv::Mat& image, int64_t timestampNs) override;
    void processIMU(const IMUSample& imu) override;

    bool getPose(Pose6DoF& pose) const override;
    TrackingStatus getStatus() const override;
    std::vector<MapPoint> getMapPoints() const override;

    void reset() override;
    void shutdown() override;

    // ORB-SLAM3 specific methods
    ORBSLAM3Config getConfig() const;
    bool isReadyForVision() const;
    int getTrackedFeatureCount() const;

private:
    // ORB-SLAM3 System instance
    std::unique_ptr<ORBSLAM3System> system_;

    // Configuration
    ORBSLAM3Config config_;
    std::string configPath_;
    std::string calibPath_;

    // State management
    std::atomic<TrackingStatus> status_;
    mutable std::mutex statusMutex_;

    // Latest pose
    mutable Pose6DoF latestPose_;
    mutable std::mutex poseMutex_;

    // IMU buffer for VI mode
    std::deque<IMUSample> imuBuffer_;
    mutable std::mutex imuMutex_;
    static constexpr size_t MAX_IMU_BUFFER_SIZE = 2000;

    // Timing and initialization
    int64_t lastImageTimestampNs_;
    int64_t lastImuTimestampNs_;
    int64_t initStartTimeNs_;
    bool initialized_;
    bool calibrated_;
    std::atomic<bool> imuInitialized_;
    std::atomic<int> trackedFeatureCount_;

    // IMU initialization threshold (seconds of IMU data needed)
    static constexpr double IMU_INIT_WINDOW = 1.0;

    // Helper methods
    bool loadConfiguration(const std::string& configPath);
    bool loadCalibrationData(const std::string& calibPath);
    bool parseYamlConfig(const std::string& configPath);
    void updateStatus(TrackingStatus newStatus);
    void updatePose(const Pose6DoF& pose);
    void processIMUQueue();
    bool checkImuInitialization(int64_t imageTimestampNs);

    // Logging helpers
    void logStatus(const std::string& message) const;
    void logError(const std::string& message) const;
};

}  // namespace vi_slam

#endif  // VI_SLAM_ORBSLAM3_ADAPTER_HPP
