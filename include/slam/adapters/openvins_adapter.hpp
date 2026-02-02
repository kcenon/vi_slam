#ifndef VI_SLAM_OPENVINS_ADAPTER_HPP
#define VI_SLAM_OPENVINS_ADAPTER_HPP

#include "slam/i_slam_framework.hpp"
#include <memory>
#include <mutex>
#include <deque>
#include <atomic>
#include <string>
#include <unordered_map>

namespace vi_slam {

// Forward declaration of OpenVINS VioManager wrapper
class OpenVINSVioManager;

/**
 * @brief OpenVINS framework adapter configuration parameters
 *
 * OpenVINS uses YAML format for configuration. These parameters control
 * the VIO system behavior including feature tracking, IMU integration,
 * and state estimation.
 */
struct OpenVINSConfig {
    // Feature tracking parameters
    int numFeatures = 200;           // Number of features to track
    int numAruco = 0;                // Number of ArUco tags (0 = disabled)
    bool useKLT = true;              // Use KLT tracker (vs descriptor-based)
    int fastThreshold = 20;          // FAST corner detection threshold
    int gridX = 5;                   // Grid rows for feature distribution
    int gridY = 5;                   // Grid columns for feature distribution
    int minPxDist = 10;              // Minimum pixel distance between features

    // IMU noise parameters
    double sigmaAcc = 0.1;           // Accelerometer white noise (m/s^2)
    double sigmaGyro = 0.01;         // Gyroscope white noise (rad/s)
    double sigmaAccBias = 0.001;     // Accelerometer bias random walk
    double sigmaGyroBias = 0.0001;   // Gyroscope bias random walk

    // State estimation parameters
    int maxClones = 11;              // Maximum number of clones in state
    int maxSlam = 50;                // Maximum SLAM features in state
    int maxMsckfUpdate = 999;        // Maximum MSCKF features per update
    bool useZeroVelocityUpdate = true;  // Enable zero velocity updates

    // Initialization parameters
    double imuInitWindow = 1.0;      // IMU initialization window (seconds)
    int maxImuWaiting = 3;           // Maximum IMU waiting time (seconds)
};

/**
 * @brief OpenVINS framework adapter for VI-SLAM
 *
 * Provides an adapter interface for the OpenVINS (Open Visual-Inertial
 * Navigation System) framework. OpenVINS is a research-grade VIO system
 * that implements the Multi-State Constraint Kalman Filter (MSCKF).
 *
 * Key features:
 * - High-precision visual-inertial odometry
 * - Support for monocular and stereo cameras
 * - Online spatial and temporal calibration
 * - ArUco marker support for initialization and scale recovery
 *
 * Configuration uses YAML format compatible with OpenVINS conventions.
 */
class OpenVINSAdapter : public ISLAMFramework {
public:
    OpenVINSAdapter();
    ~OpenVINSAdapter() override;

    // Non-copyable
    OpenVINSAdapter(const OpenVINSAdapter&) = delete;
    OpenVINSAdapter& operator=(const OpenVINSAdapter&) = delete;

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

    /**
     * @brief Get the current configuration
     * @return Current OpenVINS configuration parameters
     */
    OpenVINSConfig getConfig() const;

    /**
     * @brief Check if the system has received sufficient IMU data for init
     * @return true if ready to process images
     */
    bool isReadyForVision() const;

    /**
     * @brief Get the number of currently tracked features
     * @return Number of active feature tracks
     */
    int getTrackedFeatureCount() const;

private:
    // OpenVINS VioManager wrapper instance
    std::unique_ptr<OpenVINSVioManager> vioManager_;

    // State management
    std::atomic<TrackingStatus> status_;
    mutable std::mutex statusMutex_;

    // Latest pose
    mutable Pose6DoF latestPose_;
    mutable std::mutex poseMutex_;

    // IMU buffer for initialization and processing
    std::deque<IMUSample> imuBuffer_;
    mutable std::mutex imuMutex_;
    int64_t lastImageTimestampNs_;
    int64_t lastImuTimestampNs_;

    // Configuration
    OpenVINSConfig config_;
    std::string configPath_;
    std::string calibPath_;
    std::atomic<bool> initialized_;
    std::atomic<bool> calibrated_;

    // Feature tracking stats
    std::atomic<int> trackedFeatureCount_;

    // Internal state
    bool imuInitialized_;
    int64_t initStartTimeNs_;
    static constexpr size_t MAX_IMU_BUFFER_SIZE = 2000;

    // Helper methods
    bool loadConfiguration(const std::string& configPath);
    bool loadCalibrationData(const std::string& calibPath);
    bool parseYamlConfig(const std::string& configPath);
    void updateStatus(TrackingStatus newStatus);
    void updatePose(const Pose6DoF& pose);
    void processIMUQueue();
    bool checkImuInitialization(int64_t imageTimestampNs);
    void logStatus(const std::string& message) const;
    void logError(const std::string& message) const;
};

}  // namespace vi_slam

#endif  // VI_SLAM_OPENVINS_ADAPTER_HPP
