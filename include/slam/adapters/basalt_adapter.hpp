#ifndef VI_SLAM_BASALT_ADAPTER_HPP
#define VI_SLAM_BASALT_ADAPTER_HPP

#include "slam/i_slam_framework.hpp"
#include <memory>
#include <mutex>
#include <deque>
#include <atomic>
#include <string>

namespace vi_slam {

// Forward declaration of Basalt VIO system wrapper
class BasaltVioSystem;

/**
 * @brief Basalt framework adapter configuration parameters
 *
 * Basalt uses JSON format for configuration. These parameters control
 * the optical flow based VIO system behavior including feature tracking,
 * IMU integration, and state estimation.
 */
struct BasaltConfig {
    // VIO mode settings
    std::string vioMode = "mono";         // "mono" or "stereo"
    bool debugMode = false;

    // Camera intrinsics
    std::string cameraModel = "pinhole";  // "pinhole", "unified", "eucm", "ds"
    double fx = 458.654;
    double fy = 457.296;
    double cx = 367.215;
    double cy = 248.375;
    int imageWidth = 752;
    int imageHeight = 480;

    // Distortion coefficients (radtan model)
    double k1 = -0.28340811;
    double k2 = 0.07395907;
    double p1 = 0.00019359;
    double p2 = 1.76187114e-05;

    // Camera-IMU extrinsics (T_i_c: transform from camera to IMU)
    double ticQw = 0.0148655429818;
    double ticQx = -0.999880929698;
    double ticQy = 0.00414029679422;
    double ticQz = -0.00485987917536;
    double ticTx = 0.0652229934167;
    double ticTy = -0.0207055214469;
    double ticTz = -0.00808170117104;
    double tdCam = 0.0;                   // Time offset (td = t_imu - t_cam)

    // IMU parameters
    double imuRate = 200.0;               // Hz
    double accNoise = 0.08;               // m/s^2/sqrt(Hz)
    double gyroNoise = 0.004;             // rad/s/sqrt(Hz)
    double accBiasWalk = 4.0e-05;         // m/s^3/sqrt(Hz)
    double gyroBiasWalk = 2.0e-06;        // rad/s^2/sqrt(Hz)
    double gravity = 9.81007;             // m/s^2

    // Optical flow parameters
    int maxFlowPoints = 200;              // Maximum number of flow points
    double flowQualityThreshold = 0.7;    // Quality threshold for flow points
    int pyramidLevels = 3;                // Number of pyramid levels
    int patchSize = 21;                   // Patch size for KLT tracking
    int maxFlow = 50;                     // Maximum optical flow displacement
    bool subpixel = true;                 // Enable subpixel refinement
    int fastThreshold = 20;               // FAST corner detection threshold
    int minDistance = 10;                 // Minimum distance between features

    // VIO state estimation parameters
    int maxFrames = 5;                    // Maximum frames in optimization window
    int maxKeyframes = 7;                 // Maximum keyframes
    double minParallax = 10.0;            // Minimum parallax for new keyframe
    bool useImuPreintegration = true;     // Use IMU pre-integration
    bool loopClosure = false;             // Enable loop closure
    std::string marginalization = "oldest"; // Marginalization type

    // Initialization parameters
    double imuInitWindow = 1.0;           // IMU initialization window (seconds)
    int minInitFeatures = 20;             // Minimum features for initialization
    bool staticInit = true;               // Use static initialization
    double maxGyroNorm = 0.1;             // Max gyro norm for static init
    double maxAccDeviation = 0.1;         // Max acc deviation for static init

    // Solver parameters
    int maxIterations = 10;               // Maximum solver iterations
    double convergenceThreshold = 1.0e-06; // Convergence threshold
    double lmDamping = 1.0e-04;           // Levenberg-Marquardt damping
    bool useHuber = true;                 // Use Huber robust cost
    double huberThreshold = 1.0;          // Huber threshold

    // Output settings
    bool saveTrajectory = true;
    std::string trajectoryFormat = "tum"; // "tum", "kitti", "euroc"
    bool saveMap = false;
    int verbosity = 1;                    // 0 = quiet, 1 = normal, 2 = debug
};

/**
 * @brief Basalt VIO framework adapter for VI-SLAM
 *
 * Provides an adapter interface for the Basalt VIO framework. Basalt is
 * a highly efficient visual-inertial odometry system based on optical flow
 * tracking and non-linear optimization.
 *
 * Key features:
 * - Optical flow based tracking (faster than descriptor matching)
 * - Non-linear optimization with Schur complement
 * - Support for monocular and stereo cameras
 * - Real-time performance on embedded platforms
 *
 * Reference: https://gitlab.com/VladyslavUsenko/basalt
 */
class BasaltAdapter : public ISLAMFramework {
public:
    BasaltAdapter();
    ~BasaltAdapter() override;

    // Non-copyable
    BasaltAdapter(const BasaltAdapter&) = delete;
    BasaltAdapter& operator=(const BasaltAdapter&) = delete;

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
     * @return Current Basalt configuration parameters
     */
    BasaltConfig getConfig() const;

    /**
     * @brief Check if the system has received sufficient IMU data for init
     * @return true if ready to process images
     */
    bool isReadyForVision() const;

    /**
     * @brief Get the number of currently tracked optical flow points
     * @return Number of active flow points
     */
    int getTrackedFeatureCount() const;

private:
    // Basalt VIO system instance
    std::unique_ptr<BasaltVioSystem> vioSystem_;

    // State management
    std::atomic<TrackingStatus> status_;
    mutable std::mutex statusMutex_;

    // Latest pose
    mutable Pose6DoF latestPose_;
    mutable std::mutex poseMutex_;

    // IMU buffer for initialization and synchronization
    std::deque<IMUSample> imuBuffer_;
    mutable std::mutex imuMutex_;
    int64_t lastImageTimestampNs_;
    int64_t lastImuTimestampNs_;

    // Configuration
    BasaltConfig config_;
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

#endif  // VI_SLAM_BASALT_ADAPTER_HPP
