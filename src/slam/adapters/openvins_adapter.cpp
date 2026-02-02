#include "slam/adapters/openvins_adapter.hpp"
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <chrono>

namespace vi_slam {

/**
 * @brief OpenVINS VioManager wrapper class
 *
 * This class wraps the OpenVINS VioManager functionality. When OpenVINS
 * library is available, this class interfaces with the actual VioManager.
 * Otherwise, it provides a placeholder implementation for development.
 *
 * OpenVINS API Reference:
 * - VioManager: Main class that handles the visual-inertial odometry
 * - VioManagerOptions: Configuration options parsed from YAML
 * - State: Current state estimate including pose, velocity, biases
 * - Propagator: IMU integration using preintegration
 * - UpdaterMSCKF: MSCKF feature update
 * - UpdaterSLAM: SLAM feature update
 *
 * Integration points with actual OpenVINS:
 * 1. include "open_vins/core/VioManager.h"
 * 2. Replace placeholder methods with actual OpenVINS API calls
 * 3. Link against ov_core, ov_init, ov_msckf libraries
 */
class OpenVINSVioManager {
public:
    OpenVINSVioManager() : initialized_(false), trackCount_(0) {}

    /**
     * @brief Initialize the VIO system with configuration
     * @param config Configuration path (YAML file)
     * @param calib Calibration path (YAML file)
     * @return true if initialization successful
     *
     * OpenVINS actual implementation:
     * @code
     * auto params = std::make_shared<ov_msckf::VioManagerOptions>();
     * params->parse_and_output(config, calib);
     * vioManager_ = std::make_shared<ov_msckf::VioManager>(params);
     * @endcode
     */
    bool initialize(const std::string& config, const std::string& calib,
                    const OpenVINSConfig& params) {
        std::cout << "[OpenVINS] Initializing VioManager" << std::endl;
        std::cout << "[OpenVINS] Config: " << config << std::endl;
        std::cout << "[OpenVINS] Calibration: " << calib << std::endl;
        std::cout << "[OpenVINS] Parameters:" << std::endl;
        std::cout << "  - Number of features: " << params.numFeatures << std::endl;
        std::cout << "  - Use KLT tracker: " << (params.useKLT ? "yes" : "no") << std::endl;
        std::cout << "  - Max clones: " << params.maxClones << std::endl;

        config_ = params;
        initialized_ = true;
        return true;
    }

    /**
     * @brief Feed monocular image to the VIO system
     * @param image Input image (grayscale or color)
     * @param timestampNs Timestamp in nanoseconds
     *
     * OpenVINS actual implementation:
     * @code
     * double timestamp = timestampNs * 1e-9;
     * cv::Mat gray;
     * if (image.channels() == 3) {
     *     cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
     * } else {
     *     gray = image;
     * }
     * ov_core::CameraData cam_msg;
     * cam_msg.timestamp = timestamp;
     * cam_msg.sensor_ids.push_back(0);
     * cam_msg.images.push_back(gray.clone());
     * vioManager_->feed_measurement_camera(cam_msg);
     * @endcode
     */
    void feedImage(const cv::Mat& image, int64_t timestampNs) {
        (void)timestampNs;

        if (!initialized_) {
            std::cerr << "[OpenVINS] VioManager not initialized" << std::endl;
            return;
        }

        // Placeholder: simulate feature tracking
        // In actual implementation, OpenVINS TrackKLT or TrackDescriptor
        // extracts and tracks features
        int corners = 0;
        if (!image.empty()) {
            // Rough estimate based on image content
            cv::Mat gray;
            if (image.channels() == 3) {
                cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
            } else {
                gray = image;
            }

            // Simple corner detection for placeholder
            std::vector<cv::Point2f> corners_vec;
            cv::goodFeaturesToTrack(gray, corners_vec, config_.numFeatures,
                                    0.01, config_.minPxDist);
            corners = static_cast<int>(corners_vec.size());
        }

        trackCount_ = corners;
        std::cout << "[OpenVINS] Processing image, tracked features: "
                  << trackCount_ << std::endl;
    }

    /**
     * @brief Feed stereo image pair to the VIO system
     * @param imageLeft Left camera image
     * @param imageRight Right camera image
     * @param timestampNs Timestamp in nanoseconds
     */
    void feedStereoImage(const cv::Mat& imageLeft, const cv::Mat& imageRight,
                         int64_t timestampNs) {
        (void)imageRight;
        // For now, just process left image
        feedImage(imageLeft, timestampNs);
    }

    /**
     * @brief Feed IMU measurement to the VIO system
     * @param imu IMU sample with accelerometer and gyroscope data
     *
     * OpenVINS actual implementation:
     * @code
     * double timestamp = imu.timestampNs * 1e-9;
     * Eigen::Matrix<double, 3, 1> wm;
     * wm << imu.gyroX, imu.gyroY, imu.gyroZ;
     * Eigen::Matrix<double, 3, 1> am;
     * am << imu.accX, imu.accY, imu.accZ;
     * ov_core::ImuData imu_msg(timestamp, wm, am);
     * vioManager_->feed_measurement_imu(imu_msg);
     * @endcode
     */
    void feedIMU(const IMUSample& imu) {
        if (!initialized_) {
            return;
        }

        // Placeholder: accumulate IMU for integration
        // In actual implementation, OpenVINS Propagator handles IMU
        // preintegration using continuous-time integration
        lastImu_ = imu;
        imuCount_++;

        // Every 100 IMU samples, log status
        if (imuCount_ % 100 == 0) {
            std::cout << "[OpenVINS] IMU samples processed: " << imuCount_
                      << ", acc: [" << imu.accX << ", " << imu.accY << ", "
                      << imu.accZ << "]" << std::endl;
        }
    }

    /**
     * @brief Get current pose estimate
     * @param[out] pose Output 6DoF pose
     * @return true if valid pose available
     *
     * OpenVINS actual implementation:
     * @code
     * auto state = vioManager_->get_state();
     * if (state == nullptr || !state->initialized()) {
     *     return false;
     * }
     * Eigen::Matrix<double, 3, 1> pos = state->imu()->pos();
     * Eigen::Matrix<double, 4, 1> quat = state->imu()->quat();
     *
     * pose.timestampNs = static_cast<int64_t>(state->timestamp() * 1e9);
     * pose.position[0] = pos(0);
     * pose.position[1] = pos(1);
     * pose.position[2] = pos(2);
     * // OpenVINS uses JPL quaternion: (x, y, z, w)
     * // Convert to (w, x, y, z) format
     * pose.orientation[0] = quat(3);  // w
     * pose.orientation[1] = quat(0);  // x
     * pose.orientation[2] = quat(1);  // y
     * pose.orientation[3] = quat(2);  // z
     * pose.valid = true;
     * return true;
     * @endcode
     */
    bool getPose(Pose6DoF& pose) const {
        if (!initialized_ || imuCount_ < 10) {
            return false;
        }

        // Placeholder: return a simulated pose based on IMU integration
        // Actual pose comes from OpenVINS EKF state estimate
        pose.timestampNs = lastImu_.timestampNs;

        // Simulate slight movement based on accumulated IMU
        // In real implementation, this comes from the EKF state
        static double x = 0.0, y = 0.0, z = 0.0;
        x += lastImu_.accX * 0.00001;
        y += lastImu_.accY * 0.00001;
        z += lastImu_.accZ * 0.00001;

        pose.position[0] = x;
        pose.position[1] = y;
        pose.position[2] = z;

        // Identity quaternion (w, x, y, z)
        pose.orientation[0] = 1.0;
        pose.orientation[1] = 0.0;
        pose.orientation[2] = 0.0;
        pose.orientation[3] = 0.0;

        pose.valid = true;
        return true;
    }

    /**
     * @brief Get current tracked feature count
     */
    int getTrackedFeatureCount() const {
        return trackCount_;
    }

    /**
     * @brief Check if VIO is initialized
     */
    bool isInitialized() const {
        return initialized_ && imuCount_ > 0;
    }

    /**
     * @brief Reset the VIO state
     *
     * OpenVINS actual implementation:
     * @code
     * vioManager_.reset();
     * auto params = std::make_shared<ov_msckf::VioManagerOptions>();
     * params->parse_and_output(configPath_, calibPath_);
     * vioManager_ = std::make_shared<ov_msckf::VioManager>(params);
     * @endcode
     */
    void reset() {
        std::cout << "[OpenVINS] Resetting VioManager state" << std::endl;
        imuCount_ = 0;
        trackCount_ = 0;
    }

    /**
     * @brief Shutdown the VIO system
     */
    void shutdown() {
        std::cout << "[OpenVINS] Shutting down VioManager" << std::endl;
        initialized_ = false;
        imuCount_ = 0;
        trackCount_ = 0;
    }

private:
    bool initialized_;
    OpenVINSConfig config_;
    IMUSample lastImu_;
    int imuCount_ = 0;
    int trackCount_;
};

OpenVINSAdapter::OpenVINSAdapter()
    : vioManager_(std::make_unique<OpenVINSVioManager>()),
      status_(TrackingStatus::UNINITIALIZED),
      lastImageTimestampNs_(0),
      lastImuTimestampNs_(0),
      initialized_(false),
      calibrated_(false),
      trackedFeatureCount_(0),
      imuInitialized_(false),
      initStartTimeNs_(0) {
}

OpenVINSAdapter::~OpenVINSAdapter() {
    shutdown();
}

bool OpenVINSAdapter::initialize(const std::string& configPath) {
    std::lock_guard<std::mutex> lock(statusMutex_);

    logStatus("Initializing OpenVINS adapter with config: " + configPath);

    configPath_ = configPath;

    if (!loadConfiguration(configPath)) {
        logError("Failed to load configuration from: " + configPath);
        return false;
    }

    updateStatus(TrackingStatus::INITIALIZING);
    initialized_ = true;

    logStatus("OpenVINS adapter initialized successfully");
    return true;
}

bool OpenVINSAdapter::loadCalibration(const std::string& calibPath) {
    std::lock_guard<std::mutex> lock(statusMutex_);

    logStatus("Loading calibration from: " + calibPath);

    calibPath_ = calibPath;

    if (!loadCalibrationData(calibPath)) {
        logError("Failed to load calibration from: " + calibPath);
        return false;
    }

    calibrated_ = true;

    if (initialized_ && !configPath_.empty()) {
        // Initialize OpenVINS with both config and calibration
        if (!vioManager_->initialize(configPath_, calibPath_, config_)) {
            logError("Failed to initialize OpenVINS VioManager");
            updateStatus(TrackingStatus::UNINITIALIZED);
            return false;
        }
        logStatus("OpenVINS VioManager ready for data");
    }

    logStatus("Calibration loaded successfully");
    return true;
}

void OpenVINSAdapter::processImage(const cv::Mat& image, int64_t timestampNs) {
    if (!initialized_) {
        logError("OpenVINS adapter not initialized");
        return;
    }

    if (!calibrated_) {
        logError("Calibration not loaded");
        return;
    }

    if (image.empty()) {
        logError("Received empty image");
        return;
    }

    // Check if we have enough IMU data for initialization
    if (!checkImuInitialization(timestampNs)) {
        logStatus("Waiting for IMU initialization data...");
        return;
    }

    // Process queued IMU samples up to this image timestamp
    processIMUQueue();

    // Feed image to OpenVINS
    vioManager_->feedImage(image, timestampNs);
    lastImageTimestampNs_ = timestampNs;

    // Update tracked feature count
    trackedFeatureCount_ = vioManager_->getTrackedFeatureCount();

    // Update pose from VioManager
    Pose6DoF pose;
    if (vioManager_->getPose(pose)) {
        updatePose(pose);
        if (status_ != TrackingStatus::TRACKING) {
            updateStatus(TrackingStatus::TRACKING);
            logStatus("Tracking established");
        }
    } else {
        // If pose retrieval fails, check if we should mark as lost
        if (status_ == TrackingStatus::TRACKING) {
            updateStatus(TrackingStatus::LOST);
            logStatus("Tracking lost");
        }
    }
}

void OpenVINSAdapter::processIMU(const IMUSample& imu) {
    if (!initialized_) {
        return;
    }

    // Validate IMU data
    if (std::isnan(imu.accX) || std::isnan(imu.accY) || std::isnan(imu.accZ) ||
        std::isnan(imu.gyroX) || std::isnan(imu.gyroY) || std::isnan(imu.gyroZ)) {
        logError("Received invalid IMU data (NaN values)");
        return;
    }

    // Check for reasonable IMU values
    constexpr double MAX_ACC = 100.0;  // 100 m/s^2
    constexpr double MAX_GYRO = 10.0;  // 10 rad/s
    if (std::abs(imu.accX) > MAX_ACC || std::abs(imu.accY) > MAX_ACC ||
        std::abs(imu.accZ) > MAX_ACC || std::abs(imu.gyroX) > MAX_GYRO ||
        std::abs(imu.gyroY) > MAX_GYRO || std::abs(imu.gyroZ) > MAX_GYRO) {
        logError("IMU values out of reasonable range");
        return;
    }

    {
        std::lock_guard<std::mutex> lock(imuMutex_);

        // Initialize timing on first IMU sample
        if (initStartTimeNs_ == 0) {
            initStartTimeNs_ = imu.timestampNs;
        }

        imuBuffer_.push_back(imu);
        lastImuTimestampNs_ = imu.timestampNs;

        // Keep buffer size limited to prevent memory growth
        while (imuBuffer_.size() > MAX_IMU_BUFFER_SIZE) {
            imuBuffer_.pop_front();
        }
    }

    // Feed IMU to VioManager
    vioManager_->feedIMU(imu);
}

bool OpenVINSAdapter::getPose(Pose6DoF& pose) const {
    std::lock_guard<std::mutex> lock(poseMutex_);
    pose = latestPose_;
    return latestPose_.valid;
}

TrackingStatus OpenVINSAdapter::getStatus() const {
    return status_.load();
}

std::vector<MapPoint> OpenVINSAdapter::getMapPoints() const {
    // OpenVINS MSCKF doesn't maintain a persistent map
    // Only active SLAM features are available
    // Return empty vector for now
    return std::vector<MapPoint>();
}

void OpenVINSAdapter::reset() {
    std::lock_guard<std::mutex> lock(statusMutex_);

    logStatus("Resetting OpenVINS adapter");

    if (vioManager_) {
        vioManager_->reset();
    }

    {
        std::lock_guard<std::mutex> imuLock(imuMutex_);
        imuBuffer_.clear();
    }

    {
        std::lock_guard<std::mutex> poseLock(poseMutex_);
        latestPose_ = Pose6DoF();
    }

    lastImageTimestampNs_ = 0;
    lastImuTimestampNs_ = 0;
    imuInitialized_ = false;
    initStartTimeNs_ = 0;
    trackedFeatureCount_ = 0;

    updateStatus(TrackingStatus::INITIALIZING);
    logStatus("OpenVINS adapter reset complete");
}

void OpenVINSAdapter::shutdown() {
    std::lock_guard<std::mutex> lock(statusMutex_);

    logStatus("Shutting down OpenVINS adapter");

    if (vioManager_) {
        vioManager_->shutdown();
    }

    initialized_ = false;
    calibrated_ = false;
    updateStatus(TrackingStatus::UNINITIALIZED);
    logStatus("OpenVINS adapter shut down");
}

OpenVINSConfig OpenVINSAdapter::getConfig() const {
    return config_;
}

bool OpenVINSAdapter::isReadyForVision() const {
    std::lock_guard<std::mutex> lock(imuMutex_);

    if (initStartTimeNs_ == 0 || imuBuffer_.empty()) {
        return false;
    }

    // Check if we have enough IMU data (configured initialization window)
    double elapsedSeconds = (lastImuTimestampNs_ - initStartTimeNs_) * 1e-9;
    return elapsedSeconds >= config_.imuInitWindow && imuInitialized_;
}

int OpenVINSAdapter::getTrackedFeatureCount() const {
    return trackedFeatureCount_.load();
}

bool OpenVINSAdapter::loadConfiguration(const std::string& configPath) {
    std::ifstream configFile(configPath);
    if (!configFile.is_open()) {
        logError("Failed to open config file: " + configPath);
        return false;
    }

    // Try to parse YAML configuration
    if (!parseYamlConfig(configPath)) {
        logStatus("Using default configuration parameters");
    }

    logStatus("Configuration loaded from: " + configPath);
    return true;
}

bool OpenVINSAdapter::loadCalibrationData(const std::string& calibPath) {
    std::ifstream calibFile(calibPath);
    if (!calibFile.is_open()) {
        logError("Failed to open calibration file: " + calibPath);
        return false;
    }

    // OpenVINS calibration files are in YAML format and contain:
    // - Camera intrinsics (radtan, equidist, fisheye distortion models)
    // - Camera-IMU extrinsics (T_imu_cam)
    // - IMU noise parameters
    // - Time offset (td_cam_imu)

    // For full implementation, would need YAML parser (yaml-cpp)
    // to extract calibration parameters

    logStatus("Calibration loaded from: " + calibPath);
    return true;
}

bool OpenVINSAdapter::parseYamlConfig(const std::string& configPath) {
    // Simple line-by-line YAML parsing for key parameters
    // For production, use yaml-cpp library
    std::ifstream file(configPath);
    if (!file.is_open()) {
        return false;
    }

    std::string line;
    while (std::getline(file, line)) {
        // Skip comments and empty lines
        if (line.empty() || line[0] == '#') {
            continue;
        }

        // Find key-value pairs
        size_t colonPos = line.find(':');
        if (colonPos == std::string::npos) {
            continue;
        }

        std::string key = line.substr(0, colonPos);
        std::string value = line.substr(colonPos + 1);

        // Trim whitespace
        key.erase(0, key.find_first_not_of(" \t"));
        key.erase(key.find_last_not_of(" \t") + 1);
        value.erase(0, value.find_first_not_of(" \t"));
        value.erase(value.find_last_not_of(" \t") + 1);

        // Parse known parameters
        if (key == "num_features" || key == "num_pts") {
            config_.numFeatures = std::stoi(value);
        } else if (key == "num_aruco") {
            config_.numAruco = std::stoi(value);
        } else if (key == "use_klt") {
            config_.useKLT = (value == "true" || value == "1");
        } else if (key == "fast_threshold") {
            config_.fastThreshold = std::stoi(value);
        } else if (key == "grid_x") {
            config_.gridX = std::stoi(value);
        } else if (key == "grid_y") {
            config_.gridY = std::stoi(value);
        } else if (key == "min_px_dist") {
            config_.minPxDist = std::stoi(value);
        } else if (key == "sigma_a" || key == "accelerometer_noise_density") {
            config_.sigmaAcc = std::stod(value);
        } else if (key == "sigma_g" || key == "gyroscope_noise_density") {
            config_.sigmaGyro = std::stod(value);
        } else if (key == "sigma_ab" || key == "accelerometer_random_walk") {
            config_.sigmaAccBias = std::stod(value);
        } else if (key == "sigma_gb" || key == "gyroscope_random_walk") {
            config_.sigmaGyroBias = std::stod(value);
        } else if (key == "max_clones") {
            config_.maxClones = std::stoi(value);
        } else if (key == "max_slam") {
            config_.maxSlam = std::stoi(value);
        } else if (key == "max_msckf_update") {
            config_.maxMsckfUpdate = std::stoi(value);
        } else if (key == "use_zupt" || key == "use_zero_velocity_update") {
            config_.useZeroVelocityUpdate = (value == "true" || value == "1");
        } else if (key == "imu_init_window") {
            config_.imuInitWindow = std::stod(value);
        }
    }

    return true;
}

void OpenVINSAdapter::updateStatus(TrackingStatus newStatus) {
    status_ = newStatus;

    // Log status change
    const char* statusStr = "UNKNOWN";
    switch (newStatus) {
        case TrackingStatus::UNINITIALIZED: statusStr = "UNINITIALIZED"; break;
        case TrackingStatus::INITIALIZING: statusStr = "INITIALIZING"; break;
        case TrackingStatus::TRACKING: statusStr = "TRACKING"; break;
        case TrackingStatus::LOST: statusStr = "LOST"; break;
        case TrackingStatus::RELOCALIZATION: statusStr = "RELOCALIZATION"; break;
    }
    std::cout << "[OpenVINS] Status: " << statusStr << std::endl;
}

void OpenVINSAdapter::updatePose(const Pose6DoF& pose) {
    std::lock_guard<std::mutex> lock(poseMutex_);
    latestPose_ = pose;
}

void OpenVINSAdapter::processIMUQueue() {
    std::lock_guard<std::mutex> lock(imuMutex_);

    // Process all queued IMU samples
    // This ensures IMU data is processed in temporal order
    // OpenVINS requires IMU data to be processed before corresponding images
    while (!imuBuffer_.empty()) {
        const IMUSample& imu = imuBuffer_.front();

        // Only process IMU samples up to current image timestamp
        if (lastImageTimestampNs_ > 0 && imu.timestampNs > lastImageTimestampNs_) {
            break;
        }

        vioManager_->feedIMU(imu);
        imuBuffer_.pop_front();
    }
}

bool OpenVINSAdapter::checkImuInitialization(int64_t imageTimestampNs) {
    if (imuInitialized_) {
        return true;
    }

    std::lock_guard<std::mutex> lock(imuMutex_);

    if (initStartTimeNs_ == 0 || imuBuffer_.empty()) {
        return false;
    }

    // Check if we have enough IMU data spanning the initialization window
    double elapsedSeconds = (imageTimestampNs - initStartTimeNs_) * 1e-9;

    if (elapsedSeconds >= config_.imuInitWindow) {
        imuInitialized_ = true;
        logStatus("IMU initialization complete after " +
                  std::to_string(elapsedSeconds) + " seconds");
        return true;
    }

    return false;
}

void OpenVINSAdapter::logStatus(const std::string& message) const {
    std::cout << "[OpenVINS] " << message << std::endl;
}

void OpenVINSAdapter::logError(const std::string& message) const {
    std::cerr << "[OpenVINS ERROR] " << message << std::endl;
}

}  // namespace vi_slam
