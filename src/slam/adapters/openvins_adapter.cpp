#include "slam/adapters/openvins_adapter.hpp"
#include "common/logging.hpp"
#include <opencv2/imgproc.hpp>
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
     */
    bool initialize(const std::string& config, const std::string& calib,
                    const OpenVINSConfig& params) {
        LOG_INFO("OpenVINS", "Initializing VioManager");
        LOG_DEBUG("OpenVINS", "Config: {}", config);
        LOG_DEBUG("OpenVINS", "Calibration: {}", calib);
        LOG_DEBUG("OpenVINS", "Parameters: numFeatures={}, useKLT={}, maxClones={}",
                  params.numFeatures, params.useKLT ? 1 : 0, params.maxClones);

        config_ = params;
        initialized_ = true;
        return true;
    }

    /**
     * @brief Feed monocular image to the VIO system
     * @param image Input image (grayscale or color)
     * @param timestampNs Timestamp in nanoseconds
     */
    void feedImage(const cv::Mat& image, int64_t timestampNs) {
        (void)timestampNs;

        if (!initialized_) {
            LOG_WARN("OpenVINS", "VioManager not initialized");
            return;
        }

        // Placeholder: simulate feature tracking
        int corners = 0;
        if (!image.empty()) {
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
        LOG_DEBUG("OpenVINS", "Processing image, tracked features: {}", trackCount_);
    }

    /**
     * @brief Feed stereo image pair to the VIO system
     */
    void feedStereoImage(const cv::Mat& imageLeft, const cv::Mat& imageRight,
                         int64_t timestampNs) {
        (void)imageRight;
        feedImage(imageLeft, timestampNs);
    }

    /**
     * @brief Feed IMU measurement to the VIO system
     */
    void feedIMU(const IMUSample& imu) {
        if (!initialized_) {
            return;
        }

        lastImu_ = imu;
        imuCount_++;

        // Every 100 IMU samples, log status
        if (imuCount_ % 100 == 0) {
            LOG_DEBUG("OpenVINS", "IMU samples processed: {}, acc: [{}, {}, {}]",
                      imuCount_, imu.accX(), imu.accY(), imu.accZ());
        }
    }

    /**
     * @brief Get current pose estimate
     */
    bool getPose(Pose6DoF& pose) const {
        if (!initialized_ || imuCount_ < 10) {
            return false;
        }

        pose.timestampNs = lastImu_.timestampNs;

        // Simulate slight movement based on accumulated IMU
        static double x = 0.0, y = 0.0, z = 0.0;
        x += lastImu_.accX() * 0.00001;
        y += lastImu_.accY() * 0.00001;
        z += lastImu_.accZ() * 0.00001;

        pose.position = Eigen::Vector3d(x, y, z);
        pose.orientation = Eigen::Quaterniond::Identity();
        pose.valid = true;
        return true;
    }

    int getTrackedFeatureCount() const {
        return trackCount_;
    }

    bool isInitialized() const {
        return initialized_ && imuCount_ > 0;
    }

    void reset() {
        LOG_INFO("OpenVINS", "Resetting VioManager state");
        imuCount_ = 0;
        trackCount_ = 0;
    }

    void shutdown() {
        LOG_INFO("OpenVINS", "Shutting down VioManager");
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

    LOG_INFO("OpenVINS", "Initializing adapter with config: {}", configPath);

    configPath_ = configPath;

    if (!loadConfiguration(configPath)) {
        LOG_ERROR("OpenVINS", "Failed to load configuration from: {}", configPath);
        return false;
    }

    updateStatus(TrackingStatus::INITIALIZING);
    initialized_ = true;

    LOG_INFO("OpenVINS", "Adapter initialized successfully");
    return true;
}

bool OpenVINSAdapter::loadCalibration(const std::string& calibPath) {
    std::lock_guard<std::mutex> lock(statusMutex_);

    LOG_INFO("OpenVINS", "Loading calibration from: {}", calibPath);

    calibPath_ = calibPath;

    if (!loadCalibrationData(calibPath)) {
        LOG_ERROR("OpenVINS", "Failed to load calibration from: {}", calibPath);
        return false;
    }

    calibrated_ = true;

    if (initialized_ && !configPath_.empty()) {
        if (!vioManager_->initialize(configPath_, calibPath_, config_)) {
            LOG_ERROR("OpenVINS", "Failed to initialize VioManager");
            updateStatus(TrackingStatus::UNINITIALIZED);
            return false;
        }
        LOG_INFO("OpenVINS", "VioManager ready for data");
    }

    LOG_INFO("OpenVINS", "Calibration loaded successfully");
    return true;
}

void OpenVINSAdapter::processImage(const cv::Mat& image, int64_t timestampNs) {
    if (!initialized_) {
        LOG_WARN("OpenVINS", "Adapter not initialized");
        return;
    }

    if (!calibrated_) {
        LOG_WARN("OpenVINS", "Calibration not loaded");
        return;
    }

    if (image.empty()) {
        LOG_WARN("OpenVINS", "Received empty image");
        return;
    }

    if (!checkImuInitialization(timestampNs)) {
        LOG_DEBUG("OpenVINS", "Waiting for IMU initialization data...");
        return;
    }

    processIMUQueue();

    vioManager_->feedImage(image, timestampNs);
    lastImageTimestampNs_ = timestampNs;

    trackedFeatureCount_ = vioManager_->getTrackedFeatureCount();

    Pose6DoF pose;
    if (vioManager_->getPose(pose)) {
        updatePose(pose);
        if (status_ != TrackingStatus::TRACKING) {
            updateStatus(TrackingStatus::TRACKING);
            LOG_INFO("OpenVINS", "Tracking established");
        }
    } else {
        if (status_ == TrackingStatus::TRACKING) {
            updateStatus(TrackingStatus::LOST);
            LOG_WARN("OpenVINS", "Tracking lost");
        }
    }
}

void OpenVINSAdapter::processIMU(const IMUSample& imu) {
    if (!initialized_) {
        return;
    }

    if (!imu.acceleration.allFinite() || !imu.angularVelocity.allFinite()) {
        LOG_WARN("OpenVINS", "Received invalid IMU data (NaN or Inf values)");
        return;
    }

    constexpr double MAX_ACC = 100.0;
    constexpr double MAX_GYRO = 10.0;
    if (imu.acceleration.cwiseAbs().maxCoeff() > MAX_ACC ||
        imu.angularVelocity.cwiseAbs().maxCoeff() > MAX_GYRO) {
        LOG_WARN("OpenVINS", "IMU values out of reasonable range");
        return;
    }

    {
        std::lock_guard<std::mutex> lock(imuMutex_);

        if (initStartTimeNs_ == 0) {
            initStartTimeNs_ = imu.timestampNs;
        }

        imuBuffer_.push_back(imu);
        lastImuTimestampNs_ = imu.timestampNs;

        while (imuBuffer_.size() > MAX_IMU_BUFFER_SIZE) {
            imuBuffer_.pop_front();
        }
    }

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
    return std::vector<MapPoint>();
}

void OpenVINSAdapter::reset() {
    std::lock_guard<std::mutex> lock(statusMutex_);

    LOG_INFO("OpenVINS", "Resetting adapter");

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
    LOG_INFO("OpenVINS", "Adapter reset complete");
}

void OpenVINSAdapter::shutdown() {
    std::lock_guard<std::mutex> lock(statusMutex_);

    LOG_INFO("OpenVINS", "Shutting down adapter");

    if (vioManager_) {
        vioManager_->shutdown();
    }

    initialized_ = false;
    calibrated_ = false;
    updateStatus(TrackingStatus::UNINITIALIZED);
    LOG_INFO("OpenVINS", "Adapter shut down");
}

OpenVINSConfig OpenVINSAdapter::getConfig() const {
    return config_;
}

bool OpenVINSAdapter::isReadyForVision() const {
    std::lock_guard<std::mutex> lock(imuMutex_);

    if (initStartTimeNs_ == 0 || imuBuffer_.empty()) {
        return false;
    }

    double elapsedSeconds = (lastImuTimestampNs_ - initStartTimeNs_) * 1e-9;
    return elapsedSeconds >= config_.imuInitWindow && imuInitialized_;
}

int OpenVINSAdapter::getTrackedFeatureCount() const {
    return trackedFeatureCount_.load();
}

bool OpenVINSAdapter::loadConfiguration(const std::string& configPath) {
    std::ifstream configFile(configPath);
    if (!configFile.is_open()) {
        LOG_ERROR("OpenVINS", "Failed to open config file: {}", configPath);
        return false;
    }

    if (!parseYamlConfig(configPath)) {
        LOG_DEBUG("OpenVINS", "Using default configuration parameters");
    }

    LOG_DEBUG("OpenVINS", "Configuration loaded from: {}", configPath);
    return true;
}

bool OpenVINSAdapter::loadCalibrationData(const std::string& calibPath) {
    std::ifstream calibFile(calibPath);
    if (!calibFile.is_open()) {
        LOG_ERROR("OpenVINS", "Failed to open calibration file: {}", calibPath);
        return false;
    }

    LOG_DEBUG("OpenVINS", "Calibration loaded from: {}", calibPath);
    return true;
}

bool OpenVINSAdapter::parseYamlConfig(const std::string& configPath) {
    std::ifstream file(configPath);
    if (!file.is_open()) {
        return false;
    }

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }

        size_t colonPos = line.find(':');
        if (colonPos == std::string::npos) {
            continue;
        }

        std::string key = line.substr(0, colonPos);
        std::string value = line.substr(colonPos + 1);

        key.erase(0, key.find_first_not_of(" \t"));
        key.erase(key.find_last_not_of(" \t") + 1);
        value.erase(0, value.find_first_not_of(" \t"));
        value.erase(value.find_last_not_of(" \t") + 1);

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

    const char* statusStr = "UNKNOWN";
    switch (newStatus) {
        case TrackingStatus::UNINITIALIZED: statusStr = "UNINITIALIZED"; break;
        case TrackingStatus::INITIALIZING: statusStr = "INITIALIZING"; break;
        case TrackingStatus::TRACKING: statusStr = "TRACKING"; break;
        case TrackingStatus::LOST: statusStr = "LOST"; break;
        case TrackingStatus::RELOCALIZATION: statusStr = "RELOCALIZATION"; break;
    }
    LOG_DEBUG("OpenVINS", "Status: {}", statusStr);
}

void OpenVINSAdapter::updatePose(const Pose6DoF& pose) {
    std::lock_guard<std::mutex> lock(poseMutex_);
    latestPose_ = pose;
}

void OpenVINSAdapter::processIMUQueue() {
    std::lock_guard<std::mutex> lock(imuMutex_);

    while (!imuBuffer_.empty()) {
        const IMUSample& imu = imuBuffer_.front();

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

    double elapsedSeconds = (imageTimestampNs - initStartTimeNs_) * 1e-9;

    if (elapsedSeconds >= config_.imuInitWindow) {
        imuInitialized_ = true;
        LOG_INFO("OpenVINS", "IMU initialization complete after {} seconds",
                 elapsedSeconds);
        return true;
    }

    return false;
}

}  // namespace vi_slam
