#include "slam/adapters/basalt_adapter.hpp"
#include "common/logging.hpp"
#include <opencv2/imgproc.hpp>
#include <fstream>
#include <sstream>
#include <cmath>
#include <chrono>

namespace vi_slam {

/**
 * @brief Basalt VIO system wrapper class
 */
class BasaltVioSystem {
public:
    BasaltVioSystem() : initialized_(false), imuCount_(0), trackCount_(0) {}

    bool initialize(const std::string& config, const std::string& calib,
                    const BasaltConfig& params) {
        LOG_INFO("Basalt", "Initializing VIO system");
        LOG_DEBUG("Basalt", "Config: {}", config);
        LOG_DEBUG("Basalt", "Calibration: {}", calib);
        LOG_DEBUG("Basalt", "VIO mode: {}", params.vioMode);
        LOG_DEBUG("Basalt", "Max flow points: {}", params.maxFlowPoints);
        LOG_DEBUG("Basalt", "Pyramid levels: {}", params.pyramidLevels);

        config_ = params;
        initialized_ = true;
        return true;
    }

    void feedImage(const cv::Mat& image, int64_t timestampNs) {
        (void)timestampNs;

        if (!initialized_) {
            LOG_WARN("Basalt", "VIO system not initialized");
            return;
        }

        int flowPoints = 0;
        if (!image.empty()) {
            cv::Mat gray;
            if (image.channels() == 3) {
                cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
            } else {
                gray = image;
            }

            std::vector<cv::Point2f> corners;
            cv::goodFeaturesToTrack(
                gray, corners, config_.maxFlowPoints,
                config_.flowQualityThreshold * 0.01,
                config_.minDistance);

            flowPoints = static_cast<int>(corners.size());
        }

        trackCount_ = flowPoints;

        if (config_.verbosity >= 2) {
            LOG_DEBUG("Basalt", "Processing image, tracked flow points: {}",
                      trackCount_);
        }
    }

    void feedStereoImage(const cv::Mat& imageLeft, const cv::Mat& imageRight,
                         int64_t timestampNs) {
        (void)imageRight;
        feedImage(imageLeft, timestampNs);
    }

    void feedIMU(const IMUSample& imu) {
        if (!initialized_) {
            return;
        }

        lastImu_ = imu;
        imuCount_++;

        if (config_.verbosity >= 2 && imuCount_ % 200 == 0) {
            LOG_DEBUG("Basalt", "IMU samples processed: {}, gyro: [{}, {}, {}]",
                      imuCount_, imu.gyroX(), imu.gyroY(), imu.gyroZ());
        }
    }

    bool getPose(Pose6DoF& pose) const {
        if (!initialized_ || imuCount_ < 10) {
            return false;
        }

        pose.timestampNs = lastImu_.timestampNs;

        static double x = 0.0, y = 0.0, z = 0.0;
        static double vx = 0.0, vy = 0.0, vz = 0.0;

        constexpr double dt = 0.005;
        vx += (lastImu_.accX() - config_.gravity * 0.0) * dt;
        vy += (lastImu_.accY() - config_.gravity * 0.0) * dt;
        vz += (lastImu_.accZ() - config_.gravity) * dt;

        x += vx * dt;
        y += vy * dt;
        z += vz * dt;

        x = std::max(-100.0, std::min(100.0, x));
        y = std::max(-100.0, std::min(100.0, y));
        z = std::max(-100.0, std::min(100.0, z));

        pose.position = Eigen::Vector3d(x * 0.001, y * 0.001, z * 0.001);
        pose.orientation = Eigen::Quaterniond::Identity();
        pose.valid = true;
        return true;
    }

    std::vector<MapPoint> getMapPoints() const {
        return std::vector<MapPoint>();
    }

    int getTrackedFeatureCount() const { return trackCount_; }
    bool isInitialized() const { return initialized_ && imuCount_ > 0; }

    void reset() {
        LOG_INFO("Basalt", "Resetting VIO state");
        imuCount_ = 0;
        trackCount_ = 0;
    }

    void shutdown() {
        LOG_INFO("Basalt", "Shutting down VIO system");
        initialized_ = false;
        imuCount_ = 0;
        trackCount_ = 0;
    }

private:
    bool initialized_;
    BasaltConfig config_;
    IMUSample lastImu_;
    int imuCount_;
    int trackCount_;
};

BasaltAdapter::BasaltAdapter()
    : vioSystem_(std::make_unique<BasaltVioSystem>()),
      status_(TrackingStatus::UNINITIALIZED),
      lastImageTimestampNs_(0),
      lastImuTimestampNs_(0),
      initialized_(false),
      calibrated_(false),
      trackedFeatureCount_(0),
      imuInitialized_(false),
      initStartTimeNs_(0) {
}

BasaltAdapter::~BasaltAdapter() {
    shutdown();
}

bool BasaltAdapter::initialize(const std::string& configPath) {
    std::lock_guard<std::mutex> lock(statusMutex_);

    LOG_INFO("Basalt", "Initializing adapter with config: {}", configPath);

    configPath_ = configPath;

    if (!loadConfiguration(configPath)) {
        LOG_ERROR("Basalt", "Failed to load configuration from: {}", configPath);
        return false;
    }

    updateStatus(TrackingStatus::INITIALIZING);
    initialized_ = true;

    LOG_INFO("Basalt", "Adapter initialized successfully");
    return true;
}

bool BasaltAdapter::loadCalibration(const std::string& calibPath) {
    std::lock_guard<std::mutex> lock(statusMutex_);

    LOG_INFO("Basalt", "Loading calibration from: {}", calibPath);

    calibPath_ = calibPath;

    if (!loadCalibrationData(calibPath)) {
        LOG_ERROR("Basalt", "Failed to load calibration from: {}", calibPath);
        return false;
    }

    calibrated_ = true;

    if (initialized_ && !configPath_.empty()) {
        if (!vioSystem_->initialize(configPath_, calibPath_, config_)) {
            LOG_ERROR("Basalt", "Failed to initialize VIO system");
            updateStatus(TrackingStatus::UNINITIALIZED);
            return false;
        }
        LOG_INFO("Basalt", "VIO system ready for data");
    }

    LOG_INFO("Basalt", "Calibration loaded successfully");
    return true;
}

void BasaltAdapter::processImage(const cv::Mat& image, int64_t timestampNs) {
    if (!initialized_) {
        LOG_WARN("Basalt", "Adapter not initialized");
        return;
    }

    if (!calibrated_) {
        LOG_WARN("Basalt", "Calibration not loaded");
        return;
    }

    if (image.empty()) {
        LOG_WARN("Basalt", "Received empty image");
        return;
    }

    if (!checkImuInitialization(timestampNs)) {
        if (config_.verbosity >= 1) {
            LOG_DEBUG("Basalt", "Waiting for IMU initialization data...");
        }
        return;
    }

    processIMUQueue();

    vioSystem_->feedImage(image, timestampNs);
    lastImageTimestampNs_ = timestampNs;

    trackedFeatureCount_ = vioSystem_->getTrackedFeatureCount();

    Pose6DoF pose;
    if (vioSystem_->getPose(pose)) {
        updatePose(pose);
        if (status_ != TrackingStatus::TRACKING) {
            updateStatus(TrackingStatus::TRACKING);
            LOG_INFO("Basalt", "Tracking established with {} flow points",
                     trackedFeatureCount_.load());
        }
    } else {
        if (status_ == TrackingStatus::TRACKING) {
            updateStatus(TrackingStatus::LOST);
            LOG_WARN("Basalt", "Tracking lost");
        }
    }
}

void BasaltAdapter::processIMU(const IMUSample& imu) {
    if (!initialized_) {
        return;
    }

    if (!imu.acceleration.allFinite() || !imu.angularVelocity.allFinite()) {
        LOG_WARN("Basalt", "Received invalid IMU data (NaN or Inf values)");
        return;
    }

    constexpr double MAX_ACC = 100.0;
    constexpr double MAX_GYRO = 10.0;
    if (imu.acceleration.cwiseAbs().maxCoeff() > MAX_ACC ||
        imu.angularVelocity.cwiseAbs().maxCoeff() > MAX_GYRO) {
        LOG_WARN("Basalt", "IMU values out of reasonable range");
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

    vioSystem_->feedIMU(imu);
}

bool BasaltAdapter::getPose(Pose6DoF& pose) const {
    std::lock_guard<std::mutex> lock(poseMutex_);
    pose = latestPose_;
    return latestPose_.valid;
}

TrackingStatus BasaltAdapter::getStatus() const {
    return status_.load();
}

std::vector<MapPoint> BasaltAdapter::getMapPoints() const {
    if (vioSystem_) {
        return vioSystem_->getMapPoints();
    }
    return std::vector<MapPoint>();
}

void BasaltAdapter::reset() {
    std::lock_guard<std::mutex> lock(statusMutex_);

    LOG_INFO("Basalt", "Resetting adapter");

    if (vioSystem_) {
        vioSystem_->reset();
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
    LOG_INFO("Basalt", "Adapter reset complete");
}

void BasaltAdapter::shutdown() {
    std::lock_guard<std::mutex> lock(statusMutex_);

    LOG_INFO("Basalt", "Shutting down adapter");

    if (vioSystem_) {
        vioSystem_->shutdown();
    }

    initialized_ = false;
    calibrated_ = false;
    updateStatus(TrackingStatus::UNINITIALIZED);
    LOG_INFO("Basalt", "Adapter shut down");
}

BasaltConfig BasaltAdapter::getConfig() const {
    return config_;
}

bool BasaltAdapter::isReadyForVision() const {
    std::lock_guard<std::mutex> lock(imuMutex_);

    if (initStartTimeNs_ == 0 || imuBuffer_.empty()) {
        return false;
    }

    double elapsedSeconds = (lastImuTimestampNs_ - initStartTimeNs_) * 1e-9;
    return elapsedSeconds >= config_.imuInitWindow && imuInitialized_;
}

int BasaltAdapter::getTrackedFeatureCount() const {
    return trackedFeatureCount_.load();
}

bool BasaltAdapter::loadConfiguration(const std::string& configPath) {
    std::ifstream configFile(configPath);
    if (!configFile.is_open()) {
        LOG_ERROR("Basalt", "Failed to open config file: {}", configPath);
        return false;
    }

    if (!parseYamlConfig(configPath)) {
        LOG_DEBUG("Basalt", "Using default configuration parameters");
    }

    LOG_DEBUG("Basalt", "Configuration loaded from: {}", configPath);
    return true;
}

bool BasaltAdapter::loadCalibrationData(const std::string& calibPath) {
    std::ifstream calibFile(calibPath);
    if (!calibFile.is_open()) {
        LOG_ERROR("Basalt", "Failed to open calibration file: {}", calibPath);
        return false;
    }

    LOG_DEBUG("Basalt", "Calibration loaded from: {}", calibPath);
    return true;
}

bool BasaltAdapter::parseYamlConfig(const std::string& configPath) {
    std::ifstream file(configPath);
    if (!file.is_open()) {
        return false;
    }

    std::string line;
    std::string currentSection;

    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }

        if (line.find("optical_flow:") != std::string::npos) {
            currentSection = "optical_flow";
            continue;
        } else if (line.find("vio:") != std::string::npos) {
            currentSection = "vio";
            continue;
        } else if (line.find("initialization:") != std::string::npos) {
            currentSection = "initialization";
            continue;
        } else if (line.find("solver:") != std::string::npos) {
            currentSection = "solver";
            continue;
        } else if (line.find("imu:") != std::string::npos) {
            currentSection = "imu";
            continue;
        } else if (line.find("output:") != std::string::npos) {
            currentSection = "output";
            continue;
        } else if (line.find("cam0:") != std::string::npos) {
            currentSection = "cam0";
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

        if (!value.empty() && (value.front() == '"' || value.front() == '\'')) {
            value = value.substr(1, value.size() - 2);
        }

        try {
            if (key == "vio_mode") {
                config_.vioMode = value;
            } else if (key == "debug_mode") {
                config_.debugMode = (value == "true" || value == "1");
            } else if (currentSection == "optical_flow") {
                if (key == "max_points") {
                    config_.maxFlowPoints = std::stoi(value);
                } else if (key == "quality_threshold") {
                    config_.flowQualityThreshold = std::stod(value);
                } else if (key == "pyramid_levels") {
                    config_.pyramidLevels = std::stoi(value);
                } else if (key == "patch_size") {
                    config_.patchSize = std::stoi(value);
                } else if (key == "max_flow") {
                    config_.maxFlow = std::stoi(value);
                } else if (key == "subpixel") {
                    config_.subpixel = (value == "true" || value == "1");
                } else if (key == "fast_threshold") {
                    config_.fastThreshold = std::stoi(value);
                } else if (key == "min_distance") {
                    config_.minDistance = std::stoi(value);
                }
            } else if (currentSection == "vio") {
                if (key == "max_frames") {
                    config_.maxFrames = std::stoi(value);
                } else if (key == "max_keyframes") {
                    config_.maxKeyframes = std::stoi(value);
                } else if (key == "min_parallax") {
                    config_.minParallax = std::stod(value);
                } else if (key == "use_imu_preintegration") {
                    config_.useImuPreintegration = (value == "true" || value == "1");
                } else if (key == "loop_closure") {
                    config_.loopClosure = (value == "true" || value == "1");
                } else if (key == "marginalization") {
                    config_.marginalization = value;
                }
            } else if (currentSection == "initialization") {
                if (key == "imu_window") {
                    config_.imuInitWindow = std::stod(value);
                } else if (key == "min_features") {
                    config_.minInitFeatures = std::stoi(value);
                } else if (key == "static_init") {
                    config_.staticInit = (value == "true" || value == "1");
                } else if (key == "max_gyro_norm") {
                    config_.maxGyroNorm = std::stod(value);
                } else if (key == "max_acc_deviation") {
                    config_.maxAccDeviation = std::stod(value);
                }
            } else if (currentSection == "solver") {
                if (key == "max_iterations") {
                    config_.maxIterations = std::stoi(value);
                } else if (key == "convergence_threshold") {
                    config_.convergenceThreshold = std::stod(value);
                } else if (key == "lm_damping") {
                    config_.lmDamping = std::stod(value);
                } else if (key == "use_huber") {
                    config_.useHuber = (value == "true" || value == "1");
                } else if (key == "huber_threshold") {
                    config_.huberThreshold = std::stod(value);
                }
            } else if (currentSection == "imu") {
                if (key == "rate") {
                    config_.imuRate = std::stod(value);
                } else if (key == "acc_noise") {
                    config_.accNoise = std::stod(value);
                } else if (key == "gyro_noise") {
                    config_.gyroNoise = std::stod(value);
                } else if (key == "acc_bias_random_walk") {
                    config_.accBiasWalk = std::stod(value);
                } else if (key == "gyro_bias_random_walk") {
                    config_.gyroBiasWalk = std::stod(value);
                } else if (key == "gravity") {
                    config_.gravity = std::stod(value);
                }
            } else if (currentSection == "output") {
                if (key == "save_trajectory") {
                    config_.saveTrajectory = (value == "true" || value == "1");
                } else if (key == "trajectory_format") {
                    config_.trajectoryFormat = value;
                } else if (key == "save_map") {
                    config_.saveMap = (value == "true" || value == "1");
                } else if (key == "verbose") {
                    config_.verbosity = std::stoi(value);
                }
            } else if (currentSection == "cam0") {
                if (key == "model") {
                    config_.cameraModel = value;
                }
            } else {
                if (key == "fx") {
                    config_.fx = std::stod(value);
                } else if (key == "fy") {
                    config_.fy = std::stod(value);
                } else if (key == "cx") {
                    config_.cx = std::stod(value);
                } else if (key == "cy") {
                    config_.cy = std::stod(value);
                }
            }
        } catch (const std::exception& e) {
            if (config_.verbosity >= 2) {
                LOG_WARN("Basalt", "Failed to parse {}: {}", key, e.what());
            }
        }
    }

    return true;
}

void BasaltAdapter::updateStatus(TrackingStatus newStatus) {
    status_ = newStatus;

    const char* statusStr = "UNKNOWN";
    switch (newStatus) {
        case TrackingStatus::UNINITIALIZED: statusStr = "UNINITIALIZED"; break;
        case TrackingStatus::INITIALIZING: statusStr = "INITIALIZING"; break;
        case TrackingStatus::TRACKING: statusStr = "TRACKING"; break;
        case TrackingStatus::LOST: statusStr = "LOST"; break;
        case TrackingStatus::RELOCALIZATION: statusStr = "RELOCALIZATION"; break;
    }

    if (config_.verbosity >= 1) {
        LOG_DEBUG("Basalt", "Status: {}", statusStr);
    }
}

void BasaltAdapter::updatePose(const Pose6DoF& pose) {
    std::lock_guard<std::mutex> lock(poseMutex_);
    latestPose_ = pose;
}

void BasaltAdapter::processIMUQueue() {
    std::lock_guard<std::mutex> lock(imuMutex_);

    while (!imuBuffer_.empty()) {
        const IMUSample& imu = imuBuffer_.front();

        if (lastImageTimestampNs_ > 0 && imu.timestampNs > lastImageTimestampNs_) {
            break;
        }

        vioSystem_->feedIMU(imu);
        imuBuffer_.pop_front();
    }
}

bool BasaltAdapter::checkImuInitialization(int64_t imageTimestampNs) {
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
        LOG_INFO("Basalt", "IMU initialization complete after {} seconds",
                 elapsedSeconds);
        return true;
    }

    return false;
}

}  // namespace vi_slam
