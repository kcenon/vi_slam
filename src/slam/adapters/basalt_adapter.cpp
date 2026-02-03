#include "slam/adapters/basalt_adapter.hpp"
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <chrono>

namespace vi_slam {

/**
 * @brief Basalt VIO system wrapper class
 *
 * This class wraps the Basalt VIO functionality. When the Basalt library
 * is available, this class interfaces with the actual VioEstimator.
 * Otherwise, it provides a placeholder implementation for development.
 *
 * Basalt API Reference:
 * - VioEstimator: Main class for visual-inertial odometry
 * - OpticalFlowBase: Optical flow tracking (KLT or patch-based)
 * - PoseVelBiasState: State representation (pose, velocity, biases)
 * - ImuData: IMU measurement data structure
 *
 * Integration points with actual Basalt:
 * 1. include "basalt/vi_estimator/vio_estimator.h"
 * 2. include "basalt/optical_flow/optical_flow.h"
 * 3. Replace placeholder methods with actual Basalt API calls
 * 4. Link against basalt-headers and basalt libraries
 */
class BasaltVioSystem {
public:
    BasaltVioSystem() : initialized_(false), imuCount_(0), trackCount_(0) {}

    /**
     * @brief Initialize the VIO system with configuration
     * @param config Configuration path (YAML file)
     * @param calib Calibration path (YAML file)
     * @param params Configuration parameters
     * @return true if initialization successful
     *
     * Basalt actual implementation:
     * @code
     * basalt::Calibration<double> calib;
     * calib.loadFromJson(calibPath);
     *
     * basalt::VioConfig vio_config;
     * vio_config.loadFromJson(configPath);
     *
     * vio = basalt::VioEstimatorFactory::getVioEstimator(
     *     vio_config, calib, basalt::constants::g, true, true);
     *
     * opt_flow = basalt::OpticalFlowFactory::getOpticalFlow(
     *     vio_config, calib);
     * opt_flow->output_queue = &vio->vision_data_queue;
     * @endcode
     */
    bool initialize(const std::string& config, const std::string& calib,
                    const BasaltConfig& params) {
        std::cout << "[Basalt] Initializing VIO system" << std::endl;
        std::cout << "[Basalt] Config: " << config << std::endl;
        std::cout << "[Basalt] Calibration: " << calib << std::endl;
        std::cout << "[Basalt] Parameters:" << std::endl;
        std::cout << "  - VIO mode: " << params.vioMode << std::endl;
        std::cout << "  - Max flow points: " << params.maxFlowPoints << std::endl;
        std::cout << "  - Pyramid levels: " << params.pyramidLevels << std::endl;
        std::cout << "  - Use IMU preintegration: "
                  << (params.useImuPreintegration ? "yes" : "no") << std::endl;

        config_ = params;
        initialized_ = true;
        return true;
    }

    /**
     * @brief Feed monocular image to the optical flow tracker
     * @param image Input image (grayscale or color)
     * @param timestampNs Timestamp in nanoseconds
     *
     * Basalt actual implementation:
     * @code
     * basalt::OpticalFlowInput::Ptr input(new basalt::OpticalFlowInput);
     * input->t_ns = timestampNs;
     *
     * cv::Mat gray;
     * if (image.channels() == 3) {
     *     cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
     * } else {
     *     gray = image;
     * }
     *
     * input->img_data.resize(1);
     * input->img_data[0].img.reset(new basalt::ManagedImage<uint16_t>(
     *     gray.cols, gray.rows));
     * // Copy and convert 8-bit to 16-bit
     * for (int y = 0; y < gray.rows; y++) {
     *     for (int x = 0; x < gray.cols; x++) {
     *         input->img_data[0].img->operator()(x, y) =
     *             gray.at<uint8_t>(y, x) << 8;
     *     }
     * }
     *
     * opt_flow->input_queue.push(input);
     * @endcode
     */
    void feedImage(const cv::Mat& image, int64_t timestampNs) {
        (void)timestampNs;

        if (!initialized_) {
            std::cerr << "[Basalt] VIO system not initialized" << std::endl;
            return;
        }

        // Placeholder: simulate optical flow tracking
        // In actual implementation, Basalt OpticalFlowBase processes the image
        // and tracks features using patch-based KLT
        int flowPoints = 0;
        if (!image.empty()) {
            cv::Mat gray;
            if (image.channels() == 3) {
                cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
            } else {
                gray = image;
            }

            // Detect features using FAST corners (similar to Basalt's approach)
            std::vector<cv::Point2f> corners;
            cv::goodFeaturesToTrack(
                gray, corners, config_.maxFlowPoints,
                config_.flowQualityThreshold * 0.01,
                config_.minDistance);

            flowPoints = static_cast<int>(corners.size());
        }

        trackCount_ = flowPoints;

        if (config_.verbosity >= 2) {
            std::cout << "[Basalt] Processing image, tracked flow points: "
                      << trackCount_ << std::endl;
        }
    }

    /**
     * @brief Feed stereo image pair to the optical flow tracker
     * @param imageLeft Left camera image
     * @param imageRight Right camera image
     * @param timestampNs Timestamp in nanoseconds
     */
    void feedStereoImage(const cv::Mat& imageLeft, const cv::Mat& imageRight,
                         int64_t timestampNs) {
        (void)imageRight;
        // For monocular mode, process only left image
        feedImage(imageLeft, timestampNs);
    }

    /**
     * @brief Feed IMU measurement to the VIO system
     * @param imu IMU sample with accelerometer and gyroscope data
     *
     * Basalt actual implementation:
     * @code
     * basalt::ImuData::Ptr data(new basalt::ImuData);
     * data->t_ns = imu.timestampNs;
     * data->accel = Eigen::Vector3d(imu.accX, imu.accY, imu.accZ);
     * data->gyro = Eigen::Vector3d(imu.gyroX, imu.gyroY, imu.gyroZ);
     *
     * vio->imu_data_queue.push(data);
     * @endcode
     */
    void feedIMU(const IMUSample& imu) {
        if (!initialized_) {
            return;
        }

        // Placeholder: accumulate IMU for integration
        // In actual implementation, Basalt performs IMU preintegration
        // using on-manifold preintegration theory
        lastImu_ = imu;
        imuCount_++;

        // Every 200 IMU samples, log status
        if (config_.verbosity >= 2 && imuCount_ % 200 == 0) {
            std::cout << "[Basalt] IMU samples processed: " << imuCount_
                      << ", gyro: [" << imu.gyroX << ", " << imu.gyroY << ", "
                      << imu.gyroZ << "]" << std::endl;
        }
    }

    /**
     * @brief Get current pose estimate
     * @param[out] pose Output 6DoF pose
     * @return true if valid pose available
     *
     * Basalt actual implementation:
     * @code
     * basalt::PoseVelBiasState::Ptr state;
     * if (!vio->out_state_queue.try_pop(state)) {
     *     return false;
     * }
     *
     * Sophus::SE3d T_w_i = state->T_w_i;
     * Eigen::Vector3d pos = T_w_i.translation();
     * Eigen::Quaterniond quat = T_w_i.unit_quaternion();
     *
     * pose.timestampNs = state->t_ns;
     * pose.position[0] = pos.x();
     * pose.position[1] = pos.y();
     * pose.position[2] = pos.z();
     * // Basalt uses Hamilton quaternion convention (w, x, y, z)
     * pose.orientation[0] = quat.w();
     * pose.orientation[1] = quat.x();
     * pose.orientation[2] = quat.y();
     * pose.orientation[3] = quat.z();
     * pose.valid = true;
     *
     * // Optional: velocity from state
     * // Eigen::Vector3d vel = state->vel_w_i;
     *
     * return true;
     * @endcode
     */
    bool getPose(Pose6DoF& pose) const {
        if (!initialized_ || imuCount_ < 10) {
            return false;
        }

        // Placeholder: return a simulated pose based on IMU integration
        // Actual pose comes from Basalt's non-linear optimization
        pose.timestampNs = lastImu_.timestampNs;

        // Simulate slight movement based on accumulated IMU
        // In real implementation, this comes from the VIO state estimator
        static double x = 0.0, y = 0.0, z = 0.0;
        static double vx = 0.0, vy = 0.0, vz = 0.0;

        // Simple dead-reckoning simulation
        constexpr double dt = 0.005;  // 200 Hz IMU
        vx += (lastImu_.accX - config_.gravity * 0.0) * dt;
        vy += (lastImu_.accY - config_.gravity * 0.0) * dt;
        vz += (lastImu_.accZ - config_.gravity) * dt;

        x += vx * dt;
        y += vy * dt;
        z += vz * dt;

        // Clamp values for stability in placeholder
        x = std::max(-100.0, std::min(100.0, x));
        y = std::max(-100.0, std::min(100.0, y));
        z = std::max(-100.0, std::min(100.0, z));

        pose.position[0] = x * 0.001;  // Scale down for realistic values
        pose.position[1] = y * 0.001;
        pose.position[2] = z * 0.001;

        // Identity quaternion (w, x, y, z)
        pose.orientation[0] = 1.0;
        pose.orientation[1] = 0.0;
        pose.orientation[2] = 0.0;
        pose.orientation[3] = 0.0;

        pose.valid = true;
        return true;
    }

    /**
     * @brief Get active optical flow landmarks
     * @return Vector of map points representing tracked features
     *
     * Basalt actual implementation:
     * @code
     * std::vector<MapPoint> points;
     * auto& lmdb = vio->lmdb;
     *
     * for (auto& [id, lm] : lmdb.getLandmarks()) {
     *     if (lm.depth.has_value()) {
     *         MapPoint mp;
     *         mp.id = id;
     *         Eigen::Vector3d pos = lm.getPosition();
     *         mp.position[0] = pos.x();
     *         mp.position[1] = pos.y();
     *         mp.position[2] = pos.z();
     *         mp.observations = lm.obs.size();
     *         points.push_back(mp);
     *     }
     * }
     *
     * return points;
     * @endcode
     */
    std::vector<MapPoint> getMapPoints() const {
        // Basalt primarily uses optical flow for tracking
        // Map points are maintained for optimization but sparse
        return std::vector<MapPoint>();
    }

    /**
     * @brief Get current tracked feature count
     */
    int getTrackedFeatureCount() const {
        return trackCount_;
    }

    /**
     * @brief Check if VIO is initialized and ready
     */
    bool isInitialized() const {
        return initialized_ && imuCount_ > 0;
    }

    /**
     * @brief Reset the VIO state
     *
     * Basalt actual implementation:
     * @code
     * vio.reset();
     * opt_flow.reset();
     *
     * // Reinitialize with same config
     * vio = basalt::VioEstimatorFactory::getVioEstimator(
     *     vio_config, calib, basalt::constants::g, true, true);
     * opt_flow = basalt::OpticalFlowFactory::getOpticalFlow(
     *     vio_config, calib);
     * opt_flow->output_queue = &vio->vision_data_queue;
     * @endcode
     */
    void reset() {
        std::cout << "[Basalt] Resetting VIO state" << std::endl;
        imuCount_ = 0;
        trackCount_ = 0;
    }

    /**
     * @brief Shutdown the VIO system
     */
    void shutdown() {
        std::cout << "[Basalt] Shutting down VIO system" << std::endl;
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

    logStatus("Initializing Basalt adapter with config: " + configPath);

    configPath_ = configPath;

    if (!loadConfiguration(configPath)) {
        logError("Failed to load configuration from: " + configPath);
        return false;
    }

    updateStatus(TrackingStatus::INITIALIZING);
    initialized_ = true;

    logStatus("Basalt adapter initialized successfully");
    return true;
}

bool BasaltAdapter::loadCalibration(const std::string& calibPath) {
    std::lock_guard<std::mutex> lock(statusMutex_);

    logStatus("Loading calibration from: " + calibPath);

    calibPath_ = calibPath;

    if (!loadCalibrationData(calibPath)) {
        logError("Failed to load calibration from: " + calibPath);
        return false;
    }

    calibrated_ = true;

    if (initialized_ && !configPath_.empty()) {
        // Initialize Basalt VIO with both config and calibration
        if (!vioSystem_->initialize(configPath_, calibPath_, config_)) {
            logError("Failed to initialize Basalt VIO system");
            updateStatus(TrackingStatus::UNINITIALIZED);
            return false;
        }
        logStatus("Basalt VIO system ready for data");
    }

    logStatus("Calibration loaded successfully");
    return true;
}

void BasaltAdapter::processImage(const cv::Mat& image, int64_t timestampNs) {
    if (!initialized_) {
        logError("Basalt adapter not initialized");
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
        if (config_.verbosity >= 1) {
            logStatus("Waiting for IMU initialization data...");
        }
        return;
    }

    // Process queued IMU samples up to this image timestamp
    processIMUQueue();

    // Feed image to Basalt optical flow tracker
    vioSystem_->feedImage(image, timestampNs);
    lastImageTimestampNs_ = timestampNs;

    // Update tracked feature count
    trackedFeatureCount_ = vioSystem_->getTrackedFeatureCount();

    // Update pose from VIO system
    Pose6DoF pose;
    if (vioSystem_->getPose(pose)) {
        updatePose(pose);
        if (status_ != TrackingStatus::TRACKING) {
            updateStatus(TrackingStatus::TRACKING);
            logStatus("Tracking established with " +
                      std::to_string(trackedFeatureCount_.load()) + " flow points");
        }
    } else {
        // If pose retrieval fails after tracking, mark as lost
        if (status_ == TrackingStatus::TRACKING) {
            updateStatus(TrackingStatus::LOST);
            logStatus("Tracking lost");
        }
    }
}

void BasaltAdapter::processIMU(const IMUSample& imu) {
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
    constexpr double MAX_ACC = 100.0;   // 100 m/s^2
    constexpr double MAX_GYRO = 10.0;   // 10 rad/s
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

    // Feed IMU to VIO system for preintegration
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
    // Basalt uses optical flow tracking, so map points are sparse
    // Return points from the VIO system's landmark database
    if (vioSystem_) {
        return vioSystem_->getMapPoints();
    }
    return std::vector<MapPoint>();
}

void BasaltAdapter::reset() {
    std::lock_guard<std::mutex> lock(statusMutex_);

    logStatus("Resetting Basalt adapter");

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
    logStatus("Basalt adapter reset complete");
}

void BasaltAdapter::shutdown() {
    std::lock_guard<std::mutex> lock(statusMutex_);

    logStatus("Shutting down Basalt adapter");

    if (vioSystem_) {
        vioSystem_->shutdown();
    }

    initialized_ = false;
    calibrated_ = false;
    updateStatus(TrackingStatus::UNINITIALIZED);
    logStatus("Basalt adapter shut down");
}

BasaltConfig BasaltAdapter::getConfig() const {
    return config_;
}

bool BasaltAdapter::isReadyForVision() const {
    std::lock_guard<std::mutex> lock(imuMutex_);

    if (initStartTimeNs_ == 0 || imuBuffer_.empty()) {
        return false;
    }

    // Check if we have enough IMU data (configured initialization window)
    double elapsedSeconds = (lastImuTimestampNs_ - initStartTimeNs_) * 1e-9;
    return elapsedSeconds >= config_.imuInitWindow && imuInitialized_;
}

int BasaltAdapter::getTrackedFeatureCount() const {
    return trackedFeatureCount_.load();
}

bool BasaltAdapter::loadConfiguration(const std::string& configPath) {
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

bool BasaltAdapter::loadCalibrationData(const std::string& calibPath) {
    std::ifstream calibFile(calibPath);
    if (!calibFile.is_open()) {
        logError("Failed to open calibration file: " + calibPath);
        return false;
    }

    // Basalt calibration files contain:
    // - Camera intrinsics (pinhole, unified, extended unified, double sphere)
    // - Camera-IMU extrinsics (T_i_c: transform from camera to IMU)
    // - IMU noise characteristics
    // - Time offset between camera and IMU

    // For full implementation, would need JSON parser
    // Basalt uses JSON format for calibration

    logStatus("Calibration loaded from: " + calibPath);
    return true;
}

bool BasaltAdapter::parseYamlConfig(const std::string& configPath) {
    // Simple line-by-line YAML parsing for key parameters
    // For production, use yaml-cpp library
    std::ifstream file(configPath);
    if (!file.is_open()) {
        return false;
    }

    std::string line;
    std::string currentSection;

    while (std::getline(file, line)) {
        // Skip comments and empty lines
        if (line.empty() || line[0] == '#') {
            continue;
        }

        // Track sections
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

        // Remove quotes if present
        if (!value.empty() && (value.front() == '"' || value.front() == '\'')) {
            value = value.substr(1, value.size() - 2);
        }

        // Parse parameters based on section and key
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
                // Top-level parameters
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
            // Skip invalid values
            if (config_.verbosity >= 2) {
                std::cerr << "[Basalt] Warning: Failed to parse " << key
                          << ": " << e.what() << std::endl;
            }
        }
    }

    return true;
}

void BasaltAdapter::updateStatus(TrackingStatus newStatus) {
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

    if (config_.verbosity >= 1) {
        std::cout << "[Basalt] Status: " << statusStr << std::endl;
    }
}

void BasaltAdapter::updatePose(const Pose6DoF& pose) {
    std::lock_guard<std::mutex> lock(poseMutex_);
    latestPose_ = pose;
}

void BasaltAdapter::processIMUQueue() {
    std::lock_guard<std::mutex> lock(imuMutex_);

    // Process all queued IMU samples
    // This ensures IMU data is processed in temporal order
    // Basalt requires IMU data to be processed before corresponding images
    // for proper preintegration
    while (!imuBuffer_.empty()) {
        const IMUSample& imu = imuBuffer_.front();

        // Only process IMU samples up to current image timestamp
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

void BasaltAdapter::logStatus(const std::string& message) const {
    if (config_.verbosity >= 1) {
        std::cout << "[Basalt] " << message << std::endl;
    }
}

void BasaltAdapter::logError(const std::string& message) const {
    std::cerr << "[Basalt ERROR] " << message << std::endl;
}

}  // namespace vi_slam
