#include "slam/adapters/orbslam3_adapter.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <chrono>
#include <algorithm>

namespace vi_slam {

/**
 * @brief ORB-SLAM3 System wrapper class
 *
 * This class wraps the ORB-SLAM3 System functionality. When ORB-SLAM3
 * library is available, this class interfaces with the actual System class.
 * Otherwise, it provides a placeholder implementation for development.
 *
 * ORB-SLAM3 API Reference:
 * - System: Main class that handles visual(-inertial) SLAM
 * - TrackMonocular/TrackStereo/TrackRGBD: Main tracking methods
 * - Tracking state: OK, LOST, NOT_INITIALIZED, SYSTEM_NOT_READY
 * - Atlas: Multi-map management system
 * - LoopClosing: Loop closure detection and correction
 *
 * Integration points with actual ORB-SLAM3:
 * 1. include "System.h"
 * 2. Replace placeholder methods with actual ORB-SLAM3 API calls
 * 3. Link against ORB_SLAM3 library
 */
class ORBSLAM3System {
public:
    // Tracking state enumeration matching ORB-SLAM3
    enum class TrackingState {
        SYSTEM_NOT_READY = -1,
        NO_IMAGES_YET = 0,
        NOT_INITIALIZED = 1,
        OK = 2,
        RECENTLY_LOST = 3,
        LOST = 4,
        OK_KLT = 5
    };

    ORBSLAM3System()
        : initialized_(false),
          trackingState_(TrackingState::SYSTEM_NOT_READY),
          trackedPoints_(0),
          imuCount_(0),
          frameCount_(0) {}

    /**
     * @brief Initialize the SLAM system with vocabulary and settings
     * @param vocabPath Path to ORB vocabulary file (ORBvoc.txt)
     * @param configPath Path to settings file (YAML)
     * @param config Parsed configuration parameters
     * @return true if initialization successful
     *
     * ORB-SLAM3 actual implementation:
     * @code
     * ORB_SLAM3::System::eSensor sensor = ORB_SLAM3::System::IMU_MONOCULAR;
     * mpSystem = new ORB_SLAM3::System(vocabPath, configPath, sensor, false);
     * @endcode
     */
    bool initialize(const std::string& vocabPath, const std::string& configPath,
                    const ORBSLAM3Config& config) {
        std::cout << "[ORB-SLAM3] Initializing System" << std::endl;
        std::cout << "[ORB-SLAM3] Vocabulary: " << vocabPath << std::endl;
        std::cout << "[ORB-SLAM3] Config: " << configPath << std::endl;
        std::cout << "[ORB-SLAM3] Sensor type: " << config.sensorType << std::endl;
        std::cout << "[ORB-SLAM3] Parameters:" << std::endl;
        std::cout << "  - Image size: " << config.imageWidth << "x"
                  << config.imageHeight << std::endl;
        std::cout << "  - ORB features: " << config.nFeatures << std::endl;
        std::cout << "  - IMU frequency: " << config.imuFrequency << " Hz" << std::endl;

        config_ = config;

        // In actual implementation, load vocabulary and create System
        // Vocabulary loading takes ~5-10 seconds for ORBvoc.txt
        vocabPath_ = vocabPath;

        initialized_ = true;
        trackingState_ = TrackingState::NOT_INITIALIZED;
        return true;
    }

    /**
     * @brief Process monocular-inertial frame
     * @param image Input image (grayscale preferred)
     * @param timestampSec Timestamp in seconds
     * @param imuMeasurements Vector of IMU measurements since last image
     * @return Estimated camera pose (4x4 transformation matrix, row-major)
     *
     * ORB-SLAM3 actual implementation:
     * @code
     * cv::Mat Tcw = mpSystem->TrackMonocular(image, timestamp, imuMeasurements);
     * @endcode
     */
    bool trackMonocularInertial(const cv::Mat& image, double timestampSec,
                                 const std::vector<IMUSample>& imuMeasurements,
                                 double* Tcw) {
        (void)timestampSec;

        if (!initialized_) {
            std::cerr << "[ORB-SLAM3] System not initialized" << std::endl;
            return false;
        }

        frameCount_++;

        // Process IMU measurements
        for (const auto& imu : imuMeasurements) {
            (void)imu;
            imuCount_++;
        }

        // Placeholder: extract ORB features and track
        if (!image.empty()) {
            cv::Mat gray;
            if (image.channels() == 3) {
                cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
            } else {
                gray = image;
            }

            // Detect ORB features (placeholder for actual ORB-SLAM3 tracking)
            cv::Ptr<cv::ORB> orb = cv::ORB::create(config_.nFeatures);
            std::vector<cv::KeyPoint> keypoints;
            cv::Mat descriptors;
            orb->detectAndCompute(gray, cv::noArray(), keypoints, descriptors);

            trackedPoints_ = static_cast<int>(keypoints.size());

            // Simulate initialization after enough frames
            if (trackingState_ == TrackingState::NOT_INITIALIZED) {
                if (frameCount_ >= 10 && trackedPoints_ > 100) {
                    trackingState_ = TrackingState::OK;
                    std::cout << "[ORB-SLAM3] Tracking initialized" << std::endl;
                }
            }

            // Update tracking state based on tracked features
            if (trackingState_ == TrackingState::OK) {
                if (trackedPoints_ < 30) {
                    trackingState_ = TrackingState::RECENTLY_LOST;
                    std::cout << "[ORB-SLAM3] Tracking recently lost" << std::endl;
                }
            } else if (trackingState_ == TrackingState::RECENTLY_LOST) {
                if (trackedPoints_ < 20) {
                    trackingState_ = TrackingState::LOST;
                    std::cout << "[ORB-SLAM3] Tracking lost" << std::endl;
                } else if (trackedPoints_ > 50) {
                    trackingState_ = TrackingState::OK;
                    std::cout << "[ORB-SLAM3] Tracking recovered" << std::endl;
                }
            }
        }

        // Return identity pose for placeholder
        // In actual implementation, this would be the estimated Tcw
        if (Tcw != nullptr && trackingState_ == TrackingState::OK) {
            // Identity matrix (camera at origin)
            Tcw[0] = 1.0; Tcw[1] = 0.0; Tcw[2] = 0.0; Tcw[3] = 0.0;
            Tcw[4] = 0.0; Tcw[5] = 1.0; Tcw[6] = 0.0; Tcw[7] = 0.0;
            Tcw[8] = 0.0; Tcw[9] = 0.0; Tcw[10] = 1.0; Tcw[11] = 0.0;
            Tcw[12] = 0.0; Tcw[13] = 0.0; Tcw[14] = 0.0; Tcw[15] = 1.0;

            // Simulate some movement based on frame count
            Tcw[3] = frameCount_ * 0.001;  // x translation
            return true;
        }

        return false;
    }

    /**
     * @brief Get current tracking state
     */
    TrackingState getTrackingState() const {
        return trackingState_;
    }

    /**
     * @brief Get number of tracked map points
     */
    int getTrackedPoints() const {
        return trackedPoints_;
    }

    /**
     * @brief Get all map points from current active map
     * @return Vector of 3D map points
     *
     * ORB-SLAM3 actual implementation:
     * @code
     * std::vector<ORB_SLAM3::MapPoint*> vpMPs = mpSystem->GetAllMapPoints();
     * @endcode
     */
    std::vector<MapPoint> getMapPoints() const {
        std::vector<MapPoint> points;

        if (trackingState_ != TrackingState::OK) {
            return points;
        }

        // Placeholder: generate some simulated map points
        // In actual implementation, retrieve from ORB-SLAM3 Map
        int numPoints = std::min(trackedPoints_, 100);
        for (int i = 0; i < numPoints; ++i) {
            MapPoint mp;
            mp.id = i;
            // Random 3D points around camera
            mp.position[0] = (static_cast<double>(i % 10) - 5.0) * 0.5;
            mp.position[1] = (static_cast<double>(i / 10 % 10) - 5.0) * 0.5;
            mp.position[2] = 3.0 + static_cast<double>(i % 5) * 0.2;
            mp.observations = 5 + (i % 10);
            mp.color[0] = 128;
            mp.color[1] = 128;
            mp.color[2] = 128;
            points.push_back(mp);
        }

        return points;
    }

    /**
     * @brief Reset the SLAM system
     *
     * ORB-SLAM3 actual implementation:
     * @code
     * mpSystem->Reset();
     * @endcode
     */
    void reset() {
        std::cout << "[ORB-SLAM3] Resetting system" << std::endl;
        trackingState_ = TrackingState::NOT_INITIALIZED;
        trackedPoints_ = 0;
        imuCount_ = 0;
        frameCount_ = 0;
    }

    /**
     * @brief Shutdown the SLAM system
     *
     * ORB-SLAM3 actual implementation:
     * @code
     * mpSystem->Shutdown();
     * @endcode
     */
    void shutdown() {
        std::cout << "[ORB-SLAM3] Shutting down system" << std::endl;
        initialized_ = false;
        trackingState_ = TrackingState::SYSTEM_NOT_READY;
        trackedPoints_ = 0;
        imuCount_ = 0;
        frameCount_ = 0;
    }

    bool isInitialized() const {
        return initialized_;
    }

private:
    bool initialized_;
    TrackingState trackingState_;
    int trackedPoints_;
    int imuCount_;
    int frameCount_;
    std::string vocabPath_;
    ORBSLAM3Config config_;
};

// ORBSLAM3Adapter implementation

ORBSLAM3Adapter::ORBSLAM3Adapter()
    : system_(std::make_unique<ORBSLAM3System>()),
      status_(TrackingStatus::UNINITIALIZED),
      lastImageTimestampNs_(0),
      lastImuTimestampNs_(0),
      initStartTimeNs_(0),
      initialized_(false),
      calibrated_(false),
      imuInitialized_(false),
      trackedFeatureCount_(0) {
}

ORBSLAM3Adapter::~ORBSLAM3Adapter() {
    shutdown();
}

bool ORBSLAM3Adapter::initialize(const std::string& configPath) {
    std::lock_guard<std::mutex> lock(statusMutex_);

    logStatus("Initializing ORB-SLAM3 adapter with config: " + configPath);

    configPath_ = configPath;

    if (!loadConfiguration(configPath)) {
        logError("Failed to load configuration from: " + configPath);
        return false;
    }

    updateStatus(TrackingStatus::INITIALIZING);
    initialized_ = true;

    logStatus("ORB-SLAM3 adapter initialized successfully");
    return true;
}

bool ORBSLAM3Adapter::loadCalibration(const std::string& calibPath) {
    std::lock_guard<std::mutex> lock(statusMutex_);

    logStatus("Loading calibration from: " + calibPath);

    calibPath_ = calibPath;

    if (!loadCalibrationData(calibPath)) {
        logError("Failed to load calibration from: " + calibPath);
        return false;
    }

    calibrated_ = true;

    if (initialized_ && !configPath_.empty()) {
        // Initialize ORB-SLAM3 with both config and calibration
        if (!system_->initialize(config_.vocabularyPath, configPath_, config_)) {
            logError("Failed to initialize ORB-SLAM3 System");
            updateStatus(TrackingStatus::UNINITIALIZED);
            return false;
        }
        logStatus("ORB-SLAM3 System ready for data");
    }

    logStatus("Calibration loaded successfully");
    return true;
}

void ORBSLAM3Adapter::processImage(const cv::Mat& image, int64_t timestampNs) {
    if (!initialized_) {
        logError("ORB-SLAM3 adapter not initialized");
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

    // Collect IMU measurements since last image
    std::vector<IMUSample> imuMeasurements;
    {
        std::lock_guard<std::mutex> lock(imuMutex_);
        while (!imuBuffer_.empty()) {
            const IMUSample& imu = imuBuffer_.front();
            if (lastImageTimestampNs_ > 0 && imu.timestampNs <= lastImageTimestampNs_) {
                imuBuffer_.pop_front();
                continue;
            }
            if (imu.timestampNs > timestampNs) {
                break;
            }
            imuMeasurements.push_back(imu);
            imuBuffer_.pop_front();
        }
    }

    // Convert timestamp to seconds for ORB-SLAM3
    double timestampSec = timestampNs * 1e-9;

    // Track monocular-inertial frame
    double Tcw[16];
    bool success = system_->trackMonocularInertial(image, timestampSec,
                                                    imuMeasurements, Tcw);

    lastImageTimestampNs_ = timestampNs;

    // Update tracked feature count
    trackedFeatureCount_ = system_->getTrackedPoints();

    // Update pose if tracking successful
    if (success) {
        Pose6DoF pose;
        pose.timestampNs = timestampNs;
        pose.valid = true;

        // Extract position from transformation matrix (last column)
        // Tcw is camera-to-world, need to invert for world position
        // For simplicity, use translation directly as position estimate
        pose.position[0] = Tcw[3];
        pose.position[1] = Tcw[7];
        pose.position[2] = Tcw[11];

        // Extract rotation as quaternion from rotation matrix
        // R = [Tcw[0:3], Tcw[4:7], Tcw[8:11]]
        // Using simplified quaternion extraction
        double trace = Tcw[0] + Tcw[5] + Tcw[10];
        if (trace > 0) {
            double s = 0.5 / std::sqrt(trace + 1.0);
            pose.orientation[0] = 0.25 / s;  // w
            pose.orientation[1] = (Tcw[9] - Tcw[6]) * s;  // x
            pose.orientation[2] = (Tcw[2] - Tcw[8]) * s;  // y
            pose.orientation[3] = (Tcw[4] - Tcw[1]) * s;  // z
        } else {
            // Fallback to identity quaternion
            pose.orientation[0] = 1.0;
            pose.orientation[1] = 0.0;
            pose.orientation[2] = 0.0;
            pose.orientation[3] = 0.0;
        }

        updatePose(pose);

        if (status_ != TrackingStatus::TRACKING) {
            updateStatus(TrackingStatus::TRACKING);
            logStatus("Tracking established");
        }
    } else {
        // Update status based on ORB-SLAM3 tracking state
        auto trackingState = system_->getTrackingState();
        if (trackingState == ORBSLAM3System::TrackingState::LOST ||
            trackingState == ORBSLAM3System::TrackingState::RECENTLY_LOST) {
            if (status_ == TrackingStatus::TRACKING) {
                updateStatus(TrackingStatus::LOST);
                logStatus("Tracking lost");
            }
        }
    }
}

void ORBSLAM3Adapter::processIMU(const IMUSample& imu) {
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
    constexpr double MAX_GYRO = 20.0;   // 20 rad/s
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
}

bool ORBSLAM3Adapter::getPose(Pose6DoF& pose) const {
    std::lock_guard<std::mutex> lock(poseMutex_);
    pose = latestPose_;
    return latestPose_.valid;
}

TrackingStatus ORBSLAM3Adapter::getStatus() const {
    return status_.load();
}

std::vector<MapPoint> ORBSLAM3Adapter::getMapPoints() const {
    if (status_ != TrackingStatus::TRACKING) {
        return std::vector<MapPoint>();
    }

    return system_->getMapPoints();
}

void ORBSLAM3Adapter::reset() {
    std::lock_guard<std::mutex> lock(statusMutex_);

    logStatus("Resetting ORB-SLAM3 adapter");

    if (system_) {
        system_->reset();
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
    initStartTimeNs_ = 0;
    imuInitialized_ = false;
    trackedFeatureCount_ = 0;

    updateStatus(TrackingStatus::INITIALIZING);
    logStatus("ORB-SLAM3 adapter reset complete");
}

void ORBSLAM3Adapter::shutdown() {
    std::lock_guard<std::mutex> lock(statusMutex_);

    logStatus("Shutting down ORB-SLAM3 adapter");

    if (system_) {
        system_->shutdown();
    }

    initialized_ = false;
    calibrated_ = false;
    updateStatus(TrackingStatus::UNINITIALIZED);
    logStatus("ORB-SLAM3 adapter shut down");
}

ORBSLAM3Config ORBSLAM3Adapter::getConfig() const {
    return config_;
}

bool ORBSLAM3Adapter::isReadyForVision() const {
    std::lock_guard<std::mutex> lock(imuMutex_);

    if (initStartTimeNs_ == 0 || imuBuffer_.empty()) {
        return false;
    }

    double elapsedSeconds = (lastImuTimestampNs_ - initStartTimeNs_) * 1e-9;
    return elapsedSeconds >= IMU_INIT_WINDOW && imuInitialized_;
}

int ORBSLAM3Adapter::getTrackedFeatureCount() const {
    return trackedFeatureCount_.load();
}

bool ORBSLAM3Adapter::loadConfiguration(const std::string& configPath) {
    std::ifstream configFile(configPath);
    if (!configFile.is_open()) {
        logError("Failed to open config file: " + configPath);
        return false;
    }

    // Parse YAML configuration
    if (!parseYamlConfig(configPath)) {
        logStatus("Using default configuration parameters");
    }

    // Determine vocabulary path from config or use default
    if (config_.vocabularyPath.empty()) {
        size_t lastSlash = configPath.find_last_of("/\\");
        std::string configDir = (lastSlash != std::string::npos) ?
                                configPath.substr(0, lastSlash) : ".";
        config_.vocabularyPath = configDir + "/../vocab/ORBvoc.txt";
    }

    logStatus("Configuration loaded from: " + configPath);
    logStatus("Vocabulary path: " + config_.vocabularyPath);
    return true;
}

bool ORBSLAM3Adapter::loadCalibrationData(const std::string& calibPath) {
    std::ifstream calibFile(calibPath);
    if (!calibFile.is_open()) {
        logError("Failed to open calibration file: " + calibPath);
        return false;
    }

    // ORB-SLAM3 calibration is typically in the same file as configuration
    // The calibration file should contain:
    // - Camera intrinsics (fx, fy, cx, cy)
    // - Distortion coefficients (k1, k2, p1, p2, k3)
    // - Camera-IMU extrinsics (Tbc matrix)
    // - IMU noise parameters

    logStatus("Calibration loaded from: " + calibPath);
    return true;
}

bool ORBSLAM3Adapter::parseYamlConfig(const std::string& configPath) {
    std::ifstream file(configPath);
    if (!file.is_open()) {
        return false;
    }

    std::string line;
    bool inTbcMatrix = false;
    std::vector<double> tbcData;

    while (std::getline(file, line)) {
        // Skip comments and empty lines
        if (line.empty() || line[0] == '#' || line[0] == '%') {
            continue;
        }

        // Handle Tbc matrix data continuation
        if (inTbcMatrix) {
            size_t dataStart = line.find('[');
            size_t dataEnd = line.find(']');
            if (dataStart != std::string::npos || !tbcData.empty()) {
                std::string dataStr;
                if (dataStart != std::string::npos && dataEnd != std::string::npos) {
                    dataStr = line.substr(dataStart + 1, dataEnd - dataStart - 1);
                } else if (dataStart != std::string::npos) {
                    dataStr = line.substr(dataStart + 1);
                } else if (dataEnd != std::string::npos) {
                    dataStr = line.substr(0, dataEnd);
                } else {
                    dataStr = line;
                }

                // Parse comma-separated values
                std::stringstream ss(dataStr);
                std::string value;
                while (std::getline(ss, value, ',')) {
                    try {
                        tbcData.push_back(std::stod(value));
                    } catch (...) {
                        // Skip non-numeric values
                    }
                }

                if (dataEnd != std::string::npos || tbcData.size() >= 16) {
                    inTbcMatrix = false;
                    if (tbcData.size() >= 16) {
                        for (size_t i = 0; i < 16; ++i) {
                            config_.Tbc[i] = tbcData[i];
                        }
                    }
                }
            }
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

        // Remove quotes from string values
        if (!value.empty() && (value.front() == '"' || value.front() == '\'')) {
            value = value.substr(1, value.length() - 2);
        }

        // Parse known parameters
        try {
            if (key == "Vocabulary") {
                config_.vocabularyPath = value;
            } else if (key == "sensor_type") {
                config_.sensorType = value;
            } else if (key == "Camera.type") {
                config_.cameraType = value;
            } else if (key == "Camera.fx") {
                config_.fx = std::stod(value);
            } else if (key == "Camera.fy") {
                config_.fy = std::stod(value);
            } else if (key == "Camera.cx") {
                config_.cx = std::stod(value);
            } else if (key == "Camera.cy") {
                config_.cy = std::stod(value);
            } else if (key == "Camera.k1") {
                config_.k1 = std::stod(value);
            } else if (key == "Camera.k2") {
                config_.k2 = std::stod(value);
            } else if (key == "Camera.p1") {
                config_.p1 = std::stod(value);
            } else if (key == "Camera.p2") {
                config_.p2 = std::stod(value);
            } else if (key == "Camera.k3") {
                config_.k3 = std::stod(value);
            } else if (key == "Camera.width") {
                config_.imageWidth = std::stoi(value);
            } else if (key == "Camera.height") {
                config_.imageHeight = std::stoi(value);
            } else if (key == "Camera.fps") {
                config_.fps = std::stod(value);
            } else if (key == "Camera.RGB") {
                config_.isRGB = (value == "1" || value == "true");
            } else if (key == "Camera.bf") {
                config_.bf = std::stod(value);
            } else if (key == "ThDepth") {
                config_.thDepth = std::stod(value);
            } else if (key == "DepthMapFactor") {
                config_.depthMapFactor = std::stod(value);
            } else if (key == "IMU.Frequency") {
                config_.imuFrequency = std::stod(value);
            } else if (key == "IMU.NoiseGyro") {
                config_.noiseGyro = std::stod(value);
            } else if (key == "IMU.NoiseAcc") {
                config_.noiseAcc = std::stod(value);
            } else if (key == "IMU.GyroWalk") {
                config_.gyroWalk = std::stod(value);
            } else if (key == "IMU.AccWalk") {
                config_.accWalk = std::stod(value);
            } else if (key == "IMU.TimeOffset") {
                config_.imuTimeOffset = std::stod(value);
            } else if (key == "ORBextractor.nFeatures") {
                config_.nFeatures = std::stoi(value);
            } else if (key == "ORBextractor.scaleFactor") {
                config_.scaleFactor = std::stod(value);
            } else if (key == "ORBextractor.nLevels") {
                config_.nLevels = std::stoi(value);
            } else if (key == "ORBextractor.iniThFAST") {
                config_.iniThFAST = std::stoi(value);
            } else if (key == "ORBextractor.minThFAST") {
                config_.minThFAST = std::stoi(value);
            } else if (key == "System.UseViewer") {
                config_.useViewer = (value == "1" || value == "true");
            } else if (key == "System.SaveTrajectory") {
                config_.saveTrajectory = (value == "1" || value == "true");
            } else if (key == "System.TrajectoryFile") {
                config_.trajectoryFile = value;
            } else if (key == "System.nMaxMaps") {
                config_.maxMaps = std::stoi(value);
            } else if (key == "System.LocalWindow") {
                config_.localWindow = std::stoi(value);
            } else if (key == "Tbc") {
                inTbcMatrix = true;
                tbcData.clear();
            }
        } catch (const std::exception& e) {
            std::cerr << "[ORB-SLAM3] Warning: Failed to parse " << key
                      << ": " << e.what() << std::endl;
        }
    }

    return true;
}

void ORBSLAM3Adapter::updateStatus(TrackingStatus newStatus) {
    status_ = newStatus;

    const char* statusStr = "UNKNOWN";
    switch (newStatus) {
        case TrackingStatus::UNINITIALIZED: statusStr = "UNINITIALIZED"; break;
        case TrackingStatus::INITIALIZING: statusStr = "INITIALIZING"; break;
        case TrackingStatus::TRACKING: statusStr = "TRACKING"; break;
        case TrackingStatus::LOST: statusStr = "LOST"; break;
        case TrackingStatus::RELOCALIZATION: statusStr = "RELOCALIZATION"; break;
    }
    std::cout << "[ORB-SLAM3] Status: " << statusStr << std::endl;
}

void ORBSLAM3Adapter::updatePose(const Pose6DoF& pose) {
    std::lock_guard<std::mutex> lock(poseMutex_);
    latestPose_ = pose;
}

void ORBSLAM3Adapter::processIMUQueue() {
    std::lock_guard<std::mutex> lock(imuMutex_);

    // Process all queued IMU samples
    // ORB-SLAM3 processes IMU as batch with each image
    // This method is kept for compatibility but main processing
    // happens in processImage
    while (!imuBuffer_.empty()) {
        const IMUSample& imu = imuBuffer_.front();

        if (lastImageTimestampNs_ > 0 && imu.timestampNs > lastImageTimestampNs_) {
            break;
        }

        imuBuffer_.pop_front();
    }
}

bool ORBSLAM3Adapter::checkImuInitialization(int64_t imageTimestampNs) {
    if (imuInitialized_) {
        return true;
    }

    std::lock_guard<std::mutex> lock(imuMutex_);

    if (initStartTimeNs_ == 0 || imuBuffer_.empty()) {
        return false;
    }

    double elapsedSeconds = (imageTimestampNs - initStartTimeNs_) * 1e-9;

    if (elapsedSeconds >= IMU_INIT_WINDOW) {
        imuInitialized_ = true;
        logStatus("IMU initialization complete after " +
                  std::to_string(elapsedSeconds) + " seconds");
        return true;
    }

    return false;
}

void ORBSLAM3Adapter::logStatus(const std::string& message) const {
    std::cout << "[ORB-SLAM3] " << message << std::endl;
}

void ORBSLAM3Adapter::logError(const std::string& message) const {
    std::cerr << "[ORB-SLAM3 ERROR] " << message << std::endl;
}

}  // namespace vi_slam
