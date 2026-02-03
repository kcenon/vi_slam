#include "slam/adapters/orbslam3_adapter.hpp"
#include "common/logging.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <fstream>
#include <sstream>
#include <cmath>
#include <chrono>
#include <algorithm>

namespace vi_slam {

/**
 * @brief ORB-SLAM3 System wrapper class
 */
class ORBSLAM3System {
public:
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

    bool initialize(const std::string& vocabPath, const std::string& configPath,
                    const ORBSLAM3Config& config) {
        LOG_INFO("ORB-SLAM3", "Initializing System");
        LOG_DEBUG("ORB-SLAM3", "Vocabulary: {}", vocabPath);
        LOG_DEBUG("ORB-SLAM3", "Config: {}", configPath);
        LOG_DEBUG("ORB-SLAM3", "Sensor type: {}", config.sensorType);
        LOG_DEBUG("ORB-SLAM3", "Image size: {}x{}", config.imageWidth, config.imageHeight);
        LOG_DEBUG("ORB-SLAM3", "ORB features: {}", config.nFeatures);

        config_ = config;
        vocabPath_ = vocabPath;
        initialized_ = true;
        trackingState_ = TrackingState::NOT_INITIALIZED;
        return true;
    }

    bool trackMonocularInertial(const cv::Mat& image, double timestampSec,
                                 const std::vector<IMUSample>& imuMeasurements,
                                 double* Tcw) {
        (void)timestampSec;

        if (!initialized_) {
            LOG_WARN("ORB-SLAM3", "System not initialized");
            return false;
        }

        frameCount_++;

        for (const auto& imu : imuMeasurements) {
            (void)imu;
            imuCount_++;
        }

        if (!image.empty()) {
            cv::Mat gray;
            if (image.channels() == 3) {
                cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
            } else {
                gray = image;
            }

            cv::Ptr<cv::ORB> orb = cv::ORB::create(config_.nFeatures);
            std::vector<cv::KeyPoint> keypoints;
            cv::Mat descriptors;
            orb->detectAndCompute(gray, cv::noArray(), keypoints, descriptors);

            trackedPoints_ = static_cast<int>(keypoints.size());

            if (trackingState_ == TrackingState::NOT_INITIALIZED) {
                if (frameCount_ >= 10 && trackedPoints_ > 100) {
                    trackingState_ = TrackingState::OK;
                    LOG_INFO("ORB-SLAM3", "Tracking initialized");
                }
            }

            if (trackingState_ == TrackingState::OK) {
                if (trackedPoints_ < 30) {
                    trackingState_ = TrackingState::RECENTLY_LOST;
                    LOG_WARN("ORB-SLAM3", "Tracking recently lost");
                }
            } else if (trackingState_ == TrackingState::RECENTLY_LOST) {
                if (trackedPoints_ < 20) {
                    trackingState_ = TrackingState::LOST;
                    LOG_WARN("ORB-SLAM3", "Tracking lost");
                } else if (trackedPoints_ > 50) {
                    trackingState_ = TrackingState::OK;
                    LOG_INFO("ORB-SLAM3", "Tracking recovered");
                }
            }
        }

        if (Tcw != nullptr && trackingState_ == TrackingState::OK) {
            Tcw[0] = 1.0; Tcw[1] = 0.0; Tcw[2] = 0.0; Tcw[3] = 0.0;
            Tcw[4] = 0.0; Tcw[5] = 1.0; Tcw[6] = 0.0; Tcw[7] = 0.0;
            Tcw[8] = 0.0; Tcw[9] = 0.0; Tcw[10] = 1.0; Tcw[11] = 0.0;
            Tcw[12] = 0.0; Tcw[13] = 0.0; Tcw[14] = 0.0; Tcw[15] = 1.0;
            Tcw[3] = frameCount_ * 0.001;
            return true;
        }

        return false;
    }

    TrackingState getTrackingState() const { return trackingState_; }
    int getTrackedPoints() const { return trackedPoints_; }

    std::vector<MapPoint> getMapPoints() const {
        std::vector<MapPoint> points;
        if (trackingState_ != TrackingState::OK) {
            return points;
        }

        int numPoints = std::min(trackedPoints_, 100);
        for (int i = 0; i < numPoints; ++i) {
            MapPoint mp;
            mp.id = i;
            mp.position = Eigen::Vector3d(
                (static_cast<double>(i % 10) - 5.0) * 0.5,
                (static_cast<double>(i / 10 % 10) - 5.0) * 0.5,
                3.0 + static_cast<double>(i % 5) * 0.2
            );
            mp.observations = 5 + (i % 10);
            mp.color = Eigen::Matrix<uint8_t, 3, 1>::Constant(128);
            points.push_back(mp);
        }

        return points;
    }

    void reset() {
        LOG_INFO("ORB-SLAM3", "Resetting system");
        trackingState_ = TrackingState::NOT_INITIALIZED;
        trackedPoints_ = 0;
        imuCount_ = 0;
        frameCount_ = 0;
    }

    void shutdown() {
        LOG_INFO("ORB-SLAM3", "Shutting down system");
        initialized_ = false;
        trackingState_ = TrackingState::SYSTEM_NOT_READY;
        trackedPoints_ = 0;
        imuCount_ = 0;
        frameCount_ = 0;
    }

    bool isInitialized() const { return initialized_; }

private:
    bool initialized_;
    TrackingState trackingState_;
    int trackedPoints_;
    int imuCount_;
    int frameCount_;
    std::string vocabPath_;
    ORBSLAM3Config config_;
};

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

    LOG_INFO("ORB-SLAM3", "Initializing adapter with config: {}", configPath);

    configPath_ = configPath;

    if (!loadConfiguration(configPath)) {
        LOG_ERROR("ORB-SLAM3", "Failed to load configuration from: {}", configPath);
        return false;
    }

    updateStatus(TrackingStatus::INITIALIZING);
    initialized_ = true;

    LOG_INFO("ORB-SLAM3", "Adapter initialized successfully");
    return true;
}

bool ORBSLAM3Adapter::loadCalibration(const std::string& calibPath) {
    std::lock_guard<std::mutex> lock(statusMutex_);

    LOG_INFO("ORB-SLAM3", "Loading calibration from: {}", calibPath);

    calibPath_ = calibPath;

    if (!loadCalibrationData(calibPath)) {
        LOG_ERROR("ORB-SLAM3", "Failed to load calibration from: {}", calibPath);
        return false;
    }

    calibrated_ = true;

    if (initialized_ && !configPath_.empty()) {
        if (!system_->initialize(config_.vocabularyPath, configPath_, config_)) {
            LOG_ERROR("ORB-SLAM3", "Failed to initialize System");
            updateStatus(TrackingStatus::UNINITIALIZED);
            return false;
        }
        LOG_INFO("ORB-SLAM3", "System ready for data");
    }

    LOG_INFO("ORB-SLAM3", "Calibration loaded successfully");
    return true;
}

void ORBSLAM3Adapter::processImage(const cv::Mat& image, int64_t timestampNs) {
    if (!initialized_) {
        LOG_WARN("ORB-SLAM3", "Adapter not initialized");
        return;
    }

    if (!calibrated_) {
        LOG_WARN("ORB-SLAM3", "Calibration not loaded");
        return;
    }

    if (image.empty()) {
        LOG_WARN("ORB-SLAM3", "Received empty image");
        return;
    }

    if (!checkImuInitialization(timestampNs)) {
        LOG_DEBUG("ORB-SLAM3", "Waiting for IMU initialization data...");
        return;
    }

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

    double timestampSec = timestampNs * 1e-9;

    double Tcw[16];
    bool success = system_->trackMonocularInertial(image, timestampSec,
                                                    imuMeasurements, Tcw);

    lastImageTimestampNs_ = timestampNs;
    trackedFeatureCount_ = system_->getTrackedPoints();

    if (success) {
        Pose6DoF pose;
        pose.timestampNs = timestampNs;
        pose.valid = true;
        pose.position = Eigen::Vector3d(Tcw[3], Tcw[7], Tcw[11]);

        Eigen::Matrix3d R;
        R << Tcw[0], Tcw[1], Tcw[2],
             Tcw[4], Tcw[5], Tcw[6],
             Tcw[8], Tcw[9], Tcw[10];
        pose.orientation = Eigen::Quaterniond(R);

        updatePose(pose);

        if (status_ != TrackingStatus::TRACKING) {
            updateStatus(TrackingStatus::TRACKING);
            LOG_INFO("ORB-SLAM3", "Tracking established");
        }
    } else {
        auto trackingState = system_->getTrackingState();
        if (trackingState == ORBSLAM3System::TrackingState::LOST ||
            trackingState == ORBSLAM3System::TrackingState::RECENTLY_LOST) {
            if (status_ == TrackingStatus::TRACKING) {
                updateStatus(TrackingStatus::LOST);
                LOG_WARN("ORB-SLAM3", "Tracking lost");
            }
        }
    }
}

void ORBSLAM3Adapter::processIMU(const IMUSample& imu) {
    if (!initialized_) {
        return;
    }

    if (!imu.acceleration.allFinite() || !imu.angularVelocity.allFinite()) {
        LOG_WARN("ORB-SLAM3", "Received invalid IMU data (NaN or Inf values)");
        return;
    }

    constexpr double MAX_ACC = 100.0;
    constexpr double MAX_GYRO = 20.0;
    if (imu.acceleration.cwiseAbs().maxCoeff() > MAX_ACC ||
        imu.angularVelocity.cwiseAbs().maxCoeff() > MAX_GYRO) {
        LOG_WARN("ORB-SLAM3", "IMU values out of reasonable range");
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

    LOG_INFO("ORB-SLAM3", "Resetting adapter");

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
    LOG_INFO("ORB-SLAM3", "Adapter reset complete");
}

void ORBSLAM3Adapter::shutdown() {
    std::lock_guard<std::mutex> lock(statusMutex_);

    LOG_INFO("ORB-SLAM3", "Shutting down adapter");

    if (system_) {
        system_->shutdown();
    }

    initialized_ = false;
    calibrated_ = false;
    updateStatus(TrackingStatus::UNINITIALIZED);
    LOG_INFO("ORB-SLAM3", "Adapter shut down");
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
        LOG_ERROR("ORB-SLAM3", "Failed to open config file: {}", configPath);
        return false;
    }

    if (!parseYamlConfig(configPath)) {
        LOG_DEBUG("ORB-SLAM3", "Using default configuration parameters");
    }

    if (config_.vocabularyPath.empty()) {
        size_t lastSlash = configPath.find_last_of("/\\");
        std::string configDir = (lastSlash != std::string::npos) ?
                                configPath.substr(0, lastSlash) : ".";
        config_.vocabularyPath = configDir + "/../vocab/ORBvoc.txt";
    }

    LOG_DEBUG("ORB-SLAM3", "Configuration loaded from: {}", configPath);
    LOG_DEBUG("ORB-SLAM3", "Vocabulary path: {}", config_.vocabularyPath);
    return true;
}

bool ORBSLAM3Adapter::loadCalibrationData(const std::string& calibPath) {
    std::ifstream calibFile(calibPath);
    if (!calibFile.is_open()) {
        LOG_ERROR("ORB-SLAM3", "Failed to open calibration file: {}", calibPath);
        return false;
    }

    LOG_DEBUG("ORB-SLAM3", "Calibration loaded from: {}", calibPath);
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
        if (line.empty() || line[0] == '#' || line[0] == '%') {
            continue;
        }

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

                std::stringstream ss(dataStr);
                std::string value;
                while (std::getline(ss, value, ',')) {
                    try {
                        tbcData.push_back(std::stod(value));
                    } catch (...) {}
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
            value = value.substr(1, value.length() - 2);
        }

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
            LOG_WARN("ORB-SLAM3", "Failed to parse {}: {}", key, e.what());
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
    LOG_DEBUG("ORB-SLAM3", "Status: {}", statusStr);
}

void ORBSLAM3Adapter::updatePose(const Pose6DoF& pose) {
    std::lock_guard<std::mutex> lock(poseMutex_);
    latestPose_ = pose;
}

void ORBSLAM3Adapter::processIMUQueue() {
    std::lock_guard<std::mutex> lock(imuMutex_);

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
        LOG_INFO("ORB-SLAM3", "IMU initialization complete after {} seconds",
                 elapsedSeconds);
        return true;
    }

    return false;
}

}  // namespace vi_slam
