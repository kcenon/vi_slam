#include "slam/adapters/vins_mono_adapter.hpp"
#include "common/logging.hpp"
#include <fstream>
#include <sstream>

namespace vi_slam {

// Placeholder for VINS-Mono Estimator
// In a real implementation, this would interface with the actual VINS-Mono library
class VINSMonoEstimator {
public:
    bool initialize(const std::string& config, const std::string& calib) {
        LOG_DEBUG("VINSMono", "Initializing estimator");
        LOG_DEBUG("VINSMono", "Config: {}", config);
        LOG_DEBUG("VINSMono", "Calib: {}", calib);
        return true;
    }

    void feedImage(const cv::Mat& image, int64_t timestampNs) {
        (void)image;
        (void)timestampNs;
        LOG_DEBUG("VINSMono", "Feeding image to estimator");
    }

    void feedIMU(const IMUSample& imu) {
        (void)imu;
    }

    bool getPose(Pose6DoF& pose) const {
        (void)pose;
        return false;
    }

    void reset() {
        LOG_DEBUG("VINSMono", "Resetting estimator");
    }

    void shutdown() {
        LOG_DEBUG("VINSMono", "Shutting down estimator");
    }
};

VINSMonoAdapter::VINSMonoAdapter()
    : estimator_(std::make_unique<VINSMonoEstimator>()),
      status_(TrackingStatus::UNINITIALIZED),
      initialized_(false) {
}

VINSMonoAdapter::~VINSMonoAdapter() {
    shutdown();
}

bool VINSMonoAdapter::initialize(const std::string& configPath) {
    std::lock_guard<std::mutex> lock(statusMutex_);

    configPath_ = configPath;

    if (!loadConfiguration(configPath)) {
        LOG_ERROR("VINSMono", "Failed to load configuration from: {}", configPath);
        return false;
    }

    updateStatus(TrackingStatus::INITIALIZING);
    initialized_ = true;

    LOG_INFO("VINSMono", "Adapter initialized successfully");
    return true;
}

bool VINSMonoAdapter::loadCalibration(const std::string& calibPath) {
    std::lock_guard<std::mutex> lock(statusMutex_);

    calibPath_ = calibPath;

    if (!loadCalibrationData(calibPath)) {
        LOG_ERROR("VINSMono", "Failed to load calibration from: {}", calibPath);
        return false;
    }

    if (initialized_ && !configPath_.empty()) {
        // Initialize VINS-Mono with both config and calibration
        if (!estimator_->initialize(configPath_, calibPath_)) {
            LOG_ERROR("VINSMono", "Failed to initialize VINS-Mono estimator");
            updateStatus(TrackingStatus::UNINITIALIZED);
            return false;
        }
        updateStatus(TrackingStatus::TRACKING);
    }

    LOG_INFO("VINSMono", "Calibration loaded successfully");
    return true;
}

void VINSMonoAdapter::processImage(const cv::Mat& image, int64_t timestampNs) {
    if (!initialized_) {
        LOG_WARN("VINSMono", "Adapter not initialized");
        return;
    }

    if (image.empty()) {
        LOG_WARN("VINSMono", "Received empty image");
        return;
    }

    // Process queued IMU samples first
    processIMUQueue();

    // Feed image to VINS-Mono
    estimator_->feedImage(image, timestampNs);

    // Update pose from estimator
    Pose6DoF pose;
    if (estimator_->getPose(pose)) {
        updatePose(pose);
        updateStatus(TrackingStatus::TRACKING);
    } else {
        // If pose retrieval fails, mark as lost
        if (status_ == TrackingStatus::TRACKING) {
            updateStatus(TrackingStatus::LOST);
        }
    }
}

void VINSMonoAdapter::processIMU(const IMUSample& imu) {
    if (!initialized_) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(imuMutex_);
        imuBuffer_.push_back(imu);

        // Keep buffer size limited to prevent memory growth
        constexpr size_t MAX_IMU_BUFFER_SIZE = 1000;
        if (imuBuffer_.size() > MAX_IMU_BUFFER_SIZE) {
            imuBuffer_.pop_front();
        }
    }

    // Feed IMU to estimator
    estimator_->feedIMU(imu);
}

bool VINSMonoAdapter::getPose(Pose6DoF& pose) const {
    std::lock_guard<std::mutex> lock(poseMutex_);
    pose = latestPose_;
    return latestPose_.valid;
}

TrackingStatus VINSMonoAdapter::getStatus() const {
    std::lock_guard<std::mutex> lock(statusMutex_);
    return status_;
}

std::vector<MapPoint> VINSMonoAdapter::getMapPoints() const {
    return std::vector<MapPoint>();
}

void VINSMonoAdapter::reset() {
    std::lock_guard<std::mutex> lock(statusMutex_);

    if (estimator_) {
        estimator_->reset();
    }

    {
        std::lock_guard<std::mutex> imuLock(imuMutex_);
        imuBuffer_.clear();
    }

    {
        std::lock_guard<std::mutex> poseLock(poseMutex_);
        latestPose_ = Pose6DoF();
    }

    updateStatus(TrackingStatus::INITIALIZING);
    LOG_INFO("VINSMono", "Adapter reset");
}

void VINSMonoAdapter::shutdown() {
    std::lock_guard<std::mutex> lock(statusMutex_);

    if (estimator_) {
        estimator_->shutdown();
    }

    initialized_ = false;
    updateStatus(TrackingStatus::UNINITIALIZED);
    LOG_INFO("VINSMono", "Adapter shut down");
}

bool VINSMonoAdapter::loadConfiguration(const std::string& configPath) {
    std::ifstream configFile(configPath);
    if (!configFile.is_open()) {
        LOG_ERROR("VINSMono", "Failed to open config file: {}", configPath);
        return false;
    }

    LOG_DEBUG("VINSMono", "Configuration loaded from: {}", configPath);
    return true;
}

bool VINSMonoAdapter::loadCalibrationData(const std::string& calibPath) {
    std::ifstream calibFile(calibPath);
    if (!calibFile.is_open()) {
        LOG_ERROR("VINSMono", "Failed to open calibration file: {}", calibPath);
        return false;
    }

    LOG_DEBUG("VINSMono", "Calibration loaded from: {}", calibPath);
    return true;
}

void VINSMonoAdapter::updateStatus(TrackingStatus newStatus) {
    status_ = newStatus;
    LOG_DEBUG("VINSMono", "Status updated to: {}", static_cast<int>(newStatus));
}

void VINSMonoAdapter::updatePose(const Pose6DoF& pose) {
    std::lock_guard<std::mutex> lock(poseMutex_);
    latestPose_ = pose;
}

void VINSMonoAdapter::processIMUQueue() {
    std::lock_guard<std::mutex> lock(imuMutex_);

    // Process all queued IMU samples
    // This ensures IMU data is processed in order before image processing
    while (!imuBuffer_.empty()) {
        const IMUSample& imu = imuBuffer_.front();
        estimator_->feedIMU(imu);
        imuBuffer_.pop_front();
    }
}

}  // namespace vi_slam
