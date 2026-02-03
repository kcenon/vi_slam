#include "slam/slam_engine.hpp"
#include "common/logging.hpp"
#include "slam/adapters/vins_mono_adapter.hpp"
#include <stdexcept>

namespace vi_slam {

SLAMEngine::SLAMEngine()
    : framework_(nullptr),
      currentType_(SLAMFrameworkType::VINS_MONO),
      poseCallback_(nullptr),
      statusCallback_(nullptr),
      initialized_(false) {
    LOG_INFO("SLAMEngine", "SLAMEngine created");
}

SLAMEngine::~SLAMEngine() {
    shutdown();
    LOG_INFO("SLAMEngine", "SLAMEngine destroyed");
}

bool SLAMEngine::selectFramework(SLAMFrameworkType type) {
    std::lock_guard<std::mutex> lock(frameworkMutex_);

    // If already using this framework, no need to switch
    if (framework_ && currentType_ == type) {
        LOG_DEBUG("SLAMEngine", "Already using selected framework");
        return true;
    }

    // Shutdown current framework if exists
    if (framework_) {
        framework_->shutdown();
    }

    // Create new framework
    framework_ = createFramework(type);
    if (!framework_) {
        LOG_ERROR("SLAMEngine", "Failed to create framework");
        return false;
    }

    currentType_ = type;
    initialized_ = false;

    LOG_INFO("SLAMEngine", "Framework selected: {}", static_cast<int>(type));
    return true;
}

SLAMFrameworkType SLAMEngine::getCurrentFramework() const {
    std::lock_guard<std::mutex> lock(frameworkMutex_);
    return currentType_;
}

bool SLAMEngine::initialize(const std::string& configPath) {
    std::lock_guard<std::mutex> lock(frameworkMutex_);

    if (!framework_) {
        // Auto-select default framework if none selected
        framework_ = createFramework(currentType_);
        if (!framework_) {
            LOG_ERROR("SLAMEngine", "Failed to create default framework");
            return false;
        }
    }

    if (!framework_->initialize(configPath)) {
        LOG_ERROR("SLAMEngine", "Framework initialization failed");
        return false;
    }

    initialized_ = true;
    LOG_INFO("SLAMEngine", "SLAMEngine initialized successfully");
    return true;
}

bool SLAMEngine::loadCalibration(const std::string& calibPath) {
    std::lock_guard<std::mutex> lock(frameworkMutex_);

    if (!framework_) {
        LOG_ERROR("SLAMEngine", "No framework selected");
        return false;
    }

    if (!framework_->loadCalibration(calibPath)) {
        LOG_ERROR("SLAMEngine", "Failed to load calibration");
        return false;
    }

    LOG_INFO("SLAMEngine", "Calibration loaded successfully");
    return true;
}

void SLAMEngine::processImage(const cv::Mat& image, int64_t timestampNs) {
    std::lock_guard<std::mutex> lock(frameworkMutex_);

    if (!framework_) {
        LOG_ERROR("SLAMEngine", "No framework available for image processing");
        return;
    }

    if (!initialized_) {
        LOG_WARN("SLAMEngine", "Framework not initialized");
        return;
    }

    // Process image through current framework
    framework_->processImage(image, timestampNs);

    // Invoke callbacks if pose available
    Pose6DoF pose;
    if (framework_->getPose(pose) && pose.valid) {
        invokePoseCallback(pose);
    }

    // Invoke status callback
    TrackingStatus status = framework_->getStatus();
    invokeStatusCallback(status);
}

void SLAMEngine::processIMU(const IMUSample& imu) {
    std::lock_guard<std::mutex> lock(frameworkMutex_);

    if (!framework_) {
        LOG_ERROR("SLAMEngine", "No framework available for IMU processing");
        return;
    }

    if (!initialized_) {
        return;
    }

    framework_->processIMU(imu);
}

bool SLAMEngine::getPose(Pose6DoF& pose) const {
    std::lock_guard<std::mutex> lock(frameworkMutex_);

    if (!framework_) {
        LOG_ERROR("SLAMEngine", "No framework available");
        return false;
    }

    return framework_->getPose(pose);
}

TrackingStatus SLAMEngine::getStatus() const {
    std::lock_guard<std::mutex> lock(frameworkMutex_);

    if (!framework_) {
        return TrackingStatus::UNINITIALIZED;
    }

    return framework_->getStatus();
}

std::vector<MapPoint> SLAMEngine::getMapPoints() const {
    std::lock_guard<std::mutex> lock(frameworkMutex_);

    if (!framework_) {
        return std::vector<MapPoint>();
    }

    return framework_->getMapPoints();
}

void SLAMEngine::reset() {
    std::lock_guard<std::mutex> lock(frameworkMutex_);

    if (!framework_) {
        LOG_WARN("SLAMEngine", "No framework to reset");
        return;
    }

    framework_->reset();
    invokeStatusCallback(TrackingStatus::INITIALIZING);

    LOG_INFO("SLAMEngine", "SLAMEngine reset");
}

void SLAMEngine::shutdown() {
    std::lock_guard<std::mutex> lock(frameworkMutex_);

    if (framework_) {
        framework_->shutdown();
        framework_.reset();
    }

    initialized_ = false;
    invokeStatusCallback(TrackingStatus::UNINITIALIZED);

    LOG_INFO("SLAMEngine", "SLAMEngine shut down");
}

void SLAMEngine::setPoseCallback(PoseCallback callback) {
    std::lock_guard<std::mutex> lock(callbackMutex_);
    poseCallback_ = callback;
    LOG_DEBUG("SLAMEngine", "Pose callback set");
}

void SLAMEngine::setStatusCallback(StatusCallback callback) {
    std::lock_guard<std::mutex> lock(callbackMutex_);
    statusCallback_ = callback;
    LOG_DEBUG("SLAMEngine", "Status callback set");
}

std::unique_ptr<ISLAMFramework> SLAMEngine::createFramework(SLAMFrameworkType type) {
    switch (type) {
        case SLAMFrameworkType::VINS_MONO:
            LOG_INFO("SLAMEngine", "Creating VINS-Mono adapter");
            return std::make_unique<VINSMonoAdapter>();

        case SLAMFrameworkType::OPENVINS:
            LOG_WARN("SLAMEngine", "OpenVINS adapter not yet implemented");
            return nullptr;

        case SLAMFrameworkType::ORB_SLAM3:
            LOG_WARN("SLAMEngine", "ORB-SLAM3 adapter not yet implemented");
            return nullptr;

        case SLAMFrameworkType::BASALT:
            LOG_WARN("SLAMEngine", "Basalt adapter not yet implemented");
            return nullptr;

        default:
            LOG_ERROR("SLAMEngine", "Unknown framework type");
            return nullptr;
    }
}

void SLAMEngine::invokePoseCallback(const Pose6DoF& pose) {
    std::lock_guard<std::mutex> lock(callbackMutex_);

    if (poseCallback_) {
        poseCallback_(pose);
    }

#ifdef ENABLE_ROS
    // Publish to ROS if enabled
    if (rosPublisher_) {
        rosPublisher_->publishPose(pose);
    }
#endif

#ifdef ENABLE_ZMQ
    // Publish to ZMQ if enabled
    if (zmqPublisher_) {
        zmqPublisher_->publishPose(pose);
    }
#endif
}

void SLAMEngine::invokeStatusCallback(TrackingStatus status) {
    std::lock_guard<std::mutex> lock(callbackMutex_);

    if (statusCallback_) {
        statusCallback_(status);
    }
}

#ifdef ENABLE_ROS
void SLAMEngine::enableROSPublisher(ros::NodeHandle& nodeHandle,
                                   const output::ROSPublisherConfig& config) {
    std::lock_guard<std::mutex> lock(callbackMutex_);

    if (rosPublisher_) {
        LOG_WARN("SLAMEngine", "ROS publisher already enabled");
        return;
    }

    rosPublisher_ = std::make_unique<output::ROSPublisher>(nodeHandle, config);
    LOG_INFO("SLAMEngine", "ROS publisher enabled");
}

void SLAMEngine::disableROSPublisher() {
    std::lock_guard<std::mutex> lock(callbackMutex_);

    if (!rosPublisher_) {
        LOG_WARN("SLAMEngine", "ROS publisher not enabled");
        return;
    }

    rosPublisher_.reset();
    LOG_INFO("SLAMEngine", "ROS publisher disabled");
}

bool SLAMEngine::isROSPublisherEnabled() const {
    std::lock_guard<std::mutex> lock(callbackMutex_);
    return rosPublisher_ != nullptr;
}
#endif

#ifdef ENABLE_ZMQ
void SLAMEngine::enableZMQPublisher(const output::ZMQPublisherConfig& config) {
    std::lock_guard<std::mutex> lock(callbackMutex_);

    if (zmqPublisher_) {
        LOG_WARN("SLAMEngine", "ZMQ publisher already enabled");
        return;
    }

    zmqPublisher_ = std::make_unique<output::ZMQPublisher>(config);
    LOG_INFO("SLAMEngine", "ZMQ publisher enabled on {}", config.endpoint);
}

void SLAMEngine::disableZMQPublisher() {
    std::lock_guard<std::mutex> lock(callbackMutex_);

    if (!zmqPublisher_) {
        LOG_WARN("SLAMEngine", "ZMQ publisher not enabled");
        return;
    }

    zmqPublisher_.reset();
    LOG_INFO("SLAMEngine", "ZMQ publisher disabled");
}

bool SLAMEngine::isZMQPublisherEnabled() const {
    std::lock_guard<std::mutex> lock(callbackMutex_);
    return zmqPublisher_ != nullptr;
}

output::ZMQPublisher* SLAMEngine::getZMQPublisher() const {
    std::lock_guard<std::mutex> lock(callbackMutex_);
    return zmqPublisher_.get();
}
#endif

}  // namespace vi_slam
