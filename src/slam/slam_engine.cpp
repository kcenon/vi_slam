#include "slam/slam_engine.hpp"
#include "slam/adapters/vins_mono_adapter.hpp"
#include <iostream>
#include <stdexcept>

namespace vi_slam {

SLAMEngine::SLAMEngine()
    : framework_(nullptr),
      currentType_(SLAMFrameworkType::VINS_MONO),
      poseCallback_(nullptr),
      statusCallback_(nullptr),
      initialized_(false) {
    std::cout << "SLAMEngine created" << std::endl;
}

SLAMEngine::~SLAMEngine() {
    shutdown();
    std::cout << "SLAMEngine destroyed" << std::endl;
}

bool SLAMEngine::selectFramework(SLAMFrameworkType type) {
    std::lock_guard<std::mutex> lock(frameworkMutex_);

    // If already using this framework, no need to switch
    if (framework_ && currentType_ == type) {
        std::cout << "Already using selected framework" << std::endl;
        return true;
    }

    // Shutdown current framework if exists
    if (framework_) {
        framework_->shutdown();
    }

    // Create new framework
    framework_ = createFramework(type);
    if (!framework_) {
        std::cerr << "Failed to create framework" << std::endl;
        return false;
    }

    currentType_ = type;
    initialized_ = false;

    std::cout << "Framework selected: " << static_cast<int>(type) << std::endl;
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
            std::cerr << "Failed to create default framework" << std::endl;
            return false;
        }
    }

    if (!framework_->initialize(configPath)) {
        std::cerr << "Framework initialization failed" << std::endl;
        return false;
    }

    initialized_ = true;
    std::cout << "SLAMEngine initialized successfully" << std::endl;
    return true;
}

bool SLAMEngine::loadCalibration(const std::string& calibPath) {
    std::lock_guard<std::mutex> lock(frameworkMutex_);

    if (!framework_) {
        std::cerr << "No framework selected" << std::endl;
        return false;
    }

    if (!framework_->loadCalibration(calibPath)) {
        std::cerr << "Failed to load calibration" << std::endl;
        return false;
    }

    std::cout << "Calibration loaded successfully" << std::endl;
    return true;
}

void SLAMEngine::processImage(const cv::Mat& image, int64_t timestampNs) {
    std::lock_guard<std::mutex> lock(frameworkMutex_);

    if (!framework_) {
        std::cerr << "No framework available for image processing" << std::endl;
        return;
    }

    if (!initialized_) {
        std::cerr << "Framework not initialized" << std::endl;
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
        std::cerr << "No framework available for IMU processing" << std::endl;
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
        std::cerr << "No framework available" << std::endl;
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
        std::cerr << "No framework to reset" << std::endl;
        return;
    }

    framework_->reset();
    invokeStatusCallback(TrackingStatus::INITIALIZING);

    std::cout << "SLAMEngine reset" << std::endl;
}

void SLAMEngine::shutdown() {
    std::lock_guard<std::mutex> lock(frameworkMutex_);

    if (framework_) {
        framework_->shutdown();
        framework_.reset();
    }

    initialized_ = false;
    invokeStatusCallback(TrackingStatus::UNINITIALIZED);

    std::cout << "SLAMEngine shut down" << std::endl;
}

void SLAMEngine::setPoseCallback(PoseCallback callback) {
    std::lock_guard<std::mutex> lock(callbackMutex_);
    poseCallback_ = callback;
    std::cout << "Pose callback set" << std::endl;
}

void SLAMEngine::setStatusCallback(StatusCallback callback) {
    std::lock_guard<std::mutex> lock(callbackMutex_);
    statusCallback_ = callback;
    std::cout << "Status callback set" << std::endl;
}

std::unique_ptr<ISLAMFramework> SLAMEngine::createFramework(SLAMFrameworkType type) {
    switch (type) {
        case SLAMFrameworkType::VINS_MONO:
            std::cout << "Creating VINS-Mono adapter" << std::endl;
            return std::make_unique<VINSMonoAdapter>();

        case SLAMFrameworkType::OPENVINS:
            // TODO: Implement OpenVINS adapter
            std::cerr << "OpenVINS adapter not yet implemented" << std::endl;
            return nullptr;

        case SLAMFrameworkType::ORB_SLAM3:
            // TODO: Implement ORB-SLAM3 adapter
            std::cerr << "ORB-SLAM3 adapter not yet implemented" << std::endl;
            return nullptr;

        case SLAMFrameworkType::BASALT:
            // TODO: Implement Basalt adapter
            std::cerr << "Basalt adapter not yet implemented" << std::endl;
            return nullptr;

        default:
            std::cerr << "Unknown framework type" << std::endl;
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
        std::cerr << "ROS publisher already enabled" << std::endl;
        return;
    }

    rosPublisher_ = std::make_unique<output::ROSPublisher>(nodeHandle, config);
    std::cout << "ROS publisher enabled" << std::endl;
}

void SLAMEngine::disableROSPublisher() {
    std::lock_guard<std::mutex> lock(callbackMutex_);

    if (!rosPublisher_) {
        std::cerr << "ROS publisher not enabled" << std::endl;
        return;
    }

    rosPublisher_.reset();
    std::cout << "ROS publisher disabled" << std::endl;
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
        std::cerr << "ZMQ publisher already enabled" << std::endl;
        return;
    }

    zmqPublisher_ = std::make_unique<output::ZMQPublisher>(config);
    std::cout << "ZMQ publisher enabled on " << config.endpoint << std::endl;
}

void SLAMEngine::disableZMQPublisher() {
    std::lock_guard<std::mutex> lock(callbackMutex_);

    if (!zmqPublisher_) {
        std::cerr << "ZMQ publisher not enabled" << std::endl;
        return;
    }

    zmqPublisher_.reset();
    std::cout << "ZMQ publisher disabled" << std::endl;
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
