#ifndef VI_SLAM_SLAM_ENGINE_HPP
#define VI_SLAM_SLAM_ENGINE_HPP

#include "slam/i_slam_framework.hpp"
#include <functional>
#include <memory>
#include <mutex>
#include <string>

namespace vi_slam {

// Forward declarations
enum class SLAMFrameworkType {
    VINS_MONO,
    OPENVINS,
    ORB_SLAM3,
    BASALT
};

// Callback type for pose output
using PoseCallback = std::function<void(const Pose6DoF&)>;
using StatusCallback = std::function<void(TrackingStatus)>;

// SLAMEngine facade class managing SLAM framework lifecycle
class SLAMEngine {
public:
    SLAMEngine();
    ~SLAMEngine();

    // Prevent copying
    SLAMEngine(const SLAMEngine&) = delete;
    SLAMEngine& operator=(const SLAMEngine&) = delete;

    // Framework selection and switching
    bool selectFramework(SLAMFrameworkType type);
    SLAMFrameworkType getCurrentFramework() const;

    // Initialization
    bool initialize(const std::string& configPath);
    bool loadCalibration(const std::string& calibPath);

    // Data processing (delegates to current framework)
    void processImage(const cv::Mat& image, int64_t timestampNs);
    void processIMU(const IMUSample& imu);

    // Output
    bool getPose(Pose6DoF& pose) const;
    TrackingStatus getStatus() const;
    std::vector<MapPoint> getMapPoints() const;

    // Control
    void reset();
    void shutdown();

    // Callbacks
    void setPoseCallback(PoseCallback callback);
    void setStatusCallback(StatusCallback callback);

private:
    // Framework factory method
    std::unique_ptr<ISLAMFramework> createFramework(SLAMFrameworkType type);

    // Invoke callbacks
    void invokePoseCallback(const Pose6DoF& pose);
    void invokeStatusCallback(TrackingStatus status);

    // Current framework instance
    std::unique_ptr<ISLAMFramework> framework_;
    SLAMFrameworkType currentType_;

    // Callbacks
    PoseCallback poseCallback_;
    StatusCallback statusCallback_;

    // Thread safety
    mutable std::mutex frameworkMutex_;
    mutable std::mutex callbackMutex_;

    // State flags
    bool initialized_;
};

}  // namespace vi_slam

#endif  // VI_SLAM_SLAM_ENGINE_HPP
