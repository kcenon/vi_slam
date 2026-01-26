#ifndef VI_SLAM_BASALT_ADAPTER_HPP
#define VI_SLAM_BASALT_ADAPTER_HPP

#include "slam/i_slam_framework.hpp"
#include <memory>
#include <mutex>
#include <deque>

namespace vi_slam {

// Forward declaration of Basalt VIO system
// This would be the actual Basalt VIO system class
class BasaltVioSystem;

// Adapter for Basalt VIO framework
class BasaltAdapter : public ISLAMFramework {
public:
    BasaltAdapter();
    ~BasaltAdapter() override;

    // ISLAMFramework interface implementation
    bool initialize(const std::string& configPath) override;
    bool loadCalibration(const std::string& calibPath) override;

    void processImage(const cv::Mat& image, int64_t timestampNs) override;
    void processIMU(const IMUSample& imu) override;

    bool getPose(Pose6DoF& pose) const override;
    TrackingStatus getStatus() const override;
    std::vector<MapPoint> getMapPoints() const override;

    void reset() override;
    void shutdown() override;

private:
    // Basalt VIO system instance
    std::unique_ptr<BasaltVioSystem> vioSystem_;

    // State management
    TrackingStatus status_;
    mutable std::mutex statusMutex_;

    // Latest pose
    mutable Pose6DoF latestPose_;
    mutable std::mutex poseMutex_;

    // IMU buffer for initialization and synchronization
    std::deque<IMUSample> imuBuffer_;
    mutable std::mutex imuMutex_;

    // Configuration
    std::string configPath_;
    std::string calibPath_;
    bool initialized_;

    // Optical flow specific settings
    int maxFlowPoints_;
    double flowQualityThreshold_;

    // Helper methods
    bool loadConfiguration(const std::string& configPath);
    bool loadCalibrationData(const std::string& calibPath);
    void updateStatus(TrackingStatus newStatus);
    void updatePose(const Pose6DoF& pose);
    void processIMUQueue();
    bool parseBasaltCalibration(const std::string& calibPath);
};

}  // namespace vi_slam

#endif  // VI_SLAM_BASALT_ADAPTER_HPP
