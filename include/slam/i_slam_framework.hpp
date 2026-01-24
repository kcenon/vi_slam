#ifndef VI_SLAM_I_SLAM_FRAMEWORK_HPP
#define VI_SLAM_I_SLAM_FRAMEWORK_HPP

#include "common/types.hpp"
#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace vi_slam {

// Interface for VI-SLAM framework adapters
class ISLAMFramework {
public:
    virtual ~ISLAMFramework() = default;

    // Initialization
    virtual bool initialize(const std::string& configPath) = 0;
    virtual bool loadCalibration(const std::string& calibPath) = 0;

    // Data processing
    virtual void processImage(const cv::Mat& image, int64_t timestampNs) = 0;
    virtual void processIMU(const IMUSample& imu) = 0;

    // Output
    virtual bool getPose(Pose6DoF& pose) const = 0;
    virtual TrackingStatus getStatus() const = 0;
    virtual std::vector<MapPoint> getMapPoints() const = 0;

    // Control
    virtual void reset() = 0;
    virtual void shutdown() = 0;
};

}  // namespace vi_slam

#endif  // VI_SLAM_I_SLAM_FRAMEWORK_HPP
