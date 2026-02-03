#ifndef VI_SLAM_I_SLAM_FRAMEWORK_HPP
#define VI_SLAM_I_SLAM_FRAMEWORK_HPP

/**
 * @file i_slam_framework.hpp
 * @brief Abstract interface for SLAM framework adapters
 *
 * This header defines the ISLAMFramework interface that all SLAM
 * backend adapters must implement. The interface provides a common
 * API for initialization, data processing, and output retrieval.
 */

#include "common/types.hpp"
#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace vi_slam {

/**
 * @brief Abstract interface for VI-SLAM framework adapters
 *
 * ISLAMFramework defines the contract that all SLAM backend adapters
 * must fulfill. Each adapter wraps a specific SLAM library (e.g., VINS-Mono,
 * OpenVINS, ORB-SLAM3, Basalt) and translates its API to this common interface.
 *
 * Implementations should be thread-safe for concurrent calls to processImage()
 * and processIMU().
 */
class ISLAMFramework {
public:
    /**
     * @brief Virtual destructor
     */
    virtual ~ISLAMFramework() = default;

    /// @name Initialization
    /// @{

    /**
     * @brief Initialize the SLAM framework with configuration
     * @param configPath Path to the framework-specific configuration file
     * @return true if initialization succeeded
     */
    virtual bool initialize(const std::string& configPath) = 0;

    /**
     * @brief Load camera and IMU calibration parameters
     * @param calibPath Path to the calibration file
     * @return true if calibration loaded successfully
     */
    virtual bool loadCalibration(const std::string& calibPath) = 0;
    /// @}

    /// @name Data Processing
    /// @{

    /**
     * @brief Process a camera image frame
     * @param image Input image (grayscale or color, depending on framework)
     * @param timestampNs Timestamp in nanoseconds since epoch
     */
    virtual void processImage(const cv::Mat& image, int64_t timestampNs) = 0;

    /**
     * @brief Process an IMU measurement
     * @param imu IMU sample containing accelerometer and gyroscope data
     */
    virtual void processIMU(const IMUSample& imu) = 0;
    /// @}

    /// @name Output
    /// @{

    /**
     * @brief Get the latest estimated camera pose
     * @param pose Output 6DoF pose
     * @return true if a valid pose is available
     */
    virtual bool getPose(Pose6DoF& pose) const = 0;

    /**
     * @brief Get the current tracking status
     * @return Current tracking state
     */
    virtual TrackingStatus getStatus() const = 0;

    /**
     * @brief Get all 3D map points from the current map
     * @return Vector of map points
     */
    virtual std::vector<MapPoint> getMapPoints() const = 0;
    /// @}

    /// @name Control
    /// @{

    /**
     * @brief Reset the framework to initial state
     *
     * Clears all internal state including the map and trajectory.
     */
    virtual void reset() = 0;

    /**
     * @brief Shutdown the framework and release resources
     */
    virtual void shutdown() = 0;
    /// @}
};

}  // namespace vi_slam

#endif  // VI_SLAM_I_SLAM_FRAMEWORK_HPP
