#ifndef VI_SLAM_SLAM_ENGINE_HPP
#define VI_SLAM_SLAM_ENGINE_HPP

/**
 * @file slam_engine.hpp
 * @brief Main SLAM engine facade for VI-SLAM system
 *
 * This header defines the SLAMEngine class, which provides a unified
 * interface for different SLAM framework backends (VINS-Mono, OpenVINS,
 * ORB-SLAM3, Basalt).
 */

#include "slam/i_slam_framework.hpp"
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#ifdef ENABLE_ROS
#include "slam/output/ros_publisher.hpp"
#include <ros/ros.h>
#endif

#ifdef ENABLE_ZMQ
#include "slam/output/zmq_publisher.hpp"
#endif

namespace vi_slam {

/**
 * @brief Supported SLAM framework types
 *
 * Enumeration of available visual-inertial SLAM backends.
 */
enum class SLAMFrameworkType {
    VINS_MONO,   ///< VINS-Mono: Monocular visual-inertial SLAM
    OPENVINS,    ///< OpenVINS: MSCKF-based VIO system
    ORB_SLAM3,   ///< ORB-SLAM3: Feature-based visual SLAM
    BASALT       ///< Basalt: Optical flow based VIO
};

/// Callback type for pose updates
using PoseCallback = std::function<void(const Pose6DoF&)>;

/// Callback type for tracking status changes
using StatusCallback = std::function<void(TrackingStatus)>;

/**
 * @brief Main SLAM engine facade managing framework lifecycle
 *
 * SLAMEngine provides a unified interface for visual-inertial SLAM
 * processing. It supports multiple backend frameworks and provides
 * optional output to ROS topics and ZMQ sockets.
 *
 * @code{.cpp}
 * vi_slam::SLAMEngine engine;
 * engine.selectFramework(vi_slam::SLAMFrameworkType::OPENVINS);
 * engine.initialize("config/openvins.yaml");
 *
 * // Process sensor data
 * engine.processIMU(imuSample);
 * engine.processImage(frame, timestampNs);
 *
 * // Get current pose
 * vi_slam::Pose6DoF pose;
 * if (engine.getPose(pose)) {
 *     // Use pose...
 * }
 * @endcode
 */
class SLAMEngine {
public:
    /**
     * @brief Default constructor
     */
    SLAMEngine();

    /**
     * @brief Destructor - shuts down the engine and releases resources
     */
    ~SLAMEngine();

    /// @name Non-copyable
    /// @{
    SLAMEngine(const SLAMEngine&) = delete;
    SLAMEngine& operator=(const SLAMEngine&) = delete;
    /// @}

    /// @name Framework Selection
    /// @{

    /**
     * @brief Select and instantiate a SLAM framework backend
     * @param type The framework type to use
     * @return true if framework was created successfully
     */
    bool selectFramework(SLAMFrameworkType type);

    /**
     * @brief Get the currently selected framework type
     * @return Current framework type
     */
    SLAMFrameworkType getCurrentFramework() const;
    /// @}

    /// @name Initialization
    /// @{

    /**
     * @brief Initialize the SLAM engine with configuration
     * @param configPath Path to the framework configuration file
     * @return true if initialization succeeded
     */
    bool initialize(const std::string& configPath);

    /**
     * @brief Load camera and IMU calibration parameters
     * @param calibPath Path to the calibration file
     * @return true if calibration loaded successfully
     */
    bool loadCalibration(const std::string& calibPath);
    /// @}

    /// @name Data Processing
    /// @{

    /**
     * @brief Process a camera image frame
     * @param image Input image (OpenCV Mat)
     * @param timestampNs Timestamp in nanoseconds
     */
    void processImage(const cv::Mat& image, int64_t timestampNs);

    /**
     * @brief Process an IMU measurement
     * @param imu IMU sample with accelerometer and gyroscope data
     */
    void processIMU(const IMUSample& imu);
    /// @}

    /// @name Output
    /// @{

    /**
     * @brief Get the latest estimated pose
     * @param pose Output pose structure
     * @return true if a valid pose is available
     */
    bool getPose(Pose6DoF& pose) const;

    /**
     * @brief Get current tracking status
     * @return Current tracking status
     */
    TrackingStatus getStatus() const;

    /**
     * @brief Get all map points from the current map
     * @return Vector of 3D map points
     */
    std::vector<MapPoint> getMapPoints() const;
    /// @}

    /// @name Control
    /// @{

    /**
     * @brief Reset the SLAM system to initial state
     */
    void reset();

    /**
     * @brief Shutdown the SLAM engine and release resources
     */
    void shutdown();
    /// @}

    /// @name Callbacks
    /// @{

    /**
     * @brief Set callback for pose updates
     * @param callback Function to call when new pose is available
     */
    void setPoseCallback(PoseCallback callback);

    /**
     * @brief Set callback for status changes
     * @param callback Function to call when tracking status changes
     */
    void setStatusCallback(StatusCallback callback);
    /// @}

#ifdef ENABLE_ROS
    // ROS integration
    void enableROSPublisher(ros::NodeHandle& nodeHandle,
                          const output::ROSPublisherConfig& config = output::ROSPublisherConfig());
    void disableROSPublisher();
    bool isROSPublisherEnabled() const;
#endif

#ifdef ENABLE_ZMQ
    // ZMQ integration
    void enableZMQPublisher(const output::ZMQPublisherConfig& config = output::ZMQPublisherConfig());
    void disableZMQPublisher();
    bool isZMQPublisherEnabled() const;
    output::ZMQPublisher* getZMQPublisher() const;
#endif

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

#ifdef ENABLE_ROS
    // ROS publisher
    std::unique_ptr<output::ROSPublisher> rosPublisher_;
#endif

#ifdef ENABLE_ZMQ
    // ZMQ publisher
    std::unique_ptr<output::ZMQPublisher> zmqPublisher_;
#endif
};

}  // namespace vi_slam

#endif  // VI_SLAM_SLAM_ENGINE_HPP
