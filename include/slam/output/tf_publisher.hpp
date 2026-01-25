#ifndef VI_SLAM_SLAM_OUTPUT_TF_PUBLISHER_HPP
#define VI_SLAM_SLAM_OUTPUT_TF_PUBLISHER_HPP

#include "common/types.hpp"

#ifdef ENABLE_ROS

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <memory>
#include <string>
#include <array>

namespace vi_slam {
namespace output {

/**
 * @brief Static calibration between camera and IMU
 *
 * This represents the fixed transformation from camera frame to IMU frame.
 */
struct CameraIMUCalibration {
    std::array<double, 3> translation;  // x, y, z in meters
    std::array<double, 4> rotation;     // qw, qx, qy, qz

    CameraIMUCalibration()
        : translation{{0.0, 0.0, 0.0}},
          rotation{{1.0, 0.0, 0.0, 0.0}} {}  // Identity rotation
};

/**
 * @brief Configuration for TF tree publisher
 */
struct TFPublisherConfig {
    std::string mapFrame;           // Default: "map"
    std::string odomFrame;          // Default: "odom"
    std::string baseLinkFrame;      // Default: "base_link"
    std::string cameraFrame;        // Default: "camera_link"
    std::string imuFrame;           // Default: "imu_link"

    CameraIMUCalibration calibration;

    TFPublisherConfig()
        : mapFrame("map"),
          odomFrame("odom"),
          baseLinkFrame("base_link"),
          cameraFrame("camera_link"),
          imuFrame("imu_link") {}
};

/**
 * @brief Publishes TF tree for coordinate frame transformations
 *
 * TF tree structure:
 *   map → odom → base_link → camera_link → imu_link
 *
 * - map → odom: SLAM correction (static, initially identity)
 * - odom → base_link: Visual-inertial odometry (dynamic)
 * - base_link → camera_link: Static calibration (identity by default)
 * - camera_link → imu_link: Static calibration (from config)
 */
class TFPublisher {
public:
    /**
     * @brief Constructor
     *
     * @param config TF publisher configuration
     */
    explicit TFPublisher(const TFPublisherConfig& config = TFPublisherConfig());

    /**
     * @brief Destructor
     */
    ~TFPublisher() = default;

    // Prevent copying
    TFPublisher(const TFPublisher&) = delete;
    TFPublisher& operator=(const TFPublisher&) = delete;

    /**
     * @brief Publish dynamic transforms for a new pose
     *
     * Publishes odom → base_link transform based on the SLAM pose.
     * The map → odom transform is kept constant (identity by default).
     *
     * @param pose Current SLAM pose (in map frame)
     */
    void publishDynamicTransforms(const Pose6DoF& pose);

    /**
     * @brief Update SLAM correction transform
     *
     * Updates the map → odom transform to reflect SLAM corrections.
     * This is typically updated during loop closure or relocalization.
     *
     * @param correction Transform from map to odom frame
     */
    void updateMapToOdomTransform(const Pose6DoF& correction);

    /**
     * @brief Get current configuration
     *
     * @return Current TF publisher configuration
     */
    const TFPublisherConfig& getConfig() const { return config_; }

private:
    /**
     * @brief Publish static transforms
     *
     * Publishes:
     * - base_link → camera_link (identity)
     * - camera_link → imu_link (from calibration)
     */
    void publishStaticTransforms();

    /**
     * @brief Create transform message from pose
     *
     * @param pose Source pose
     * @param frameId Parent frame ID
     * @param childFrameId Child frame ID
     * @return TransformStamped message
     */
    geometry_msgs::TransformStamped createTransform(
        const Pose6DoF& pose,
        const std::string& frameId,
        const std::string& childFrameId) const;

    /**
     * @brief Create static transform message
     *
     * @param translation Translation [x, y, z]
     * @param rotation Rotation quaternion [qw, qx, qy, qz]
     * @param frameId Parent frame ID
     * @param childFrameId Child frame ID
     * @param timestamp Timestamp for the transform
     * @return TransformStamped message
     */
    geometry_msgs::TransformStamped createStaticTransform(
        const std::array<double, 3>& translation,
        const std::array<double, 4>& rotation,
        const std::string& frameId,
        const std::string& childFrameId,
        const ros::Time& timestamp) const;

    // TF broadcasters
    tf2_ros::TransformBroadcaster dynamicBroadcaster_;
    tf2_ros::StaticTransformBroadcaster staticBroadcaster_;

    // Configuration
    TFPublisherConfig config_;

    // Current map → odom transform (for SLAM correction)
    Pose6DoF mapToOdom_;

    // Flag to track if static transforms have been published
    bool staticTransformsPublished_;
};

}  // namespace output
}  // namespace vi_slam

#endif  // ENABLE_ROS

#endif  // VI_SLAM_SLAM_OUTPUT_TF_PUBLISHER_HPP
