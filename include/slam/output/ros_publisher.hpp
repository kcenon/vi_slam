#ifndef VI_SLAM_SLAM_OUTPUT_ROS_PUBLISHER_HPP
#define VI_SLAM_SLAM_OUTPUT_ROS_PUBLISHER_HPP

#include "common/types.hpp"

#ifdef ENABLE_ROS

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <memory>
#include <string>
#include <vector>

namespace vi_slam {
namespace output {

/**
 * @brief Configuration for ROS publisher
 */
struct ROSPublisherConfig {
    std::string poseTopicName;      // Default: "/slam/pose"
    std::string odomTopicName;      // Default: "/slam/odom"
    std::string pathTopicName;      // Default: "/slam/path"
    std::string frameId;            // Default: "map"
    std::string childFrameId;       // Default: "base_link"
    int queueSize;                  // Default: 10
    int maxPathLength;              // Default: 1000 (0 = unlimited)

    ROSPublisherConfig()
        : poseTopicName("/slam/pose"),
          odomTopicName("/slam/odom"),
          pathTopicName("/slam/path"),
          frameId("map"),
          childFrameId("base_link"),
          queueSize(10),
          maxPathLength(1000) {}
};

/**
 * @brief Publishes SLAM output to ROS topics
 *
 * This class provides ROS integration for VI-SLAM output, publishing
 * pose information as PoseStamped, Odometry, and Path messages.
 */
class ROSPublisher {
public:
    /**
     * @brief Constructor with ROS node handle
     *
     * @param nodeHandle ROS node handle for creating publishers
     * @param config Publisher configuration
     */
    explicit ROSPublisher(ros::NodeHandle& nodeHandle,
                         const ROSPublisherConfig& config = ROSPublisherConfig());

    /**
     * @brief Destructor
     */
    ~ROSPublisher() = default;

    // Prevent copying
    ROSPublisher(const ROSPublisher&) = delete;
    ROSPublisher& operator=(const ROSPublisher&) = delete;

    /**
     * @brief Publish a new pose
     *
     * Publishes the pose as:
     * - geometry_msgs/PoseStamped on pose topic
     * - nav_msgs/Odometry on odometry topic
     * - Updates nav_msgs/Path on path topic
     *
     * @param pose 6DoF pose with timestamp
     */
    void publishPose(const Pose6DoF& pose);

    /**
     * @brief Clear accumulated path history
     */
    void clearPath();

    /**
     * @brief Get current configuration
     *
     * @return Current publisher configuration
     */
    const ROSPublisherConfig& getConfig() const { return config_; }

    /**
     * @brief Get number of accumulated poses in path
     *
     * @return Path length
     */
    size_t getPathLength() const { return path_.poses.size(); }

private:
    /**
     * @brief Convert Pose6DoF to geometry_msgs/PoseStamped
     *
     * @param pose Input pose
     * @return ROS PoseStamped message
     */
    geometry_msgs::PoseStamped toPoseStamped(const Pose6DoF& pose) const;

    /**
     * @brief Convert Pose6DoF to nav_msgs/Odometry
     *
     * @param pose Input pose
     * @return ROS Odometry message
     */
    nav_msgs::Odometry toOdometry(const Pose6DoF& pose) const;

    /**
     * @brief Append pose to path
     *
     * @param poseStamped Pose to append
     */
    void appendToPath(const geometry_msgs::PoseStamped& poseStamped);

    // ROS publishers
    ros::Publisher posePub_;
    ros::Publisher odomPub_;
    ros::Publisher pathPub_;

    // Configuration
    ROSPublisherConfig config_;

    // Path accumulator
    nav_msgs::Path path_;

    // Previous pose for velocity computation
    bool hasPreviousPose_;
    Pose6DoF previousPose_;
};

}  // namespace output
}  // namespace vi_slam

#endif  // ENABLE_ROS

#endif  // VI_SLAM_SLAM_OUTPUT_ROS_PUBLISHER_HPP
