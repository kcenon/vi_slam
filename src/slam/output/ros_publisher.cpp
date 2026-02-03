#include "slam/output/ros_publisher.hpp"

#ifdef ENABLE_ROS

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace vi_slam {
namespace output {

ROSPublisher::ROSPublisher(ros::NodeHandle& nodeHandle,
                          const ROSPublisherConfig& config)
    : config_(config),
      hasPreviousPose_(false) {

    // Create publishers
    posePub_ = nodeHandle.advertise<geometry_msgs::PoseStamped>(
        config_.poseTopicName, config_.queueSize);

    odomPub_ = nodeHandle.advertise<nav_msgs::Odometry>(
        config_.odomTopicName, config_.queueSize);

    pathPub_ = nodeHandle.advertise<nav_msgs::Path>(
        config_.pathTopicName, config_.queueSize);

    // Initialize TF publisher if enabled
    if (config_.enableTF) {
        tfPublisher_ = std::make_unique<TFPublisher>(config_.tfConfig);
    }

    // Initialize path header
    path_.header.frame_id = config_.frameId;
}

void ROSPublisher::publishPose(const Pose6DoF& pose) {
    if (!pose.valid) {
        return;
    }

    // Convert to ROS messages
    geometry_msgs::PoseStamped poseMsg = toPoseStamped(pose);
    nav_msgs::Odometry odomMsg = toOdometry(pose);

    // Publish individual messages
    posePub_.publish(poseMsg);
    odomPub_.publish(odomMsg);

    // Update and publish path
    appendToPath(poseMsg);
    path_.header.stamp = poseMsg.header.stamp;
    pathPub_.publish(path_);

    // Publish TF transforms if enabled
    if (tfPublisher_) {
        tfPublisher_->publishDynamicTransforms(pose);
    }

    // Store for next iteration
    previousPose_ = pose;
    hasPreviousPose_ = true;
}

void ROSPublisher::clearPath() {
    path_.poses.clear();
}

geometry_msgs::PoseStamped ROSPublisher::toPoseStamped(const Pose6DoF& pose) const {
    geometry_msgs::PoseStamped msg;

    // Header
    msg.header.stamp = ros::Time(pose.timestampNs / 1e9);  // Convert ns to seconds
    msg.header.frame_id = config_.frameId;

    // Position
    msg.pose.position.x = pose.position.x();
    msg.pose.position.y = pose.position.y();
    msg.pose.position.z = pose.position.z();

    // Orientation
    msg.pose.orientation.x = pose.orientation.x();
    msg.pose.orientation.y = pose.orientation.y();
    msg.pose.orientation.z = pose.orientation.z();
    msg.pose.orientation.w = pose.orientation.w();

    return msg;
}

nav_msgs::Odometry ROSPublisher::toOdometry(const Pose6DoF& pose) const {
    nav_msgs::Odometry msg;

    // Header
    msg.header.stamp = ros::Time(pose.timestampNs / 1e9);
    msg.header.frame_id = config_.frameId;
    msg.child_frame_id = config_.childFrameId;

    // Pose
    msg.pose.pose.position.x = pose.position.x();
    msg.pose.pose.position.y = pose.position.y();
    msg.pose.pose.position.z = pose.position.z();
    msg.pose.pose.orientation.x = pose.orientation.x();
    msg.pose.pose.orientation.y = pose.orientation.y();
    msg.pose.pose.orientation.z = pose.orientation.z();
    msg.pose.pose.orientation.w = pose.orientation.w();

    // Pose covariance (6x6 matrix)
    Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(msg.pose.covariance.data()) = pose.covariance;

    // Twist (velocity) computation
    if (hasPreviousPose_) {
        double dt = (pose.timestampNs - previousPose_.timestampNs) / 1e9;  // seconds

        if (dt > 0) {
            // Linear velocity
            Eigen::Vector3d velocity = (pose.position - previousPose_.position) / dt;
            msg.twist.twist.linear.x = velocity.x();
            msg.twist.twist.linear.y = velocity.y();
            msg.twist.twist.linear.z = velocity.z();

            // Angular velocity computation would require quaternion differentiation
            // For simplicity, set to zero (can be enhanced later)
            msg.twist.twist.angular.x = 0.0;
            msg.twist.twist.angular.y = 0.0;
            msg.twist.twist.angular.z = 0.0;
        }
    }

    return msg;
}

void ROSPublisher::appendToPath(const geometry_msgs::PoseStamped& poseStamped) {
    path_.poses.push_back(poseStamped);

    // Enforce maximum path length if configured
    if (config_.maxPathLength > 0 &&
        path_.poses.size() > static_cast<size_t>(config_.maxPathLength)) {
        path_.poses.erase(path_.poses.begin());
    }
}

}  // namespace output
}  // namespace vi_slam

#endif  // ENABLE_ROS
