#include "slam/output/tf_publisher.hpp"

#ifdef ENABLE_ROS

namespace vi_slam {
namespace output {

TFPublisher::TFPublisher(const TFPublisherConfig& config)
    : config_(config),
      staticTransformsPublished_(false) {

    // Initialize map → odom transform as identity
    mapToOdom_.timestampNs = 0;
    mapToOdom_.position = Eigen::Vector3d::Zero();
    mapToOdom_.orientation = Eigen::Quaterniond::Identity();
    mapToOdom_.valid = true;
}

void TFPublisher::publishDynamicTransforms(const Pose6DoF& pose) {
    if (!pose.valid) {
        return;
    }

    // Publish static transforms on first call
    if (!staticTransformsPublished_) {
        publishStaticTransforms();
        staticTransformsPublished_ = true;
    }

    // Publish map → odom (SLAM correction, usually static)
    geometry_msgs::TransformStamped mapToOdomTf =
        createTransform(mapToOdom_, config_.mapFrame, config_.odomFrame);
    mapToOdomTf.header.stamp = ros::Time(pose.timestampNs / 1e9);
    dynamicBroadcaster_.sendTransform(mapToOdomTf);

    // Publish odom → base_link (dynamic odometry)
    geometry_msgs::TransformStamped odomToBaseLinkTf =
        createTransform(pose, config_.odomFrame, config_.baseLinkFrame);
    dynamicBroadcaster_.sendTransform(odomToBaseLinkTf);
}

void TFPublisher::updateMapToOdomTransform(const Pose6DoF& correction) {
    if (!correction.valid) {
        return;
    }

    mapToOdom_ = correction;
}

void TFPublisher::publishStaticTransforms() {
    ros::Time timestamp = ros::Time::now();

    // base_link → camera_link (identity transform)
    std::array<double, 3> identityTranslation = {{0.0, 0.0, 0.0}};
    std::array<double, 4> identityRotation = {{1.0, 0.0, 0.0, 0.0}};

    geometry_msgs::TransformStamped baseLinkToCameraTf = createStaticTransform(
        identityTranslation,
        identityRotation,
        config_.baseLinkFrame,
        config_.cameraFrame,
        timestamp);

    // camera_link → imu_link (from calibration)
    geometry_msgs::TransformStamped cameraToImuTf = createStaticTransform(
        config_.calibration.translation,
        config_.calibration.rotation,
        config_.cameraFrame,
        config_.imuFrame,
        timestamp);

    // Publish both static transforms
    std::vector<geometry_msgs::TransformStamped> staticTransforms = {
        baseLinkToCameraTf,
        cameraToImuTf
    };
    staticBroadcaster_.sendTransform(staticTransforms);
}

geometry_msgs::TransformStamped TFPublisher::createTransform(
    const Pose6DoF& pose,
    const std::string& frameId,
    const std::string& childFrameId) const {

    geometry_msgs::TransformStamped transform;

    // Header
    transform.header.stamp = ros::Time(pose.timestampNs / 1e9);
    transform.header.frame_id = frameId;
    transform.child_frame_id = childFrameId;

    // Translation
    transform.transform.translation.x = pose.position.x();
    transform.transform.translation.y = pose.position.y();
    transform.transform.translation.z = pose.position.z();

    // Rotation
    transform.transform.rotation.x = pose.orientation.x();
    transform.transform.rotation.y = pose.orientation.y();
    transform.transform.rotation.z = pose.orientation.z();
    transform.transform.rotation.w = pose.orientation.w();

    return transform;
}

geometry_msgs::TransformStamped TFPublisher::createStaticTransform(
    const std::array<double, 3>& translation,
    const std::array<double, 4>& rotation,
    const std::string& frameId,
    const std::string& childFrameId,
    const ros::Time& timestamp) const {

    geometry_msgs::TransformStamped transform;

    // Header
    transform.header.stamp = timestamp;
    transform.header.frame_id = frameId;
    transform.child_frame_id = childFrameId;

    // Translation
    transform.transform.translation.x = translation[0];
    transform.transform.translation.y = translation[1];
    transform.transform.translation.z = translation[2];

    // Rotation (qw, qx, qy, qz → x, y, z, w)
    transform.transform.rotation.x = rotation[1];
    transform.transform.rotation.y = rotation[2];
    transform.transform.rotation.z = rotation[3];
    transform.transform.rotation.w = rotation[0];

    return transform;
}

}  // namespace output
}  // namespace vi_slam

#endif  // ENABLE_ROS
