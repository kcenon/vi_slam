#ifdef ENABLE_ROS

#include "slam/output/tf_publisher.hpp"
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <chrono>
#include <thread>

namespace vi_slam {
namespace output {
namespace test {

class TFPublisherTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Initialize ROS node for testing
        if (!ros::isInitialized()) {
            int argc = 0;
            char** argv = nullptr;
            ros::init(argc, argv, "tf_publisher_test");
        }
        nodeHandle_ = std::make_unique<ros::NodeHandle>();

        // Create TF buffer and listener for verification
        tfBuffer_ = std::make_unique<tf2_ros::Buffer>();
        tfListener_ = std::make_unique<tf2_ros::TransformListener>(*tfBuffer_);
    }

    void TearDown() override {
        tfListener_.reset();
        tfBuffer_.reset();
        nodeHandle_.reset();
    }

    std::unique_ptr<ros::NodeHandle> nodeHandle_;
    std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
    std::unique_ptr<tf2_ros::TransformListener> tfListener_;
};

TEST_F(TFPublisherTest, ConstructorWithDefaultConfig) {
    TFPublisherConfig config;
    TFPublisher publisher(config);

    EXPECT_EQ(publisher.getConfig().mapFrame, "map");
    EXPECT_EQ(publisher.getConfig().odomFrame, "odom");
    EXPECT_EQ(publisher.getConfig().baseLinkFrame, "base_link");
    EXPECT_EQ(publisher.getConfig().cameraFrame, "camera_link");
    EXPECT_EQ(publisher.getConfig().imuFrame, "imu_link");
}

TEST_F(TFPublisherTest, ConstructorWithCustomConfig) {
    TFPublisherConfig config;
    config.mapFrame = "world";
    config.odomFrame = "odom_custom";
    config.baseLinkFrame = "robot";
    config.cameraFrame = "cam";
    config.imuFrame = "imu";

    TFPublisher publisher(config);

    EXPECT_EQ(publisher.getConfig().mapFrame, "world");
    EXPECT_EQ(publisher.getConfig().odomFrame, "odom_custom");
    EXPECT_EQ(publisher.getConfig().baseLinkFrame, "robot");
    EXPECT_EQ(publisher.getConfig().cameraFrame, "cam");
    EXPECT_EQ(publisher.getConfig().imuFrame, "imu");
}

TEST_F(TFPublisherTest, PublishDynamicTransforms) {
    TFPublisher publisher;

    Pose6DoF pose;
    pose.timestampNs = ros::Time::now().toNSec();
    pose.position[0] = 1.0;
    pose.position[1] = 2.0;
    pose.position[2] = 3.0;
    pose.orientation[0] = 1.0;  // qw
    pose.orientation[1] = 0.0;  // qx
    pose.orientation[2] = 0.0;  // qy
    pose.orientation[3] = 0.0;  // qz
    pose.valid = true;

    // Should not throw
    EXPECT_NO_THROW(publisher.publishDynamicTransforms(pose));

    // Allow time for TF to propagate
    ros::spinOnce();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

TEST_F(TFPublisherTest, IgnoreInvalidPose) {
    TFPublisher publisher;

    Pose6DoF pose;
    pose.valid = false;

    // Should not throw, but also should not publish
    EXPECT_NO_THROW(publisher.publishDynamicTransforms(pose));
}

TEST_F(TFPublisherTest, UpdateMapToOdomTransform) {
    TFPublisher publisher;

    Pose6DoF correction;
    correction.timestampNs = ros::Time::now().toNSec();
    correction.position[0] = 0.1;
    correction.position[1] = 0.2;
    correction.position[2] = 0.3;
    correction.orientation[0] = 0.9239;  // qw (small rotation)
    correction.orientation[1] = 0.3827;  // qx
    correction.orientation[2] = 0.0;     // qy
    correction.orientation[3] = 0.0;     // qz
    correction.valid = true;

    // Should not throw
    EXPECT_NO_THROW(publisher.updateMapToOdomTransform(correction));
}

TEST_F(TFPublisherTest, PublishAfterMapCorrection) {
    TFPublisher publisher;

    // Update map → odom transform
    Pose6DoF correction;
    correction.timestampNs = ros::Time::now().toNSec();
    correction.position[0] = 0.5;
    correction.position[1] = 0.0;
    correction.position[2] = 0.0;
    correction.orientation[0] = 1.0;
    correction.orientation[1] = 0.0;
    correction.orientation[2] = 0.0;
    correction.orientation[3] = 0.0;
    correction.valid = true;

    publisher.updateMapToOdomTransform(correction);

    // Publish pose after correction
    Pose6DoF pose;
    pose.timestampNs = ros::Time::now().toNSec();
    pose.position[0] = 1.0;
    pose.position[1] = 0.0;
    pose.position[2] = 0.0;
    pose.orientation[0] = 1.0;
    pose.orientation[1] = 0.0;
    pose.orientation[2] = 0.0;
    pose.orientation[3] = 0.0;
    pose.valid = true;

    EXPECT_NO_THROW(publisher.publishDynamicTransforms(pose));

    ros::spinOnce();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

TEST_F(TFPublisherTest, CustomCalibration) {
    TFPublisherConfig config;
    config.calibration.translation = {{0.05, 0.02, 0.01}};  // 5cm, 2cm, 1cm offset
    config.calibration.rotation = {{1.0, 0.0, 0.0, 0.0}};   // Identity rotation

    TFPublisher publisher(config);

    // Publish to trigger static transform publishing
    Pose6DoF pose;
    pose.timestampNs = ros::Time::now().toNSec();
    pose.position[0] = 0.0;
    pose.position[1] = 0.0;
    pose.position[2] = 0.0;
    pose.orientation[0] = 1.0;
    pose.orientation[1] = 0.0;
    pose.orientation[2] = 0.0;
    pose.orientation[3] = 0.0;
    pose.valid = true;

    EXPECT_NO_THROW(publisher.publishDynamicTransforms(pose));

    ros::spinOnce();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

TEST_F(TFPublisherTest, MultiplePosePublish) {
    TFPublisher publisher;

    for (int i = 0; i < 10; ++i) {
        Pose6DoF pose;
        pose.timestampNs = ros::Time::now().toNSec();
        pose.position[0] = i * 0.1;
        pose.position[1] = i * 0.05;
        pose.position[2] = 0.0;
        pose.orientation[0] = 1.0;
        pose.orientation[1] = 0.0;
        pose.orientation[2] = 0.0;
        pose.orientation[3] = 0.0;
        pose.valid = true;

        EXPECT_NO_THROW(publisher.publishDynamicTransforms(pose));

        ros::spinOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

}  // namespace test
}  // namespace output
}  // namespace vi_slam

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

#else

int main() {
    // ROS not enabled, skip tests
    return 0;
}

#endif  // ENABLE_ROS
