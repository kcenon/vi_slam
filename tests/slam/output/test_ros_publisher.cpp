#ifdef ENABLE_ROS

#include "slam/output/ros_publisher.hpp"
#include <gtest/gtest.h>
#include <ros/ros.h>

namespace vi_slam {
namespace output {
namespace test {

class ROSPublisherTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Initialize ROS node for testing
        if (!ros::isInitialized()) {
            int argc = 0;
            char** argv = nullptr;
            ros::init(argc, argv, "ros_publisher_test");
        }
        nodeHandle_ = std::make_unique<ros::NodeHandle>();
    }

    void TearDown() override {
        nodeHandle_.reset();
    }

    std::unique_ptr<ros::NodeHandle> nodeHandle_;
};

TEST_F(ROSPublisherTest, ConstructorWithDefaultConfig) {
    ROSPublisherConfig config;
    ROSPublisher publisher(*nodeHandle_, config);

    EXPECT_EQ(publisher.getConfig().poseTopicName, "/slam/pose");
    EXPECT_EQ(publisher.getConfig().odomTopicName, "/slam/odom");
    EXPECT_EQ(publisher.getConfig().pathTopicName, "/slam/path");
    EXPECT_EQ(publisher.getConfig().frameId, "map");
    EXPECT_EQ(publisher.getConfig().childFrameId, "base_link");
    EXPECT_EQ(publisher.getConfig().queueSize, 10);
    EXPECT_EQ(publisher.getConfig().maxPathLength, 1000);
}

TEST_F(ROSPublisherTest, ConstructorWithCustomConfig) {
    ROSPublisherConfig config;
    config.poseTopicName = "/custom/pose";
    config.odomTopicName = "/custom/odom";
    config.pathTopicName = "/custom/path";
    config.frameId = "world";
    config.childFrameId = "robot";
    config.queueSize = 20;
    config.maxPathLength = 500;

    ROSPublisher publisher(*nodeHandle_, config);

    EXPECT_EQ(publisher.getConfig().poseTopicName, "/custom/pose");
    EXPECT_EQ(publisher.getConfig().odomTopicName, "/custom/odom");
    EXPECT_EQ(publisher.getConfig().pathTopicName, "/custom/path");
    EXPECT_EQ(publisher.getConfig().frameId, "world");
    EXPECT_EQ(publisher.getConfig().childFrameId, "robot");
    EXPECT_EQ(publisher.getConfig().queueSize, 20);
    EXPECT_EQ(publisher.getConfig().maxPathLength, 500);
}

TEST_F(ROSPublisherTest, PublishValidPose) {
    ROSPublisher publisher(*nodeHandle_);

    Pose6DoF pose;
    pose.timestampNs = 1000000000;  // 1 second
    pose.position[0] = 1.0;
    pose.position[1] = 2.0;
    pose.position[2] = 3.0;
    pose.orientation[0] = 1.0;  // qw
    pose.orientation[1] = 0.0;  // qx
    pose.orientation[2] = 0.0;  // qy
    pose.orientation[3] = 0.0;  // qz
    pose.valid = true;

    // Should not throw
    EXPECT_NO_THROW(publisher.publishPose(pose));

    // Path should have one entry
    EXPECT_EQ(publisher.getPathLength(), 1);
}

TEST_F(ROSPublisherTest, IgnoreInvalidPose) {
    ROSPublisher publisher(*nodeHandle_);

    Pose6DoF pose;
    pose.valid = false;

    publisher.publishPose(pose);

    // Path should remain empty
    EXPECT_EQ(publisher.getPathLength(), 0);
}

TEST_F(ROSPublisherTest, AccumulatePath) {
    ROSPublisher publisher(*nodeHandle_);

    for (int i = 0; i < 5; ++i) {
        Pose6DoF pose;
        pose.timestampNs = (i + 1) * 1000000000;
        pose.position[0] = i * 1.0;
        pose.position[1] = i * 2.0;
        pose.position[2] = i * 3.0;
        pose.orientation[0] = 1.0;
        pose.valid = true;

        publisher.publishPose(pose);
    }

    EXPECT_EQ(publisher.getPathLength(), 5);
}

TEST_F(ROSPublisherTest, EnforceMaxPathLength) {
    ROSPublisherConfig config;
    config.maxPathLength = 3;

    ROSPublisher publisher(*nodeHandle_, config);

    for (int i = 0; i < 5; ++i) {
        Pose6DoF pose;
        pose.timestampNs = (i + 1) * 1000000000;
        pose.position[0] = i * 1.0;
        pose.valid = true;

        publisher.publishPose(pose);
    }

    // Should not exceed maxPathLength
    EXPECT_EQ(publisher.getPathLength(), 3);
}

TEST_F(ROSPublisherTest, ClearPath) {
    ROSPublisher publisher(*nodeHandle_);

    for (int i = 0; i < 3; ++i) {
        Pose6DoF pose;
        pose.timestampNs = (i + 1) * 1000000000;
        pose.valid = true;

        publisher.publishPose(pose);
    }

    EXPECT_EQ(publisher.getPathLength(), 3);

    publisher.clearPath();

    EXPECT_EQ(publisher.getPathLength(), 0);
}

TEST_F(ROSPublisherTest, UnlimitedPathLength) {
    ROSPublisherConfig config;
    config.maxPathLength = 0;  // Unlimited

    ROSPublisher publisher(*nodeHandle_, config);

    for (int i = 0; i < 2000; ++i) {
        Pose6DoF pose;
        pose.timestampNs = (i + 1) * 1000000000;
        pose.valid = true;

        publisher.publishPose(pose);
    }

    EXPECT_EQ(publisher.getPathLength(), 2000);
}

}  // namespace test
}  // namespace output
}  // namespace vi_slam

#endif  // ENABLE_ROS
