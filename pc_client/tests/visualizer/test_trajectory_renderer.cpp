#include "visualizer/trajectory_renderer.hpp"
#include "visualizer/renderer.hpp"
#include <gtest/gtest.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

using namespace vi_slam::visualizer;

class TrajectoryRendererTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Initialize base renderer for OpenGL context
        Renderer::Config config;
        config.width = 800;
        config.height = 600;
        config.title = "Trajectory Renderer Test";

        ASSERT_TRUE(baseRenderer_.initialize(config));
    }

    void TearDown() override {
        baseRenderer_.shutdown();
    }

    Renderer baseRenderer_;
};

TEST_F(TrajectoryRendererTest, Initialization) {
    TrajectoryRenderer renderer;

    EXPECT_FALSE(renderer.isInitialized());
    EXPECT_EQ(renderer.getPoseCount(), 0);

    TrajectoryRenderer::Config config;
    EXPECT_TRUE(renderer.initialize(config));
    EXPECT_TRUE(renderer.isInitialized());

    renderer.shutdown();
    EXPECT_FALSE(renderer.isInitialized());
}

TEST_F(TrajectoryRendererTest, CustomConfiguration) {
    TrajectoryRenderer renderer;

    TrajectoryRenderer::Config config;
    config.lineWidth = 3.0f;
    config.frustumSize = 0.2f;
    config.frustumInterval = 5;
    config.goodTrackColor = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
    config.poorTrackColor = Eigen::Vector3f(1.0f, 0.0f, 0.0f);

    EXPECT_TRUE(renderer.initialize(config));
    EXPECT_TRUE(renderer.isInitialized());
}

TEST_F(TrajectoryRendererTest, UpdateTrajectory) {
    TrajectoryRenderer renderer;
    TrajectoryRenderer::Config config;
    ASSERT_TRUE(renderer.initialize(config));

    // Create test trajectory (straight line)
    std::vector<TrajectoryRenderer::Pose> poses;
    for (int i = 0; i < 50; ++i) {
        TrajectoryRenderer::Pose pose;
        pose.transform = Eigen::Isometry3d::Identity();
        pose.transform.translate(Eigen::Vector3d(i * 0.1, 0.0, 0.0));
        pose.trackingQuality = 1.0f; // Perfect tracking
        poses.push_back(pose);
    }

    renderer.updateTrajectory(poses);
    EXPECT_EQ(renderer.getPoseCount(), 50);
}

TEST_F(TrajectoryRendererTest, UpdateTrajectoryWithVaryingQuality) {
    TrajectoryRenderer renderer;
    TrajectoryRenderer::Config config;
    ASSERT_TRUE(renderer.initialize(config));

    // Create test trajectory with varying tracking quality
    std::vector<TrajectoryRenderer::Pose> poses;
    for (int i = 0; i < 100; ++i) {
        TrajectoryRenderer::Pose pose;
        pose.transform = Eigen::Isometry3d::Identity();
        pose.transform.translate(Eigen::Vector3d(
            i * 0.1,
            std::sin(i * 0.1) * 2.0,
            std::cos(i * 0.1) * 2.0
        ));
        // Quality varies sinusoidally between 0.2 and 1.0
        pose.trackingQuality = 0.6f + 0.4f * std::sin(i * 0.1);
        poses.push_back(pose);
    }

    renderer.updateTrajectory(poses);
    EXPECT_EQ(renderer.getPoseCount(), 100);
}

TEST_F(TrajectoryRendererTest, UpdateEmptyTrajectory) {
    TrajectoryRenderer renderer;
    TrajectoryRenderer::Config config;
    ASSERT_TRUE(renderer.initialize(config));

    // Update with empty trajectory
    std::vector<TrajectoryRenderer::Pose> poses;
    renderer.updateTrajectory(poses);
    EXPECT_EQ(renderer.getPoseCount(), 0);
}

TEST_F(TrajectoryRendererTest, Clear) {
    TrajectoryRenderer renderer;
    TrajectoryRenderer::Config config;
    ASSERT_TRUE(renderer.initialize(config));

    // Create and update trajectory
    std::vector<TrajectoryRenderer::Pose> poses;
    for (int i = 0; i < 20; ++i) {
        TrajectoryRenderer::Pose pose;
        pose.transform = Eigen::Isometry3d::Identity();
        pose.transform.translate(Eigen::Vector3d(i * 0.1, 0.0, 0.0));
        pose.trackingQuality = 1.0f;
        poses.push_back(pose);
    }

    renderer.updateTrajectory(poses);
    EXPECT_EQ(renderer.getPoseCount(), 20);

    // Clear trajectory
    renderer.clear();
    EXPECT_EQ(renderer.getPoseCount(), 0);
}

TEST_F(TrajectoryRendererTest, RenderEmptyTrajectory) {
    TrajectoryRenderer renderer;
    TrajectoryRenderer::Config config;
    ASSERT_TRUE(renderer.initialize(config));

    // Rendering empty trajectory should not crash
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f proj = Eigen::Matrix4f::Identity();

    EXPECT_NO_THROW(renderer.render(view, proj));
}

TEST_F(TrajectoryRendererTest, RenderTrajectory) {
    TrajectoryRenderer renderer;
    TrajectoryRenderer::Config config;
    ASSERT_TRUE(renderer.initialize(config));

    // Create test trajectory
    std::vector<TrajectoryRenderer::Pose> poses;
    for (int i = 0; i < 30; ++i) {
        TrajectoryRenderer::Pose pose;
        pose.transform = Eigen::Isometry3d::Identity();
        pose.transform.translate(Eigen::Vector3d(i * 0.1, 0.0, 0.0));
        pose.trackingQuality = 1.0f;
        poses.push_back(pose);
    }

    renderer.updateTrajectory(poses);

    // Get view and projection matrices from base renderer
    Eigen::Matrix4f view = baseRenderer_.getViewMatrix();
    Eigen::Matrix4f proj = baseRenderer_.getProjectionMatrix();

    // Render should not crash
    baseRenderer_.beginFrame();
    EXPECT_NO_THROW(renderer.render(view, proj));
    baseRenderer_.endFrame();
}

TEST_F(TrajectoryRendererTest, SetLineWidth) {
    TrajectoryRenderer renderer;
    TrajectoryRenderer::Config config;
    ASSERT_TRUE(renderer.initialize(config));

    // Set custom line width
    renderer.setLineWidth(4.0f);

    // Create and render trajectory
    std::vector<TrajectoryRenderer::Pose> poses;
    for (int i = 0; i < 10; ++i) {
        TrajectoryRenderer::Pose pose;
        pose.transform = Eigen::Isometry3d::Identity();
        pose.transform.translate(Eigen::Vector3d(i * 0.1, 0.0, 0.0));
        pose.trackingQuality = 1.0f;
        poses.push_back(pose);
    }

    renderer.updateTrajectory(poses);

    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f proj = Eigen::Matrix4f::Identity();

    EXPECT_NO_THROW(renderer.render(view, proj));
}

TEST_F(TrajectoryRendererTest, SetFrustumSize) {
    TrajectoryRenderer renderer;
    TrajectoryRenderer::Config config;
    ASSERT_TRUE(renderer.initialize(config));

    // Set custom frustum size
    renderer.setFrustumSize(0.3f);

    // Create and render trajectory
    std::vector<TrajectoryRenderer::Pose> poses;
    for (int i = 0; i < 10; ++i) {
        TrajectoryRenderer::Pose pose;
        pose.transform = Eigen::Isometry3d::Identity();
        pose.transform.translate(Eigen::Vector3d(i * 0.1, 0.0, 0.0));
        pose.trackingQuality = 1.0f;
        poses.push_back(pose);
    }

    renderer.updateTrajectory(poses);

    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f proj = Eigen::Matrix4f::Identity();

    EXPECT_NO_THROW(renderer.render(view, proj));
}

TEST_F(TrajectoryRendererTest, UpdateMultipleTimes) {
    TrajectoryRenderer renderer;
    TrajectoryRenderer::Config config;
    ASSERT_TRUE(renderer.initialize(config));

    // First trajectory
    std::vector<TrajectoryRenderer::Pose> poses1;
    for (int i = 0; i < 20; ++i) {
        TrajectoryRenderer::Pose pose;
        pose.transform = Eigen::Isometry3d::Identity();
        pose.transform.translate(Eigen::Vector3d(i * 0.1, 0.0, 0.0));
        pose.trackingQuality = 1.0f;
        poses1.push_back(pose);
    }

    renderer.updateTrajectory(poses1);
    EXPECT_EQ(renderer.getPoseCount(), 20);

    // Second trajectory (should replace first)
    std::vector<TrajectoryRenderer::Pose> poses2;
    for (int i = 0; i < 50; ++i) {
        TrajectoryRenderer::Pose pose;
        pose.transform = Eigen::Isometry3d::Identity();
        pose.transform.translate(Eigen::Vector3d(i * 0.05, 0.0, 0.0));
        pose.trackingQuality = 0.8f;
        poses2.push_back(pose);
    }

    renderer.updateTrajectory(poses2);
    EXPECT_EQ(renderer.getPoseCount(), 50);
}

TEST_F(TrajectoryRendererTest, TrajectoryWithRotations) {
    TrajectoryRenderer renderer;
    TrajectoryRenderer::Config config;
    ASSERT_TRUE(renderer.initialize(config));

    // Create trajectory with rotations
    std::vector<TrajectoryRenderer::Pose> poses;
    for (int i = 0; i < 30; ++i) {
        TrajectoryRenderer::Pose pose;
        pose.transform = Eigen::Isometry3d::Identity();

        // Translation
        pose.transform.translate(Eigen::Vector3d(i * 0.1, 0.0, 0.0));

        // Rotation around Y-axis
        Eigen::AngleAxisd rotation(i * 0.1, Eigen::Vector3d::UnitY());
        pose.transform.rotate(rotation);

        pose.trackingQuality = 1.0f;
        poses.push_back(pose);
    }

    renderer.updateTrajectory(poses);
    EXPECT_EQ(renderer.getPoseCount(), 30);

    // Render
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f proj = Eigen::Matrix4f::Identity();

    EXPECT_NO_THROW(renderer.render(view, proj));
}
