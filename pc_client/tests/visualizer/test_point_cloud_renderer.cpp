#include "visualizer/point_cloud_renderer.hpp"
#include "visualizer/renderer.hpp"
#include <gtest/gtest.h>
#include <Eigen/Core>
#include <vector>

using namespace vi_slam::visualizer;

class PointCloudRendererTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Initialize base renderer for OpenGL context
        Renderer::Config config;
        config.width = 800;
        config.height = 600;
        config.title = "Point Cloud Renderer Test";

        ASSERT_TRUE(baseRenderer_.initialize(config));
    }

    void TearDown() override {
        baseRenderer_.shutdown();
    }

    Renderer baseRenderer_;
};

TEST_F(PointCloudRendererTest, Initialization) {
    PointCloudRenderer renderer;

    EXPECT_FALSE(renderer.isInitialized());
    EXPECT_EQ(renderer.getPointCount(), 0);

    PointCloudRenderer::Config config;
    EXPECT_TRUE(renderer.initialize(config));
    EXPECT_TRUE(renderer.isInitialized());

    renderer.shutdown();
    EXPECT_FALSE(renderer.isInitialized());
}

TEST_F(PointCloudRendererTest, CustomConfiguration) {
    PointCloudRenderer renderer;

    PointCloudRenderer::Config config;
    config.pointSize = 5.0f;
    config.defaultColor = Eigen::Vector3f(1.0f, 0.0f, 0.0f); // Red
    config.useVertexColors = true;

    EXPECT_TRUE(renderer.initialize(config));
    EXPECT_TRUE(renderer.isInitialized());
}

TEST_F(PointCloudRendererTest, UpdatePointCloud) {
    PointCloudRenderer renderer;
    PointCloudRenderer::Config config;
    ASSERT_TRUE(renderer.initialize(config));

    // Create test point cloud
    std::vector<Eigen::Vector3d> points;
    for (int i = 0; i < 100; ++i) {
        points.push_back(Eigen::Vector3d(
            static_cast<double>(i) * 0.1,
            static_cast<double>(i) * 0.05,
            static_cast<double>(i) * 0.02
        ));
    }

    renderer.updatePointCloud(points);
    EXPECT_EQ(renderer.getPointCount(), 100);
}

TEST_F(PointCloudRendererTest, UpdatePointCloudWithColors) {
    PointCloudRenderer renderer;
    PointCloudRenderer::Config config;
    ASSERT_TRUE(renderer.initialize(config));

    // Create test point cloud with colors
    std::vector<Eigen::Vector3d> points;
    std::vector<Eigen::Vector3f> colors;

    for (int i = 0; i < 50; ++i) {
        points.push_back(Eigen::Vector3d(
            static_cast<double>(i) * 0.1,
            0.0,
            0.0
        ));
        colors.push_back(Eigen::Vector3f(
            static_cast<float>(i) / 50.0f,
            0.5f,
            1.0f - static_cast<float>(i) / 50.0f
        ));
    }

    renderer.updatePointCloud(points, colors);
    EXPECT_EQ(renderer.getPointCount(), 50);
}

TEST_F(PointCloudRendererTest, UpdateLargePointCloud) {
    PointCloudRenderer renderer;
    PointCloudRenderer::Config config;
    ASSERT_TRUE(renderer.initialize(config));

    // Create large point cloud (100K points)
    std::vector<Eigen::Vector3d> points;
    points.reserve(100000);

    for (int i = 0; i < 100000; ++i) {
        points.push_back(Eigen::Vector3d(
            (rand() % 1000) / 100.0,
            (rand() % 1000) / 100.0,
            (rand() % 1000) / 100.0
        ));
    }

    renderer.updatePointCloud(points);
    EXPECT_EQ(renderer.getPointCount(), 100000);
}

TEST_F(PointCloudRendererTest, ClearPointCloud) {
    PointCloudRenderer renderer;
    PointCloudRenderer::Config config;
    ASSERT_TRUE(renderer.initialize(config));

    // Add points
    std::vector<Eigen::Vector3d> points = {
        Eigen::Vector3d(1.0, 2.0, 3.0),
        Eigen::Vector3d(4.0, 5.0, 6.0)
    };
    renderer.updatePointCloud(points);
    EXPECT_EQ(renderer.getPointCount(), 2);

    // Clear
    renderer.clear();
    EXPECT_EQ(renderer.getPointCount(), 0);
}

TEST_F(PointCloudRendererTest, RenderWithoutCrash) {
    PointCloudRenderer renderer;
    PointCloudRenderer::Config config;
    ASSERT_TRUE(renderer.initialize(config));

    // Create test point cloud
    std::vector<Eigen::Vector3d> points;
    for (int i = 0; i < 1000; ++i) {
        points.push_back(Eigen::Vector3d(
            (rand() % 100) / 10.0,
            (rand() % 100) / 10.0,
            (rand() % 100) / 10.0
        ));
    }
    renderer.updatePointCloud(points);

    // Get view and projection matrices from base renderer
    Eigen::Matrix4f viewMatrix = baseRenderer_.getViewMatrix();
    Eigen::Matrix4f projMatrix = baseRenderer_.getProjectionMatrix();

    // Render should not crash
    baseRenderer_.beginFrame();
    renderer.render(viewMatrix, projMatrix);
    baseRenderer_.endFrame();
}

TEST_F(PointCloudRendererTest, SetPointSize) {
    PointCloudRenderer renderer;
    PointCloudRenderer::Config config;
    ASSERT_TRUE(renderer.initialize(config));

    renderer.setPointSize(10.0f);
    // No direct getter, but should not crash
}

TEST_F(PointCloudRendererTest, SetDefaultColor) {
    PointCloudRenderer renderer;
    PointCloudRenderer::Config config;
    ASSERT_TRUE(renderer.initialize(config));

    renderer.setDefaultColor(Eigen::Vector3f(0.5f, 0.5f, 1.0f));
    // No direct getter, but should not crash
}

TEST_F(PointCloudRendererTest, MismatchedColorSize) {
    PointCloudRenderer renderer;
    PointCloudRenderer::Config config;
    ASSERT_TRUE(renderer.initialize(config));

    std::vector<Eigen::Vector3d> points = {
        Eigen::Vector3d(1.0, 2.0, 3.0),
        Eigen::Vector3d(4.0, 5.0, 6.0)
    };

    std::vector<Eigen::Vector3f> colors = {
        Eigen::Vector3f(1.0f, 0.0f, 0.0f)
        // Intentionally one color short
    };

    // Should handle gracefully
    renderer.updatePointCloud(points, colors);
    EXPECT_EQ(renderer.getPointCount(), 2);
}

TEST_F(PointCloudRendererTest, EmptyPointCloud) {
    PointCloudRenderer renderer;
    PointCloudRenderer::Config config;
    ASSERT_TRUE(renderer.initialize(config));

    std::vector<Eigen::Vector3d> emptyPoints;
    renderer.updatePointCloud(emptyPoints);
    EXPECT_EQ(renderer.getPointCount(), 0);

    // Render empty cloud should not crash
    Eigen::Matrix4f viewMatrix = baseRenderer_.getViewMatrix();
    Eigen::Matrix4f projMatrix = baseRenderer_.getProjectionMatrix();

    baseRenderer_.beginFrame();
    renderer.render(viewMatrix, projMatrix);
    baseRenderer_.endFrame();
}
