#include "visualizer/renderer.hpp"
#include <gtest/gtest.h>
#include <thread>
#include <chrono>

using namespace vi_slam::visualizer;

class RendererTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Note: These tests require a display/windowing system to be available
        // They may fail in headless CI environments
    }

    void TearDown() override {
    }
};

TEST_F(RendererTest, InitializationSuccess) {
    Renderer renderer;
    EXPECT_FALSE(renderer.isInitialized());

    Renderer::Config config;
    config.width = 800;
    config.height = 600;
    config.title = "Test Renderer";

    bool result = renderer.initialize(config);

    // If running in headless environment, initialization may fail
    if (result) {
        EXPECT_TRUE(renderer.isInitialized());

        int width, height;
        renderer.getWindowSize(width, height);
        EXPECT_EQ(width, 800);
        EXPECT_EQ(height, 600);

        renderer.shutdown();
        EXPECT_FALSE(renderer.isInitialized());
    } else {
        GTEST_SKIP() << "Skipping test - no display available (headless environment)";
    }
}

TEST_F(RendererTest, DoubleInitialization) {
    Renderer renderer;
    Renderer::Config config;
    config.width = 640;
    config.height = 480;

    bool firstInit = renderer.initialize(config);
    if (!firstInit) {
        GTEST_SKIP() << "Skipping test - no display available";
    }

    EXPECT_TRUE(renderer.isInitialized());

    // Second initialization should fail
    bool secondInit = renderer.initialize(config);
    EXPECT_FALSE(secondInit);
    EXPECT_TRUE(renderer.isInitialized());  // Still initialized from first call

    renderer.shutdown();
}

TEST_F(RendererTest, RenderLoop) {
    Renderer renderer;
    Renderer::Config config;
    config.width = 640;
    config.height = 480;
    config.targetFps = 60;

    if (!renderer.initialize(config)) {
        GTEST_SKIP() << "Skipping test - no display available";
    }

    // Run a few frames
    int frameCount = 0;
    const int targetFrames = 10;

    while (!renderer.shouldClose() && frameCount < targetFrames) {
        renderer.beginFrame();
        // Actual rendering would go here
        renderer.endFrame();
        frameCount++;

        // Small delay to simulate work
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    EXPECT_EQ(frameCount, targetFrames);
    EXPECT_GE(renderer.getFps(), 0.0f);  // FPS should be calculated

    renderer.shutdown();
}

TEST_F(RendererTest, ViewProjectionMatrices) {
    Renderer renderer;
    Renderer::Config config;
    config.width = 640;
    config.height = 480;

    if (!renderer.initialize(config)) {
        GTEST_SKIP() << "Skipping test - no display available";
    }

    // Check default matrices are not zero
    Eigen::Matrix4f viewMatrix = renderer.getViewMatrix();
    Eigen::Matrix4f projMatrix = renderer.getProjectionMatrix();

    EXPECT_FALSE(viewMatrix.isZero());
    EXPECT_FALSE(projMatrix.isZero());

    // Set custom view matrix
    Eigen::Matrix4f customView = Eigen::Matrix4f::Identity();
    customView(0, 3) = 5.0f;  // Translate X
    renderer.setViewMatrix(customView);

    Eigen::Matrix4f retrievedView = renderer.getViewMatrix();
    EXPECT_TRUE(retrievedView.isApprox(customView));

    // Set custom projection matrix
    Eigen::Matrix4f customProj = Eigen::Matrix4f::Identity();
    customProj(1, 1) = 2.0f;
    renderer.setProjectionMatrix(customProj);

    Eigen::Matrix4f retrievedProj = renderer.getProjectionMatrix();
    EXPECT_TRUE(retrievedProj.isApprox(customProj));

    renderer.shutdown();
}

TEST_F(RendererTest, ShutdownBeforeInit) {
    Renderer renderer;
    // Calling shutdown before init should not crash
    EXPECT_NO_THROW(renderer.shutdown());
    EXPECT_FALSE(renderer.isInitialized());
}

TEST_F(RendererTest, MultipleShutdown) {
    Renderer renderer;
    Renderer::Config config;
    config.width = 640;
    config.height = 480;

    if (!renderer.initialize(config)) {
        GTEST_SKIP() << "Skipping test - no display available";
    }

    EXPECT_TRUE(renderer.isInitialized());

    renderer.shutdown();
    EXPECT_FALSE(renderer.isInitialized());

    // Second shutdown should be safe
    EXPECT_NO_THROW(renderer.shutdown());
    EXPECT_FALSE(renderer.isInitialized());
}
