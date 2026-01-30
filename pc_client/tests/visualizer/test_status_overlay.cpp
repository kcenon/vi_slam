#include <gtest/gtest.h>
#include "visualizer/status_overlay.hpp"

using namespace vi_slam::visualizer;

class StatusOverlayTest : public ::testing::Test {
protected:
    void SetUp() override {
        overlay_ = std::make_unique<StatusOverlay>();
    }

    void TearDown() override {
        overlay_.reset();
    }

    std::unique_ptr<StatusOverlay> overlay_;
};

TEST_F(StatusOverlayTest, InitializationTest) {
    EXPECT_FALSE(overlay_->isInitialized());

    StatusOverlay::Config config;
    EXPECT_TRUE(overlay_->initialize(config));
    EXPECT_TRUE(overlay_->isInitialized());

    // Double initialization should succeed
    EXPECT_TRUE(overlay_->initialize(config));
}

TEST_F(StatusOverlayTest, ShutdownTest) {
    StatusOverlay::Config config;
    overlay_->initialize(config);
    EXPECT_TRUE(overlay_->isInitialized());

    overlay_->shutdown();
    EXPECT_FALSE(overlay_->isInitialized());

    // Double shutdown should be safe
    overlay_->shutdown();
    EXPECT_FALSE(overlay_->isInitialized());
}

TEST_F(StatusOverlayTest, VisibilityTest) {
    StatusOverlay::Config config;
    config.visible = true;
    overlay_->initialize(config);

    EXPECT_TRUE(overlay_->isVisible());

    overlay_->setVisible(false);
    EXPECT_FALSE(overlay_->isVisible());

    overlay_->toggleVisibility();
    EXPECT_TRUE(overlay_->isVisible());

    overlay_->toggleVisibility();
    EXPECT_FALSE(overlay_->isVisible());
}

TEST_F(StatusOverlayTest, UpdateMetricsTest) {
    StatusOverlay::Config config;
    overlay_->initialize(config);

    StatusOverlay::Metrics metrics;
    metrics.fps = 60.5f;
    metrics.trackingState = StatusOverlay::TrackingState::Tracking;
    metrics.pointCount = 125430;
    metrics.keyframeCount = 42;
    metrics.cameraPosition = Eigen::Vector3f(1.23f, 4.56f, 7.89f);

    // Should not throw or crash
    overlay_->updateMetrics(metrics);
}

TEST_F(StatusOverlayTest, ConfigurationTest) {
    StatusOverlay::Config config;
    config.xPosition = 20;
    config.yPosition = 30;
    config.fontSize = 18;
    config.lineSpacing = 25;
    config.visible = false;

    overlay_->initialize(config);

    EXPECT_FALSE(overlay_->isVisible());
}

TEST_F(StatusOverlayTest, TrackingStateTest) {
    StatusOverlay::Config config;
    overlay_->initialize(config);

    StatusOverlay::Metrics metrics;

    // Test tracking state
    metrics.trackingState = StatusOverlay::TrackingState::Tracking;
    overlay_->updateMetrics(metrics);

    // Test lost state
    metrics.trackingState = StatusOverlay::TrackingState::Lost;
    overlay_->updateMetrics(metrics);
}

TEST_F(StatusOverlayTest, PerformanceTest) {
    StatusOverlay::Config config;
    overlay_->initialize(config);

    StatusOverlay::Metrics metrics;
    metrics.fps = 60.0f;
    metrics.trackingState = StatusOverlay::TrackingState::Tracking;
    metrics.pointCount = 100000;
    metrics.keyframeCount = 50;
    metrics.cameraPosition = Eigen::Vector3f(0.0f, 0.0f, 0.0f);

    overlay_->updateMetrics(metrics);

    // Note: Actual render performance testing would require OpenGL context
    // which is not available in unit tests
    // Integration tests should verify <2ms render time
}

TEST_F(StatusOverlayTest, LargePointCountTest) {
    StatusOverlay::Config config;
    overlay_->initialize(config);

    StatusOverlay::Metrics metrics;
    metrics.pointCount = 9999999; // Large number

    overlay_->updateMetrics(metrics);
}

TEST_F(StatusOverlayTest, ColorConfigurationTest) {
    StatusOverlay::Config config;
    config.goodColor = Eigen::Vector4f(0.0f, 1.0f, 0.0f, 1.0f);  // Green
    config.badColor = Eigen::Vector4f(1.0f, 0.0f, 0.0f, 1.0f);   // Red
    config.neutralColor = Eigen::Vector4f(1.0f, 1.0f, 1.0f, 0.8f); // Semi-transparent white

    overlay_->initialize(config);
    EXPECT_TRUE(overlay_->isInitialized());
}

TEST_F(StatusOverlayTest, UninitializedRenderTest) {
    // Render without initialization should be safe (no-op)
    // This should not crash or throw
    overlay_->render(1280, 720);
}

TEST_F(StatusOverlayTest, HiddenRenderTest) {
    StatusOverlay::Config config;
    config.visible = false;
    overlay_->initialize(config);

    // Render while hidden should be fast (early return)
    overlay_->render(1280, 720);
    EXPECT_EQ(overlay_->getLastRenderTime(), 0.0);
}
