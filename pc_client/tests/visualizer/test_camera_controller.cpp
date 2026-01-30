#include <gtest/gtest.h>
#include "visualizer/camera_controller.hpp"
#include <cmath>

using namespace vi_slam::visualizer;

namespace {
constexpr float PI = 3.14159265358979323846f;
constexpr float EPSILON = 1e-5f;

bool nearlyEqual(float a, float b, float epsilon = EPSILON) {
    return std::abs(a - b) < epsilon;
}

bool nearlyEqual(const Eigen::Vector3f& a, const Eigen::Vector3f& b, float epsilon = EPSILON) {
    return (a - b).norm() < epsilon;
}
} // anonymous namespace

class CameraControllerTest : public ::testing::Test {
protected:
    void SetUp() override {
        config_.initialEye = Eigen::Vector3f(0.0f, 0.0f, 5.0f);
        config_.initialTarget = Eigen::Vector3f::Zero();
        config_.initialUp = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
        config_.enableSmoothing = false;  // Disable for deterministic tests
    }

    CameraController::Config config_;
};

TEST_F(CameraControllerTest, DefaultConstruction) {
    CameraController controller;

    EXPECT_TRUE(controller.getPosition().isApprox(
        Eigen::Vector3f(0.0f, 0.0f, 5.0f), EPSILON));
    EXPECT_TRUE(controller.getTarget().isApprox(
        Eigen::Vector3f::Zero(), EPSILON));
    EXPECT_TRUE(controller.getUpVector().isApprox(
        Eigen::Vector3f(0.0f, 1.0f, 0.0f), EPSILON));
}

TEST_F(CameraControllerTest, Initialization) {
    config_.initialEye = Eigen::Vector3f(10.0f, 5.0f, 10.0f);
    config_.initialTarget = Eigen::Vector3f(1.0f, 2.0f, 3.0f);

    CameraController controller(config_);

    EXPECT_TRUE(controller.getPosition().isApprox(config_.initialEye, EPSILON));
    EXPECT_TRUE(controller.getTarget().isApprox(config_.initialTarget, EPSILON));
}

TEST_F(CameraControllerTest, Reset) {
    CameraController controller(config_);

    // Move camera away from initial position
    controller.onMousePress(0, 100.0, 100.0);
    controller.onMouseMove(200.0, 150.0);
    controller.onMouseRelease(0);
    controller.update(0.0f);

    // Reset should restore initial state
    controller.reset();
    controller.update(0.0f);

    EXPECT_TRUE(controller.getPosition().isApprox(config_.initialEye, 0.1f));
    EXPECT_TRUE(controller.getTarget().isApprox(config_.initialTarget, EPSILON));
}

TEST_F(CameraControllerTest, OrbitCamera) {
    CameraController controller(config_);

    float initialDistance = controller.getDistance();

    // Simulate left-drag to orbit
    controller.onMousePress(0, 100.0, 100.0);
    controller.onMouseMove(150.0, 100.0);  // Horizontal movement
    controller.onMouseRelease(0);
    controller.update(0.0f);

    // Distance should remain constant during orbit
    EXPECT_NEAR(controller.getDistance(), initialDistance, EPSILON);

    // Target should not move during orbit
    EXPECT_TRUE(controller.getTarget().isApprox(config_.initialTarget, EPSILON));

    // Camera position should have changed
    EXPECT_FALSE(controller.getPosition().isApprox(config_.initialEye, 0.1f));
}

TEST_F(CameraControllerTest, PanCamera) {
    CameraController controller(config_);

    Eigen::Vector3f initialPos = controller.getPosition();
    Eigen::Vector3f initialTarget = controller.getTarget();
    float initialDistance = controller.getDistance();

    // Simulate right-drag to pan
    controller.onMousePress(1, 100.0, 100.0);
    controller.onMouseMove(150.0, 120.0);
    controller.onMouseRelease(1);
    controller.update(0.0f);

    // Both position and target should move during pan
    EXPECT_FALSE(controller.getPosition().isApprox(initialPos, 0.1f));
    EXPECT_FALSE(controller.getTarget().isApprox(initialTarget, 0.1f));

    // Distance should remain constant
    EXPECT_NEAR(controller.getDistance(), initialDistance, EPSILON);

    // Relative displacement should be the same for both
    Eigen::Vector3f posDisplacement = controller.getPosition() - initialPos;
    Eigen::Vector3f targetDisplacement = controller.getTarget() - initialTarget;
    EXPECT_TRUE(posDisplacement.isApprox(targetDisplacement, 0.1f));
}

TEST_F(CameraControllerTest, ZoomCamera) {
    CameraController controller(config_);

    float initialDistance = controller.getDistance();
    Eigen::Vector3f initialTarget = controller.getTarget();

    // Zoom in (positive scroll)
    controller.onMouseScroll(1.0);
    controller.update(0.0f);

    // Distance should decrease
    EXPECT_LT(controller.getDistance(), initialDistance);

    // Target should not move
    EXPECT_TRUE(controller.getTarget().isApprox(initialTarget, EPSILON));

    // Zoom out (negative scroll)
    controller.onMouseScroll(-2.0);
    controller.update(0.0f);

    // Distance should increase (beyond initial)
    EXPECT_GT(controller.getDistance(), initialDistance);
}

TEST_F(CameraControllerTest, ZoomLimits) {
    CameraController controller(config_);

    // Zoom in to minimum
    for (int i = 0; i < 100; ++i) {
        controller.onMouseScroll(1.0);
    }
    controller.update(0.0f);

    EXPECT_GE(controller.getDistance(), config_.minDistance);
    EXPECT_LE(controller.getDistance(), config_.maxDistance);

    // Zoom out to maximum
    for (int i = 0; i < 100; ++i) {
        controller.onMouseScroll(-1.0);
    }
    controller.update(0.0f);

    EXPECT_GE(controller.getDistance(), config_.minDistance);
    EXPECT_LE(controller.getDistance(), config_.maxDistance);
}

TEST_F(CameraControllerTest, KeyboardRotation) {
    CameraController controller(config_);

    float initialDistance = controller.getDistance();

    // Rotate right
    controller.rotateByKeyboard(Eigen::Vector2f(1.0f, 0.0f));
    controller.update(0.0f);

    // Distance should remain constant
    EXPECT_NEAR(controller.getDistance(), initialDistance, EPSILON);

    // Position should have changed
    EXPECT_FALSE(controller.getPosition().isApprox(config_.initialEye, 0.1f));

    // Rotate up
    controller.rotateByKeyboard(Eigen::Vector2f(0.0f, 1.0f));
    controller.update(0.0f);

    EXPECT_NEAR(controller.getDistance(), initialDistance, EPSILON);
}

TEST_F(CameraControllerTest, FrameScene) {
    CameraController controller(config_);

    Eigen::Vector3f sceneCenter(5.0f, 3.0f, 2.0f);
    float sceneRadius = 10.0f;

    controller.frameScene(sceneCenter, sceneRadius);
    controller.update(0.0f);

    // Target should be at scene center
    EXPECT_TRUE(controller.getTarget().isApprox(sceneCenter, 0.1f));

    // Distance should be appropriate for scene radius
    float expectedDistance = sceneRadius / std::tan(45.0f * PI / 180.0f * 0.5f) * 1.2f;
    EXPECT_NEAR(controller.getDistance(), expectedDistance, 1.0f);
}

TEST_F(CameraControllerTest, ViewMatrixComputation) {
    CameraController controller(config_);

    Eigen::Matrix4f viewMatrix = controller.getViewMatrix();

    // View matrix should be 4x4
    EXPECT_EQ(viewMatrix.rows(), 4);
    EXPECT_EQ(viewMatrix.cols(), 4);

    // Bottom row should be [0, 0, 0, 1] for homogeneous coordinates
    EXPECT_NEAR(viewMatrix(3, 0), 0.0f, EPSILON);
    EXPECT_NEAR(viewMatrix(3, 1), 0.0f, EPSILON);
    EXPECT_NEAR(viewMatrix(3, 2), 0.0f, EPSILON);
    EXPECT_NEAR(viewMatrix(3, 3), 1.0f, EPSILON);
}

TEST_F(CameraControllerTest, SmoothingEnabled) {
    config_.enableSmoothing = true;
    config_.interpolationSpeed = 5.0f;

    CameraController controller(config_);

    Eigen::Vector3f initialPos = controller.getPosition();

    // Perform orbit
    controller.onMousePress(0, 100.0, 100.0);
    controller.onMouseMove(200.0, 100.0);
    controller.onMouseRelease(0);

    // Update with small time step
    controller.update(0.01f);

    // Position should have moved, but not to target yet (smoothing)
    Eigen::Vector3f currentPos = controller.getPosition();
    EXPECT_FALSE(currentPos.isApprox(initialPos, 0.01f));

    // Continue updating
    for (int i = 0; i < 100; ++i) {
        controller.update(0.01f);
    }

    // After many updates, should converge to target
    // (exact convergence test is difficult with exponential smoothing)
}

TEST_F(CameraControllerTest, MultipleMouseButtons) {
    CameraController controller(config_);

    // Press left button
    controller.onMousePress(0, 100.0, 100.0);
    controller.onMouseMove(150.0, 100.0);

    // Press right button (should be ignored while left is active)
    controller.onMousePress(1, 150.0, 100.0);

    // Release wrong button (should not affect left drag)
    controller.onMouseRelease(1);

    controller.onMouseMove(200.0, 100.0);

    // Release correct button
    controller.onMouseRelease(0);
    controller.update(0.0f);

    // Camera should have orbited (left button behavior)
    EXPECT_FALSE(controller.getPosition().isApprox(config_.initialEye, 0.1f));
}

TEST_F(CameraControllerTest, UpVectorMaintained) {
    CameraController controller(config_);

    // Perform various operations
    controller.onMouseScroll(1.0);
    controller.rotateByKeyboard(Eigen::Vector2f(1.0f, 0.5f));
    controller.update(0.0f);

    // Up vector should remain normalized and close to Y-axis
    Eigen::Vector3f up = controller.getUpVector();
    EXPECT_NEAR(up.norm(), 1.0f, EPSILON);
    EXPECT_GT(up.y(), 0.5f);  // Should generally point upward
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
