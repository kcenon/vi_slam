#include "visualizer/camera_controller.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>

namespace vi_slam {
namespace visualizer {

namespace {
constexpr float PI = 3.14159265358979323846f;
constexpr float EPSILON = 1e-6f;

// Clamp value between min and max
template<typename T>
T clamp(T value, T minVal, T maxVal) {
    return std::max(minVal, std::min(value, maxVal));
}

// Linear interpolation
Eigen::Vector3f lerp(const Eigen::Vector3f& a, const Eigen::Vector3f& b, float t) {
    return a + t * (b - a);
}
} // anonymous namespace

CameraController::CameraController()
    : isDragging_(false)
    , activeButton_(-1)
    , lastMouseX_(0.0)
    , lastMouseY_(0.0)
    , radius_(5.0f)
    , theta_(0.0f)
    , phi_(0.0f)
{
    Config defaultConfig;
    initialize(defaultConfig);
}

CameraController::CameraController(const Config& config)
    : isDragging_(false)
    , activeButton_(-1)
    , lastMouseX_(0.0)
    , lastMouseY_(0.0)
    , radius_(5.0f)
    , theta_(0.0f)
    , phi_(0.0f)
{
    initialize(config);
}

void CameraController::initialize(const Config& config) {
    config_ = config;

    // Set initial camera state
    currentEye_ = config_.initialEye;
    currentTarget_ = config_.initialTarget;
    currentUp_ = config_.initialUp;

    targetEye_ = currentEye_;
    targetTarget_ = currentTarget_;
    targetUp_ = currentUp_;

    // Compute initial spherical coordinates
    Eigen::Vector3f relativePos = currentEye_ - currentTarget_;
    cartesianToSpherical(relativePos, radius_, theta_, phi_);

    std::cout << "CameraController initialized" << std::endl;
    std::cout << "  Initial position: (" << currentEye_.transpose() << ")" << std::endl;
    std::cout << "  Initial target: (" << currentTarget_.transpose() << ")" << std::endl;
    std::cout << "  Spherical coords: r=" << radius_ << " theta=" << theta_
              << " phi=" << phi_ << std::endl;
}

void CameraController::reset() {
    targetEye_ = config_.initialEye;
    targetTarget_ = config_.initialTarget;
    targetUp_ = config_.initialUp;

    if (!config_.enableSmoothing) {
        currentEye_ = targetEye_;
        currentTarget_ = targetTarget_;
        currentUp_ = targetUp_;
    }

    Eigen::Vector3f relativePos = targetEye_ - targetTarget_;
    cartesianToSpherical(relativePos, radius_, theta_, phi_);

    std::cout << "Camera reset to initial position" << std::endl;
}

Eigen::Matrix4f CameraController::getViewMatrix() const {
    return computeViewMatrix(currentEye_, currentTarget_, currentUp_);
}

void CameraController::update(float deltaTime) {
    if (!config_.enableSmoothing) {
        currentEye_ = targetEye_;
        currentTarget_ = targetTarget_;
        currentUp_ = targetUp_;
        return;
    }

    // Exponential smoothing
    float t = 1.0f - std::exp(-config_.interpolationSpeed * deltaTime);
    t = clamp(t, 0.0f, 1.0f);

    currentEye_ = lerp(currentEye_, targetEye_, t);
    currentTarget_ = lerp(currentTarget_, targetTarget_, t);
    currentUp_ = lerp(currentUp_, targetUp_, t);

    // Renormalize up vector
    currentUp_.normalize();
}

void CameraController::onMousePress(int button, double x, double y) {
    isDragging_ = true;
    activeButton_ = button;
    lastMouseX_ = x;
    lastMouseY_ = y;
}

void CameraController::onMouseRelease(int button) {
    if (activeButton_ == button) {
        isDragging_ = false;
        activeButton_ = -1;
    }
}

void CameraController::onMouseMove(double x, double y) {
    if (!isDragging_) {
        return;
    }

    double deltaX = x - lastMouseX_;
    double deltaY = y - lastMouseY_;

    if (activeButton_ == 0) {
        // Left button: orbit
        orbit(deltaX, deltaY);
    } else if (activeButton_ == 1) {
        // Right button: pan
        pan(deltaX, deltaY);
    }

    lastMouseX_ = x;
    lastMouseY_ = y;
}

void CameraController::onMouseScroll(double offsetY) {
    zoom(offsetY);
}

void CameraController::rotateByKeyboard(const Eigen::Vector2f& direction) {
    // Convert keyboard input to mouse-like delta
    float pixelDelta = 50.0f;  // Simulate 50 pixels of mouse movement
    orbit(direction.x() * pixelDelta, direction.y() * pixelDelta);
}

void CameraController::frameScene(const Eigen::Vector3f& sceneCenter, float sceneRadius) {
    // Position camera to frame the scene
    // Use 45-degree FOV, ensure entire scene is visible
    float fovRadians = 45.0f * PI / 180.0f;
    float requiredDistance = sceneRadius / std::tan(fovRadians * 0.5f);

    // Add 20% margin
    requiredDistance *= 1.2f;

    // Maintain current viewing direction, just adjust distance
    Eigen::Vector3f viewDir = (currentTarget_ - currentEye_).normalized();

    targetTarget_ = sceneCenter;
    targetEye_ = sceneCenter - viewDir * requiredDistance;
    targetUp_ = currentUp_;

    // Update spherical coordinates
    Eigen::Vector3f relativePos = targetEye_ - targetTarget_;
    cartesianToSpherical(relativePos, radius_, theta_, phi_);

    std::cout << "Camera framed scene: center=" << sceneCenter.transpose()
              << " radius=" << sceneRadius << " distance=" << requiredDistance << std::endl;
}

Eigen::Matrix4f CameraController::computeViewMatrix(const Eigen::Vector3f& eye,
                                                     const Eigen::Vector3f& target,
                                                     const Eigen::Vector3f& up) const {
    Eigen::Vector3f f = (target - eye).normalized();
    Eigen::Vector3f s = f.cross(up).normalized();
    Eigen::Vector3f u = s.cross(f);

    Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
    result(0, 0) = s.x();
    result(0, 1) = s.y();
    result(0, 2) = s.z();
    result(1, 0) = u.x();
    result(1, 1) = u.y();
    result(1, 2) = u.z();
    result(2, 0) = -f.x();
    result(2, 1) = -f.y();
    result(2, 2) = -f.z();
    result(0, 3) = -s.dot(eye);
    result(1, 3) = -u.dot(eye);
    result(2, 3) = f.dot(eye);

    return result;
}

void CameraController::orbit(double deltaX, double deltaY) {
    // Update spherical coordinates
    theta_ -= static_cast<float>(deltaX) * config_.rotationSpeed;
    phi_ += static_cast<float>(deltaY) * config_.rotationSpeed;

    // Clamp phi to avoid gimbal lock
    phi_ = clamp(phi_, -PI * 0.49f, PI * 0.49f);

    // Wrap theta to [-PI, PI]
    while (theta_ > PI) theta_ -= 2.0f * PI;
    while (theta_ < -PI) theta_ += 2.0f * PI;

    // Convert back to Cartesian
    Eigen::Vector3f newRelativePos = sphericalToCartesian(radius_, theta_, phi_);
    targetEye_ = targetTarget_ + newRelativePos;

    // Update up vector to maintain orientation
    targetUp_ = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
}

void CameraController::pan(double deltaX, double deltaY) {
    // Get camera coordinate system
    Eigen::Vector3f viewDir = (targetTarget_ - targetEye_).normalized();
    Eigen::Vector3f right = viewDir.cross(targetUp_).normalized();
    Eigen::Vector3f up = right.cross(viewDir).normalized();

    // Pan in screen space
    Eigen::Vector3f panOffset =
        -right * static_cast<float>(deltaX) * config_.panSpeed +
        up * static_cast<float>(deltaY) * config_.panSpeed;

    targetEye_ += panOffset;
    targetTarget_ += panOffset;

    // Update spherical coordinates
    Eigen::Vector3f relativePos = targetEye_ - targetTarget_;
    cartesianToSpherical(relativePos, radius_, theta_, phi_);
}

void CameraController::zoom(double delta) {
    // Adjust radius
    radius_ *= (1.0f - static_cast<float>(delta) * config_.zoomSpeed);
    radius_ = clamp(radius_, config_.minDistance, config_.maxDistance);

    // Convert back to Cartesian
    Eigen::Vector3f newRelativePos = sphericalToCartesian(radius_, theta_, phi_);
    targetEye_ = targetTarget_ + newRelativePos;
}

Eigen::Vector3f CameraController::sphericalToCartesian(float radius, float theta, float phi) {
    // Spherical coordinates: (r, θ, φ)
    // θ = azimuth (around Y axis), φ = elevation (from XZ plane)
    float x = radius * std::cos(phi) * std::sin(theta);
    float y = radius * std::sin(phi);
    float z = radius * std::cos(phi) * std::cos(theta);
    return Eigen::Vector3f(x, y, z);
}

void CameraController::cartesianToSpherical(const Eigen::Vector3f& position,
                                           float& radius, float& theta, float& phi) {
    radius = position.norm();

    if (radius < EPSILON) {
        theta = 0.0f;
        phi = 0.0f;
        return;
    }

    // θ = atan2(x, z)
    theta = std::atan2(position.x(), position.z());

    // φ = asin(y / r)
    phi = std::asin(clamp(position.y() / radius, -1.0f, 1.0f));
}

} // namespace visualizer
} // namespace vi_slam
