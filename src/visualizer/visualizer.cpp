#include "visualizer/visualizer.hpp"
#include "common/logging.hpp"
#include <GLFW/glfw3.h>
#include <stdexcept>

namespace vi_slam {
namespace visualizer {

namespace {
// GLFW error callback
void glfwErrorCallback(int error, const char* description) {
    LOG_ERROR("Visualizer", "GLFW Error {}: {}", error, description);
}
}  // namespace

Visualizer::Visualizer()
    : window_(nullptr),
      initialized_(false),
      running_(false),
      trackingStatus_(TrackingStatus::UNINITIALIZED),
      fps_(0.0),
      frameTime_(0.0),
      cpuUsage_(0.0) {
}

Visualizer::~Visualizer() {
    shutdown();
}

bool Visualizer::initialize(const VisualizerConfig& config) {
    std::lock_guard<std::mutex> lock(dataMutex_);

    if (initialized_) {
        LOG_WARN("Visualizer", "Visualizer already initialized");
        return false;
    }

    config_ = config;

    if (!initializeGLFW()) {
        return false;
    }

    if (!initializeOpenGL()) {
        cleanupGLFW();
        return false;
    }

    initialized_ = true;
    running_ = true;
    initTime_ = std::chrono::steady_clock::now();
    lastFrameTime_ = initTime_;

    return true;
}

void Visualizer::shutdown() {
    std::lock_guard<std::mutex> lock(dataMutex_);

    if (!initialized_) {
        return;
    }

    cleanupGLFW();

    initialized_ = false;
    running_ = false;
}

bool Visualizer::isRunning() const {
    std::lock_guard<std::mutex> lock(dataMutex_);
    return running_ && window_ != nullptr && !glfwWindowShouldClose(window_);
}

void Visualizer::processEvents() {
    if (!initialized_ || !window_) {
        return;
    }

    glfwPollEvents();

    if (glfwWindowShouldClose(window_)) {
        running_ = false;
    }
}

void Visualizer::beginFrame() {
    if (!initialized_ || !window_) {
        return;
    }

    calculateFrameTiming();
    glfwMakeContextCurrent(window_);
}

void Visualizer::endFrame() {
    if (!initialized_ || !window_) {
        return;
    }

    glfwSwapBuffers(window_);
}

void Visualizer::clear() {
    if (!initialized_) {
        return;
    }

    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void Visualizer::updateTrajectory(const std::vector<Pose6DoF>& trajectory) {
    std::lock_guard<std::mutex> lock(dataMutex_);
    trajectory_ = trajectory;
}

void Visualizer::updateMapPoints(const std::vector<MapPoint>& points) {
    std::lock_guard<std::mutex> lock(dataMutex_);
    mapPoints_ = points;
}

void Visualizer::updateCurrentPose(const Pose6DoF& pose) {
    std::lock_guard<std::mutex> lock(dataMutex_);
    currentPose_ = pose;
}

void Visualizer::updateFPS(double fps) {
    std::lock_guard<std::mutex> lock(dataMutex_);
    fps_ = fps;
}

void Visualizer::updateTrackingStatus(TrackingStatus status) {
    std::lock_guard<std::mutex> lock(dataMutex_);
    trackingStatus_ = status;
}

double Visualizer::getFrameTime() const {
    std::lock_guard<std::mutex> lock(dataMutex_);
    return frameTime_;
}

double Visualizer::getFPS() const {
    std::lock_guard<std::mutex> lock(dataMutex_);
    return fps_;
}

double Visualizer::getCPUUsage() const {
    std::lock_guard<std::mutex> lock(dataMutex_);
    return cpuUsage_;
}

bool Visualizer::initializeGLFW() {
    glfwSetErrorCallback(glfwErrorCallback);

    if (!glfwInit()) {
        LOG_ERROR("Visualizer", "Failed to initialize GLFW");
        return false;
    }

    // Set OpenGL version (3.3 Core)
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

    // Window hints
    glfwWindowHint(GLFW_RESIZABLE, GL_TRUE);
    glfwWindowHint(GLFW_SAMPLES, 4);  // 4x MSAA

    // Create window
    window_ = glfwCreateWindow(
        config_.windowWidth,
        config_.windowHeight,
        config_.windowTitle.c_str(),
        nullptr,
        nullptr
    );

    if (!window_) {
        LOG_ERROR("Visualizer", "Failed to create GLFW window");
        glfwTerminate();
        return false;
    }

    glfwMakeContextCurrent(window_);

    // VSync
    glfwSwapInterval(config_.enableVSync ? 1 : 0);

    return true;
}

bool Visualizer::initializeOpenGL() {
    // Enable depth testing
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    // Enable blending for transparency
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Enable multisampling
    glEnable(GL_MULTISAMPLE);

    // Setup viewport
    setupViewport();

    return true;
}

void Visualizer::cleanupGLFW() {
    if (window_) {
        glfwDestroyWindow(window_);
        window_ = nullptr;
    }

    glfwTerminate();
}

void Visualizer::setupViewport() {
    int width, height;
    glfwGetFramebufferSize(window_, &width, &height);
    glViewport(0, 0, width, height);
}

void Visualizer::calculateFrameTiming() {
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now - lastFrameTime_);
    frameTime_ = duration.count() / 1000.0;  // Convert to milliseconds
    fps_ = (frameTime_ > 0.0) ? 1000.0 / frameTime_ : 0.0;
    lastFrameTime_ = now;

    // Simple CPU usage estimation based on frame time vs target frame time
    double targetFrameTime = 1000.0 / config_.targetFPS;
    cpuUsage_ = (frameTime_ / targetFrameTime) * 100.0;
    cpuUsage_ = std::min(cpuUsage_, 100.0);  // Cap at 100%
}

}  // namespace visualizer
}  // namespace vi_slam
