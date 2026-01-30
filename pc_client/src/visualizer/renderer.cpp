#include "visualizer/renderer.hpp"

#ifdef __APPLE__
#define GL_SILENCE_DEPRECATION
#endif

#include <GLFW/glfw3.h>
#include <iostream>
#include <cmath>

namespace vi_slam {
namespace visualizer {

namespace {
// GLFW error callback
void glfwErrorCallback(int error, const char* description) {
    std::cerr << "GLFW Error " << error << ": " << description << std::endl;
}

// Window resize callback
void framebufferSizeCallback(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
}
} // anonymous namespace

Renderer::Renderer()
    : window_(nullptr)
    , initialized_(false)
    , currentFps_(0.0f)
    , lastFrameTime_(0.0)
    , frameCount_(0)
    , fpsUpdateTime_(0.0)
{
    viewMatrix_ = Eigen::Matrix4f::Identity();
    projMatrix_ = Eigen::Matrix4f::Identity();
}

Renderer::~Renderer() {
    shutdown();
}

bool Renderer::initialize(const Config& config) {
    if (initialized_) {
        std::cerr << "Renderer already initialized" << std::endl;
        return false;
    }

    config_ = config;

    // Set error callback
    glfwSetErrorCallback(glfwErrorCallback);

    // Initialize GLFW
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return false;
    }

    // Configure GLFW
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    // Create window
    GLFWwindow* window = glfwCreateWindow(
        config_.width,
        config_.height,
        config_.title.c_str(),
        nullptr,
        nullptr
    );

    if (!window) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return false;
    }

    window_ = window;
    glfwMakeContextCurrent(window);

    // Set framebuffer resize callback
    glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);

    // Enable vsync
    glfwSwapInterval(config_.vsync ? 1 : 0);

    // Initialize OpenGL viewport
    int framebufferWidth, framebufferHeight;
    glfwGetFramebufferSize(window, &framebufferWidth, &framebufferHeight);
    glViewport(0, 0, framebufferWidth, framebufferHeight);

    // Enable depth testing
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    // Set clear color
    glClearColor(
        config_.clearColor[0],
        config_.clearColor[1],
        config_.clearColor[2],
        config_.clearColor[3]
    );

    // Initialize projection matrix
    float aspect = static_cast<float>(config_.width) / static_cast<float>(config_.height);
    projMatrix_ = createPerspective(45.0f, aspect, 0.1f, 1000.0f);

    // Initialize view matrix (camera at origin looking down -Z)
    viewMatrix_ = createLookAt(
        Eigen::Vector3f(0.0f, 0.0f, 3.0f),
        Eigen::Vector3f(0.0f, 0.0f, 0.0f),
        Eigen::Vector3f(0.0f, 1.0f, 0.0f)
    );

    initialized_ = true;
    lastFrameTime_ = glfwGetTime();
    fpsUpdateTime_ = lastFrameTime_;

    std::cout << "Renderer initialized successfully" << std::endl;
    std::cout << "  Window: " << config_.width << "x" << config_.height << std::endl;
    std::cout << "  OpenGL version: " << glGetString(GL_VERSION) << std::endl;
    std::cout << "  GLSL version: " << glGetString(GL_SHADING_LANGUAGE_VERSION) << std::endl;

    return true;
}

void Renderer::shutdown() {
    if (!initialized_) {
        return;
    }

    if (window_) {
        GLFWwindow* window = static_cast<GLFWwindow*>(window_);
        glfwDestroyWindow(window);
        window_ = nullptr;
    }

    glfwTerminate();
    initialized_ = false;

    std::cout << "Renderer shutdown complete" << std::endl;
}

bool Renderer::shouldClose() const {
    if (!initialized_ || !window_) {
        return true;
    }
    return glfwWindowShouldClose(static_cast<GLFWwindow*>(window_));
}

void Renderer::beginFrame() {
    if (!initialized_) {
        return;
    }

    // Clear buffers
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Update FPS counter
    double currentTime = glfwGetTime();
    frameCount_++;

    if (currentTime - fpsUpdateTime_ >= 1.0) {
        currentFps_ = static_cast<float>(frameCount_) / static_cast<float>(currentTime - fpsUpdateTime_);
        frameCount_ = 0;
        fpsUpdateTime_ = currentTime;
    }

    lastFrameTime_ = currentTime;
}

void Renderer::endFrame() {
    if (!initialized_ || !window_) {
        return;
    }

    GLFWwindow* window = static_cast<GLFWwindow*>(window_);

    // Swap buffers
    glfwSwapBuffers(window);

    // Poll events
    glfwPollEvents();
}

void Renderer::setViewMatrix(const Eigen::Matrix4f& viewMatrix) {
    viewMatrix_ = viewMatrix;
}

void Renderer::setProjectionMatrix(const Eigen::Matrix4f& projMatrix) {
    projMatrix_ = projMatrix;
}

void Renderer::getWindowSize(int& width, int& height) const {
    if (!initialized_ || !window_) {
        width = 0;
        height = 0;
        return;
    }

    GLFWwindow* window = static_cast<GLFWwindow*>(window_);
    glfwGetWindowSize(window, &width, &height);
}

Eigen::Matrix4f Renderer::createPerspective(float fov, float aspect, float near, float far) {
    float tanHalfFov = std::tan(fov * 0.5f * M_PI / 180.0f);

    Eigen::Matrix4f result = Eigen::Matrix4f::Zero();
    result(0, 0) = 1.0f / (aspect * tanHalfFov);
    result(1, 1) = 1.0f / tanHalfFov;
    result(2, 2) = -(far + near) / (far - near);
    result(2, 3) = -(2.0f * far * near) / (far - near);
    result(3, 2) = -1.0f;

    return result;
}

Eigen::Matrix4f Renderer::createLookAt(const Eigen::Vector3f& eye,
                                       const Eigen::Vector3f& center,
                                       const Eigen::Vector3f& up) {
    Eigen::Vector3f f = (center - eye).normalized();
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

} // namespace visualizer
} // namespace vi_slam
