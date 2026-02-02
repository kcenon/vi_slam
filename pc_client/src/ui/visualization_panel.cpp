#include "ui/visualization_panel.hpp"
#include <iostream>

// OpenGL and GLFW must be included in correct order
#define GL_SILENCE_DEPRECATION  // Silence OpenGL deprecation warnings on macOS
#include <GLFW/glfw3.h>

// Include OpenGL extensions for framebuffer functions
#ifdef __APPLE__
#include <OpenGL/gl3.h>
#else
#include <GL/gl.h>
#endif

#include "imgui.h"

namespace vi_slam {
namespace ui {

VisualizationPanel::VisualizationPanel()
    : initialized_(false),
      mainWindow_(nullptr),
      renderer_(nullptr),
      cameraController_(nullptr),
      trajectoryRenderer_(nullptr),
      pointCloudRenderer_(nullptr),
      framebuffer_(0),
      textureId_(0),
      depthRenderbuffer_(0),
      framebufferWidth_(800),
      framebufferHeight_(600),
      isPanelHovered_(false),
      isPanelFocused_(false) {
}

VisualizationPanel::~VisualizationPanel() {
    shutdown();
}

bool VisualizationPanel::initialize(GLFWwindow* window) {
    if (initialized_) {
        return true;
    }

    if (!window) {
        std::cerr << "VisualizationPanel: Invalid window" << std::endl;
        return false;
    }

    mainWindow_ = window;

    // Create renderer
    renderer_ = std::make_unique<visualizer::Renderer>();
    visualizer::Renderer::Config config;
    config.width = framebufferWidth_;
    config.height = framebufferHeight_;
    config.title = "VI-SLAM 3D Visualization";
    config.vsync = true;

    // Note: We don't call renderer_->initialize() because we're rendering to a framebuffer
    // instead of a separate window. The main window's OpenGL context is used.

    // Create camera controller
    cameraController_ = std::make_unique<visualizer::CameraController>();
    visualizer::CameraController::Config camConfig;
    camConfig.initialEye = Eigen::Vector3f(0.0f, 5.0f, 10.0f);
    camConfig.initialTarget = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    camConfig.initialUp = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
    cameraController_->initialize(camConfig);

    // Create trajectory renderer
    trajectoryRenderer_ = std::make_unique<visualizer::TrajectoryRenderer>();
    visualizer::TrajectoryRenderer::Config trajConfig;
    if (!trajectoryRenderer_->initialize(trajConfig)) {
        std::cerr << "VisualizationPanel: Failed to initialize trajectory renderer" << std::endl;
        return false;
    }

    // Create point cloud renderer
    pointCloudRenderer_ = std::make_unique<visualizer::PointCloudRenderer>();
    visualizer::PointCloudRenderer::Config pcConfig;
    if (!pointCloudRenderer_->initialize(pcConfig)) {
        std::cerr << "VisualizationPanel: Failed to initialize point cloud renderer" << std::endl;
        return false;
    }

    // Create framebuffer
    if (!createFramebuffer(framebufferWidth_, framebufferHeight_)) {
        std::cerr << "VisualizationPanel: Failed to create framebuffer" << std::endl;
        return false;
    }

    initialized_ = true;
    std::cout << "VisualizationPanel: Initialized successfully" << std::endl;
    return true;
}

void VisualizationPanel::shutdown() {
    if (!initialized_) {
        return;
    }

    cleanupFramebuffer();

    if (pointCloudRenderer_) {
        pointCloudRenderer_->shutdown();
        pointCloudRenderer_.reset();
    }

    if (trajectoryRenderer_) {
        trajectoryRenderer_->shutdown();
        trajectoryRenderer_.reset();
    }

    cameraController_.reset();
    renderer_.reset();

    initialized_ = false;
}

void VisualizationPanel::update() {
    if (!initialized_) {
        return;
    }

    // Update camera controller with fixed timestep (60 FPS)
    if (cameraController_) {
        cameraController_->update(1.0f / 60.0f);
    }
}

void VisualizationPanel::render() {
    if (!initialized_) {
        return;
    }

    ImGui::Begin("3D Visualization");

    // Get available content region size
    ImVec2 availSize = ImGui::GetContentRegionAvail();
    int newWidth = static_cast<int>(availSize.x);
    int newHeight = static_cast<int>(availSize.y);

    // Ensure minimum size
    if (newWidth < 100) newWidth = 100;
    if (newHeight < 100) newHeight = 100;

    // Resize framebuffer if needed
    if (newWidth != framebufferWidth_ || newHeight != framebufferHeight_) {
        resizeFramebuffer(newWidth, newHeight);
    }

    // Get window state for input handling
    isPanelHovered_ = ImGui::IsWindowHovered();
    isPanelFocused_ = ImGui::IsWindowFocused();
    ImVec2 windowPos = ImGui::GetCursorScreenPos();

    // Handle mouse input
    if (isPanelHovered_ || isPanelFocused_) {
        handleMouseInput(availSize, windowPos);
    }

    // Render to framebuffer
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer_);
    glViewport(0, 0, framebufferWidth_, framebufferHeight_);

    // Clear framebuffer
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);

    // Update view and projection matrices
    if (cameraController_) {
        Eigen::Matrix4f viewMatrix = cameraController_->getViewMatrix();
        float aspect = static_cast<float>(framebufferWidth_) / framebufferHeight_;
        Eigen::Matrix4f projMatrix = visualizer::Renderer::createPerspective(
            45.0f, aspect, 0.1f, 1000.0f
        );

        // Render trajectory
        if (trajectoryRenderer_) {
            trajectoryRenderer_->render(viewMatrix, projMatrix);
        }

        // Render point cloud
        if (pointCloudRenderer_) {
            pointCloudRenderer_->render(viewMatrix, projMatrix);
        }
    }

    // Unbind framebuffer
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    // Display framebuffer texture in ImGui
    ImGui::Image(
        reinterpret_cast<ImTextureID>(static_cast<intptr_t>(textureId_)),
        ImVec2(static_cast<float>(framebufferWidth_), static_cast<float>(framebufferHeight_)),
        ImVec2(0, 1),  // UV coordinates (OpenGL's origin is bottom-left)
        ImVec2(1, 0)
    );

    // Display camera info
    if (cameraController_) {
        ImGui::Separator();
        ImGui::Text("Camera Controls:");
        ImGui::BulletText("Left Mouse: Orbit");
        ImGui::BulletText("Middle Mouse: Pan");
        ImGui::BulletText("Scroll: Zoom");
        ImGui::BulletText("Right Mouse: Roll");
    }

    ImGui::End();
}

bool VisualizationPanel::createFramebuffer(int width, int height) {
    // Generate framebuffer
    glGenFramebuffers(1, &framebuffer_);
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer_);

    // Create texture for color attachment
    glGenTextures(1, &textureId_);
    glBindTexture(GL_TEXTURE_2D, textureId_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, textureId_, 0);

    // Create renderbuffer for depth attachment
    glGenRenderbuffers(1, &depthRenderbuffer_);
    glBindRenderbuffer(GL_RENDERBUFFER, depthRenderbuffer_);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, width, height);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depthRenderbuffer_);

    // Check framebuffer completeness
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        std::cerr << "VisualizationPanel: Framebuffer is not complete" << std::endl;
        cleanupFramebuffer();
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        return false;
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    framebufferWidth_ = width;
    framebufferHeight_ = height;

    return true;
}

void VisualizationPanel::resizeFramebuffer(int width, int height) {
    if (width == framebufferWidth_ && height == framebufferHeight_) {
        return;
    }

    cleanupFramebuffer();
    createFramebuffer(width, height);
}

void VisualizationPanel::cleanupFramebuffer() {
    if (framebuffer_ != 0) {
        glDeleteFramebuffers(1, &framebuffer_);
        framebuffer_ = 0;
    }

    if (textureId_ != 0) {
        glDeleteTextures(1, &textureId_);
        textureId_ = 0;
    }

    if (depthRenderbuffer_ != 0) {
        glDeleteRenderbuffers(1, &depthRenderbuffer_);
        depthRenderbuffer_ = 0;
    }
}

void VisualizationPanel::handleMouseInput(const ImVec2& windowSize, const ImVec2& windowPos) {
    if (!cameraController_) {
        return;
    }

    ImGuiIO& io = ImGui::GetIO();

    // Get mouse position relative to the window
    ImVec2 mousePos = ImGui::GetMousePos();
    double mouseX = mousePos.x - windowPos.x;
    double mouseY = mousePos.y - windowPos.y;

    // Mouse button state
    bool leftButton = ImGui::IsMouseDown(ImGuiMouseButton_Left);
    bool middleButton = ImGui::IsMouseDown(ImGuiMouseButton_Middle);
    bool rightButton = ImGui::IsMouseDown(ImGuiMouseButton_Right);

    // Scroll (zoom)
    float scroll = io.MouseWheel;

    // Handle mouse button press/release
    static bool wasLeftPressed = false;
    static bool wasMiddlePressed = false;

    if (leftButton && !wasLeftPressed) {
        cameraController_->onMousePress(0, mouseX, mouseY);
    } else if (!leftButton && wasLeftPressed) {
        cameraController_->onMouseRelease(0);
    }

    if (middleButton && !wasMiddlePressed) {
        cameraController_->onMousePress(1, mouseX, mouseY);
    } else if (!middleButton && wasMiddlePressed) {
        cameraController_->onMouseRelease(1);
    }

    wasLeftPressed = leftButton;
    wasMiddlePressed = middleButton;

    // Handle mouse movement
    if (isPanelHovered_ && (leftButton || middleButton)) {
        cameraController_->onMouseMove(mouseX, mouseY);
    }

    // Handle scroll (zoom)
    if (isPanelHovered_ && scroll != 0.0f) {
        cameraController_->onMouseScroll(scroll);
    }
}

}  // namespace ui
}  // namespace vi_slam
