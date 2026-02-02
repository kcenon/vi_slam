#include "ui/visualization_panel.hpp"
#include "visualizer/renderer.hpp"
#include "visualizer/point_cloud_renderer.hpp"
#include "visualizer/trajectory_renderer.hpp"
#include "visualizer/camera_controller.hpp"
#include "visualizer/status_overlay.hpp"
#include "imgui.h"
#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif
#include <iostream>
#include <cmath>

namespace vi_slam {
namespace ui {

VisualizationPanel::VisualizationPanel()
    : initialized_(false),
      framebuffer_(0),
      textureColorBuffer_(0),
      renderbuffer_(0),
      viewportWidth_(0),
      viewportHeight_(0),
      viewportResized_(false),
      windowFocused_(false),
      windowHovered_(false) {
}

VisualizationPanel::~VisualizationPanel() {
    shutdown();
}

bool VisualizationPanel::initialize(const Config& config) {
    if (initialized_) {
        std::cerr << "VisualizationPanel already initialized" << std::endl;
        return false;
    }

    config_ = config;
    viewportWidth_ = config.initialWidth;
    viewportHeight_ = config.initialHeight;

    // Initialize renderer
    renderer_ = std::make_unique<visualizer::Renderer>();
    visualizer::Renderer::Config rendererConfig;
    rendererConfig.width = viewportWidth_;
    rendererConfig.height = viewportHeight_;
    rendererConfig.title = "VI-SLAM Visualizer";

    if (!renderer_->initialize(rendererConfig)) {
        std::cerr << "Failed to initialize renderer" << std::endl;
        return false;
    }

    // Initialize visualizer components
    pointCloudRenderer_ = std::make_unique<visualizer::PointCloudRenderer>();
    visualizer::PointCloudRenderer::Config pcrConfig;
    if (!pointCloudRenderer_->initialize(pcrConfig)) {
        std::cerr << "Failed to initialize point cloud renderer" << std::endl;
        return false;
    }

    trajectoryRenderer_ = std::make_unique<visualizer::TrajectoryRenderer>();
    visualizer::TrajectoryRenderer::Config trajConfig;
    if (!trajectoryRenderer_->initialize(trajConfig)) {
        std::cerr << "Failed to initialize trajectory renderer" << std::endl;
        return false;
    }

    cameraController_ = std::make_unique<visualizer::CameraController>();
    visualizer::CameraController::Config camConfig;
    cameraController_->initialize(camConfig);

    if (config.showStatusOverlay) {
        statusOverlay_ = std::make_unique<visualizer::StatusOverlay>();
        visualizer::StatusOverlay::Config overlayConfig;
        if (!statusOverlay_->initialize(overlayConfig)) {
            std::cerr << "Failed to initialize status overlay" << std::endl;
            return false;
        }
    }

    // Create framebuffer for offscreen rendering
    if (!createFramebuffer(viewportWidth_, viewportHeight_)) {
        std::cerr << "Failed to create framebuffer" << std::endl;
        shutdown();
        return false;
    }

    initialized_ = true;
    std::cout << "VisualizationPanel initialized successfully" << std::endl;
    return true;
}

void VisualizationPanel::shutdown() {
    if (!initialized_) {
        return;
    }

    deleteFramebuffer();

    if (statusOverlay_) {
        statusOverlay_->shutdown();
        statusOverlay_.reset();
    }

    if (trajectoryRenderer_) {
        trajectoryRenderer_->shutdown();
        trajectoryRenderer_.reset();
    }

    if (pointCloudRenderer_) {
        pointCloudRenderer_->shutdown();
        pointCloudRenderer_.reset();
    }

    if (renderer_) {
        renderer_->shutdown();
        renderer_.reset();
    }

    cameraController_.reset();

    initialized_ = false;
    std::cout << "VisualizationPanel shut down" << std::endl;
}

void VisualizationPanel::render() {
    if (!initialized_) {
        return;
    }

    ImGui::Begin("3D Visualization", nullptr,
                 ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);

    // Get available content region size
    ImVec2 contentRegion = ImGui::GetContentRegionAvail();
    int newWidth = static_cast<int>(contentRegion.x);
    int newHeight = static_cast<int>(contentRegion.y);

    // Check if viewport needs resizing
    if (newWidth > 0 && newHeight > 0 &&
        (newWidth != viewportWidth_ || newHeight != viewportHeight_)) {
        viewportWidth_ = newWidth;
        viewportHeight_ = newHeight;
        viewportResized_ = true;
    }

    // Render scene to framebuffer
    if (viewportWidth_ > 0 && viewportHeight_ > 0) {
        renderScene();

        // Display framebuffer texture as ImGui image
        ImGui::Image(reinterpret_cast<void*>(static_cast<intptr_t>(textureColorBuffer_)),
                    ImVec2(static_cast<float>(viewportWidth_),
                           static_cast<float>(viewportHeight_)),
                    ImVec2(0, 1), ImVec2(1, 0));  // Flip Y coordinate

        // Track window focus and hover state
        windowFocused_ = ImGui::IsWindowFocused();
        windowHovered_ = ImGui::IsWindowHovered();
    } else {
        ImGui::Text("Viewport size: %dx%d (invalid)", newWidth, newHeight);
    }

    ImGui::End();
}

void VisualizationPanel::update() {
    if (!initialized_) {
        return;
    }

    // Resize framebuffer if needed
    if (viewportResized_) {
        resizeFramebuffer(viewportWidth_, viewportHeight_);
        viewportResized_ = false;
    }

    // Handle input if window is focused
    if (config_.enableControls && windowFocused_ && windowHovered_) {
        handleInput();
    }

    // Update camera controller
    if (cameraController_) {
        cameraController_->update(1.0f / 60.0f);  // Assume 60 FPS

        // Update renderer view matrix from camera
        Eigen::Matrix4f viewMatrix = cameraController_->getViewMatrix();
        renderer_->setViewMatrix(viewMatrix);
    }
}

bool VisualizationPanel::createFramebuffer(int width, int height) {
    // Generate framebuffer
    glGenFramebuffers(1, &framebuffer_);
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer_);

    // Create texture for color attachment
    glGenTextures(1, &textureColorBuffer_);
    glBindTexture(GL_TEXTURE_2D, textureColorBuffer_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
                          textureColorBuffer_, 0);

    // Create renderbuffer for depth and stencil attachment
    glGenRenderbuffers(1, &renderbuffer_);
    glBindRenderbuffer(GL_RENDERBUFFER, renderbuffer_);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, width, height);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT,
                             GL_RENDERBUFFER, renderbuffer_);

    // Check framebuffer completeness
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        std::cerr << "Framebuffer is not complete" << std::endl;
        deleteFramebuffer();
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        return false;
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    return true;
}

void VisualizationPanel::resizeFramebuffer(int width, int height) {
    if (width <= 0 || height <= 0) {
        return;
    }

    // Delete old framebuffer
    deleteFramebuffer();

    // Create new framebuffer with new size
    createFramebuffer(width, height);

    // Update renderer projection matrix
    float aspect = static_cast<float>(width) / static_cast<float>(height);
    float fov = 60.0f;
    float nearPlane = 0.1f;
    float farPlane = 1000.0f;

    // Create perspective projection matrix manually
    float tanHalfFovy = std::tan(fov * 0.5f * M_PI / 180.0f);
    Eigen::Matrix4f projMatrix = Eigen::Matrix4f::Zero();
    projMatrix(0, 0) = 1.0f / (aspect * tanHalfFovy);
    projMatrix(1, 1) = 1.0f / tanHalfFovy;
    projMatrix(2, 2) = -(farPlane + nearPlane) / (farPlane - nearPlane);
    projMatrix(2, 3) = -(2.0f * farPlane * nearPlane) / (farPlane - nearPlane);
    projMatrix(3, 2) = -1.0f;

    renderer_->setProjectionMatrix(projMatrix);
}

void VisualizationPanel::deleteFramebuffer() {
    if (renderbuffer_ != 0) {
        glDeleteRenderbuffers(1, &renderbuffer_);
        renderbuffer_ = 0;
    }
    if (textureColorBuffer_ != 0) {
        glDeleteTextures(1, &textureColorBuffer_);
        textureColorBuffer_ = 0;
    }
    if (framebuffer_ != 0) {
        glDeleteFramebuffers(1, &framebuffer_);
        framebuffer_ = 0;
    }
}

void VisualizationPanel::renderScene() {
    // Bind framebuffer for offscreen rendering
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer_);
    glViewport(0, 0, viewportWidth_, viewportHeight_);

    // Clear framebuffer
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Render 3D content
    if (renderer_) {
        renderer_->beginFrame();

        // Render visualizer components
        if (pointCloudRenderer_) {
            pointCloudRenderer_->render(renderer_->getViewMatrix(),
                                       renderer_->getProjectionMatrix());
        }

        if (trajectoryRenderer_) {
            trajectoryRenderer_->render(renderer_->getViewMatrix(),
                                       renderer_->getProjectionMatrix());
        }

        if (statusOverlay_ && config_.showStatusOverlay) {
            statusOverlay_->render(viewportWidth_, viewportHeight_);
        }

        renderer_->endFrame();
    }

    // Unbind framebuffer (return to default)
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void VisualizationPanel::handleInput() {
    if (!cameraController_) {
        return;
    }

    ImGuiIO& io = ImGui::GetIO();
    ImVec2 mousePos = ImGui::GetMousePos();

    // Track mouse button state for camera control
    static bool leftButtonPressed = false;
    static bool rightButtonPressed = false;
    static bool middleButtonPressed = false;

    // Mouse button press events
    if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        cameraController_->onMousePress(0, mousePos.x, mousePos.y);
        leftButtonPressed = true;
    }
    if (ImGui::IsMouseClicked(ImGuiMouseButton_Right)) {
        cameraController_->onMousePress(1, mousePos.x, mousePos.y);
        rightButtonPressed = true;
    }
    if (ImGui::IsMouseClicked(ImGuiMouseButton_Middle)) {
        cameraController_->onMousePress(2, mousePos.x, mousePos.y);
        middleButtonPressed = true;
    }

    // Mouse button release events
    if (ImGui::IsMouseReleased(ImGuiMouseButton_Left) && leftButtonPressed) {
        cameraController_->onMouseRelease(0);
        leftButtonPressed = false;
    }
    if (ImGui::IsMouseReleased(ImGuiMouseButton_Right) && rightButtonPressed) {
        cameraController_->onMouseRelease(1);
        rightButtonPressed = false;
    }
    if (ImGui::IsMouseReleased(ImGuiMouseButton_Middle) && middleButtonPressed) {
        cameraController_->onMouseRelease(2);
        middleButtonPressed = false;
    }

    // Mouse move events
    if (leftButtonPressed || rightButtonPressed || middleButtonPressed) {
        cameraController_->onMouseMove(mousePos.x, mousePos.y);
    }

    // Mouse wheel for zoom
    if (io.MouseWheel != 0.0f) {
        cameraController_->onMouseScroll(io.MouseWheel);
    }

    // Keyboard shortcuts
    if (ImGui::IsKeyPressed(ImGuiKey_R)) {
        cameraController_->reset();
    }
}

}  // namespace ui
}  // namespace vi_slam
