#ifndef VI_SLAM_UI_VISUALIZATION_PANEL_HPP
#define VI_SLAM_UI_VISUALIZATION_PANEL_HPP

#include <memory>
#include "visualizer/renderer.hpp"
#include "visualizer/camera_controller.hpp"
#include "visualizer/trajectory_renderer.hpp"
#include "visualizer/point_cloud_renderer.hpp"

struct GLFWwindow;
struct ImVec2;

namespace vi_slam {
namespace ui {

/**
 * 3D Visualization panel for ImGui.
 *
 * Integrates the 3D visualizer into the ImGui dashboard by:
 * - Creating an OpenGL framebuffer for off-screen rendering
 * - Rendering the 3D scene to the framebuffer
 * - Displaying the framebuffer texture in an ImGui window
 * - Forwarding mouse input to the camera controller
 */
class VisualizationPanel {
public:
    VisualizationPanel();
    ~VisualizationPanel();

    /**
     * Initialize the visualization panel.
     *
     * @param window Main GLFW window for sharing OpenGL context
     * @return true if initialization succeeded
     */
    bool initialize(GLFWwindow* window);

    /**
     * Shutdown and cleanup resources.
     */
    void shutdown();

    /**
     * Update panel state (call in main loop before render).
     */
    void update();

    /**
     * Render the visualization panel.
     */
    void render();

    /**
     * Check if the panel is initialized.
     */
    bool isInitialized() const { return initialized_; }

    /**
     * Get the underlying renderer.
     */
    visualizer::Renderer* getRenderer() { return renderer_.get(); }

    /**
     * Get the camera controller.
     */
    visualizer::CameraController* getCameraController() { return cameraController_.get(); }

    /**
     * Get the trajectory renderer.
     */
    visualizer::TrajectoryRenderer* getTrajectoryRenderer() { return trajectoryRenderer_.get(); }

    /**
     * Get the point cloud renderer.
     */
    visualizer::PointCloudRenderer* getPointCloudRenderer() { return pointCloudRenderer_.get(); }

private:
    bool initialized_;

    // Main GLFW window reference
    GLFWwindow* mainWindow_;

    // Visualizer components
    std::unique_ptr<visualizer::Renderer> renderer_;
    std::unique_ptr<visualizer::CameraController> cameraController_;
    std::unique_ptr<visualizer::TrajectoryRenderer> trajectoryRenderer_;
    std::unique_ptr<visualizer::PointCloudRenderer> pointCloudRenderer_;

    // Framebuffer for off-screen rendering
    unsigned int framebuffer_;
    unsigned int textureId_;
    unsigned int depthRenderbuffer_;
    int framebufferWidth_;
    int framebufferHeight_;

    // UI state
    bool isPanelHovered_;
    bool isPanelFocused_;

    /**
     * Create OpenGL framebuffer for off-screen rendering.
     *
     * @param width Framebuffer width
     * @param height Framebuffer height
     * @return true if creation succeeded
     */
    bool createFramebuffer(int width, int height);

    /**
     * Resize the framebuffer.
     *
     * @param width New width
     * @param height New height
     */
    void resizeFramebuffer(int width, int height);

    /**
     * Cleanup framebuffer resources.
     */
    void cleanupFramebuffer();

    /**
     * Handle mouse input for camera control.
     *
     * @param windowSize Size of the ImGui window
     * @param windowPos Position of the ImGui window
     */
    void handleMouseInput(const ImVec2& windowSize, const ImVec2& windowPos);
};

}  // namespace ui
}  // namespace vi_slam

#endif  // VI_SLAM_UI_VISUALIZATION_PANEL_HPP
