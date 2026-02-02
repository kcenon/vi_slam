#ifndef VI_SLAM_UI_VISUALIZATION_PANEL_HPP
#define VI_SLAM_UI_VISUALIZATION_PANEL_HPP

#include <memory>
#include <Eigen/Core>
#include <Eigen/Geometry>

// Forward declarations
namespace vi_slam {
namespace visualizer {
class Renderer;
class PointCloudRenderer;
class TrajectoryRenderer;
class CameraController;
class StatusOverlay;
}  // namespace visualizer
}  // namespace vi_slam

namespace vi_slam {
namespace ui {

/**
 * 3D Visualization panel for ImGui dashboard.
 *
 * Integrates the 3D visualizer module into ImGui by rendering to a framebuffer
 * and displaying it as an ImGui image. Forwards mouse and keyboard input to
 * the visualizer for camera controls.
 */
class VisualizationPanel {
public:
    /**
     * Panel configuration.
     */
    struct Config {
        int initialWidth;
        int initialHeight;
        bool enableControls;
        bool showStatusOverlay;

        Config()
            : initialWidth(800),
              initialHeight(600),
              enableControls(true),
              showStatusOverlay(true) {}
    };

    VisualizationPanel();
    ~VisualizationPanel();

    /**
     * Initialize the visualization panel.
     *
     * Creates the visualizer renderer and framebuffer for offscreen rendering.
     *
     * @param config Panel configuration
     * @return true if initialization succeeded
     */
    bool initialize(const Config& config = Config());

    /**
     * Shutdown and cleanup resources.
     */
    void shutdown();

    /**
     * Check if the panel is initialized.
     */
    bool isInitialized() const { return initialized_; }

    /**
     * Render the visualization panel.
     *
     * Displays the 3D visualizer viewport within an ImGui window.
     * Handles input forwarding and viewport resizing.
     */
    void render();

    /**
     * Update panel state (call in main loop before render).
     *
     * Updates camera controller and renderer state.
     */
    void update();

    /**
     * Get the visualizer renderer.
     *
     * @return Pointer to renderer, or nullptr if not initialized
     */
    visualizer::Renderer* getRenderer() const { return renderer_.get(); }

    /**
     * Get the point cloud renderer.
     *
     * @return Pointer to point cloud renderer, or nullptr if not initialized
     */
    visualizer::PointCloudRenderer* getPointCloudRenderer() const {
        return pointCloudRenderer_.get();
    }

    /**
     * Get the trajectory renderer.
     *
     * @return Pointer to trajectory renderer, or nullptr if not initialized
     */
    visualizer::TrajectoryRenderer* getTrajectoryRenderer() const {
        return trajectoryRenderer_.get();
    }

private:
    bool initialized_;
    Config config_;

    // Visualizer components
    std::unique_ptr<visualizer::Renderer> renderer_;
    std::unique_ptr<visualizer::PointCloudRenderer> pointCloudRenderer_;
    std::unique_ptr<visualizer::TrajectoryRenderer> trajectoryRenderer_;
    std::unique_ptr<visualizer::CameraController> cameraController_;
    std::unique_ptr<visualizer::StatusOverlay> statusOverlay_;

    // Framebuffer for offscreen rendering
    unsigned int framebuffer_;
    unsigned int textureColorBuffer_;
    unsigned int renderbuffer_;

    // Viewport state
    int viewportWidth_;
    int viewportHeight_;
    bool viewportResized_;

    // Window state
    bool windowFocused_;
    bool windowHovered_;

    /**
     * Create framebuffer for offscreen rendering.
     *
     * @param width Framebuffer width
     * @param height Framebuffer height
     * @return true if creation succeeded
     */
    bool createFramebuffer(int width, int height);

    /**
     * Resize framebuffer when viewport size changes.
     *
     * @param width New width
     * @param height New height
     */
    void resizeFramebuffer(int width, int height);

    /**
     * Delete framebuffer and associated resources.
     */
    void deleteFramebuffer();

    /**
     * Render the 3D scene to the framebuffer.
     */
    void renderScene();

    /**
     * Handle mouse input for camera controls.
     */
    void handleInput();
};

}  // namespace ui
}  // namespace vi_slam

#endif  // VI_SLAM_UI_VISUALIZATION_PANEL_HPP
