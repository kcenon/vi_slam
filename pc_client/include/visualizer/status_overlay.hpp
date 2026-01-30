#ifndef VI_SLAM_VISUALIZER_STATUS_OVERLAY_HPP
#define VI_SLAM_VISUALIZER_STATUS_OVERLAY_HPP

#include <string>
#include <Eigen/Core>

namespace vi_slam {
namespace visualizer {

/**
 * @brief Status overlay displaying real-time system metrics
 *
 * Renders text overlay showing FPS, tracking status, point cloud size,
 * keyframe count, and camera pose on the 3D visualization.
 */
class StatusOverlay {
public:
    /**
     * @brief Tracking state
     */
    enum class TrackingState {
        Tracking,   // SLAM is actively tracking
        Lost        // Tracking is lost
    };

    /**
     * @brief Status metrics to display
     */
    struct Metrics {
        float fps = 0.0f;
        TrackingState trackingState = TrackingState::Lost;
        size_t pointCount = 0;
        size_t keyframeCount = 0;
        Eigen::Vector3f cameraPosition = Eigen::Vector3f::Zero();
    };

    /**
     * @brief Overlay configuration
     */
    struct Config {
        int xPosition = 10;         // X position in pixels from left
        int yPosition = 10;         // Y position in pixels from top
        int fontSize = 16;          // Font size in pixels
        int lineSpacing = 20;       // Spacing between lines in pixels
        bool visible = true;        // Initial visibility
        Eigen::Vector4f goodColor = Eigen::Vector4f(0.0f, 1.0f, 0.0f, 1.0f);   // Green
        Eigen::Vector4f badColor = Eigen::Vector4f(1.0f, 0.0f, 0.0f, 1.0f);    // Red
        Eigen::Vector4f neutralColor = Eigen::Vector4f(1.0f, 1.0f, 1.0f, 1.0f); // White
    };

    StatusOverlay();
    ~StatusOverlay();

    /**
     * @brief Initialize the overlay
     * @param config Overlay configuration
     * @return true if initialization succeeded
     */
    bool initialize(const Config& config);

    /**
     * @brief Shutdown and cleanup resources
     */
    void shutdown();

    /**
     * @brief Check if the overlay is initialized
     */
    bool isInitialized() const { return initialized_; }

    /**
     * @brief Update metrics to display
     * @param metrics Current system metrics
     */
    void updateMetrics(const Metrics& metrics);

    /**
     * @brief Render the overlay
     * @param windowWidth Current window width
     * @param windowHeight Current window height
     */
    void render(int windowWidth, int windowHeight);

    /**
     * @brief Toggle overlay visibility
     */
    void toggleVisibility();

    /**
     * @brief Set overlay visibility
     * @param visible Visibility state
     */
    void setVisible(bool visible) { config_.visible = visible; }

    /**
     * @brief Check if overlay is visible
     */
    bool isVisible() const { return config_.visible; }

    /**
     * @brief Get the last render time in milliseconds
     * @return Render time in ms
     */
    double getLastRenderTime() const { return lastRenderTime_; }

private:
    /**
     * @brief Render a single line of text
     * @param text Text to render
     * @param x X position in pixels
     * @param y Y position in pixels
     * @param color Text color (RGBA)
     */
    void renderText(const std::string& text, float x, float y,
                    const Eigen::Vector4f& color);

    /**
     * @brief Format metrics into display strings
     */
    void formatMetrics();

    bool initialized_;
    Config config_;
    Metrics currentMetrics_;

    // Formatted display strings
    std::string fpsText_;
    std::string trackingText_;
    std::string pointCountText_;
    std::string keyframeText_;
    std::string poseText_;

    // Performance tracking
    double lastRenderTime_;
};

} // namespace visualizer
} // namespace vi_slam

#endif // VI_SLAM_VISUALIZER_STATUS_OVERLAY_HPP
