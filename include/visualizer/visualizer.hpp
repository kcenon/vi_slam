#ifndef VI_SLAM_VISUALIZER_HPP
#define VI_SLAM_VISUALIZER_HPP

/**
 * @file visualizer.hpp
 * @brief OpenGL-based SLAM visualizer implementation
 */

#include "visualizer/i_visualizer.hpp"
#include <chrono>
#include <memory>
#include <mutex>

// Forward declarations for GLFW
struct GLFWwindow;

namespace vi_slam {
namespace visualizer {

/**
 * @brief OpenGL-based visualizer implementation using GLFW
 *
 * Provides real-time 3D visualization of SLAM output including
 * camera trajectory, map points, and performance metrics.
 * Uses GLFW for window management and OpenGL for rendering.
 */
class Visualizer : public IVisualizer {
public:
    Visualizer();
    ~Visualizer() override;

    // Prevent copying
    Visualizer(const Visualizer&) = delete;
    Visualizer& operator=(const Visualizer&) = delete;

    // IVisualizer interface
    bool initialize(const VisualizerConfig& config) override;
    void shutdown() override;

    bool isRunning() const override;
    void processEvents() override;

    void beginFrame() override;
    void endFrame() override;
    void clear() override;

    void updateTrajectory(const std::vector<Pose6DoF>& trajectory) override;
    void updateMapPoints(const std::vector<MapPoint>& points) override;
    void updateCurrentPose(const Pose6DoF& pose) override;

    void updateFPS(double fps) override;
    void updateTrackingStatus(TrackingStatus status) override;

    double getFrameTime() const override;
    double getFPS() const override;
    double getCPUUsage() const override;

private:
    // GLFW window initialization
    bool initializeGLFW();
    bool initializeOpenGL();
    void cleanupGLFW();

    // Rendering helpers
    void setupViewport();
    void calculateFrameTiming();

    // GLFW window handle
    GLFWwindow* window_;

    // Configuration
    VisualizerConfig config_;

    // State
    bool initialized_;
    bool running_;

    // Data
    std::vector<Pose6DoF> trajectory_;
    std::vector<MapPoint> mapPoints_;
    Pose6DoF currentPose_;
    TrackingStatus trackingStatus_;

    // Performance metrics
    double fps_;
    double frameTime_;
    double cpuUsage_;
    std::chrono::steady_clock::time_point lastFrameTime_;
    std::chrono::steady_clock::time_point initTime_;

    // Thread safety
    mutable std::mutex dataMutex_;
};

}  // namespace visualizer
}  // namespace vi_slam

#endif  // VI_SLAM_VISUALIZER_HPP
