#ifndef VI_SLAM_I_VISUALIZER_HPP
#define VI_SLAM_I_VISUALIZER_HPP

/**
 * @file i_visualizer.hpp
 * @brief Abstract interface for SLAM visualization backends
 */

#include "common/types.hpp"
#include <memory>
#include <string>
#include <vector>

namespace vi_slam {
namespace visualizer {

/**
 * @brief Configuration parameters for visualizer
 */
struct VisualizerConfig {
    int windowWidth = 1280;          ///< Window width in pixels
    int windowHeight = 720;          ///< Window height in pixels
    std::string windowTitle = "VI-SLAM Visualizer"; ///< Window title
    int targetFPS = 60;              ///< Target frame rate
    bool enableVSync = true;         ///< Enable vertical sync
};

/**
 * @brief Abstract interface for visualization implementations
 *
 * IVisualizer defines the contract for rendering SLAM output including
 * camera trajectory, 3D map points, and tracking status overlays.
 */
class IVisualizer {
public:
    virtual ~IVisualizer() = default;

    // Initialization
    virtual bool initialize(const VisualizerConfig& config) = 0;
    virtual void shutdown() = 0;

    // Window management
    virtual bool isRunning() const = 0;
    virtual void processEvents() = 0;

    // Rendering
    virtual void beginFrame() = 0;
    virtual void endFrame() = 0;
    virtual void clear() = 0;

    // Data visualization
    virtual void updateTrajectory(const std::vector<Pose6DoF>& trajectory) = 0;
    virtual void updateMapPoints(const std::vector<MapPoint>& points) = 0;
    virtual void updateCurrentPose(const Pose6DoF& pose) = 0;

    // Status overlay
    virtual void updateFPS(double fps) = 0;
    virtual void updateTrackingStatus(TrackingStatus status) = 0;

    // Performance monitoring
    virtual double getFrameTime() const = 0;
    virtual double getFPS() const = 0;
    virtual double getCPUUsage() const = 0;
};

}  // namespace visualizer
}  // namespace vi_slam

#endif  // VI_SLAM_I_VISUALIZER_HPP
