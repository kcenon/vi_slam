#ifndef VI_SLAM_I_VISUALIZER_HPP
#define VI_SLAM_I_VISUALIZER_HPP

#include "common/types.hpp"
#include <memory>
#include <string>
#include <vector>

namespace vi_slam {
namespace visualizer {

// Visualizer configuration
struct VisualizerConfig {
    int windowWidth = 1280;
    int windowHeight = 720;
    std::string windowTitle = "VI-SLAM Visualizer";
    int targetFPS = 60;
    bool enableVSync = true;
};

// Interface for visualizer implementations
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
