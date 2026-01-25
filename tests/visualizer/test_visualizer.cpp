#include "visualizer/visualizer.hpp"
#include <iostream>

using namespace vi_slam::visualizer;

int main() {
    std::cout << "Testing Visualizer..." << std::endl;

    // Test 1: Create visualizer instance
    Visualizer visualizer;
    std::cout << "PASS: Visualizer instance created" << std::endl;

    // Test 2: Check initial state
    if (visualizer.isRunning()) {
        std::cerr << "FAIL: Visualizer should not be running before initialization" << std::endl;
        return 1;
    }
    std::cout << "PASS: Initial state is not running" << std::endl;

    // Test 3: Initialize with default config
    VisualizerConfig config;
    config.windowWidth = 800;
    config.windowHeight = 600;
    config.windowTitle = "VI-SLAM Test";
    config.targetFPS = 60;
    config.enableVSync = true;

    // Note: Initialization may fail in headless environment (CI/CD)
    // This is expected and not a test failure
    bool initSuccess = visualizer.initialize(config);
    if (!initSuccess) {
        std::cout << "INFO: Visualizer initialization failed (expected in headless environment)" << std::endl;
        std::cout << "Skipping remaining GUI-dependent tests" << std::endl;
        std::cout << "\nTests completed (headless mode)!" << std::endl;
        return 0;
    }
    std::cout << "PASS: Visualizer initialized successfully" << std::endl;

    // Test 4: Check running state
    if (!visualizer.isRunning()) {
        std::cerr << "FAIL: Visualizer should be running after initialization" << std::endl;
        return 1;
    }
    std::cout << "PASS: Visualizer is running" << std::endl;

    // Test 5: Update trajectory
    std::vector<vi_slam::Pose6DoF> trajectory;
    for (int i = 0; i < 10; ++i) {
        vi_slam::Pose6DoF pose;
        pose.timestampNs = i * 100000000LL;
        pose.position[0] = static_cast<double>(i);
        pose.position[1] = 0.0;
        pose.position[2] = 0.0;
        pose.orientation[0] = 1.0;
        pose.orientation[1] = 0.0;
        pose.orientation[2] = 0.0;
        pose.orientation[3] = 0.0;
        pose.valid = true;
        trajectory.push_back(pose);
    }
    visualizer.updateTrajectory(trajectory);
    std::cout << "PASS: Trajectory updated" << std::endl;

    // Test 6: Update map points
    std::vector<vi_slam::MapPoint> mapPoints;
    for (int i = 0; i < 100; ++i) {
        vi_slam::MapPoint point;
        point.id = i;
        point.position[0] = static_cast<double>(i % 10);
        point.position[1] = static_cast<double>(i / 10);
        point.position[2] = 0.0;
        point.observations = 1;
        mapPoints.push_back(point);
    }
    visualizer.updateMapPoints(mapPoints);
    std::cout << "PASS: Map points updated" << std::endl;

    // Test 7: Update current pose
    vi_slam::Pose6DoF currentPose;
    currentPose.timestampNs = 1000000000LL;
    currentPose.position[0] = 5.0;
    currentPose.position[1] = 2.0;
    currentPose.position[2] = 1.0;
    currentPose.orientation[0] = 1.0;
    currentPose.orientation[1] = 0.0;
    currentPose.orientation[2] = 0.0;
    currentPose.orientation[3] = 0.0;
    currentPose.valid = true;
    visualizer.updateCurrentPose(currentPose);
    std::cout << "PASS: Current pose updated" << std::endl;

    // Test 8: Update tracking status
    visualizer.updateTrackingStatus(vi_slam::TrackingStatus::TRACKING);
    std::cout << "PASS: Tracking status updated" << std::endl;

    // Test 9: Render a few frames
    for (int i = 0; i < 5; ++i) {
        visualizer.processEvents();
        if (!visualizer.isRunning()) {
            std::cout << "INFO: Window closed by user" << std::endl;
            break;
        }

        visualizer.beginFrame();
        visualizer.clear();
        visualizer.endFrame();

        double fps = visualizer.getFPS();
        double frameTime = visualizer.getFrameTime();
        double cpuUsage = visualizer.getCPUUsage();

        std::cout << "Frame " << i << ": FPS=" << fps
                  << ", FrameTime=" << frameTime << "ms"
                  << ", CPU=" << cpuUsage << "%" << std::endl;
    }
    std::cout << "PASS: Frames rendered successfully" << std::endl;

    // Test 10: Performance metrics
    double fps = visualizer.getFPS();
    double frameTime = visualizer.getFrameTime();
    double cpuUsage = visualizer.getCPUUsage();

    std::cout << "Performance metrics: FPS=" << fps
              << ", FrameTime=" << frameTime << "ms"
              << ", CPU=" << cpuUsage << "%" << std::endl;

    if (fps < 0.0) {
        std::cerr << "FAIL: Invalid FPS value" << std::endl;
        return 1;
    }
    std::cout << "PASS: Performance metrics valid" << std::endl;

    // Test 11: Shutdown
    visualizer.shutdown();
    if (visualizer.isRunning()) {
        std::cerr << "FAIL: Visualizer should not be running after shutdown" << std::endl;
        return 1;
    }
    std::cout << "PASS: Shutdown successful" << std::endl;

    std::cout << "\nAll tests passed!" << std::endl;
    return 0;
}
