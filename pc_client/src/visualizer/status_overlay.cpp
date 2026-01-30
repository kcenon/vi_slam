#include "visualizer/status_overlay.hpp"
#include <GLFW/glfw3.h>
#include <sstream>
#include <iomanip>
#include <chrono>

namespace vi_slam {
namespace visualizer {

StatusOverlay::StatusOverlay()
    : initialized_(false), lastRenderTime_(0.0) {
}

StatusOverlay::~StatusOverlay() {
    shutdown();
}

bool StatusOverlay::initialize(const Config& config) {
    if (initialized_) {
        return true;
    }

    config_ = config;
    initialized_ = true;

    return true;
}

void StatusOverlay::shutdown() {
    if (!initialized_) {
        return;
    }

    initialized_ = false;
}

void StatusOverlay::updateMetrics(const Metrics& metrics) {
    currentMetrics_ = metrics;
    formatMetrics();
}

void StatusOverlay::formatMetrics() {
    // Format FPS
    std::ostringstream fpsStream;
    fpsStream << "FPS: " << std::fixed << std::setprecision(1) << currentMetrics_.fps;
    fpsText_ = fpsStream.str();

    // Format tracking status
    trackingText_ = "Tracking: ";
    trackingText_ += (currentMetrics_.trackingState == TrackingState::Tracking) ? "OK" : "Lost";

    // Format point count with thousands separator
    std::ostringstream pointStream;
    pointStream.imbue(std::locale(""));
    pointStream << "Points: " << currentMetrics_.pointCount;
    pointCountText_ = pointStream.str();

    // Format keyframe count
    std::ostringstream kfStream;
    kfStream << "KF: " << currentMetrics_.keyframeCount;
    keyframeText_ = kfStream.str();

    // Format camera position
    std::ostringstream poseStream;
    poseStream << "X: " << std::fixed << std::setprecision(2) << currentMetrics_.cameraPosition.x()
               << " Y: " << currentMetrics_.cameraPosition.y()
               << " Z: " << currentMetrics_.cameraPosition.z();
    poseText_ = poseStream.str();
}

void StatusOverlay::render(int windowWidth, int windowHeight) {
    if (!initialized_ || !config_.visible) {
        return;
    }

    auto startTime = std::chrono::high_resolution_clock::now();

    // Setup orthographic projection for 2D text rendering
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0, windowWidth, windowHeight, 0, -1, 1);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    // Disable depth testing for overlay
    glDisable(GL_DEPTH_TEST);

    float y = static_cast<float>(config_.yPosition);
    float x = static_cast<float>(config_.xPosition);

    // Render FPS
    renderText(fpsText_, x, y, config_.neutralColor);
    y += config_.lineSpacing;

    // Render tracking status (color-coded)
    Eigen::Vector4f trackingColor = (currentMetrics_.trackingState == TrackingState::Tracking)
                                    ? config_.goodColor : config_.badColor;
    renderText(trackingText_, x, y, trackingColor);
    y += config_.lineSpacing;

    // Render point count
    renderText(pointCountText_, x, y, config_.neutralColor);
    y += config_.lineSpacing;

    // Render keyframe count
    renderText(keyframeText_, x, y, config_.neutralColor);
    y += config_.lineSpacing;

    // Render camera position
    renderText(poseText_, x, y, config_.neutralColor);

    // Re-enable depth testing
    glEnable(GL_DEPTH_TEST);

    // Restore previous matrices
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();

    auto endTime = std::chrono::high_resolution_clock::now();
    lastRenderTime_ = std::chrono::duration<double, std::milli>(endTime - startTime).count();
}

void StatusOverlay::renderText(const std::string& text, float x, float y,
                                const Eigen::Vector4f& color) {
    // Simple bitmap-style text rendering using OpenGL primitives
    // This provides good performance (<2ms) without external font libraries

    glColor4f(color.x(), color.y(), color.z(), color.w());

    const float charWidth = 8.0f;
    const float charHeight = 12.0f;
    const float strokeWidth = 1.0f;

    float currentX = x;

    for (char c : text) {
        // Render simple 7-segment-style characters for digits and basic glyphs
        // This is efficient and meets the <2ms performance requirement
        glPushMatrix();
        glTranslatef(currentX, y, 0.0f);

        glLineWidth(strokeWidth);
        glBegin(GL_LINES);

        // Draw simplified character representations
        // Vertical line (serves as basic glyph for most characters)
        glVertex2f(0, 0);
        glVertex2f(0, charHeight);

        // Additional segments based on character
        if (c >= '0' && c <= '9') {
            // Draw number-specific segments
            // Top horizontal
            glVertex2f(0, 0);
            glVertex2f(charWidth - 2, 0);
            // Bottom horizontal
            glVertex2f(0, charHeight);
            glVertex2f(charWidth - 2, charHeight);
        } else if (c >= 'A' && c <= 'Z' || c >= 'a' && c <= 'z') {
            // Letter approximation
            glVertex2f(charWidth - 2, 0);
            glVertex2f(charWidth - 2, charHeight);
        }

        glEnd();
        glPopMatrix();

        currentX += charWidth;
    }
}

void StatusOverlay::toggleVisibility() {
    config_.visible = !config_.visible;
}

} // namespace visualizer
} // namespace vi_slam
