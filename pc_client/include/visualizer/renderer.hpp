#ifndef VI_SLAM_VISUALIZER_RENDERER_HPP
#define VI_SLAM_VISUALIZER_RENDERER_HPP

#include <memory>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

namespace vi_slam {
namespace visualizer {

/**
 * @brief Base renderer class for 3D visualization
 *
 * Provides the foundational rendering infrastructure using OpenGL.
 * Handles window creation, OpenGL context management, and the basic render loop.
 */
class Renderer {
public:
    /**
     * @brief Rendering configuration
     */
    struct Config {
        int width = 1280;           // Window width
        int height = 720;           // Window height
        std::string title = "VI-SLAM Visualizer";
        bool vsync = true;          // Enable vertical sync
        int targetFps = 60;         // Target FPS
        Eigen::Vector4f clearColor = Eigen::Vector4f(0.1f, 0.1f, 0.1f, 1.0f);
    };

    Renderer();
    virtual ~Renderer();

    /**
     * @brief Initialize the renderer
     * @param config Rendering configuration
     * @return true if initialization succeeded
     */
    bool initialize(const Config& config);

    /**
     * @brief Shutdown and cleanup resources
     */
    void shutdown();

    /**
     * @brief Check if the renderer is initialized
     */
    bool isInitialized() const { return initialized_; }

    /**
     * @brief Check if the window should close
     */
    bool shouldClose() const;

    /**
     * @brief Begin a new frame
     *
     * Clears the framebuffer and prepares for rendering
     */
    void beginFrame();

    /**
     * @brief End the current frame
     *
     * Swaps buffers and polls events
     */
    void endFrame();

    /**
     * @brief Set the view matrix
     * @param viewMatrix 4x4 view transformation matrix
     */
    void setViewMatrix(const Eigen::Matrix4f& viewMatrix);

    /**
     * @brief Set the projection matrix
     * @param projMatrix 4x4 projection matrix
     */
    void setProjectionMatrix(const Eigen::Matrix4f& projMatrix);

    /**
     * @brief Get the current view matrix
     */
    const Eigen::Matrix4f& getViewMatrix() const { return viewMatrix_; }

    /**
     * @brief Get the current projection matrix
     */
    const Eigen::Matrix4f& getProjectionMatrix() const { return projMatrix_; }

    /**
     * @brief Get window dimensions
     */
    void getWindowSize(int& width, int& height) const;

    /**
     * @brief Get the current FPS
     */
    float getFps() const { return currentFps_; }

    /**
     * @brief Create perspective projection matrix
     * @param fov Field of view in degrees
     * @param aspect Aspect ratio (width/height)
     * @param near Near clipping plane
     * @param far Far clipping plane
     */
    static Eigen::Matrix4f createPerspective(float fov, float aspect, float near, float far);

    /**
     * @brief Create look-at view matrix
     * @param eye Camera position
     * @param center Look-at target
     * @param up Up vector
     */
    static Eigen::Matrix4f createLookAt(const Eigen::Vector3f& eye,
                                        const Eigen::Vector3f& center,
                                        const Eigen::Vector3f& up);

protected:

private:
    // OpenGL window handle (using void* to avoid exposing GLFW in header)
    void* window_;

    bool initialized_;
    Config config_;

    Eigen::Matrix4f viewMatrix_;
    Eigen::Matrix4f projMatrix_;

    float currentFps_;
    double lastFrameTime_;
    int frameCount_;
    double fpsUpdateTime_;
};

} // namespace visualizer
} // namespace vi_slam

#endif // VI_SLAM_VISUALIZER_RENDERER_HPP
