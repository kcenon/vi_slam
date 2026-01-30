#ifndef VI_SLAM_VISUALIZER_TRAJECTORY_RENDERER_HPP
#define VI_SLAM_VISUALIZER_TRAJECTORY_RENDERER_HPP

#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace vi_slam {
namespace visualizer {

/**
 * @brief Trajectory renderer for camera path visualization
 *
 * Renders camera trajectory as connected line segments with color-coded tracking
 * quality and camera frustums at key poses.
 */
class TrajectoryRenderer {
public:
    /**
     * @brief Trajectory rendering configuration
     */
    struct Config {
        float lineWidth = 2.0f;                            // Trajectory line width
        float frustumSize = 0.1f;                          // Camera frustum size
        int frustumInterval = 10;                          // Draw frustum every N poses
        Eigen::Vector3f goodTrackColor = Eigen::Vector3f(0.0f, 1.0f, 0.0f);  // Green for good tracking
        Eigen::Vector3f poorTrackColor = Eigen::Vector3f(1.0f, 0.0f, 0.0f);  // Red for poor tracking
    };

    /**
     * @brief Camera pose with tracking quality
     */
    struct Pose {
        Eigen::Isometry3d transform;    // Camera pose (position + orientation)
        float trackingQuality;          // Quality metric [0.0, 1.0], 1.0 = best
    };

    TrajectoryRenderer();
    ~TrajectoryRenderer();

    /**
     * @brief Initialize the trajectory renderer
     * @param config Rendering configuration
     * @return true if initialization succeeded
     */
    bool initialize(const Config& config);

    /**
     * @brief Shutdown and cleanup OpenGL resources
     */
    void shutdown();

    /**
     * @brief Check if the renderer is initialized
     */
    bool isInitialized() const { return initialized_; }

    /**
     * @brief Update trajectory data
     * @param poses Vector of camera poses with tracking quality
     */
    void updateTrajectory(const std::vector<Pose>& poses);

    /**
     * @brief Render the trajectory
     * @param viewMatrix 4x4 view transformation matrix
     * @param projMatrix 4x4 projection matrix
     */
    void render(const Eigen::Matrix4f& viewMatrix, const Eigen::Matrix4f& projMatrix);

    /**
     * @brief Clear the trajectory
     */
    void clear();

    /**
     * @brief Get the number of poses in the trajectory
     */
    size_t getPoseCount() const { return poseCount_; }

    /**
     * @brief Set line width
     * @param width Line width in pixels
     */
    void setLineWidth(float width);

    /**
     * @brief Set frustum size
     * @param size Frustum size scale
     */
    void setFrustumSize(float size);

private:
    /**
     * @brief Compile and link shaders
     * @return true if shader compilation succeeded
     */
    bool createShaders();

    /**
     * @brief Create and bind vertex buffer objects
     */
    void createBuffers();

    /**
     * @brief Delete OpenGL buffers and vertex arrays
     */
    void deleteBuffers();

    /**
     * @brief Generate frustum geometry for a camera pose
     * @param pose Camera pose
     * @param vertices Output vertex positions
     * @param colors Output vertex colors
     */
    void generateFrustum(const Eigen::Isometry3d& pose,
                        std::vector<Eigen::Vector3f>& vertices,
                        std::vector<Eigen::Vector3f>& colors,
                        const Eigen::Vector3f& color) const;

    /**
     * @brief Interpolate color based on tracking quality
     * @param quality Tracking quality [0.0, 1.0]
     * @return Interpolated color
     */
    Eigen::Vector3f interpolateColor(float quality) const;

    bool initialized_;
    Config config_;

    // OpenGL objects
    unsigned int lineVbo_;          // Vertex Buffer Object for trajectory lines
    unsigned int lineColorVbo_;     // Color Buffer Object for lines
    unsigned int frustumVbo_;       // Vertex Buffer Object for frustums
    unsigned int frustumColorVbo_;  // Color Buffer Object for frustums
    unsigned int shaderProgram_;

    size_t poseCount_;
    size_t lineVertexCount_;
    size_t frustumVertexCount_;
};

} // namespace visualizer
} // namespace vi_slam

#endif // VI_SLAM_VISUALIZER_TRAJECTORY_RENDERER_HPP
