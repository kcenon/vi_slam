#ifndef VI_SLAM_VISUALIZER_POINT_CLOUD_RENDERER_HPP
#define VI_SLAM_VISUALIZER_POINT_CLOUD_RENDERER_HPP

#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace vi_slam {
namespace visualizer {

/**
 * @brief Point cloud renderer for 3D map points visualization
 *
 * Renders point clouds using OpenGL vertex buffer objects (VBO) for efficient
 * rendering of large point sets (100K+ points).
 */
class PointCloudRenderer {
public:
    /**
     * @brief Point cloud rendering configuration
     */
    struct Config {
        float pointSize = 2.0f;                         // Point size in pixels
        Eigen::Vector3f defaultColor = Eigen::Vector3f(1.0f, 1.0f, 1.0f);  // Default white
        bool useVertexColors = false;                   // Use per-vertex colors if available
    };

    PointCloudRenderer();
    ~PointCloudRenderer();

    /**
     * @brief Initialize the point cloud renderer
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
     * @brief Update point cloud data
     * @param points Vector of 3D point positions
     * @param colors Optional vector of RGB colors (must match points size if provided)
     */
    void updatePointCloud(const std::vector<Eigen::Vector3d>& points,
                          const std::vector<Eigen::Vector3f>& colors = {});

    /**
     * @brief Render the point cloud
     * @param viewMatrix 4x4 view transformation matrix
     * @param projMatrix 4x4 projection matrix
     */
    void render(const Eigen::Matrix4f& viewMatrix, const Eigen::Matrix4f& projMatrix);

    /**
     * @brief Clear the point cloud
     */
    void clear();

    /**
     * @brief Get the number of points in the cloud
     */
    size_t getPointCount() const { return pointCount_; }

    /**
     * @brief Set point size
     * @param size Point size in pixels
     */
    void setPointSize(float size);

    /**
     * @brief Set default color for points without individual colors
     * @param color RGB color (0.0 to 1.0)
     */
    void setDefaultColor(const Eigen::Vector3f& color);

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

    bool initialized_;
    Config config_;

    // OpenGL objects
    unsigned int vbo_;          // Vertex Buffer Object (positions)
    unsigned int colorVbo_;     // Color Buffer Object
    unsigned int shaderProgram_;

    size_t pointCount_;
    bool hasColors_;
};

} // namespace visualizer
} // namespace vi_slam

#endif // VI_SLAM_VISUALIZER_POINT_CLOUD_RENDERER_HPP
