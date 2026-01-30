#ifndef VI_SLAM_VISUALIZER_CAMERA_CONTROLLER_HPP
#define VI_SLAM_VISUALIZER_CAMERA_CONTROLLER_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace vi_slam {
namespace visualizer {

/**
 * @brief Interactive camera controller for 3D view manipulation
 *
 * Handles mouse and keyboard input to control the camera view.
 * Supports orbit (rotate), pan, and zoom operations.
 */
class CameraController {
public:
    /**
     * @brief Camera controller configuration
     */
    struct Config {
        Eigen::Vector3f initialEye = Eigen::Vector3f(0.0f, 0.0f, 5.0f);
        Eigen::Vector3f initialTarget = Eigen::Vector3f::Zero();
        Eigen::Vector3f initialUp = Eigen::Vector3f(0.0f, 1.0f, 0.0f);

        float rotationSpeed = 0.005f;      // Radians per pixel
        float panSpeed = 0.01f;            // Units per pixel
        float zoomSpeed = 0.1f;            // Distance factor per scroll
        float minDistance = 0.1f;          // Minimum camera distance
        float maxDistance = 100.0f;        // Maximum camera distance

        float interpolationSpeed = 10.0f;  // Camera smoothing factor
        bool enableSmoothing = true;       // Enable camera interpolation
    };

    CameraController();
    explicit CameraController(const Config& config);
    ~CameraController() = default;

    /**
     * @brief Initialize the controller
     * @param config Configuration parameters
     */
    void initialize(const Config& config);

    /**
     * @brief Reset camera to initial position
     */
    void reset();

    /**
     * @brief Get the current view matrix
     * @return 4x4 view transformation matrix
     */
    Eigen::Matrix4f getViewMatrix() const;

    /**
     * @brief Update camera state with time interpolation
     * @param deltaTime Time since last update in seconds
     */
    void update(float deltaTime);

    /**
     * @brief Handle mouse button press
     * @param button Mouse button (0=left, 1=right, 2=middle)
     * @param x Mouse X position
     * @param y Mouse Y position
     */
    void onMousePress(int button, double x, double y);

    /**
     * @brief Handle mouse button release
     * @param button Mouse button (0=left, 1=right, 2=middle)
     */
    void onMouseRelease(int button);

    /**
     * @brief Handle mouse movement
     * @param x Mouse X position
     * @param y Mouse Y position
     */
    void onMouseMove(double x, double y);

    /**
     * @brief Handle mouse scroll
     * @param offsetY Scroll amount (positive = zoom in)
     */
    void onMouseScroll(double offsetY);

    /**
     * @brief Handle keyboard input for camera rotation
     * @param direction Direction vector (-1, 0, 1 for each axis)
     */
    void rotateByKeyboard(const Eigen::Vector2f& direction);

    /**
     * @brief Frame all objects in view (fit to scene)
     * @param sceneCenter Center of the scene
     * @param sceneRadius Bounding radius of the scene
     */
    void frameScene(const Eigen::Vector3f& sceneCenter, float sceneRadius);

    /**
     * @brief Get camera position
     */
    const Eigen::Vector3f& getPosition() const { return currentEye_; }

    /**
     * @brief Get look-at target
     */
    const Eigen::Vector3f& getTarget() const { return currentTarget_; }

    /**
     * @brief Get up vector
     */
    const Eigen::Vector3f& getUpVector() const { return currentUp_; }

    /**
     * @brief Get current distance from target
     */
    float getDistance() const { return (currentEye_ - currentTarget_).norm(); }

private:
    /**
     * @brief Compute view matrix from camera parameters
     */
    Eigen::Matrix4f computeViewMatrix(const Eigen::Vector3f& eye,
                                      const Eigen::Vector3f& target,
                                      const Eigen::Vector3f& up) const;

    /**
     * @brief Rotate camera around target (orbit)
     * @param deltaX Horizontal rotation amount
     * @param deltaY Vertical rotation amount
     */
    void orbit(double deltaX, double deltaY);

    /**
     * @brief Pan camera (translate in view plane)
     * @param deltaX Horizontal pan amount
     * @param deltaY Vertical pan amount
     */
    void pan(double deltaX, double deltaY);

    /**
     * @brief Zoom camera (move along view direction)
     * @param delta Zoom amount
     */
    void zoom(double delta);

    /**
     * @brief Convert spherical coordinates to Cartesian
     * @param radius Distance from origin
     * @param theta Azimuth angle (around Y axis)
     * @param phi Elevation angle (from XZ plane)
     * @return Cartesian position
     */
    static Eigen::Vector3f sphericalToCartesian(float radius, float theta, float phi);

    /**
     * @brief Convert Cartesian to spherical coordinates
     * @param position Cartesian position relative to target
     * @param radius Output: distance from origin
     * @param theta Output: azimuth angle
     * @param phi Output: elevation angle
     */
    static void cartesianToSpherical(const Eigen::Vector3f& position,
                                    float& radius, float& theta, float& phi);

    Config config_;

    // Current camera state (smoothed)
    Eigen::Vector3f currentEye_;
    Eigen::Vector3f currentTarget_;
    Eigen::Vector3f currentUp_;

    // Target camera state (from input)
    Eigen::Vector3f targetEye_;
    Eigen::Vector3f targetTarget_;
    Eigen::Vector3f targetUp_;

    // Mouse tracking
    bool isDragging_;
    int activeButton_;  // 0=left (orbit), 1=right (pan)
    double lastMouseX_;
    double lastMouseY_;

    // Spherical coordinates for orbit camera
    float radius_;
    float theta_;   // Azimuth (around Y axis)
    float phi_;     // Elevation (from XZ plane)
};

} // namespace visualizer
} // namespace vi_slam

#endif // VI_SLAM_VISUALIZER_CAMERA_CONTROLLER_HPP
