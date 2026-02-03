#ifndef VI_SLAM_COMMON_TYPES_HPP
#define VI_SLAM_COMMON_TYPES_HPP

/**
 * @file types.hpp
 * @brief Common data types for VI-SLAM system
 *
 * This header defines the fundamental data structures used throughout
 * the VI-SLAM system, including pose representations, IMU samples,
 * map points, and tracking status.
 */

#include <cstdint>
#include <vector>

namespace vi_slam {

/**
 * @brief 6 Degrees of Freedom pose representation
 *
 * Represents the position and orientation of the camera/robot in 3D space.
 * The orientation is stored as a quaternion (qw, qx, qy, qz) where qw is
 * the scalar component.
 */
struct Pose6DoF {
    int64_t timestampNs;      ///< Timestamp in nanoseconds since epoch
    double position[3];       ///< Position (x, y, z) in meters
    double orientation[4];    ///< Orientation quaternion (qw, qx, qy, qz)
    double covariance[36];    ///< 6x6 covariance matrix (row-major)
    bool valid;               ///< Pose validity flag

    Pose6DoF() : timestampNs(0), valid(false) {
        position[0] = position[1] = position[2] = 0.0;
        orientation[0] = 1.0;  // qw = 1 for identity rotation
        orientation[1] = orientation[2] = orientation[3] = 0.0;
        for (int i = 0; i < 36; ++i) {
            covariance[i] = 0.0;
        }
    }
};

/**
 * @brief IMU (Inertial Measurement Unit) sample data
 *
 * Contains accelerometer and gyroscope measurements from an IMU sensor.
 * All measurements are in SI units: m/s^2 for acceleration and rad/s for
 * angular velocity.
 */
struct IMUSample {
    int64_t timestampNs;      ///< Timestamp in nanoseconds since epoch
    double accX;              ///< Acceleration X-axis in m/s^2
    double accY;              ///< Acceleration Y-axis in m/s^2
    double accZ;              ///< Acceleration Z-axis in m/s^2
    double gyroX;             ///< Angular velocity X-axis in rad/s
    double gyroY;             ///< Angular velocity Y-axis in rad/s
    double gyroZ;             ///< Angular velocity Z-axis in rad/s

    IMUSample() : timestampNs(0), accX(0), accY(0), accZ(0),
                  gyroX(0), gyroY(0), gyroZ(0) {}
};

/**
 * @brief 3D map point with color and observation count
 *
 * Represents a landmark in the SLAM map with its 3D position,
 * color information, and the number of times it has been observed.
 */
struct MapPoint {
    int64_t id;               ///< Unique point identifier
    double position[3];       ///< 3D position (x, y, z) in meters
    uint8_t color[3];         ///< RGB color values (0-255)
    int observations;         ///< Number of times this point was observed

    MapPoint() : id(0), observations(0) {
        position[0] = position[1] = position[2] = 0.0;
        color[0] = color[1] = color[2] = 128;
    }
};

/**
 * @brief SLAM tracking status enumeration
 *
 * Indicates the current state of the visual-inertial tracking system.
 */
enum class TrackingStatus {
    UNINITIALIZED,    ///< System not yet initialized
    INITIALIZING,     ///< Initialization in progress
    TRACKING,         ///< Normal tracking operation
    LOST,             ///< Tracking lost, cannot localize
    RELOCALIZATION    ///< Attempting to recover from lost state
};

}  // namespace vi_slam

#endif  // VI_SLAM_COMMON_TYPES_HPP
