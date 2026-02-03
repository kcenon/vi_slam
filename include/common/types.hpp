#ifndef VI_SLAM_COMMON_TYPES_HPP
#define VI_SLAM_COMMON_TYPES_HPP

/**
 * @file types.hpp
 * @brief Common data types for VI-SLAM system
 *
 * This header defines the fundamental data structures used throughout
 * the VI-SLAM system, including pose representations, IMU samples,
 * map points, and tracking status.
 *
 * All geometric types use Eigen for type safety, SIMD optimization,
 * and direct interoperability with SLAM frameworks.
 */

#include <cstdint>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

namespace vi_slam {

/**
 * @brief 6 Degrees of Freedom pose representation
 *
 * Represents the position and orientation of the camera/robot in 3D space.
 * Uses Eigen types for type safety and optimal performance.
 *
 * The orientation is stored as a quaternion using Eigen::Quaterniond
 * (internally stored as [x, y, z, w] but constructed as (w, x, y, z)).
 */
struct Pose6DoF {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    int64_t timestampNs;                      ///< Timestamp in nanoseconds since epoch
    Eigen::Vector3d position;                 ///< Position (x, y, z) in meters
    Eigen::Quaterniond orientation;           ///< Orientation quaternion
    Eigen::Matrix<double, 6, 6> covariance;   ///< 6x6 covariance matrix
    bool valid;                               ///< Pose validity flag

    Pose6DoF()
        : timestampNs(0),
          position(Eigen::Vector3d::Zero()),
          orientation(Eigen::Quaterniond::Identity()),
          covariance(Eigen::Matrix<double, 6, 6>::Zero()),
          valid(false) {}

    /**
     * @brief Get position as C-style array for backward compatibility
     * @param out Output array of size 3
     */
    void getPositionArray(double* out) const {
        out[0] = position.x();
        out[1] = position.y();
        out[2] = position.z();
    }

    /**
     * @brief Get orientation as C-style array (qw, qx, qy, qz) for backward compatibility
     * @param out Output array of size 4
     */
    void getOrientationArray(double* out) const {
        out[0] = orientation.w();
        out[1] = orientation.x();
        out[2] = orientation.y();
        out[3] = orientation.z();
    }

    /**
     * @brief Get covariance as C-style array (row-major) for backward compatibility
     * @param out Output array of size 36
     */
    void getCovarianceArray(double* out) const {
        Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> outMap(out);
        outMap = covariance;
    }

    /**
     * @brief Set position from C-style array
     * @param in Input array of size 3
     */
    void setPositionFromArray(const double* in) {
        position = Eigen::Vector3d(in[0], in[1], in[2]);
    }

    /**
     * @brief Set orientation from C-style array (qw, qx, qy, qz)
     * @param in Input array of size 4
     */
    void setOrientationFromArray(const double* in) {
        orientation = Eigen::Quaterniond(in[0], in[1], in[2], in[3]);
    }

    /**
     * @brief Set covariance from C-style array (row-major)
     * @param in Input array of size 36
     */
    void setCovarianceFromArray(const double* in) {
        covariance = Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(in);
    }
};

/**
 * @brief IMU (Inertial Measurement Unit) sample data
 *
 * Contains accelerometer and gyroscope measurements from an IMU sensor.
 * Uses Eigen types for vector operations.
 * All measurements are in SI units: m/s^2 for acceleration and rad/s for
 * angular velocity.
 */
struct IMUSample {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    int64_t timestampNs;          ///< Timestamp in nanoseconds since epoch
    Eigen::Vector3d acceleration; ///< Acceleration (x, y, z) in m/s^2
    Eigen::Vector3d angularVelocity; ///< Angular velocity (x, y, z) in rad/s

    IMUSample()
        : timestampNs(0),
          acceleration(Eigen::Vector3d::Zero()),
          angularVelocity(Eigen::Vector3d::Zero()) {}

    // Convenience accessors for individual components (backward compatibility)
    double accX() const { return acceleration.x(); }
    double accY() const { return acceleration.y(); }
    double accZ() const { return acceleration.z(); }
    double gyroX() const { return angularVelocity.x(); }
    double gyroY() const { return angularVelocity.y(); }
    double gyroZ() const { return angularVelocity.z(); }

    void setAccX(double v) { acceleration.x() = v; }
    void setAccY(double v) { acceleration.y() = v; }
    void setAccZ(double v) { acceleration.z() = v; }
    void setGyroX(double v) { angularVelocity.x() = v; }
    void setGyroY(double v) { angularVelocity.y() = v; }
    void setGyroZ(double v) { angularVelocity.z() = v; }
};

/**
 * @brief 3D map point with color and observation count
 *
 * Represents a landmark in the SLAM map with its 3D position,
 * color information, and the number of times it has been observed.
 */
struct MapPoint {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    int64_t id;                               ///< Unique point identifier
    Eigen::Vector3d position;                 ///< 3D position (x, y, z) in meters
    Eigen::Matrix<uint8_t, 3, 1> color;       ///< RGB color values (0-255)
    int observations;                         ///< Number of times this point was observed

    MapPoint()
        : id(0),
          position(Eigen::Vector3d::Zero()),
          color(Eigen::Matrix<uint8_t, 3, 1>::Constant(128)),
          observations(0) {}

    /**
     * @brief Get position as C-style array for backward compatibility
     * @param out Output array of size 3
     */
    void getPositionArray(double* out) const {
        out[0] = position.x();
        out[1] = position.y();
        out[2] = position.z();
    }

    /**
     * @brief Get color as C-style array for backward compatibility
     * @param out Output array of size 3
     */
    void getColorArray(uint8_t* out) const {
        out[0] = color(0);
        out[1] = color(1);
        out[2] = color(2);
    }

    /**
     * @brief Set position from C-style array
     * @param in Input array of size 3
     */
    void setPositionFromArray(const double* in) {
        position = Eigen::Vector3d(in[0], in[1], in[2]);
    }

    /**
     * @brief Set color from C-style array
     * @param in Input array of size 3
     */
    void setColorFromArray(const uint8_t* in) {
        color(0) = in[0];
        color(1) = in[1];
        color(2) = in[2];
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

// Enable STL container support for Eigen-aligned types
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(vi_slam::Pose6DoF)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(vi_slam::IMUSample)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(vi_slam::MapPoint)

#endif  // VI_SLAM_COMMON_TYPES_HPP
