#ifndef VI_SLAM_COMMON_TYPES_HPP
#define VI_SLAM_COMMON_TYPES_HPP

#include <cstdint>
#include <vector>

namespace vi_slam {

// 6DoF pose representation
struct Pose6DoF {
    int64_t timestampNs;      // Timestamp in nanoseconds
    double position[3];       // x, y, z in meters
    double orientation[4];    // qw, qx, qy, qz (quaternion)
    double covariance[36];    // 6x6 covariance matrix (optional)
    bool valid;               // Pose validity flag

    Pose6DoF() : timestampNs(0), valid(false) {
        position[0] = position[1] = position[2] = 0.0;
        orientation[0] = 1.0;  // qw = 1 for identity rotation
        orientation[1] = orientation[2] = orientation[3] = 0.0;
        for (int i = 0; i < 36; ++i) {
            covariance[i] = 0.0;
        }
    }
};

// IMU sample data
struct IMUSample {
    int64_t timestampNs;      // Timestamp in nanoseconds
    double accX;              // Acceleration X in m/s^2
    double accY;              // Acceleration Y in m/s^2
    double accZ;              // Acceleration Z in m/s^2
    double gyroX;             // Angular velocity X in rad/s
    double gyroY;             // Angular velocity Y in rad/s
    double gyroZ;             // Angular velocity Z in rad/s

    IMUSample() : timestampNs(0), accX(0), accY(0), accZ(0),
                  gyroX(0), gyroY(0), gyroZ(0) {}
};

// Map point in 3D space
struct MapPoint {
    int64_t id;               // Unique identifier
    double position[3];       // x, y, z coordinates
    uint8_t color[3];         // RGB color
    int observations;         // Number of times observed

    MapPoint() : id(0), observations(0) {
        position[0] = position[1] = position[2] = 0.0;
        color[0] = color[1] = color[2] = 128;
    }
};

// Tracking status enumeration
enum class TrackingStatus {
    UNINITIALIZED,
    INITIALIZING,
    TRACKING,
    LOST,
    RELOCALIZATION
};

}  // namespace vi_slam

#endif  // VI_SLAM_COMMON_TYPES_HPP
