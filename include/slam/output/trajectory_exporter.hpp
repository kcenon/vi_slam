#ifndef VI_SLAM_SLAM_OUTPUT_TRAJECTORY_EXPORTER_HPP
#define VI_SLAM_SLAM_OUTPUT_TRAJECTORY_EXPORTER_HPP

#include <string>
#include <vector>
#include "common/types.hpp"

namespace vi_slam {
namespace output {

/**
 * @brief Exports SLAM trajectories to standard formats (TUM, KITTI)
 *
 * This class provides functionality to export trajectory data in formats
 * commonly used for SLAM evaluation and comparison.
 */
class TrajectoryExporter {
public:
    /**
     * @brief Export trajectory in TUM format
     *
     * TUM format specification:
     * - Each line: timestamp tx ty tz qx qy qz qw
     * - timestamp: in seconds (double precision)
     * - tx, ty, tz: translation in meters
     * - qx, qy, qz, qw: quaternion (scalar-last convention)
     * - Space-separated values
     *
     * @param filepath Output file path
     * @param poses Vector of 6DoF poses
     * @return true if export succeeded, false otherwise
     */
    static bool exportTUM(const std::string& filepath,
                         const std::vector<Pose6DoF>& poses);

    /**
     * @brief Export trajectory in KITTI format
     *
     * KITTI format specification:
     * - Each line: r11 r12 r13 tx r21 r22 r23 ty r31 r32 r33 tz
     * - 3x4 transformation matrix [R|t] in row-major order
     * - R: 3x3 rotation matrix (from quaternion)
     * - t: 3x1 translation vector
     * - Space-separated values
     *
     * @param filepath Output file path
     * @param poses Vector of 6DoF poses
     * @return true if export succeeded, false otherwise
     */
    static bool exportKITTI(const std::string& filepath,
                           const std::vector<Pose6DoF>& poses);

private:
    /**
     * @brief Convert quaternion to 3x3 rotation matrix
     *
     * @param qw Quaternion scalar component
     * @param qx Quaternion x component
     * @param qy Quaternion y component
     * @param qz Quaternion z component
     * @param R Output 3x3 rotation matrix (row-major)
     */
    static void quaternionToRotationMatrix(double qw, double qx, double qy, double qz,
                                          double R[9]);
};

}  // namespace output
}  // namespace vi_slam

#endif  // VI_SLAM_SLAM_OUTPUT_TRAJECTORY_EXPORTER_HPP
