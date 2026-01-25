#include "slam/output/trajectory_exporter.hpp"
#include <fstream>
#include <iomanip>
#include <cmath>

namespace vi_slam {
namespace output {

bool TrajectoryExporter::exportTUM(const std::string& filepath,
                                   const std::vector<Pose6DoF>& poses) {
    std::ofstream file(filepath);
    if (!file.is_open()) {
        return false;
    }

    // Set precision for floating point numbers
    file << std::fixed << std::setprecision(9);

    for (const auto& pose : poses) {
        if (!pose.valid) {
            continue;  // Skip invalid poses
        }

        // Convert nanoseconds to seconds
        double timestamp = pose.timestampNs / 1e9;

        // TUM format: timestamp tx ty tz qx qy qz qw
        // Note: Our quaternion is [qw, qx, qy, qz], TUM expects [qx, qy, qz, qw]
        file << timestamp << " "
             << pose.position[0] << " "
             << pose.position[1] << " "
             << pose.position[2] << " "
             << pose.orientation[1] << " "  // qx
             << pose.orientation[2] << " "  // qy
             << pose.orientation[3] << " "  // qz
             << pose.orientation[0]         // qw
             << "\n";
    }

    file.close();
    return true;
}

bool TrajectoryExporter::exportKITTI(const std::string& filepath,
                                     const std::vector<Pose6DoF>& poses) {
    std::ofstream file(filepath);
    if (!file.is_open()) {
        return false;
    }

    // Set precision for floating point numbers
    file << std::fixed << std::setprecision(9);

    for (const auto& pose : poses) {
        if (!pose.valid) {
            continue;  // Skip invalid poses
        }

        // Convert quaternion to rotation matrix
        double R[9];
        quaternionToRotationMatrix(pose.orientation[0],  // qw
                                  pose.orientation[1],  // qx
                                  pose.orientation[2],  // qy
                                  pose.orientation[3],  // qz
                                  R);

        // KITTI format: r11 r12 r13 tx r21 r22 r23 ty r31 r32 r33 tz
        // 3x4 transformation matrix [R|t] in row-major order
        file << R[0] << " " << R[1] << " " << R[2] << " " << pose.position[0] << " "
             << R[3] << " " << R[4] << " " << R[5] << " " << pose.position[1] << " "
             << R[6] << " " << R[7] << " " << R[8] << " " << pose.position[2]
             << "\n";
    }

    file.close();
    return true;
}

void TrajectoryExporter::quaternionToRotationMatrix(double qw, double qx, double qy, double qz,
                                                    double R[9]) {
    // Normalize quaternion
    double norm = std::sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
    if (norm < 1e-9) {
        // Identity matrix for invalid quaternion
        R[0] = 1.0; R[1] = 0.0; R[2] = 0.0;
        R[3] = 0.0; R[4] = 1.0; R[5] = 0.0;
        R[6] = 0.0; R[7] = 0.0; R[8] = 1.0;
        return;
    }

    qw /= norm;
    qx /= norm;
    qy /= norm;
    qz /= norm;

    // Compute rotation matrix from quaternion
    // Formula: https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/
    double qx2 = qx * qx;
    double qy2 = qy * qy;
    double qz2 = qz * qz;

    // Row 0
    R[0] = 1.0 - 2.0 * (qy2 + qz2);
    R[1] = 2.0 * (qx*qy - qw*qz);
    R[2] = 2.0 * (qx*qz + qw*qy);

    // Row 1
    R[3] = 2.0 * (qx*qy + qw*qz);
    R[4] = 1.0 - 2.0 * (qx2 + qz2);
    R[5] = 2.0 * (qy*qz - qw*qx);

    // Row 2
    R[6] = 2.0 * (qx*qz - qw*qy);
    R[7] = 2.0 * (qy*qz + qw*qx);
    R[8] = 1.0 - 2.0 * (qx2 + qy2);
}

}  // namespace output
}  // namespace vi_slam
