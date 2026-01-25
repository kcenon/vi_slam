#ifndef VI_SLAM_SLAM_OUTPUT_POINTCLOUD_EXPORTER_HPP
#define VI_SLAM_SLAM_OUTPUT_POINTCLOUD_EXPORTER_HPP

#include <string>
#include <vector>
#include "common/types.hpp"

namespace vi_slam {
namespace output {

/**
 * @brief Exports 3D point clouds to PLY format
 *
 * This class provides functionality to export point cloud data in PLY format,
 * supporting both ASCII and binary (little-endian) variants.
 * PLY format is widely supported by 3D visualization tools like MeshLab and CloudCompare.
 */
class PointCloudExporter {
public:
    /**
     * @brief Export point cloud in PLY ASCII format
     *
     * PLY ASCII format specification:
     * - Header section:
     *   ply
     *   format ascii 1.0
     *   element vertex N
     *   property float x
     *   property float y
     *   property float z
     *   property uchar red
     *   property uchar green
     *   property uchar blue
     *   end_header
     * - Data section:
     *   x y z r g b (space-separated values, one point per line)
     *
     * @param filepath Output file path
     * @param points Vector of 3D map points
     * @return true if export succeeded, false otherwise
     */
    static bool exportPLY(const std::string& filepath,
                         const std::vector<MapPoint>& points);

    /**
     * @brief Export point cloud in PLY binary format (little-endian)
     *
     * PLY binary format specification:
     * - Header section (ASCII):
     *   ply
     *   format binary_little_endian 1.0
     *   element vertex N
     *   property float x
     *   property float y
     *   property float z
     *   property uchar red
     *   property uchar green
     *   property uchar blue
     *   end_header
     * - Data section (binary):
     *   [x (float)][y (float)][z (float)][r (uchar)][g (uchar)][b (uchar)] (repeated N times)
     *
     * Binary format is more compact and efficient for large point clouds.
     *
     * @param filepath Output file path
     * @param points Vector of 3D map points
     * @return true if export succeeded, false otherwise
     */
    static bool exportPLYBinary(const std::string& filepath,
                               const std::vector<MapPoint>& points);
};

}  // namespace output
}  // namespace vi_slam

#endif  // VI_SLAM_SLAM_OUTPUT_POINTCLOUD_EXPORTER_HPP
