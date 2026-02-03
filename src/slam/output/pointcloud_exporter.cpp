#include "slam/output/pointcloud_exporter.hpp"
#include <fstream>
#include <cstring>

namespace vi_slam {
namespace output {

bool PointCloudExporter::exportPLY(const std::string& filepath,
                                   const std::vector<MapPoint>& points) {
    std::ofstream file(filepath);
    if (!file.is_open()) {
        return false;
    }

    // Write PLY header
    file << "ply\n";
    file << "format ascii 1.0\n";
    file << "element vertex " << points.size() << "\n";
    file << "property float x\n";
    file << "property float y\n";
    file << "property float z\n";
    file << "property uchar red\n";
    file << "property uchar green\n";
    file << "property uchar blue\n";
    file << "end_header\n";

    // Write vertex data
    for (const auto& point : points) {
        file << point.position.x() << " "
             << point.position.y() << " "
             << point.position.z() << " "
             << static_cast<int>(point.color(0)) << " "
             << static_cast<int>(point.color(1)) << " "
             << static_cast<int>(point.color(2)) << "\n";
    }

    file.close();
    return true;
}

bool PointCloudExporter::exportPLYBinary(const std::string& filepath,
                                         const std::vector<MapPoint>& points) {
    std::ofstream file(filepath, std::ios::binary);
    if (!file.is_open()) {
        return false;
    }

    // Write PLY header (ASCII)
    std::string header = "ply\n";
    header += "format binary_little_endian 1.0\n";
    header += "element vertex " + std::to_string(points.size()) + "\n";
    header += "property float x\n";
    header += "property float y\n";
    header += "property float z\n";
    header += "property uchar red\n";
    header += "property uchar green\n";
    header += "property uchar blue\n";
    header += "end_header\n";

    file.write(header.c_str(), header.size());

    // Write vertex data (binary)
    for (const auto& point : points) {
        // Write position as floats (x, y, z)
        float x = static_cast<float>(point.position.x());
        float y = static_cast<float>(point.position.y());
        float z = static_cast<float>(point.position.z());

        file.write(reinterpret_cast<const char*>(&x), sizeof(float));
        file.write(reinterpret_cast<const char*>(&y), sizeof(float));
        file.write(reinterpret_cast<const char*>(&z), sizeof(float));

        // Write color as unsigned chars (r, g, b)
        uint8_t r = point.color(0);
        uint8_t g = point.color(1);
        uint8_t b = point.color(2);
        file.write(reinterpret_cast<const char*>(&r), sizeof(uint8_t));
        file.write(reinterpret_cast<const char*>(&g), sizeof(uint8_t));
        file.write(reinterpret_cast<const char*>(&b), sizeof(uint8_t));
    }

    file.close();
    return true;
}

}  // namespace output
}  // namespace vi_slam
