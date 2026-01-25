#include "slam/output/pointcloud_exporter.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <cassert>

using namespace vi_slam;
using namespace vi_slam::output;

void testExportPLYAscii() {
    std::cout << "Testing PLY ASCII export..." << std::endl;

    // Create sample point cloud
    std::vector<MapPoint> points(3);

    // Point 1: Origin with red color
    points[0].id = 1;
    points[0].position[0] = 0.0;
    points[0].position[1] = 0.0;
    points[0].position[2] = 0.0;
    points[0].color[0] = 255;
    points[0].color[1] = 0;
    points[0].color[2] = 0;
    points[0].observations = 10;

    // Point 2: Translation along X with green color
    points[1].id = 2;
    points[1].position[0] = 1.0;
    points[1].position[1] = 0.0;
    points[1].position[2] = 0.0;
    points[1].color[0] = 0;
    points[1].color[1] = 255;
    points[1].color[2] = 0;
    points[1].observations = 5;

    // Point 3: 3D position with blue color
    points[2].id = 3;
    points[2].position[0] = 1.0;
    points[2].position[1] = 1.0;
    points[2].position[2] = 1.0;
    points[2].color[0] = 0;
    points[2].color[1] = 0;
    points[2].color[2] = 255;
    points[2].observations = 20;

    const std::string filepath = "/tmp/test_pointcloud_ascii.ply";
    assert(PointCloudExporter::exportPLY(filepath, points));

    // Verify file contents
    std::ifstream file(filepath);
    assert(file.is_open());

    std::string line;

    // Verify header
    assert(std::getline(file, line));
    assert(line == "ply");

    assert(std::getline(file, line));
    assert(line == "format ascii 1.0");

    assert(std::getline(file, line));
    assert(line == "element vertex 3");

    assert(std::getline(file, line));
    assert(line == "property float x");

    assert(std::getline(file, line));
    assert(line == "property float y");

    assert(std::getline(file, line));
    assert(line == "property float z");

    assert(std::getline(file, line));
    assert(line == "property uchar red");

    assert(std::getline(file, line));
    assert(line == "property uchar green");

    assert(std::getline(file, line));
    assert(line == "property uchar blue");

    assert(std::getline(file, line));
    assert(line == "end_header");

    // Verify first point
    assert(std::getline(file, line));
    std::istringstream iss1(line);
    double x1, y1, z1;
    int r1, g1, b1;
    iss1 >> x1 >> y1 >> z1 >> r1 >> g1 >> b1;
    assert(std::fabs(x1) < 1e-6);
    assert(std::fabs(y1) < 1e-6);
    assert(std::fabs(z1) < 1e-6);
    assert(r1 == 255);
    assert(g1 == 0);
    assert(b1 == 0);

    // Verify second point
    assert(std::getline(file, line));
    std::istringstream iss2(line);
    double x2, y2, z2;
    int r2, g2, b2;
    iss2 >> x2 >> y2 >> z2 >> r2 >> g2 >> b2;
    assert(std::fabs(x2 - 1.0) < 1e-6);
    assert(std::fabs(y2) < 1e-6);
    assert(std::fabs(z2) < 1e-6);
    assert(r2 == 0);
    assert(g2 == 255);
    assert(b2 == 0);

    // Verify third point
    assert(std::getline(file, line));
    std::istringstream iss3(line);
    double x3, y3, z3;
    int r3, g3, b3;
    iss3 >> x3 >> y3 >> z3 >> r3 >> g3 >> b3;
    assert(std::fabs(x3 - 1.0) < 1e-6);
    assert(std::fabs(y3 - 1.0) < 1e-6);
    assert(std::fabs(z3 - 1.0) < 1e-6);
    assert(r3 == 0);
    assert(g3 == 0);
    assert(b3 == 255);

    file.close();
    std::remove(filepath.c_str());

    std::cout << "  PLY ASCII export test passed!" << std::endl;
}

void testExportPLYBinary() {
    std::cout << "Testing PLY binary export..." << std::endl;

    // Create sample point cloud
    std::vector<MapPoint> points(2);

    // Point 1: Origin
    points[0].id = 1;
    points[0].position[0] = 0.0;
    points[0].position[1] = 0.0;
    points[0].position[2] = 0.0;
    points[0].color[0] = 128;
    points[0].color[1] = 128;
    points[0].color[2] = 128;

    // Point 2: Translation
    points[1].id = 2;
    points[1].position[0] = 2.5;
    points[1].position[1] = 3.7;
    points[1].position[2] = -1.2;
    points[1].color[0] = 200;
    points[1].color[1] = 150;
    points[1].color[2] = 100;

    const std::string filepath = "/tmp/test_pointcloud_binary.ply";
    assert(PointCloudExporter::exportPLYBinary(filepath, points));

    // Verify file exists and has correct size
    std::ifstream file(filepath, std::ios::binary);
    assert(file.is_open());

    // Read header (ASCII part)
    std::string line;
    assert(std::getline(file, line));
    assert(line == "ply");

    assert(std::getline(file, line));
    assert(line == "format binary_little_endian 1.0");

    assert(std::getline(file, line));
    assert(line == "element vertex 2");

    // Skip remaining header lines
    while (std::getline(file, line)) {
        if (line == "end_header") {
            break;
        }
    }

    // Read binary data
    float x, y, z;
    uint8_t r, g, b;

    // First point
    file.read(reinterpret_cast<char*>(&x), sizeof(float));
    file.read(reinterpret_cast<char*>(&y), sizeof(float));
    file.read(reinterpret_cast<char*>(&z), sizeof(float));
    file.read(reinterpret_cast<char*>(&r), sizeof(uint8_t));
    file.read(reinterpret_cast<char*>(&g), sizeof(uint8_t));
    file.read(reinterpret_cast<char*>(&b), sizeof(uint8_t));

    assert(std::fabs(x) < 1e-6);
    assert(std::fabs(y) < 1e-6);
    assert(std::fabs(z) < 1e-6);
    assert(r == 128);
    assert(g == 128);
    assert(b == 128);

    // Second point
    file.read(reinterpret_cast<char*>(&x), sizeof(float));
    file.read(reinterpret_cast<char*>(&y), sizeof(float));
    file.read(reinterpret_cast<char*>(&z), sizeof(float));
    file.read(reinterpret_cast<char*>(&r), sizeof(uint8_t));
    file.read(reinterpret_cast<char*>(&g), sizeof(uint8_t));
    file.read(reinterpret_cast<char*>(&b), sizeof(uint8_t));

    assert(std::fabs(x - 2.5f) < 1e-6);
    assert(std::fabs(y - 3.7f) < 1e-6);
    assert(std::fabs(z - (-1.2f)) < 1e-6);
    assert(r == 200);
    assert(g == 150);
    assert(b == 100);

    file.close();
    std::remove(filepath.c_str());

    std::cout << "  PLY binary export test passed!" << std::endl;
}

void testInvalidFilepath() {
    std::cout << "Testing invalid filepath..." << std::endl;

    std::vector<MapPoint> points(1);
    points[0].position[0] = 0.0;

    assert(!PointCloudExporter::exportPLY("/invalid/path/test.ply", points));
    assert(!PointCloudExporter::exportPLYBinary("/invalid/path/test.ply", points));

    std::cout << "  Invalid filepath test passed!" << std::endl;
}

void testEmptyPointCloud() {
    std::cout << "Testing empty point cloud..." << std::endl;

    std::vector<MapPoint> emptyPoints;
    const std::string filepath = "/tmp/test_empty.ply";

    assert(PointCloudExporter::exportPLY(filepath, emptyPoints));

    // Verify file contents
    std::ifstream file(filepath);
    assert(file.is_open());

    std::string line;
    assert(std::getline(file, line));
    assert(line == "ply");

    assert(std::getline(file, line));
    assert(line == "format ascii 1.0");

    assert(std::getline(file, line));
    assert(line == "element vertex 0");

    file.close();
    std::remove(filepath.c_str());

    std::cout << "  Empty point cloud test passed!" << std::endl;
}

void testLargePointCloud() {
    std::cout << "Testing large point cloud..." << std::endl;

    const size_t numPoints = 10000;
    std::vector<MapPoint> points(numPoints);

    for (size_t i = 0; i < numPoints; ++i) {
        points[i].id = static_cast<int64_t>(i);
        points[i].position[0] = static_cast<double>(i) * 0.1;
        points[i].position[1] = static_cast<double>(i) * 0.2;
        points[i].position[2] = static_cast<double>(i) * 0.3;
        points[i].color[0] = static_cast<uint8_t>(i % 256);
        points[i].color[1] = static_cast<uint8_t>((i * 2) % 256);
        points[i].color[2] = static_cast<uint8_t>((i * 3) % 256);
    }

    // Test ASCII format
    const std::string asciiPath = "/tmp/test_large_ascii.ply";
    assert(PointCloudExporter::exportPLY(asciiPath, points));

    std::ifstream asciiFile(asciiPath);
    assert(asciiFile.is_open());

    // Verify header contains correct vertex count
    std::string line;
    bool foundVertexCount = false;
    while (std::getline(asciiFile, line)) {
        if (line.find("element vertex") != std::string::npos) {
            assert(line == "element vertex 10000");
            foundVertexCount = true;
            break;
        }
    }
    (void)foundVertexCount;  // Suppress unused variable warning in release builds
    assert(foundVertexCount);

    asciiFile.close();
    std::remove(asciiPath.c_str());

    // Test binary format
    const std::string binaryPath = "/tmp/test_large_binary.ply";
    assert(PointCloudExporter::exportPLYBinary(binaryPath, points));

    std::ifstream binaryFile(binaryPath);
    assert(binaryFile.is_open());
    binaryFile.close();

    std::remove(binaryPath.c_str());

    std::cout << "  Large point cloud test passed!" << std::endl;
}

int main() {
    std::cout << "Running point cloud exporter tests..." << std::endl;
    std::cout << std::endl;

    try {
        testExportPLYAscii();
        testExportPLYBinary();
        testInvalidFilepath();
        testEmptyPointCloud();
        testLargePointCloud();

        std::cout << std::endl;
        std::cout << "All tests passed!" << std::endl;
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Test failed with exception: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "Test failed with unknown exception" << std::endl;
        return 1;
    }
}
