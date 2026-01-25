#include "slam/output/trajectory_exporter.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <cassert>

using namespace vi_slam;
using namespace vi_slam::output;

void testExportTUM() {
    std::cout << "Testing TUM export..." << std::endl;

    // Create sample trajectory
    std::vector<Pose6DoF> poses(3);

    // Pose 1: Identity at origin
    poses[0].timestampNs = 1000000000;  // 1.0 seconds
    poses[0].position[0] = 0.0;
    poses[0].position[1] = 0.0;
    poses[0].position[2] = 0.0;
    poses[0].orientation[0] = 1.0;  // qw
    poses[0].orientation[1] = 0.0;  // qx
    poses[0].orientation[2] = 0.0;  // qy
    poses[0].orientation[3] = 0.0;  // qz
    poses[0].valid = true;

    // Pose 2: Translation along X
    poses[1].timestampNs = 2000000000;  // 2.0 seconds
    poses[1].position[0] = 1.0;
    poses[1].position[1] = 0.0;
    poses[1].position[2] = 0.0;
    poses[1].orientation[0] = 1.0;  // qw
    poses[1].orientation[1] = 0.0;  // qx
    poses[1].orientation[2] = 0.0;  // qy
    poses[1].orientation[3] = 0.0;  // qz
    poses[1].valid = true;

    // Pose 3: 90 degree rotation around Z
    poses[2].timestampNs = 3000000000;  // 3.0 seconds
    poses[2].position[0] = 1.0;
    poses[2].position[1] = 1.0;
    poses[2].position[2] = 0.0;
    poses[2].orientation[0] = std::cos(M_PI / 4.0);  // qw
    poses[2].orientation[1] = 0.0;                    // qx
    poses[2].orientation[2] = 0.0;                    // qy
    poses[2].orientation[3] = std::sin(M_PI / 4.0);  // qz
    poses[2].valid = true;

    const std::string filepath = "/tmp/test_tum.txt";
    assert(TrajectoryExporter::exportTUM(filepath, poses));

    // Verify file contents
    std::ifstream file(filepath);
    assert(file.is_open());

    std::string line;

    // First pose
    assert(std::getline(file, line));
    std::istringstream iss1(line);
    double t1, x1, y1, z1, qx1, qy1, qz1, qw1;
    iss1 >> t1 >> x1 >> y1 >> z1 >> qx1 >> qy1 >> qz1 >> qw1;
    assert(std::fabs(t1 - 1.0) < 1e-6);
    assert(std::fabs(x1) < 1e-6);
    assert(std::fabs(qw1 - 1.0) < 1e-6);

    // Second pose
    assert(std::getline(file, line));
    std::istringstream iss2(line);
    double t2, x2, y2, z2, qx2, qy2, qz2, qw2;
    iss2 >> t2 >> x2 >> y2 >> z2 >> qx2 >> qy2 >> qz2 >> qw2;
    assert(std::fabs(t2 - 2.0) < 1e-6);
    assert(std::fabs(x2 - 1.0) < 1e-6);

    file.close();
    std::remove(filepath.c_str());

    std::cout << "  TUM export test passed!" << std::endl;
}

void testExportKITTI() {
    std::cout << "Testing KITTI export..." << std::endl;

    // Create sample trajectory
    std::vector<Pose6DoF> poses(2);

    // Pose 1: Identity at origin
    poses[0].timestampNs = 1000000000;
    poses[0].position[0] = 0.0;
    poses[0].position[1] = 0.0;
    poses[0].position[2] = 0.0;
    poses[0].orientation[0] = 1.0;
    poses[0].orientation[1] = 0.0;
    poses[0].orientation[2] = 0.0;
    poses[0].orientation[3] = 0.0;
    poses[0].valid = true;

    // Pose 2: Translation
    poses[1].timestampNs = 2000000000;
    poses[1].position[0] = 1.0;
    poses[1].position[1] = 2.0;
    poses[1].position[2] = 3.0;
    poses[1].orientation[0] = 1.0;
    poses[1].orientation[1] = 0.0;
    poses[1].orientation[2] = 0.0;
    poses[1].orientation[3] = 0.0;
    poses[1].valid = true;

    const std::string filepath = "/tmp/test_kitti.txt";
    assert(TrajectoryExporter::exportKITTI(filepath, poses));

    // Verify file contents
    std::ifstream file(filepath);
    assert(file.is_open());

    std::string line;

    // First pose: identity
    assert(std::getline(file, line));
    std::istringstream iss1(line);
    double r11, r12, r13, tx1, r21, r22, r23, ty1, r31, r32, r33, tz1;
    iss1 >> r11 >> r12 >> r13 >> tx1 >> r21 >> r22 >> r23 >> ty1 >> r31 >> r32 >> r33 >> tz1;
    assert(std::fabs(r11 - 1.0) < 1e-6);
    assert(std::fabs(r22 - 1.0) < 1e-6);
    assert(std::fabs(r33 - 1.0) < 1e-6);
    assert(std::fabs(tx1) < 1e-6);

    // Second pose: translation
    assert(std::getline(file, line));
    std::istringstream iss2(line);
    double tx2, ty2, tz2;
    double r11_2, r12_2, r13_2, r21_2, r22_2, r23_2, r31_2, r32_2, r33_2;
    iss2 >> r11_2 >> r12_2 >> r13_2 >> tx2 >> r21_2 >> r22_2 >> r23_2 >> ty2
         >> r31_2 >> r32_2 >> r33_2 >> tz2;
    assert(std::fabs(tx2 - 1.0) < 1e-6);
    assert(std::fabs(ty2 - 2.0) < 1e-6);
    assert(std::fabs(tz2 - 3.0) < 1e-6);

    file.close();
    std::remove(filepath.c_str());

    std::cout << "  KITTI export test passed!" << std::endl;
}

void testInvalidFilepath() {
    std::cout << "Testing invalid filepath..." << std::endl;

    std::vector<Pose6DoF> poses(1);
    poses[0].valid = true;

    assert(!TrajectoryExporter::exportTUM("/invalid/path/test.txt", poses));
    assert(!TrajectoryExporter::exportKITTI("/invalid/path/test.txt", poses));

    std::cout << "  Invalid filepath test passed!" << std::endl;
}

void testEmptyTrajectory() {
    std::cout << "Testing empty trajectory..." << std::endl;

    std::vector<Pose6DoF> emptyPoses;
    const std::string filepath = "/tmp/test_empty.txt";

    assert(TrajectoryExporter::exportTUM(filepath, emptyPoses));

    // Verify empty file
    std::ifstream file(filepath);
    assert(file.is_open());

    std::string line;
    assert(!std::getline(file, line));

    file.close();
    std::remove(filepath.c_str());

    std::cout << "  Empty trajectory test passed!" << std::endl;
}

void testSkipInvalidPoses() {
    std::cout << "Testing skip invalid poses..." << std::endl;

    std::vector<Pose6DoF> poses(2);

    // Valid pose
    poses[0].timestampNs = 1000000000;
    poses[0].position[0] = 1.0;
    poses[0].valid = true;

    // Invalid pose
    poses[1].timestampNs = 2000000000;
    poses[1].position[0] = 2.0;
    poses[1].valid = false;

    const std::string filepath = "/tmp/test_skip.txt";
    assert(TrajectoryExporter::exportTUM(filepath, poses));

    // Verify only 1 pose exported
    std::ifstream file(filepath);
    assert(file.is_open());

    std::string line;
    int lineCount = 0;
    while (std::getline(file, line)) {
        lineCount++;
    }

    (void)lineCount;  // Suppress unused variable warning in release builds
    assert(lineCount == 1);

    file.close();
    std::remove(filepath.c_str());

    std::cout << "  Skip invalid poses test passed!" << std::endl;
}

int main() {
    std::cout << "Running trajectory exporter tests..." << std::endl;
    std::cout << std::endl;

    try {
        testExportTUM();
        testExportKITTI();
        testInvalidFilepath();
        testEmptyTrajectory();
        testSkipInvalidPoses();

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
