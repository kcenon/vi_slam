#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>

int main(int /* argc */, char** /* argv */) {
    std::cout << "VI-SLAM PC Client" << std::endl;
    std::cout << "OpenCV version: " << CV_VERSION << std::endl;
    std::cout << "Eigen version: " << EIGEN_WORLD_VERSION << "."
              << EIGEN_MAJOR_VERSION << "." << EIGEN_MINOR_VERSION << std::endl;

#ifdef HAVE_CERES
    std::cout << "Ceres Solver: Available" << std::endl;
#else
    std::cout << "Ceres Solver: Not available" << std::endl;
#endif

#ifdef HAVE_ZMQ
    std::cout << "ZeroMQ: Available" << std::endl;
#else
    std::cout << "ZeroMQ: Not available" << std::endl;
#endif

    std::cout << "\nReady to receive sensor data from Android device." << std::endl;

    return 0;
}
