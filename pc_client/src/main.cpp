#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include "webrtc_receiver.hpp"

int main(int argc, char** argv) {
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

    std::cout << "\nInitializing WebRTC receiver..." << std::endl;

    // Create WebRTC receiver
    vi_slam::WebRTCReceiver receiver;

    // Initialize
    if (!receiver.initialize()) {
        std::cerr << "Failed to initialize WebRTC receiver" << std::endl;
        return 1;
    }

    // Set up callbacks
    receiver.setVideoCallback([](const cv::Mat& frame, int64_t timestamp) {
        static int frameCount = 0;
        if (++frameCount % 30 == 0) {  // Print every 30 frames
            std::cout << "Received frame: " << frame.cols << "x" << frame.rows
                      << " at " << timestamp << std::endl;
        }
    });

    receiver.setIMUCallback([](const vi_slam::IMUSample& imu) {
        static int imuCount = 0;
        if (++imuCount % 100 == 0) {  // Print every 100 samples
            std::cout << "Received IMU: acc=(" << imu.accX << ", " << imu.accY
                      << ", " << imu.accZ << ") gyro=(" << imu.gyroX << ", "
                      << imu.gyroY << ", " << imu.gyroZ << ")" << std::endl;
        }
    });

    // Default signaling URL or from command line
    std::string signalingUrl = "ws://localhost:8080";
    if (argc > 1) {
        signalingUrl = argv[1];
    }

    // Connect
    std::cout << "Connecting to signaling server: " << signalingUrl << std::endl;
    if (!receiver.connect(signalingUrl)) {
        std::cerr << "Failed to connect" << std::endl;
        return 1;
    }

    std::cout << "Connected! Press Enter to stop..." << std::endl;
    std::cin.get();

    // Cleanup
    std::cout << "Disconnecting..." << std::endl;
    receiver.disconnect();

    std::cout << "Done." << std::endl;
    return 0;
}
