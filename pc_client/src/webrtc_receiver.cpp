#include "webrtc_receiver.hpp"
#include <chrono>
#include <iostream>
#include <vector>

namespace vi_slam {

// Implementation class (Pimpl pattern)
class WebRTCReceiver::Impl {
public:
    Impl() : connected_(false), frameCount_(0) {}

    ~Impl() {
        disconnect();
    }

    bool initialize() {
        std::lock_guard<std::mutex> lock(mutex_);

        // TODO: Initialize libwebrtc
        // For now, this is a placeholder implementation

        std::cout << "[WebRTCReceiver] Initialized" << std::endl;
        return true;
    }

    bool connect(const std::string& signalingUrl) {
        std::lock_guard<std::mutex> lock(mutex_);

        if (connected_) {
            std::cerr << "[WebRTCReceiver] Already connected" << std::endl;
            return false;
        }

        // TODO: Actual WebRTC connection implementation
        // This would include:
        // 1. Connect to signaling server
        // 2. Exchange SDP offers/answers
        // 3. Establish ICE connection
        // 4. Set up video and data channels

        std::cout << "[WebRTCReceiver] Connecting to: " << signalingUrl << std::endl;

        // Simulate connection success
        connected_ = true;
        signalingUrl_ = signalingUrl;

        // Start receive thread
        receiveThread_ = std::thread(&Impl::receiveLoop, this);

        return true;
    }

    void disconnect() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!connected_) {
                return;
            }
            connected_ = false;
        }

        // Wait for receive thread to finish
        if (receiveThread_.joinable()) {
            receiveThread_.join();
        }

        std::cout << "[WebRTCReceiver] Disconnected" << std::endl;
    }

    bool isConnected() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return connected_;
    }

    void setVideoCallback(VideoCallback callback) {
        std::lock_guard<std::mutex> lock(mutex_);
        videoCallback_ = callback;
    }

    void setIMUCallback(IMUCallback callback) {
        std::lock_guard<std::mutex> lock(mutex_);
        imuCallback_ = callback;
    }

    double getFrameRate() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return frameRate_;
    }

private:
    void receiveLoop() {
        std::cout << "[WebRTCReceiver] Receive loop started" << std::endl;

        auto lastUpdate = std::chrono::steady_clock::now();
        int localFrameCount = 0;

        while (isConnected()) {
            // TODO: Actual WebRTC frame reception
            // This would include:
            // 1. Receive encoded H.264 frame from WebRTC
            // 2. Decode using hardware/software decoder
            // 3. Convert to cv::Mat
            // 4. Call video callback

            // Simulate frame processing
            std::this_thread::sleep_for(std::chrono::milliseconds(33));  // ~30 FPS

            // Create dummy frame for testing
            cv::Mat frame(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
            int64_t timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();

            // Call video callback if set
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (videoCallback_) {
                    videoCallback_(frame, timestamp);
                }
            }

            // Update frame rate
            localFrameCount++;
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                now - lastUpdate).count();

            if (elapsed >= 1) {
                std::lock_guard<std::mutex> lock(mutex_);
                frameRate_ = static_cast<double>(localFrameCount) / elapsed;
                lastUpdate = now;
                localFrameCount = 0;
                frameCount_ += localFrameCount;
            }

            // Simulate IMU data reception
            if (localFrameCount % 3 == 0) {  // Every 3rd frame
                IMUSample imu;
                imu.timestampNs = timestamp;
                imu.accX = 0.0;
                imu.accY = 0.0;
                imu.accZ = 9.81;
                imu.gyroX = 0.0;
                imu.gyroY = 0.0;
                imu.gyroZ = 0.0;

                std::lock_guard<std::mutex> lock(mutex_);
                if (imuCallback_) {
                    imuCallback_(imu);
                }
            }
        }

        std::cout << "[WebRTCReceiver] Receive loop stopped" << std::endl;
    }

    mutable std::mutex mutex_;
    bool connected_;
    std::string signalingUrl_;
    std::thread receiveThread_;

    VideoCallback videoCallback_;
    IMUCallback imuCallback_;

    int64_t frameCount_;
    double frameRate_ = 0.0;
};

// WebRTCReceiver implementation

WebRTCReceiver::WebRTCReceiver() : impl_(std::make_unique<Impl>()) {}

WebRTCReceiver::~WebRTCReceiver() = default;

bool WebRTCReceiver::initialize() {
    return impl_->initialize();
}

bool WebRTCReceiver::connect(const std::string& signalingUrl) {
    return impl_->connect(signalingUrl);
}

void WebRTCReceiver::disconnect() {
    impl_->disconnect();
}

bool WebRTCReceiver::isConnected() const {
    return impl_->isConnected();
}

void WebRTCReceiver::setVideoCallback(VideoCallback callback) {
    impl_->setVideoCallback(callback);
}

void WebRTCReceiver::setIMUCallback(IMUCallback callback) {
    impl_->setIMUCallback(callback);
}

double WebRTCReceiver::getFrameRate() const {
    return impl_->getFrameRate();
}

}  // namespace vi_slam
