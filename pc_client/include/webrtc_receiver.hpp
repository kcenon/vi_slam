#ifndef VI_SLAM_WEBRTC_RECEIVER_HPP
#define VI_SLAM_WEBRTC_RECEIVER_HPP

#include <functional>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <common/types.hpp>

namespace vi_slam {

/**
 * WebRTC receiver for Android video stream and IMU data.
 *
 * Handles WebRTC connection, H.264 video decoding, and UDP IMU packet parsing.
 */
class WebRTCReceiver {
public:
    using VideoCallback = std::function<void(const cv::Mat&, int64_t)>;
    using IMUCallback = std::function<void(const IMUSample&)>;

    WebRTCReceiver();
    ~WebRTCReceiver();

    // Disable copy and move
    WebRTCReceiver(const WebRTCReceiver&) = delete;
    WebRTCReceiver& operator=(const WebRTCReceiver&) = delete;
    WebRTCReceiver(WebRTCReceiver&&) = delete;
    WebRTCReceiver& operator=(WebRTCReceiver&&) = delete;

    /**
     * Initialize the WebRTC receiver.
     *
     * @return true if initialization successful
     */
    bool initialize();

    /**
     * Connect to Android device via WebRTC signaling server.
     *
     * @param signalingUrl WebRTC signaling server URL
     * @return true if connection established
     */
    bool connect(const std::string& signalingUrl);

    /**
     * Disconnect from Android device.
     */
    void disconnect();

    /**
     * Check if currently connected.
     *
     * @return true if connected
     */
    bool isConnected() const;

    /**
     * Set callback for decoded video frames.
     *
     * @param callback Function to call when frame is decoded
     */
    void setVideoCallback(VideoCallback callback);

    /**
     * Set callback for IMU data.
     *
     * @param callback Function to call when IMU sample received
     */
    void setIMUCallback(IMUCallback callback);

    /**
     * Get current frame rate.
     *
     * @return Frames per second
     */
    double getFrameRate() const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace vi_slam

#endif  // VI_SLAM_WEBRTC_RECEIVER_HPP
