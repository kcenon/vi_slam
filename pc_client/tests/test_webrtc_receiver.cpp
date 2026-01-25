#include "webrtc_receiver.hpp"
#include <gtest/gtest.h>
#include <chrono>
#include <thread>

namespace vi_slam {
namespace test {

class WebRTCReceiverTest : public ::testing::Test {
protected:
    void SetUp() override {
        receiver_ = std::make_unique<WebRTCReceiver>();
    }

    void TearDown() override {
        if (receiver_ && receiver_->isConnected()) {
            receiver_->disconnect();
        }
        receiver_.reset();
    }

    std::unique_ptr<WebRTCReceiver> receiver_;
};

TEST_F(WebRTCReceiverTest, Initialization) {
    ASSERT_TRUE(receiver_->initialize());
}

TEST_F(WebRTCReceiverTest, ConnectDisconnect) {
    ASSERT_TRUE(receiver_->initialize());
    ASSERT_TRUE(receiver_->connect("ws://localhost:8080"));
    ASSERT_TRUE(receiver_->isConnected());

    receiver_->disconnect();
    ASSERT_FALSE(receiver_->isConnected());
}

TEST_F(WebRTCReceiverTest, VideoCallback) {
    ASSERT_TRUE(receiver_->initialize());

    bool callbackCalled = false;
    int64_t receivedTimestamp = 0;

    receiver_->setVideoCallback([&](const cv::Mat& frame, int64_t timestamp) {
        callbackCalled = true;
        receivedTimestamp = timestamp;
        EXPECT_GT(frame.cols, 0);
        EXPECT_GT(frame.rows, 0);
    });

    ASSERT_TRUE(receiver_->connect("ws://localhost:8080"));

    // Wait for callback
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    EXPECT_TRUE(callbackCalled);
    EXPECT_GT(receivedTimestamp, 0);
}

TEST_F(WebRTCReceiverTest, IMUCallback) {
    ASSERT_TRUE(receiver_->initialize());

    bool callbackCalled = false;

    receiver_->setIMUCallback([&](const IMUSample& imu) {
        callbackCalled = true;
        EXPECT_GT(imu.timestampNs, 0);
    });

    ASSERT_TRUE(receiver_->connect("ws://localhost:8080"));

    // Wait for callback
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    EXPECT_TRUE(callbackCalled);
}

TEST_F(WebRTCReceiverTest, FrameRate) {
    ASSERT_TRUE(receiver_->initialize());
    ASSERT_TRUE(receiver_->connect("ws://localhost:8080"));

    // Wait for frame rate calculation
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));

    double fps = receiver_->getFrameRate();
    EXPECT_GT(fps, 0.0);
    EXPECT_LT(fps, 100.0);  // Reasonable upper bound
}

TEST_F(WebRTCReceiverTest, MultipleConnect) {
    ASSERT_TRUE(receiver_->initialize());
    ASSERT_TRUE(receiver_->connect("ws://localhost:8080"));

    // Second connect should fail
    EXPECT_FALSE(receiver_->connect("ws://localhost:8080"));
}

}  // namespace test
}  // namespace vi_slam
