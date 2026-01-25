#ifdef ENABLE_ZMQ

#include "slam/output/zmq_publisher.hpp"
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <zmq.hpp>
#include <thread>
#include <chrono>

using json = nlohmann::json;

namespace vi_slam {
namespace output {
namespace test {

class ZMQPublisherTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Use a unique port for each test to avoid conflicts
        testPort_ = 15555 + testCount_++;
    }

    void TearDown() override {
        // Allow socket cleanup
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    static int testCount_;
    int testPort_;
};

int ZMQPublisherTest::testCount_ = 0;

TEST_F(ZMQPublisherTest, ConstructorWithDefaultConfig) {
    ZMQPublisherConfig config;
    ZMQPublisher publisher(config);

    EXPECT_EQ(publisher.getConfig().endpoint, "tcp://*:5555");
    EXPECT_EQ(publisher.getConfig().ioThreads, 1);
    EXPECT_EQ(publisher.getConfig().sendTimeout, 100);
    EXPECT_EQ(publisher.getConfig().highWaterMark, 10);
}

TEST_F(ZMQPublisherTest, ConstructorWithCustomConfig) {
    ZMQPublisherConfig config;
    config.endpoint = "tcp://*:" + std::to_string(testPort_);
    config.ioThreads = 2;
    config.sendTimeout = 200;
    config.highWaterMark = 20;

    ZMQPublisher publisher(config);

    EXPECT_EQ(publisher.getConfig().endpoint, "tcp://*:" + std::to_string(testPort_));
    EXPECT_EQ(publisher.getConfig().ioThreads, 2);
    EXPECT_EQ(publisher.getConfig().sendTimeout, 200);
    EXPECT_EQ(publisher.getConfig().highWaterMark, 20);
}

TEST_F(ZMQPublisherTest, PublishValidPose) {
    ZMQPublisherConfig config;
    config.endpoint = "tcp://*:" + std::to_string(testPort_);
    ZMQPublisher publisher(config);

    Pose6DoF pose;
    pose.timestampNs = 1234567890000000;  // 1234567890.0 seconds
    pose.position[0] = 1.5;
    pose.position[1] = 2.5;
    pose.position[2] = 3.5;
    pose.orientation[0] = 1.0;  // qw
    pose.orientation[1] = 0.0;  // qx
    pose.orientation[2] = 0.0;  // qy
    pose.orientation[3] = 0.0;  // qz
    pose.valid = true;

    // Should succeed
    EXPECT_TRUE(publisher.publishPose(pose));
}

TEST_F(ZMQPublisherTest, IgnoreInvalidPose) {
    ZMQPublisherConfig config;
    config.endpoint = "tcp://*:" + std::to_string(testPort_);
    ZMQPublisher publisher(config);

    Pose6DoF pose;
    pose.valid = false;

    // Should return false for invalid pose
    EXPECT_FALSE(publisher.publishPose(pose));
}

TEST_F(ZMQPublisherTest, JSONFormat) {
    ZMQPublisherConfig config;
    config.endpoint = "tcp://*:" + std::to_string(testPort_);
    ZMQPublisher publisher(config);

    // Create ZMQ subscriber to receive message
    zmq::context_t context(1);
    zmq::socket_t subscriber(context, zmq::socket_type::sub);
    subscriber.connect("tcp://localhost:" + std::to_string(testPort_));
    subscriber.set(zmq::sockopt::subscribe, "");
    subscriber.set(zmq::sockopt::rcvtimeo, 1000);  // 1 second timeout

    // Allow connection to establish
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Publish pose
    Pose6DoF pose;
    pose.timestampNs = 1234567890123456789;  // Nanoseconds
    pose.position[0] = 1.5;
    pose.position[1] = 2.5;
    pose.position[2] = 3.5;
    pose.orientation[0] = 0.707;  // qw
    pose.orientation[1] = 0.707;  // qx
    pose.orientation[2] = 0.0;    // qy
    pose.orientation[3] = 0.0;    // qz
    pose.valid = true;

    EXPECT_TRUE(publisher.publishPose(pose));

    // Receive message
    zmq::message_t message;
    auto result = subscriber.recv(message, zmq::recv_flags::none);

    ASSERT_TRUE(result.has_value());

    // Parse JSON
    std::string jsonStr(static_cast<char*>(message.data()), message.size());
    json j = json::parse(jsonStr);

    // Verify JSON structure
    EXPECT_TRUE(j.contains("timestamp"));
    EXPECT_TRUE(j.contains("pose"));
    EXPECT_TRUE(j["pose"].contains("position"));
    EXPECT_TRUE(j["pose"].contains("orientation"));
    EXPECT_TRUE(j.contains("velocity"));
    EXPECT_TRUE(j["velocity"].contains("linear"));
    EXPECT_TRUE(j["velocity"].contains("angular"));

    // Verify values (timestamp in nanoseconds / 1e9 = seconds)
    EXPECT_NEAR(j["timestamp"].get<double>(), 1234567890.123456789, 1e-3);
    EXPECT_NEAR(j["pose"]["position"]["x"].get<double>(), 1.5, 1e-6);
    EXPECT_NEAR(j["pose"]["position"]["y"].get<double>(), 2.5, 1e-6);
    EXPECT_NEAR(j["pose"]["position"]["z"].get<double>(), 3.5, 1e-6);
    EXPECT_NEAR(j["pose"]["orientation"]["x"].get<double>(), 0.707, 1e-3);
    EXPECT_NEAR(j["pose"]["orientation"]["y"].get<double>(), 0.0, 1e-6);
    EXPECT_NEAR(j["pose"]["orientation"]["z"].get<double>(), 0.0, 1e-6);
    EXPECT_NEAR(j["pose"]["orientation"]["w"].get<double>(), 0.707, 1e-3);

    // Close subscriber
    subscriber.close();
    context.close();
}

TEST_F(ZMQPublisherTest, VelocityComputation) {
    ZMQPublisherConfig config;
    config.endpoint = "tcp://*:" + std::to_string(testPort_);
    ZMQPublisher publisher(config);

    // Create ZMQ subscriber
    zmq::context_t context(1);
    zmq::socket_t subscriber(context, zmq::socket_type::sub);
    subscriber.connect("tcp://localhost:" + std::to_string(testPort_));
    subscriber.set(zmq::sockopt::subscribe, "");
    subscriber.set(zmq::sockopt::rcvtimeo, 1000);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Publish first pose (velocity should be zero)
    Pose6DoF pose1;
    pose1.timestampNs = 1000000000;  // 1.0 second
    pose1.position[0] = 0.0;
    pose1.position[1] = 0.0;
    pose1.position[2] = 0.0;
    pose1.orientation[0] = 1.0;
    pose1.valid = true;

    EXPECT_TRUE(publisher.publishPose(pose1));

    zmq::message_t msg1;
    subscriber.recv(msg1, zmq::recv_flags::none);
    std::string jsonStr1(static_cast<char*>(msg1.data()), msg1.size());
    json j1 = json::parse(jsonStr1);

    // First pose should have zero velocity
    EXPECT_NEAR(j1["velocity"]["linear"]["x"].get<double>(), 0.0, 1e-6);

    // Publish second pose (moved 1.0m in x over 1.0s)
    Pose6DoF pose2;
    pose2.timestampNs = 2000000000;  // 2.0 seconds
    pose2.position[0] = 1.0;
    pose2.position[1] = 0.0;
    pose2.position[2] = 0.0;
    pose2.orientation[0] = 1.0;
    pose2.valid = true;

    EXPECT_TRUE(publisher.publishPose(pose2));

    zmq::message_t msg2;
    subscriber.recv(msg2, zmq::recv_flags::none);
    std::string jsonStr2(static_cast<char*>(msg2.data()), msg2.size());
    json j2 = json::parse(jsonStr2);

    // Second pose should have velocity of 1.0 m/s in x
    EXPECT_NEAR(j2["velocity"]["linear"]["x"].get<double>(), 1.0, 1e-3);
    EXPECT_NEAR(j2["velocity"]["linear"]["y"].get<double>(), 0.0, 1e-6);
    EXPECT_NEAR(j2["velocity"]["linear"]["z"].get<double>(), 0.0, 1e-6);

    subscriber.close();
    context.close();
}

TEST_F(ZMQPublisherTest, LatencyTracking) {
    ZMQPublisherConfig config;
    config.endpoint = "tcp://*:" + std::to_string(testPort_);
    ZMQPublisher publisher(config);

    // Publish several poses
    for (int i = 0; i < 100; ++i) {
        Pose6DoF pose;
        pose.timestampNs = (i + 1) * 1000000000;
        pose.position[0] = i * 0.1;
        pose.orientation[0] = 1.0;
        pose.valid = true;

        EXPECT_TRUE(publisher.publishPose(pose));
    }

    // Check latency statistics
    double avgLatency = publisher.getAverageLatencyUs();
    double p99Latency = publisher.getP99LatencyUs();

    EXPECT_GT(avgLatency, 0.0);
    EXPECT_GT(p99Latency, 0.0);
    EXPECT_GE(p99Latency, avgLatency);

    // Target: <10ms = 10000us (should be much lower in practice)
    EXPECT_LT(p99Latency, 10000.0);
}

TEST_F(ZMQPublisherTest, ResetLatencyStats) {
    ZMQPublisherConfig config;
    config.endpoint = "tcp://*:" + std::to_string(testPort_);
    ZMQPublisher publisher(config);

    // Publish some poses
    for (int i = 0; i < 10; ++i) {
        Pose6DoF pose;
        pose.timestampNs = (i + 1) * 1000000000;
        pose.valid = true;
        publisher.publishPose(pose);
    }

    EXPECT_GT(publisher.getAverageLatencyUs(), 0.0);

    // Reset stats
    publisher.resetLatencyStats();

    EXPECT_EQ(publisher.getAverageLatencyUs(), 0.0);
    EXPECT_EQ(publisher.getP99LatencyUs(), 0.0);
}

TEST_F(ZMQPublisherTest, HighFrequencyPublish) {
    ZMQPublisherConfig config;
    config.endpoint = "tcp://*:" + std::to_string(testPort_);
    ZMQPublisher publisher(config);

    // Simulate high-frequency publishing (e.g., 100Hz)
    auto startTime = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < 100; ++i) {
        Pose6DoF pose;
        pose.timestampNs = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::high_resolution_clock::now().time_since_epoch()).count();
        pose.position[0] = i * 0.01;
        pose.orientation[0] = 1.0;
        pose.valid = true;

        EXPECT_TRUE(publisher.publishPose(pose));

        // Sleep for ~10ms (100Hz)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        endTime - startTime).count();

    // Should take approximately 1 second (100 * 10ms)
    EXPECT_GT(duration, 950);   // Allow some tolerance
    EXPECT_LT(duration, 1500);  // Increased tolerance for slower systems

    // Verify latency is low
    double p99Latency = publisher.getP99LatencyUs();
    EXPECT_LT(p99Latency, 10000.0);  // <10ms target
}

}  // namespace test
}  // namespace output
}  // namespace vi_slam

#endif  // ENABLE_ZMQ
