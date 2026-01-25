#ifndef VI_SLAM_SLAM_OUTPUT_ZMQ_PUBLISHER_HPP
#define VI_SLAM_SLAM_OUTPUT_ZMQ_PUBLISHER_HPP

#include "common/types.hpp"

#ifdef ENABLE_ZMQ

#include <zmq.hpp>
#include <string>
#include <memory>
#include <chrono>

namespace vi_slam {
namespace output {

/**
 * @brief Configuration for ZMQ publisher
 */
struct ZMQPublisherConfig {
    std::string endpoint;       // Default: "tcp://*:5555"
    int ioThreads;             // Default: 1
    int sendTimeout;           // Default: 100 (ms)
    int highWaterMark;         // Default: 10 (queue size)

    ZMQPublisherConfig()
        : endpoint("tcp://*:5555"),
          ioThreads(1),
          sendTimeout(100),
          highWaterMark(10) {}
};

/**
 * @brief Publishes SLAM output via ZMQ PUB-SUB pattern
 *
 * This class provides low-latency real-time pose output via ZeroMQ,
 * targeting <10ms end-to-end latency for control and AR/VR applications.
 */
class ZMQPublisher {
public:
    /**
     * @brief Constructor with configuration
     *
     * @param config Publisher configuration
     */
    explicit ZMQPublisher(const ZMQPublisherConfig& config = ZMQPublisherConfig());

    /**
     * @brief Destructor
     */
    ~ZMQPublisher();

    // Prevent copying
    ZMQPublisher(const ZMQPublisher&) = delete;
    ZMQPublisher& operator=(const ZMQPublisher&) = delete;

    /**
     * @brief Publish a new pose
     *
     * Serializes pose to JSON and publishes via ZMQ.
     * Format:
     * {
     *   "timestamp": 1234567890.123,
     *   "pose": {
     *     "position": {"x": 0.0, "y": 0.0, "z": 0.0},
     *     "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
     *   },
     *   "velocity": {
     *     "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
     *     "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
     *   }
     * }
     *
     * @param pose 6DoF pose with timestamp
     * @return true if publish succeeded, false otherwise
     */
    bool publishPose(const Pose6DoF& pose);

    /**
     * @brief Get current configuration
     *
     * @return Current publisher configuration
     */
    const ZMQPublisherConfig& getConfig() const { return config_; }

    /**
     * @brief Get average publish latency
     *
     * @return Average latency in microseconds
     */
    double getAverageLatencyUs() const;

    /**
     * @brief Get P99 publish latency
     *
     * @return P99 latency in microseconds
     */
    double getP99LatencyUs() const;

    /**
     * @brief Reset latency statistics
     */
    void resetLatencyStats();

private:
    /**
     * @brief Convert Pose6DoF to JSON string
     *
     * @param pose Input pose
     * @return JSON-formatted string
     */
    std::string toJSON(const Pose6DoF& pose) const;

    /**
     * @brief Record latency measurement
     *
     * @param latencyUs Latency in microseconds
     */
    void recordLatency(double latencyUs);

    // ZMQ context and socket
    std::unique_ptr<zmq::context_t> context_;
    std::unique_ptr<zmq::socket_t> publisher_;

    // Configuration
    ZMQPublisherConfig config_;

    // Previous pose for velocity computation
    bool hasPreviousPose_;
    Pose6DoF previousPose_;

    // Latency tracking
    std::vector<double> latencySamples_;  // Ring buffer of last 1000 samples
    size_t maxSamples_;
};

}  // namespace output
}  // namespace vi_slam

#endif  // ENABLE_ZMQ

#endif  // VI_SLAM_SLAM_OUTPUT_ZMQ_PUBLISHER_HPP
