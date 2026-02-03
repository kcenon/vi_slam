#include "slam/output/zmq_publisher.hpp"

#ifdef ENABLE_ZMQ

#include <nlohmann/json.hpp>
#include <algorithm>
#include <numeric>
#include <sstream>
#include <iomanip>
#include <Eigen/Core>

using json = nlohmann::json;

namespace vi_slam {
namespace output {

ZMQPublisher::ZMQPublisher(const ZMQPublisherConfig& config)
    : config_(config),
      hasPreviousPose_(false),
      maxSamples_(1000) {

    // Create ZMQ context
    context_ = std::make_unique<zmq::context_t>(config_.ioThreads);

    // Create publisher socket
    publisher_ = std::make_unique<zmq::socket_t>(*context_, zmq::socket_type::pub);

    // Set socket options
    publisher_->set(zmq::sockopt::sndhwm, config_.highWaterMark);
    publisher_->set(zmq::sockopt::sndtimeo, config_.sendTimeout);

    // Bind to endpoint
    publisher_->bind(config_.endpoint);

    // Reserve space for latency samples
    latencySamples_.reserve(maxSamples_);
}

ZMQPublisher::~ZMQPublisher() {
    // Close socket and context gracefully
    if (publisher_) {
        publisher_->close();
    }
    if (context_) {
        context_->close();
    }
}

bool ZMQPublisher::publishPose(const Pose6DoF& pose) {
    if (!pose.valid) {
        return false;
    }

    auto startTime = std::chrono::high_resolution_clock::now();

    try {
        // Convert to JSON
        std::string jsonStr = toJSON(pose);

        // Create ZMQ message
        zmq::message_t message(jsonStr.data(), jsonStr.size());

        // Send message (non-blocking)
        zmq::send_result_t result = publisher_->send(message, zmq::send_flags::dontwait);

        // Record latency
        auto endTime = std::chrono::high_resolution_clock::now();
        auto latencyUs = std::chrono::duration_cast<std::chrono::microseconds>(
            endTime - startTime).count();
        recordLatency(static_cast<double>(latencyUs));

        // Store for next iteration
        previousPose_ = pose;
        hasPreviousPose_ = true;

        return result.has_value();

    } catch (const zmq::error_t& e) {
        // ZMQ error (e.g., EAGAIN if high water mark reached)
        return false;
    }
}

std::string ZMQPublisher::toJSON(const Pose6DoF& pose) const {
    json j;

    // Timestamp (convert nanoseconds to seconds with microsecond precision)
    j["timestamp"] = pose.timestampNs / 1e9;

    // Pose
    j["pose"]["position"]["x"] = pose.position.x();
    j["pose"]["position"]["y"] = pose.position.y();
    j["pose"]["position"]["z"] = pose.position.z();

    // Orientation
    j["pose"]["orientation"]["x"] = pose.orientation.x();
    j["pose"]["orientation"]["y"] = pose.orientation.y();
    j["pose"]["orientation"]["z"] = pose.orientation.z();
    j["pose"]["orientation"]["w"] = pose.orientation.w();

    // Velocity computation
    if (hasPreviousPose_) {
        double dt = (pose.timestampNs - previousPose_.timestampNs) / 1e9;  // seconds

        if (dt > 0) {
            // Linear velocity
            Eigen::Vector3d velocity = (pose.position - previousPose_.position) / dt;
            j["velocity"]["linear"]["x"] = velocity.x();
            j["velocity"]["linear"]["y"] = velocity.y();
            j["velocity"]["linear"]["z"] = velocity.z();

            // Angular velocity computation would require quaternion differentiation
            // For simplicity, set to zero (can be enhanced later)
            j["velocity"]["angular"]["x"] = 0.0;
            j["velocity"]["angular"]["y"] = 0.0;
            j["velocity"]["angular"]["z"] = 0.0;
        } else {
            // Zero velocity if dt is invalid
            j["velocity"]["linear"]["x"] = 0.0;
            j["velocity"]["linear"]["y"] = 0.0;
            j["velocity"]["linear"]["z"] = 0.0;
            j["velocity"]["angular"]["x"] = 0.0;
            j["velocity"]["angular"]["y"] = 0.0;
            j["velocity"]["angular"]["z"] = 0.0;
        }
    } else {
        // No previous pose, initialize velocity to zero
        j["velocity"]["linear"]["x"] = 0.0;
        j["velocity"]["linear"]["y"] = 0.0;
        j["velocity"]["linear"]["z"] = 0.0;
        j["velocity"]["angular"]["x"] = 0.0;
        j["velocity"]["angular"]["y"] = 0.0;
        j["velocity"]["angular"]["z"] = 0.0;
    }

    // Serialize to compact string (no indentation)
    return j.dump();
}

void ZMQPublisher::recordLatency(double latencyUs) {
    if (latencySamples_.size() >= maxSamples_) {
        // Ring buffer behavior: remove oldest sample
        latencySamples_.erase(latencySamples_.begin());
    }
    latencySamples_.push_back(latencyUs);
}

double ZMQPublisher::getAverageLatencyUs() const {
    if (latencySamples_.empty()) {
        return 0.0;
    }

    double sum = std::accumulate(latencySamples_.begin(), latencySamples_.end(), 0.0);
    return sum / latencySamples_.size();
}

double ZMQPublisher::getP99LatencyUs() const {
    if (latencySamples_.empty()) {
        return 0.0;
    }

    // Sort samples to compute percentile
    std::vector<double> sorted = latencySamples_;
    std::sort(sorted.begin(), sorted.end());

    // P99 index
    size_t p99Index = static_cast<size_t>(sorted.size() * 0.99);
    if (p99Index >= sorted.size()) {
        p99Index = sorted.size() - 1;
    }

    return sorted[p99Index];
}

void ZMQPublisher::resetLatencyStats() {
    latencySamples_.clear();
}

}  // namespace output
}  // namespace vi_slam

#endif  // ENABLE_ZMQ
