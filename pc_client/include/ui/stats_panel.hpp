#ifndef VI_SLAM_UI_STATS_PANEL_HPP
#define VI_SLAM_UI_STATS_PANEL_HPP

#include <chrono>
#include <deque>
#include "webrtc_receiver.hpp"

namespace vi_slam {
namespace ui {

/**
 * Data reception statistics panel for ImGui.
 *
 * Displays:
 * - Frame rate (FPS) for camera data
 * - IMU data rate (samples/sec)
 * - Data throughput (KB/s or MB/s)
 * - Buffer utilization indicators
 * - Historical graphs (optional)
 */
class StatsPanel {
public:
    StatsPanel();
    ~StatsPanel() = default;

    /**
     * Render the statistics panel.
     *
     * @param receiver WebRTC receiver instance
     * @param frameCount Total frames received
     * @param imuCount Total IMU samples received
     */
    void render(WebRTCReceiver& receiver, int frameCount, int imuCount);

    /**
     * Update panel state (call in main loop).
     *
     * Updates statistics calculations and historical data.
     */
    void update();

    /**
     * Record a frame reception event.
     *
     * @param size Frame size in bytes
     */
    void recordFrame(size_t size);

    /**
     * Record an IMU sample reception event.
     *
     * @param size Sample size in bytes
     */
    void recordIMUSample(size_t size);

    /**
     * Calculate current rates from frame/IMU counts.
     *
     * @param frameCount Current total frame count
     * @param imuCount Current total IMU count
     */
    void updateRates(int frameCount, int imuCount);

private:
    // Timing
    std::chrono::steady_clock::time_point lastUpdateTime_;

    // Frame statistics
    int lastFrameCount_;
    double currentFPS_;
    std::deque<std::pair<std::chrono::steady_clock::time_point, size_t>> frameHistory_;

    // IMU statistics
    int lastIMUCount_;
    double currentIMURate_;
    std::deque<std::pair<std::chrono::steady_clock::time_point, size_t>> imuHistory_;

    // Data rate statistics
    size_t totalBytesReceived_;
    double currentDataRate_;  // Bytes per second

    // Historical data for graphs (last 60 seconds)
    static constexpr int HISTORY_SIZE = 60;
    std::deque<float> fpsHistory_;
    std::deque<float> imuRateHistory_;
    std::deque<float> dataRateHistory_;

    /**
     * Format data rate in appropriate units (B/KB/MB/s).
     *
     * @param bytesPerSecond Data rate in bytes per second
     * @return Formatted string
     */
    std::string formatDataRate(double bytesPerSecond) const;

    /**
     * Clean up old entries from history (older than 60 seconds).
     */
    void cleanupHistory();
};

}  // namespace ui
}  // namespace vi_slam

#endif  // VI_SLAM_UI_STATS_PANEL_HPP
