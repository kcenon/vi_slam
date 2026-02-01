#ifndef VI_SLAM_UI_CONNECTION_PANEL_HPP
#define VI_SLAM_UI_CONNECTION_PANEL_HPP

#include <chrono>
#include <string>
#include "webrtc_receiver.hpp"

namespace vi_slam {
namespace ui {

/**
 * Connection status dashboard panel for ImGui.
 *
 * Displays:
 * - Connection status indicator (connected/disconnected)
 * - Server IP and port
 * - Connection uptime counter
 * - Auto-reconnect toggle and status
 */
class ConnectionPanel {
public:
    ConnectionPanel();
    ~ConnectionPanel() = default;

    /**
     * Render the connection panel.
     *
     * @param receiver WebRTC receiver instance
     * @param signalingUrl Current signaling server URL
     */
    void render(WebRTCReceiver& receiver, const std::string& signalingUrl);

    /**
     * Enable or disable auto-reconnect.
     *
     * @param enable true to enable auto-reconnect
     */
    void setAutoReconnect(bool enable);

    /**
     * Check if auto-reconnect is enabled.
     *
     * @return true if auto-reconnect is enabled
     */
    bool isAutoReconnectEnabled() const;

    /**
     * Update panel state (call in main loop).
     *
     * Handles auto-reconnect logic and uptime updates.
     *
     * @param receiver WebRTC receiver instance
     * @param signalingUrl Current signaling server URL
     */
    void update(WebRTCReceiver& receiver, const std::string& signalingUrl);

private:
    bool autoReconnect_;
    bool wasConnected_;
    std::chrono::steady_clock::time_point connectionStartTime_;
    std::chrono::steady_clock::time_point lastReconnectAttempt_;

    /**
     * Format uptime as HH:MM:SS.
     *
     * @return Formatted uptime string
     */
    std::string formatUptime() const;

    /**
     * Attempt to reconnect if auto-reconnect is enabled.
     *
     * @param receiver WebRTC receiver instance
     * @param signalingUrl Signaling server URL to reconnect to
     */
    void attemptReconnect(WebRTCReceiver& receiver, const std::string& signalingUrl);
};

}  // namespace ui
}  // namespace vi_slam

#endif  // VI_SLAM_UI_CONNECTION_PANEL_HPP
