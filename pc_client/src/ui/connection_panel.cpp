#include "ui/connection_panel.hpp"
#include <sstream>
#include <iomanip>
#include <iostream>
#include "imgui.h"

namespace vi_slam {
namespace ui {

ConnectionPanel::ConnectionPanel()
    : autoReconnect_(false),
      wasConnected_(false),
      connectionStartTime_(),
      lastReconnectAttempt_() {
}

void ConnectionPanel::render(WebRTCReceiver& receiver, const std::string& signalingUrl) {
    ImGui::Begin("Connection Status");

    // Connection status indicator with color
    bool isConnected = receiver.isConnected();
    if (isConnected) {
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.0f, 1.0f, 0.0f, 1.0f));  // Green
        ImGui::Text("● Connected");
        ImGui::PopStyleColor();
    } else {
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.0f, 0.0f, 1.0f));  // Red
        ImGui::Text("● Disconnected");
        ImGui::PopStyleColor();
    }

    ImGui::Separator();

    // Server information
    ImGui::Text("Server Information");
    ImGui::Text("Signaling URL: %s", signalingUrl.c_str());

    // Extract and display host/port from URL
    size_t hostStart = signalingUrl.find("://");
    if (hostStart != std::string::npos) {
        hostStart += 3;  // Skip "://"
        size_t hostEnd = signalingUrl.find('/', hostStart);
        std::string hostPort = signalingUrl.substr(hostStart,
            hostEnd != std::string::npos ? hostEnd - hostStart : std::string::npos);

        size_t colonPos = hostPort.find(':');
        if (colonPos != std::string::npos) {
            std::string host = hostPort.substr(0, colonPos);
            std::string port = hostPort.substr(colonPos + 1);
            ImGui::Text("Host: %s", host.c_str());
            ImGui::Text("Port: %s", port.c_str());
        } else {
            ImGui::Text("Host: %s", hostPort.c_str());
            ImGui::Text("Port: (default)");
        }
    }

    ImGui::Separator();

    // Connection uptime
    if (isConnected) {
        std::string uptime = formatUptime();
        ImGui::Text("Uptime: %s", uptime.c_str());
    } else {
        ImGui::TextDisabled("Uptime: --:--:--");
    }

    ImGui::Separator();

    // Auto-reconnect controls
    ImGui::Text("Auto-Reconnect");
    ImGui::Checkbox("Enable Auto-Reconnect", &autoReconnect_);

    if (autoReconnect_) {
        ImGui::SameLine();
        ImGui::TextDisabled("(?)");
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Automatically reconnect when connection is lost");
        }

        if (!isConnected) {
            auto now = std::chrono::steady_clock::now();
            auto timeSinceAttempt = std::chrono::duration_cast<std::chrono::seconds>(
                now - lastReconnectAttempt_
            ).count();

            if (timeSinceAttempt < 5) {
                ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f),
                    "Reconnecting in %lld seconds...", 5 - timeSinceAttempt);
            }
        }
    }

    ImGui::Separator();

    // Manual connection controls
    if (!isConnected) {
        if (ImGui::Button("Connect", ImVec2(120, 0))) {
            std::cout << "Connecting to signaling server: " << signalingUrl << std::endl;
            if (receiver.connect(signalingUrl)) {
                connectionStartTime_ = std::chrono::steady_clock::now();
                std::cout << "Connected!" << std::endl;
            } else {
                std::cerr << "Failed to connect" << std::endl;
            }
        }
    } else {
        if (ImGui::Button("Disconnect", ImVec2(120, 0))) {
            std::cout << "Disconnecting..." << std::endl;
            receiver.disconnect();
            std::cout << "Disconnected" << std::endl;
        }
    }

    ImGui::End();
}

void ConnectionPanel::setAutoReconnect(bool enable) {
    autoReconnect_ = enable;
}

bool ConnectionPanel::isAutoReconnectEnabled() const {
    return autoReconnect_;
}

void ConnectionPanel::update(WebRTCReceiver& receiver, const std::string& signalingUrl) {
    bool isConnected = receiver.isConnected();

    // Detect connection state changes
    if (isConnected && !wasConnected_) {
        // Just connected
        connectionStartTime_ = std::chrono::steady_clock::now();
    } else if (!isConnected && wasConnected_) {
        // Just disconnected
        if (autoReconnect_) {
            lastReconnectAttempt_ = std::chrono::steady_clock::now();
        }
    }

    wasConnected_ = isConnected;

    // Handle auto-reconnect
    if (autoReconnect_ && !isConnected) {
        attemptReconnect(receiver, signalingUrl);
    }
}

std::string ConnectionPanel::formatUptime() const {
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(
        now - connectionStartTime_
    ).count();

    int hours = duration / 3600;
    int minutes = (duration % 3600) / 60;
    int seconds = duration % 60;

    std::ostringstream oss;
    oss << std::setfill('0') << std::setw(2) << hours << ":"
        << std::setfill('0') << std::setw(2) << minutes << ":"
        << std::setfill('0') << std::setw(2) << seconds;

    return oss.str();
}

void ConnectionPanel::attemptReconnect(WebRTCReceiver& receiver,
                                       const std::string& signalingUrl) {
    auto now = std::chrono::steady_clock::now();
    auto timeSinceAttempt = std::chrono::duration_cast<std::chrono::seconds>(
        now - lastReconnectAttempt_
    ).count();

    // Try to reconnect every 5 seconds
    if (timeSinceAttempt >= 5) {
        std::cout << "Auto-reconnect: Attempting to reconnect..." << std::endl;
        if (receiver.connect(signalingUrl)) {
            connectionStartTime_ = std::chrono::steady_clock::now();
            std::cout << "Auto-reconnect: Connected!" << std::endl;
        } else {
            std::cerr << "Auto-reconnect: Failed to connect" << std::endl;
            lastReconnectAttempt_ = now;
        }
    }
}

}  // namespace ui
}  // namespace vi_slam
