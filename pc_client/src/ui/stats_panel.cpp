#include "ui/stats_panel.hpp"
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <vector>
#include "imgui.h"

namespace vi_slam {
namespace ui {

StatsPanel::StatsPanel()
    : lastUpdateTime_(std::chrono::steady_clock::now()),
      lastFrameCount_(0),
      currentFPS_(0.0),
      lastIMUCount_(0),
      currentIMURate_(0.0),
      totalBytesReceived_(0),
      currentDataRate_(0.0) {
}

void StatsPanel::render(WebRTCReceiver& receiver, int frameCount, int imuCount) {
    ImGui::Begin("Data Reception Statistics");

    // Connection status indicator
    bool isConnected = receiver.isConnected();
    if (!isConnected) {
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.5f, 0.0f, 1.0f));  // Orange
        ImGui::Text("⚠ Not Connected");
        ImGui::PopStyleColor();
        ImGui::Separator();
        ImGui::TextDisabled("Connect to view statistics");
        ImGui::End();
        return;
    }

    // Real-time metrics section
    ImGui::Text("Real-Time Metrics");
    ImGui::Separator();

    // Frame rate (FPS)
    ImGui::Text("Frame Rate:");
    ImGui::SameLine(150);
    if (currentFPS_ > 0) {
        ImGui::Text("%.1f FPS", currentFPS_);
    } else {
        ImGui::TextDisabled("-- FPS");
    }

    // IMU data rate
    ImGui::Text("IMU Rate:");
    ImGui::SameLine(150);
    if (currentIMURate_ > 0) {
        ImGui::Text("%.1f samples/s", currentIMURate_);
    } else {
        ImGui::TextDisabled("-- samples/s");
    }

    // Data throughput
    ImGui::Text("Data Rate:");
    ImGui::SameLine(150);
    if (currentDataRate_ > 0) {
        ImGui::Text("%s", formatDataRate(currentDataRate_).c_str());
    } else {
        ImGui::TextDisabled("-- B/s");
    }

    ImGui::Separator();

    // Cumulative statistics
    ImGui::Text("Cumulative Statistics");
    ImGui::Separator();

    ImGui::Text("Total Frames:");
    ImGui::SameLine(150);
    ImGui::Text("%d", frameCount);

    ImGui::Text("Total IMU Samples:");
    ImGui::SameLine(150);
    ImGui::Text("%d", imuCount);

    ImGui::Text("Total Data:");
    ImGui::SameLine(150);
    if (totalBytesReceived_ > 0) {
        ImGui::Text("%s", formatDataRate(totalBytesReceived_).c_str());
    } else {
        ImGui::Text("0 B");
    }

    ImGui::Separator();

    // Buffer utilization (placeholder)
    ImGui::Text("Buffer Utilization");
    ImGui::Separator();

    // Frame buffer indicator (simulated - would need actual buffer data)
    ImGui::Text("Frame Buffer:");
    ImGui::SameLine(150);
    float frameBufferUtil = std::min(1.0f, static_cast<float>(frameHistory_.size()) / 100.0f);
    ImGui::PushStyleColor(ImGuiCol_PlotHistogram,
        frameBufferUtil > 0.8f ? ImVec4(1.0f, 0.0f, 0.0f, 1.0f) :  // Red if high
        frameBufferUtil > 0.5f ? ImVec4(1.0f, 1.0f, 0.0f, 1.0f) :  // Yellow if medium
        ImVec4(0.0f, 1.0f, 0.0f, 1.0f));                           // Green if low
    ImGui::ProgressBar(frameBufferUtil, ImVec2(-1, 0), "");
    ImGui::PopStyleColor();
    ImGui::SameLine(0, 5);
    ImGui::Text("%.0f%%", frameBufferUtil * 100);

    // IMU buffer indicator (simulated)
    ImGui::Text("IMU Buffer:");
    ImGui::SameLine(150);
    float imuBufferUtil = std::min(1.0f, static_cast<float>(imuHistory_.size()) / 100.0f);
    ImGui::PushStyleColor(ImGuiCol_PlotHistogram,
        imuBufferUtil > 0.8f ? ImVec4(1.0f, 0.0f, 0.0f, 1.0f) :
        imuBufferUtil > 0.5f ? ImVec4(1.0f, 1.0f, 0.0f, 1.0f) :
        ImVec4(0.0f, 1.0f, 0.0f, 1.0f));
    ImGui::ProgressBar(imuBufferUtil, ImVec2(-1, 0), "");
    ImGui::PopStyleColor();
    ImGui::SameLine(0, 5);
    ImGui::Text("%.0f%%", imuBufferUtil * 100);

    ImGui::Separator();

    // Historical graphs
    ImGui::Text("Historical Data (Last 60s)");
    ImGui::Separator();

    if (!fpsHistory_.empty()) {
        // Convert deque to vector for ImGui::PlotLines
        std::vector<float> fpsVec(fpsHistory_.begin(), fpsHistory_.end());
        std::vector<float> imuRateVec(imuRateHistory_.begin(), imuRateHistory_.end());
        std::vector<float> dataRateVec(dataRateHistory_.begin(), dataRateHistory_.end());

        ImGui::Text("FPS History:");
        ImGui::PlotLines("##fps", fpsVec.data(),
            static_cast<int>(fpsVec.size()), 0, nullptr,
            0.0f, 60.0f, ImVec2(0, 80));

        ImGui::Text("IMU Rate History:");
        ImGui::PlotLines("##imu", imuRateVec.data(),
            static_cast<int>(imuRateVec.size()), 0, nullptr,
            0.0f, 200.0f, ImVec2(0, 80));

        ImGui::Text("Data Rate History:");
        ImGui::PlotLines("##data", dataRateVec.data(),
            static_cast<int>(dataRateVec.size()), 0, nullptr,
            0.0f, FLT_MAX, ImVec2(0, 80));
    } else {
        ImGui::TextDisabled("No historical data yet");
    }

    ImGui::End();
}

void StatsPanel::update() {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - lastUpdateTime_
    ).count();

    // Update every second
    if (elapsed >= 1000) {
        // Add current rates to history
        if (fpsHistory_.size() >= HISTORY_SIZE) {
            fpsHistory_.pop_front();
        }
        fpsHistory_.push_back(static_cast<float>(currentFPS_));

        if (imuRateHistory_.size() >= HISTORY_SIZE) {
            imuRateHistory_.pop_front();
        }
        imuRateHistory_.push_back(static_cast<float>(currentIMURate_));

        if (dataRateHistory_.size() >= HISTORY_SIZE) {
            dataRateHistory_.pop_front();
        }
        dataRateHistory_.push_back(static_cast<float>(currentDataRate_ / 1024.0));  // KB/s

        // Clean up old history entries
        cleanupHistory();

        lastUpdateTime_ = now;
    }
}

void StatsPanel::recordFrame(size_t size) {
    auto now = std::chrono::steady_clock::now();
    frameHistory_.push_back({now, size});
    totalBytesReceived_ += size;
}

void StatsPanel::recordIMUSample(size_t size) {
    auto now = std::chrono::steady_clock::now();
    imuHistory_.push_back({now, size});
    totalBytesReceived_ += size;
}

void StatsPanel::updateRates(int frameCount, int imuCount) {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - lastUpdateTime_
    ).count();

    if (elapsed > 0) {
        double elapsedSeconds = elapsed / 1000.0;

        // Calculate FPS
        int frameDelta = frameCount - lastFrameCount_;
        currentFPS_ = frameDelta / elapsedSeconds;

        // Calculate IMU rate
        int imuDelta = imuCount - lastIMUCount_;
        currentIMURate_ = imuDelta / elapsedSeconds;

        // Calculate data rate from history
        size_t bytesInLastSecond = 0;
        auto cutoffTime = now - std::chrono::seconds(1);
        for (const auto& entry : frameHistory_) {
            if (entry.first >= cutoffTime) {
                bytesInLastSecond += entry.second;
            }
        }
        for (const auto& entry : imuHistory_) {
            if (entry.first >= cutoffTime) {
                bytesInLastSecond += entry.second;
            }
        }
        currentDataRate_ = bytesInLastSecond;
    }

    lastFrameCount_ = frameCount;
    lastIMUCount_ = imuCount;
}

std::string StatsPanel::formatDataRate(double bytesPerSecond) const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2);

    if (bytesPerSecond >= 1024 * 1024) {
        oss << (bytesPerSecond / (1024 * 1024)) << " MB/s";
    } else if (bytesPerSecond >= 1024) {
        oss << (bytesPerSecond / 1024) << " KB/s";
    } else {
        oss << bytesPerSecond << " B/s";
    }

    return oss.str();
}

void StatsPanel::cleanupHistory() {
    auto now = std::chrono::steady_clock::now();
    auto cutoffTime = now - std::chrono::seconds(60);

    // Remove old frame history entries
    while (!frameHistory_.empty() && frameHistory_.front().first < cutoffTime) {
        frameHistory_.pop_front();
    }

    // Remove old IMU history entries
    while (!imuHistory_.empty() && imuHistory_.front().first < cutoffTime) {
        imuHistory_.pop_front();
    }
}

}  // namespace ui
}  // namespace vi_slam
