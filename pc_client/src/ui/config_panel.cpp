#include "ui/config_panel.hpp"
#include "imgui.h"
#include <iostream>

namespace vi_slam {
namespace ui {

ConfigPanel::ConfigPanel()
    : hasUnsavedChanges_(false),
      showValidationError_(false) {
    // Load settings from file or use defaults
    if (!settings_.load()) {
        std::cout << "Using default settings" << std::endl;
    }
    pendingSettings_ = settings_;
}

void ConfigPanel::render() {
    ImGui::Begin("Configuration", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

    // Tab bar for categories
    if (ImGui::BeginTabBar("ConfigTabs")) {
        if (ImGui::BeginTabItem("Display")) {
            renderDisplaySettings();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Performance")) {
            renderPerformanceSettings();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Network")) {
            renderNetworkSettings();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Paths")) {
            renderPathSettings();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Advanced")) {
            renderAdvancedSettings();
            ImGui::EndTabItem();
        }

        ImGui::EndTabBar();
    }

    ImGui::Separator();

    // Show validation error if any
    if (showValidationError_) {
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.3f, 0.3f, 1.0f));
        ImGui::TextWrapped("Error: %s", validationErrorMessage_.c_str());
        ImGui::PopStyleColor();
        ImGui::Separator();
    }

    // Action buttons
    renderActionButtons();

    ImGui::End();
}

void ConfigPanel::update() {
    hasUnsavedChanges_ = checkUnsavedChanges();
}

void ConfigPanel::setApplyCallback(ApplyCallback callback) {
    applyCallback_ = callback;
}

config::Settings& ConfigPanel::getSettings() {
    return settings_;
}

bool ConfigPanel::loadSettings(const std::string& filepath) {
    if (settings_.load(filepath)) {
        pendingSettings_ = settings_;
        return true;
    }
    return false;
}

bool ConfigPanel::saveSettings(const std::string& filepath) {
    return settings_.save(filepath);
}

void ConfigPanel::renderDisplaySettings() {
    // Theme selection
    const char* themes[] = {"Dark", "Light", "Classic"};
    std::string currentTheme = pendingSettings_.getString("display.theme");
    int themeIndex = 0;
    if (currentTheme == "light") themeIndex = 1;
    else if (currentTheme == "classic") themeIndex = 2;

    if (ImGui::Combo("Theme", &themeIndex, themes, IM_ARRAYSIZE(themes))) {
        std::string themeValue = themeIndex == 0 ? "dark" : (themeIndex == 1 ? "light" : "classic");
        pendingSettings_.set("display.theme", themeValue);
    }
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("UI color scheme");
    }

    // Font size
    int fontSize = pendingSettings_.getInt("display.font_size");
    if (ImGui::SliderInt("Font Size", &fontSize, 8, 32)) {
        pendingSettings_.set("display.font_size", fontSize);
    }
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Size of UI text (8-32)");
    }

    // Panel layout
    const char* layouts[] = {"Horizontal", "Vertical", "Grid"};
    std::string currentLayout = pendingSettings_.getString("display.panel_layout");
    int layoutIndex = 0;
    if (currentLayout == "vertical") layoutIndex = 1;
    else if (currentLayout == "grid") layoutIndex = 2;

    if (ImGui::Combo("Panel Layout", &layoutIndex, layouts, IM_ARRAYSIZE(layouts))) {
        std::string layoutValue = layoutIndex == 0 ? "horizontal" : (layoutIndex == 1 ? "vertical" : "grid");
        pendingSettings_.set("display.panel_layout", layoutValue);
    }
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Arrangement of dashboard panels");
    }

    // Show FPS
    bool showFps = pendingSettings_.getBool("display.show_fps");
    if (ImGui::Checkbox("Show FPS Counter", &showFps)) {
        pendingSettings_.set("display.show_fps", showFps);
    }
}

void ConfigPanel::renderPerformanceSettings() {
    // FPS limit
    int fpsLimit = pendingSettings_.getInt("performance.fps_limit");
    if (ImGui::SliderInt("FPS Limit", &fpsLimit, 10, 240)) {
        if (pendingSettings_.set("performance.fps_limit", fpsLimit)) {
            showValidationError_ = false;
        } else {
            showError("FPS limit must be between 10 and 240");
        }
    }
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Maximum frames per second (10-240)");
    }

    // Frame buffer size
    int frameBufferSize = pendingSettings_.getInt("performance.frame_buffer_size");
    if (ImGui::InputInt("Frame Buffer Size", &frameBufferSize)) {
        if (pendingSettings_.set("performance.frame_buffer_size", frameBufferSize)) {
            showValidationError_ = false;
        } else {
            showError("Frame buffer size must be between 10 and 1000");
        }
    }
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Number of frames to buffer (10-1000)");
    }

    // IMU buffer size
    int imuBufferSize = pendingSettings_.getInt("performance.imu_buffer_size");
    if (ImGui::InputInt("IMU Buffer Size", &imuBufferSize)) {
        if (pendingSettings_.set("performance.imu_buffer_size", imuBufferSize)) {
            showValidationError_ = false;
        } else {
            showError("IMU buffer size must be between 10 and 10000");
        }
    }
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Number of IMU samples to buffer (10-10000)");
    }

    // Thread count
    int threadCount = pendingSettings_.getInt("performance.thread_count");
    if (ImGui::SliderInt("Thread Count", &threadCount, 1, 32)) {
        if (pendingSettings_.set("performance.thread_count", threadCount)) {
            showValidationError_ = false;
        } else {
            showError("Thread count must be between 1 and 32");
        }
    }
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Number of processing threads (1-32)");
    }
}

void ConfigPanel::renderNetworkSettings() {
    // Connection timeout
    int connectionTimeout = pendingSettings_.getInt("network.connection_timeout");
    if (ImGui::InputInt("Connection Timeout (s)", &connectionTimeout)) {
        if (pendingSettings_.set("network.connection_timeout", connectionTimeout)) {
            showValidationError_ = false;
        } else {
            showError("Connection timeout must be between 5 and 300 seconds");
        }
    }
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Time to wait for connection (5-300 seconds)");
    }

    // Retry count
    int retryCount = pendingSettings_.getInt("network.retry_count");
    if (ImGui::InputInt("Retry Count", &retryCount)) {
        if (pendingSettings_.set("network.retry_count", retryCount)) {
            showValidationError_ = false;
        } else {
            showError("Retry count must be between 0 and 10");
        }
    }
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Number of connection retries (0-10)");
    }

    // Retry delay
    int retryDelay = pendingSettings_.getInt("network.retry_delay");
    if (ImGui::InputInt("Retry Delay (ms)", &retryDelay)) {
        if (pendingSettings_.set("network.retry_delay", retryDelay)) {
            showValidationError_ = false;
        } else {
            showError("Retry delay must be between 100 and 10000 milliseconds");
        }
    }
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Delay between retries (100-10000 ms)");
    }

    // Auto-reconnect
    bool autoReconnect = pendingSettings_.getBool("network.auto_reconnect");
    if (ImGui::Checkbox("Auto-Reconnect", &autoReconnect)) {
        pendingSettings_.set("network.auto_reconnect", autoReconnect);
    }
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Automatically reconnect when disconnected");
    }
}

void ConfigPanel::renderPathSettings() {
    char dataDir[256];
    char exportDir[256];

    std::string dataDirStr = pendingSettings_.getString("paths.data_directory");
    std::string exportDirStr = pendingSettings_.getString("paths.export_directory");

    strncpy(dataDir, dataDirStr.c_str(), sizeof(dataDir) - 1);
    strncpy(exportDir, exportDirStr.c_str(), sizeof(exportDir) - 1);

    // Data directory
    if (ImGui::InputText("Data Directory", dataDir, sizeof(dataDir))) {
        pendingSettings_.set("paths.data_directory", std::string(dataDir));
    }
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Directory for storing received data");
    }

    // Export directory
    if (ImGui::InputText("Export Directory", exportDir, sizeof(exportDir))) {
        pendingSettings_.set("paths.export_directory", std::string(exportDir));
    }
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Directory for exported results");
    }
}

void ConfigPanel::renderAdvancedSettings() {
    // Debug logging
    bool debugLogging = pendingSettings_.getBool("advanced.debug_logging");
    if (ImGui::Checkbox("Debug Logging", &debugLogging)) {
        pendingSettings_.set("advanced.debug_logging", debugLogging);
    }
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Enable verbose debug output");
    }

    // Developer mode
    bool developerMode = pendingSettings_.getBool("advanced.developer_mode");
    if (ImGui::Checkbox("Developer Mode", &developerMode)) {
        pendingSettings_.set("advanced.developer_mode", developerMode);
    }
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Enable developer tools and options");
    }

    // Show metrics
    bool showMetrics = pendingSettings_.getBool("advanced.show_metrics");
    if (ImGui::Checkbox("Show Performance Metrics", &showMetrics)) {
        pendingSettings_.set("advanced.show_metrics", showMetrics);
    }
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Display detailed performance metrics");
    }

    // Log level
    const char* logLevels[] = {"debug", "info", "warning", "error"};
    std::string currentLogLevel = pendingSettings_.getString("advanced.log_level");
    int logLevelIndex = 1;  // Default: info
    if (currentLogLevel == "debug") logLevelIndex = 0;
    else if (currentLogLevel == "warning") logLevelIndex = 2;
    else if (currentLogLevel == "error") logLevelIndex = 3;

    if (ImGui::Combo("Log Level", &logLevelIndex, logLevels, IM_ARRAYSIZE(logLevels))) {
        pendingSettings_.set("advanced.log_level", std::string(logLevels[logLevelIndex]));
    }
    ImGui::SameLine();
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Minimum log level to display");
    }
}

void ConfigPanel::renderActionButtons() {
    // Apply button
    if (hasUnsavedChanges_) {
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.8f, 0.2f, 1.0f));
    }

    if (ImGui::Button("Apply", ImVec2(100, 0))) {
        applyChanges();
    }

    if (hasUnsavedChanges_) {
        ImGui::PopStyleColor();
    }

    ImGui::SameLine();

    // Cancel button
    if (ImGui::Button("Cancel", ImVec2(100, 0))) {
        cancelChanges();
    }

    ImGui::SameLine();

    // Reset to defaults button
    if (ImGui::Button("Reset to Defaults", ImVec2(150, 0))) {
        resetToDefaults();
    }

    // Show unsaved changes indicator
    if (hasUnsavedChanges_) {
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.0f, 1.0f), "Unsaved changes");
    }
}

void ConfigPanel::applyChanges() {
    settings_ = pendingSettings_;
    settings_.save();
    hasUnsavedChanges_ = false;
    showValidationError_ = false;

    std::cout << "Settings applied and saved" << std::endl;

    if (applyCallback_) {
        applyCallback_(settings_);
    }
}

void ConfigPanel::cancelChanges() {
    pendingSettings_ = settings_;
    hasUnsavedChanges_ = false;
    showValidationError_ = false;

    std::cout << "Changes cancelled" << std::endl;
}

void ConfigPanel::resetToDefaults() {
    pendingSettings_.resetToDefaults();
    std::cout << "Settings reset to defaults (not applied yet)" << std::endl;
}

bool ConfigPanel::checkUnsavedChanges() {
    // Compare all settings keys
    auto categories = {config::Settings::Category::Display,
                      config::Settings::Category::Performance,
                      config::Settings::Category::Network,
                      config::Settings::Category::Paths,
                      config::Settings::Category::Advanced};

    for (auto category : categories) {
        auto keys = settings_.getKeysInCategory(category);
        for (const auto& key : keys) {
            auto currentValue = settings_.get(key);
            auto pendingValue = pendingSettings_.get(key);

            // Compare values (variant comparison)
            if (currentValue.index() != pendingValue.index()) {
                return true;
            }

            if (std::holds_alternative<int>(currentValue)) {
                if (std::get<int>(currentValue) != std::get<int>(pendingValue)) {
                    return true;
                }
            } else if (std::holds_alternative<float>(currentValue)) {
                if (std::get<float>(currentValue) != std::get<float>(pendingValue)) {
                    return true;
                }
            } else if (std::holds_alternative<bool>(currentValue)) {
                if (std::get<bool>(currentValue) != std::get<bool>(pendingValue)) {
                    return true;
                }
            } else if (std::holds_alternative<std::string>(currentValue)) {
                if (std::get<std::string>(currentValue) != std::get<std::string>(pendingValue)) {
                    return true;
                }
            }
        }
    }

    return false;
}

void ConfigPanel::showError(const std::string& message) {
    showValidationError_ = true;
    validationErrorMessage_ = message;
}

}  // namespace ui
}  // namespace vi_slam
