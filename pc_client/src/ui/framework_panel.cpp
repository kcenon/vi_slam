#include "ui/framework_panel.hpp"
#include <sstream>
#include <fstream>
#include <algorithm>
#include <cmath>
#include "imgui.h"

namespace vi_slam {
namespace ui {

FrameworkPanel::FrameworkPanel()
    : selectedFrameworkIndex_(0),
      configChanged_(false),
      statusMessageTime_(0.0f) {
    // Initialize with default frameworks
    // In a real implementation, these would be loaded from backend
    Framework orbslam3;
    orbslam3.id = "orbslam3";
    orbslam3.displayName = "ORB-SLAM3";
    orbslam3.description = "Feature-based monocular/stereo/RGB-D SLAM";
    orbslam3.parameters = {
        {"Features Per Image", "nFeatures", Parameter::INT, "1000", "1000", {},
         "Number of features to extract per image", 100.0f, 5000.0f},
        {"Scale Factor", "scaleFactor", Parameter::FLOAT, "1.2", "1.2", {},
         "Scale factor between pyramid levels", 1.1f, 2.0f},
        {"Pyramid Levels", "nLevels", Parameter::INT, "8", "8", {},
         "Number of pyramid levels", 1.0f, 12.0f},
        {"Feature Type", "featureType", Parameter::ENUM, "ORB", "ORB",
         {"ORB", "SIFT", "SURF"}, "Feature detector type"},
        {"Use Viewer", "useViewer", Parameter::BOOL, "true", "true", {},
         "Enable 3D visualization"}
    };
    addFramework(orbslam3);

    Framework openvslam;
    openvslam.id = "openvslam";
    openvslam.displayName = "OpenVSLAM";
    openvslam.description = "Versatile visual SLAM framework";
    openvslam.parameters = {
        {"Max Features", "maxFeatures", Parameter::INT, "2000", "2000", {},
         "Maximum number of features", 500.0f, 10000.0f},
        {"Tracking Quality", "trackingQuality", Parameter::ENUM, "HIGH", "HIGH",
         {"LOW", "MEDIUM", "HIGH"}, "Tracking quality preset"},
        {"Enable Loop Closure", "enableLoopClosure", Parameter::BOOL, "true", "true", {},
         "Enable loop closure detection"},
        {"Vocabulary Path", "vocabPath", Parameter::STRING, "./vocab.dbow2", "./vocab.dbow2", {},
         "Path to vocabulary file"}
    };
    addFramework(openvslam);
}

void FrameworkPanel::render() {
    ImGui::Begin("SLAM Framework Selection");

    if (frameworks_.empty()) {
        ImGui::TextDisabled("No frameworks available");
        ImGui::End();
        return;
    }

    // Framework selection dropdown
    ImGui::Text("Select Framework:");
    ImGui::Separator();

    const char* currentFramework = frameworks_[selectedFrameworkIndex_].displayName.c_str();
    if (ImGui::BeginCombo("##framework", currentFramework)) {
        for (size_t i = 0; i < frameworks_.size(); i++) {
            bool isSelected = (selectedFrameworkIndex_ == static_cast<int>(i));
            if (ImGui::Selectable(frameworks_[i].displayName.c_str(), isSelected)) {
                if (selectedFrameworkIndex_ != static_cast<int>(i)) {
                    selectedFrameworkIndex_ = static_cast<int>(i);
                    configChanged_ = true;
                    if (frameworkChangeCallback_) {
                        frameworkChangeCallback_(frameworks_[i].id);
                    }
                    showStatusMessage("Framework changed to " + frameworks_[i].displayName);
                }
            }
            if (isSelected) {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndCombo();
    }

    // Framework description
    ImGui::TextWrapped("%s", frameworks_[selectedFrameworkIndex_].description.c_str());
    ImGui::Separator();

    // Parameters section
    ImGui::Text("Configuration Parameters:");
    ImGui::Separator();

    // Render parameter widgets
    bool anyChanged = false;
    auto& params = frameworks_[selectedFrameworkIndex_].parameters;
    for (auto& param : params) {
        if (renderParameterWidget(param)) {
            anyChanged = true;
            configChanged_ = true;
        }
    }

    ImGui::Separator();

    // Action buttons
    ImGui::BeginDisabled(!configChanged_);
    if (ImGui::Button("Apply")) {
        applyConfiguration();
    }
    ImGui::EndDisabled();

    ImGui::SameLine();
    if (ImGui::Button("Reset to Defaults")) {
        resetConfiguration();
    }

    // Status message
    if (statusMessageTime_ > 0.0f) {
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "%s", statusMessage_.c_str());
    }

    ImGui::Separator();

    // Configuration persistence
    ImGui::Text("Configuration File:");
    if (ImGui::Button("Load Configuration")) {
        if (loadConfiguration("slam_config.json")) {
            showStatusMessage("Configuration loaded");
        } else {
            showStatusMessage("Failed to load configuration");
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("Save Configuration")) {
        if (saveConfiguration("slam_config.json")) {
            showStatusMessage("Configuration saved");
        } else {
            showStatusMessage("Failed to save configuration");
        }
    }

    ImGui::End();
}

void FrameworkPanel::update() {
    // Update status message timer
    if (statusMessageTime_ > 0.0f) {
        statusMessageTime_ -= ImGui::GetIO().DeltaTime;
        if (statusMessageTime_ < 0.0f) {
            statusMessageTime_ = 0.0f;
        }
    }
}

void FrameworkPanel::addFramework(const Framework& framework) {
    frameworks_.push_back(framework);
}

void FrameworkPanel::setSelectedFramework(const std::string& frameworkId) {
    for (size_t i = 0; i < frameworks_.size(); i++) {
        if (frameworks_[i].id == frameworkId) {
            selectedFrameworkIndex_ = static_cast<int>(i);
            configChanged_ = false;
            break;
        }
    }
}

std::string FrameworkPanel::getSelectedFramework() const {
    if (selectedFrameworkIndex_ >= 0 &&
        selectedFrameworkIndex_ < static_cast<int>(frameworks_.size())) {
        return frameworks_[selectedFrameworkIndex_].id;
    }
    return "";
}

void FrameworkPanel::setFrameworkChangeCallback(FrameworkChangeCallback callback) {
    frameworkChangeCallback_ = callback;
}

void FrameworkPanel::setConfigApplyCallback(ConfigApplyCallback callback) {
    configApplyCallback_ = callback;
}

bool FrameworkPanel::renderParameterWidget(Parameter& param) {
    bool changed = false;
    ImGui::PushID(param.key.c_str());

    // Parameter label
    ImGui::Text("%s:", param.name.c_str());
    if (!param.description.empty() && ImGui::IsItemHovered()) {
        ImGui::SetTooltip("%s", param.description.c_str());
    }
    ImGui::SameLine(200);

    // Render appropriate widget based on type
    switch (param.type) {
        case Parameter::INT: {
            int value = std::stoi(param.currentValue);
            if (ImGui::SliderInt("##value", &value,
                static_cast<int>(param.minValue),
                static_cast<int>(param.maxValue))) {
                param.currentValue = std::to_string(value);
                changed = true;
            }
            break;
        }
        case Parameter::FLOAT: {
            float value = std::stof(param.currentValue);
            if (ImGui::SliderFloat("##value", &value, param.minValue, param.maxValue, "%.2f")) {
                param.currentValue = std::to_string(value);
                changed = true;
            }
            break;
        }
        case Parameter::BOOL: {
            bool value = (param.currentValue == "true");
            if (ImGui::Checkbox("##value", &value)) {
                param.currentValue = value ? "true" : "false";
                changed = true;
            }
            break;
        }
        case Parameter::STRING: {
            char buffer[256];
            strncpy(buffer, param.currentValue.c_str(), sizeof(buffer) - 1);
            buffer[sizeof(buffer) - 1] = '\0';
            if (ImGui::InputText("##value", buffer, sizeof(buffer))) {
                param.currentValue = buffer;
                changed = true;
            }
            break;
        }
        case Parameter::ENUM: {
            const char* currentOption = param.currentValue.c_str();
            if (ImGui::BeginCombo("##value", currentOption)) {
                for (const auto& option : param.enumOptions) {
                    bool isSelected = (param.currentValue == option);
                    if (ImGui::Selectable(option.c_str(), isSelected)) {
                        param.currentValue = option;
                        changed = true;
                    }
                    if (isSelected) {
                        ImGui::SetItemDefaultFocus();
                    }
                }
                ImGui::EndCombo();
            }
            break;
        }
    }

    ImGui::PopID();
    return changed;
}

void FrameworkPanel::applyConfiguration() {
    if (!configChanged_) {
        return;
    }

    // Validate all parameters
    auto& params = frameworks_[selectedFrameworkIndex_].parameters;
    for (const auto& param : params) {
        if (!validateParameter(param)) {
            showStatusMessage("Invalid parameter: " + param.name);
            return;
        }
    }

    // Get current configuration
    auto config = getCurrentConfiguration();

    // Apply configuration through callback
    if (configApplyCallback_) {
        configApplyCallback_(frameworks_[selectedFrameworkIndex_].id, config);
    }

    configChanged_ = false;
    showStatusMessage("Configuration applied successfully");
}

void FrameworkPanel::resetConfiguration() {
    auto& params = frameworks_[selectedFrameworkIndex_].parameters;
    for (auto& param : params) {
        param.currentValue = param.defaultValue;
    }
    configChanged_ = true;
    showStatusMessage("Configuration reset to defaults");
}

std::map<std::string, std::string> FrameworkPanel::getCurrentConfiguration() const {
    std::map<std::string, std::string> config;
    const auto& params = frameworks_[selectedFrameworkIndex_].parameters;
    for (const auto& param : params) {
        config[param.key] = param.currentValue;
    }
    return config;
}

bool FrameworkPanel::validateParameter(const Parameter& param) const {
    try {
        switch (param.type) {
            case Parameter::INT: {
                int value = std::stoi(param.currentValue);
                return value >= param.minValue && value <= param.maxValue;
            }
            case Parameter::FLOAT: {
                float value = std::stof(param.currentValue);
                return value >= param.minValue && value <= param.maxValue;
            }
            case Parameter::BOOL:
                return param.currentValue == "true" || param.currentValue == "false";
            case Parameter::STRING:
                return !param.currentValue.empty();
            case Parameter::ENUM:
                return std::find(param.enumOptions.begin(), param.enumOptions.end(),
                                param.currentValue) != param.enumOptions.end();
        }
    } catch (...) {
        return false;
    }
    return true;
}

void FrameworkPanel::showStatusMessage(const std::string& message, float duration) {
    statusMessage_ = message;
    statusMessageTime_ = duration;
}

bool FrameworkPanel::loadConfiguration(const std::string& filepath) {
    // Placeholder implementation
    // In a real system, this would parse a JSON configuration file
    std::ifstream file(filepath);
    if (!file.is_open()) {
        return false;
    }

    // Simple implementation - would use a JSON library in production
    // For now, just indicate success
    file.close();
    return true;
}

bool FrameworkPanel::saveConfiguration(const std::string& filepath) {
    // Placeholder implementation
    // In a real system, this would write a JSON configuration file
    std::ofstream file(filepath);
    if (!file.is_open()) {
        return false;
    }

    // Simple implementation - would use a JSON library in production
    auto config = getCurrentConfiguration();
    file << "{\n";
    file << "  \"framework\": \"" << frameworks_[selectedFrameworkIndex_].id << "\",\n";
    file << "  \"parameters\": {\n";

    bool first = true;
    for (const auto& [key, value] : config) {
        if (!first) file << ",\n";
        file << "    \"" << key << "\": \"" << value << "\"";
        first = false;
    }

    file << "\n  }\n";
    file << "}\n";

    file.close();
    return true;
}

}  // namespace ui
}  // namespace vi_slam
