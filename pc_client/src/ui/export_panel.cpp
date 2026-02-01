#include "ui/export_panel.hpp"
#include <sstream>
#include <chrono>
#include <thread>
#include "imgui.h"

#ifdef _WIN32
#include <windows.h>
#include <commdlg.h>
#elif __APPLE__
#include <unistd.h>
#else
#include <unistd.h>
#endif

namespace vi_slam {
namespace ui {

ExportPanel::ExportPanel()
    : selectedFormat_(ExportFormat::TUM),
      selectedScope_(ExportScope::TRAJECTORY_ONLY),
      outputPath_(""),
      statusMessageTime_(0.0f),
      exportStatus_(ExportStatus::IDLE),
      exportProgress_(0.0f),
      exportCancelled_(false) {
}

ExportPanel::~ExportPanel() {
    if (exportFuture_.valid()) {
        exportCancelled_ = true;
        exportFuture_.wait();
    }
}

void ExportPanel::render() {
    ImGui::Begin("Export Controls");

    // Format selection
    ImGui::Text("Export Format:");
    ImGui::Separator();

    const char* currentFormat = getFormatName(selectedFormat_);
    if (ImGui::BeginCombo("##format", currentFormat)) {
        for (int i = 0; i < 4; i++) {
            ExportFormat format = static_cast<ExportFormat>(i);
            bool isSelected = (selectedFormat_ == format);
            if (ImGui::Selectable(getFormatName(format), isSelected)) {
                selectedFormat_ = format;
            }
            if (isSelected) {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndCombo();
    }

    // Format description
    ImGui::Spacing();
    switch (selectedFormat_) {
        case ExportFormat::TUM:
            ImGui::TextWrapped("TUM format: timestamp tx ty tz qx qy qz qw");
            break;
        case ExportFormat::KITTI:
            ImGui::TextWrapped("KITTI format: 3x4 transformation matrices");
            break;
        case ExportFormat::EUROC:
            ImGui::TextWrapped("EuRoC format: MAV dataset format");
            break;
        case ExportFormat::CUSTOM:
            ImGui::TextWrapped("Custom format: JSON or YAML");
            break;
    }

    ImGui::Spacing();
    ImGui::Separator();

    // Scope selection
    ImGui::Text("Export Scope:");
    ImGui::Separator();

    const char* currentScope = getScopeName(selectedScope_);
    if (ImGui::BeginCombo("##scope", currentScope)) {
        for (int i = 0; i < 3; i++) {
            ExportScope scope = static_cast<ExportScope>(i);
            bool isSelected = (selectedScope_ == scope);
            if (ImGui::Selectable(getScopeName(scope), isSelected)) {
                selectedScope_ = scope;
            }
            if (isSelected) {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndCombo();
    }

    ImGui::Spacing();
    ImGui::Separator();

    // Output path selection
    ImGui::Text("Output File:");
    ImGui::Separator();

    char pathBuffer[512];
    strncpy(pathBuffer, outputPath_.c_str(), sizeof(pathBuffer) - 1);
    pathBuffer[sizeof(pathBuffer) - 1] = '\0';

    ImGui::PushItemWidth(-100);
    if (ImGui::InputText("##outputpath", pathBuffer, sizeof(pathBuffer))) {
        outputPath_ = pathBuffer;
    }
    ImGui::PopItemWidth();

    ImGui::SameLine();
    if (ImGui::Button("Browse...")) {
        if (openFileDialog()) {
            showStatusMessage("Output path selected", 2.0f);
        }
    }

    ImGui::Spacing();
    ImGui::Separator();

    // Export button and progress
    bool isExportingNow = isExporting();

    if (isExportingNow) {
        // Show progress bar
        float progress = exportProgress_.load();
        ImGui::ProgressBar(progress, ImVec2(-1, 0), nullptr);

        ImGui::SameLine();
        if (ImGui::Button("Cancel")) {
            cancelExport();
            showStatusMessage("Export cancelled", 2.0f);
        }
    } else {
        // Export button
        bool canExport = !outputPath_.empty() && validateOutputPath();
        if (!canExport) {
            ImGui::PushStyleVar(ImGuiStyleVar_Alpha, 0.5f);
        }

        if (ImGui::Button("Export", ImVec2(-1, 0))) {
            if (canExport) {
                startExport();
            }
        }

        if (!canExport) {
            ImGui::PopStyleVar();
        }

        if (!canExport && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
            ImGui::SetTooltip("Please select a valid output path");
        }
    }

    // Status message
    if (statusMessageTime_ > 0.0f) {
        ImGui::Spacing();
        ImGui::Separator();

        ImVec4 color;
        switch (exportStatus_) {
            case ExportStatus::COMPLETED:
                color = ImVec4(0.0f, 1.0f, 0.0f, 1.0f); // Green
                break;
            case ExportStatus::FAILED:
                color = ImVec4(1.0f, 0.0f, 0.0f, 1.0f); // Red
                break;
            case ExportStatus::CANCELLED:
                color = ImVec4(1.0f, 1.0f, 0.0f, 1.0f); // Yellow
                break;
            default:
                color = ImVec4(1.0f, 1.0f, 1.0f, 1.0f); // White
                break;
        }

        ImGui::TextColored(color, "%s", statusMessage_.c_str());
    }

    ImGui::End();
}

void ExportPanel::update() {
    // Update status message timer
    if (statusMessageTime_ > 0.0f) {
        statusMessageTime_ -= ImGui::GetIO().DeltaTime;
    }

    // Check export completion
    if (exportStatus_ == ExportStatus::IN_PROGRESS) {
        checkExportCompletion();
    }
}

void ExportPanel::setExportCallback(ExportCallback callback) {
    exportCallback_ = callback;
}

bool ExportPanel::isExporting() const {
    return exportStatus_ == ExportStatus::IN_PROGRESS;
}

void ExportPanel::cancelExport() {
    if (exportStatus_ == ExportStatus::IN_PROGRESS) {
        exportCancelled_ = true;
        exportStatus_ = ExportStatus::CANCELLED;
    }
}

bool ExportPanel::openFileDialog() {
#ifdef _WIN32
    // Windows file dialog
    OPENFILENAME ofn;
    char fileName[MAX_PATH] = "";

    ZeroMemory(&ofn, sizeof(ofn));
    ofn.lStructSize = sizeof(ofn);
    ofn.hwndOwner = NULL;
    ofn.lpstrFilter = "Text Files (*.txt)\0*.txt\0JSON Files (*.json)\0*.json\0All Files (*.*)\0*.*\0";
    ofn.lpstrFile = fileName;
    ofn.nMaxFile = MAX_PATH;
    ofn.Flags = OFN_EXPLORER | OFN_OVERWRITEPROMPT;
    ofn.lpstrDefExt = getFileExtension(selectedFormat_) + 1; // Skip leading dot

    if (GetSaveFileName(&ofn)) {
        outputPath_ = fileName;
        return true;
    }
    return false;
#else
    // Simplified path input for Unix-like systems
    // In a real implementation, you would use native file dialogs
    // For now, we rely on manual path input
    outputPath_ = "/tmp/slam_export" + std::string(getFileExtension(selectedFormat_));
    return true;
#endif
}

void ExportPanel::startExport() {
    if (!exportCallback_) {
        showStatusMessage("Export callback not set", 3.0f);
        exportStatus_ = ExportStatus::FAILED;
        return;
    }

    exportStatus_ = ExportStatus::IN_PROGRESS;
    exportProgress_ = 0.0f;
    exportCancelled_ = false;

    // Launch asynchronous export
    exportFuture_ = std::async(std::launch::async, [this]() {
        return exportCallback_(
            selectedFormat_,
            selectedScope_,
            outputPath_,
            exportProgress_,
            exportCancelled_
        );
    });

    showStatusMessage("Export started...", 2.0f);
}

void ExportPanel::checkExportCompletion() {
    if (!exportFuture_.valid()) {
        return;
    }

    // Check if export task is complete
    auto status = exportFuture_.wait_for(std::chrono::milliseconds(0));
    if (status == std::future_status::ready) {
        bool success = exportFuture_.get();

        if (exportCancelled_) {
            exportStatus_ = ExportStatus::CANCELLED;
            showStatusMessage("Export cancelled", 3.0f);
        } else if (success) {
            exportStatus_ = ExportStatus::COMPLETED;
            exportProgress_ = 1.0f;
            showStatusMessage("Export completed successfully!", 3.0f);
        } else {
            exportStatus_ = ExportStatus::FAILED;
            showStatusMessage("Export failed - check logs", 3.0f);
        }
    }
}

const char* ExportPanel::getFormatName(ExportFormat format) const {
    switch (format) {
        case ExportFormat::TUM:    return "TUM";
        case ExportFormat::KITTI:  return "KITTI";
        case ExportFormat::EUROC:  return "EuRoC";
        case ExportFormat::CUSTOM: return "Custom (JSON/YAML)";
        default:                   return "Unknown";
    }
}

const char* ExportPanel::getScopeName(ExportScope scope) const {
    switch (scope) {
        case ExportScope::TRAJECTORY_ONLY: return "Trajectory Only";
        case ExportScope::MAP_POINTS_ONLY: return "Map Points Only";
        case ExportScope::FULL_MAP:        return "Full Map (Trajectory + Points)";
        default:                           return "Unknown";
    }
}

const char* ExportPanel::getFileExtension(ExportFormat format) const {
    switch (format) {
        case ExportFormat::TUM:    return ".txt";
        case ExportFormat::KITTI:  return ".txt";
        case ExportFormat::EUROC:  return ".csv";
        case ExportFormat::CUSTOM: return ".json";
        default:                   return ".txt";
    }
}

void ExportPanel::showStatusMessage(const std::string& message, float duration) {
    statusMessage_ = message;
    statusMessageTime_ = duration;
}

bool ExportPanel::validateOutputPath() const {
    if (outputPath_.empty()) {
        return false;
    }

    // Basic validation - check if path looks reasonable
    // In a real implementation, you would check:
    // - Parent directory exists
    // - Write permissions
    // - Disk space

    return true;
}

}  // namespace ui
}  // namespace vi_slam
