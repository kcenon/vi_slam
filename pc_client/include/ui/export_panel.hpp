#ifndef VI_SLAM_UI_EXPORT_PANEL_HPP
#define VI_SLAM_UI_EXPORT_PANEL_HPP

#include <string>
#include <functional>
#include <vector>
#include <atomic>
#include <future>

namespace vi_slam {
namespace ui {

/**
 * Export controls panel for exporting SLAM data and results.
 *
 * Features:
 * - Format selection (TUM, KITTI, EuRoC, custom)
 * - Export scope selection (trajectory only, full map, etc.)
 * - File browser/path selection
 * - Export progress indicator
 * - Success/error notifications
 * - Asynchronous export (non-blocking UI)
 */
class ExportPanel {
public:
    /**
     * Export format enum.
     */
    enum class ExportFormat {
        TUM,      // Timestamp tx ty tz qx qy qz qw
        KITTI,    // 3x4 transformation matrices
        EUROC,    // EuRoC MAV dataset format
        CUSTOM    // JSON or YAML
    };

    /**
     * Export scope enum.
     */
    enum class ExportScope {
        TRAJECTORY_ONLY,  // Trajectory poses only
        MAP_POINTS_ONLY,  // 3D map points only
        FULL_MAP          // Both trajectory and map points
    };

    /**
     * Export status enum.
     */
    enum class ExportStatus {
        IDLE,
        IN_PROGRESS,
        COMPLETED,
        FAILED,
        CANCELLED
    };

    /**
     * Callback type for export operation.
     * Returns true if export successful, false otherwise.
     */
    using ExportCallback = std::function<bool(ExportFormat format,
                                               ExportScope scope,
                                               const std::string& filepath,
                                               std::atomic<float>& progress,
                                               std::atomic<bool>& cancelled)>;

    ExportPanel();
    ~ExportPanel();

    /**
     * Render the export panel.
     */
    void render();

    /**
     * Update panel state (call in main loop).
     */
    void update();

    /**
     * Set export callback.
     *
     * @param callback Callback function for export operation
     */
    void setExportCallback(ExportCallback callback);

    /**
     * Check if export is currently in progress.
     *
     * @return true if exporting
     */
    bool isExporting() const;

    /**
     * Cancel current export operation.
     */
    void cancelExport();

private:
    ExportFormat selectedFormat_;
    ExportScope selectedScope_;
    std::string outputPath_;
    std::string statusMessage_;
    float statusMessageTime_;

    ExportStatus exportStatus_;
    std::atomic<float> exportProgress_;
    std::atomic<bool> exportCancelled_;
    std::future<bool> exportFuture_;

    ExportCallback exportCallback_;

    /**
     * Open file dialog for selecting output path.
     *
     * @return true if path selected
     */
    bool openFileDialog();

    /**
     * Start asynchronous export operation.
     */
    void startExport();

    /**
     * Check export task completion and update status.
     */
    void checkExportCompletion();

    /**
     * Get format name string.
     *
     * @param format Export format
     * @return Format name
     */
    const char* getFormatName(ExportFormat format) const;

    /**
     * Get scope name string.
     *
     * @param scope Export scope
     * @return Scope name
     */
    const char* getScopeName(ExportScope scope) const;

    /**
     * Get file extension for format.
     *
     * @param format Export format
     * @return File extension (e.g., ".txt", ".json")
     */
    const char* getFileExtension(ExportFormat format) const;

    /**
     * Show status message temporarily.
     *
     * @param message Message to display
     * @param duration Duration in seconds
     */
    void showStatusMessage(const std::string& message, float duration = 3.0f);

    /**
     * Validate output path.
     *
     * @return true if path is valid
     */
    bool validateOutputPath() const;
};

}  // namespace ui
}  // namespace vi_slam

#endif  // VI_SLAM_UI_EXPORT_PANEL_HPP
