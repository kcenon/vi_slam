#ifndef VI_SLAM_UI_CONFIG_PANEL_HPP
#define VI_SLAM_UI_CONFIG_PANEL_HPP

#include <functional>
#include <memory>
#include "config/settings.hpp"

namespace vi_slam {
namespace ui {

/**
 * Configuration panel for system settings.
 *
 * Provides UI for:
 * - Display settings (theme, font size, layout)
 * - Performance settings (FPS limit, buffer sizes, threading)
 * - Network settings (timeout, retry, auto-reconnect)
 * - Path settings (data directory, export directory)
 * - Advanced settings (debug logging, developer options)
 *
 * Features:
 * - Category-based tabs
 * - Input validation with error messages
 * - Apply/Cancel/Reset to defaults buttons
 * - Settings persistence across restarts
 */
class ConfigPanel {
public:
    using ApplyCallback = std::function<void(const config::Settings&)>;

    ConfigPanel();
    ~ConfigPanel() = default;

    /**
     * Render the configuration panel.
     */
    void render();

    /**
     * Update panel state (call in main loop).
     */
    void update();

    /**
     * Set callback for when settings are applied.
     *
     * @param callback Function to call when Apply button is clicked
     */
    void setApplyCallback(ApplyCallback callback);

    /**
     * Get current settings.
     *
     * @return Current settings object
     */
    config::Settings& getSettings();

    /**
     * Load settings from file.
     *
     * @param filepath Path to configuration file
     * @return true if loaded successfully
     */
    bool loadSettings(const std::string& filepath = "config.json");

    /**
     * Save current settings to file.
     *
     * @param filepath Path to configuration file
     * @return true if saved successfully
     */
    bool saveSettings(const std::string& filepath = "config.json");

private:
    config::Settings settings_;
    config::Settings pendingSettings_;  // Staging area for changes
    ApplyCallback applyCallback_;

    bool hasUnsavedChanges_;
    bool showValidationError_;
    std::string validationErrorMessage_;

    /**
     * Render display settings tab.
     */
    void renderDisplaySettings();

    /**
     * Render performance settings tab.
     */
    void renderPerformanceSettings();

    /**
     * Render network settings tab.
     */
    void renderNetworkSettings();

    /**
     * Render path settings tab.
     */
    void renderPathSettings();

    /**
     * Render advanced settings tab.
     */
    void renderAdvancedSettings();

    /**
     * Render action buttons (Apply, Cancel, Reset).
     */
    void renderActionButtons();

    /**
     * Apply pending changes to active settings.
     */
    void applyChanges();

    /**
     * Cancel pending changes and revert to active settings.
     */
    void cancelChanges();

    /**
     * Reset all settings to defaults.
     */
    void resetToDefaults();

    /**
     * Check if there are unsaved changes.
     *
     * @return true if pending settings differ from active settings
     */
    bool checkUnsavedChanges();

    /**
     * Show validation error message.
     *
     * @param message Error message to display
     */
    void showError(const std::string& message);
};

}  // namespace ui
}  // namespace vi_slam

#endif  // VI_SLAM_UI_CONFIG_PANEL_HPP
