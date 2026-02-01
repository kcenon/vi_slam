#ifndef VI_SLAM_UI_FRAMEWORK_PANEL_HPP
#define VI_SLAM_UI_FRAMEWORK_PANEL_HPP

#include <string>
#include <vector>
#include <map>
#include <functional>

namespace vi_slam {
namespace ui {

/**
 * SLAM framework selection and configuration panel for ImGui.
 *
 * Features:
 * - Framework selection dropdown (ORB-SLAM3, OpenVSLAM, etc.)
 * - Dynamic parameter configuration UI
 * - Apply/Reset buttons
 * - Runtime algorithm switching support
 * - Configuration persistence
 */
class FrameworkPanel {
public:
    /**
     * Parameter metadata for dynamic UI generation.
     */
    struct Parameter {
        std::string name;          // Display name
        std::string key;           // Configuration key
        enum Type {
            INT,
            FLOAT,
            BOOL,
            STRING,
            ENUM
        } type;
        std::string defaultValue;  // Default value as string
        std::string currentValue;  // Current value as string
        std::vector<std::string> enumOptions;  // For ENUM type
        std::string description;   // Help text
        float minValue;            // For INT/FLOAT (optional)
        float maxValue;            // For INT/FLOAT (optional)
    };

    /**
     * Framework information.
     */
    struct Framework {
        std::string id;                    // Internal identifier
        std::string displayName;           // Display name
        std::string description;           // Short description
        std::vector<Parameter> parameters; // Configuration parameters
    };

    /**
     * Callback type for framework selection change.
     */
    using FrameworkChangeCallback = std::function<void(const std::string& frameworkId)>;

    /**
     * Callback type for configuration apply.
     */
    using ConfigApplyCallback = std::function<void(const std::string& frameworkId,
                                                    const std::map<std::string, std::string>& config)>;

    FrameworkPanel();
    ~FrameworkPanel() = default;

    /**
     * Render the framework selection panel.
     */
    void render();

    /**
     * Update panel state (call in main loop).
     */
    void update();

    /**
     * Add a framework to the selection list.
     *
     * @param framework Framework information
     */
    void addFramework(const Framework& framework);

    /**
     * Set the currently selected framework by ID.
     *
     * @param frameworkId Framework identifier
     */
    void setSelectedFramework(const std::string& frameworkId);

    /**
     * Get the currently selected framework ID.
     *
     * @return Framework identifier
     */
    std::string getSelectedFramework() const;

    /**
     * Set framework change callback.
     *
     * @param callback Callback function
     */
    void setFrameworkChangeCallback(FrameworkChangeCallback callback);

    /**
     * Set configuration apply callback.
     *
     * @param callback Callback function
     */
    void setConfigApplyCallback(ConfigApplyCallback callback);

    /**
     * Load configuration from file.
     *
     * @param filepath Configuration file path
     * @return true if successful
     */
    bool loadConfiguration(const std::string& filepath);

    /**
     * Save configuration to file.
     *
     * @param filepath Configuration file path
     * @return true if successful
     */
    bool saveConfiguration(const std::string& filepath);

private:
    std::vector<Framework> frameworks_;
    int selectedFrameworkIndex_;
    bool configChanged_;
    std::string statusMessage_;
    float statusMessageTime_;

    FrameworkChangeCallback frameworkChangeCallback_;
    ConfigApplyCallback configApplyCallback_;

    /**
     * Render parameter input widget based on parameter type.
     *
     * @param param Parameter to render
     * @return true if value changed
     */
    bool renderParameterWidget(Parameter& param);

    /**
     * Apply configuration changes.
     */
    void applyConfiguration();

    /**
     * Reset configuration to defaults.
     */
    void resetConfiguration();

    /**
     * Get current configuration as key-value map.
     *
     * @return Configuration map
     */
    std::map<std::string, std::string> getCurrentConfiguration() const;

    /**
     * Validate parameter value.
     *
     * @param param Parameter to validate
     * @return true if valid
     */
    bool validateParameter(const Parameter& param) const;

    /**
     * Show status message temporarily.
     *
     * @param message Message to display
     * @param duration Duration in seconds
     */
    void showStatusMessage(const std::string& message, float duration = 3.0f);
};

}  // namespace ui
}  // namespace vi_slam

#endif  // VI_SLAM_UI_FRAMEWORK_PANEL_HPP
