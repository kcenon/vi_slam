#ifndef VI_SLAM_CONFIG_SETTINGS_HPP
#define VI_SLAM_CONFIG_SETTINGS_HPP

#include <map>
#include <string>
#include <variant>

namespace vi_slam {
namespace config {

/**
 * System settings manager.
 *
 * Manages application-wide configuration settings with:
 * - Category-based organization (Display, Performance, Network, etc.)
 * - Type-safe value storage (int, float, bool, string)
 * - Persistent storage (JSON file)
 * - Validation and default values
 */
class Settings {
public:
    enum class Category {
        Display,
        Performance,
        Network,
        Paths,
        Advanced
    };

    using Value = std::variant<int, float, bool, std::string>;

    Settings();
    ~Settings() = default;

    /**
     * Load settings from configuration file.
     *
     * @param filepath Path to configuration file (default: config.json)
     * @return true if loaded successfully
     */
    bool load(const std::string& filepath = "config.json");

    /**
     * Save settings to configuration file.
     *
     * @param filepath Path to configuration file (default: config.json)
     * @return true if saved successfully
     */
    bool save(const std::string& filepath = "config.json") const;

    /**
     * Get setting value by key.
     *
     * @param key Setting key (e.g., "display.theme")
     * @return Setting value or default if not found
     */
    Value get(const std::string& key) const;

    /**
     * Set setting value by key.
     *
     * @param key Setting key
     * @param value Setting value
     * @return true if validation passed and value was set
     */
    bool set(const std::string& key, const Value& value);

    /**
     * Get integer setting value.
     *
     * @param key Setting key
     * @param defaultValue Default value if key not found
     * @return Setting value as integer
     */
    int getInt(const std::string& key, int defaultValue = 0) const;

    /**
     * Get float setting value.
     *
     * @param key Setting key
     * @param defaultValue Default value if key not found
     * @return Setting value as float
     */
    float getFloat(const std::string& key, float defaultValue = 0.0f) const;

    /**
     * Get boolean setting value.
     *
     * @param key Setting key
     * @param defaultValue Default value if key not found
     * @return Setting value as boolean
     */
    bool getBool(const std::string& key, bool defaultValue = false) const;

    /**
     * Get string setting value.
     *
     * @param key Setting key
     * @param defaultValue Default value if key not found
     * @return Setting value as string
     */
    std::string getString(const std::string& key, const std::string& defaultValue = "") const;

    /**
     * Reset all settings to default values.
     */
    void resetToDefaults();

    /**
     * Get all settings keys in a category.
     *
     * @param category Settings category
     * @return Vector of setting keys in the category
     */
    std::vector<std::string> getKeysInCategory(Category category) const;

private:
    std::map<std::string, Value> settings_;
    std::map<std::string, Value> defaults_;

    /**
     * Initialize default settings.
     */
    void initializeDefaults();

    /**
     * Validate setting value against constraints.
     *
     * @param key Setting key
     * @param value Setting value
     * @return true if value is valid
     */
    bool validate(const std::string& key, const Value& value) const;

    /**
     * Get category from setting key.
     *
     * @param key Setting key (e.g., "display.theme")
     * @return Category enum value
     */
    static Category getCategoryFromKey(const std::string& key);
};

}  // namespace config
}  // namespace vi_slam

#endif  // VI_SLAM_CONFIG_SETTINGS_HPP
