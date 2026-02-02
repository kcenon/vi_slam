#include "config/settings.hpp"
#include <fstream>
#include <iostream>
#include <sstream>
#include <algorithm>

// Simple JSON parsing for configuration
// Note: In production, consider using a proper JSON library (nlohmann/json, RapidJSON, etc.)

namespace vi_slam {
namespace config {

Settings::Settings() {
    initializeDefaults();
    settings_ = defaults_;
}

void Settings::initializeDefaults() {
    // Display settings
    defaults_["display.theme"] = std::string("dark");
    defaults_["display.font_size"] = 14;
    defaults_["display.panel_layout"] = std::string("horizontal");
    defaults_["display.show_fps"] = true;

    // Performance settings
    defaults_["performance.fps_limit"] = 60;
    defaults_["performance.frame_buffer_size"] = 30;
    defaults_["performance.imu_buffer_size"] = 100;
    defaults_["performance.thread_count"] = 4;

    // Network settings
    defaults_["network.connection_timeout"] = 30;
    defaults_["network.retry_count"] = 3;
    defaults_["network.retry_delay"] = 1000;
    defaults_["network.auto_reconnect"] = true;

    // Paths
    defaults_["paths.data_directory"] = std::string("./data");
    defaults_["paths.export_directory"] = std::string("./exports");
    defaults_["paths.config_file"] = std::string("config.json");

    // Advanced settings
    defaults_["advanced.debug_logging"] = false;
    defaults_["advanced.developer_mode"] = false;
    defaults_["advanced.show_metrics"] = false;
    defaults_["advanced.log_level"] = std::string("info");
}

bool Settings::load(const std::string& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "Cannot open config file: " << filepath << std::endl;
        std::cerr << "Using default settings" << std::endl;
        return false;
    }

    // Simple line-by-line parsing (key=value format)
    // In production, use a proper JSON library
    std::string line;
    while (std::getline(file, line)) {
        // Skip comments and empty lines
        if (line.empty() || line[0] == '#') {
            continue;
        }

        size_t pos = line.find('=');
        if (pos == std::string::npos) {
            continue;
        }

        std::string key = line.substr(0, pos);
        std::string valueStr = line.substr(pos + 1);

        // Trim whitespace
        key.erase(0, key.find_first_not_of(" \t"));
        key.erase(key.find_last_not_of(" \t") + 1);
        valueStr.erase(0, valueStr.find_first_not_of(" \t"));
        valueStr.erase(valueStr.find_last_not_of(" \t") + 1);

        // Try to parse value based on expected type from defaults
        if (defaults_.find(key) != defaults_.end()) {
            const Value& defaultValue = defaults_[key];

            if (std::holds_alternative<int>(defaultValue)) {
                try {
                    settings_[key] = std::stoi(valueStr);
                } catch (...) {
                    std::cerr << "Invalid integer value for " << key << std::endl;
                }
            } else if (std::holds_alternative<float>(defaultValue)) {
                try {
                    settings_[key] = std::stof(valueStr);
                } catch (...) {
                    std::cerr << "Invalid float value for " << key << std::endl;
                }
            } else if (std::holds_alternative<bool>(defaultValue)) {
                settings_[key] = (valueStr == "true" || valueStr == "1");
            } else if (std::holds_alternative<std::string>(defaultValue)) {
                // Remove quotes if present
                if (valueStr.size() >= 2 && valueStr.front() == '"' && valueStr.back() == '"') {
                    valueStr = valueStr.substr(1, valueStr.size() - 2);
                }
                settings_[key] = valueStr;
            }
        }
    }

    file.close();
    std::cout << "Loaded settings from: " << filepath << std::endl;
    return true;
}

bool Settings::save(const std::string& filepath) const {
    std::ofstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "Cannot create config file: " << filepath << std::endl;
        return false;
    }

    file << "# VI-SLAM PC Client Configuration\n";
    file << "# Generated automatically - modify with care\n\n";

    // Group settings by category
    std::vector<std::string> categories = {"display", "performance", "network", "paths", "advanced"};

    for (const auto& category : categories) {
        file << "# " << category << " settings\n";

        for (const auto& [key, value] : settings_) {
            if (key.find(category + ".") == 0) {
                file << key << "=";

                if (std::holds_alternative<int>(value)) {
                    file << std::get<int>(value);
                } else if (std::holds_alternative<float>(value)) {
                    file << std::get<float>(value);
                } else if (std::holds_alternative<bool>(value)) {
                    file << (std::get<bool>(value) ? "true" : "false");
                } else if (std::holds_alternative<std::string>(value)) {
                    file << "\"" << std::get<std::string>(value) << "\"";
                }

                file << "\n";
            }
        }

        file << "\n";
    }

    file.close();
    std::cout << "Saved settings to: " << filepath << std::endl;
    return true;
}

Settings::Value Settings::get(const std::string& key) const {
    auto it = settings_.find(key);
    if (it != settings_.end()) {
        return it->second;
    }

    auto defaultIt = defaults_.find(key);
    if (defaultIt != defaults_.end()) {
        return defaultIt->second;
    }

    return std::string("");
}

bool Settings::set(const std::string& key, const Value& value) {
    if (!validate(key, value)) {
        std::cerr << "Validation failed for " << key << std::endl;
        return false;
    }

    settings_[key] = value;
    return true;
}

int Settings::getInt(const std::string& key, int defaultValue) const {
    Value value = get(key);
    if (std::holds_alternative<int>(value)) {
        return std::get<int>(value);
    }
    return defaultValue;
}

float Settings::getFloat(const std::string& key, float defaultValue) const {
    Value value = get(key);
    if (std::holds_alternative<float>(value)) {
        return std::get<float>(value);
    }
    return defaultValue;
}

bool Settings::getBool(const std::string& key, bool defaultValue) const {
    Value value = get(key);
    if (std::holds_alternative<bool>(value)) {
        return std::get<bool>(value);
    }
    return defaultValue;
}

std::string Settings::getString(const std::string& key, const std::string& defaultValue) const {
    Value value = get(key);
    if (std::holds_alternative<std::string>(value)) {
        return std::get<std::string>(value);
    }
    return defaultValue;
}

void Settings::resetToDefaults() {
    settings_ = defaults_;
    std::cout << "Settings reset to defaults" << std::endl;
}

std::vector<std::string> Settings::getKeysInCategory(Category category) const {
    std::string prefix;
    switch (category) {
        case Category::Display:
            prefix = "display.";
            break;
        case Category::Performance:
            prefix = "performance.";
            break;
        case Category::Network:
            prefix = "network.";
            break;
        case Category::Paths:
            prefix = "paths.";
            break;
        case Category::Advanced:
            prefix = "advanced.";
            break;
    }

    std::vector<std::string> keys;
    for (const auto& [key, _] : settings_) {
        if (key.find(prefix) == 0) {
            keys.push_back(key);
        }
    }

    return keys;
}

bool Settings::validate(const std::string& key, const Value& value) const {
    // Validation rules
    if (key == "performance.fps_limit") {
        int fps = std::get<int>(value);
        return fps >= 10 && fps <= 240;
    }
    if (key == "performance.frame_buffer_size") {
        int size = std::get<int>(value);
        return size >= 10 && size <= 1000;
    }
    if (key == "performance.imu_buffer_size") {
        int size = std::get<int>(value);
        return size >= 10 && size <= 10000;
    }
    if (key == "performance.thread_count") {
        int threads = std::get<int>(value);
        return threads >= 1 && threads <= 32;
    }
    if (key == "network.connection_timeout") {
        int timeout = std::get<int>(value);
        return timeout >= 5 && timeout <= 300;
    }
    if (key == "network.retry_count") {
        int retries = std::get<int>(value);
        return retries >= 0 && retries <= 10;
    }
    if (key == "network.retry_delay") {
        int delay = std::get<int>(value);
        return delay >= 100 && delay <= 10000;
    }
    if (key == "display.font_size") {
        int size = std::get<int>(value);
        return size >= 8 && size <= 32;
    }

    // Default: validation passes
    return true;
}

Settings::Category Settings::getCategoryFromKey(const std::string& key) {
    if (key.find("display.") == 0) return Category::Display;
    if (key.find("performance.") == 0) return Category::Performance;
    if (key.find("network.") == 0) return Category::Network;
    if (key.find("paths.") == 0) return Category::Paths;
    if (key.find("advanced.") == 0) return Category::Advanced;
    return Category::Display;
}

}  // namespace config
}  // namespace vi_slam
