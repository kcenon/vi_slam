#ifndef VI_SLAM_COMMON_LOGGING_HPP
#define VI_SLAM_COMMON_LOGGING_HPP

/**
 * @file logging.hpp
 * @brief Structured logging framework for VI-SLAM system
 *
 * Provides a lightweight, thread-safe logging system with:
 * - Multiple log levels (DEBUG, INFO, WARN, ERROR, FATAL)
 * - Module-based filtering
 * - Timestamp and thread ID support
 * - Configurable output destinations
 * - Compile-time log level filtering via NDEBUG
 */

#include <cstdint>
#include <memory>
#include <string>
#include <string_view>

namespace vi_slam {

/**
 * @brief Log severity levels
 *
 * Ordered from least to most severe. Setting a minimum level
 * will filter out all messages below that level.
 */
enum class LogLevel : uint8_t {
    DEBUG = 0,  ///< Detailed debugging information
    INFO = 1,   ///< General informational messages
    WARN = 2,   ///< Warning conditions
    ERROR = 3,  ///< Error conditions
    FATAL = 4,  ///< Fatal error, system may be unstable
    OFF = 5     ///< Disable all logging
};

/**
 * @brief Convert LogLevel to string representation
 * @param level The log level to convert
 * @return String representation of the level
 */
const char* logLevelToString(LogLevel level);

/**
 * @brief Parse string to LogLevel
 * @param str String representation (case-insensitive)
 * @return Corresponding LogLevel, defaults to INFO if unknown
 */
LogLevel stringToLogLevel(std::string_view str);

/**
 * @brief Logger configuration settings
 */
struct LogConfig {
    LogLevel minLevel = LogLevel::INFO;    ///< Minimum level to log
    bool showTimestamp = true;             ///< Include timestamp in output
    bool showThreadId = true;              ///< Include thread ID in output
    bool showModule = true;                ///< Include module name in output
    bool showLevel = true;                 ///< Include log level in output
    bool colorOutput = true;               ///< Use ANSI colors (if terminal)
    bool flushOnError = true;              ///< Flush output on ERROR/FATAL
};

// Forward declaration
class LoggerImpl;

/**
 * @brief Singleton logger class
 *
 * Thread-safe logging implementation with configurable output and filtering.
 *
 * Usage:
 * @code
 * Logger::instance().info("SLAM", "Initialized with {} frames", frameCount);
 * Logger::instance().error("Adapter", "Failed to load: {}", errorMsg);
 * @endcode
 */
class Logger {
public:
    /**
     * @brief Get the singleton logger instance
     * @return Reference to the global logger
     */
    static Logger& instance();

    // Deleted copy/move operations for singleton
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;
    Logger(Logger&&) = delete;
    Logger& operator=(Logger&&) = delete;

    /**
     * @brief Configure the logger
     * @param config New configuration settings
     */
    void configure(const LogConfig& config);

    /**
     * @brief Get current configuration
     * @return Current logger configuration
     */
    LogConfig getConfig() const;

    /**
     * @brief Set minimum log level
     * @param level Minimum level to log
     */
    void setLevel(LogLevel level);

    /**
     * @brief Get current minimum log level
     * @return Current minimum level
     */
    LogLevel getLevel() const;

    /**
     * @brief Enable/disable logging for a specific module
     * @param module Module name
     * @param enabled Whether to enable logging for this module
     */
    void setModuleEnabled(const std::string& module, bool enabled);

    /**
     * @brief Check if a module is enabled for logging
     * @param module Module name to check
     * @return True if module is enabled
     */
    bool isModuleEnabled(const std::string& module) const;

    /**
     * @brief Log a message with the specified level
     * @param level Log level
     * @param module Module name for filtering
     * @param message The message to log
     */
    void log(LogLevel level, const char* module, const std::string& message);

    /**
     * @brief Check if a level would be logged
     * @param level Level to check
     * @return True if messages at this level would be logged
     */
    bool isLevelEnabled(LogLevel level) const;

    // Convenience methods for each log level
    void debug(const char* module, const std::string& message);
    void info(const char* module, const std::string& message);
    void warn(const char* module, const std::string& message);
    void error(const char* module, const std::string& message);
    void fatal(const char* module, const std::string& message);

private:
    Logger();
    ~Logger();

    std::unique_ptr<LoggerImpl> impl_;
};

// Helper function for variadic formatting
namespace detail {

/**
 * @brief Format a message with arguments
 *
 * Simple formatter supporting {} placeholders.
 * Thread-safe implementation.
 */
std::string formatMessage(const char* fmt);

template <typename T, typename... Args>
std::string formatMessage(const char* fmt, T&& arg, Args&&... args) {
    std::string result;
    const char* pos = fmt;

    while (*pos) {
        if (*pos == '{' && *(pos + 1) == '}') {
            // Found placeholder, insert argument
            if constexpr (std::is_same_v<std::decay_t<T>, std::string>) {
                result += arg;
            } else if constexpr (std::is_same_v<std::decay_t<T>, const char*> ||
                                 std::is_same_v<std::decay_t<T>, char*>) {
                result += arg;
            } else if constexpr (std::is_arithmetic_v<std::decay_t<T>>) {
                result += std::to_string(arg);
            } else {
                result += "<?>";
            }
            // Recurse for remaining arguments
            return result + formatMessage(pos + 2, std::forward<Args>(args)...);
        }
        result += *pos++;
    }
    return result;
}

}  // namespace detail

}  // namespace vi_slam

/**
 * @brief Log at DEBUG level
 * @param module Module name (string literal)
 * @param fmt Format string with {} placeholders
 * @param ... Arguments to format
 */
#ifdef NDEBUG
#define LOG_DEBUG(module, fmt, ...) ((void)0)
#else
#define LOG_DEBUG(module, fmt, ...)                                  \
    do {                                                             \
        if (vi_slam::Logger::instance().isLevelEnabled(              \
                vi_slam::LogLevel::DEBUG)) {                         \
            vi_slam::Logger::instance().debug(                       \
                module, vi_slam::detail::formatMessage(fmt, ##__VA_ARGS__)); \
        }                                                            \
    } while (0)
#endif

/**
 * @brief Log at INFO level
 */
#define LOG_INFO(module, fmt, ...)                                   \
    do {                                                             \
        if (vi_slam::Logger::instance().isLevelEnabled(              \
                vi_slam::LogLevel::INFO)) {                          \
            vi_slam::Logger::instance().info(                        \
                module, vi_slam::detail::formatMessage(fmt, ##__VA_ARGS__)); \
        }                                                            \
    } while (0)

/**
 * @brief Log at WARN level
 */
#define LOG_WARN(module, fmt, ...)                                   \
    do {                                                             \
        if (vi_slam::Logger::instance().isLevelEnabled(              \
                vi_slam::LogLevel::WARN)) {                          \
            vi_slam::Logger::instance().warn(                        \
                module, vi_slam::detail::formatMessage(fmt, ##__VA_ARGS__)); \
        }                                                            \
    } while (0)

/**
 * @brief Log at ERROR level
 */
#define LOG_ERROR(module, fmt, ...)                                  \
    do {                                                             \
        if (vi_slam::Logger::instance().isLevelEnabled(              \
                vi_slam::LogLevel::ERROR)) {                         \
            vi_slam::Logger::instance().error(                       \
                module, vi_slam::detail::formatMessage(fmt, ##__VA_ARGS__)); \
        }                                                            \
    } while (0)

/**
 * @brief Log at FATAL level
 */
#define LOG_FATAL(module, fmt, ...)                                  \
    do {                                                             \
        if (vi_slam::Logger::instance().isLevelEnabled(              \
                vi_slam::LogLevel::FATAL)) {                         \
            vi_slam::Logger::instance().fatal(                       \
                module, vi_slam::detail::formatMessage(fmt, ##__VA_ARGS__)); \
        }                                                            \
    } while (0)

#endif  // VI_SLAM_COMMON_LOGGING_HPP
