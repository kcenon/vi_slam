/**
 * @file logging.cpp
 * @brief Implementation of the structured logging framework
 */

#include "common/logging.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <thread>
#include <unordered_map>
#include <unordered_set>

#ifndef _WIN32
#include <unistd.h>
#endif

namespace vi_slam {

const char* logLevelToString(LogLevel level) {
    switch (level) {
        case LogLevel::DEBUG:
            return "DEBUG";
        case LogLevel::INFO:
            return "INFO";
        case LogLevel::WARN:
            return "WARN";
        case LogLevel::ERROR:
            return "ERROR";
        case LogLevel::FATAL:
            return "FATAL";
        case LogLevel::OFF:
            return "OFF";
        default:
            return "UNKNOWN";
    }
}

LogLevel stringToLogLevel(std::string_view str) {
    std::string lower;
    lower.reserve(str.size());
    for (char c : str) {
        lower += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }

    if (lower == "debug")
        return LogLevel::DEBUG;
    if (lower == "info")
        return LogLevel::INFO;
    if (lower == "warn" || lower == "warning")
        return LogLevel::WARN;
    if (lower == "error")
        return LogLevel::ERROR;
    if (lower == "fatal")
        return LogLevel::FATAL;
    if (lower == "off")
        return LogLevel::OFF;

    return LogLevel::INFO;  // Default
}

/**
 * @brief Internal implementation of the Logger
 *
 * Hides implementation details from the header to reduce compile-time
 * dependencies and improve encapsulation.
 */
class LoggerImpl {
public:
    LoggerImpl() : config_{} {
        // Check if stdout is a terminal for color support
        isTerminal_ = isatty(fileno(stderr)) != 0;
    }

    void configure(const LogConfig& config) {
        std::lock_guard<std::mutex> lock(mutex_);
        config_ = config;
    }

    LogConfig getConfig() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return config_;
    }

    void setLevel(LogLevel level) {
        std::lock_guard<std::mutex> lock(mutex_);
        config_.minLevel = level;
    }

    LogLevel getLevel() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return config_.minLevel;
    }

    void setModuleEnabled(const std::string& module, bool enabled) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (enabled) {
            disabledModules_.erase(module);
        } else {
            disabledModules_.insert(module);
        }
    }

    bool isModuleEnabled(const std::string& module) const {
        std::lock_guard<std::mutex> lock(mutex_);
        return disabledModules_.find(module) == disabledModules_.end();
    }

    bool isLevelEnabled(LogLevel level) const {
        std::lock_guard<std::mutex> lock(mutex_);
        return level >= config_.minLevel;
    }

    void log(LogLevel level, const char* module, const std::string& message) {
        std::lock_guard<std::mutex> lock(mutex_);

        // Check if level is enabled
        if (level < config_.minLevel) {
            return;
        }

        // Check if module is enabled
        if (disabledModules_.find(module) != disabledModules_.end()) {
            return;
        }

        // Build the log line
        std::ostringstream oss;

        // Color codes
        const char* colorStart = "";
        const char* colorEnd = "";
        if (config_.colorOutput && isTerminal_) {
            colorEnd = "\033[0m";
            switch (level) {
                case LogLevel::DEBUG:
                    colorStart = "\033[36m";  // Cyan
                    break;
                case LogLevel::INFO:
                    colorStart = "\033[32m";  // Green
                    break;
                case LogLevel::WARN:
                    colorStart = "\033[33m";  // Yellow
                    break;
                case LogLevel::ERROR:
                    colorStart = "\033[31m";  // Red
                    break;
                case LogLevel::FATAL:
                    colorStart = "\033[35;1m";  // Bold Magenta
                    break;
                default:
                    break;
            }
        }

        // Timestamp
        if (config_.showTimestamp) {
            auto now = std::chrono::system_clock::now();
            auto time = std::chrono::system_clock::to_time_t(now);
            auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                          now.time_since_epoch()) %
                      1000;
            std::tm tm_buf{};
#ifdef _WIN32
            localtime_s(&tm_buf, &time);
#else
            localtime_r(&time, &tm_buf);
#endif
            oss << std::put_time(&tm_buf, "%Y-%m-%d %H:%M:%S") << "."
                << std::setfill('0') << std::setw(3) << ms.count() << " ";
        }

        // Thread ID
        if (config_.showThreadId) {
            oss << "[" << std::hex << std::this_thread::get_id() << std::dec
                << "] ";
        }

        // Level
        if (config_.showLevel) {
            oss << colorStart << std::setw(5) << std::left
                << logLevelToString(level) << colorEnd << " ";
        }

        // Module
        if (config_.showModule) {
            oss << "[" << module << "] ";
        }

        // Message
        oss << message;

        // Output to stderr
        std::cerr << oss.str() << std::endl;

        // Flush on error/fatal
        if (config_.flushOnError &&
            (level == LogLevel::ERROR || level == LogLevel::FATAL)) {
            std::cerr.flush();
        }
    }

private:
    mutable std::mutex mutex_;
    LogConfig config_;
    std::unordered_set<std::string> disabledModules_;
    bool isTerminal_;

#ifdef _WIN32
    static int isatty(int) { return 0; }  // Windows fallback
    static int fileno(FILE*) { return 0; }
#endif
};

// Logger implementation

Logger& Logger::instance() {
    static Logger instance;
    return instance;
}

Logger::Logger() : impl_(std::make_unique<LoggerImpl>()) {}

Logger::~Logger() = default;

void Logger::configure(const LogConfig& config) { impl_->configure(config); }

LogConfig Logger::getConfig() const { return impl_->getConfig(); }

void Logger::setLevel(LogLevel level) { impl_->setLevel(level); }

LogLevel Logger::getLevel() const { return impl_->getLevel(); }

void Logger::setModuleEnabled(const std::string& module, bool enabled) {
    impl_->setModuleEnabled(module, enabled);
}

bool Logger::isModuleEnabled(const std::string& module) const {
    return impl_->isModuleEnabled(module);
}

bool Logger::isLevelEnabled(LogLevel level) const {
    return impl_->isLevelEnabled(level);
}

void Logger::log(LogLevel level, const char* module,
                 const std::string& message) {
    impl_->log(level, module, message);
}

void Logger::debug(const char* module, const std::string& message) {
    log(LogLevel::DEBUG, module, message);
}

void Logger::info(const char* module, const std::string& message) {
    log(LogLevel::INFO, module, message);
}

void Logger::warn(const char* module, const std::string& message) {
    log(LogLevel::WARN, module, message);
}

void Logger::error(const char* module, const std::string& message) {
    log(LogLevel::ERROR, module, message);
}

void Logger::fatal(const char* module, const std::string& message) {
    log(LogLevel::FATAL, module, message);
}

namespace detail {

std::string formatMessage(const char* fmt) { return std::string(fmt); }

}  // namespace detail

}  // namespace vi_slam
