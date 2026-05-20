#pragma once

#include <Elite/Log.hpp>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <map>
#include <mutex>
#include <string>
#include <utility>

namespace ELITE_CS_ROBOT_ROS_DRIVER {

// Wraps SDK log output and throttles repeated logs from the same source file
// and log level. The first occurrence of any (file_basename, level) pair logs
// immediately; repeats within kLogThrottleSeconds are suppressed. All other
// (file_basename, level) pairs pass through without delay.
//
// Thread-safe: mutex guards the throttle map. Registered once per process via
// ELITE::registerLogHandler() in EliteCSPositionHardwareInterface::on_configure().
class ThrottledSdkLogHandler : public ELITE::LogHandler {
public:
    void log(const char* file, int line, ELITE::LogLevel level, const char* msg) override {
        if (file) {
            const char* slash = std::strrchr(file, '/');
            const char* basename = slash ? slash + 1 : file;

            auto now = std::chrono::steady_clock::now();
            std::lock_guard<std::mutex> lock(mutex_);
            auto key = std::make_pair(std::string(basename), level);
            auto it = throttle_map_.find(key);
            if (it != throttle_map_.end()) {
                if (std::chrono::duration<double>(now - it->second).count() < kLogThrottleSeconds) {
                    return;
                }
                it->second = now;
            } else {
                throttle_map_.emplace(key, now);
            }
        }
        const char* level_str =
            level == ELITE::LogLevel::ELI_DEBUG ? "DEBUG" :
            level == ELITE::LogLevel::ELI_INFO  ? "INFO"  :
            level == ELITE::LogLevel::ELI_WARN  ? "WARN"  :
            level == ELITE::LogLevel::ELI_ERROR ? "ERROR" : "FATAL";
        std::printf("[%s] %s:%d: %s\n", level_str, file ? file : "", line, msg ? msg : "");
        std::fflush(stdout);
    }

private:
    static constexpr double kLogThrottleSeconds = 10.0;
    std::mutex mutex_;
    std::map<std::pair<std::string, ELITE::LogLevel>,
             std::chrono::steady_clock::time_point> throttle_map_;
};

}  // namespace ELITE_CS_ROBOT_ROS_DRIVER
