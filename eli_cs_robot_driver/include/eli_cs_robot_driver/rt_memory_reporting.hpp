// Periodic real-time memory REPORTING for the control node: logging page-fault
// counts and process memory footprint so the lock's effectiveness (and the peak
// RSS needed to size the memlock ulimit cap) are observable in normal container
// logs, without perf/proc tooling.
//
// Split out from rt_memory.hpp/.cpp (which handle locking/mallopt/reserve) since
// that file grew to hold two logically distinct responsibilities -- this one is
// reporting, that one is management. This header's formatting/parsing helpers
// are pure and ROS-free (unit-tested in test/test_rt_memory_reporting.cpp);
// RtMemoryMonitor::tick() needs rclcpp for logging, so rclcpp::Logger is only
// forward-declared here and the implementation lives in rt_memory_reporting.cpp.
#pragma once

#include <cstdio>
#include <string>

namespace rclcpp {
class Logger;
}

namespace ELITE_CS_ROBOT_ROS_DRIVER {
namespace rt_memory {

// Format one periodic page-fault report line for the RT control thread. Pure and
// allocation-cheap so it is unit-tested and safe to build inside the loop. Deltas
// are since the previous report; totals are thread-lifetime.
inline std::string format_fault_report(long minor_delta, long major_delta,
                                       long minor_total, long major_total,
                                       double interval_seconds) {
    const double major_per_s =
        interval_seconds > 0.0 ? static_cast<double>(major_delta) / interval_seconds : 0.0;
    char buf[256];
    std::snprintf(buf, sizeof(buf),
        "page-fault check (control thread): +%ld major, +%ld minor over %.0fs "
        "(%.2f major/s); lifetime %ld major / %ld minor",
        major_delta, minor_delta, interval_seconds, major_per_s, major_total, minor_total);
    return std::string(buf);
}

// Parse a "Key:   <n> kB" entry from /proc/<pid>/status contents; returns n in
// kB, or -1 if the key is absent. Pure/testable (feed it file contents).
inline long parse_status_kb(const std::string& status, const char* key) {
    const std::string needle = std::string(key) + ":";
    std::size_t pos = status.find(needle);
    if (pos == std::string::npos) {
        return -1;
    }
    pos += needle.size();
    long value = 0;
    bool seen_digit = false;
    for (; pos < status.size(); ++pos) {
        const char c = status[pos];
        if (c >= '0' && c <= '9') {
            value = value * 10 + (c - '0');
            seen_digit = true;
        } else if (seen_digit || c == '\n') {
            break;  // end of the number, or end of line with no number
        }
        // otherwise: skip leading whitespace before the number
    }
    return seen_digit ? value : -1;
}

// Format the process memory-footprint line. Inputs are kB (as read from
// /proc/<pid>/status); -1 renders as -1 (field unavailable). Pure/testable.
// VmHWM (peak RSS) is the value to size the memlock ulimit cap from.
inline std::string format_memory_report(long vmrss_kb, long vmlck_kb, long vmhwm_kb) {
    auto to_mib = [](long kb) -> long { return kb < 0 ? -1 : kb / 1024; };
    char buf[256];
    std::snprintf(buf, sizeof(buf),
        "memory check (process): RSS=%ld MiB, locked=%ld MiB, peak RSS(VmHWM)=%ld MiB "
        "-- size the memlock cap from the peak",
        to_mib(vmrss_kb), to_mib(vmlck_kb), to_mib(vmhwm_kb));
    return std::string(buf);
}

// Default period between monitor reports, used when the rt_memory.log_interval_sec
// ROS2 parameter (see config/rt_memory.yaml) is not set. Long enough to keep the
// logs quiet; the RT concern is any nonzero major-fault rate at all, which shows
// up regardless of interval length.
inline constexpr double kDefaultLogIntervalSeconds = 30.0;

// Periodically logs the calling (control) thread's page-fault counts (via
// getrusage(RUSAGE_THREAD)) and the process memory footprint (VmRSS/VmLck/VmHWM
// from /proc/self/status), so fault activity and the peak RSS needed to size the
// memlock cap are visible in normal container logs without perf/proc tooling.
// RT-friendly: call tick() once per loop iteration; it is O(1) and only does the
// syscall/file read + log once per log interval (gated by iteration count, so no
// per-iteration clock call). The first tick reports memory and establishes the
// fault baseline. A log interval <= 0 disables it entirely (tick() is a no-op).
// Defined in rt_memory_reporting.cpp (needs getrusage + rclcpp).
class RtMemoryMonitor {
public:
    RtMemoryMonitor(double loop_rate_hz, double log_interval_seconds = kDefaultLogIntervalSeconds);
    void tick(const rclcpp::Logger& logger);

private:
    bool enabled_;
    long iters_per_log_;
    double interval_seconds_;
    long counter_ = 0;
    bool have_baseline_ = false;
    long baseline_minflt_ = 0;
    long baseline_majflt_ = 0;
};

}  // namespace rt_memory
}  // namespace ELITE_CS_ROBOT_ROS_DRIVER
