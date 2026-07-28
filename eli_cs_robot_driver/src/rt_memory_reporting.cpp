// Effectful real-time memory REPORTING for the control node: reads
// /proc/self/status and getrusage(RUSAGE_THREAD), formats via the pure helpers in
// rt_memory_reporting.hpp, and logs through rclcpp. This is the only file in the
// rt_memory split that depends on rclcpp; locking/mallopt/reserve (rt_memory.cpp)
// deliberately have no such dependency.

// RUSAGE_THREAD (per-thread page-fault counts) is a GNU extension; request it
// before <sys/resource.h> is pulled in.
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include "eli_cs_robot_driver/rt_memory_reporting.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>

#include <sys/resource.h>

#include <rclcpp/logging.hpp>
#include <rclcpp/logger.hpp>

namespace ELITE_CS_ROBOT_ROS_DRIVER {
namespace rt_memory {

namespace {
std::string read_proc_self_status() {
    std::ifstream f("/proc/self/status");
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}
}  // namespace

RtMemoryMonitor::RtMemoryMonitor(double loop_rate_hz, double log_interval_seconds)
    : enabled_(log_interval_seconds > 0.0),
      iters_per_log_(enabled_
          ? std::max(1L, static_cast<long>(std::lround(loop_rate_hz * log_interval_seconds)))
          : 0),
      interval_seconds_(log_interval_seconds) {}

void RtMemoryMonitor::tick(const rclcpp::Logger& logger) {
    if (!enabled_) {
        return;
    }
    if (++counter_ < iters_per_log_) {
        return;
    }
    counter_ = 0;

    // Process memory footprint, so the memlock cap can be sized from the real
    // peak (VmHWM) observed over a representative run.
    const std::string status = read_proc_self_status();
    RCLCPP_INFO(logger, "%s",
        format_memory_report(parse_status_kb(status, "VmRSS"),
                             parse_status_kb(status, "VmLck"),
                             parse_status_kb(status, "VmHWM")).c_str());

    struct rusage ru{};
    if (getrusage(RUSAGE_THREAD, &ru) != 0) {
        return;
    }

    if (!have_baseline_) {
        // First report interval: capture the post-startup baseline, log nothing.
        baseline_minflt_ = ru.ru_minflt;
        baseline_majflt_ = ru.ru_majflt;
        have_baseline_ = true;
        return;
    }

    const long minor_delta = ru.ru_minflt - baseline_minflt_;
    const long major_delta = ru.ru_majflt - baseline_majflt_;
    baseline_minflt_ = ru.ru_minflt;
    baseline_majflt_ = ru.ru_majflt;

    const std::string report =
        format_fault_report(minor_delta, major_delta, ru.ru_minflt, ru.ru_majflt, interval_seconds_);
    // A nonzero major-fault rate means the control thread is blocking on I/O --
    // exactly what the lock should prevent -- so surface it as a warning.
    if (major_delta > 0) {
        RCLCPP_WARN(logger, "%s", report.c_str());
    } else {
        RCLCPP_INFO(logger, "%s", report.c_str());
    }
}

}  // namespace rt_memory
}  // namespace ELITE_CS_ROBOT_ROS_DRIVER
