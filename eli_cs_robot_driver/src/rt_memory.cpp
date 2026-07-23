// Effectful real-time memory setup for the control node. Kept out of the header
// (and out of control_node.cpp) so the glibc-only syscalls and rclcpp logging
// live in one place; the pure, unit-tested helpers stay in rt_memory.hpp.

// RUSAGE_THREAD (per-thread page-fault counts) is a GNU extension; request it
// before <sys/resource.h> is pulled in (via rt_memory.hpp).
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include "eli_cs_robot_driver/rt_memory.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>

#include <malloc.h>
#include <sys/mman.h>
#include <sys/resource.h>

#include <rclcpp/logging.hpp>
#include <rclcpp/logger.hpp>

namespace ELITE_CS_ROBOT_ROS_DRIVER {
namespace rt_memory {

PageFaultMonitor::PageFaultMonitor(double loop_rate_hz, double log_interval_seconds)
    : iters_per_log_(std::max(1L, static_cast<long>(std::lround(loop_rate_hz * log_interval_seconds)))),
      interval_seconds_(log_interval_seconds) {}

void PageFaultMonitor::tick(const rclcpp::Logger& logger) {
    if (++counter_ < iters_per_log_) {
        return;
    }
    counter_ = 0;

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

void configure_realtime_memory(const rclcpp::Logger& logger, std::size_t reserve_bytes) {
    // Log the memlock rlimit this process is working with, so a later
    // memory-exhaustion condition is diagnosable and an absent/insufficient
    // ulimit is obvious in the logs (mlockall below silently fails without it).
    struct rlimit memlock_limit{};
    if (getrlimit(RLIMIT_MEMLOCK, &memlock_limit) == 0) {
        RCLCPP_INFO(logger, "RLIMIT_MEMLOCK: soft=%s hard=%s",
            describe_rlimit(memlock_limit.rlim_cur).c_str(),
            describe_rlimit(memlock_limit.rlim_max).c_str());
    }

    // Lock all of this PROCESS's current and future pages in RAM. mlockall is
    // process-scoped, not thread-scoped, even though it is called from the
    // control thread: a page fault in the RT control thread blocks on the kernel
    // mm subsystem (possibly I/O) for tens to hundreds of ms, which RT priority
    // does not prevent — long enough to starve the cyclic bus update (e.g.
    // EtherCAT slave SM watchdogs). Requires the container to grant a sufficient
    // memlock ulimit (the deployed compose sets ulimits.memlock = -1); warn and
    // continue so a missing ulimit degrades RT behavior instead of preventing
    // startup.
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        RCLCPP_WARN(logger,
            "mlockall failed (%s) — control thread may page-fault under memory pressure",
            std::strerror(errno));
    }

    // Complement the lock so future allocations don't reintroduce page-fault
    // stalls in the RT loop:
    //   M_MMAP_MAX = 0        -> never service allocations via mmap (the mmap path
    //                            faults page-by-page on first touch); heap growth
    //                            goes through the main arena.
    //   M_TRIM_THRESHOLD = -1 -> never return freed memory to the kernel, so it
    //                            stays mapped (and locked) for later reuse.
    mallopt(M_MMAP_MAX, 0);
    mallopt(M_TRIM_THRESHOLD, -1);

    // Front-load heap growth: fault in (and, via MCL_FUTURE, lock) a reserve now
    // so the RT loop reuses already-resident pages instead of faulting.
    reserve_process_memory(reserve_bytes);
}

}  // namespace rt_memory
}  // namespace ELITE_CS_ROBOT_ROS_DRIVER
