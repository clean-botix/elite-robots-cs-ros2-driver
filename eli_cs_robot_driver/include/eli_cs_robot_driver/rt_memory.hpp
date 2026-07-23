// Real-time memory helpers for the control node.
//
// The inline helpers here are deliberately free of ROS/rclcpp and of any
// glibc-only calls, so they can be unit-tested on any POSIX host (see
// test/test_rt_memory.cpp). The effectful orchestration that ties them together
// with mlockall()/mallopt() and rclcpp logging lives in rt_memory.cpp behind
// configure_realtime_memory(); rclcpp::Logger is only forward-declared here so
// this header stays cheap to include and the pure helpers remain testable
// without linking rclcpp.
#pragma once

#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <new>
#include <string>

#include <sys/resource.h>
#include <unistd.h>

namespace rclcpp {
class Logger;
}

namespace ELITE_CS_ROBOT_ROS_DRIVER {
namespace rt_memory {

// Human-readable form of an RLIMIT_MEMLOCK value, for startup logging.
// RLIM_INFINITY renders as "unlimited"; everything else is reported in whole MiB.
inline std::string describe_rlimit(rlim_t v) {
    return v == RLIM_INFINITY ? std::string("unlimited")
                              : std::to_string(v / (1024 * 1024)) + " MiB";
}

// How the std::terminate handler should treat the exception (if any) in flight.
enum class TerminateDisposition {
    FatalBadAlloc,       // std::bad_alloc -> memlock ceiling / memory exhaustion; exit non-zero
    FatalException,      // any other std::exception; exit non-zero
    IgnoredShutdownRace, // no identifiable std::exception -> known ROS2 Humble CM shutdown race; exit 0
};

// Result of classifying a terminate: the disposition plus the exception's what()
// text when one was recovered (empty otherwise), so the handler can log detail.
struct TerminateInfo {
    TerminateDisposition disposition;
    std::string detail;
};

// Decide how to handle the exception in flight at std::terminate time (pass
// std::current_exception(); may be null). Pure: performs no logging and never
// terminates the process, so the policy is unit-testable.
inline TerminateInfo classify_terminate(std::exception_ptr ex) {
    if (ex) {
        try {
            std::rethrow_exception(ex);
        } catch (const std::bad_alloc& e) {
            return {TerminateDisposition::FatalBadAlloc, e.what()};
        } catch (const std::exception& e) {
            return {TerminateDisposition::FatalException, e.what()};
        } catch (...) {
            // Non-std exception in flight -- treat like the shutdown-race case.
        }
    }
    return {TerminateDisposition::IgnoredShutdownRace, std::string()};
}

// Process exit code for a disposition: 0 for the tolerated shutdown race, 1 for
// the fatal memory/exception cases.
inline int terminate_exit_code(TerminateDisposition d) {
    return d == TerminateDisposition::IgnoredShutdownRace ? 0 : 1;
}

// Front-load heap growth: touch every page of a scratch buffer so the first-touch
// page faults for the process's working set happen here, at startup, rather than
// inside the real-time control loop. When paired with glibc M_TRIM_THRESHOLD=-1
// and mlockall(MCL_FUTURE) (both set in control_node.cpp), the pages faulted here
// stay resident and locked and back later malloc()/new from the RT loop.
inline void reserve_process_memory(std::size_t size) {
    // volatile so the page-touching writes are not elided as dead stores (the
    // buffer is freed without ever being read).
    volatile char* buffer = static_cast<volatile char*>(std::malloc(size));
    if (buffer == nullptr) {
        return;
    }
    const long page_size = sysconf(_SC_PAGESIZE);
    for (std::size_t i = 0; i < size; i += static_cast<std::size_t>(page_size)) {
        buffer[i] = 0; // fault the page in
    }
    std::free(const_cast<char*>(buffer));
}

// Conservative heap headroom for reserve_process_memory() to pre-fault before
// the RT loop. With glibc M_TRIM_THRESHOLD=-1 in effect this becomes a permanent
// RSS floor, so it is sized as headroom over the measured footprint; tune with
// monitoring (SW-933).
inline constexpr std::size_t kDefaultHeapReserveBytes = 100UL * 1024 * 1024; // 100 MiB

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

// Default period between monitor reports. Long enough to keep the logs quiet; the
// RT concern is any nonzero major-fault rate at all, which shows up regardless.
inline constexpr double kDefaultLogIntervalSeconds = 30.0;

// Env var (read in control_node.cpp) that overrides the report interval in
// seconds. A value <= 0 disables the monitor; absent/empty/invalid uses the
// default above. Set it in the bot's .env (loaded into the container).
inline constexpr const char* kLogIntervalEnvVar = "OPTIMUSCLEAN_DRIVERS_MEMORY_LOG_INTERVAL_SEC";

// Parse a seconds value from an env-var string. Null/empty/non-numeric returns
// `fallback`; a valid value (including <= 0, meaning "disabled") is returned as
// parsed. Pure/testable.
inline double parse_interval_seconds(const char* value, double fallback) {
    if (value == nullptr || *value == '\0') {
        return fallback;
    }
    char* end = nullptr;
    const double parsed = std::strtod(value, &end);
    if (end == value) {  // no digits consumed
        return fallback;
    }
    return parsed;
}

// Periodically logs the calling (control) thread's page-fault counts (via
// getrusage(RUSAGE_THREAD)) and the process memory footprint (VmRSS/VmLck/VmHWM
// from /proc/self/status), so fault activity and the peak RSS needed to size the
// memlock cap are visible in normal container logs without perf/proc tooling.
// RT-friendly: call tick() once per loop iteration; it is O(1) and only does the
// syscall/file read + log once per log interval (gated by iteration count, so no
// per-iteration clock call). The first tick reports memory and establishes the
// fault baseline. A log interval <= 0 disables it entirely (tick() is a no-op).
// Defined in rt_memory.cpp (needs getrusage + rclcpp).
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

// Configure the process for real-time memory behavior, logging via `logger`:
//   1. log the RLIMIT_MEMLOCK the process is working with (an absent/insufficient
//      ulimit makes step 2 silently fail -- surfacing it is the point);
//   2. mlockall(MCL_CURRENT | MCL_FUTURE) to keep every page resident (warn and
//      continue on failure rather than block startup);
//   3. mallopt() so future allocations don't reintroduce page-fault stalls;
//   4. pre-fault `reserve_bytes` of heap so first-touch faults happen here.
// Effectful and glibc/rclcpp-dependent, hence defined in rt_memory.cpp rather
// than inline. Intended to be called once from the control loop thread, after
// RT scheduling is configured. See src/control_node.cpp.
void configure_realtime_memory(const rclcpp::Logger& logger,
                               std::size_t reserve_bytes = kDefaultHeapReserveBytes);

}  // namespace rt_memory
}  // namespace ELITE_CS_ROBOT_ROS_DRIVER
