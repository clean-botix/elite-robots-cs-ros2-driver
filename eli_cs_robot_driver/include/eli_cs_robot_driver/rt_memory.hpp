// Real-time memory helpers for the control node.
//
// The logic here is deliberately free of ROS/rclcpp and of any glibc-only calls
// (mlockall/mallopt live in control_node.cpp) so it can be unit-tested on any
// POSIX host. See src/control_node.cpp for how these are wired into startup and
// the std::terminate handler.
#pragma once

#include <cstddef>
#include <cstdlib>
#include <exception>
#include <new>
#include <string>

#include <sys/resource.h>
#include <unistd.h>

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

}  // namespace rt_memory
}  // namespace ELITE_CS_ROBOT_ROS_DRIVER
