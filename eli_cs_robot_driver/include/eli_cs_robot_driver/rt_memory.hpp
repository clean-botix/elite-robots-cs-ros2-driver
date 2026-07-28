// Real-time memory MANAGEMENT for the control node: locking the process into RAM
// and preparing the allocator/heap for a real-time loop.
//
// Everything here -- including configure_realtime_memory() -- is deliberately
// free of ROS/rclcpp, performs no logging, and only uses ordinary Linux/glibc
// calls. That makes it unit-testable (see test/test_rt_memory.cpp) and lets
// test/manual/mlock_demo.cpp call the exact same production locking code with a
// plain compiler, no ROS/colcon required. The caller (control_node.cpp) reads the
// returned facts and does its own rclcpp logging.
//
// Periodic *reporting* (the page-fault/memory-footprint monitor, which does need
// rclcpp for logging) lives separately in rt_memory_reporting.hpp/.cpp.
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
// and mlockall(MCL_FUTURE) (both set by configure_realtime_memory below), the
// pages faulted here stay resident and locked and back later malloc()/new from
// the RT loop.
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

// Default heap headroom for reserve_process_memory() to pre-fault before the RT
// loop. With glibc M_TRIM_THRESHOLD=-1 in effect this becomes a permanent RSS
// floor, so it should be sized as headroom over the measured footprint (SW-933).
inline constexpr std::size_t kDefaultHeapReserveBytes = 100UL * 1024 * 1024; // 100 MiB

// Raw facts from one configure_realtime_memory() call. No interpretation and no
// logging is done here -- the caller decides how (or whether) to report these.
// Pass memlock_soft/memlock_hard to describe_rlimit() to format them.
struct RealtimeMemorySetup {
    rlim_t memlock_soft = 0;       // from getrlimit(RLIMIT_MEMLOCK); meaningful only if getrlimit_succeeded
    rlim_t memlock_hard = 0;
    bool getrlimit_succeeded = false;
    bool mlockall_succeeded = false;
    int mlockall_errno = 0;        // valid only when !mlockall_succeeded
};

// Lock the process into RAM and prepare it for a real-time loop:
//   1. read RLIMIT_MEMLOCK (reported in the return value; an absent/insufficient
//      ulimit is why step 2 below would fail -- the caller decides how to log it);
//   2. mlockall(MCL_CURRENT | MCL_FUTURE) to keep every page resident (failure is
//      reported in the return value, not fatal -- this function never aborts);
//   3. mallopt() so future allocations don't reintroduce page-fault stalls:
//      M_MMAP_MAX=0 keeps allocations off the fault-heavy mmap path, and
//      M_TRIM_THRESHOLD=-1 stops glibc from returning freed memory to the kernel;
//   4. pre-fault `reserve_bytes` of heap (reserve_process_memory) so first-touch
//      faults for the steady-state working set happen here, not in the RT loop.
// ROS-free: ordinary Linux/glibc calls only, no rclcpp. This is the exact
// function both control_node.cpp and test/manual/mlock_demo.cpp call -- the demo
// links this file directly with a plain compiler to exercise the real locking
// code, no ROS install required.
RealtimeMemorySetup configure_realtime_memory(std::size_t reserve_bytes = kDefaultHeapReserveBytes);

}  // namespace rt_memory
}  // namespace ELITE_CS_ROBOT_ROS_DRIVER
