// Effectful real-time memory setup for the control node. Kept out of the header
// (and out of control_node.cpp) so the glibc-only syscalls and rclcpp logging
// live in one place; the pure, unit-tested helpers stay in rt_memory.hpp.
#include "eli_cs_robot_driver/rt_memory.hpp"

#include <cerrno>
#include <cstring>

#include <malloc.h>
#include <sys/mman.h>
#include <sys/resource.h>

#include <rclcpp/logging.hpp>
#include <rclcpp/logger.hpp>

namespace ELITE_CS_ROBOT_ROS_DRIVER {
namespace rt_memory {

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
