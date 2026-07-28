// Effectful real-time memory MANAGEMENT for the control node: mlockall(),
// mallopt(), and pre-faulting a heap reserve. Deliberately has NO rclcpp
// dependency (see rt_memory.hpp) -- this file builds with a plain compiler, no
// ROS/colcon required, which is what lets test/manual/mlock_demo.cpp link it
// directly to exercise the exact same production locking code.
//
// The periodic REPORTING monitor (which does need rclcpp for logging) lives
// separately in rt_memory_reporting.cpp.
#include "eli_cs_robot_driver/rt_memory.hpp"

#include <cerrno>
#include <cstring>

#include <malloc.h>
#include <sys/mman.h>
#include <sys/resource.h>

namespace ELITE_CS_ROBOT_ROS_DRIVER {
namespace rt_memory {

RealtimeMemorySetup configure_realtime_memory(std::size_t reserve_bytes) {
    RealtimeMemorySetup setup{};

    // Read the memlock rlimit this process is working with. An
    // absent/insufficient ulimit is why the mlockall() call below would fail;
    // the caller decides how to log this (see control_node.cpp).
    struct rlimit memlock_limit{};
    if (getrlimit(RLIMIT_MEMLOCK, &memlock_limit) == 0) {
        setup.getrlimit_succeeded = true;
        setup.memlock_soft = memlock_limit.rlim_cur;
        setup.memlock_hard = memlock_limit.rlim_max;
    }

    // Lock all of this PROCESS's current and future pages in RAM. mlockall is
    // process-scoped, not thread-scoped, even though it may be called from a
    // worker thread: a page fault in the RT control thread blocks on the kernel
    // mm subsystem (possibly I/O) for tens to hundreds of ms, which RT priority
    // does not prevent — long enough to starve the cyclic bus update (e.g.
    // EtherCAT slave SM watchdogs). Requires the container to grant a sufficient
    // memlock ulimit (the deployed compose sets ulimits.memlock = -1); failure is
    // reported in the return value rather than treated as fatal here, so a
    // missing ulimit degrades RT behavior instead of preventing startup.
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == 0) {
        setup.mlockall_succeeded = true;
    } else {
        setup.mlockall_succeeded = false;
        setup.mlockall_errno = errno;
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

    return setup;
}

}  // namespace rt_memory
}  // namespace ELITE_CS_ROBOT_ROS_DRIVER
