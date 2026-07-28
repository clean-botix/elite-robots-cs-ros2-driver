// mlock_demo — deterministically demonstrate the mechanism control_node relies
// on: the kernel will not reclaim memory locked via
// ELITE_CS_ROBOT_ROS_DRIVER::rt_memory::configure_realtime_memory() (rt_memory.hpp).
//
// This is intended to stick around (not throwaway): it calls the exact same
// production locking function the control node uses -- not a reimplementation
// of mlockall() -- so it stays a faithful sanity check of the mechanism as that
// code evolves. rt_memory.{hpp,cpp} are deliberately ROS-free, so this still
// builds and runs with a plain compiler; no ROS/colcon install required.
//
// It allocates a buffer, faults every page in, optionally calls
// configure_realtime_memory() (which locks the whole process, this buffer
// included), then asks the kernel to discard the pages (madvise(MADV_DONTNEED))
// and touches them again while counting minor page faults via getrusage():
//
//   --nolock : discard succeeds  -> re-touch re-faults every page (minflt ~= pages)
//   --lock   : discard is refused on locked pages (EINVAL) -> re-touch faults 0
//
// A large minor-fault delta with --nolock and ~0 with --lock is direct proof the
// lock keeps the working set resident. No root, no swap, runs in seconds; run it
// in the dev/drivers container (or any Linux box).
//
// Build & run (from the eli_cs_robot_driver package root):
//   g++ -O2 -std=c++17 -Iinclude test/manual/mlock_demo.cpp src/rt_memory.cpp -o /tmp/mlock_demo
//   ulimit -l unlimited   # mirror the deployed drivers ulimit; see doc/rt_memory_test_plan.md
//   /tmp/mlock_demo --nolock
//   /tmp/mlock_demo --lock
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <string>

#include <sys/mman.h>
#include <sys/resource.h>
#include <unistd.h>

#include "eli_cs_robot_driver/rt_memory.hpp"

namespace rt = ELITE_CS_ROBOT_ROS_DRIVER::rt_memory;

static long minor_faults() {
    struct rusage ru{};
    getrusage(RUSAGE_SELF, &ru);
    return ru.ru_minflt;
}

int main(int argc, char** argv) {
    const bool lock = (argc > 1 && std::string(argv[1]) == "--lock");
    const size_t bytes = 128UL * 1024 * 1024;            // 128 MiB
    const long page = sysconf(_SC_PAGESIZE);
    const size_t pages = bytes / static_cast<size_t>(page);

    // Anonymous, page-aligned region so madvise/mlock operate on it cleanly.
    void* p = mmap(nullptr, bytes, PROT_READ | PROT_WRITE,
                   MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (p == MAP_FAILED) { perror("mmap"); return 2; }
    char* buf = static_cast<char*>(p);

    // Fault every page in once (this is the startup cost the RT loop wants to avoid).
    for (size_t i = 0; i < bytes; i += page) buf[i] = 1;

    if (lock) {
        // The exact production call: mlockall(MCL_CURRENT|MCL_FUTURE) covers this
        // already-touched buffer too, plus mallopt()/reserve_process_memory().
        const rt::RealtimeMemorySetup setup = rt::configure_realtime_memory();
        printf("RLIMIT_MEMLOCK: soft=%s hard=%s\n",
               setup.getrlimit_succeeded ? rt::describe_rlimit(setup.memlock_soft).c_str() : "?",
               setup.getrlimit_succeeded ? rt::describe_rlimit(setup.memlock_hard).c_str() : "?");
        if (!setup.mlockall_succeeded) {
            fprintf(stderr, "mlockall failed (%s) -- run with a sufficient memlock "
                            "ulimit (e.g. `ulimit -l unlimited`) to see the locked case\n",
                    std::strerror(setup.mlockall_errno));
            return 3;
        }
    }

    // Ask the kernel to drop the pages. On a locked region this is refused, which
    // is exactly the protection the lock buys the control thread.
    const int rc = madvise(buf, bytes, MADV_DONTNEED);
    printf("madvise(MADV_DONTNEED): %s\n",
           rc == 0 ? "succeeded (pages dropped)"
                   : (std::string("refused -- ") + std::strerror(errno)).c_str());

    // Re-touch every page and count how many re-faulted.
    const long before = minor_faults();
    volatile long sink = 0;
    for (size_t i = 0; i < bytes; i += page) sink += buf[i];
    const long delta = minor_faults() - before;
    (void)sink;

    printf("%s: re-touch caused %ld page faults across %zu pages\n",
           lock ? "LOCKED  " : "UNLOCKED", delta, pages);
    printf(lock ? "  -> locked memory stayed resident; the RT loop would NOT fault here.\n"
                : "  -> unlocked memory was reclaimed; the RT loop WOULD fault here.\n");
    return 0;
}
