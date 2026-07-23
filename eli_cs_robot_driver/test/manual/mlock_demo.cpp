// mlock_demo — deterministically demonstrate the mechanism control_node relies on:
// the kernel will not reclaim memory that has been locked with mlockall().
//
// It allocates a buffer, faults every page in, optionally locks the process,
// then asks the kernel to discard the pages (madvise(MADV_DONTNEED)) and touches
// them again while counting minor page faults via getrusage():
//
//   --nolock : discard succeeds  -> re-touch re-faults every page (minflt ~= pages)
//   --lock   : discard is refused on locked pages (EINVAL) -> re-touch faults 0
//
// A large minor-fault delta with --nolock and ~0 with --lock is direct proof the
// lock keeps the working set resident. No root, no swap, runs in seconds; run it
// in the dev/drivers container (or any Linux box).
//
// Build & run:
//   g++ -O2 -std=c++17 mlock_demo.cpp -o mlock_demo
//   ./mlock_demo --nolock
//   ./mlock_demo --lock
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <string>

#include <sys/mman.h>
#include <sys/resource.h>
#include <unistd.h>

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
        // Mirror control_node: lock the whole process into RAM.
        if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
            fprintf(stderr, "mlockall failed (%s) -- run with a sufficient memlock "
                            "ulimit (e.g. `ulimit -l unlimited`) to see the locked case\n",
                    std::strerror(errno));
            return 3;
        }
    }

    // Ask the kernel to drop the pages. On a locked region this is refused, which
    // is exactly the protection mlockall() buys the control thread.
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
