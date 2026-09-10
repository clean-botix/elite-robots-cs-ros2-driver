// Real-time TIMEBASE for the control node's cyclic loop: the period grid the
// loop sleeps on, how a late wake-up is classified, and the lock-free period
// statistics the loop records for a reporter on another thread.
//
// Companion to rt_memory.hpp and rt_sched.hpp and modeled on them: no rclcpp,
// no logging, no allocation after construction, pure helpers unit-tested in
// test/test_rt_timebase.cpp. control_node.cpp owns the loop and the logging.
//
// Why this exists: the loop used to build its grid on std::chrono::system_clock
// and sleep with std::this_thread::sleep_until. libstdc++ implements that as
// `while (now < t) sleep_for(t - now)`, so a backward wall-clock step (an NTP
// correction on the robot: -0.894 s) stretched one cycle by the size of the step,
// which tripped the EtherCAT drives' 500 ms watchdog. Afterwards the loop ran the
// missed cycles back to back to catch up with the grid. Two rules fix both:
//   1. The grid lives on CLOCK_MONOTONIC and each cycle sleeps to an ABSOLUTE
//      deadline with clock_nanosleep(TIMER_ABSTIME). A wall-clock STEP cannot
//      move it and there is no per-cycle drift from relative sleeps. (NTP
//      frequency slew does reach CLOCK_MONOTONIC on Linux, capped at 500 ppm =
//      10 us per 20 ms cycle, smooth, and it reached the old clock identically.)
//   2. A wake-up a full period or more late is an OVERRUN: the loop counts it,
//      records how late it was, and moves the grid to now + period. It never
//      bursts to catch up. A wake-up less than a period late is merely LATE and
//      the grid stays where it was.
// The loop thread only updates PeriodStats (relaxed atomics, single writer);
// summarising and logging happen on a wall timer on the executor thread.
#pragma once

#include <atomic>
#include <cstdint>
#include <ctime>
#include <string>

namespace ELITE_CS_ROBOT_ROS_DRIVER {
namespace rt_timebase {

inline constexpr int64_t kNsPerSec = 1'000'000'000;
inline constexpr int64_t kNsPerUs = 1'000;

// Any measured period above this multiple of the nominal period is handed to
// the controller manager clamped (see clamp_measured_period). Integrators in
// the controllers must not see a multi-second dt after a stall.
inline constexpr int64_t kMaxMeasuredPeriods = 10;

// Default for the rt_timebase.report_interval_sec parameter: how often the
// executor-side reporter logs one line of PeriodStats. <= 0 disables the report.
inline constexpr double kDefaultReportIntervalSeconds = 30.0;

// ---------------------------------------------------------------------------
// timespec arithmetic (CLOCK_MONOTONIC values). All pure.

// Nominal period for an update rate; 0 Hz maps to 0 ns (caller must reject it).
inline int64_t period_ns_from_rate(unsigned update_rate_hz) {
    return update_rate_hz == 0 ? 0 : kNsPerSec / static_cast<int64_t>(update_rate_hz);
}

// t + ns, normalised so 0 <= tv_nsec < 1e9 (ns may be negative).
timespec add_ns(const timespec& t, int64_t ns);

// a - b in nanoseconds.
inline int64_t diff_ns(const timespec& a, const timespec& b) {
    return (static_cast<int64_t>(a.tv_sec) - static_cast<int64_t>(b.tv_sec)) * kNsPerSec +
           (static_cast<int64_t>(a.tv_nsec) - static_cast<int64_t>(b.tv_nsec));
}

// The grid point one period after `prev`.
inline timespec next_deadline(const timespec& prev, int64_t period_ns) {
    return add_ns(prev, period_ns);
}

// ---------------------------------------------------------------------------
// Wake-up classification.

enum class Verdict {
    OnTime,   // woke at or before the deadline (lateness_ns <= 0)
    Late,     // 0 < lateness_ns < period_ns; grid unchanged
    Overrun,  // lateness_ns >= period_ns; grid resynchronised to now
};

struct Classification {
    Verdict verdict = Verdict::OnTime;
    int64_t lateness_ns = 0;  // now - deadline, may be negative when OnTime
};

// How late `now` is relative to `deadline`, bucketed by `period_ns`.
Classification classify(const timespec& now, const timespec& deadline, int64_t period_ns);

// One loop iteration's scheduling decision: classify the wake-up and produce the
// deadline for the next cycle. OnTime and Late advance the grid by exactly one
// period; Overrun restarts it from `now`, so the missed cycles are dropped
// rather than run back to back (design decision 2).
struct Step {
    Classification wake;
    timespec next_deadline{};
};
Step step(const timespec& deadline, const timespec& now, int64_t period_ns);

// The dt handed to controller_manager->read/update/write. Clamped to
// [0, kMaxMeasuredPeriods x period]; `clamped` reports that the clamp bit.
struct ClampedPeriod {
    int64_t ns = 0;
    bool clamped = false;
};
ClampedPeriod clamp_measured_period(int64_t measured_ns, int64_t period_ns);

// ---------------------------------------------------------------------------
// Effectful wrappers over the two syscalls the loop needs (in rt_timebase.cpp).

// clock_gettime(CLOCK_MONOTONIC). A vDSO call on Linux: no syscall, ~20-40 ns.
timespec now_monotonic();

// clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, deadline). Retries on EINTR
// (an absolute deadline makes the retry exact). Returns 0 or the errno of a
// non-EINTR failure; never throws. A deadline already in the past returns at once.
int sleep_until_monotonic(const timespec& deadline);

// ---------------------------------------------------------------------------
// Period statistics: written by the loop, read by a reporter on another thread.
//
// Threading model
//   * One writer (the loop thread) calls record() once per cycle. Everything it
//     touches lives on two 64-byte-aligned lines that nothing else shares and is
//     stored with memory_order_relaxed.
//   * One reader (the reporter) calls snapshot() and request_window_reset() at
//     a low rate. It never writes the loop's line; request_window_reset() bumps a
//     counter on a SEPARATE line, and the loop applies the reset itself on its
//     next record(). So the loop's lines are only ever read from another core,
//     once per report. Cost on the loop: ~10 relaxed stores, one release fence
//     and one release store per cycle. No locks, no waits, no allocation.
//   * Consistency: a sequence counter (seqlock pattern) brackets each record().
//     snapshot() retries if it observed a record() in flight, so a snapshot is
//     never a mix of two cycles (count from one, sum from another). The
//     writer never retries or waits; only the reader does, and at most for the
//     duration of a few stores.
//
// Two kinds of fields
//   * Monotonic counters (count, sum, sum of squares, overruns, clamps) are
//     never reset. The reporter subtracts its previous snapshot (summarize()).
//   * Window extremes (min, max, max lateness) cannot be differenced, so they
//     are per window: the reporter asks for a new window IMMEDIATELY after each
//     snapshot (before it formats or logs anything), so the cycles counted in a
//     report and the extremes shown in it are the same set, give or take one
//     record() in flight between the two calls.
//     `window` in the snapshot says which window the extremes belong to; if it
//     lags the last request the loop has not run since (it is stalled), and the
//     extremes are stale.
//
// Units: periods and lateness in ns except the moment sums, which are in
// microseconds (sum) and microseconds squared (sum of squares) so the latter
// cannot overflow int64 in a robot's lifetime (20 ms = 2e4 us -> 4e8 per
// sample; ~2e10 samples fit).

struct Snapshot {
    uint64_t count = 0;         // cycles recorded
    int64_t sum_us = 0;         // sum of measured periods, us (rounded)
    uint64_t sum_sq_us2 = 0;    // sum of squared measured periods, us^2
    uint64_t overruns = 0;      // cycles classified Overrun
    uint64_t clamps = 0;        // cycles whose dt was clamped for the controller manager
    int64_t min_ns = 0;         // window extremes; 0/0/0 when the window has no sample
    int64_t max_ns = 0;
    int64_t max_lateness_ns = 0;
    uint32_t window = 0;        // window the extremes belong to
};

struct CycleSample {
    int64_t period_ns = 0;      // measured interval since the previous wake-up
    int64_t lateness_ns = 0;    // Classification::lateness_ns (negative is fine)
    bool overrun = false;
    bool clamped = false;
};

class PeriodStats {
public:
    PeriodStats() = default;
    PeriodStats(const PeriodStats&) = delete;
    PeriodStats& operator=(const PeriodStats&) = delete;

    // Loop thread only. Wait-free.
    void record(const CycleSample& s);

    // Reporter only. Consistent view; retries while a record() is in flight.
    Snapshot snapshot() const;

    // Reporter only. The loop starts a fresh min/max/max-lateness window on its
    // next record(). Returns the window number the loop will report from then on.
    uint32_t request_window_reset();

private:
    // Lines 1-2: written by the loop only.
    struct alignas(64) Writer {
        std::atomic<uint32_t> seq{0};       // odd while record() is in flight
        std::atomic<uint32_t> window{0};    // window the extremes belong to
        std::atomic<uint64_t> count{0};
        std::atomic<int64_t> sum_us{0};
        std::atomic<uint64_t> sum_sq_us2{0};
        std::atomic<uint64_t> overruns{0};
        std::atomic<uint64_t> clamps{0};
        std::atomic<int64_t> min_ns{0};
        std::atomic<int64_t> max_ns{0};
        std::atomic<int64_t> max_lateness_ns{0};
    };
    static_assert(sizeof(Writer) <= 128, "PeriodStats writer state must stay within two cache lines");
    Writer w_;
    uint32_t window_seen_ = 0;  // loop-private: last window request applied

    // Line 3: written by the reporter, read by the loop once per cycle.
    struct alignas(64) Reader {
        std::atomic<uint32_t> window_request{0};
    };
    Reader r_;
};

// ---------------------------------------------------------------------------
// Reporting helpers (pure). The reporter feeds its previous and current
// snapshots; the result is one window's worth of statistics.

struct Report {
    uint64_t count = 0;         // cycles in the window
    int64_t elapsed_us = 0;     // monotonic time those cycles span (sum of their periods);
                                // the report cadence itself is a wall timer and shifts with
                                // wall-clock steps, this figure does not
    uint64_t overruns = 0;
    uint64_t clamps = 0;
    double mean_us = 0.0;       // 0 when count == 0
    double std_us = 0.0;        // population standard deviation, 0 when count < 2
    int64_t min_ns = 0;         // extremes of the window `cur` reports (see stale)
    int64_t max_ns = 0;
    int64_t max_lateness_ns = 0;
    bool stale = false;         // cur.window != expected_window: loop has not run
                                // since the last reset request; extremes are old
};

Report summarize(const Snapshot& prev, const Snapshot& cur, uint32_t expected_window);

// One human-readable line for the node's periodic INFO message, e.g.
// "period 20.00 ms: n=1500 over 30.00 s min/mean/max 19.81/20.00/20.21 ms std 0.050 ms, overruns 0, clamps 0, max lateness 0.19 ms".
// `over N s` is the monotonic span of the counted cycles, so a window that the
// executor's wall timer stretched or shortened across a wall-clock step still
// reads correctly (n=350 over 7.00 s, not "350 cycles in 5 s").
std::string describe(const Report& r, int64_t period_ns);

}  // namespace rt_timebase
}  // namespace ELITE_CS_ROBOT_ROS_DRIVER
