// Real-time TIMEBASE helpers for the control loop (see rt_timebase.hpp): grid
// arithmetic, wake-up classification, the two CLOCK_MONOTONIC syscall wrappers,
// and the single-writer PeriodStats accumulator with its reporter-side summary.
// No rclcpp dependency.
#include "eli_cs_robot_driver/rt_timebase.hpp"

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <limits>

#include <time.h>  // clock_gettime, clock_nanosleep, TIMER_ABSTIME

namespace ELITE_CS_ROBOT_ROS_DRIVER {
namespace rt_timebase {

// ---------------------------------------------------------------------------
// Grid arithmetic and classification

timespec add_ns(const timespec& t, int64_t ns) {
    int64_t total_ns = static_cast<int64_t>(t.tv_nsec) + ns;
    int64_t carry_s = total_ns / kNsPerSec;
    total_ns -= carry_s * kNsPerSec;
    if (total_ns < 0) {  // C++ division truncates toward zero; renormalise
        total_ns += kNsPerSec;
        carry_s -= 1;
    }
    timespec out{};
    out.tv_sec = static_cast<time_t>(static_cast<int64_t>(t.tv_sec) + carry_s);
    out.tv_nsec = static_cast<long>(total_ns);
    return out;
}

Classification classify(const timespec& now, const timespec& deadline, int64_t period_ns) {
    Classification c{};
    c.lateness_ns = diff_ns(now, deadline);
    if (c.lateness_ns <= 0) {
        c.verdict = Verdict::OnTime;
    } else if (c.lateness_ns < period_ns) {
        c.verdict = Verdict::Late;
    } else {
        c.verdict = Verdict::Overrun;
    }
    return c;
}

Step step(const timespec& deadline, const timespec& now, int64_t period_ns) {
    Step s{};
    s.wake = classify(now, deadline, period_ns);
    // Overrun: the grid restarts from the moment the loop actually ran, so the
    // next cycle is one period from now and the missed ones are dropped.
    // Otherwise the grid is untouched: a Late wake-up shortens the next sleep
    // and the loop is back on the original grid one cycle later.
    s.next_deadline = (s.wake.verdict == Verdict::Overrun) ? add_ns(now, period_ns) : add_ns(deadline, period_ns);
    return s;
}

ClampedPeriod clamp_measured_period(int64_t measured_ns, int64_t period_ns) {
    ClampedPeriod p{};
    const int64_t upper = kMaxMeasuredPeriods * period_ns;
    if (measured_ns < 0) {
        p.ns = 0;
        p.clamped = true;
    } else if (measured_ns > upper) {
        p.ns = upper;
        p.clamped = true;
    } else {
        p.ns = measured_ns;
    }
    return p;
}

// ---------------------------------------------------------------------------
// Syscall wrappers

timespec now_monotonic() {
    timespec t{};
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t;
}

int sleep_until_monotonic(const timespec& deadline) {
    for (;;) {
        // clock_nanosleep returns the error directly (it does not set errno).
        const int rc = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline, nullptr);
        if (rc != EINTR) {
            return rc;
        }
        // Interrupted by a signal: with an absolute deadline, simply sleep again.
    }
}

// ---------------------------------------------------------------------------
// PeriodStats

namespace {

inline int64_t ns_to_us_rounded(int64_t ns) {
    // Periods are non-negative in practice; keep the negative branch correct anyway.
    return ns >= 0 ? (ns + kNsPerUs / 2) / kNsPerUs : -((-ns + kNsPerUs / 2) / kNsPerUs);
}

}  // namespace

void PeriodStats::record(const CycleSample& s) {
    // Seqlock writer side. Relaxed loads of our own fields are exact: this
    // thread is the only writer. The first store makes seq odd, the release
    // fence keeps the data stores from becoming visible before it, and the
    // final release store publishes the data together with the even seq.
    const uint32_t seq = w_.seq.load(std::memory_order_relaxed);
    w_.seq.store(seq + 1, std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_release);

    // Apply a pending window reset before folding this sample in, so the new
    // window's extremes start from this sample.
    const uint32_t requested = r_.window_request.load(std::memory_order_relaxed);
    const bool new_window = (requested != window_seen_) || w_.count.load(std::memory_order_relaxed) == 0;
    if (new_window) {
        window_seen_ = requested;
        w_.window.store(requested, std::memory_order_relaxed);
        w_.min_ns.store(s.period_ns, std::memory_order_relaxed);
        w_.max_ns.store(s.period_ns, std::memory_order_relaxed);
        w_.max_lateness_ns.store(s.lateness_ns, std::memory_order_relaxed);
    } else {
        if (s.period_ns < w_.min_ns.load(std::memory_order_relaxed)) {
            w_.min_ns.store(s.period_ns, std::memory_order_relaxed);
        }
        if (s.period_ns > w_.max_ns.load(std::memory_order_relaxed)) {
            w_.max_ns.store(s.period_ns, std::memory_order_relaxed);
        }
        if (s.lateness_ns > w_.max_lateness_ns.load(std::memory_order_relaxed)) {
            w_.max_lateness_ns.store(s.lateness_ns, std::memory_order_relaxed);
        }
    }

    const int64_t us = ns_to_us_rounded(s.period_ns);
    // Square in unsigned arithmetic: a signed us * us would be undefined
    // behaviour for a single gap above ~50 min, unsigned merely wraps, and the
    // magnitude is |us| either way.
    const uint64_t abs_us = static_cast<uint64_t>(us < 0 ? -us : us);
    w_.count.store(w_.count.load(std::memory_order_relaxed) + 1, std::memory_order_relaxed);
    w_.sum_us.store(w_.sum_us.load(std::memory_order_relaxed) + us, std::memory_order_relaxed);
    w_.sum_sq_us2.store(w_.sum_sq_us2.load(std::memory_order_relaxed) + abs_us * abs_us,
                        std::memory_order_relaxed);
    if (s.overrun) {
        w_.overruns.store(w_.overruns.load(std::memory_order_relaxed) + 1, std::memory_order_relaxed);
    }
    if (s.clamped) {
        w_.clamps.store(w_.clamps.load(std::memory_order_relaxed) + 1, std::memory_order_relaxed);
    }

    w_.seq.store(seq + 2, std::memory_order_release);
}

Snapshot PeriodStats::snapshot() const {
    // Seqlock reader side: read seq (acquire), read the data, fence (acquire),
    // re-read seq. Retry if a record() was in flight (odd) or completed between
    // the two reads. The writer's critical section is a handful of stores, so a
    // retry costs the reader nanoseconds and the writer nothing.
    for (;;) {
        const uint32_t before = w_.seq.load(std::memory_order_acquire);
        if (before & 1u) {
            continue;
        }
        Snapshot s{};
        s.count = w_.count.load(std::memory_order_relaxed);
        s.sum_us = w_.sum_us.load(std::memory_order_relaxed);
        s.sum_sq_us2 = w_.sum_sq_us2.load(std::memory_order_relaxed);
        s.overruns = w_.overruns.load(std::memory_order_relaxed);
        s.clamps = w_.clamps.load(std::memory_order_relaxed);
        s.min_ns = w_.min_ns.load(std::memory_order_relaxed);
        s.max_ns = w_.max_ns.load(std::memory_order_relaxed);
        s.max_lateness_ns = w_.max_lateness_ns.load(std::memory_order_relaxed);
        s.window = w_.window.load(std::memory_order_relaxed);
        std::atomic_thread_fence(std::memory_order_acquire);
        if (w_.seq.load(std::memory_order_relaxed) == before) {
            return s;
        }
    }
}

uint32_t PeriodStats::request_window_reset() {
    return r_.window_request.fetch_add(1, std::memory_order_relaxed) + 1;
}

// ---------------------------------------------------------------------------
// Reporting

Report summarize(const Snapshot& prev, const Snapshot& cur, uint32_t expected_window) {
    Report r{};
    r.count = cur.count - prev.count;
    r.elapsed_us = cur.sum_us - prev.sum_us;
    r.overruns = cur.overruns - prev.overruns;
    r.clamps = cur.clamps - prev.clamps;
    r.min_ns = cur.min_ns;
    r.max_ns = cur.max_ns;
    r.max_lateness_ns = cur.max_lateness_ns;
    r.stale = cur.window != expected_window;
    if (r.count == 0) {
        return r;
    }
    const double n = static_cast<double>(r.count);
    const double sum = static_cast<double>(cur.sum_us - prev.sum_us);
    const double sum_sq = static_cast<double>(cur.sum_sq_us2 - prev.sum_sq_us2);
    r.mean_us = sum / n;
    if (r.count >= 2) {
        // Population variance from the two moments; clamp the rounding residue
        // that can make it slightly negative when all samples are equal.
        const double var = sum_sq / n - r.mean_us * r.mean_us;
        r.std_us = var > 0.0 ? std::sqrt(var) : 0.0;
    }
    return r;
}

std::string describe(const Report& r, int64_t period_ns) {
    char buf[256];
    const double ms = 1e-6;
    if (r.count == 0) {
        std::snprintf(buf, sizeof(buf), "period %.2f ms: no cycles in this window%s", static_cast<double>(period_ns) * ms,
                      r.stale ? " (loop has not run since the last report)" : "");
        return buf;
    }
    std::snprintf(buf, sizeof(buf),
                  "period %.2f ms: n=%llu over %.2f s min/mean/max %.2f/%.2f/%.2f ms std %.3f ms, overruns %llu, "
                  "clamps %llu, max lateness %.2f ms%s",
                  static_cast<double>(period_ns) * ms, static_cast<unsigned long long>(r.count),
                  static_cast<double>(r.elapsed_us) * 1e-6, static_cast<double>(r.min_ns) * ms, r.mean_us * 1e-3, static_cast<double>(r.max_ns) * ms,
                  r.std_us * 1e-3, static_cast<unsigned long long>(r.overruns),
                  static_cast<unsigned long long>(r.clamps), static_cast<double>(r.max_lateness_ns) * ms,
                  r.stale ? " (extremes stale: loop has not run since the last report)" : "");
    return buf;
}

}  // namespace rt_timebase
}  // namespace ELITE_CS_ROBOT_ROS_DRIVER
