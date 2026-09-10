// Unit tests for the control loop TIMEBASE helpers in rt_timebase.hpp: grid
// arithmetic, wake-up classification and the overrun resync rule, the measured
// period clamp, the single-writer PeriodStats accumulator (including a torn-read
// check against a concurrent writer), the reporter summary, and the two
// CLOCK_MONOTONIC wrappers. No rclcpp dependency, like rt_timebase.hpp/.cpp.
#include <atomic>
#include <cmath>
#include <cstdint>
#include <string>
#include <thread>

#include <gtest/gtest.h>

#include "eli_cs_robot_driver/rt_timebase.hpp"

namespace tb = ELITE_CS_ROBOT_ROS_DRIVER::rt_timebase;

namespace {

constexpr int64_t kMs = 1'000'000;
constexpr int64_t kPeriod50Hz = 20 * kMs;

timespec ts(time_t sec, long nsec) {
    timespec t{};
    t.tv_sec = sec;
    t.tv_nsec = nsec;
    return t;
}

tb::CycleSample sample(int64_t period_ns, int64_t lateness_ns = 0, bool overrun = false, bool clamped = false) {
    tb::CycleSample s{};
    s.period_ns = period_ns;
    s.lateness_ns = lateness_ns;
    s.overrun = overrun;
    s.clamped = clamped;
    return s;
}

}  // namespace

// --- grid arithmetic ----------------------------------------------------------

TEST(PeriodFromRate, FiftyHertzIsTwentyMilliseconds) {
    EXPECT_EQ(tb::period_ns_from_rate(50), kPeriod50Hz);
    EXPECT_EQ(tb::period_ns_from_rate(500), 2 * kMs);
    EXPECT_EQ(tb::period_ns_from_rate(0), 0);  // caller must reject
}

TEST(AddNs, CarriesAcrossTheSecondBoundary) {
    const timespec n = tb::add_ns(ts(5, 990 * kMs), 20 * kMs);
    EXPECT_EQ(n.tv_sec, 6);
    EXPECT_EQ(n.tv_nsec, 10 * kMs);
}

TEST(AddNs, NegativeOffsetBorrowsASecond) {
    const timespec b = tb::add_ns(ts(6, 10 * kMs), -20 * kMs);
    EXPECT_EQ(b.tv_sec, 5);
    EXPECT_EQ(b.tv_nsec, 990 * kMs);
}

TEST(AddNs, ExactSecondsAndMultiSecondOffsets) {
    const timespec a = tb::add_ns(ts(1, 0), 3 * tb::kNsPerSec);
    EXPECT_EQ(a.tv_sec, 4);
    EXPECT_EQ(a.tv_nsec, 0);
    const timespec b = tb::add_ns(ts(1, 999'999'999), 1);
    EXPECT_EQ(b.tv_sec, 2);
    EXPECT_EQ(b.tv_nsec, 0);
    const timespec c = tb::add_ns(ts(10, 5), -(2 * tb::kNsPerSec + 10));
    EXPECT_EQ(c.tv_sec, 7);
    EXPECT_EQ(c.tv_nsec, 999'999'995);
}

TEST(DiffNs, IsSignedAndSpansSeconds) {
    EXPECT_EQ(tb::diff_ns(ts(6, 10 * kMs), ts(5, 990 * kMs)), 20 * kMs);
    EXPECT_EQ(tb::diff_ns(ts(5, 990 * kMs), ts(6, 10 * kMs)), -20 * kMs);
    EXPECT_EQ(tb::diff_ns(ts(7, 0), ts(7, 0)), 0);
}

TEST(NextDeadline, FiftyCyclesLandExactlyOneSecondLater) {
    timespec d = ts(100, 0);
    for (int i = 0; i < 50; ++i) {
        d = tb::next_deadline(d, kPeriod50Hz);
    }
    EXPECT_EQ(d.tv_sec, 101);
    EXPECT_EQ(d.tv_nsec, 0);
}

// --- classify -------------------------------------------------------------------

TEST(Classify, OnTimeAtOrBeforeTheDeadline) {
    const timespec d = ts(10, 0);
    auto c = tb::classify(d, d, kPeriod50Hz);
    EXPECT_EQ(c.verdict, tb::Verdict::OnTime);
    EXPECT_EQ(c.lateness_ns, 0);
    c = tb::classify(tb::add_ns(d, -50'000), d, kPeriod50Hz);  // woke 50 us early
    EXPECT_EQ(c.verdict, tb::Verdict::OnTime);
    EXPECT_EQ(c.lateness_ns, -50'000);
}

TEST(Classify, LateBelowOnePeriod) {
    const timespec d = ts(10, 0);
    auto c = tb::classify(tb::add_ns(d, 5 * kMs), d, kPeriod50Hz);
    EXPECT_EQ(c.verdict, tb::Verdict::Late);
    EXPECT_EQ(c.lateness_ns, 5 * kMs);
    c = tb::classify(tb::add_ns(d, 19'900'000), d, kPeriod50Hz);
    EXPECT_EQ(c.verdict, tb::Verdict::Late);
    EXPECT_EQ(c.lateness_ns, 19'900'000);
}

TEST(Classify, OverrunAtOnePeriodAndBeyond) {
    const timespec d = ts(10, 0);
    auto c = tb::classify(tb::add_ns(d, kPeriod50Hz), d, kPeriod50Hz);
    EXPECT_EQ(c.verdict, tb::Verdict::Overrun);
    EXPECT_EQ(c.lateness_ns, kPeriod50Hz);
    c = tb::classify(tb::add_ns(d, 100 * kMs), d, kPeriod50Hz);
    EXPECT_EQ(c.verdict, tb::Verdict::Overrun);
    EXPECT_EQ(c.lateness_ns, 100 * kMs);
}

// --- step: the resync rule ------------------------------------------------------

TEST(Step, OnTimeAdvancesTheGridByOnePeriod) {
    const timespec d = ts(10, 0);
    const auto s = tb::step(d, tb::add_ns(d, -100'000), kPeriod50Hz);
    EXPECT_EQ(s.wake.verdict, tb::Verdict::OnTime);
    EXPECT_EQ(tb::diff_ns(s.next_deadline, d), kPeriod50Hz);
}

TEST(Step, LateKeepsTheOriginalGrid) {
    // 5 ms late: the next deadline is still deadline + period, so the next sleep
    // is 15 ms and the loop is back on the grid without any burst.
    const timespec d = ts(10, 0);
    const timespec now = tb::add_ns(d, 5 * kMs);
    const auto s = tb::step(d, now, kPeriod50Hz);
    EXPECT_EQ(s.wake.verdict, tb::Verdict::Late);
    EXPECT_EQ(tb::diff_ns(s.next_deadline, d), kPeriod50Hz);
    EXPECT_EQ(tb::diff_ns(s.next_deadline, now), 15 * kMs);
}

TEST(Step, OverrunResyncsTheGridToNowPlusOnePeriod) {
    // Stalled 100 ms: five cycles were missed. The next deadline is one period
    // after the stall ended, not five back-to-back catch-up cycles.
    const timespec d = ts(10, 0);
    const timespec now = tb::add_ns(d, 100 * kMs);
    const auto s = tb::step(d, now, kPeriod50Hz);
    EXPECT_EQ(s.wake.verdict, tb::Verdict::Overrun);
    EXPECT_EQ(s.wake.lateness_ns, 100 * kMs);
    EXPECT_EQ(tb::diff_ns(s.next_deadline, now), kPeriod50Hz);
    EXPECT_GT(tb::diff_ns(s.next_deadline, d), 5 * kPeriod50Hz);
}

TEST(Step, ASimulatedStallProducesExactlyOneOverrunAndNoBurst) {
    // Drive the rule the way the loop will: the thread is blocked for 100 ms
    // once, then wakes exactly on every deadline. Expect one Overrun, then only
    // OnTime cycles spaced one period apart.
    const int64_t P = kPeriod50Hz;
    timespec deadline = ts(10, 0);
    timespec now = deadline;
    int overruns = 0;
    int64_t min_gap = INT64_MAX;
    timespec prev_wake = now;
    for (int i = 0; i < 20; ++i) {
        if (i == 5) {
            now = tb::add_ns(deadline, 100 * kMs);  // the stall
        } else {
            now = deadline;                          // perfect wake-up
        }
        const auto s = tb::step(deadline, now, P);
        if (s.wake.verdict == tb::Verdict::Overrun) {
            ++overruns;
            EXPECT_EQ(s.wake.lateness_ns, 100 * kMs);
        }
        if (i > 0) {
            min_gap = std::min(min_gap, tb::diff_ns(now, prev_wake));
        }
        prev_wake = now;
        deadline = s.next_deadline;
    }
    EXPECT_EQ(overruns, 1);
    EXPECT_EQ(min_gap, P);  // never two cycles closer than one period
}

// --- clamp_measured_period ------------------------------------------------------

TEST(ClampMeasuredPeriod, NominalAndModeratelyLateValuesPassThrough) {
    auto p = tb::clamp_measured_period(kPeriod50Hz, kPeriod50Hz);
    EXPECT_EQ(p.ns, kPeriod50Hz);
    EXPECT_FALSE(p.clamped);
    p = tb::clamp_measured_period(tb::kMaxMeasuredPeriods * kPeriod50Hz, kPeriod50Hz);
    EXPECT_EQ(p.ns, tb::kMaxMeasuredPeriods * kPeriod50Hz);
    EXPECT_FALSE(p.clamped);  // the bound itself is allowed
}

TEST(ClampMeasuredPeriod, AStallIsClampedToTenPeriods) {
    const auto p = tb::clamp_measured_period(900 * kMs, kPeriod50Hz);  // the incident's 0.9 s
    EXPECT_EQ(p.ns, 200 * kMs);
    EXPECT_TRUE(p.clamped);
}

TEST(ClampMeasuredPeriod, NegativeBecomesZero) {
    const auto p = tb::clamp_measured_period(-5, kPeriod50Hz);
    EXPECT_EQ(p.ns, 0);
    EXPECT_TRUE(p.clamped);
}

// --- PeriodStats ----------------------------------------------------------------

TEST(PeriodStats, FreshInstanceIsEmpty) {
    tb::PeriodStats st;
    const auto s = st.snapshot();
    EXPECT_EQ(s.count, 0u);
    EXPECT_EQ(s.sum_us, 0);
    EXPECT_EQ(s.sum_sq_us2, 0u);
    EXPECT_EQ(s.overruns, 0u);
    EXPECT_EQ(s.clamps, 0u);
    EXPECT_EQ(s.window, 0u);
}

TEST(PeriodStats, MeanAndStdFromAKnownSequence) {
    tb::PeriodStats st;
    const auto before = st.snapshot();
    for (int64_t ms : {19, 20, 21}) {
        st.record(sample(ms * kMs, ms == 21 ? kMs : -500'000));
    }
    const auto r = tb::summarize(before, st.snapshot(), 0);
    EXPECT_EQ(r.count, 3u);
    EXPECT_EQ(r.elapsed_us, 60'000);  // 19 + 20 + 21 ms
    EXPECT_NEAR(r.mean_us, 20'000.0, 1e-9);
    EXPECT_NEAR(r.std_us, 816.4966, 1e-3);  // population std of {19,20,21} ms
    EXPECT_EQ(r.min_ns, 19 * kMs);
    EXPECT_EQ(r.max_ns, 21 * kMs);
    EXPECT_EQ(r.max_lateness_ns, kMs);
    EXPECT_EQ(r.overruns, 0u);
    EXPECT_EQ(r.clamps, 0u);
    EXPECT_FALSE(r.stale);
}

TEST(PeriodStats, IdenticalSamplesHaveZeroStd) {
    tb::PeriodStats st;
    for (int i = 0; i < 1000; ++i) {
        st.record(sample(kPeriod50Hz + 900));  // 20.0009 ms -> rounds to 20001 us
    }
    const auto r = tb::summarize(tb::Snapshot{}, st.snapshot(), 0);
    EXPECT_EQ(r.count, 1000u);
    EXPECT_NEAR(r.mean_us, 20'001.0, 1e-9);
    EXPECT_DOUBLE_EQ(r.std_us, 0.0);  // the rounding residue must be clamped, never NaN
}

TEST(PeriodStats, OverrunsAndClampsAreCounted) {
    tb::PeriodStats st;
    st.record(sample(kPeriod50Hz));
    st.record(sample(120 * kMs, 100 * kMs, /*overrun=*/true));
    st.record(sample(900 * kMs, 880 * kMs, /*overrun=*/true, /*clamped=*/true));
    const auto s = st.snapshot();
    EXPECT_EQ(s.count, 3u);
    EXPECT_EQ(s.overruns, 2u);
    EXPECT_EQ(s.clamps, 1u);
    EXPECT_EQ(s.max_lateness_ns, 880 * kMs);
    EXPECT_EQ(s.max_ns, 900 * kMs);
}

TEST(PeriodStats, CountersAreMonotonicAndTheReporterTakesDeltas) {
    tb::PeriodStats st;
    for (int i = 0; i < 10; ++i) {
        st.record(sample(kPeriod50Hz, 0, i == 3));
    }
    const auto first = st.snapshot();
    for (int i = 0; i < 5; ++i) {
        st.record(sample(kPeriod50Hz));
    }
    const auto second = st.snapshot();
    EXPECT_EQ(second.count, 15u);  // never reset
    EXPECT_EQ(second.overruns, 1u);
    const auto r = tb::summarize(first, second, 0);
    EXPECT_EQ(r.count, 5u);
    EXPECT_EQ(r.elapsed_us, 5 * 20'000);  // only this window's cycles
    EXPECT_EQ(r.overruns, 0u);  // the overrun was in the previous window
}

TEST(PeriodStats, WindowResetRestartsTheExtremesFromTheNextSample) {
    tb::PeriodStats st;
    st.record(sample(19 * kMs, 2 * kMs));
    st.record(sample(21 * kMs));
    auto prev = st.snapshot();
    EXPECT_EQ(prev.window, 0u);

    const uint32_t win = st.request_window_reset();
    EXPECT_EQ(win, 1u);

    // Before the loop runs again the extremes are still the old window's and
    // the summary says so, with zero cycles rather than stale numbers.
    auto r = tb::summarize(prev, st.snapshot(), win);
    EXPECT_EQ(r.count, 0u);
    EXPECT_TRUE(r.stale);

    // One cycle later the new window starts from that sample alone.
    st.record(sample(20'500'000, -100'000));
    const auto cur = st.snapshot();
    EXPECT_EQ(cur.window, win);
    r = tb::summarize(prev, cur, win);
    EXPECT_FALSE(r.stale);
    EXPECT_EQ(r.count, 1u);
    EXPECT_EQ(r.min_ns, 20'500'000);
    EXPECT_EQ(r.max_ns, 20'500'000);
    EXPECT_EQ(r.max_lateness_ns, -100'000);  // no longer the 2 ms from before
    EXPECT_EQ(cur.count, 3u);                 // monotonic counters untouched by the reset
}

TEST(PeriodStats, TwoResetRequestsBeforeTheLoopRunsCollapseIntoOne) {
    tb::PeriodStats st;
    st.record(sample(kPeriod50Hz));
    st.request_window_reset();
    const uint32_t win = st.request_window_reset();
    EXPECT_EQ(win, 2u);
    st.record(sample(kPeriod50Hz));
    EXPECT_EQ(st.snapshot().window, win);  // the loop reports the latest request
}

TEST(PeriodStats, SnapshotWhileWritingIsNeverTorn) {
    // A writer thread records a constant sample as fast as it can. Every
    // snapshot must satisfy the per-sample invariants exactly: a torn read
    // (count from one record, sums from another) would break them.
    tb::PeriodStats st;
    std::atomic<bool> stop{false};
    std::thread writer([&] {
        const auto s = sample(kPeriod50Hz, 100'000);
        while (!stop.load(std::memory_order_relaxed)) {
            st.record(s);
        }
    });
    // ASSERT_* returns from the test body early; destroying a joinable thread
    // then calls std::terminate and takes the whole binary down instead of
    // reporting one failure. Stop and join on every exit path.
    struct StopAndJoin {
        std::atomic<bool>& stop;
        std::thread& thread;
        ~StopAndJoin() {
            stop.store(true);
            if (thread.joinable()) {
                thread.join();
            }
        }
    } guard{stop, writer};
    uint64_t last_count = 0;
    for (int i = 0; i < 100'000; ++i) {
        const auto s = st.snapshot();
        ASSERT_EQ(s.sum_us, static_cast<int64_t>(s.count) * 20'000);
        ASSERT_EQ(s.sum_sq_us2, s.count * 400'000'000ull);
        ASSERT_GE(s.count, last_count);
        ASSERT_EQ(s.overruns, 0u);
        last_count = s.count;
    }
    EXPECT_GT(last_count, 0u);
}

TEST(PeriodStats, HugeSingleGapDoesNotOverflowTheSquaredSum) {
    // A gap of 2 hours: us^2 = 5.2e19 wraps uint64 but must not be signed UB.
    tb::PeriodStats st;
    st.record(sample(int64_t{7200} * tb::kNsPerSec));
    const auto s = st.snapshot();
    EXPECT_EQ(s.count, 1u);
    EXPECT_EQ(s.sum_us, int64_t{7200} * 1'000'000);
    EXPECT_EQ(s.max_ns, int64_t{7200} * tb::kNsPerSec);
}

TEST(PeriodStats, IsCacheLineAlignedAndFixedSize) {
    // Design decision 3: nothing else may share the loop's cache lines.
    EXPECT_EQ(alignof(tb::PeriodStats), 64u);
    EXPECT_EQ(sizeof(tb::PeriodStats) % 64, 0u);
}

// --- summarize / describe -------------------------------------------------------

TEST(Summarize, EmptyWindowReportsZeroCountAndNoNaN) {
    tb::Snapshot a{};
    a.count = 7;
    const auto r = tb::summarize(a, a, 0);
    EXPECT_EQ(r.count, 0u);
    EXPECT_DOUBLE_EQ(r.mean_us, 0.0);
    EXPECT_DOUBLE_EQ(r.std_us, 0.0);
}

TEST(Summarize, SingleSampleHasMeanButNoStd) {
    tb::PeriodStats st;
    st.record(sample(21 * kMs));
    const auto r = tb::summarize(tb::Snapshot{}, st.snapshot(), 0);
    EXPECT_EQ(r.count, 1u);
    EXPECT_NEAR(r.mean_us, 21'000.0, 1e-9);
    EXPECT_DOUBLE_EQ(r.std_us, 0.0);
}

TEST(Describe, MentionsEveryStatistic) {
    tb::Report r{};
    r.count = 1500;
    r.elapsed_us = 30'000'000;
    r.overruns = 2;
    r.clamps = 1;
    r.mean_us = 20'000.0;
    r.std_us = 50.0;
    r.min_ns = 19'810'000;
    r.max_ns = 120 * kMs;
    r.max_lateness_ns = 100 * kMs;
    const std::string line = tb::describe(r, kPeriod50Hz);
    EXPECT_NE(line.find("period 20.00 ms"), std::string::npos) << line;
    EXPECT_NE(line.find("n=1500 over 30.00 s"), std::string::npos) << line;
    EXPECT_NE(line.find("19.81/20.00/120.00 ms"), std::string::npos) << line;
    EXPECT_NE(line.find("std 0.050 ms"), std::string::npos) << line;
    EXPECT_NE(line.find("overruns 2"), std::string::npos) << line;
    EXPECT_NE(line.find("clamps 1"), std::string::npos) << line;
    EXPECT_NE(line.find("max lateness 100.00 ms"), std::string::npos) << line;
    EXPECT_EQ(line.find("stale"), std::string::npos) << line;
}

TEST(Describe, StretchedWindowReadsAsItsTrueSpan) {
    // A wall-clock step stretched the executor's 5 s report window to 7 s: the
    // line must say 350 cycles over 7.00 s, so nobody reads it as a fast loop.
    tb::Report r{};
    r.count = 350;
    r.elapsed_us = 7'000'000;
    r.mean_us = 20'000.0;
    r.min_ns = 19 * kMs;
    r.max_ns = 21 * kMs;
    EXPECT_NE(tb::describe(r, kPeriod50Hz).find("n=350 over 7.00 s"), std::string::npos);
}

TEST(Describe, EmptyAndStaleWindowsSaySo) {
    tb::Report empty{};
    EXPECT_NE(tb::describe(empty, kPeriod50Hz).find("no cycles"), std::string::npos);
    empty.stale = true;
    EXPECT_NE(tb::describe(empty, kPeriod50Hz).find("loop has not run"), std::string::npos);
    tb::Report stale{};
    stale.count = 5;
    stale.stale = true;
    EXPECT_NE(tb::describe(stale, kPeriod50Hz).find("extremes stale"), std::string::npos);
}

// --- CLOCK_MONOTONIC wrappers ---------------------------------------------------

TEST(Monotonic, NowIsNormalisedAndAdvances) {
    const timespec a = tb::now_monotonic();
    EXPECT_GE(a.tv_nsec, 0);
    EXPECT_LT(a.tv_nsec, tb::kNsPerSec);
    const timespec b = tb::now_monotonic();
    EXPECT_GE(tb::diff_ns(b, a), 0);
}

TEST(Monotonic, SleepUntilHonoursAnAbsoluteDeadline) {
    // Not a latency test (CI is not real-time): only that the call returns
    // success, never early, and within a generous bound.
    const timespec start = tb::now_monotonic();
    const timespec deadline = tb::add_ns(start, 5 * kMs);
    EXPECT_EQ(tb::sleep_until_monotonic(deadline), 0);
    const timespec woke = tb::now_monotonic();
    EXPECT_GE(tb::diff_ns(woke, deadline), 0);
    EXPECT_LT(tb::diff_ns(woke, start), 500 * kMs);
}

TEST(Monotonic, SleepUntilAPastDeadlineReturnsImmediately) {
    const timespec start = tb::now_monotonic();
    EXPECT_EQ(tb::sleep_until_monotonic(tb::add_ns(start, -tb::kNsPerSec)), 0);
    EXPECT_LT(tb::diff_ns(tb::now_monotonic(), start), 100 * kMs);
}
