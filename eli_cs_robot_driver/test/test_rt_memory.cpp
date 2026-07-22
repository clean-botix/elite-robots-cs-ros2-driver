// Unit tests for the pure real-time memory helpers in rt_memory.hpp
// (RLIMIT_MEMLOCK formatting, std::terminate classification, and the heap
// pre-fault smoke path). The effectful orchestration in rt_memory.cpp
// (configure_realtime_memory's mlockall()/mallopt()/logging) is not covered here
// as it requires the deployed container environment; these tests link only the
// header, not rclcpp.
#include <exception>
#include <stdexcept>
#include <string>

#include <sys/resource.h>

#include <gtest/gtest.h>

#include "eli_cs_robot_driver/rt_memory.hpp"

namespace rt = ELITE_CS_ROBOT_ROS_DRIVER::rt_memory;

// --- describe_rlimit ---------------------------------------------------------

TEST(DescribeRlimit, InfinityRendersAsUnlimited) {
    EXPECT_EQ(rt::describe_rlimit(RLIM_INFINITY), "unlimited");
}

TEST(DescribeRlimit, ReportsWholeMiB) {
    EXPECT_EQ(rt::describe_rlimit(512UL * 1024 * 1024), "512 MiB");
    EXPECT_EQ(rt::describe_rlimit(1UL * 1024 * 1024), "1 MiB");
}

TEST(DescribeRlimit, ZeroIsZeroMiB) {
    // The Docker default (64 KiB) or an unset ulimit both floor to 0 MiB here,
    // which is exactly the "mlockall will fail" signal we want visible in logs.
    EXPECT_EQ(rt::describe_rlimit(0), "0 MiB");
    EXPECT_EQ(rt::describe_rlimit(65536), "0 MiB");
}

// --- classify_terminate ------------------------------------------------------

TEST(ClassifyTerminate, NullExceptionIsShutdownRace) {
    const auto info = rt::classify_terminate(nullptr);
    EXPECT_EQ(info.disposition, rt::TerminateDisposition::IgnoredShutdownRace);
    EXPECT_TRUE(info.detail.empty());
}

TEST(ClassifyTerminate, BadAllocIsFatal) {
    const auto info = rt::classify_terminate(std::make_exception_ptr(std::bad_alloc{}));
    EXPECT_EQ(info.disposition, rt::TerminateDisposition::FatalBadAlloc);
}

TEST(ClassifyTerminate, OtherStdExceptionIsFatalWithDetail) {
    const auto info =
        rt::classify_terminate(std::make_exception_ptr(std::runtime_error("boom")));
    EXPECT_EQ(info.disposition, rt::TerminateDisposition::FatalException);
    EXPECT_EQ(info.detail, "boom");
}

TEST(ClassifyTerminate, NonStdExceptionFallsThroughToShutdownRace) {
    // A non-std throwable (e.g. a bare int) is not identifiable; the handler must
    // treat it like the tolerated Humble shutdown race rather than crash.
    const auto info = rt::classify_terminate(std::make_exception_ptr(42));
    EXPECT_EQ(info.disposition, rt::TerminateDisposition::IgnoredShutdownRace);
}

// --- terminate_exit_code -----------------------------------------------------

TEST(TerminateExitCode, ShutdownRaceExitsClean) {
    EXPECT_EQ(rt::terminate_exit_code(rt::TerminateDisposition::IgnoredShutdownRace), 0);
}

TEST(TerminateExitCode, FatalCasesExitNonZero) {
    EXPECT_EQ(rt::terminate_exit_code(rt::TerminateDisposition::FatalBadAlloc), 1);
    EXPECT_EQ(rt::terminate_exit_code(rt::TerminateDisposition::FatalException), 1);
}

// --- reserve_process_memory --------------------------------------------------

TEST(ReserveProcessMemory, SmallReserveDoesNotCrash) {
    // Smoke test: a modest reserve should fault its pages and return cleanly.
    // (We can't portably assert RSS growth here.)
    rt::reserve_process_memory(1UL * 1024 * 1024); // 1 MiB
    SUCCEED();
}

TEST(ReserveProcessMemory, ZeroSizeIsSafe) {
    rt::reserve_process_memory(0);
    SUCCEED();
}
