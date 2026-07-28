// Unit tests for the real-time memory REPORTING helpers in
// rt_memory_reporting.hpp (page-fault report formatting, /proc/status field
// parsing, memory-footprint report formatting). RtMemoryMonitor::tick()'s actual
// getrusage()/file-read/logging behavior is not covered here (it needs rclcpp);
// these tests link only the header's pure functions.
#include <string>

#include <gtest/gtest.h>

#include "eli_cs_robot_driver/rt_memory_reporting.hpp"

namespace rt = ELITE_CS_ROBOT_ROS_DRIVER::rt_memory;

// --- format_fault_report -----------------------------------------------------

TEST(FormatFaultReport, ReportsDeltasTotalsAndRate) {
    const std::string s = rt::format_fault_report(/*minor_delta=*/12, /*major_delta=*/0,
                                                  /*minor_total=*/40000, /*major_total=*/3,
                                                  /*interval_seconds=*/10.0);
    EXPECT_NE(s.find("+0 major"), std::string::npos);
    EXPECT_NE(s.find("+12 minor"), std::string::npos);
    EXPECT_NE(s.find("over 10s"), std::string::npos);
    EXPECT_NE(s.find("0.00 major/s"), std::string::npos);
    EXPECT_NE(s.find("lifetime 3 major / 40000 minor"), std::string::npos);
}

TEST(FormatFaultReport, ComputesMajorRate) {
    const std::string s = rt::format_fault_report(5, 4, 100, 8, 2.0);
    EXPECT_NE(s.find("+4 major"), std::string::npos);
    EXPECT_NE(s.find("2.00 major/s"), std::string::npos);
}

TEST(FormatFaultReport, ZeroIntervalDoesNotDivideByZero) {
    const std::string s = rt::format_fault_report(1, 1, 1, 1, 0.0);
    EXPECT_NE(s.find("0.00 major/s"), std::string::npos);
}

// --- parse_status_kb / format_memory_report ----------------------------------

namespace {
const char* const kSampleStatus =
    "Name:\tcontrol_node\n"
    "State:\tR (running)\n"
    "VmPeak:\t  870400 kB\n"
    "VmSize:\t  860160 kB\n"
    "VmHWM:\t  245760 kB\n"
    "VmRSS:\t  243712 kB\n"
    "VmLck:\t  243712 kB\n"
    "Threads:\t12\n";
}  // namespace

TEST(ParseStatusKb, ReadsNamedFields) {
    EXPECT_EQ(rt::parse_status_kb(kSampleStatus, "VmHWM"), 245760);
    EXPECT_EQ(rt::parse_status_kb(kSampleStatus, "VmRSS"), 243712);
    EXPECT_EQ(rt::parse_status_kb(kSampleStatus, "VmLck"), 243712);
}

TEST(ParseStatusKb, MissingKeyReturnsMinusOne) {
    EXPECT_EQ(rt::parse_status_kb(kSampleStatus, "NoSuchKey"), -1);
    EXPECT_EQ(rt::parse_status_kb("", "VmRSS"), -1);
}

TEST(FormatMemoryReport, ConvertsKbToMiB) {
    // 245760 kB = 240 MiB, 243712 kB = 238 MiB
    const std::string s = rt::format_memory_report(243712, 243712, 245760);
    EXPECT_NE(s.find("RSS=238 MiB"), std::string::npos);
    EXPECT_NE(s.find("locked=238 MiB"), std::string::npos);
    EXPECT_NE(s.find("peak RSS(VmHWM)=240 MiB"), std::string::npos);
}

TEST(FormatMemoryReport, MissingFieldRendersAsMinusOne) {
    const std::string s = rt::format_memory_report(-1, -1, -1);
    EXPECT_NE(s.find("RSS=-1 MiB"), std::string::npos);
}

// --- kDefaultLogIntervalSeconds -----------------------------------------------

TEST(DefaultLogIntervalSeconds, MatchesDocumentedDefault) {
    EXPECT_DOUBLE_EQ(rt::kDefaultLogIntervalSeconds, 30.0);
}
