#include <chrono>
#include <memory>
#include <thread>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <exception>
#include <string>

#include <unistd.h>

#include <rclcpp/rclcpp.hpp>

// Real-time memory MANAGEMENT: pure, unit-tested policy (classify_terminate)
// plus configure_realtime_memory(), which does the mlockall()/mallopt()/reserve
// setup. ROS-free (see rt_memory.hpp); this file does the rclcpp logging around
// its result. See test/test_rt_memory.cpp.
#include "eli_cs_robot_driver/rt_memory.hpp"
// Real-time memory REPORTING: the periodic page-fault/memory-footprint monitor.
// See test/test_rt_memory_reporting.cpp.
#include "eli_cs_robot_driver/rt_memory_reporting.hpp"
// Real-time SCHEDULING of the control loop thread: SCHED_FIFO priority and CPU
// pin from the rt_sched.* parameters. ROS-free like rt_memory (see rt_sched.hpp);
// this file logs its result. See test/test_rt_sched.cpp.
#include "eli_cs_robot_driver/rt_sched.hpp"
// Real-time TIMEBASE of the control loop: the CLOCK_MONOTONIC deadline grid,
// overrun classification/resync, and the lock-free PeriodStats the loop fills
// for a reporter on the executor thread. ROS-free (see rt_timebase.hpp).
// See test/test_rt_timebase.cpp.
#include "eli_cs_robot_driver/rt_timebase.hpp"

// Rely on a subclass of ControllerManager to intercept the pre-shutdown hook
// and execute a workaround for a ROS2 Humble bug to ensure orderly shutdown at termination.
#include "eli_cs_robot_driver/elite_controller_manager.hpp"

// Elite code is inspired by:
// https://github.com/ros-controls/ros2_control/blob/master/controller_manager/src/ros2_control_node.cpp

std::atomic<int> exit_code{0};

// Read an rt_* tuning parameter, declaring it with `default_value` only if it is
// not already declared. The controller manager is built with
// automatically_declare_parameters_from_overrides (controller_manager::get_cm_node_options),
// so any parameter supplied by the launch file / -p is ALREADY declared by the
// time main() runs, and a plain declare_parameter() throws
// ParameterAlreadyDeclaredException -- which std::terminate()s the node before
// the control loop starts. A value of the wrong type (e.g. 80.0 for an int) is
// also just a tuning mistake: warn and use the default rather than abort.
template <typename T>
T rt_param_or_default(rclcpp::Node& node, const std::string& name, T default_value) {
    try {
        if (!node.has_parameter(name)) {
            node.declare_parameter<T>(name, default_value);
        }
        return node.get_parameter(name).get_value<T>();
    } catch (const std::exception& ex) {
        RCLCPP_WARN(node.get_logger(), "Parameter '%s' unusable (%s); using default",
            name.c_str(), ex.what());
        return default_value;
    }
}

void signal_handler(int signal) {
    // SIGUSR1 is our custom error-exit signal. Only set the exit code here.
    // rclcpp::shutdown() is not async-signal-safe and must not be called from a signal handler.
    // The control loop calls it directly after detecting the error (std::raise returns synchronously).
    if (signal == SIGUSR1) {
        exit_code = 1;
    }
}

int main(int argc, char** argv) {
    // Only register a handler for our custom error signal. SIGINT and SIGTERM
    // are left to rclcpp::init()'s safe handlers — overriding them here and
    // calling rclcpp::shutdown() from signal context would be unsafe
    // (not async-signal-safe) and can crash MultiThreadedExecutor.
    std::signal(SIGUSR1, signal_handler);

    std::set_terminate([]() {
        // Classify the exception (if any) in flight. A bad_alloc here is the
        // signature of a memory failure the mlockall()/RLIMIT_MEMLOCK handling
        // can bring on (memlock ceiling or exhaustion); surface it as a fatal,
        // non-zero exit rather than silently swallowing it. An unidentifiable
        // exception matches the known ROS2 Humble CM shutdown race this handler
        // was originally added to absorb, and exits cleanly. Classification is
        // factored into rt_memory so its policy is unit-tested (see rt_memory.hpp).
        namespace rt = ELITE_CS_ROBOT_ROS_DRIVER::rt_memory;
        const rt::TerminateInfo info = rt::classify_terminate(std::current_exception());
        switch (info.disposition) {
            case rt::TerminateDisposition::FatalBadAlloc:
                fprintf(stderr, "[ControllerManager] terminate() from bad_alloc "
                    "-- likely a memlock/RLIMIT_MEMLOCK ceiling or memory exhaustion\n");
                break;
            case rt::TerminateDisposition::FatalException:
                fprintf(stderr, "[ControllerManager] terminate() from unexpected exception: %s\n",
                    info.detail.c_str());
                break;
            case rt::TerminateDisposition::IgnoredShutdownRace:
                fprintf(stderr, "[ControllerManager] ignoring terminate() -- likely CM shutdown race in ROS2 Humble\n");
                break;
        }
        _exit(rt::terminate_exit_code(info.disposition)); // no core dump
    });

    rclcpp::init(argc, argv);
    rclcpp::install_signal_handlers(); // Ensures SIGINT and SIGTERM handled

    // Create Executor
    std::shared_ptr<rclcpp::Executor> executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    // Create Controller Manager
    std::shared_ptr<ELITE_CS_ROBOT_ROS_DRIVER::EliteControllerManager> controller_manager;

    try {
        // create controller manager instance
        controller_manager = std::make_shared<ELITE_CS_ROBOT_ROS_DRIVER::EliteControllerManager>(executor, "controller_manager");
    } catch (const std::exception& ex) {
        RCLCPP_FATAL(
            rclcpp::get_logger("controller_manager"),
            "Exception during controller manager creation: %s", ex.what()
        );
        rclcpp::shutdown();
        return 1;
    } catch (...) {
        RCLCPP_FATAL(rclcpp::get_logger("controller_manager"), "Unknown exception during controller manager creation");
        rclcpp::shutdown();
        return 1;
    }

    // Add node before starting the control loop thread.
    // controller_manager->now() requires the node to be associated with an executor.
    executor->add_node(controller_manager);

    namespace rt = ELITE_CS_ROBOT_ROS_DRIVER::rt_memory;

    // RT memory tuning is exposed as ROS2 parameters (siblings of the node's
    // other configuration, e.g. update_rate) rather than environment variables,
    // so they can be set the same way as everything else the node reads --
    // config/rt_memory.yaml, a launch override, or --ros-args -p -- without a
    // recompile. Declared/read once here (main thread) and passed into the
    // control loop by value. Declared via rt_param_or_default (see above): the CM
    // auto-declares anything passed from launch, so an unconditional
    // declare_parameter() here threw and killed the node whenever a launch file
    // actually set one of these.
    const double heap_reserve_mb = rt_param_or_default<double>(*controller_manager,
        "rt_memory.heap_reserve_mb",
        static_cast<double>(rt::kDefaultHeapReserveBytes) / (1024.0 * 1024.0));
    const double log_interval_sec = rt_param_or_default<double>(*controller_manager,
        "rt_memory.log_interval_sec", rt::kDefaultLogIntervalSeconds);
    const std::size_t heap_reserve_bytes =
        static_cast<std::size_t>(heap_reserve_mb * 1024.0 * 1024.0);

    // Same treatment for the control loop's scheduling: SCHED_FIFO priority and
    // the CPU it is pinned to are parameters so each deployed node (arm, hose
    // reel) can be placed in the host's priority ladder from its launch file.
    // Defaults reproduce the previous compiled-in behaviour (FIFO 50, no pin).
    // Range/fallback policy lives in rt_sched.hpp and is unit-tested.
    namespace rts = ELITE_CS_ROBOT_ROS_DRIVER::rt_sched;
    const int rt_priority =
        rt_param_or_default<int>(*controller_manager, "rt_sched.priority", rts::kDefaultPriority);
    const int rt_cpu = rt_param_or_default<int>(*controller_manager, "rt_sched.cpu", rts::kNoCpuPin);

    // Control loop timebase. The nominal period comes from the controller
    // manager's update_rate; the loop schedules on CLOCK_MONOTONIC (see the loop
    // below and rt_timebase.hpp).
    namespace rttb = ELITE_CS_ROBOT_ROS_DRIVER::rt_timebase;
    const int64_t period_ns = rttb::period_ns_from_rate(controller_manager->get_update_rate());
    if (period_ns <= 0) {
        RCLCPP_FATAL(controller_manager->get_logger(),
            "update_rate %u Hz is invalid; control loop not started", controller_manager->get_update_rate());
        rclcpp::shutdown();
        return 1;
    }

    // Period statistics: written by the loop thread once per cycle, read by the
    // reporter below on the executor thread (see rt_timebase.hpp for the
    // threading model). Owned here so both sides can reach it.
    auto period_stats = std::make_shared<rttb::PeriodStats>();

    // Timing report, OFF the loop thread: a wall timer on the controller manager
    // node (so it runs in executor->spin(), never on the SCHED_FIFO loop)
    // snapshots PeriodStats every rt_timebase.report_interval_sec seconds and
    // logs one INFO line with the measured period's min/mean/max/std, the
    // overrun and clamp counts, and the largest lateness since the previous
    // report. Counters are monotonic and differenced here; the min/max window is
    // restarted by asking the loop for a new window (request_window_reset). A
    // value <= 0 disables the report; the loop keeps recording either way.
    const double report_interval_sec = rt_param_or_default<double>(*controller_manager,
        "rt_timebase.report_interval_sec", rttb::kDefaultReportIntervalSeconds);
    rclcpp::TimerBase::SharedPtr timebase_report_timer;
    if (report_interval_sec > 0.0) {
        struct ReporterState {
            rttb::Snapshot previous;
            uint32_t expected_window = 0;
        };
        auto state = std::make_shared<ReporterState>();
        timebase_report_timer = controller_manager->create_wall_timer(
            std::chrono::duration<double>(report_interval_sec),
            [controller_manager, period_stats, state, period_ns]() {
                // Snapshot, then IMMEDIATELY open the next extremes window, so the
                // cycles the loop records while this callback formats and writes
                // the log line (milliseconds when the launch stdout pipe is busy)
                // belong to the window they are counted in. The only remaining gap
                // is a record() in flight between the two calls: at most one cycle.
                const rttb::Snapshot current = period_stats->snapshot();
                const uint32_t reported_window = state->expected_window;
                state->expected_window = period_stats->request_window_reset();
                const rttb::Report report = rttb::summarize(state->previous, current, reported_window);
                state->previous = current;
                RCLCPP_INFO(controller_manager->get_logger(), "Control loop timing: %s",
                    rttb::describe(report, period_ns).c_str());
            });
        RCLCPP_INFO(controller_manager->get_logger(),
            "Control loop timing report: every %.0fs (rt_timebase.report_interval_sec)", report_interval_sec);
    } else {
        RCLCPP_INFO(controller_manager->get_logger(),
            "Control loop timing report: disabled (rt_timebase.report_interval_sec <= 0)");
    }

    // Control loop thread
    std::thread control_loop([controller_manager, heap_reserve_bytes, log_interval_sec,
                              rt_priority, rt_cpu, period_stats, period_ns]() {
        // Pin (if asked) and switch this thread to SCHED_FIFO. configure_realtime_sched
        // never aborts: a bad tuning value or a refused syscall degrades to the
        // old behaviour and is logged, it must not take the drivers down.
        namespace rts = ELITE_CS_ROBOT_ROS_DRIVER::rt_sched;
        const rts::RtSchedSetup sched = rts::configure_realtime_sched(rt_cpu, rt_priority);
        if (!sched.priority_valid) {
            RCLCPP_WARN(controller_manager->get_logger(),
                "rt_sched.priority %d is outside %d..%d; using default %d",
                sched.requested_priority, rts::kMinPriority, rts::kMaxPriority,
                sched.applied_priority);
        }
        if (sched.affinity_attempted && !sched.affinity_succeeded) {
            RCLCPP_WARN(controller_manager->get_logger(),
                "Could not pin control thread to CPU %d (%s); affinity left unchanged",
                sched.requested_cpu, sched.affinity_error.c_str());
        }
        if (!sched.sched_succeeded) {
            RCLCPP_WARN(controller_manager->get_logger(),
                "Could not enable FIFO RT scheduling policy (priority %d): %s",
                sched.applied_priority, std::strerror(sched.sched_errno));
        }
        RCLCPP_INFO(controller_manager->get_logger(), "%s", rts::describe(sched).c_str());

        namespace rt = ELITE_CS_ROBOT_ROS_DRIVER::rt_memory;

        // Lock the process into RAM and pre-fault a heap reserve so the RT loop
        // doesn't page-fault under memory pressure. configure_realtime_memory is
        // ROS-free (see rt_memory.hpp) and returns raw facts; this is the logging
        // half of what mlockall/mallopt/rlimit setup needs.
        const rt::RealtimeMemorySetup setup = rt::configure_realtime_memory(heap_reserve_bytes);
        if (setup.getrlimit_succeeded) {
            RCLCPP_INFO(controller_manager->get_logger(),
                "RLIMIT_MEMLOCK: soft=%s hard=%s",
                rt::describe_rlimit(setup.memlock_soft).c_str(),
                rt::describe_rlimit(setup.memlock_hard).c_str());
        }
        if (!setup.mlockall_succeeded) {
            RCLCPP_WARN(controller_manager->get_logger(),
                "mlockall failed (%s) — control thread may page-fault under memory pressure",
                std::strerror(setup.mlockall_errno));
        }

        // Periodically report page-fault counts and memory footprint to the logs
        // so the lock's effectiveness (and the peak RSS for sizing the memlock
        // cap) is observable in the field without perf/proc tooling. The report
        // interval is the rt_memory.log_interval_sec ROS2 parameter; <= 0 disables it.
        if (log_interval_sec > 0.0) {
            RCLCPP_INFO(controller_manager->get_logger(),
                "RT memory monitor: reporting every %.0fs", log_interval_sec);
        } else {
            RCLCPP_INFO(controller_manager->get_logger(),
                "RT memory monitor: disabled (rt_memory.log_interval_sec <= 0)");
        }
        rt::RtMemoryMonitor memory_monitor(controller_manager->get_update_rate(), log_interval_sec);

        // The cycle grid lives on CLOCK_MONOTONIC and every sleep is to an
        // ABSOLUTE deadline (clock_nanosleep TIMER_ABSTIME). This replaced a
        // std::chrono::system_clock grid + std::this_thread::sleep_until, which
        // libstdc++ runs as `while (now < t) sleep_for(t - now)`: an NTP step of
        // -0.894 s on the robot stretched one cycle by 0.9 s (the EtherCAT drives'
        // 500 ms watchdog tripped) and the loop then ran the missed cycles back to
        // back to catch up. The wall clock cannot move a monotonic deadline, and
        // the overrun rule below never bursts. rt_timebase.hpp has the policy;
        // controller_manager->now() is still used for the ROS time stamps.
        namespace rttb = ELITE_CS_ROBOT_ROS_DRIVER::rt_timebase;
        RCLCPP_INFO(controller_manager->get_logger(),
            "Period (ns): %ld Update Rate: %u (Hz), CLOCK_MONOTONIC absolute deadlines",
            static_cast<long>(period_ns), controller_manager->get_update_rate());

        // First deadline one period from now; the previous wake-up is "now" so the
        // first measured period is close to nominal.
        timespec previous_wake = rttb::now_monotonic();
        timespec deadline = rttb::next_deadline(previous_wake, period_ns);

        while (rclcpp::ok()) {
            try {
                // Sleep to the absolute deadline. The return code is deliberately
                // not logged here (nothing on this thread logs per cycle); with
                // valid arguments clock_nanosleep only ever returns 0 or EINTR,
                // and EINTR is retried inside.
                rttb::sleep_until_monotonic(deadline);
                const timespec wake = rttb::now_monotonic();

                // Classify the wake-up and pick the next deadline: on time or late
                // (< one period) advances the grid by exactly one period; an
                // overrun (>= one period late) restarts the grid from now, so the
                // missed cycles are dropped, not run back to back.
                const rttb::Step step = rttb::step(deadline, wake, period_ns);
                deadline = step.next_deadline;

                // The measured period is the monotonic gap between wake-ups: the
                // real dt the controllers experienced, immune to wall-clock steps.
                // The controller manager receives it clamped to [0, 10 x period]
                // so a stall never feeds a multi-second dt into an integrator; the
                // clamp is counted with the overruns.
                const int64_t wake_gap_ns = rttb::diff_ns(wake, previous_wake);
                previous_wake = wake;
                const rttb::ClampedPeriod dt = rttb::clamp_measured_period(wake_gap_ns, period_ns);
                const rclcpp::Duration measured_period = rclcpp::Duration::from_nanoseconds(dt.ns);

                // Lock-free, wait-free; read by the reporter on the executor thread.
                rttb::CycleSample sample;
                sample.period_ns = wake_gap_ns;
                sample.lateness_ns = step.wake.lateness_ns;
                sample.overrun = step.wake.verdict == rttb::Verdict::Overrun;
                sample.clamped = dt.clamped;
                period_stats->record(sample);

                // execute update loop (ROS time stamps from the controller manager clock)
                controller_manager->read(controller_manager->now(), measured_period);
                controller_manager->update(controller_manager->now(), measured_period);
                controller_manager->write(controller_manager->now(), measured_period);

                // Emit a fault/memory report at most once per log interval.
                memory_monitor.tick(controller_manager->get_logger());

            } catch (const std::exception& ex) {
                RCLCPP_FATAL_STREAM(rclcpp::get_logger("controller_manager"), ex.what());
                // Signal main thread with error
                std::raise(SIGUSR1);
                break;
            } catch (...) {
                RCLCPP_FATAL(rclcpp::get_logger("controller_manager"), "Unknown exception in control loop");
                // Signal main thread with error
                std::raise(SIGUSR1);
                break;
            }
        }
    });

    try {
        executor->spin();
    } catch (const std::exception& ex) {
        exit_code = 1;
        RCLCPP_FATAL(
            rclcpp::get_logger("main"),
            "Exception in executor: %s", ex.what()
        );
    } catch (...) {
        exit_code = 1;
        RCLCPP_FATAL(
            rclcpp::get_logger("main"),
            "Unknown exception in executor");
    }

    // Wait for control loop to finish
    control_loop.join();

    try {
        rclcpp::shutdown();
    } catch (const std::exception & e) {
        exit_code = 1;
        fprintf(stderr, "Caught exception during rclcpp::shutdown: %s\n", e.what());
    }

    return exit_code;
}
