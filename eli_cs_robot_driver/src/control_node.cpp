#include <memory>
#include <thread>
#include <csignal>
#include <cerrno>
#include <cstring>
#include <cstdio>
#include <exception>
#include <string>

#include <malloc.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>

// ROS-free real-time memory helpers (describe_rlimit, classify_terminate,
// reserve_process_memory). Kept in a header so they are unit-tested directly;
// see test/test_rt_memory.cpp.
#include "eli_cs_robot_driver/rt_memory.hpp"

// Rely on a subclass of ControllerManager to intercept the pre-shutdown hook
// and execute a workaround for a ROS2 Humble bug to ensure orderly shutdown at termination.
#include "eli_cs_robot_driver/elite_controller_manager.hpp"

// This include directive triggers a compilation warning to use <realtime_tools/realtime_helpers.hpp> instead.
// However, while that change is a drop-in replacement that clears the warning, something about it introduces
// a breaking change of the worse kind -- silent failures. Arm motions fail in real life without explicit errors, etc.
#include <realtime_tools/thread_priority.hpp>

// Elite code is inspired by:
// https://github.com/ros-controls/ros2_control/blob/master/controller_manager/src/ros2_control_node.cpp

std::atomic<int> exit_code{0};

void signal_handler(int signal) {
    // SIGUSR1 is our custom error-exit signal. Only set the exit code here.
    // rclcpp::shutdown() is not async-signal-safe and must not be called from a signal handler.
    // The control loop calls it directly after detecting the error (std::raise returns synchronously).
    if (signal == SIGUSR1) {
        exit_code = 1;
    }
}

// Conservative heap headroom to pre-fault before entering the RT loop. With
// glibc M_TRIM_THRESHOLD=-1 in effect this becomes a permanent RSS floor, so it
// is sized as headroom over the measured footprint; tune with monitoring (SW-933).
static constexpr size_t kHeapReserveBytes = 100UL * 1024 * 1024; // 100 MiB

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

    // Control loop thread
    std::thread control_loop([controller_manager]() {
        if (!realtime_tools::configure_sched_fifo(50)) {
            RCLCPP_WARN(controller_manager->get_logger(), "Could not enable FIFO RT scheduling policy");
        }

        // Log the memlock rlimit this process is working with, so a later
        // memory-exhaustion condition is diagnosable and an absent/insufficient
        // ulimit is obvious in the logs (mlockall below silently fails without it).
        namespace rt = ELITE_CS_ROBOT_ROS_DRIVER::rt_memory;
        struct rlimit memlock_limit{};
        if (getrlimit(RLIMIT_MEMLOCK, &memlock_limit) == 0) {
            RCLCPP_INFO(controller_manager->get_logger(),
                "RLIMIT_MEMLOCK: soft=%s hard=%s",
                rt::describe_rlimit(memlock_limit.rlim_cur).c_str(),
                rt::describe_rlimit(memlock_limit.rlim_max).c_str());
        }

        // Lock all of this PROCESS's current and future pages in RAM. mlockall is
        // process-scoped, not thread-scoped, even though it is called here from
        // the control thread: a page fault in the RT control thread blocks on the
        // kernel mm subsystem (possibly I/O) for tens to hundreds of ms, which RT
        // priority does not prevent — long enough to starve the cyclic bus update
        // (e.g. EtherCAT slave SM watchdogs). Requires the container to grant a
        // sufficient memlock ulimit (the deployed compose sets ulimits.memlock =
        // -1); warn-and-continue so a missing ulimit degrades RT behavior instead
        // of preventing startup.
        if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
            RCLCPP_WARN(controller_manager->get_logger(),
                "mlockall failed (%s) — control thread may page-fault under memory pressure",
                std::strerror(errno));
        }

        // Complement the lock so future allocations don't reintroduce page-fault
        // stalls in the RT loop:
        //   M_MMAP_MAX = 0       -> never service allocations via mmap (the mmap
        //                           path faults page-by-page on first touch);
        //                           heap growth goes through the main arena.
        //   M_TRIM_THRESHOLD = -1 -> never return freed memory to the kernel, so
        //                           it stays mapped (and locked) for later reuse.
        mallopt(M_MMAP_MAX, 0);
        mallopt(M_TRIM_THRESHOLD, -1);

        // Front-load heap growth: fault in (and, via MCL_FUTURE, lock) a reserve
        // now so the RT loop reuses already-resident pages instead of faulting.
        rt::reserve_process_memory(kHeapReserveBytes);

        // for calculating sleep time
        auto const period = std::chrono::nanoseconds(1'000'000'000 / controller_manager->get_update_rate());
        auto const cm_now = std::chrono::nanoseconds(controller_manager->now().nanoseconds());
        std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> next_iteration_time{cm_now};

        RCLCPP_INFO(controller_manager->get_logger(),
            "Period (ns): %lu Update Rate: %u (Hz)", period.count(), controller_manager->get_update_rate());

        // for calculating the measured period of the loop
        rclcpp::Time previous_time = controller_manager->now();

        while (rclcpp::ok()) {
            try {
                // calculate measured period
                auto const current_time = controller_manager->now();
                auto const measured_period = current_time - previous_time;
                previous_time = current_time;

                // execute update loop
                controller_manager->read(controller_manager->now(), measured_period);
                controller_manager->update(controller_manager->now(), measured_period);
                controller_manager->write(controller_manager->now(), measured_period);

                // wait until we hit the end of the period
                next_iteration_time += period;
                std::this_thread::sleep_until(next_iteration_time);

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
