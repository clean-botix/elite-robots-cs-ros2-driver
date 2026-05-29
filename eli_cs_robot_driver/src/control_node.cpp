#include <memory>
#include <thread>
#include <csignal>

#include <rclcpp/rclcpp.hpp>

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

void signal_handler(int /*signal*/) {
    // SIGUSR1 is our custom error-exit signal. Only set the exit code here.
    // rclcpp::shutdown() is not async-signal-safe and must not be called from a signal handler.
    // The control loop calls it directly after detecting the error (std::raise returns synchronously).
    exit_code = 1;
}

int main(int argc, char** argv) {
    // Only register a handler for our custom error signal. SIGINT and SIGTERM
    // are left to rclcpp::init()'s safe handlers — overriding them here and
    // calling rclcpp::shutdown() from signal context would be unsafe
    // (not async-signal-safe) and can crash MultiThreadedExecutor.
    std::signal(SIGUSR1, signal_handler);

    std::set_terminate([]() {
        fprintf(stderr, "[ControllerManager] ignoring terminate() -- likely CM shutdown race in ROS2 Humble\n");
        _exit(0); // clean exit code, no core dump
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
