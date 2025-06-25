#include <memory>
#include <thread>
#include <csignal>

#include <controller_manager/controller_manager.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/thread_priority.hpp>

// code is inspired by
// https://github.com/ros-controls/ros2_control/blob/master/controller_manager/src/ros2_control_node.cpp

std::atomic<int> exit_code{0};

void signal_handler(int signal) {
    // We're using SIGUSR1 as our custom error signal
    if (signal == SIGUSR1) {
        exit_code = 1;
    }
    rclcpp::shutdown();
}

int main(int argc, char** argv) {
    // Register signal handler
    std::signal(SIGUSR1, signal_handler);
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    rclcpp::init(argc, argv);

    // Create Executor
    std::shared_ptr<rclcpp::Executor> executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    // Create Controller Manager
    std::shared_ptr<controller_manager::ControllerManager> controller_manager;

    try {
        // create controller manager instance
        controller_manager = std::make_shared<controller_manager::ControllerManager>(executor, "controller_manager");
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
            "Period (ns): %u Update Rate: %u (Hz)", period, controller_manager->get_update_rate());

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

    // spin the executor with controller manager node
    executor->add_node(controller_manager);

    try {
        executor->spin();
        
    } catch (const std::exception& ex) {
        RCLCPP_FATAL(
            rclcpp::get_logger("main"),
            "Exception in executor: %s", ex.what()
        );
        exit_code = 1;
        rclcpp::shutdown();
    } catch (...) {
        RCLCPP_FATAL(rclcpp::get_logger("main"), "Unknown exception in executor");
        exit_code = 1;
        rclcpp::shutdown();
    }

    // Wait for control loop to finish
    control_loop.join();

    return exit_code;
}
