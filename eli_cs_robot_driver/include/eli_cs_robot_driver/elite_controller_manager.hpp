#pragma once

#include <hardware_interface/types/lifecycle_state_names.hpp>
#include <controller_manager/controller_manager.hpp>
#include <rclcpp/rclcpp.hpp>

// ROS2 Humble contains a bug in controller management.
//
// Shutdown handling is not internally wired up. As such, on_shutdown() in a
// hardware interface can never be called.
//
// As a workaround, we intercept the pre-shutdown hook and force lifecycle transitions
// that ensure hardware interface processes execute on_cleanup() before exiting.

namespace ELITE_CS_ROBOT_ROS_DRIVER
{

class EliteControllerManager : public controller_manager::ControllerManager
{
public:
  EliteControllerManager(
    std::shared_ptr<rclcpp::Executor> executor,
    const std::string & manager_node_name = "controller_manager",
    const std::string & namespace_ = "",
    const rclcpp::NodeOptions & options = controller_manager::get_cm_node_options())
  : ControllerManager(executor, manager_node_name, namespace_, options)
  {
    // Register pre-shutdown callback while the context is still live.
    // This fires BEFORE spin() returns, so ROS logging still works.
    auto context = this->get_node_base_interface()->get_context();
    pre_shutdown_handle_ =
      std::make_shared<rclcpp::PreShutdownCallbackHandle>(
        context->add_pre_shutdown_callback(
          [this]() { orderly_hardware_shutdown(); }));
  }

  ~EliteControllerManager() override
  {
    // Remove the callback to avoid a dangling reference the CM is destoyed without going through rclcpp shutdown
    if (pre_shutdown_handle_) {
      auto context = this->get_node_base_interface()->get_context();
      if (context) {
        context->remove_pre_shutdown_callback(*pre_shutdown_handle_);
      }
    }
  }

private:

  std::shared_ptr<rclcpp::PreShutdownCallbackHandle> pre_shutdown_handle_;
  std::atomic<bool> hardware_shutdown_done_{false};

  void orderly_hardware_shutdown()
  {
    using namespace hardware_interface::lifecycle_state_names;

    if (hardware_shutdown_done_.exchange(true)) {
      return; // Already ran -- don't run again
    }

    RCLCPP_INFO(get_logger(), "Orderly hardware shutdown beginning...");

    // Stop controllers first -- release command interfaces & stops update loop writing to hardware
    shutdown_controllers();

    if (resource_manager_) {
      for (const auto & component : resource_manager_->get_components_status()) {
        const std::string & name = component.first;
        const uint8_t current_state = component.second.state.id();

        /*
         * Force an orderly transition visiting ACTIVE → INACTIVE → UNCONFIGURED
         * (only transitioning if in a logically preceding state).
         * This ensures on_cleanup() is called in hardware interfaces at termination.
         */

        // ACTIVE → INACTIVE: calls on_deactivate()
        if (current_state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
          rclcpp_lifecycle::State inactive(
            lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
            INACTIVE);
          resource_manager_->set_component_state(name, inactive);
        }

        // INACTIVE → UNCONFIGURED: calls on_cleanup()
        if (current_state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE ||
            current_state == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
          rclcpp_lifecycle::State unconfigured(
            lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
            UNCONFIGURED);
          resource_manager_->set_component_state(name, unconfigured);
        }
      }
    }

    RCLCPP_INFO(get_logger(), "Hardware shutdown complete");
  }
};

}  // namespace ELITE_CS_ROBOT_ROS_DRIVER
