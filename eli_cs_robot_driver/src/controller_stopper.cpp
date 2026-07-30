#include <atomic>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/utilities.hpp>

#include "eli_cs_robot_driver/controller_stopper.hpp"

ControllerStopper::ControllerStopper(const rclcpp::Node::SharedPtr& node, bool stop_controllers_on_startup)
    : node_(node), stop_controllers_on_startup_(stop_controllers_on_startup), robot_running_(true) {
    // Subscribes to a robot's running state topic. Ideally this topic is latched and only publishes
    // on changes. However, this node only reacts on state changes, so a state published each cycle
    // would also be fine.
    robot_running_sub_ = node->create_subscription<std_msgs::msg::Bool>(
        "io_and_status_controller/robot_task_running", 1,
        std::bind(&ControllerStopper::robotRunningCallback, this, std::placeholders::_1));

    // Controller manager service to switch controllers
    controller_manager_srv_ = node_->create_client<controller_manager_msgs::srv::SwitchController>(
        "controller_manager/"
        "switch_controller");
    // Controller manager service to list controllers
    controller_list_srv_ = node_->create_client<controller_manager_msgs::srv::ListControllers>(
        "controller_manager/"
        "list_controllers");
    // Controller manager service to configure controllers (unconfigured -> inactive)
    configure_controller_srv_ = node_->create_client<controller_manager_msgs::srv::ConfigureController>(
        "controller_manager/"
        "configure_controller");

    // Get robot mode from dashboard
    dashboard_robot_mode_srv_ = node->create_client<eli_common_interface::srv::GetRobotMode>("dashboard_client/robot_mode");

    // Wait "controller_manager/switch_controller"
    RCLCPP_INFO(rclcpp::get_logger("Controller stopper"),
                "Waiting for switch controller service to come up on "
                "controller_manager/switch_controller");
    controller_manager_srv_->wait_for_service();
    RCLCPP_INFO(rclcpp::get_logger("Controller stopper"), "Service available");

    // Wait "controller_manager/list_controllers"
    RCLCPP_INFO(rclcpp::get_logger("Controller stopper"),
                "Waiting for list controllers service to come up on "
                "controller_manager/list_controllers");
    controller_list_srv_->wait_for_service();
    RCLCPP_INFO(rclcpp::get_logger("Controller stopper"), "Service available");

    // Wait "controller_manager/configure_controller"
    RCLCPP_INFO(rclcpp::get_logger("Controller stopper"),
                "Waiting for configure controller service to come up on "
                "controller_manager/configure_controller");
    configure_controller_srv_->wait_for_service();
    RCLCPP_INFO(rclcpp::get_logger("Controller stopper"), "Service available");

    // Wait "dashboard_client/robot_mode"
    RCLCPP_INFO(rclcpp::get_logger("Controller stopper"),
                "Waiting for robot mode(dashboard) service to come up on "
                "dashboard_client/robot_mode");
    dashboard_robot_mode_srv_->wait_for_service();
    RCLCPP_INFO(rclcpp::get_logger("Controller stopper"), "Service available");

    consistent_controllers_ = node_->declare_parameter<std::vector<std::string>>("consistent_controllers");

    // Get robot mode and if robot is power off, stop controller on startup
    auto robot_mode_request = std::make_shared<eli_common_interface::srv::GetRobotMode::Request>();
    auto robot_mode_response = dashboard_robot_mode_srv_->async_send_request(robot_mode_request);
    rclcpp::spin_until_future_complete(node_, robot_mode_response);
    auto robot_mode = robot_mode_response.get()->mode.mode;
    if (robot_mode != eli_common_interface::msg::RobotMode::POWER_ON &&
        robot_mode != eli_common_interface::msg::RobotMode::IDLE &&
        robot_mode != eli_common_interface::msg::RobotMode::RUNNING) {
            stop_controllers_on_startup_ = true;
            RCLCPP_INFO(rclcpp::get_logger("Controller stopper"), "Robot mode: %d. Must stop controllers", robot_mode);
    }

    if (stop_controllers_on_startup_ == true) {
        while (stopped_controllers_.empty()) {
            auto request = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
            auto future = controller_list_srv_->async_send_request(request);
            rclcpp::spin_until_future_complete(node_, future);
            auto result = future.get();
            for (auto& controller : result->controller) {
                // Check if in consistent_controllers
                // Else:
                //   Add to stopped_controllers
                if (controller.state == "active") {
                    auto it = std::find(consistent_controllers_.begin(), consistent_controllers_.end(), controller.name);
                    if (it == consistent_controllers_.end()) {
                        stopped_controllers_.push_back(controller.name);
                    }
                }
            }
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }
        auto request_switch_controller = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
        // BEST_EFFORT: one controller that cannot be deactivated must not abort the whole switch
        // and leave the rest running while the robot program is not.
        request_switch_controller->strictness = request_switch_controller->BEST_EFFORT;
        request_switch_controller->deactivate_controllers = stopped_controllers_;
        auto future = controller_manager_srv_->async_send_request(request_switch_controller);
        rclcpp::spin_until_future_complete(node_, future);
        if (future.get()->ok == false) {
            RCLCPP_ERROR(rclcpp::get_logger("Controller stopper"), "Could not deactivate requested controllers");
        }
        robot_running_ = false;
    }
}

void ControllerStopper::findAndStopControllers() {
    stopped_controllers_.clear();
    auto request_switch_controller = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    auto request_list_controllers = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();

    // Callback to switch controllers
    auto callback_switch_controller =
        [this](rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future_response) {
            auto result = future_response.get();
            if (result->ok == false) {
                RCLCPP_ERROR(rclcpp::get_logger("Controller stopper"), "Could not deactivate requested controllers");
            }
        };

    // Callback to list controllers
    auto callback_list_controller =
        [this, request_switch_controller,
         callback_switch_controller](rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedFuture future_response) {
            auto result = future_response.get();
            for (auto& controller : result->controller) {
                // Check if in consistent_controllers
                // Else:
                //   Add to stopped_controllers
                if (controller.state == "active") {
                    auto it = std::find(consistent_controllers_.begin(), consistent_controllers_.end(), controller.name);
                    if (it == consistent_controllers_.end()) {
                        stopped_controllers_.push_back(controller.name);
                    }
                }
            }
            // BEST_EFFORT: see the constructor's startup deactivation for why.
            request_switch_controller->strictness = request_switch_controller->BEST_EFFORT;
            if (!stopped_controllers_.empty()) {
                request_switch_controller->deactivate_controllers = stopped_controllers_;
                auto future = controller_manager_srv_->async_send_request(request_switch_controller, callback_switch_controller);
            }
        };

    auto future = controller_list_srv_->async_send_request(request_list_controllers, callback_list_controller);
}

void ControllerStopper::startControllers() {
    if (stopped_controllers_.empty()) {
        return;
    }

    // A controller can only be activated out of the 'inactive' state. Anything that resets the
    // hardware interfaces underneath a controller -- notably cycling the cs612 hardware component,
    // which a driver reset does -- drops that controller back to 'unconfigured' instead. Activating
    // it from there is impossible, so re-query the current states first and configure whatever came
    // back unconfigured before attempting the switch. Without this the controller is stranded
    // permanently: nothing else ever reconfigures it, and every subsequent robot-program cycle
    // retries the same impossible activation.
    auto request_list_controllers = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();

    auto callback_list_controller =
        [this](rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedFuture future_response) {
            auto result = future_response.get();
            std::map<std::string, std::string> state_by_name;
            for (const auto& controller : result->controller) {
                state_by_name[controller.name] = controller.state;
            }

            std::vector<std::string> to_configure;
            for (const auto& name : stopped_controllers_) {
                auto it = state_by_name.find(name);
                if (it != state_by_name.end() && it->second == "unconfigured") {
                    to_configure.push_back(name);
                }
            }

            if (to_configure.empty()) {
                activateStoppedControllers();
                return;
            }

            // Activate only once every configure request has come back, otherwise the switch would
            // race the transitions it depends on. Configure failures are logged and then ignored --
            // BEST_EFFORT activation will simply skip anything still unconfigured.
            auto pending = std::make_shared<std::atomic<size_t>>(to_configure.size());
            for (const auto& name : to_configure) {
                RCLCPP_WARN(rclcpp::get_logger("Controller stopper"),
                            "Controller '%s' is unconfigured; configuring it before activation", name.c_str());
                auto request = std::make_shared<controller_manager_msgs::srv::ConfigureController::Request>();
                request->name = name;
                // `name` captured by value: the lambda outlives this loop iteration.
                configure_controller_srv_->async_send_request(
                    request, [this, pending, name](
                                 rclcpp::Client<controller_manager_msgs::srv::ConfigureController>::SharedFuture future) {
                        if (future.get()->ok == false) {
                            RCLCPP_ERROR(rclcpp::get_logger("Controller stopper"), "Could not configure controller '%s'",
                                         name.c_str());
                        }
                        if (pending->fetch_sub(1) == 1) {
                            activateStoppedControllers();
                        }
                    });
            }
        };

    auto future = controller_list_srv_->async_send_request(request_list_controllers, callback_list_controller);
}

void ControllerStopper::activateStoppedControllers() {
    auto callback = [this](rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future_response) {
        auto result = future_response.get();
        if (result->ok == false) {
            RCLCPP_ERROR(rclcpp::get_logger("Controller stopper"), "Could not activate requested controllers");
        }
    };
    if (!stopped_controllers_.empty()) {
        auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
        // BEST_EFFORT: a controller that still cannot be activated (unloaded, configure failed,
        // hardware interface unavailable) must not take every other stopped controller down with
        // it. Under STRICT the controller manager rejects the entire switch, so one stranded
        // controller blocks the robot indefinitely.
        request->strictness = request->BEST_EFFORT;
        request->activate_controllers = stopped_controllers_;
        auto future = controller_manager_srv_->async_send_request(request, callback);
    }
}

void ControllerStopper::robotRunningCallback(const std_msgs::msg::Bool::ConstSharedPtr msg) {
    RCLCPP_DEBUG(rclcpp::get_logger("Controller stopper"), "robotRunningCallback with data %d", msg->data);

    if (msg->data && !robot_running_) {
        RCLCPP_DEBUG(rclcpp::get_logger("Controller stopper"), "Starting controllers");
        startControllers();
    } else if (!msg->data && robot_running_) {
        RCLCPP_DEBUG(rclcpp::get_logger("Controller stopper"), "Stopping controllers");
        // stop all controllers except the once in consistent_controllers_
        findAndStopControllers();
    }
    robot_running_ = msg->data;
}
