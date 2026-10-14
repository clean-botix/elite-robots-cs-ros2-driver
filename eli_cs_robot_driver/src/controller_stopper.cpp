#include <algorithm>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <action_msgs/srv/cancel_goal.hpp>
#include <rclcpp/utilities.hpp>

#include "eli_cs_robot_driver/controller_stopper.hpp"

namespace {

std::string joinNames(const std::vector<std::string>& names) {
    std::string joined;
    for (const auto& name : names) {
        joined += (joined.empty() ? "" : ", ") + name;
    }
    return joined;
}

}  // namespace

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

    // Wait "dashboard_client/robot_mode"
    RCLCPP_INFO(rclcpp::get_logger("Controller stopper"),
                "Waiting for robot mode(dashboard) service to come up on "
                "dashboard_client/robot_mode");
    dashboard_robot_mode_srv_->wait_for_service();
    RCLCPP_INFO(rclcpp::get_logger("Controller stopper"), "Service available");

    consistent_controllers_ = node_->declare_parameter<std::vector<std::string>>("consistent_controllers");

    primeTrajectoryActionClients();
    // The spawners load the controllers after this node starts
    prime_timer_ = node_->create_wall_timer(std::chrono::seconds(2), [this]() { primeTrajectoryActionClients(); });

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
                if (controller.state == "active" && !isConsistent(controller.name)) {
                    stopped_controllers_.push_back(controller.name);
                }
            }
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }
        auto request_switch_controller = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
        request_switch_controller->strictness = request_switch_controller->STRICT;
        request_switch_controller->deactivate_controllers = stopped_controllers_;
        auto future = controller_manager_srv_->async_send_request(request_switch_controller);
        rclcpp::spin_until_future_complete(node_, future);
        if (future.get()->ok == false) {
            RCLCPP_ERROR(rclcpp::get_logger("Controller stopper"), "Could not deactivate requested controllers");
        }
        robot_running_ = false;
    }
}

bool ControllerStopper::isConsistent(const std::string& controller) const {
    return std::find(consistent_controllers_.begin(), consistent_controllers_.end(), controller) != consistent_controllers_.end();
}

void ControllerStopper::findAndStopControllers() {
    stopped_controllers_.clear();
    auto request_switch_controller = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    auto request_list_controllers = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();

    // Callback to switch controllers
    auto callback_switch_controller =
        [request_switch_controller](rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future_response) {
            auto result = future_response.get();
            if (result->ok == false) {
                RCLCPP_ERROR(rclcpp::get_logger("Controller stopper"), "Could not deactivate all of [%s]",
                             joinNames(request_switch_controller->deactivate_controllers).c_str());
            } else {
                RCLCPP_INFO(rclcpp::get_logger("Controller stopper"), "Deactivated [%s]",
                            joinNames(request_switch_controller->deactivate_controllers).c_str());
            }
        };

    // Callback to list controllers
    auto callback_list_controller =
        [this, request_switch_controller,
         callback_switch_controller](rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedFuture future_response) {
            auto result = future_response.get();
            for (auto& controller : result->controller) {
                if (controller.state == "active" && !isConsistent(controller.name)) {
                    stopped_controllers_.push_back(controller.name);
                }
            }
            if (stopped_controllers_.empty()) {
                return;
            }
            request_switch_controller->strictness = request_switch_controller->STRICT;
            request_switch_controller->deactivate_controllers = stopped_controllers_;
            cancelTrajectoryGoals(stopped_controllers_, [this, request_switch_controller, callback_switch_controller]() {
                RCLCPP_INFO(rclcpp::get_logger("Controller stopper"), "Deactivating [%s]",
                            joinNames(request_switch_controller->deactivate_controllers).c_str());
                controller_manager_srv_->async_send_request(request_switch_controller, callback_switch_controller);
            });
        };

    auto future = controller_list_srv_->async_send_request(request_list_controllers, callback_list_controller);
}

void ControllerStopper::cancelTrajectoryGoals(const std::vector<std::string>& controllers, std::function<void()> on_canceled) {
    using CancelGoal = action_msgs::srv::CancelGoal;

    // A client created now would not discover its server in time
    std::vector<std::pair<std::string, rclcpp::Client<CancelGoal>::SharedPtr>> to_cancel;
    for (const auto& controller : controllers) {
        auto it = trajectory_cancel_clients_.find(controller);
        if (it == trajectory_cancel_clients_.end()) {
            RCLCPP_INFO(rclcpp::get_logger("Controller stopper"),
                        "'%s' has no trajectory action server; deactivating it without a cancel", controller.c_str());
        } else if (it->second->service_is_ready()) {
            to_cancel.emplace_back(controller, it->second);
        } else {
            RCLCPP_WARN(rclcpp::get_logger("Controller stopper"),
                        "Trajectory action server for '%s' has gone away; deactivating it without a cancel", controller.c_str());
        }
    }

    if (to_cancel.empty()) {
        on_canceled();
        return;
    }

    RCLCPP_INFO(rclcpp::get_logger("Controller stopper"), "Canceling trajectory goals on %zu controller(s) before deactivating",
                to_cancel.size());

    // on_canceled runs once, after the last answer
    struct PendingCancel {
        std::set<std::string> unanswered;
        std::function<void()> on_canceled;
    };
    auto pending = std::make_shared<PendingCancel>();
    pending->on_canceled = std::move(on_canceled);
    for (const auto& entry : to_cancel) {
        pending->unanswered.insert(entry.first);
    }

    // A zero goal ID and a zero stamp cancel every goal
    auto cancel_all = std::make_shared<CancelGoal::Request>();
    for (const auto& [controller, client] : to_cancel) {
        client->async_send_request(cancel_all, [pending, controller = controller](rclcpp::Client<CancelGoal>::SharedFuture future) {
            const auto response = future.get();
            if (response->return_code == CancelGoal::Response::ERROR_REJECTED) {
                // REJECTED means no goal or a refusal. A refusal surfaces as a failed deactivate.
                RCLCPP_INFO(rclcpp::get_logger("Controller stopper"),
                            "Trajectory cancel for '%s' canceled nothing (no goal, or refused)", controller.c_str());
            } else if (response->return_code != CancelGoal::Response::ERROR_NONE) {
                RCLCPP_WARN(rclcpp::get_logger("Controller stopper"), "Trajectory cancel for '%s' answered with return code %d",
                            controller.c_str(), response->return_code);
            }
            pending->unanswered.erase(controller);
            if (pending->unanswered.empty()) {
                pending->on_canceled();
            }
        });
    }
}

void ControllerStopper::primeTrajectoryActionClients() {
    // A timer callback must not spin its own node. The request is therefore asynchronous.
    auto request = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
    controller_list_srv_->async_send_request(
        request, [this](rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedFuture future) {
            auto response = future.get();
            const auto services = node_->get_service_names_and_types();
            std::size_t added = 0;
            for (const auto& controller : response->controller) {
                if (trajectory_cancel_clients_.count(controller.name) > 0 || isConsistent(controller.name)) {
                    continue;
                }
                const std::string cancel_service = node_->get_node_base_interface()->resolve_topic_or_service_name(
                    controller.name + "/follow_joint_trajectory/_action/cancel_goal", true);
                // A client for a service that does not exist is a dangling graph endpoint, and
                // MoveIt Pro's web bridge throws on those
                if (services.count(cancel_service) == 0) {
                    continue;
                }
                trajectory_cancel_clients_.emplace(controller.name, node_->create_client<action_msgs::srv::CancelGoal>(cancel_service));
                ++added;
            }
            if (added > 0) {
                RCLCPP_INFO(rclcpp::get_logger("Controller stopper"),
                            "Prepared trajectory cancel clients for %zu new controller(s), %zu total", added,
                            trajectory_cancel_clients_.size());
            }
        });
}

void ControllerStopper::startControllers() {
    // Callback to switch controllers
    auto callback = [this](rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future_response) {
        auto result = future_response.get();
        if (result->ok == false) {
            RCLCPP_ERROR(rclcpp::get_logger("Controller stopper"), "Could not activate requested controllers");
        }
    };
    if (!stopped_controllers_.empty()) {
        auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
        request->strictness = request->STRICT;
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
