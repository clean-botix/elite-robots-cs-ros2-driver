#include <algorithm>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <action_msgs/srv/cancel_goal.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
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

    rcl_interfaces::msg::ParameterDescriptor cancel_timeout_descriptor;
    cancel_timeout_descriptor.description = "Seconds a stop waits for trajectory cancels before it deactivates anyway.";
    // A double declaration throws on an integer such as `2` from YAML. Dynamic typing accepts both.
    cancel_timeout_descriptor.dynamic_typing = true;
    const rclcpp::ParameterValue cancel_timeout_value =
        node_->declare_parameter("trajectory_cancel_timeout", rclcpp::ParameterValue(1.0), cancel_timeout_descriptor);
    const double cancel_timeout_s = cancel_timeout_value.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER
                                        ? static_cast<double>(cancel_timeout_value.get<int64_t>())
                                        : cancel_timeout_value.get<double>();
    cancel_timeout_ = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(cancel_timeout_s));

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

ControllerStopper::~ControllerStopper() {
    prime_timer_->cancel();
    for (auto& timer : cancel_timeout_timers_) {
        timer->cancel();
    }
    robot_running_sub_.reset();
    controller_list_srv_->prune_pending_requests();
    controller_manager_srv_->prune_pending_requests();
    // Dropping the action clients drops their pending cancel callbacks
    trajectory_action_clients_.clear();
}

bool ControllerStopper::isConsistent(const std::string& controller) const {
    return std::find(consistent_controllers_.begin(), consistent_controllers_.end(), controller) != consistent_controllers_.end();
}

void ControllerStopper::findAndStopControllers() {
    stopped_controllers_.clear();
    const uint64_t generation = ++stop_generation_;
    auto request_switch_controller = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    auto request_list_controllers = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();

    // Callback to switch controllers
    auto callback_switch_controller =
        [request_switch_controller](rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future_response) {
            auto result = future_response.get();
            if (result->ok == false) {
                RCLCPP_ERROR(rclcpp::get_logger("Controller stopper"), "Could not deactivate all of [%s]",
                             joinNames(request_switch_controller->deactivate_controllers).c_str());
            }
        };

    // Callback to list controllers
    auto callback_list_controller =
        [this, generation, request_switch_controller,
         callback_switch_controller](rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedFuture future_response) {
            if (generation != stop_generation_) {
                RCLCPP_INFO(rclcpp::get_logger("Controller stopper"),
                            "Robot task came back before the controllers were listed; not stopping them");
                return;
            }
            auto result = future_response.get();
            for (auto& controller : result->controller) {
                if (controller.state == "active" && !isConsistent(controller.name)) {
                    stopped_controllers_.push_back(controller.name);
                }
            }
            if (stopped_controllers_.empty()) {
                return;
            }
            // A controller that keeps its goal stays active. BEST_EFFORT still stops the rest.
            request_switch_controller->strictness = request_switch_controller->BEST_EFFORT;
            request_switch_controller->deactivate_controllers = stopped_controllers_;
            cancelTrajectoryGoals(
                stopped_controllers_, [this, generation, request_switch_controller, callback_switch_controller]() {
                    if (generation != stop_generation_) {
                        RCLCPP_INFO(rclcpp::get_logger("Controller stopper"),
                                    "Robot task came back while trajectory goals were being canceled; "
                                    "leaving the controllers active");
                        return;
                    }
                    deactivated_generation_ = generation;
                    controller_manager_srv_->async_send_request(request_switch_controller, callback_switch_controller);
                });
        };

    auto future = controller_list_srv_->async_send_request(request_list_controllers, callback_list_controller);
}

void ControllerStopper::cancelTrajectoryGoals(const std::vector<std::string>& controllers, std::function<void()> on_canceled) {
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

    // A client created now would not discover its server in time
    std::vector<std::pair<std::string, rclcpp_action::Client<FollowJointTrajectory>::SharedPtr>> to_cancel;
    for (const auto& controller : controllers) {
        auto it = trajectory_action_clients_.find(controller);
        if (it == trajectory_action_clients_.end()) {
            RCLCPP_WARN(rclcpp::get_logger("Controller stopper"),
                        "No trajectory cancel client for '%s' yet; deactivating it without a cancel", controller.c_str());
            continue;
        }
        if (it->second->action_server_is_ready()) {
            to_cancel.emplace_back(controller, it->second);
        } else if (trajectory_servers_seen_.count(controller) > 0) {
            RCLCPP_WARN(rclcpp::get_logger("Controller stopper"),
                        "Trajectory action server for '%s' has gone away; deactivating it without a cancel", controller.c_str());
        } else {
            RCLCPP_INFO(rclcpp::get_logger("Controller stopper"),
                        "'%s' has no trajectory action server; deactivating it without a cancel", controller.c_str());
        }
    }

    if (to_cancel.empty()) {
        on_canceled();
        return;
    }

    RCLCPP_INFO(rclcpp::get_logger("Controller stopper"), "Canceling trajectory goals on %zu controller(s) before deactivating",
                to_cancel.size());

    // on_canceled runs once. It runs after the last answer or after the timeout.
    struct PendingCancel {
        std::set<std::string> unanswered;
        std::function<void()> on_canceled;
        rclcpp::TimerBase::WeakPtr timeout;
        bool finished = false;
    };
    auto pending = std::make_shared<PendingCancel>();
    pending->on_canceled = std::move(on_canceled);
    for (const auto& entry : to_cancel) {
        pending->unanswered.insert(entry.first);
    }

    auto finish = [this, pending]() {
        if (pending->finished) {
            return;
        }
        pending->finished = true;
        if (auto timer = pending->timeout.lock()) {
            timer->cancel();
            cancel_timeout_timers_.erase(std::remove(cancel_timeout_timers_.begin(), cancel_timeout_timers_.end(), timer),
                                         cancel_timeout_timers_.end());
        }
        pending->on_canceled();
    };

    auto timer = node_->create_wall_timer(cancel_timeout_, [pending, finish]() {
        if (pending->finished) {
            return;
        }
        RCLCPP_ERROR(rclcpp::get_logger("Controller stopper"),
                     "Trajectory cancel got no answer from [%s] within the timeout; deactivating anyway",
                     joinNames({pending->unanswered.begin(), pending->unanswered.end()}).c_str());
        finish();
    });
    pending->timeout = timer;
    cancel_timeout_timers_.push_back(timer);

    for (const auto& [controller, client] : to_cancel) {
        client->async_cancel_all_goals([pending, finish, controller = controller](
                                           const rclcpp_action::Client<FollowJointTrajectory>::CancelResponse::SharedPtr response) {
            if (response->return_code == action_msgs::srv::CancelGoal::Response::ERROR_REJECTED) {
                // REJECTED means no goal or a refusal. A refusal surfaces as a failed deactivate.
                RCLCPP_INFO(rclcpp::get_logger("Controller stopper"),
                            "Trajectory cancel for '%s' canceled nothing (no goal, or refused)", controller.c_str());
            } else if (response->return_code != action_msgs::srv::CancelGoal::Response::ERROR_NONE) {
                RCLCPP_WARN(rclcpp::get_logger("Controller stopper"), "Trajectory cancel for '%s' answered with return code %d",
                            controller.c_str(), response->return_code);
            }
            pending->unanswered.erase(controller);
            if (pending->unanswered.empty()) {
                finish();
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
            for (const auto& [name, client] : trajectory_action_clients_) {
                if (client->action_server_is_ready()) {
                    trajectory_servers_seen_.insert(name);
                }
            }
            std::size_t added = 0;
            for (const auto& controller : response->controller) {
                if (trajectory_action_clients_.count(controller.name) > 0 || isConsistent(controller.name)) {
                    continue;
                }
                trajectory_action_clients_.emplace(controller.name,
                                                   rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
                                                       node_, controller.name + "/follow_joint_trajectory"));
                ++added;
            }
            if (added > 0) {
                RCLCPP_INFO(rclcpp::get_logger("Controller stopper"),
                            "Prepared trajectory cancel clients for %zu new controller(s), %zu total", added,
                            trajectory_action_clients_.size());
            }
        });
}

void ControllerStopper::startControllers() {
    const bool current_stop_deactivated = deactivated_generation_ == stop_generation_;
    ++stop_generation_;
    if (!current_stop_deactivated) {
        RCLCPP_INFO(rclcpp::get_logger("Controller stopper"), "No controllers were deactivated; nothing to restart");
        return;
    }
    auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    // A controller that kept its goal is still active. STRICT would refuse the whole activate.
    request->strictness = request->BEST_EFFORT;
    request->activate_controllers = stopped_controllers_;
    auto callback = [request](rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future_response) {
        if (future_response.get()->ok == false) {
            RCLCPP_ERROR(rclcpp::get_logger("Controller stopper"), "Could not activate all of [%s]",
                         joinNames(request->activate_controllers).c_str());
        }
    };
    controller_manager_srv_->async_send_request(request, callback);
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
