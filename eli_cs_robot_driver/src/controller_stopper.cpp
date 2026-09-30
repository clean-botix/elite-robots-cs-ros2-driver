#include <algorithm>
#include <memory>
#include <set>
#include <utility>
#include <string>
#include <vector>

#include <action_msgs/srv/cancel_goal.hpp>
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

    // How long a stop waits for trajectory cancels to be answered before deactivating anyway
    const double cancel_timeout_s = node_->declare_parameter<double>("trajectory_cancel_timeout", 1.0);
    cancel_timeout_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(cancel_timeout_s > 0.0 ? cancel_timeout_s : 1.0));

    primeTrajectoryActionClients();
    // The controllers are not loaded yet at this point, so keep looking until they are
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
    // Every one of these calls back into this object; none may fire once it is gone
    if (prime_timer_) {
        prime_timer_->cancel();
        prime_timer_.reset();
    }
    for (auto& timer : cancel_timeout_timers_) {
        timer->cancel();
    }
    cancel_timeout_timers_.clear();
    robot_running_sub_.reset();
    if (controller_list_srv_) {
        controller_list_srv_->prune_pending_requests();
    }
    if (controller_manager_srv_) {
        controller_manager_srv_->prune_pending_requests();
    }
    // Dropping the action clients drops the cancel callbacks still waiting on them
    trajectory_action_clients_.clear();
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
            request_switch_controller->strictness = request_switch_controller->STRICT;
            if (!stopped_controllers_.empty()) {
                request_switch_controller->deactivate_controllers = stopped_controllers_;
                // A controller still holding a trajectory goal refuses to deactivate, so the goal
                // has to go first -- see cancelTrajectoryGoals.
                cancelTrajectoryGoals(
                    stopped_controllers_, [this, request_switch_controller, callback_switch_controller]() {
                        controller_manager_srv_->async_send_request(request_switch_controller, callback_switch_controller);
                    });
            }
        };

    auto future = controller_list_srv_->async_send_request(request_list_controllers, callback_list_controller);
}

void ControllerStopper::cancelTrajectoryGoals(const std::vector<std::string>& controllers,
                                              std::function<void()> on_cancelled) {
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

    // Only clients primed earlier are usable. One created here would not have discovered its
    // server yet, would report not ready, and would be skipped -- which is indistinguishable from
    // having no cancel at all, so there is nothing to gain by making one.
    std::vector<std::pair<std::string, rclcpp_action::Client<FollowJointTrajectory>::SharedPtr>> to_cancel;
    for (const auto& controller : controllers) {
        auto it = trajectory_action_clients_.find(controller);
        if (it == trajectory_action_clients_.end()) {
            RCLCPP_WARN(rclcpp::get_logger("Controller stopper"),
                        "No trajectory cancel client for '%s' yet; deactivating it without a cancel",
                        controller.c_str());
            continue;
        }
        // Not every stoppable controller runs a trajectory action; those have no server to ask
        if (it->second->action_server_is_ready()) {
            to_cancel.emplace_back(controller, it->second);
        } else {
            RCLCPP_WARN(rclcpp::get_logger("Controller stopper"),
                        "No trajectory action server ready for '%s'; deactivating it without a cancel",
                        controller.c_str());
        }
    }

    if (to_cancel.empty()) {
        on_cancelled();
        return;
    }

    RCLCPP_WARN(rclcpp::get_logger("Controller stopper"), "Cancelling %zu trajectory goal(s) before deactivating",
                to_cancel.size());

    // The switch runs once: after the last controller answers, or after the timeout if one never
    // does. Cancelling is best effort, so a refused cancel counts as an answer -- it must not
    // strand the switch that follows -- and so does silence once the timeout expires.
    struct PendingCancel {
        std::set<std::string> unanswered;
        std::function<void()> on_cancelled;
        rclcpp::TimerBase::WeakPtr timeout;
        bool finished = false;
    };
    auto pending = std::make_shared<PendingCancel>();
    pending->on_cancelled = std::move(on_cancelled);
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
            cancel_timeout_timers_.erase(
                std::remove(cancel_timeout_timers_.begin(), cancel_timeout_timers_.end(), timer),
                cancel_timeout_timers_.end());
        }
        pending->on_cancelled();
    };

    auto timer = node_->create_wall_timer(cancel_timeout_, [pending, finish]() {
        if (pending->finished) {
            return;
        }
        std::string names;
        for (const auto& name : pending->unanswered) {
            names += (names.empty() ? "" : ", ") + name;
        }
        RCLCPP_ERROR(rclcpp::get_logger("Controller stopper"),
                     "Trajectory cancel got no answer from [%s] within the timeout; deactivating anyway",
                     names.c_str());
        finish();
    });
    pending->timeout = timer;
    cancel_timeout_timers_.push_back(timer);

    for (const auto& entry : to_cancel) {
        const std::string controller = entry.first;
        entry.second->async_cancel_all_goals(
            [pending, finish, controller](const rclcpp_action::Client<FollowJointTrajectory>::CancelResponse::SharedPtr response) {
                if (response && response->return_code != action_msgs::srv::CancelGoal::Response::ERROR_NONE) {
                    // Refused (or nothing to cancel). The deactivate is attempted regardless, and
                    // reports its own failure if the controller still refuses it.
                    RCLCPP_WARN(rclcpp::get_logger("Controller stopper"),
                                "Trajectory cancel for '%s' answered with return code %d", controller.c_str(),
                                response->return_code);
                }
                pending->unanswered.erase(controller);
                if (pending->unanswered.empty()) {
                    finish();
                }
            });
    }
}

void ControllerStopper::primeTrajectoryActionClients() {
    // Asynchronous because this also runs from a timer, and spinning the node from inside its own
    // callback would re-enter the executor.
    auto request = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
    controller_list_srv_->async_send_request(
        request, [this](rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedFuture future) {
            auto response = future.get();
            std::size_t added = 0;
            for (const auto& controller : response->controller) {
                if (trajectory_action_clients_.count(controller.name) > 0) {
                    continue;
                }
                trajectory_action_clients_.emplace(
                    controller.name, rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
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
