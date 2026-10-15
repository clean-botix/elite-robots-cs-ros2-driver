#ifndef __ELITE_CS_ROBOT_ROS_DRIVER__CONTROLLER_STOPPER_HPP__
#define __ELITE_CS_ROBOT_ROS_DRIVER__CONTROLLER_STOPPER_HPP__

#include "eli_common_interface/srv/get_robot_mode.hpp"

#include <chrono>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <action_msgs/msg/goal_status_array.hpp>
#include <action_msgs/srv/cancel_goal.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

class ControllerStopper {
   public:
    ControllerStopper() = delete;
    ControllerStopper(const rclcpp::Node::SharedPtr& node, bool stop_controllers_on_startup);
    /*!
     * \brief Cancels every timer and pending request that calls back into this object.
     */
    virtual ~ControllerStopper();

   private:
    void robotRunningCallback(const std_msgs::msg::Bool::ConstSharedPtr msg);

    /*!
     * \brief Queries running stoppable controllers and the controllers are stopped.
     *
     * Queries the controller manager for running controllers and compares the result with the
     * consistent_controllers_. The remaining running controllers are stored in stopped_controllers_
     * and stopped afterwards.
     */
    void findAndStopControllers();

    /*!
     * \brief Cancels the trajectory goals of the controllers about to stop, then runs `on_canceled`.
     *
     * PickNik's joint_trajectory_admittance_controller refuses to deactivate while it holds a goal.
     * The goal then reports success for a motion the robot abandoned. This cancels goals another
     * node sent. Normal ROS 2 action usage does not do that.
     *
     * The cancel goes to the action's cancel service, which has one type for every action. The
     * admittance controller serves FollowJointTrajectoryWithAdmittance under the same name.
     *
     * The cancel answer only says the server accepted it. The goal ends in a later control cycle,
     * and the controller refuses to deactivate until then. The wait therefore continues on the
     * action's status topic until the controller holds no goal at all. A goal the sender starts
     * after the cancel is canceled too, so the deactivate never leaves one behind.
     */
    void cancelTrajectoryGoals(const std::vector<std::string>& controllers, std::function<void()> on_canceled);

    using GoalId = unique_identifier_msgs::msg::UUID::_uuid_type;

    // One stop's cancel round. `on_canceled` runs once: after the last goal ends or after the timeout.
    struct PendingCancel {
        // Every controller this round cancels on
        std::set<std::string> controllers;
        // Controllers whose latest cancel has not answered
        std::set<std::string> unanswered;
        // Goals the controller's cancels have covered; any other live goal gets a new cancel
        std::map<std::string, std::set<GoalId>> canceling;
        std::function<void()> on_canceled;
        rclcpp::TimerBase::SharedPtr timeout;
        bool finished = false;
    };

    void sendCancel(const std::shared_ptr<PendingCancel>& pending, const std::string& controller);
    void settlePendingCancel(const std::shared_ptr<PendingCancel>& pending);
    void finishCancel(const std::shared_ptr<PendingCancel>& pending);
    // Controllers the round still waits on: a cancel unanswered, or a goal not yet terminal
    std::vector<std::string> stillWaitingOn(const PendingCancel& pending) const;
    // Ids of the goals the controller's latest status reports as not yet terminal
    std::set<GoalId> liveGoals(const std::string& controller) const;

    /*!
     * \brief Creates a cancel client and a status subscription for each trajectory action in the graph.
     *
     * A client created at the stop has not discovered its server in time. The spawners load the
     * controllers after this node starts. A timer therefore repeats the prime.
     */
    void primeTrajectoryActionClients();

    bool isConsistent(const std::string& controller) const;

    /*!
     * \brief Starts the controllers stored in stopped_controllers_.
     *
     */
    void startControllers();

    rclcpp::TimerBase::SharedPtr prime_timer_;
    std::vector<std::shared_ptr<PendingCancel>> pending_cancels_;
    std::chrono::nanoseconds cancel_timeout_;

    // The status topic can return to `true` before a stop deactivates. Each deferred step of a stop
    // checks its generation and does nothing once a later start or stop has moved it on.
    uint64_t stop_generation_ = 0;
    // The generation of the last stop that sent its deactivate
    uint64_t deactivated_generation_ = 0;

    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr controller_manager_srv_;
    rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr controller_list_srv_;
    rclcpp::Client<eli_common_interface::srv::GetRobotMode>::SharedPtr dashboard_robot_mode_srv_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr robot_running_sub_;

    std::map<std::string, rclcpp::Client<action_msgs::srv::CancelGoal>::SharedPtr> trajectory_cancel_clients_;
    std::map<std::string, rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr> trajectory_status_subs_;
    // The latest status array from each controller's trajectory action
    std::map<std::string, action_msgs::msg::GoalStatusArray::ConstSharedPtr> trajectory_goal_status_;

    std::vector<std::string> consistent_controllers_;
    std::vector<std::string> stopped_controllers_;

    bool stop_controllers_on_startup_;
    bool robot_running_;
};
#endif
