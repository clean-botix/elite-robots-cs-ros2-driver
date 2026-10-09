#ifndef __ELITE_CS_ROBOT_ROS_DRIVER__CONTROLLER_STOPPER_HPP__
#define __ELITE_CS_ROBOT_ROS_DRIVER__CONTROLLER_STOPPER_HPP__

#include "eli_common_interface/srv/get_robot_mode.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <action_msgs/srv/cancel_goal.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <functional>
#include <map>
#include <set>

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
     */
    void cancelTrajectoryGoals(const std::vector<std::string>& controllers, std::function<void()> on_canceled);

    /*!
     * \brief Creates a trajectory cancel client for each controller that has none yet.
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
    // One timer per stop that still waits for cancel answers
    std::vector<rclcpp::TimerBase::SharedPtr> cancel_timeout_timers_;
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
    // Controllers whose trajectory server has been ready at least once
    std::set<std::string> trajectory_servers_seen_;

    std::vector<std::string> consistent_controllers_;
    std::vector<std::string> stopped_controllers_;

    bool stop_controllers_on_startup_;
    bool robot_running_;
};
#endif
