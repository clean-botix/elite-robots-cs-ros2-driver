#ifndef __ELITE_CS_ROBOT_ROS_DRIVER__CONTROLLER_STOPPER_HPP__
#define __ELITE_CS_ROBOT_ROS_DRIVER__CONTROLLER_STOPPER_HPP__

#include "eli_common_interface/srv/get_robot_mode.hpp"

#include <memory>
#include <string>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>

#include <functional>
#include <map>

class ControllerStopper {
   public:
    ControllerStopper() = delete;
    ControllerStopper(const rclcpp::Node::SharedPtr& node, bool stop_controllers_on_startup);
    virtual ~ControllerStopper() = default;

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
     * \brief Cancels any trajectory goal held by the controllers about to be deactivated, then
     * runs `on_cancelled`.
     *
     * A controller holding an active FollowJointTrajectory goal refuses to deactivate, so the
     * switch fails and the goal outlives the stop. It then reports success once its trajectory
     * buffer drains, telling the caller a motion completed that the robot abandoned where it
     * stood. Cancelling first lets the switch through and gives the caller the failure it is
     * owed.
     *
     * Goals are cancelled on `<controller>/follow_joint_trajectory`. A controller with no such
     * action has no server to answer and is skipped.
     */
    void cancelTrajectoryGoals(const std::vector<std::string>& controllers, std::function<void()> on_cancelled);

    /*!
     * \brief Creates the trajectory action clients up front, one per known controller.
     *
     * Discovery is why this cannot wait until a stop arrives. A client created at that moment has
     * not found its server yet, so it reports not ready, the cancel is skipped, and the deactivate
     * fails exactly as it would with no cancel at all. There is no time to wait for discovery
     * then: the goal reaches its terminal state a few hundred milliseconds after the stop.
     */
    void primeTrajectoryActionClients();

    /*!
     * \brief Starts the controllers stored in stopped_controllers_.
     *
     */
    void startControllers();

    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr controller_manager_srv_;
    rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr controller_list_srv_;
    rclcpp::Client<eli_common_interface::srv::GetRobotMode>::SharedPtr dashboard_robot_mode_srv_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr robot_running_sub_;

    // One action client per controller name, created on first use and reused afterwards:
    // creating one per stop would rediscover the server every time.
    std::map<std::string, rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr>
        trajectory_action_clients_;

    std::vector<std::string> consistent_controllers_;
    std::vector<std::string> stopped_controllers_;

    bool stop_controllers_on_startup_;
    bool robot_running_;
};
#endif
