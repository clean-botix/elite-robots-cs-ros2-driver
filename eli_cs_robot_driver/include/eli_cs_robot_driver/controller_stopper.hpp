#ifndef __ELITE_CS_ROBOT_ROS_DRIVER__CONTROLLER_STOPPER_HPP__
#define __ELITE_CS_ROBOT_ROS_DRIVER__CONTROLLER_STOPPER_HPP__

#include "eli_common_interface/srv/get_robot_mode.hpp"

#include <memory>
#include <string>
#include <vector>

#include <controller_manager_msgs/srv/configure_controller.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

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
     * \brief Starts the controllers stored in stopped_controllers_.
     *
     * A controller can only be activated from the 'inactive' state. Anything that resets the
     * hardware interfaces underneath a controller (e.g. cycling the hardware component after a
     * driver reset) drops it all the way back to 'unconfigured', where it can never be activated
     * again. So this first queries the current state of every stopped controller and configures
     * the unconfigured ones, then activates. See activateStoppedControllers().
     */
    void startControllers();

    /*!
     * \brief Activates the controllers stored in stopped_controllers_ with BEST_EFFORT strictness.
     *
     * Called once every controller in stopped_controllers_ that needed configuring has been
     * configured. BEST_EFFORT (rather than STRICT) so that a single controller that cannot be
     * brought back -- unloaded, still unconfigured, hardware interface unavailable -- does not
     * cause the controller manager to reject the switch wholesale and leave every other
     * controller stopped too.
     */
    void activateStoppedControllers();

    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr controller_manager_srv_;
    rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr controller_list_srv_;
    rclcpp::Client<controller_manager_msgs::srv::ConfigureController>::SharedPtr configure_controller_srv_;
    rclcpp::Client<eli_common_interface::srv::GetRobotMode>::SharedPtr dashboard_robot_mode_srv_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr robot_running_sub_;

    std::vector<std::string> consistent_controllers_;
    std::vector<std::string> stopped_controllers_;

    bool stop_controllers_on_startup_;
    bool robot_running_;
};
#endif
