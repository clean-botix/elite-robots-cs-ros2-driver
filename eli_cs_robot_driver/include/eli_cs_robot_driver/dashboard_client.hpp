#ifndef __ELITE_CS_ROBOT_ROS_DRIVER__DASHBOARD_CLIENT_HPP__
#define __ELITE_CS_ROBOT_ROS_DRIVER__DASHBOARD_CLIENT_HPP__

#include <Elite/DashboardClient.hpp>
#include <Elite/EliteException.hpp>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "eli_dashboard_interface/srv/load.hpp"
#include "eli_dashboard_interface/srv/popup.hpp"
#include "eli_dashboard_interface/srv/log.hpp"
#include "eli_common_interface/srv/get_task_status.hpp"
#include "eli_dashboard_interface/srv/is_saved.hpp"
#include "eli_common_interface/srv/get_robot_mode.hpp"
#include "eli_common_interface/srv/get_safety_mode.hpp"
#include "eli_dashboard_interface/srv/custom_request.hpp"

namespace ELITE_CS_ROBOT_ROS_DRIVER {

/**
 * ROS2 node that wraps the Elite dashboard client and exposes its operations
 * as ROS services. Maintains a persistent, self-healing connection to the
 * robot's dashboard port.
 *
 * Resilience mechanisms:
 *
 * Startup connection: a periodic timer fires attemptConnection() every
 * connect_retry_interval_s_ seconds until the connection succeeds or connect_grace_period_s_
 * seconds have elapsed. Log messages are throttled to at most one per 5 seconds.
 *
 * Liveness monitoring: once connected, a health-check timer fires checkConnection()
 * every health_check_interval_ seconds. An echo probe detects silent connection
 * drops. On failure, the health-check timer is cancelled and the connection timer
 * is re-armed, beginning a new reconnection window.
 *
 * Service fault tolerance: all service callbacks share a single log throttle gate
 * (last_service_failure_log_time_). Exceptions from underlying dashboard calls emit
 * at most one WARN per 5 seconds, preventing log flooding when the arm is transiently
 * unreachable while services are being invoked.
 */
class DashboardClient : public rclcpp::Node {
   private:
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr connect_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr power_on_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr remote_control_on_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr remote_control_off_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr power_off_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr play_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr brake_release_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr close_safety_dialog_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr unlock_protective_stop_service_;
    rclcpp::Service<eli_dashboard_interface::srv::Load>::SharedPtr load_configure_service_;
    rclcpp::Service<eli_dashboard_interface::srv::Load>::SharedPtr load_task_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr get_task_path_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr shutdown_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reboot_service_;
    rclcpp::Service<eli_dashboard_interface::srv::Popup>::SharedPtr popup_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr quit_service_;
    rclcpp::Service<eli_dashboard_interface::srv::Log>::SharedPtr add_log_service_;
    rclcpp::Service<eli_common_interface::srv::GetTaskStatus>::SharedPtr task_status_service_;
    rclcpp::Service<eli_dashboard_interface::srv::IsSaved>::SharedPtr is_task_saved_service_;
    rclcpp::Service<eli_dashboard_interface::srv::IsSaved>::SharedPtr is_configuration_saved_service_;
    rclcpp::Service<eli_common_interface::srv::GetRobotMode>::SharedPtr robot_mode_service_;
    rclcpp::Service<eli_common_interface::srv::GetSafetyMode>::SharedPtr safety_mode_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr safety_system_restart_service_;
    rclcpp::Service<eli_dashboard_interface::srv::CustomRequest>::SharedPtr custom_request_service_;

    ELITE::DashboardClient client_;

    std::string robot_ip_;
    double connect_grace_period_s_{ 30.0 };
    std::chrono::time_point<std::chrono::steady_clock> start_time_;

    // Connection lifecycle timers.
    // connection_timer_ drives attemptConnection() until connected.
    // health_check_timer_ drives checkConnection() while connected; replaced by
    // connection_timer_ when a liveness probe fails.
    rclcpp::TimerBase::SharedPtr connection_timer_;
    rclcpp::TimerBase::SharedPtr health_check_timer_;
    double connect_retry_interval_s_{ 2.0 };
    double health_check_interval_{ 5.0 };
    bool connected_{ false };
    bool is_reconnecting_{ false };
    // Log throttle gates — each caps its associated message category to 1 per 5 s
    std::chrono::steady_clock::time_point last_connect_attempt_log_time_{};
    std::chrono::steady_clock::time_point last_service_failure_log_time_{};

    // Standard throttle interval for all repeated log messages — connection attempts,
    // failures, and service exceptions. One log per this many seconds prevents flooding
    // during extended outages.
    static constexpr double kLogThrottleSeconds = 5.0;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr createTriggerService(const std::string& name, std::function<bool()> func) {
        return this->create_service<std_srvs::srv::Trigger>(name, [&, func](const std_srvs::srv::Trigger::Request::SharedPtr req,
                                                                            std_srvs::srv::Trigger::Response::SharedPtr resp) {
            (void)req;
            try {
                resp->success = func();
            } catch (const ELITE::EliteException& e) {
                logServiceFailure(e);
                resp->success = false;
                resp->message = e.what();
            }
        });
    }

    // Rate-limited WARN logger for service handler catch blocks.
    // Emits at most one warning per kServiceFailureLogThrottleSeconds to avoid
    // log flooding when the robot is transiently unreachable. All service
    // handlers share the same throttle gate (last_service_failure_log_time_),
    // so the limit applies across all services combined.
    void logServiceFailure(const ELITE::EliteException& e) {
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration<double>(now - last_service_failure_log_time_).count()
                >= kLogThrottleSeconds) {
            RCLCPP_WARN(rclcpp::get_logger("EliteCSDashboardInterface"),
                "Dashboard service call failed: %s", e.what());
            last_service_failure_log_time_ = now;
        }
    }

    void attemptConnection();
    void checkConnection();
    bool timeoutExpired(std::chrono::time_point<std::chrono::steady_clock>, int);

   public:
    DashboardClient(const rclcpp::NodeOptions& options);
    ~DashboardClient();
};


}

#endif
