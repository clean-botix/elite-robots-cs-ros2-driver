// Regression tests for ControllerStopper's restart path.
//
// Field failure these cover: cycling the cs612 hardware component (which a driver reset does)
// drops controllers that claim its interfaces from 'inactive' all the way back to 'unconfigured'.
// ControllerStopper used to re-activate its captured stopped_controllers_ list with STRICT
// strictness and no configure step, so:
//   1. the unconfigured controller could never be activated (STRICT or not, activation is only
//      legal from 'inactive'), and
//   2. under STRICT the controller manager rejected the *entire* switch, so every other stopped
//      controller stayed stopped too.
// Nothing ever reconfigured the stranded controller, so every subsequent robot-program cycle
// retried the same impossible switch and the arm never recovered without manual intervention.
//
// These tests drive the real ControllerStopper against a fake controller manager that models the
// controller lifecycle (unconfigured -> inactive -> active) and the STRICT/BEST_EFFORT switch
// semantics, then assert on the requests it actually issued.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include <controller_manager_msgs/srv/configure_controller.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include "eli_common_interface/msg/robot_mode.hpp"
#include "eli_common_interface/srv/get_robot_mode.hpp"
#include "eli_cs_robot_driver/controller_stopper.hpp"

using namespace std::chrono_literals;

using ConfigureController = controller_manager_msgs::srv::ConfigureController;
using GetRobotMode = eli_common_interface::srv::GetRobotMode;
using ListControllers = controller_manager_msgs::srv::ListControllers;
using SwitchController = controller_manager_msgs::srv::SwitchController;

namespace {

constexpr char kJointStateBroadcaster[] = "joint_state_broadcaster";
constexpr char kAdmittanceController[] = "joint_trajectory_admittance_controller";
constexpr char kForwardPositionController[] = "forward_position_controller";

/// One switch_controller request as ControllerStopper issued it.
struct SwitchRequest {
    std::vector<std::string> activate;
    std::vector<std::string> deactivate;
    int32_t strictness;
};

/// Fake controller_manager + dashboard, good enough to exercise the lifecycle rules
/// ControllerStopper depends on. Spun on its own executor thread so ControllerStopper can block on
/// wait_for_service()/spin_until_future_complete() from the test thread during construction.
class FakeControllerManager {
   public:
    FakeControllerManager() : node_(std::make_shared<rclcpp::Node>("fake_controller_manager")) {
        list_srv_ = node_->create_service<ListControllers>(
            "controller_manager/list_controllers",
            [this](const ListControllers::Request::SharedPtr, ListControllers::Response::SharedPtr response) {
                std::lock_guard<std::mutex> lock(mutex_);
                for (const auto& [name, state] : states_) {
                    controller_manager_msgs::msg::ControllerState controller;
                    controller.name = name;
                    controller.state = state;
                    response->controller.push_back(controller);
                }
            });

        switch_srv_ = node_->create_service<SwitchController>(
            "controller_manager/switch_controller",
            [this](const SwitchController::Request::SharedPtr request, SwitchController::Response::SharedPtr response) {
                std::lock_guard<std::mutex> lock(mutex_);
                switch_requests_.push_back(
                    SwitchRequest{request->activate_controllers, request->deactivate_controllers, request->strictness});

                // STRICT: reject the whole switch if any requested transition is illegal, changing
                // nothing. This is what stranded the arm in the field.
                if (request->strictness == SwitchController::Request::STRICT) {
                    for (const auto& name : request->activate_controllers) {
                        if (states_[name] != "inactive") {
                            response->ok = false;
                            return;
                        }
                    }
                    for (const auto& name : request->deactivate_controllers) {
                        if (states_[name] != "active") {
                            response->ok = false;
                            return;
                        }
                    }
                }

                // BEST_EFFORT (and a legal STRICT switch): apply what is legal, skip what is not.
                for (const auto& name : request->activate_controllers) {
                    if (states_[name] == "inactive") {
                        states_[name] = "active";
                    }
                }
                for (const auto& name : request->deactivate_controllers) {
                    if (states_[name] == "active") {
                        states_[name] = "inactive";
                    }
                }
                response->ok = true;
            });

        configure_srv_ = node_->create_service<ConfigureController>(
            "controller_manager/configure_controller",
            [this](const ConfigureController::Request::SharedPtr request,
                   ConfigureController::Response::SharedPtr response) {
                std::lock_guard<std::mutex> lock(mutex_);
                configure_requests_.push_back(request->name);
                if (configure_should_fail_) {
                    response->ok = false;
                    return;
                }
                if (states_[request->name] == "unconfigured") {
                    states_[request->name] = "inactive";
                }
                response->ok = true;
            });

        robot_mode_srv_ = node_->create_service<GetRobotMode>(
            "dashboard_client/robot_mode",
            [](const GetRobotMode::Request::SharedPtr, GetRobotMode::Response::SharedPtr response) {
                response->mode.mode = eli_common_interface::msg::RobotMode::RUNNING;
            });

        executor_.add_node(node_);
        thread_ = std::thread([this]() { executor_.spin(); });
    }

    ~FakeControllerManager() {
        executor_.cancel();
        if (thread_.joinable()) {
            thread_.join();
        }
    }

    void setState(const std::string& name, const std::string& state) {
        std::lock_guard<std::mutex> lock(mutex_);
        states_[name] = state;
    }

    std::string state(const std::string& name) {
        std::lock_guard<std::mutex> lock(mutex_);
        return states_[name];
    }

    void setConfigureShouldFail(bool should_fail) {
        std::lock_guard<std::mutex> lock(mutex_);
        configure_should_fail_ = should_fail;
    }

    std::vector<SwitchRequest> switchRequests() {
        std::lock_guard<std::mutex> lock(mutex_);
        return switch_requests_;
    }

    std::vector<std::string> configureRequests() {
        std::lock_guard<std::mutex> lock(mutex_);
        return configure_requests_;
    }

   private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Service<ListControllers>::SharedPtr list_srv_;
    rclcpp::Service<SwitchController>::SharedPtr switch_srv_;
    rclcpp::Service<ConfigureController>::SharedPtr configure_srv_;
    rclcpp::Service<GetRobotMode>::SharedPtr robot_mode_srv_;
    rclcpp::executors::MultiThreadedExecutor executor_;
    std::thread thread_;

    std::mutex mutex_;
    std::map<std::string, std::string> states_;
    std::vector<SwitchRequest> switch_requests_;
    std::vector<std::string> configure_requests_;
    bool configure_should_fail_ = false;
};

/// Poll `predicate` until it holds or the timeout expires. Returns whether it held.
template <typename Predicate>
bool waitFor(Predicate predicate, std::chrono::milliseconds timeout = 5s) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (predicate()) {
            return true;
        }
        std::this_thread::sleep_for(10ms);
    }
    return predicate();
}

class ControllerStopperTest : public ::testing::Test {
   protected:
    void SetUp() override {
        fake_ = std::make_unique<FakeControllerManager>();
        // The consistent controller stays up across robot-program cycles; the other two are the
        // ones ControllerStopper captures and must bring back.
        fake_->setState(kJointStateBroadcaster, "active");
        fake_->setState(kAdmittanceController, "active");
        fake_->setState(kForwardPositionController, "active");

        rclcpp::NodeOptions options;
        options.parameter_overrides(
            {rclcpp::Parameter("consistent_controllers", std::vector<std::string>{kJointStateBroadcaster})});
        node_ = std::make_shared<rclcpp::Node>("controller_stopper_under_test", options);

        robot_running_pub_ = node_->create_publisher<std_msgs::msg::Bool>("io_and_status_controller/robot_task_running", 1);

        // Construct before spinning: the constructor drives its own spin_until_future_complete().
        stopper_ = std::make_unique<ControllerStopper>(node_, /*stop_controllers_on_startup=*/false);

        executor_.add_node(node_);
        spin_thread_ = std::thread([this]() { executor_.spin(); });

        // ControllerStopper only reacts to *changes*, and starts believing the robot is running.
        ASSERT_TRUE(waitFor([this]() { return robot_running_pub_->get_subscription_count() > 0; }));
    }

    void TearDown() override {
        executor_.cancel();
        if (spin_thread_.joinable()) {
            spin_thread_.join();
        }
        stopper_.reset();
        node_.reset();
        fake_.reset();
    }

    void publishRobotRunning(bool running) {
        std_msgs::msg::Bool msg;
        msg.data = running;
        robot_running_pub_->publish(msg);
    }

    /// Drive a full stop cycle so ControllerStopper captures the two non-consistent controllers.
    void stopControllers() {
        publishRobotRunning(false);
        ASSERT_TRUE(waitFor([this]() {
            return fake_->state(kAdmittanceController) == "inactive" &&
                   fake_->state(kForwardPositionController) == "inactive";
        })) << "controllers were never deactivated";
    }

    std::unique_ptr<FakeControllerManager> fake_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr robot_running_pub_;
    std::unique_ptr<ControllerStopper> stopper_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    std::thread spin_thread_;
};

// --- the field regression ----------------------------------------------------

TEST_F(ControllerStopperTest, ConfiguresAnUnconfiguredControllerBeforeActivatingIt) {
    stopControllers();

    // A hardware-component cycle (driver reset) knocks the admittance controller past 'inactive'
    // all the way back to 'unconfigured' while the robot program is stopped.
    fake_->setState(kAdmittanceController, "unconfigured");

    publishRobotRunning(true);

    ASSERT_TRUE(waitFor([this]() { return fake_->state(kAdmittanceController) == "active"; }))
        << "unconfigured controller was never recovered -- it must be configured before activation";

    const auto configured = fake_->configureRequests();
    EXPECT_NE(std::find(configured.begin(), configured.end(), kAdmittanceController), configured.end())
        << "expected a configure_controller request for the unconfigured controller";
}

TEST_F(ControllerStopperTest, OneUnrecoverableControllerDoesNotBlockTheOthers) {
    stopControllers();

    // The admittance controller is unconfigured *and* cannot be configured (e.g. its hardware
    // interfaces are still unavailable). Everything else must still come back.
    fake_->setState(kAdmittanceController, "unconfigured");
    fake_->setConfigureShouldFail(true);

    publishRobotRunning(true);

    ASSERT_TRUE(waitFor([this]() { return fake_->state(kForwardPositionController) == "active"; }))
        << "a controller that could not be recovered blocked activation of every other controller";
    EXPECT_EQ(fake_->state(kAdmittanceController), "unconfigured");
}

// --- strictness --------------------------------------------------------------

TEST_F(ControllerStopperTest, ActivationUsesBestEffortStrictness) {
    stopControllers();
    publishRobotRunning(true);

    ASSERT_TRUE(waitFor([this]() { return fake_->state(kAdmittanceController) == "active"; }));

    bool saw_activation = false;
    for (const auto& request : fake_->switchRequests()) {
        if (request.activate.empty()) {
            continue;
        }
        saw_activation = true;
        EXPECT_EQ(request.strictness, SwitchController::Request::BEST_EFFORT)
            << "STRICT activation lets one stranded controller reject the whole switch";
    }
    EXPECT_TRUE(saw_activation) << "no activation switch was ever issued";
}

TEST_F(ControllerStopperTest, DeactivationUsesBestEffortStrictness) {
    stopControllers();

    bool saw_deactivation = false;
    for (const auto& request : fake_->switchRequests()) {
        if (request.deactivate.empty()) {
            continue;
        }
        saw_deactivation = true;
        EXPECT_EQ(request.strictness, SwitchController::Request::BEST_EFFORT)
            << "STRICT deactivation leaves controllers running when the robot program has stopped";
    }
    EXPECT_TRUE(saw_deactivation) << "no deactivation switch was ever issued";
}

}  // namespace

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    const int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
