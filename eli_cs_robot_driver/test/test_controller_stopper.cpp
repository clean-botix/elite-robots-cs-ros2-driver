// Regression tests for ControllerStopper's trajectory cancel.
//
// The invariant: when the robot task stops, the trajectory goal is canceled BEFORE the
// controllers are deactivated. A controller holding an active FollowJointTrajectory goal
// refuses to deactivate, so without the cancel the switch fails and the goal outlives the
// stop, reporting success once its buffer drains. The caller is then told a motion
// completed that the robot abandoned where it stood.
//
// These stand up the real collaborators the stopper talks to -- both controller manager
// services, the dashboard mode service, and a trajectory action server holding a live goal
// -- because the failure this pins was invisible to any test of the class in isolation: the
// action client was created at the moment of the stop and had not discovered the server yet,
// so the cancel was skipped and everything else behaved normally.

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <eli_common_interface/srv/get_robot_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>

#include "eli_cs_robot_driver/controller_stopper.hpp"

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using namespace std::chrono_literals;

namespace {

constexpr char TRAJECTORY_CONTROLLER[] = "joint_trajectory_admittance_controller";
constexpr char CONSISTENT_CONTROLLER[] = "joint_state_broadcaster";

// What the stopper did, in the order it did it. Ordering is the contract under test.
class EventLog {
   public:
    void record(const std::string& event) {
        std::lock_guard<std::mutex> lock(mutex_);
        events_.push_back(event);
    }

    std::vector<std::string> events() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return events_;
    }

    bool contains(const std::string& event) const {
        const auto seen = events();
        return std::find(seen.begin(), seen.end(), event) != seen.end();
    }

   private:
    mutable std::mutex mutex_;
    std::vector<std::string> events_;
};

// The controller manager, the dashboard and the arm, as far as the stopper can tell.
class FakeRobot {
   public:
    explicit FakeRobot(EventLog& log) : node_(std::make_shared<rclcpp::Node>("fake_robot")), log_(log) {
        list_srv_ = node_->create_service<controller_manager_msgs::srv::ListControllers>(
            "controller_manager/list_controllers",
            [this](const controller_manager_msgs::srv::ListControllers::Request::SharedPtr,
                   controller_manager_msgs::srv::ListControllers::Response::SharedPtr response) {
                if (!controllers_loaded_.load()) {
                    return;
                }
                controller_manager_msgs::msg::ControllerState trajectory;
                trajectory.name = TRAJECTORY_CONTROLLER;
                trajectory.state = "active";
                controller_manager_msgs::msg::ControllerState consistent;
                consistent.name = CONSISTENT_CONTROLLER;
                consistent.state = "active";
                response->controller = {trajectory, consistent};
            });

        switch_srv_ = node_->create_service<controller_manager_msgs::srv::SwitchController>(
            "controller_manager/switch_controller",
            [this](const controller_manager_msgs::srv::SwitchController::Request::SharedPtr request,
                   controller_manager_msgs::srv::SwitchController::Response::SharedPtr response) {
                if (!request->deactivate_controllers.empty()) {
                    log_.record("deactivate");
                }
                response->ok = true;
            });

        mode_srv_ = node_->create_service<eli_common_interface::srv::GetRobotMode>(
            "dashboard_client/robot_mode",
            [](const eli_common_interface::srv::GetRobotMode::Request::SharedPtr,
               eli_common_interface::srv::GetRobotMode::Response::SharedPtr response) {
                response->mode.mode = eli_common_interface::msg::RobotMode::RUNNING;
            });

        action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
            node_, std::string(TRAJECTORY_CONTROLLER) + "/follow_joint_trajectory",
            [](const rclcpp_action::GoalUUID&, std::shared_ptr<const FollowJointTrajectory::Goal>) {
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            },
            [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJointTrajectory>>) {
                log_.record("cancel");
                return rclcpp_action::CancelResponse::ACCEPT;
            },
            [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJointTrajectory>> handle) {
                // Held, never completed: a trajectory still running when the arm stops
                std::lock_guard<std::mutex> lock(goal_mutex_);
                goal_handle_ = handle;
            });

        task_running_pub_ = node_->create_publisher<std_msgs::msg::Bool>("io_and_status_controller/robot_task_running", 1);
    }

    rclcpp::Node::SharedPtr node() { return node_; }

    void publishTaskRunning(bool running) {
        std_msgs::msg::Bool msg;
        msg.data = running;
        task_running_pub_->publish(msg);
    }

    // The spawners load the controllers well after this node starts; until then the controller
    // manager lists nothing.
    void loadControllers() { controllers_loaded_.store(true); }

    void hideControllers() { controllers_loaded_.store(false); }

    bool holdsGoal() {
        std::lock_guard<std::mutex> lock(goal_mutex_);
        return goal_handle_ != nullptr;
    }

   private:
    rclcpp::Node::SharedPtr node_;
    EventLog& log_;
    rclcpp::Service<controller_manager_msgs::srv::ListControllers>::SharedPtr list_srv_;
    rclcpp::Service<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_srv_;
    rclcpp::Service<eli_common_interface::srv::GetRobotMode>::SharedPtr mode_srv_;
    rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr task_running_pub_;
    std::mutex goal_mutex_;
    std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJointTrajectory>> goal_handle_;
    std::atomic<bool> controllers_loaded_{true};
};

class ControllerStopperTest : public ::testing::Test {
   protected:
    void SetUp() override {
        robot_ = std::make_unique<FakeRobot>(log_);
        client_node_ = std::make_shared<rclcpp::Node>("trajectory_goal_sender");
        robot_executor_.add_node(robot_->node());
        robot_executor_.add_node(client_node_);
        robot_thread_ = std::thread([this]() { robot_executor_.spin(); });

        rclcpp::NodeOptions options;
        options.parameter_overrides(
            {rclcpp::Parameter("consistent_controllers", std::vector<std::string>{CONSISTENT_CONTROLLER})});
        stopper_node_ = std::make_shared<rclcpp::Node>("controller_stopper_under_test", options);

        // The constructor waits on every service and spins its own node while it does
        stopper_ = std::make_unique<ControllerStopper>(stopper_node_, false);

        sendTrajectoryGoal();
    }

    void TearDown() override {
        robot_executor_.cancel();
        if (robot_thread_.joinable()) {
            robot_thread_.join();
        }
        stopper_.reset();
    }

    // A trajectory is running when the arm stops; without a live goal there is nothing to cancel
    void sendTrajectoryGoal() {
        auto client = rclcpp_action::create_client<FollowJointTrajectory>(
            client_node_, std::string(TRAJECTORY_CONTROLLER) + "/follow_joint_trajectory");
        ASSERT_TRUE(client->wait_for_action_server(10s)) << "fake trajectory action server never came up";
        client->async_send_goal(FollowJointTrajectory::Goal());
        ASSERT_TRUE(spinUntil([this]() { return robot_->holdsGoal(); }, 10s)) << "goal never reached the server";
        goal_client_ = client;
    }

    bool spinUntil(const std::function<bool()>& done, std::chrono::nanoseconds timeout) {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline) {
            if (done()) {
                return true;
            }
            rclcpp::spin_some(stopper_node_);
            std::this_thread::sleep_for(10ms);
        }
        return done();
    }

    EventLog log_;
    std::unique_ptr<FakeRobot> robot_;
    rclcpp::executors::SingleThreadedExecutor robot_executor_;
    std::thread robot_thread_;
    rclcpp::Node::SharedPtr client_node_;
    rclcpp::Node::SharedPtr stopper_node_;
    std::unique_ptr<ControllerStopper> stopper_;
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr goal_client_;
};

// Boot order on the robot: this node comes up with the driver, before the spawners have loaded
// any controller, so the first listing is empty. A client created only at that moment covers
// nothing, and a client created at the stop has not discovered its server in time.
class ControllerStopperLateControllersTest : public ::testing::Test {
   protected:
    void SetUp() override {
        robot_ = std::make_unique<FakeRobot>(log_);
        robot_->hideControllers();
        client_node_ = std::make_shared<rclcpp::Node>("late_trajectory_goal_sender");
        robot_executor_.add_node(robot_->node());
        robot_executor_.add_node(client_node_);
        robot_thread_ = std::thread([this]() { robot_executor_.spin(); });

        rclcpp::NodeOptions options;
        options.parameter_overrides(
            {rclcpp::Parameter("consistent_controllers", std::vector<std::string>{CONSISTENT_CONTROLLER})});
        stopper_node_ = std::make_shared<rclcpp::Node>("controller_stopper_late_controllers", options);
        stopper_ = std::make_unique<ControllerStopper>(stopper_node_, false);
    }

    void TearDown() override {
        robot_executor_.cancel();
        if (robot_thread_.joinable()) {
            robot_thread_.join();
        }
        stopper_.reset();
    }

    bool spinFor(std::chrono::nanoseconds duration) {
        const auto deadline = std::chrono::steady_clock::now() + duration;
        while (std::chrono::steady_clock::now() < deadline) {
            rclcpp::spin_some(stopper_node_);
            std::this_thread::sleep_for(10ms);
        }
        return true;
    }

    bool spinUntil(const std::function<bool()>& done, std::chrono::nanoseconds timeout) {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline) {
            if (done()) {
                return true;
            }
            rclcpp::spin_some(stopper_node_);
            std::this_thread::sleep_for(10ms);
        }
        return done();
    }

    EventLog log_;
    std::unique_ptr<FakeRobot> robot_;
    rclcpp::executors::SingleThreadedExecutor robot_executor_;
    std::thread robot_thread_;
    rclcpp::Node::SharedPtr client_node_;
    rclcpp::Node::SharedPtr stopper_node_;
    std::unique_ptr<ControllerStopper> stopper_;
};

TEST_F(ControllerStopperLateControllersTest, CancelsAControllerThatLoadedAfterStartup) {
    robot_->loadControllers();
    // Long enough for the repeating prime to notice them and for discovery to settle
    spinFor(6s);

    auto client = rclcpp_action::create_client<FollowJointTrajectory>(
        client_node_, std::string(TRAJECTORY_CONTROLLER) + "/follow_joint_trajectory");
    ASSERT_TRUE(client->wait_for_action_server(10s));
    client->async_send_goal(FollowJointTrajectory::Goal());
    ASSERT_TRUE(spinUntil([this]() { return robot_->holdsGoal(); }, 10s)) << "goal never reached the server";

    robot_->publishTaskRunning(false);

    ASSERT_TRUE(spinUntil([this]() { return log_.contains("cancel"); }, 10s))
        << "a controller loaded after this node started was never given a cancel client";
}

TEST_F(ControllerStopperTest, CancelsTheTrajectoryGoalWhenTheRobotTaskStops) {
    robot_->publishTaskRunning(false);

    ASSERT_TRUE(spinUntil([this]() { return log_.contains("cancel"); }, 10s))
        << "no cancel reached the trajectory action server, so the controller would refuse to "
           "deactivate and the goal would outlive the stop";
}

TEST_F(ControllerStopperTest, CancelsBeforeDeactivating) {
    robot_->publishTaskRunning(false);

    ASSERT_TRUE(spinUntil([this]() { return log_.contains("deactivate"); }, 10s)) << "controllers were never deactivated";

    const auto events = log_.events();
    const auto cancel = std::find(events.begin(), events.end(), "cancel");
    const auto deactivate = std::find(events.begin(), events.end(), "deactivate");
    ASSERT_NE(cancel, events.end()) << "the goal was never canceled";
    EXPECT_LT(cancel, deactivate) << "deactivating before the cancel is what the controller refuses";
}

TEST_F(ControllerStopperTest, LeavesTheGoalAloneWhileTheRobotTaskRuns) {
    robot_->publishTaskRunning(true);
    spinUntil([]() { return false; }, 1s);

    EXPECT_FALSE(log_.contains("cancel")) << "a running task is not a reason to cancel: this is the pause case";
    EXPECT_FALSE(log_.contains("deactivate"));
}

}  // namespace

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    const int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
