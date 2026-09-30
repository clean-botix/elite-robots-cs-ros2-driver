// Regression tests for ControllerStopper's trajectory cancel.
//
// The invariant: when the robot task stops, the trajectory goal is canceled BEFORE the
// controllers are deactivated. A controller holding an active FollowJointTrajectory goal
// refuses to deactivate, so without the cancel the switch fails and the goal outlives the
// stop, reporting success once its buffer drains. The caller is then told a motion
// completed that the robot abandoned where it stood.
//
// The cancel is best effort, so the second invariant is that the deactivate always follows:
// whether the cancel is accepted, refused or never answered, and whether or not a stopped
// controller has a trajectory action at all.
//
// These stand up the real collaborators the stopper talks to -- both controller manager
// services, the dashboard mode service, and trajectory action servers holding live goals
// -- because the failure this pins was invisible to any test of the class in isolation: the
// action client was created at the moment of the stop and had not discovered the server yet,
// so the cancel was skipped and everything else behaved normally.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <map>
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
using GoalHandle = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;
using namespace std::chrono_literals;

namespace {

constexpr char TRAJECTORY_CONTROLLER[] = "joint_trajectory_admittance_controller";
constexpr char SECOND_TRAJECTORY_CONTROLLER[] = "scaled_joint_trajectory_controller";
constexpr char NO_ACTION_CONTROLLER[] = "forward_position_controller";
constexpr char CONSISTENT_CONTROLLER[] = "joint_state_broadcaster";

// Short so the never-answered case finishes quickly; the default on the robot is 1 s
constexpr double CANCEL_TIMEOUT_S = 0.5;

std::string cancelEvent(const std::string& controller) { return "cancel:" + controller; }

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

    bool contains(const std::string& event) const { return count(event) > 0; }

    std::size_t count(const std::string& event) const {
        const auto seen = events();
        return static_cast<std::size_t>(std::count(seen.begin(), seen.end(), event));
    }

    // Index of the first occurrence, or the size of the log if there is none
    std::size_t indexOf(const std::string& event) const {
        const auto seen = events();
        return static_cast<std::size_t>(std::find(seen.begin(), seen.end(), event) - seen.begin());
    }

   private:
    mutable std::mutex mutex_;
    std::vector<std::string> events_;
};

// How the fake trajectory servers answer a cancel request
enum class CancelMode {
    ACCEPT,
    REJECT,
    // The server is wedged: the cancel is received but never answered
    NEVER_ANSWER,
};

struct FakeController {
    std::string name;
    // Whether it runs <name>/follow_joint_trajectory. A forward command controller does not.
    bool has_trajectory_action;
};

struct FakeRobotConfig {
    std::vector<FakeController> controllers{{TRAJECTORY_CONTROLLER, true}, {CONSISTENT_CONTROLLER, false}};
    CancelMode cancel_mode = CancelMode::ACCEPT;
    // The spawners load the controllers well after the driver starts; until then nothing is listed
    bool controllers_loaded = true;
};

// The controller manager, the dashboard and the arm, as far as the stopper can tell.
//
// The trajectory action servers live on their own node so they can be spun by their own executor:
// a wedged server must not also wedge the controller manager, or the timeout could not be seen.
class FakeRobot {
   public:
    FakeRobot(EventLog& log, FakeRobotConfig config)
        : node_(std::make_shared<rclcpp::Node>("fake_robot")),
          action_node_(std::make_shared<rclcpp::Node>("fake_robot_actions")),
          log_(log),
          config_(std::move(config)),
          controllers_loaded_(config_.controllers_loaded) {
        list_srv_ = node_->create_service<controller_manager_msgs::srv::ListControllers>(
            "controller_manager/list_controllers",
            [this](const controller_manager_msgs::srv::ListControllers::Request::SharedPtr,
                   controller_manager_msgs::srv::ListControllers::Response::SharedPtr response) {
                list_requests_.fetch_add(1);
                if (!controllers_loaded_.load()) {
                    return;
                }
                for (const auto& controller : config_.controllers) {
                    controller_manager_msgs::msg::ControllerState state;
                    state.name = controller.name;
                    state.state = "active";
                    response->controller.push_back(state);
                }
            });

        switch_srv_ = node_->create_service<controller_manager_msgs::srv::SwitchController>(
            "controller_manager/switch_controller",
            [this](const controller_manager_msgs::srv::SwitchController::Request::SharedPtr request,
                   controller_manager_msgs::srv::SwitchController::Response::SharedPtr response) {
                if (!request->deactivate_controllers.empty()) {
                    {
                        std::lock_guard<std::mutex> lock(mutex_);
                        deactivated_ = request->deactivate_controllers;
                    }
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

        for (const auto& controller : config_.controllers) {
            if (!controller.has_trajectory_action) {
                continue;
            }
            const std::string name = controller.name;
            action_servers_.push_back(rclcpp_action::create_server<FollowJointTrajectory>(
                action_node_, name + "/follow_joint_trajectory",
                [](const rclcpp_action::GoalUUID&, std::shared_ptr<const FollowJointTrajectory::Goal>) {
                    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
                },
                [this, name](const std::shared_ptr<GoalHandle>) { return answerCancel(name); },
                [this, name](const std::shared_ptr<GoalHandle> handle) {
                    // Held, never completed: a trajectory still running when the arm stops
                    std::lock_guard<std::mutex> lock(mutex_);
                    goal_handles_[name] = handle;
                }));
        }

        task_running_pub_ = node_->create_publisher<std_msgs::msg::Bool>("io_and_status_controller/robot_task_running", 1);
    }

    rclcpp::Node::SharedPtr node() { return node_; }
    rclcpp::Node::SharedPtr actionNode() { return action_node_; }

    void publishTaskRunning(bool running) {
        std_msgs::msg::Bool msg;
        msg.data = running;
        task_running_pub_->publish(msg);
    }

    void loadControllers() { controllers_loaded_.store(true); }

    bool holdsGoal(const std::string& controller) {
        std::lock_guard<std::mutex> lock(mutex_);
        return goal_handles_.count(controller) > 0;
    }

    std::vector<std::string> deactivated() {
        std::lock_guard<std::mutex> lock(mutex_);
        return deactivated_;
    }

    int listRequests() const { return list_requests_.load(); }

    // Lets a wedged cancel return so the action executor can shut down
    void releaseCancels() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            released_ = true;
        }
        released_cv_.notify_all();
    }

   private:
    rclcpp_action::CancelResponse answerCancel(const std::string& controller) {
        log_.record(cancelEvent(controller));
        switch (config_.cancel_mode) {
            case CancelMode::ACCEPT:
                return rclcpp_action::CancelResponse::ACCEPT;
            case CancelMode::REJECT:
                return rclcpp_action::CancelResponse::REJECT;
            case CancelMode::NEVER_ANSWER: {
                // Blocks the action executor, so no response is ever sent while the test runs
                std::unique_lock<std::mutex> lock(mutex_);
                released_cv_.wait(lock, [this]() { return released_; });
                return rclcpp_action::CancelResponse::REJECT;
            }
        }
        return rclcpp_action::CancelResponse::REJECT;
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Node::SharedPtr action_node_;
    EventLog& log_;
    FakeRobotConfig config_;
    rclcpp::Service<controller_manager_msgs::srv::ListControllers>::SharedPtr list_srv_;
    rclcpp::Service<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_srv_;
    rclcpp::Service<eli_common_interface::srv::GetRobotMode>::SharedPtr mode_srv_;
    std::vector<rclcpp_action::Server<FollowJointTrajectory>::SharedPtr> action_servers_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr task_running_pub_;
    std::mutex mutex_;
    std::condition_variable released_cv_;
    bool released_ = false;
    std::map<std::string, std::shared_ptr<GoalHandle>> goal_handles_;
    std::vector<std::string> deactivated_;
    std::atomic<bool> controllers_loaded_;
    std::atomic<int> list_requests_{0};
};

class ControllerStopperTest : public ::testing::Test {
   protected:
    // Not SetUp: each test picks the robot it runs against
    void start(FakeRobotConfig config = FakeRobotConfig()) {
        robot_ = std::make_unique<FakeRobot>(log_, std::move(config));
        client_node_ = std::make_shared<rclcpp::Node>("trajectory_goal_sender");
        robot_executor_.add_node(robot_->node());
        robot_executor_.add_node(client_node_);
        action_executor_.add_node(robot_->actionNode());
        robot_thread_ = std::thread([this]() { robot_executor_.spin(); });
        action_thread_ = std::thread([this]() { action_executor_.spin(); });

        rclcpp::NodeOptions options;
        options.parameter_overrides(
            {rclcpp::Parameter("consistent_controllers", std::vector<std::string>{CONSISTENT_CONTROLLER}),
             rclcpp::Parameter("trajectory_cancel_timeout", CANCEL_TIMEOUT_S)});
        stopper_node_ = std::make_shared<rclcpp::Node>("controller_stopper_under_test", options);

        // The constructor waits on every service and spins its own node while it does
        stopper_ = std::make_unique<ControllerStopper>(stopper_node_, false);
    }

    void TearDown() override {
        if (robot_) {
            robot_->releaseCancels();
        }
        robot_executor_.cancel();
        action_executor_.cancel();
        if (robot_thread_.joinable()) {
            robot_thread_.join();
        }
        if (action_thread_.joinable()) {
            action_thread_.join();
        }
        stopper_.reset();
    }

    // A trajectory is running when the arm stops; without a live goal there is nothing to cancel.
    // The stopper is spun meanwhile so its prime completes and its clients discover the server.
    void sendTrajectoryGoal(const std::string& controller) {
        auto client =
            rclcpp_action::create_client<FollowJointTrajectory>(client_node_, controller + "/follow_joint_trajectory");
        ASSERT_TRUE(client->wait_for_action_server(10s)) << "fake trajectory action server never came up";
        client->async_send_goal(FollowJointTrajectory::Goal());
        ASSERT_TRUE(spinUntil([this, controller]() { return robot_->holdsGoal(controller); }, 10s))
            << "goal never reached the server";
        goal_clients_.push_back(client);
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

    void spinFor(std::chrono::nanoseconds duration) {
        spinUntil([]() { return false; }, duration);
    }

    bool deactivated(const std::string& controller) {
        const auto names = robot_->deactivated();
        return std::find(names.begin(), names.end(), controller) != names.end();
    }

    EventLog log_;
    std::unique_ptr<FakeRobot> robot_;
    rclcpp::executors::SingleThreadedExecutor robot_executor_;
    rclcpp::executors::SingleThreadedExecutor action_executor_;
    std::thread robot_thread_;
    std::thread action_thread_;
    rclcpp::Node::SharedPtr client_node_;
    rclcpp::Node::SharedPtr stopper_node_;
    std::unique_ptr<ControllerStopper> stopper_;
    std::vector<rclcpp_action::Client<FollowJointTrajectory>::SharedPtr> goal_clients_;
};

TEST_F(ControllerStopperTest, CancelsTheTrajectoryGoalWhenTheRobotTaskStops) {
    start();
    sendTrajectoryGoal(TRAJECTORY_CONTROLLER);

    robot_->publishTaskRunning(false);

    ASSERT_TRUE(spinUntil([this]() { return log_.contains(cancelEvent(TRAJECTORY_CONTROLLER)); }, 10s))
        << "no cancel reached the trajectory action server, so the controller would refuse to "
           "deactivate and the goal would outlive the stop";
}

TEST_F(ControllerStopperTest, CancelsBeforeDeactivating) {
    start();
    sendTrajectoryGoal(TRAJECTORY_CONTROLLER);

    robot_->publishTaskRunning(false);

    ASSERT_TRUE(spinUntil([this]() { return log_.contains("deactivate"); }, 10s)) << "controllers were never deactivated";
    ASSERT_TRUE(log_.contains(cancelEvent(TRAJECTORY_CONTROLLER))) << "the goal was never canceled";
    EXPECT_LT(log_.indexOf(cancelEvent(TRAJECTORY_CONTROLLER)), log_.indexOf("deactivate"))
        << "deactivating before the cancel is what the controller refuses";
    EXPECT_TRUE(deactivated(TRAJECTORY_CONTROLLER));
    EXPECT_FALSE(deactivated(CONSISTENT_CONTROLLER));
}

TEST_F(ControllerStopperTest, LeavesTheGoalAloneWhileTheRobotTaskRuns) {
    start();
    sendTrajectoryGoal(TRAJECTORY_CONTROLLER);

    robot_->publishTaskRunning(true);
    spinFor(1s);

    EXPECT_FALSE(log_.contains(cancelEvent(TRAJECTORY_CONTROLLER)))
        << "a running task is not a reason to cancel: this is the pause case";
    EXPECT_FALSE(log_.contains("deactivate"));
}

// Boot order on the robot: this node comes up with the driver, before the spawners have loaded
// any controller, so the first listing is empty. A client created only at that moment covers
// nothing, and a client created at the stop has not discovered its server in time.
TEST_F(ControllerStopperTest, CancelsAControllerThatLoadedAfterStartup) {
    FakeRobotConfig config;
    config.controllers_loaded = false;
    start(config);

    robot_->loadControllers();
    // Long enough for the repeating prime to notice them and for discovery to settle
    spinFor(6s);
    sendTrajectoryGoal(TRAJECTORY_CONTROLLER);

    robot_->publishTaskRunning(false);

    ASSERT_TRUE(spinUntil([this]() { return log_.contains(cancelEvent(TRAJECTORY_CONTROLLER)); }, 10s))
        << "a controller loaded after this node started was never given a cancel client";
}

// The controller may refuse to give up its goal. The cancel is best effort, so the deactivate is
// still attempted; whether the controller then allows it is the controller manager's to report.
TEST_F(ControllerStopperTest, StillDeactivatesWhenTheCancelIsRefused) {
    FakeRobotConfig config;
    config.cancel_mode = CancelMode::REJECT;
    start(config);
    sendTrajectoryGoal(TRAJECTORY_CONTROLLER);

    robot_->publishTaskRunning(false);

    ASSERT_TRUE(spinUntil([this]() { return log_.contains("deactivate"); }, 10s))
        << "a refused cancel stranded the deactivate";
    EXPECT_LT(log_.indexOf(cancelEvent(TRAJECTORY_CONTROLLER)), log_.indexOf("deactivate"));
    EXPECT_TRUE(deactivated(TRAJECTORY_CONTROLLER));
}

// A wedged trajectory server must not keep every other stopped controller running: after the
// timeout the deactivate goes ahead without its answer.
TEST_F(ControllerStopperTest, DeactivatesAfterTheTimeoutWhenTheCancelIsNeverAnswered) {
    FakeRobotConfig config;
    config.cancel_mode = CancelMode::NEVER_ANSWER;
    start(config);
    sendTrajectoryGoal(TRAJECTORY_CONTROLLER);

    const auto stopped_at = std::chrono::steady_clock::now();
    robot_->publishTaskRunning(false);

    ASSERT_TRUE(spinUntil([this]() { return log_.contains("deactivate"); }, 10s))
        << "an unanswered cancel blocked the deactivate indefinitely";
    const auto waited = std::chrono::steady_clock::now() - stopped_at;
    EXPECT_TRUE(log_.contains(cancelEvent(TRAJECTORY_CONTROLLER))) << "the cancel was never sent";
    EXPECT_GE(waited, std::chrono::duration<double>(CANCEL_TIMEOUT_S * 0.9))
        << "the deactivate did not wait for the cancel it was meant to wait for";
    EXPECT_TRUE(deactivated(TRAJECTORY_CONTROLLER));
    EXPECT_EQ(log_.count("deactivate"), 1u);
}

// Every trajectory controller is canceled, and the switch waits for the last of them and runs once.
TEST_F(ControllerStopperTest, CancelsEveryTrajectoryControllerBeforeASingleDeactivate) {
    FakeRobotConfig config;
    config.controllers = {{TRAJECTORY_CONTROLLER, true},
                          {SECOND_TRAJECTORY_CONTROLLER, true},
                          {CONSISTENT_CONTROLLER, false}};
    start(config);
    sendTrajectoryGoal(TRAJECTORY_CONTROLLER);
    sendTrajectoryGoal(SECOND_TRAJECTORY_CONTROLLER);

    robot_->publishTaskRunning(false);

    ASSERT_TRUE(spinUntil([this]() { return log_.contains("deactivate"); }, 10s)) << "controllers were never deactivated";
    // Let any stray second switch arrive before counting
    spinFor(1s);
    EXPECT_LT(log_.indexOf(cancelEvent(TRAJECTORY_CONTROLLER)), log_.indexOf("deactivate"));
    EXPECT_LT(log_.indexOf(cancelEvent(SECOND_TRAJECTORY_CONTROLLER)), log_.indexOf("deactivate"));
    EXPECT_EQ(log_.count("deactivate"), 1u);
    EXPECT_TRUE(deactivated(TRAJECTORY_CONTROLLER));
    EXPECT_TRUE(deactivated(SECOND_TRAJECTORY_CONTROLLER));
}

// A controller without a trajectory action gets a primed client that never becomes ready. It is
// skipped for the cancel but still deactivated, and does not hold up the controllers that do have one.
TEST_F(ControllerStopperTest, DeactivatesAControllerWithNoTrajectoryServerAlongsideOneThatHasOne) {
    FakeRobotConfig config;
    config.controllers = {{TRAJECTORY_CONTROLLER, true}, {NO_ACTION_CONTROLLER, false}, {CONSISTENT_CONTROLLER, false}};
    start(config);
    sendTrajectoryGoal(TRAJECTORY_CONTROLLER);

    robot_->publishTaskRunning(false);

    ASSERT_TRUE(spinUntil([this]() { return log_.contains("deactivate"); }, 10s)) << "controllers were never deactivated";
    EXPECT_LT(log_.indexOf(cancelEvent(TRAJECTORY_CONTROLLER)), log_.indexOf("deactivate"));
    EXPECT_TRUE(deactivated(TRAJECTORY_CONTROLLER));
    EXPECT_TRUE(deactivated(NO_ACTION_CONTROLLER));
}

TEST_F(ControllerStopperTest, DeactivatesWithoutACancelWhenNoControllerHasATrajectoryServer) {
    FakeRobotConfig config;
    config.controllers = {{NO_ACTION_CONTROLLER, false}, {CONSISTENT_CONTROLLER, false}};
    start(config);
    // Give the prime time to create the (never ready) client, so the skip is what is exercised
    spinFor(1s);

    robot_->publishTaskRunning(false);

    ASSERT_TRUE(spinUntil([this]() { return log_.contains("deactivate"); }, 10s))
        << "a controller with no trajectory server was never deactivated";
    EXPECT_TRUE(deactivated(NO_ACTION_CONTROLLER));
}

// The prime timer repeats for the life of the stopper and captures `this`; once the stopper is gone,
// spinning the node it was built on must not call back into it. rclcpp already frees the timer
// with its owning member, so this is a guard on the contract rather than a reproduction of a crash.
TEST_F(ControllerStopperTest, StopsPollingTheControllerManagerOnceDestroyed) {
    start();
    // At least one timer period, to show the timer does poll while the stopper lives
    spinFor(2500ms);
    ASSERT_GT(robot_->listRequests(), 1) << "the prime timer never fired; this test proves nothing";

    stopper_.reset();
    const int requests_at_destruction = robot_->listRequests();
    spinFor(4500ms);

    EXPECT_EQ(robot_->listRequests(), requests_at_destruction)
        << "the prime timer outlived the stopper and called back into a destroyed object";
}

}  // namespace

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    const int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
