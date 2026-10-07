// ControllerStopper cancels trajectory goals before it deactivates, and the deactivate always follows.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <set>
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

constexpr double CANCEL_TIMEOUT_S = 0.5;

std::string cancelEvent(const std::string& controller) { return "cancel:" + controller; }

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

    std::size_t indexOf(const std::string& event) const {
        const auto seen = events();
        return static_cast<std::size_t>(std::find(seen.begin(), seen.end(), event) - seen.begin());
    }

   private:
    mutable std::mutex mutex_;
    std::vector<std::string> events_;
};

enum class CancelMode {
    ACCEPT,
    REJECT,
    NEVER_ANSWER,
};

struct FakeController {
    std::string name;
    bool has_trajectory_action;
};

struct FakeRobotConfig {
    std::vector<FakeController> controllers{{TRAJECTORY_CONTROLLER, true}, {CONSISTENT_CONTROLLER, false}};
    CancelMode cancel_mode = CancelMode::ACCEPT;
    bool controllers_loaded = true;
};

// The controller manager follows controller_manager 2.53.1. STRICT refuses a whole request that lists
// a controller already in the requested state. A controller that holds a goal refuses to deactivate.
class FakeRobot {
   public:
    FakeRobot(EventLog& log, FakeRobotConfig config)
        : node_(std::make_shared<rclcpp::Node>("fake_robot")),
          action_node_(std::make_shared<rclcpp::Node>("fake_robot_actions")),
          log_(log),
          config_(std::move(config)),
          controllers_loaded_(config_.controllers_loaded) {
        for (const auto& controller : config_.controllers) {
            active_.insert(controller.name);
        }
        list_srv_ = node_->create_service<controller_manager_msgs::srv::ListControllers>(
            "controller_manager/list_controllers",
            [this](const controller_manager_msgs::srv::ListControllers::Request::SharedPtr,
                   controller_manager_msgs::srv::ListControllers::Response::SharedPtr response) {
                list_requests_.fetch_add(1);
                std::unique_lock<std::mutex> lock(mutex_);
                listings_cv_.wait(lock, [this]() { return !listings_held_; });
                if (!controllers_loaded_.load()) {
                    return;
                }
                for (const auto& controller : config_.controllers) {
                    controller_manager_msgs::msg::ControllerState state;
                    state.name = controller.name;
                    state.state = active_.count(controller.name) > 0 ? "active" : "inactive";
                    response->controller.push_back(state);
                }
            });

        switch_srv_ = node_->create_service<controller_manager_msgs::srv::SwitchController>(
            "controller_manager/switch_controller",
            [this](const controller_manager_msgs::srv::SwitchController::Request::SharedPtr request,
                   controller_manager_msgs::srv::SwitchController::Response::SharedPtr response) {
                if (!request->deactivate_controllers.empty()) {
                    log_.record("deactivate");
                }
                if (!request->activate_controllers.empty()) {
                    log_.record("activate");
                }
                std::lock_guard<std::mutex> lock(mutex_);
                response->ok = switchControllers(*request);
            });

        mode_srv_ = node_->create_service<eli_common_interface::srv::GetRobotMode>(
            "dashboard_client/robot_mode", [](const eli_common_interface::srv::GetRobotMode::Request::SharedPtr,
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

    bool isActive(const std::string& controller) {
        std::lock_guard<std::mutex> lock(mutex_);
        return active_.count(controller) > 0;
    }

    int listRequests() const { return list_requests_.load(); }

    void releaseCancels() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            released_ = true;
        }
        released_cv_.notify_all();
    }

    void holdListings() {
        std::lock_guard<std::mutex> lock(mutex_);
        listings_held_ = true;
    }

    void releaseListings() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            listings_held_ = false;
        }
        listings_cv_.notify_all();
    }

   private:
    bool switchControllers(const controller_manager_msgs::srv::SwitchController::Request& request) {
        const bool strict = request.strictness == controller_manager_msgs::srv::SwitchController::Request::STRICT;
        std::vector<std::string> to_deactivate;
        std::vector<std::string> to_activate;
        for (const auto& name : request.deactivate_controllers) {
            if (active_.count(name) > 0) {
                to_deactivate.push_back(name);
            } else if (strict) {
                return false;
            }
        }
        for (const auto& name : request.activate_controllers) {
            if (active_.count(name) == 0) {
                to_activate.push_back(name);
            } else if (strict) {
                return false;
            }
        }
        bool ok = true;
        for (const auto& name : to_deactivate) {
            if (goal_handles_.count(name) > 0) {
                ok = false;
            } else {
                active_.erase(name);
            }
        }
        for (const auto& name : to_activate) {
            active_.insert(name);
        }
        return ok;
    }

    rclcpp_action::CancelResponse answerCancel(const std::string& controller) {
        log_.record(cancelEvent(controller));
        switch (config_.cancel_mode) {
            case CancelMode::ACCEPT: {
                std::lock_guard<std::mutex> lock(mutex_);
                auto it = goal_handles_.find(controller);
                if (it != goal_handles_.end()) {
                    canceled_goal_handles_.push_back(it->second);
                    goal_handles_.erase(it);
                }
                return rclcpp_action::CancelResponse::ACCEPT;
            }
            case CancelMode::REJECT:
                return rclcpp_action::CancelResponse::REJECT;
            case CancelMode::NEVER_ANSWER: {
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
    std::condition_variable listings_cv_;
    bool listings_held_ = false;
    std::map<std::string, std::shared_ptr<GoalHandle>> goal_handles_;
    std::vector<std::shared_ptr<GoalHandle>> canceled_goal_handles_;
    std::set<std::string> active_;
    std::atomic<bool> controllers_loaded_;
    std::atomic<int> list_requests_{0};
};

class ControllerStopperTest : public ::testing::Test {
   protected:
    void start(FakeRobotConfig config = FakeRobotConfig(),
               rclcpp::ParameterValue cancel_timeout = rclcpp::ParameterValue(CANCEL_TIMEOUT_S)) {
        robot_ = std::make_unique<FakeRobot>(log_, std::move(config));
        client_node_ = std::make_shared<rclcpp::Node>("trajectory_goal_sender");
        robot_executor_.add_node(robot_->node());
        robot_executor_.add_node(client_node_);
        action_executor_.add_node(robot_->actionNode());
        robot_thread_ = std::thread([this]() { robot_executor_.spin(); });
        action_thread_ = std::thread([this]() { action_executor_.spin(); });

        rclcpp::NodeOptions options;
        options.parameter_overrides({rclcpp::Parameter("consistent_controllers", std::vector<std::string>{CONSISTENT_CONTROLLER}),
                                     rclcpp::Parameter("trajectory_cancel_timeout", cancel_timeout)});
        stopper_node_ = std::make_shared<rclcpp::Node>("controller_stopper_under_test", options);

        stopper_ = std::make_unique<ControllerStopper>(stopper_node_, false);
    }

    void TearDown() override {
        if (robot_) {
            robot_->releaseCancels();
            robot_->releaseListings();
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

    void sendTrajectoryGoal(const std::string& controller) {
        auto client = rclcpp_action::create_client<FollowJointTrajectory>(client_node_, controller + "/follow_joint_trajectory");
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
    EXPECT_FALSE(robot_->isActive(TRAJECTORY_CONTROLLER));
    EXPECT_TRUE(robot_->isActive(CONSISTENT_CONTROLLER));
}

TEST_F(ControllerStopperTest, IgnoresARepeatedTrueWhileTheRobotTaskRuns) {
    start();
    sendTrajectoryGoal(TRAJECTORY_CONTROLLER);

    robot_->publishTaskRunning(true);
    spinFor(1s);

    EXPECT_FALSE(log_.contains(cancelEvent(TRAJECTORY_CONTROLLER))) << "a repeated true is not a reason to cancel";
    EXPECT_FALSE(log_.contains("deactivate"));
    EXPECT_FALSE(log_.contains("activate"));
}

TEST_F(ControllerStopperTest, DoesNotDeactivateWhenTheRobotTaskComesBackDuringTheCancel) {
    FakeRobotConfig config;
    config.cancel_mode = CancelMode::NEVER_ANSWER;
    start(config);
    sendTrajectoryGoal(TRAJECTORY_CONTROLLER);

    robot_->publishTaskRunning(false);
    ASSERT_TRUE(spinUntil([this]() { return log_.contains(cancelEvent(TRAJECTORY_CONTROLLER)); }, 10s))
        << "the cancel was never sent, so the window under test was never open";
    ASSERT_FALSE(log_.contains("deactivate")) << "the deactivate did not wait for the cancel; nothing to race";

    robot_->publishTaskRunning(true);
    spinFor(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(CANCEL_TIMEOUT_S * 3)));

    EXPECT_FALSE(log_.contains("deactivate")) << "the deferred deactivate landed after the restart, leaving the controllers dead";
    EXPECT_FALSE(log_.contains("activate")) << "nothing was deactivated, so there was nothing to activate";

    robot_->releaseCancels();
    robot_->publishTaskRunning(false);
    EXPECT_TRUE(spinUntil([this]() { return log_.contains("deactivate"); }, 10s))
        << "the stop after a superseded stop never deactivated";
}

TEST_F(ControllerStopperTest, DoesNotDeactivateWhenTheRobotTaskComesBackWhileTheControllersAreListed) {
    start();
    sendTrajectoryGoal(TRAJECTORY_CONTROLLER);
    robot_->holdListings();
    const int listed_before = robot_->listRequests();

    robot_->publishTaskRunning(false);
    spinFor(300ms);
    ASSERT_GT(robot_->listRequests(), listed_before) << "the stop never asked for the controllers";
    robot_->publishTaskRunning(true);
    spinFor(300ms);
    robot_->releaseListings();
    spinFor(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(CANCEL_TIMEOUT_S * 3)));

    EXPECT_FALSE(log_.contains(cancelEvent(TRAJECTORY_CONTROLLER))) << "the superseded stop still canceled the goal";
    EXPECT_FALSE(log_.contains("deactivate")) << "the superseded stop still deactivated the controllers";
    EXPECT_FALSE(log_.contains("activate"));
}

TEST_F(ControllerStopperTest, RestartsTheStoppedControllersWhenTheRobotTaskComesBack) {
    start();
    sendTrajectoryGoal(TRAJECTORY_CONTROLLER);
    robot_->publishTaskRunning(false);
    ASSERT_TRUE(spinUntil([this]() { return !robot_->isActive(TRAJECTORY_CONTROLLER); }, 10s))
        << "controllers were never deactivated";

    robot_->publishTaskRunning(true);

    EXPECT_TRUE(spinUntil([this]() { return robot_->isActive(TRAJECTORY_CONTROLLER); }, 10s))
        << "the stopped controller was never restarted";
}

TEST_F(ControllerStopperTest, RestartsTheControllersThatStoppedWhenOneKeptItsGoal) {
    FakeRobotConfig config;
    config.controllers = {{TRAJECTORY_CONTROLLER, true}, {NO_ACTION_CONTROLLER, false}, {CONSISTENT_CONTROLLER, false}};
    config.cancel_mode = CancelMode::REJECT;
    start(config);
    sendTrajectoryGoal(TRAJECTORY_CONTROLLER);
    robot_->publishTaskRunning(false);
    ASSERT_TRUE(spinUntil([this]() { return !robot_->isActive(NO_ACTION_CONTROLLER); }, 10s))
        << "controllers were never deactivated";
    ASSERT_TRUE(robot_->isActive(TRAJECTORY_CONTROLLER)) << "the controller gave up a goal it was meant to keep";

    robot_->publishTaskRunning(true);

    EXPECT_TRUE(spinUntil([this]() { return robot_->isActive(NO_ACTION_CONTROLLER); }, 10s))
        << "the controller that did stop was never restarted";
}

TEST_F(ControllerStopperTest, DeactivatesAControllerThePrimeHasNotSeenWithoutACancel) {
    FakeRobotConfig config;
    config.controllers_loaded = false;
    start(config);
    robot_->loadControllers();

    robot_->publishTaskRunning(false);

    ASSERT_TRUE(spinUntil([this]() { return !robot_->isActive(TRAJECTORY_CONTROLLER); }, 10s))
        << "a controller with no cancel client was never deactivated";
    EXPECT_FALSE(log_.contains(cancelEvent(TRAJECTORY_CONTROLLER)));
}

TEST_F(ControllerStopperTest, AcceptsAnIntegerCancelTimeout) {
    ASSERT_NO_THROW(start(FakeRobotConfig(), rclcpp::ParameterValue(static_cast<int64_t>(1))));
    sendTrajectoryGoal(TRAJECTORY_CONTROLLER);

    robot_->publishTaskRunning(false);

    EXPECT_TRUE(spinUntil([this]() { return log_.contains("deactivate"); }, 10s))
        << "the stopper did not work with an integer timeout";
}

TEST_F(ControllerStopperTest, CancelsAControllerThatLoadedAfterStartup) {
    FakeRobotConfig config;
    config.controllers_loaded = false;
    start(config);

    robot_->loadControllers();
    spinFor(6s);
    sendTrajectoryGoal(TRAJECTORY_CONTROLLER);

    robot_->publishTaskRunning(false);

    ASSERT_TRUE(spinUntil([this]() { return log_.contains(cancelEvent(TRAJECTORY_CONTROLLER)); }, 10s))
        << "a controller loaded after this node started was never given a cancel client";
}

TEST_F(ControllerStopperTest, StillDeactivatesWhenTheCancelIsRefused) {
    FakeRobotConfig config;
    config.cancel_mode = CancelMode::REJECT;
    start(config);
    sendTrajectoryGoal(TRAJECTORY_CONTROLLER);

    robot_->publishTaskRunning(false);

    ASSERT_TRUE(spinUntil([this]() { return log_.contains("deactivate"); }, 10s)) << "a refused cancel stranded the deactivate";
    EXPECT_LT(log_.indexOf(cancelEvent(TRAJECTORY_CONTROLLER)), log_.indexOf("deactivate"));
}

TEST_F(ControllerStopperTest, DeactivatesAfterTheTimeoutWhenTheCancelIsNeverAnswered) {
    FakeRobotConfig config;
    config.controllers = {{TRAJECTORY_CONTROLLER, true}, {NO_ACTION_CONTROLLER, false}, {CONSISTENT_CONTROLLER, false}};
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
    EXPECT_FALSE(robot_->isActive(NO_ACTION_CONTROLLER)) << "the controller holding a goal kept the others running";
    EXPECT_EQ(log_.count("deactivate"), 1u);
}

TEST_F(ControllerStopperTest, CancelsEveryTrajectoryControllerBeforeASingleDeactivate) {
    FakeRobotConfig config;
    config.controllers = {{TRAJECTORY_CONTROLLER, true}, {SECOND_TRAJECTORY_CONTROLLER, true}, {CONSISTENT_CONTROLLER, false}};
    start(config);
    sendTrajectoryGoal(TRAJECTORY_CONTROLLER);
    sendTrajectoryGoal(SECOND_TRAJECTORY_CONTROLLER);

    robot_->publishTaskRunning(false);

    ASSERT_TRUE(spinUntil([this]() { return log_.contains("deactivate"); }, 10s)) << "controllers were never deactivated";
    spinFor(1s);
    EXPECT_LT(log_.indexOf(cancelEvent(TRAJECTORY_CONTROLLER)), log_.indexOf("deactivate"));
    EXPECT_LT(log_.indexOf(cancelEvent(SECOND_TRAJECTORY_CONTROLLER)), log_.indexOf("deactivate"));
    EXPECT_EQ(log_.count("deactivate"), 1u);
    EXPECT_FALSE(robot_->isActive(TRAJECTORY_CONTROLLER));
    EXPECT_FALSE(robot_->isActive(SECOND_TRAJECTORY_CONTROLLER));
}

TEST_F(ControllerStopperTest, DeactivatesAControllerWithNoTrajectoryServerAlongsideOneThatHasOne) {
    FakeRobotConfig config;
    config.controllers = {{TRAJECTORY_CONTROLLER, true}, {NO_ACTION_CONTROLLER, false}, {CONSISTENT_CONTROLLER, false}};
    start(config);
    sendTrajectoryGoal(TRAJECTORY_CONTROLLER);

    robot_->publishTaskRunning(false);

    ASSERT_TRUE(spinUntil([this]() { return log_.contains("deactivate"); }, 10s)) << "controllers were never deactivated";
    EXPECT_LT(log_.indexOf(cancelEvent(TRAJECTORY_CONTROLLER)), log_.indexOf("deactivate"));
    EXPECT_FALSE(robot_->isActive(TRAJECTORY_CONTROLLER));
    EXPECT_FALSE(robot_->isActive(NO_ACTION_CONTROLLER));
}

TEST_F(ControllerStopperTest, DeactivatesWithoutACancelWhenNoControllerHasATrajectoryServer) {
    FakeRobotConfig config;
    config.controllers = {{NO_ACTION_CONTROLLER, false}, {CONSISTENT_CONTROLLER, false}};
    start(config);
    spinFor(1s);

    robot_->publishTaskRunning(false);

    ASSERT_TRUE(spinUntil([this]() { return log_.contains("deactivate"); }, 10s))
        << "a controller with no trajectory server was never deactivated";
    EXPECT_FALSE(robot_->isActive(NO_ACTION_CONTROLLER));
}

TEST_F(ControllerStopperTest, StopsPollingTheControllerManagerOnceDestroyed) {
    start();
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
