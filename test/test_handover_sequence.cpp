
#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "crane_msgs/msg/solver_health.hpp"
#include "crane_msgs/msg/supervisor_status.hpp"
#include "crane_msgs/msg/velocity_controller_health.hpp"
#include "crane_msgs/srv/set_mode.hpp"
#include "crane_supervisor/supervisor_node.hpp"
#include "epsilon_crane_msgs/msg/remote_ctrl_states.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace
{

using crane_msgs::msg::SolverHealth;
using crane_msgs::msg::SupervisorStatus;
using crane_msgs::srv::SetMode;
using epsilon_crane_msgs::msg::RemoteCtrlStates;
using std_srvs::srv::Trigger;

using ListControllers = controller_manager_msgs::srv::ListControllers;
using SwitchController = controller_manager_msgs::srv::SwitchController;

/// The two controllers of the arm claim, named as the deployment names them.
constexpr char kInnerLoop[] = "crane_velocity_controller";
constexpr char kFollower[] = "trajectory_controller_a2b";
/// The node `crane_bringup` composes the horizon producer under.
constexpr char kProducerNode[] = "crane_mpc";

constexpr double kBudget = 20.0;
constexpr std::chrono::milliseconds kPollPeriod{2};

std::string stepped_domain()
{
  const char * const given = ::getenv("ROS_DOMAIN_ID");
  const int base = (given == nullptr) ? 0 : std::atoi(given);
  return std::to_string(1 + ((base + 25) % 101 + 101) % 101);
}

class RclcppEnvironment : public ::testing::Environment
{
public:
  void SetUp() override
  {
    ::unsetenv("CYCLONEDDS_URI");
    ::unsetenv("FASTRTPS_DEFAULT_PROFILES_FILE");
    ::setenv("ROS_LOCALHOST_ONLY", "1", 1);
    ::setenv("ROS_DOMAIN_ID", stepped_domain().c_str(), 1);
    rclcpp::init(0, nullptr);
  }
  void TearDown() override {rclcpp::shutdown();}
};

[[maybe_unused]] const ::testing::Environment * const kEnvironment =
  ::testing::AddGlobalTestEnvironment(new RclcppEnvironment);

/// A controller manager that answers, remembers and counts.
class ManagerSpy : public rclcpp::Node
{
public:
  ManagerSpy()
  : rclcpp::Node("controller_manager")
  {
    group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    list_ = create_service<ListControllers>(
      crane_supervisor::kListControllersService,
      [this](
        const ListControllers::Request::SharedPtr, ListControllers::Response::SharedPtr response) {
        const std::lock_guard<std::mutex> lock(mutex_);
        ++list_calls_;
        for (const auto & entry : state_) {
          controller_manager_msgs::msg::ControllerState controller;
          controller.name = entry.first;
          controller.state = entry.second;
          if (entry.second == "active") {
            controller.claimed_interfaces = {entry.first + "/theta1_slewing_joint/velocity"};
          }
          response->controller.push_back(controller);
        }
      },
      rmw_qos_profile_services_default, group_);
    switch_ = create_service<SwitchController>(
      crane_supervisor::kSwitchControllerService,
      [this](
        const SwitchController::Request::SharedPtr request,
        SwitchController::Response::SharedPtr response) {
        const std::lock_guard<std::mutex> lock(mutex_);
        ++switch_calls_;
        activated_ = request->activate_controllers;
        deactivated_ = request->deactivate_controllers;
        for (const std::string & name : request->deactivate_controllers) {
          state_[name] = "inactive";
        }
        for (const std::string & name : request->activate_controllers) {
          state_[name] = "active";
        }
        response->ok = true;
        response->message = "the spy performed the switch";
      },
      rmw_qos_profile_services_default, group_);
  }

  int switch_calls() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return switch_calls_;
  }
  int list_calls() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return list_calls_;
  }
  std::vector<std::string> activated() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return activated_;
  }
  std::vector<std::string> deactivated() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return deactivated_;
  }
  std::string state_of(const std::string & name) const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    const auto found = state_.find(name);
    return found == state_.end() ? std::string("unloaded") : found->second;
  }
  void set_state(const std::string & name, const std::string & state)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    state_[name] = state;
  }

private:
  mutable std::mutex mutex_;
  std::map<std::string, std::string> state_{{kInnerLoop, "inactive"}, {kFollower, "inactive"}};
  int switch_calls_{0};
  int list_calls_{0};
  std::vector<std::string> activated_;
  std::vector<std::string> deactivated_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::Service<ListControllers>::SharedPtr list_;
  rclcpp::Service<SwitchController>::SharedPtr switch_;
};

/// The horizon producer, as much of it as this sequence depends on.
class ProducerStub : public rclcpp::Node
{
public:
  explicit ProducerStub(const ManagerSpy & manager)
  : rclcpp::Node(kProducerNode), manager_(manager)
  {
    declare_parameter<std::string>(crane_supervisor::kHorizonProducerModeParameter, "shadow");
    health_ = create_publisher<SolverHealth>(
      crane_supervisor::kSolverHealthTopic, crane_supervisor::contract_qos());
    callback_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = accept_;
        result.reason = accept_ ? "" : "the stub was told to refuse";
        if (accept_) {
          const std::lock_guard<std::mutex> lock(mutex_);
          for (const rclcpp::Parameter & parameter : parameters) {
            if (parameter.get_name() == crane_supervisor::kHorizonProducerModeParameter) {
              changes_.push_back({parameter.as_string(), manager_.switch_calls()});
            }
          }
        }
        return result;
      });
  }

  /// One `SolverHealth`, stamped now, as a producer that solved would publish it.
  void publish_health(std::uint8_t outcome, bool applied_previous_solution = true)
  {
    SolverHealth message;
    message.header.stamp = now();
    message.outcome = outcome;
    message.fault = outcome == SolverHealth::SOLVE_CONVERGED
      ? SupervisorStatus::FAULT_NONE
      : SupervisorStatus::FAULT_SOLVER;
    message.solve_time = 0.012;
    message.solve_budget = 0.03;
    message.applied_previous_solution =
      outcome != SolverHealth::SOLVE_CONVERGED && applied_previous_solution;
    message.message = "the stub reports one solve";
    health_->publish(message);
  }

  std::string mode()
  {
    return get_parameter(crane_supervisor::kHorizonProducerModeParameter).as_string();
  }

  /// Every mode this producer was moved to, and the switch count at the time.
  struct Change
  {
    std::string mode;
    int switch_calls;
  };
  std::vector<Change> changes() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return changes_;
  }

  void refuse() {accept_ = false;}

private:
  const ManagerSpy & manager_;
  mutable std::mutex mutex_;
  std::vector<Change> changes_;
  bool accept_{true};
  rclcpp::Publisher<SolverHealth>::SharedPtr health_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_;
};

class HandoverSequence : public ::testing::Test
{
protected:
  /// Whether this test wants a controller manager on the graph at all.
  virtual bool compose_manager() const {return true;}

  void SetUp() override
  {
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
      rclcpp::ExecutorOptions(), 4);

    if (compose_manager()) {
      manager_ = std::make_shared<ManagerSpy>();
    } else {
      manager_ = std::make_shared<ManagerSpy>();
    }
    producer_ = std::make_shared<ProducerStub>(*manager_);
    supervisor_ = std::make_shared<crane_supervisor::SupervisorNode>(supervisor_options());

    observer_ = std::make_shared<rclcpp::Node>("crane_supervisor_handover_observer");
    status_ = observer_->create_subscription<SupervisorStatus>(
      crane_supervisor::kStatusTopic, crane_supervisor::contract_qos(),
      [this](SupervisorStatus::ConstSharedPtr message) {
        const std::lock_guard<std::mutex> lock(status_mutex_);
        latest_ = *message;
      });
    remote_ctrl_ = observer_->create_publisher<RemoteCtrlStates>(
      crane_supervisor::kRemoteCtrlStatesTopic, crane_supervisor::contract_qos());
    pendulum_state_ = observer_->create_publisher<sensor_msgs::msg::JointState>(
      crane_supervisor::kPassiveStateTopic, crane_supervisor::contract_qos());
    controller_state_ =
      observer_->create_publisher<control_msgs::msg::JointTrajectoryControllerState>(
      crane_supervisor::kControllerStateTopic, rclcpp::SystemDefaultsQoS());
    controller_health_ = observer_->create_publisher<crane_msgs::msg::VelocityControllerHealth>(
      crane_supervisor::kControllerHealthTopic, crane_supervisor::contract_qos());
    set_mode_ = observer_->create_client<SetMode>(crane_supervisor::kSetModeService);
    clear_fault_ = observer_->create_client<Trigger>(crane_supervisor::kClearFaultService);

    if (compose_manager()) {
      executor_->add_node(manager_);
    }
    executor_->add_node(producer_);
    executor_->add_node(supervisor_);
    executor_->add_node(observer_);
    spinner_ = std::thread([this]() {executor_->spin();});
  }

  void TearDown() override
  {
    executor_->cancel();
    if (spinner_.joinable()) {
      spinner_.join();
    }
    executor_->remove_node(observer_);
    executor_->remove_node(supervisor_);
    executor_->remove_node(producer_);
    if (compose_manager()) {
      executor_->remove_node(manager_);
    }
    set_mode_.reset();
    clear_fault_.reset();
    status_.reset();
    remote_ctrl_.reset();
    pendulum_state_.reset();
    controller_state_.reset();
    controller_health_.reset();
    observer_.reset();
    supervisor_.reset();
    producer_.reset();
    manager_.reset();
    executor_.reset();
  }

  /// The deployment's own wiring, nesting and all.
  static rclcpp::NodeOptions supervisor_options()
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
      {rclcpp::Parameter("pendulum_state_timeout", 0.5),
        rclcpp::Parameter("remote_ctrl_timeout", 0.5),
        rclcpp::Parameter("controller_state_timeout", 0.5),
        rclcpp::Parameter("controller_health_timeout", 0.5),
        rclcpp::Parameter("controller_manager_timeout", 2.0),
        rclcpp::Parameter("horizon_timeout", 2.0),
        rclcpp::Parameter("mpc_node", std::string(kProducerNode)),
        rclcpp::Parameter(
          "mode_controllers.follow", std::vector<std::string>{kInnerLoop, kFollower}),
        rclcpp::Parameter("mode_controllers.mpc", std::vector<std::string>{kInnerLoop})});
    return options;
  }

  /// Wait on an observable outcome, never on a sleep of a fixed length.
  bool wait_until(const std::function<bool()> & ready)
  {
    const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::duration<double>(kBudget);
    while (std::chrono::steady_clock::now() < deadline) {
      if (ready()) {
        return true;
      }
      publish_inputs();
      std::this_thread::sleep_for(kPollPeriod);
    }
    return ready();
  }

  /// All four of the supervisor's inputs, healthy and stamped now.
  void publish_inputs()
  {
    const rclcpp::Time stamp = observer_->now();

    RemoteCtrlStates message;
    message.header.stamp = stamp;
    message.button12 = true;
    message.em_stop = false;
    remote_ctrl_->publish(message);

    sensor_msgs::msg::JointState pendulum;
    pendulum.header.stamp = stamp;
    for (const auto & axis : crane_supervisor::kPassiveAxisNames) {
      pendulum.name.push_back(axis.joint);
      pendulum.position.push_back(0.0);
      pendulum.velocity.push_back(0.0);
    }
    pendulum_state_->publish(pendulum);

    control_msgs::msg::JointTrajectoryControllerState controller_state;
    controller_state.header.stamp = stamp;
    controller_state.joint_names = {kInnerLoop};
    controller_state.error.positions = {0.0};
    controller_state.error.velocities = {0.0};
    controller_state_->publish(controller_state);

    crane_msgs::msg::VelocityControllerHealth health;
    health.header.stamp = stamp;
    health.fault = inner_loop_fault_;
    controller_health_->publish(health);
  }

  /// Bring the supervisor out of the stop every freshly started one may be in.
  void clear_the_starting_latch()
  {
    ASSERT_TRUE(wait_until([this]() {return clear_fault_->service_is_ready();}));
    auto future =
      clear_fault_->async_send_request(std::make_shared<Trigger::Request>()).future.share();
    ASSERT_TRUE(wait_until([&future]() {
      return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
    }));
  }

  /// One `/crane/set_mode` call, with the remote kept alive underneath it.
  SetMode::Response::SharedPtr request(std::uint8_t mode)
  {
    if (!wait_until([this]() {return set_mode_->service_is_ready();})) {
      return nullptr;
    }
    auto message = std::make_shared<SetMode::Request>();
    message->mode = mode;
    auto future = set_mode_->async_send_request(message).future.share();
    const bool answered = wait_until([&future]() {
        return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
      });
    return answered ? future.get() : nullptr;
  }

  /// Keep the producer's health stream alive until `ready` holds.
  bool solve_until(std::uint8_t outcome, const std::function<bool()> & ready)
  {
    return wait_until([this, outcome, &ready]() {
      producer_->publish_health(outcome);
      return ready();
    });
  }

  SetMode::Response::SharedPtr request_while_solving(std::uint8_t mode, std::uint8_t outcome)
  {
    if (!solve_until(outcome, [this]() {return set_mode_->service_is_ready();})) {
      return nullptr;
    }
    auto message = std::make_shared<SetMode::Request>();
    message->mode = mode;
    auto future = set_mode_->async_send_request(message).future.share();
    const bool answered = solve_until(outcome, [&future]() {
        return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
      });
    return answered ? future.get() : nullptr;
  }

  /// The newest status this observer has seen, or nothing before the first.
  std::optional<SupervisorStatus> status()
  {
    const std::lock_guard<std::mutex> lock(status_mutex_);
    return latest_;
  }

  /// Whether the supervisor's own stream is reporting `mode` **and** `fault`.
  bool saw(std::uint8_t mode, std::uint8_t fault)
  {
    const std::lock_guard<std::mutex> lock(status_mutex_);
    return latest_.has_value() && latest_->mode == mode && latest_->fault == fault;
  }

  /// The mode the supervisor's own stream reports, re-derived from the spy.
  bool saw_mode(std::uint8_t mode)
  {
    const std::lock_guard<std::mutex> lock(status_mutex_);
    return latest_.has_value() && latest_->mode == mode;
  }

  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::shared_ptr<ManagerSpy> manager_;
  std::shared_ptr<ProducerStub> producer_;
  std::shared_ptr<crane_supervisor::SupervisorNode> supervisor_;
  rclcpp::Node::SharedPtr observer_;
  rclcpp::Subscription<SupervisorStatus>::SharedPtr status_;
  rclcpp::Publisher<RemoteCtrlStates>::SharedPtr remote_ctrl_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pendulum_state_;
  rclcpp::Publisher<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr
    controller_state_;
  rclcpp::Publisher<crane_msgs::msg::VelocityControllerHealth>::SharedPtr controller_health_;
  std::uint8_t inner_loop_fault_{SupervisorStatus::FAULT_NONE};
  rclcpp::Client<SetMode>::SharedPtr set_mode_;
  rclcpp::Client<Trigger>::SharedPtr clear_fault_;
  mutable std::mutex status_mutex_;
  std::optional<SupervisorStatus> latest_;
  std::thread spinner_;
};

/// The same fixture with the controller manager off the graph entirely.
class NoControllerManager : public HandoverSequence
{
protected:
  bool compose_manager() const override {return false;}
};

}  // namespace

TEST_F(NoControllerManager, FreshnessIsVerifiedBeforeTheManagerIsAskedForAnything)
{
  clear_the_starting_latch();

  const auto refused = request(SupervisorStatus::MODE_MPC);
  ASSERT_NE(refused, nullptr);
  EXPECT_FALSE(refused->success);
  EXPECT_NE(refused->message.find("freshness"), std::string::npos) << refused->message;
  EXPECT_EQ(refused->message.find("will not switch blind"), std::string::npos)
    << "the manager was consulted before the horizon was: " << refused->message;
  EXPECT_NE(refused->message.find("nothing was asked of"), std::string::npos)
    << refused->message;

  // And nothing was called, because there was nothing to call it on.
  EXPECT_EQ(manager_->switch_calls(), 0);

  const auto follow = request(SupervisorStatus::MODE_FOLLOW);
  ASSERT_NE(follow, nullptr);
  EXPECT_FALSE(follow->success);
  EXPECT_NE(follow->message.find("cannot see the controller manager"), std::string::npos)
    << follow->message;
}

TEST_F(HandoverSequence, AStaleOrAbsentHorizonNeverReachesTheSwitchService)
{
  clear_the_starting_latch();
  manager_->set_state(kInnerLoop, "active");
  manager_->set_state(kFollower, "active");
  ASSERT_TRUE(wait_until([this]() {return saw_mode(SupervisorStatus::MODE_FOLLOW);}));

  // Nothing has ever published `/crane/mpc/solver_health` in this test.
  const auto absent = request(SupervisorStatus::MODE_MPC);
  ASSERT_NE(absent, nullptr);
  EXPECT_FALSE(absent->success);
  EXPECT_NE(absent->message.find("freshness"), std::string::npos) << absent->message;
  EXPECT_EQ(manager_->switch_calls(), 0) << "the switch service was reached anyway";

  // The machine is still in the mode it was already in, and the response says so.
  EXPECT_EQ(absent->active_mode, SupervisorStatus::MODE_FOLLOW) << absent->message;
  EXPECT_EQ(manager_->state_of(kFollower), "active")
    << "a refused switch deactivated the trajectory controller";

  const auto failing =
    request_while_solving(SupervisorStatus::MODE_MPC, SolverHealth::SOLVE_FAILED);
  ASSERT_NE(failing, nullptr);
  EXPECT_FALSE(failing->success);
  EXPECT_NE(failing->message.find("SOLVE_FAILED"), std::string::npos) << failing->message;
  EXPECT_EQ(manager_->switch_calls(), 0) << "a failing optimizer reached the switch service";
  EXPECT_EQ(manager_->state_of(kFollower), "active");

  // And the producer was never moved either: a refusal touches nothing.
  EXPECT_EQ(producer_->mode(), "shadow");
  EXPECT_TRUE(producer_->changes().empty());
}

TEST_F(HandoverSequence, AWarmHorizonMovesTheClaimAndTheProducerInTheOrderStepTwoNeeds)
{
  // The whole sequence, in one test, because the order is what it is about.
  clear_the_starting_latch();
  manager_->set_state(kInnerLoop, "active");
  manager_->set_state(kFollower, "active");
  ASSERT_TRUE(wait_until([this]() {return saw_mode(SupervisorStatus::MODE_FOLLOW);}));

  const auto to_mpc =
    request_while_solving(SupervisorStatus::MODE_MPC, SolverHealth::SOLVE_CONVERGED);
  ASSERT_NE(to_mpc, nullptr);
  EXPECT_TRUE(to_mpc->success) << to_mpc->message;
  EXPECT_EQ(to_mpc->active_mode, SupervisorStatus::MODE_MPC) << to_mpc->message;

  ASSERT_EQ(manager_->switch_calls(), 1);
  EXPECT_EQ(manager_->deactivated(), (std::vector<std::string>{kFollower}));
  EXPECT_TRUE(manager_->activated().empty()) << "the inner loop was cycled across the handover";
  EXPECT_EQ(manager_->state_of(kInnerLoop), "active");

  ASSERT_EQ(producer_->changes().size(), 1U);
  EXPECT_EQ(producer_->changes().front().mode, "active");
  EXPECT_EQ(producer_->changes().front().switch_calls, 0)
    << "the producer was moved after the claim, so the inner loop had no horizon to fit";
  EXPECT_EQ(producer_->mode(), "active");
  EXPECT_NE(to_mpc->message.find("horizon producer was put into active"), std::string::npos)
    << to_mpc->message;

  const auto to_follow =
    request_while_solving(SupervisorStatus::MODE_FOLLOW, SolverHealth::SOLVE_CONVERGED);
  ASSERT_NE(to_follow, nullptr);
  EXPECT_TRUE(to_follow->success) << to_follow->message;
  EXPECT_EQ(to_follow->active_mode, SupervisorStatus::MODE_FOLLOW) << to_follow->message;

  ASSERT_EQ(manager_->switch_calls(), 2);
  EXPECT_EQ(manager_->activated(), (std::vector<std::string>{kFollower}));
  EXPECT_TRUE(manager_->deactivated().empty()) << "the inner loop was cycled leaving MODE_MPC";

  ASSERT_EQ(producer_->changes().size(), 2U);
  EXPECT_EQ(producer_->changes().back().mode, "shadow");
  EXPECT_EQ(producer_->changes().back().switch_calls, 2)
    << "the producer was put back into shadow before the claim moved, leaving the inner loop "
       "unchained with no horizon";
  EXPECT_EQ(producer_->mode(), "shadow");
}

TEST_F(HandoverSequence, TheSupervisorAndNothingElseDecidesWhichPathIsLive)
{
  clear_the_starting_latch();
  manager_->set_state(kInnerLoop, "active");
  manager_->set_state(kFollower, "active");
  ASSERT_TRUE(wait_until([this]() {return saw_mode(SupervisorStatus::MODE_FOLLOW);}));

  ASSERT_NE(
    request_while_solving(SupervisorStatus::MODE_MPC, SolverHealth::SOLVE_CONVERGED), nullptr);
  ASSERT_EQ(producer_->mode(), "active");

  const auto to_idle = request(SupervisorStatus::MODE_IDLE);
  ASSERT_NE(to_idle, nullptr);
  EXPECT_TRUE(to_idle->success) << to_idle->message;
  EXPECT_EQ(producer_->mode(), "shadow")
    << "the arm claim was released and the producer was left in active";
  EXPECT_EQ(producer_->changes().back().mode, "shadow");
}

TEST_F(HandoverSequence, AProducerThatRefusesTheModeIsReportedRatherThanAssumed)
{
  clear_the_starting_latch();
  manager_->set_state(kInnerLoop, "active");
  manager_->set_state(kFollower, "active");
  ASSERT_TRUE(wait_until([this]() {return saw_mode(SupervisorStatus::MODE_FOLLOW);}));
  producer_->refuse();

  const auto to_mpc =
    request_while_solving(SupervisorStatus::MODE_MPC, SolverHealth::SOLVE_CONVERGED);
  ASSERT_NE(to_mpc, nullptr);
  EXPECT_NE(to_mpc->message.find("would not be put into active"), std::string::npos)
    << to_mpc->message;
  EXPECT_NE(to_mpc->message.find("the stub was told to refuse"), std::string::npos)
    << to_mpc->message;
  EXPECT_EQ(producer_->mode(), "shadow");
  EXPECT_TRUE(producer_->changes().empty());
}

TEST_F(HandoverSequence, TheStatusStreamCarriesModeMpcAndTheProducersOwnCode)
{
  clear_the_starting_latch();
  manager_->set_state(kInnerLoop, "active");
  manager_->set_state(kFollower, "active");
  ASSERT_TRUE(wait_until([this]() {return saw_mode(SupervisorStatus::MODE_FOLLOW);}));

  const auto to_mpc =
    request_while_solving(SupervisorStatus::MODE_MPC, SolverHealth::SOLVE_CONVERGED);
  ASSERT_NE(to_mpc, nullptr);
  ASSERT_TRUE(to_mpc->success) << to_mpc->message;

  ASSERT_TRUE(
    solve_until(
      SolverHealth::SOLVE_CONVERGED, [this]() {
        return saw(SupervisorStatus::MODE_MPC, SupervisorStatus::FAULT_NONE);
      }))
    << "the stream never carried MODE_MPC with a clear report";

  ASSERT_TRUE(
    solve_until(
      SolverHealth::SOLVE_BUDGET_EXCEEDED, [this]() {
        return saw(SupervisorStatus::MODE_MPC, SupervisorStatus::FAULT_SOLVER);
      }))
    << "FAULT_SOLVER never reached /crane/supervisor/status";
  const auto reported = status();
  ASSERT_TRUE(reported.has_value());
  EXPECT_NE(reported->message.find("shifted by one step"), std::string::npos)
    << reported->message;
  EXPECT_NE(reported->message.find("the stub reports one solve"), std::string::npos)
    << "the producer's own account was restated rather than carried: " << reported->message;
  EXPECT_EQ(manager_->switch_calls(), 1) << "a shifted fallback moved the claim";
  EXPECT_EQ(producer_->mode(), "active");
}

TEST_F(HandoverSequence, TheRepeatedFailureEscalationTakesTheModeOutOfMpc)
{
  clear_the_starting_latch();
  manager_->set_state(kInnerLoop, "active");
  manager_->set_state(kFollower, "active");
  ASSERT_TRUE(wait_until([this]() {return saw_mode(SupervisorStatus::MODE_FOLLOW);}));

  ASSERT_NE(
    request_while_solving(SupervisorStatus::MODE_MPC, SolverHealth::SOLVE_CONVERGED), nullptr);
  ASSERT_TRUE(wait_until([this]() {return saw_mode(SupervisorStatus::MODE_MPC);}));
  ASSERT_EQ(producer_->mode(), "active");
  const int switches_before = manager_->switch_calls();

  const auto escalate = [this]() {
      producer_->publish_health(SolverHealth::SOLVE_FAILED, false);
    };
  ASSERT_TRUE(
    wait_until([this, &escalate]() {
      escalate();
      return saw(SupervisorStatus::MODE_MPC, SupervisorStatus::FAULT_SOLVER);
    }))
    << "the escalation never reached the status stream";
  EXPECT_EQ(manager_->switch_calls(), switches_before)
    << "the claim was released while the receiver still had plan to run";
  EXPECT_EQ(producer_->mode(), "active");

  inner_loop_fault_ = SupervisorStatus::FAULT_REFERENCE_STALE;
  ASSERT_TRUE(
    wait_until([this, &escalate]() {
      escalate();
      return saw_mode(SupervisorStatus::MODE_IDLE);
    }))
    << "the mode never left MODE_MPC after the escalation";

  EXPECT_EQ(manager_->switch_calls(), switches_before + 1) << "the release was retried in a loop";
  EXPECT_EQ(manager_->deactivated().back(), kInnerLoop);
  EXPECT_EQ(manager_->state_of(kInnerLoop), "inactive");
  EXPECT_EQ(manager_->state_of(kFollower), "inactive")
    << "the supervisor chose what happens next instead of stopping";

  ASSERT_TRUE(wait_until([this]() {return producer_->mode() == "shadow";}));
  EXPECT_EQ(producer_->changes().back().mode, "shadow");
  EXPECT_EQ(producer_->changes().back().switch_calls, switches_before + 1)
    << "the producer was put back into shadow before the claim moved";

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
  while (std::chrono::steady_clock::now() < deadline) {
    escalate();
    publish_inputs();
    std::this_thread::sleep_for(kPollPeriod);
  }
  EXPECT_EQ(manager_->switch_calls(), switches_before + 1)
    << "the hand-back is being reissued on every status cycle";
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
