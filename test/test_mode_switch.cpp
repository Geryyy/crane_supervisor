
#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <future>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "controller_manager/controller_manager.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "crane_msgs/msg/supervisor_status.hpp"
#include "crane_msgs/srv/set_mode.hpp"
#include "crane_supervisor/supervisor_node.hpp"
#include "epsilon_crane_msgs/msg/remote_ctrl_states.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace
{

using crane_msgs::msg::SupervisorStatus;
using crane_msgs::srv::SetMode;
using epsilon_crane_msgs::msg::RemoteCtrlStates;
using std_srvs::srv::Trigger;

constexpr char kFollowController[] = "follow_velocity_controller";
constexpr char kManualController[] = "manual_velocity_controller";
constexpr char kToolController[] = "tool_velocity_controller";

const std::vector<std::string> & arm_joints()
{
  static const std::vector<std::string> joints{
    "theta1_slewing_joint", "theta2_boom_joint", "theta3_arm_joint", "q4_big_telescope",
    "theta8_rotator_joint"};
  return joints;
}

constexpr char kToolJoint[] = "q9_left_rail_joint";

/// The manager's rate, and the period the harness drives it at.
constexpr double kRate = 100.0;

constexpr double kBudget = 20.0;

/// How long the harness pauses between control cycles.
constexpr std::chrono::milliseconds kCyclePeriod{1};

std::string stepped_domain()
{
  const char * const given = ::getenv("ROS_DOMAIN_ID");
  const int base = (given == nullptr) ? 0 : std::atoi(given);
  return std::to_string(1 + ((base + 50) % 101 + 101) % 101);
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

/// Six actuated joints on mock hardware, each with a `velocity` command.
std::string make_urdf()
{
  std::ostringstream urdf;
  urdf << "<?xml version=\"1.0\"?>\n<robot name=\"crane_mode_switch_harness\">\n"
       << "  <link name=\"base_link\"/>\n";
  std::vector<std::string> joints = arm_joints();
  joints.push_back(kToolJoint);
  for (std::size_t i = 0; i < joints.size(); ++i) {
    urdf << "  <link name=\"l" << i + 1 << "\"/>\n";
  }
  for (std::size_t i = 0; i < joints.size(); ++i) {
    const std::string parent = (i == 0) ? std::string("base_link") : "l" + std::to_string(i);
    urdf << "  <joint name=\"" << joints[i] << "\" type=\"revolute\">\n"
         << "    <parent link=\"" << parent << "\"/><child link=\"l" << i + 1 << "\"/>\n"
         << "    <origin xyz=\"1 0 0\"/><axis xyz=\"0 1 0\"/>\n"
         << "    <limit lower=\"-3.0\" upper=\"3.0\" effort=\"1000\" velocity=\"2.0\"/>\n"
         << "  </joint>\n";
  }
  urdf << "  <ros2_control name=\"CraneModeSwitchHardware\" type=\"system\">\n"
       << "    <hardware>\n"
       << "      <plugin>mock_components/GenericSystem</plugin>\n"
       << "      <param name=\"calculate_dynamics\">false</param>\n"
       << "    </hardware>\n";
  for (const std::string & joint : joints) {
    urdf << "    <joint name=\"" << joint << "\">\n"
         << "      <command_interface name=\"velocity\"/>\n"
         << "      <state_interface name=\"position\">"
         << "<param name=\"initial_value\">0.0</param></state_interface>\n"
         << "      <state_interface name=\"velocity\">"
         << "<param name=\"initial_value\">0.0</param></state_interface>\n"
         << "    </joint>\n";
  }
  urdf << "  </ros2_control>\n</robot>\n";
  return urdf.str();
}

class ModeSwitch : public ::testing::Test
{
protected:
  void SetUp() override
  {
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
      rclcpp::ExecutorOptions(), 4);

    auto options = controller_manager::get_cm_node_options();
    auto arguments = options.arguments();
    arguments.push_back("--ros-args");
    arguments.push_back("--params-file");
    arguments.push_back(
      std::string(CRANE_SUPERVISOR_TEST_CONFIG_DIR) + "/mode_switch_controllers.yaml");
    options.arguments(arguments);

    cm_ = std::make_shared<controller_manager::ControllerManager>(
      executor_, make_urdf(), true, "controller_manager", "", options);

    for (const char * name : {kFollowController, kManualController, kToolController}) {
      ASSERT_TRUE(cm_->load_controller(name)) << name;
      ASSERT_EQ(cm_->configure_controller(name), controller_interface::return_type::OK) << name;
    }
    ASSERT_EQ(cm_->get_update_rate(), static_cast<unsigned int>(kRate));

    supervisor_ = std::make_shared<crane_supervisor::SupervisorNode>(supervisor_options());

    observer_ = std::make_shared<rclcpp::Node>("crane_supervisor_mode_observer");
    const rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    status_ = observer_->create_subscription<SupervisorStatus>(
      crane_supervisor::kStatusTopic, qos,
      [this](SupervisorStatus::ConstSharedPtr message) {latest_ = *message;});
    remote_ctrl_ = observer_->create_publisher<RemoteCtrlStates>(
      crane_supervisor::kRemoteCtrlStatesTopic, qos);
    set_mode_ = observer_->create_client<SetMode>(crane_supervisor::kSetModeService);
    clear_fault_ = observer_->create_client<Trigger>(crane_supervisor::kClearFaultService);

    executor_->add_node(cm_);
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
    executor_->remove_node(cm_);
    set_mode_.reset();
    clear_fault_.reset();
    status_.reset();
    remote_ctrl_.reset();
    observer_.reset();
    supervisor_.reset();
    cm_.reset();
    executor_.reset();
  }

  static rclcpp::NodeOptions supervisor_options()
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
      {rclcpp::Parameter("pendulum_state_timeout", 0.2),
        rclcpp::Parameter("remote_ctrl_timeout", 0.5),
        rclcpp::Parameter("controller_state_timeout", 0.2),
        rclcpp::Parameter("controller_health_timeout", 0.2),
        rclcpp::Parameter("controller_manager_timeout", 2.0),
        rclcpp::Parameter(
          "mode_controllers.follow", std::vector<std::string>{kFollowController}),
        rclcpp::Parameter(
          "mode_controllers.manual", std::vector<std::string>{kManualController}),
        rclcpp::Parameter("tool_controllers", std::vector<std::string>{kToolController}),
        rclcpp::Parameter("mpc_node", std::string{})});
    return options;
  }

  /// One control cycle, on the harness's own thread.
  void step()
  {
    cm_->read(now_, period_);
    cm_->update(now_, period_);
    cm_->write(now_, period_);
    now_ = now_ + period_;
  }

  /// Turn the control loop until `ready` holds. Returns false on the budget.
  bool drive_until(const std::function<bool()> & ready)
  {
    const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::duration<double>(kBudget);
    while (std::chrono::steady_clock::now() < deadline) {
      if (ready()) {
        return true;
      }
      step();
      std::this_thread::sleep_for(kCyclePeriod);
    }
    return ready();
  }

  /// One `/crane/set_mode` call, with the control loop turning underneath it.
  SetMode::Response::SharedPtr request(std::uint8_t mode)
  {
    if (!drive_until([this]() {return set_mode_->service_is_ready();})) {
      return nullptr;
    }
    auto message = std::make_shared<SetMode::Request>();
    message->mode = mode;
    auto future = set_mode_->async_send_request(message).future.share();
    const bool answered = drive_until([&future]() {
        return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
      });
    return answered ? future.get() : nullptr;
  }

  /// The manager's own view, read directly rather than through the supervisor.
  bool is_active(const std::string & name) const
  {
    for (const auto & spec : cm_->get_loaded_controllers()) {
      if (spec.info.name == name) {
        return spec.c->get_lifecycle_state().id() ==
               lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
      }
    }
    return false;
  }

  /// A remote with the stop released and the deadman held, stamped now.
  void publish_remote()
  {
    RemoteCtrlStates message;
    message.header.stamp = observer_->now();
    message.button12 = true;
    message.em_stop = false;
    remote_ctrl_->publish(message);
  }

  bool drive_with_remote_until(const std::function<bool()> & ready)
  {
    return drive_until([this, &ready]() {
        publish_remote();
        return ready();
      });
  }

  controller_interface::return_type switch_outside_the_supervisor(
    const std::vector<std::string> & activate, const std::vector<std::string> & deactivate)
  {
    auto switched = std::async(std::launch::async, [this, activate, deactivate]() {
        return cm_->switch_controller(
          activate, deactivate, controller_manager_msgs::srv::SwitchController::Request::STRICT,
          true, rclcpp::Duration::from_seconds(kBudget));
      });
    drive_until([&switched]() {
      return switched.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
    });
    return switched.get();
  }

  void clear_the_starting_latch()
  {
    ASSERT_TRUE(drive_with_remote_until([this]() {return clear_fault_->service_is_ready();}));
    ASSERT_TRUE(drive_with_remote_until([this]() {return latest_.has_value();}));
    auto future = clear_fault_->async_send_request(std::make_shared<Trigger::Request>())
      .future.share();
    ASSERT_TRUE(
      drive_with_remote_until([&future]() {
        return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
      }));
    ASSERT_TRUE(
      drive_with_remote_until([this]() {
        return latest_->fault != SupervisorStatus::FAULT_ESTOP;
      }))
      << latest_->message;
  }

  /// The newest status report, or none yet.
  bool saw_mode(std::uint8_t mode)
  {
    return latest_.has_value() && latest_->mode == mode;
  }

  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::shared_ptr<controller_manager::ControllerManager> cm_;
  std::shared_ptr<crane_supervisor::SupervisorNode> supervisor_;
  rclcpp::Node::SharedPtr observer_;
  rclcpp::Subscription<SupervisorStatus>::SharedPtr status_;
  rclcpp::Publisher<RemoteCtrlStates>::SharedPtr remote_ctrl_;
  rclcpp::Client<SetMode>::SharedPtr set_mode_;
  rclcpp::Client<Trigger>::SharedPtr clear_fault_;
  std::optional<SupervisorStatus> latest_;
  std::thread spinner_;
  const rclcpp::Duration period_ = rclcpp::Duration::from_seconds(1.0 / kRate);
  rclcpp::Time now_{0, 0, RCL_ROS_TIME};
};

}  // namespace

TEST_F(ModeSwitch, TheServiceIsTheOnlyWayAModeChangesAndTheSwitchIsReal)
{
  clear_the_starting_latch();

  ASSERT_TRUE(drive_with_remote_until([this]() {return saw_mode(SupervisorStatus::MODE_IDLE);}));
  EXPECT_FALSE(is_active(kFollowController));

  const auto to_follow = request(SupervisorStatus::MODE_FOLLOW);
  ASSERT_NE(to_follow, nullptr);
  EXPECT_TRUE(to_follow->success) << to_follow->message;
  EXPECT_FALSE(to_follow->message.empty());
  EXPECT_EQ(to_follow->active_mode, SupervisorStatus::MODE_FOLLOW) << to_follow->message;
  EXPECT_TRUE(is_active(kFollowController));
  EXPECT_FALSE(is_active(kManualController));

  // And it reaches the stream, re-derived rather than remembered.
  ASSERT_TRUE(drive_with_remote_until([this]() {return saw_mode(SupervisorStatus::MODE_FOLLOW);}))
    << latest_->message;
  EXPECT_NE(latest_->message.find(kFollowController), std::string::npos) << latest_->message;
}

TEST_F(ModeSwitch, ManualAndAutonomousExcludeEachOtherAndTheSupervisorArbitratesIt)
{
  clear_the_starting_latch();
  ASSERT_NE(request(SupervisorStatus::MODE_FOLLOW), nullptr);
  ASSERT_TRUE(is_active(kFollowController));

  const auto to_manual = request(SupervisorStatus::MODE_MANUAL);
  ASSERT_NE(to_manual, nullptr);
  EXPECT_TRUE(to_manual->success) << to_manual->message;
  EXPECT_EQ(to_manual->active_mode, SupervisorStatus::MODE_MANUAL) << to_manual->message;
  EXPECT_TRUE(is_active(kManualController));
  EXPECT_FALSE(is_active(kFollowController)) << "both claimants are active at once";

  const auto to_idle = request(SupervisorStatus::MODE_IDLE);
  ASSERT_NE(to_idle, nullptr);
  EXPECT_TRUE(to_idle->success) << to_idle->message;
  EXPECT_EQ(to_idle->active_mode, SupervisorStatus::MODE_IDLE) << to_idle->message;
  EXPECT_FALSE(is_active(kManualController));
  EXPECT_NE(to_idle->message.find("not a stop"), std::string::npos) << to_idle->message;

  // Asking again for the mode that is active is a no-op, reported as one.
  const auto again = request(SupervisorStatus::MODE_IDLE);
  ASSERT_NE(again, nullptr);
  EXPECT_TRUE(again->success) << again->message;
  EXPECT_NE(again->message.find("no switch was issued"), std::string::npos) << again->message;
}

TEST_F(ModeSwitch, AnArmModeChangeLeavesTheToolClaimExactlyWhereItWas)
{
  clear_the_starting_latch();

  ASSERT_EQ(
    switch_outside_the_supervisor({kToolController}, {}), controller_interface::return_type::OK);
  ASSERT_TRUE(is_active(kToolController));

  ASSERT_NE(request(SupervisorStatus::MODE_FOLLOW), nullptr);
  EXPECT_TRUE(is_active(kToolController)) << "activating an arm mode disturbed the tool claim";

  const auto to_manual = request(SupervisorStatus::MODE_MANUAL);
  ASSERT_NE(to_manual, nullptr);
  EXPECT_TRUE(to_manual->success) << to_manual->message;
  EXPECT_TRUE(is_active(kToolController)) << "an arm mode change deactivated the tool claim";

  const auto to_idle = request(SupervisorStatus::MODE_IDLE);
  ASSERT_NE(to_idle, nullptr);
  EXPECT_TRUE(is_active(kToolController)) << "releasing the arm claim released the tool claim";

  EXPECT_NE(to_idle->message.find("Tool claim"), std::string::npos) << to_idle->message;
  EXPECT_NE(to_idle->message.find(kToolController), std::string::npos) << to_idle->message;
}

TEST_F(ModeSwitch, MpcIsRefusedBeforeTheTrajectoryControllerIsDeactivated)
{
  clear_the_starting_latch();
  ASSERT_NE(request(SupervisorStatus::MODE_FOLLOW), nullptr);
  ASSERT_TRUE(is_active(kFollowController));

  const auto refused = request(SupervisorStatus::MODE_MPC);
  ASSERT_NE(refused, nullptr);
  EXPECT_FALSE(refused->success);
  EXPECT_NE(refused->message.find("freshness"), std::string::npos) << refused->message;
  EXPECT_NE(refused->message.find("Nothing was deactivated"), std::string::npos)
    << refused->message;
  EXPECT_EQ(refused->active_mode, SupervisorStatus::MODE_FOLLOW) << refused->message;
  EXPECT_TRUE(is_active(kFollowController)) << "a refused switch deactivated the active mode";
}

TEST_F(ModeSwitch, ALatchedFaultRefusesAMotionModeAndIdleStaysReachable)
{
  ASSERT_TRUE(drive_until([this]() {return set_mode_->service_is_ready();}));

  const auto refused = request(SupervisorStatus::MODE_FOLLOW);
  ASSERT_NE(refused, nullptr);
  EXPECT_FALSE(refused->success);
  EXPECT_NE(refused->message.find("latched"), std::string::npos) << refused->message;
  EXPECT_NE(refused->message.find("/crane/clear_fault"), std::string::npos) << refused->message;
  EXPECT_FALSE(is_active(kFollowController));

  const auto to_idle = request(SupervisorStatus::MODE_IDLE);
  ASSERT_NE(to_idle, nullptr);
  EXPECT_TRUE(to_idle->success) << to_idle->message;

  clear_the_starting_latch();
  const auto accepted = request(SupervisorStatus::MODE_FOLLOW);
  ASSERT_NE(accepted, nullptr);
  EXPECT_TRUE(accepted->success) << accepted->message;
  EXPECT_TRUE(is_active(kFollowController));
}

TEST_F(ModeSwitch, TheModeFollowsTheWorldWhenTheWorldChangesBehindTheSupervisor)
{
  clear_the_starting_latch();
  ASSERT_NE(request(SupervisorStatus::MODE_FOLLOW), nullptr);
  ASSERT_TRUE(drive_with_remote_until([this]() {return saw_mode(SupervisorStatus::MODE_FOLLOW);}));

  ASSERT_EQ(
    switch_outside_the_supervisor({}, {kFollowController}),
    controller_interface::return_type::OK);

  ASSERT_TRUE(drive_with_remote_until([this]() {return saw_mode(SupervisorStatus::MODE_IDLE);}))
    << latest_->message;
}

TEST_F(ModeSwitch, EveryAnswerCarriesACauseAndNeverAnEmptySuccess)
{
  clear_the_starting_latch();
  for (const std::uint8_t mode : {SupervisorStatus::MODE_IDLE, SupervisorStatus::MODE_MANUAL,
      SupervisorStatus::MODE_FOLLOW, SupervisorStatus::MODE_MPC, std::uint8_t{200}})
  {
    const auto answer = request(mode);
    ASSERT_NE(answer, nullptr) << static_cast<int>(mode);
    EXPECT_FALSE(answer->message.empty()) << static_cast<int>(mode);
    const std::uint8_t truth = is_active(kFollowController)
      ? SupervisorStatus::MODE_FOLLOW
      : (is_active(kManualController) ? SupervisorStatus::MODE_MANUAL
                                      : SupervisorStatus::MODE_IDLE);
    EXPECT_EQ(answer->active_mode, truth) << answer->message;
  }
}
