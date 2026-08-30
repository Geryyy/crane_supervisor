
#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <future>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "crane_msgs/msg/supervisor_status.hpp"
#include "crane_msgs/msg/sway_settled.hpp"
#include "crane_msgs/msg/velocity_controller_health.hpp"
#include "crane_supervisor/supervisor_node.hpp"
#include "epsilon_crane_msgs/msg/remote_ctrl_states.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace
{

using control_msgs::msg::JointTrajectoryControllerState;
using crane_msgs::msg::SupervisorStatus;
using crane_msgs::msg::VelocityControllerHealth;
using epsilon_crane_msgs::msg::RemoteCtrlStates;
using std_srvs::srv::Trigger;

using SwaySettledMsg = crane_msgs::msg::SwaySettled;

const std::vector<std::string> kActuatedJoints{
  "theta1_slewing_joint", "theta2_boom_joint", "theta3_arm_joint",
  "q4_big_telescope", "theta8_rotator_joint", "q9_left_rail_joint"};

constexpr std::size_t kGripperAxis = 5;

constexpr double kTestTimeout = 0.2;

/// The passive pair as `joint_state_broadcaster` publishes it on `/joint_states`.
/**
 * `age` is how far in the past the pair is stamped when it goes out. It is the only way this
 * fixture can degrade the input, because `sensor_msgs/JointState` carries no validity flag: a
 * producer that has stopped publishing is the whole of what this source can report about itself.
 */
struct PassiveState
{
  double age{0.0};
  std::array<double, crane_supervisor::kPassiveAxisCount> velocity{{0.0, 0.0}};
};

/// How far past its deadline a degraded pair is stamped, s.
constexpr double kDeadAge = 10.0 * kTestTimeout;

constexpr double kBudget = 10.0;

/// The domain this binary runs on: one step off the one it was given.
std::string stepped_domain()
{
  const char * const given = ::getenv("ROS_DOMAIN_ID");
  const int base = (given == nullptr) ? 0 : std::atoi(given);
  return std::to_string(1 + ((base % 101) + 101) % 101);
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

class StatusStream : public ::testing::Test
{
protected:
  virtual rclcpp::NodeOptions node_options() const
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
      {rclcpp::Parameter("pendulum_state_timeout", kTestTimeout),
        rclcpp::Parameter("remote_ctrl_timeout", kTestTimeout),
        rclcpp::Parameter("controller_state_timeout", kTestTimeout),
        rclcpp::Parameter("controller_health_timeout", kTestTimeout)});
    return options;
  }

  void SetUp() override
  {
    supervisor_ = std::make_shared<crane_supervisor::SupervisorNode>(node_options());

    observer_ = std::make_shared<rclcpp::Node>("crane_supervisor_stream_observer");
    const rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    status_ = observer_->create_subscription<SupervisorStatus>(
      crane_supervisor::kStatusTopic, qos,
      [this](SupervisorStatus::ConstSharedPtr message) {received_.push_back(*message);});
    sway_settled_ = observer_->create_subscription<SwaySettledMsg>(
      crane_supervisor::kSwaySettledTopic, qos,
      [this](SwaySettledMsg::ConstSharedPtr message) {settled_.push_back(*message);});
    pendulum_state_ = observer_->create_publisher<sensor_msgs::msg::JointState>(
      crane_supervisor::kPassiveStateTopic, qos);
    remote_ctrl_ = observer_->create_publisher<RemoteCtrlStates>(
      crane_supervisor::kRemoteCtrlStatesTopic, qos);
    controller_state_ = observer_->create_publisher<JointTrajectoryControllerState>(
      crane_supervisor::kControllerStateTopic, qos);
    controller_health_ = observer_->create_publisher<VelocityControllerHealth>(
      crane_supervisor::kControllerHealthTopic, qos);
    clear_fault_ = observer_->create_client<Trigger>(crane_supervisor::kClearFaultService);

    executor_.add_node(supervisor_);
    executor_.add_node(observer_);
  }

  void TearDown() override
  {
    executor_.remove_node(observer_);
    executor_.remove_node(supervisor_);
  }

  /// Spin until `done` holds, running `each_cycle` on every pass.
  bool spin_until(
    const std::function<bool()> & done, const std::function<void()> & each_cycle = nullptr)
  {
    const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::duration<double>(kBudget);
    while (std::chrono::steady_clock::now() < deadline) {
      if (each_cycle) {
        each_cycle();
      }
      if (done()) {
        return true;
      }
      executor_.spin_once(std::chrono::milliseconds(10));
    }
    return done();
  }

  /// The pair the broadcaster would publish with both bracketing units refreshing.
  PassiveState trusted_state() const {return PassiveState{};}

  /// One passive pair on `/joint_states`, named and stamped as the broadcaster stamps it.
  void publish_passive(const PassiveState & state)
  {
    sensor_msgs::msg::JointState message;
    message.header.stamp = observer_->now() - rclcpp::Duration::from_seconds(state.age);
    for (std::size_t axis = 0; axis < crane_supervisor::kPassiveAxisCount; ++axis) {
      message.name.push_back(crane_supervisor::kPassiveAxisNames[axis].joint);
      message.position.push_back(0.0);
      message.velocity.push_back(state.velocity[axis]);
    }
    pendulum_state_->publish(message);
  }

  /// What gpio_controller publishes with the stop released and button 12 held.
  RemoteCtrlStates held_remote() const
  {
    RemoteCtrlStates message;
    message.header.stamp = observer_->now();
    message.button12 = true;
    message.em_stop = false;
    return message;
  }

  /// What the forked trajectory controller publishes when it is tracking.
  JointTrajectoryControllerState tracking_controller_state() const
  {
    JointTrajectoryControllerState message;
    message.header.stamp = observer_->now();
    message.joint_names = kActuatedJoints;
    message.error.positions.assign(kActuatedJoints.size(), 0.0);
    message.error.velocities.assign(kActuatedJoints.size(), 0.0);
    return message;
  }

  /// The same, with one axis out by `position_error` and `velocity_error`.
  JointTrajectoryControllerState controller_state_with_error(
    const std::string & joint, double position_error, double velocity_error) const
  {
    JointTrajectoryControllerState message = tracking_controller_state();
    for (std::size_t i = 0; i < message.joint_names.size(); ++i) {
      if (message.joint_names[i] == joint) {
        message.error.positions[i] = position_error;
        message.error.velocities[i] = velocity_error;
      }
    }
    return message;
  }

  /// What `crane_velocity_controller` publishes when it has nothing to report.
  VelocityControllerHealth healthy_inner_loop() const
  {
    VelocityControllerHealth message;
    message.header.stamp = observer_->now();
    message.fault = SupervisorStatus::FAULT_NONE;
    for (std::size_t i = 0; i < kActuatedJoints.size(); ++i) {
      message.joint_names[i] = kActuatedJoints[i];
      message.feedforward_applied[i] = true;
    }
    return message;
  }

  /// What it publishes on the `hardware` profile today.
  VelocityControllerHealth uncommissioned_gripper() const
  {
    VelocityControllerHealth message = healthy_inner_loop();
    message.fault = SupervisorStatus::FAULT_NOT_COMMISSIONED;
    message.feedforward_applied[kGripperAxis] = false;
    return message;
  }

  /// What the same loop publishes on `fake`, for the same machine state.
  VelocityControllerHealth fake_profile_inner_loop() const
  {
    VelocityControllerHealth message = healthy_inner_loop();
    message.feedforward_applied[kGripperAxis] = false;
    return message;
  }

  void publish_controllers()
  {
    JointTrajectoryControllerState controller = tracking_controller_state();
    controller.header.stamp = observer_->now();
    controller_state_->publish(controller);

    VelocityControllerHealth health = healthy_inner_loop();
    health.header.stamp = observer_->now();
    controller_health_->publish(health);
  }

  /// Publish all four inputs once, stamped now.
  void publish(
    const PassiveState & state, const RemoteCtrlStates & remote,
    const JointTrajectoryControllerState & controller, const VelocityControllerHealth & health)
  {
    publish_passive(state);

    RemoteCtrlStates fresh_remote = remote;
    fresh_remote.header.stamp = observer_->now();
    remote_ctrl_->publish(fresh_remote);

    JointTrajectoryControllerState fresh_controller = controller;
    fresh_controller.header.stamp = observer_->now();
    controller_state_->publish(fresh_controller);

    VelocityControllerHealth fresh_health = health;
    fresh_health.header.stamp = observer_->now();
    controller_health_->publish(fresh_health);
  }

  void publish(
    const PassiveState & state, const RemoteCtrlStates & remote,
    const JointTrajectoryControllerState & controller)
  {
    publish(state, remote, controller, healthy_inner_loop());
  }

  void publish(const PassiveState & state, const RemoteCtrlStates & remote)
  {
    publish(state, remote, tracking_controller_state());
  }

  /// Spin, publishing every input every pass, until the newest report is `fault`.
  bool drive_to(
    std::uint8_t fault, const PassiveState & state, const RemoteCtrlStates & remote,
    const JointTrajectoryControllerState & controller, const VelocityControllerHealth & health)
  {
    return spin_until(
      [this, fault]() {
        return !received_.empty() && received_.back().fault == fault;
      },
      [this, &state, &remote, &controller, &health]() {
        publish(state, remote, controller, health);
      });
  }

  bool drive_to(
    std::uint8_t fault, const PassiveState & state, const RemoteCtrlStates & remote,
    const JointTrajectoryControllerState & controller)
  {
    return drive_to(fault, state, remote, controller, healthy_inner_loop());
  }

  bool drive_to(std::uint8_t fault, const PassiveState & state, const RemoteCtrlStates & remote)
  {
    return drive_to(fault, state, remote, tracking_controller_state());
  }

  bool drive_to(std::uint8_t fault, const PassiveState & state)
  {
    return drive_to(fault, state, held_remote());
  }

  /// Call `/crane/clear_fault` and spin until it answers, publishing meanwhile.
  Trigger::Response::SharedPtr acknowledge(
    const PassiveState & state, const RemoteCtrlStates & remote)
  {
    if (!clear_fault_->wait_for_service(std::chrono::seconds(5))) {
      return nullptr;
    }
    auto future = clear_fault_->async_send_request(std::make_shared<Trigger::Request>());
    const bool answered = spin_until(
      [&future]() {
        return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
      },
      [this, &state, &remote]() {publish(state, remote);});
    return answered ? future.get() : nullptr;
  }

  /// The invariants every published report owes, whatever else a test asserts.
  void expect_contract_of_every_report() const
  {
    for (const SupervisorStatus & status : received_) {
      EXPECT_NE(rclcpp::Time(status.header.stamp).nanoseconds(), 0);
      EXPECT_EQ(status.header.frame_id, "");
      EXPECT_EQ(status.mode, SupervisorStatus::MODE_IDLE);
      EXPECT_NE(status.message.find(crane_supervisor::kModeClausePrefix), std::string::npos)
        << status.message;
      EXPECT_NE(status.message.find("not known"), std::string::npos) << status.message;
      // PRD user story 53: every report carries a cause.
      EXPECT_FALSE(status.message.empty()) << static_cast<int>(status.fault);
    }
  }

  /// Every cycle both streams carried, paired by the stamp they share.
  std::vector<std::pair<SupervisorStatus, SwaySettledMsg>> paired_reports() const
  {
    std::map<std::int64_t, SwaySettledMsg> by_stamp;
    for (const SwaySettledMsg & message : settled_) {
      by_stamp.emplace(rclcpp::Time(message.header.stamp).nanoseconds(), message);
    }
    std::vector<std::pair<SupervisorStatus, SwaySettledMsg>> paired;
    for (const SupervisorStatus & status : received_) {
      const auto found = by_stamp.find(rclcpp::Time(status.header.stamp).nanoseconds());
      if (found != by_stamp.end()) {
        paired.emplace_back(status, found->second);
      }
    }
    return paired;
  }

  /// The invariant the second stream exists to keep: one decision, two carriers.
  void expect_the_field_and_the_sentence_agree() const
  {
    const auto paired = paired_reports();
    EXPECT_FALSE(paired.empty()) << "no cycle reached both streams";
    const std::string head = std::string(crane_supervisor::kSettledClausePrefix).substr(1);
    for (const auto & [status, verdict] : paired) {
      EXPECT_EQ(verdict.header.frame_id, "");
      EXPECT_NE(rclcpp::Time(verdict.header.stamp).nanoseconds(), 0);
      EXPECT_FALSE(verdict.message.empty());
      // The same bytes, not a second sentence about the same thing.
      EXPECT_NE(status.message.find(verdict.message), std::string::npos)
        << status.message << "\n---\n" << verdict.message;
      const auto predicate = static_cast<crane_supervisor::SwaySettled>(verdict.settled);
      EXPECT_EQ(verdict.message.rfind(head + crane_supervisor::settled_word(predicate), 0), 0u)
        << verdict.message;
    }
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<crane_supervisor::SupervisorNode> supervisor_;
  rclcpp::Node::SharedPtr observer_;
  rclcpp::Subscription<SupervisorStatus>::SharedPtr status_;
  rclcpp::Subscription<SwaySettledMsg>::SharedPtr sway_settled_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pendulum_state_;
  rclcpp::Publisher<RemoteCtrlStates>::SharedPtr remote_ctrl_;
  rclcpp::Publisher<JointTrajectoryControllerState>::SharedPtr controller_state_;
  rclcpp::Publisher<VelocityControllerHealth>::SharedPtr controller_health_;
  rclcpp::Client<Trigger>::SharedPtr clear_fault_;
  std::vector<SupervisorStatus> received_;
  std::vector<SwaySettledMsg> settled_;
};

class StatusStreamWithTolerances : public StatusStream
{
protected:
  rclcpp::NodeOptions node_options() const override
  {
    rclcpp::NodeOptions options = StatusStream::node_options();
    options.arguments(
      {"--ros-args", "--params-file",
        std::string(CRANE_SUPERVISOR_TEST_CONFIG_DIR) + "/tracking_tolerance_fixture.yaml"});
    return options;
  }
};

/// The same node, with a different freshness deadline on every input.
class StatusStreamWithDistinctDeadlines : public StatusStream
{
protected:
  rclcpp::NodeOptions node_options() const override
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
      {rclcpp::Parameter("pendulum_state_timeout", 0.11),
        rclcpp::Parameter("remote_ctrl_timeout", 0.22),
        rclcpp::Parameter("controller_state_timeout", 0.33),
        rclcpp::Parameter("controller_health_timeout", 0.44)});
    return options;
  }
};

class StatusStreamWithShortDwell : public StatusStream
{
protected:
  rclcpp::NodeOptions node_options() const override
  {
    rclcpp::NodeOptions options = StatusStream::node_options();
    options.append_parameter_override("sway.settle_dwell", 0.3);
    return options;
  }
};

}  // namespace

TEST_F(StatusStream, TheGraphEndpointIsTheContractOneCraneMsgsAsserts)
{
  ASSERT_TRUE(
    spin_until(
      [this]() {
        return !observer_->get_publishers_info_by_topic(crane_supervisor::kStatusTopic).empty();
      }));

  const auto endpoints =
    observer_->get_publishers_info_by_topic(crane_supervisor::kStatusTopic);
  ASSERT_EQ(endpoints.size(), 1u);
  EXPECT_EQ(endpoints[0].node_name(), "crane_supervisor");
  EXPECT_EQ(endpoints[0].topic_type(), "crane_msgs/msg/SupervisorStatus");

  const rclcpp::QoS & qos = endpoints[0].qos_profile();
  EXPECT_EQ(qos.reliability(), rclcpp::ReliabilityPolicy::Reliable);
  EXPECT_EQ(qos.durability(), rclcpp::DurabilityPolicy::Volatile);
  if (qos.depth() != 0) {
    EXPECT_EQ(qos.depth(), 1u);
  }
}

TEST_F(StatusStream, TheStreamRunsAtTheRateRos2InterfacesGivesIt)
{
  EXPECT_EQ(crane_supervisor::kStatusRate, 20.0);

  constexpr std::size_t kSamples = 20;
  ASSERT_TRUE(spin_until([this]() {return received_.size() >= kSamples;}))
    << "only " << received_.size() << " reports arrived";

  for (std::size_t i = 1; i < kSamples; ++i) {
    EXPECT_GT(
      rclcpp::Time(received_[i].header.stamp).nanoseconds(),
      rclcpp::Time(received_[i - 1].header.stamp).nanoseconds());
  }

  const double span =
    (rclcpp::Time(received_[kSamples - 1].header.stamp) -
    rclcpp::Time(received_[0].header.stamp)).seconds();
  const double gap = span / static_cast<double>(kSamples - 1);
  EXPECT_GT(gap, 0.025) << "the stream is faster than 40 Hz";
  EXPECT_LT(gap, 0.150) << "the stream is slower than 6.7 Hz";

  expect_contract_of_every_report();
}

TEST_F(StatusStream, AbsenceOfTheStopSignalIsAssertedBeforeAnythingArrives)
{
  ASSERT_TRUE(spin_until([this]() {return !received_.empty();}));

  const SupervisorStatus & status = received_.front();
  EXPECT_EQ(status.fault, SupervisorStatus::FAULT_ESTOP);
  EXPECT_FALSE(status.deadman_held);
  EXPECT_NE(status.message.find("has arrived"), std::string::npos) << status.message;
  EXPECT_NE(status.message.find("not protection"), std::string::npos) << status.message;
  expect_contract_of_every_report();
}

TEST_F(StatusStream, AbsenceIsNotHealthBeforeTheFirstPassiveStateArrives)
{
  ASSERT_TRUE(
    spin_until(
      [this]() {
        return !received_.empty() && received_.back().fault == SupervisorStatus::FAULT_STATE_HEALTH;
      },
      [this]() {
        RemoteCtrlStates remote = held_remote();
        remote.header.stamp = observer_->now();
        remote_ctrl_->publish(remote);
      }))
    << received_.back().message;

  EXPECT_NE(received_.back().fault, SupervisorStatus::FAULT_NONE);
  EXPECT_TRUE(received_.back().deadman_held);
  EXPECT_FALSE(received_.back().message.empty());
  expect_contract_of_every_report();
}

TEST_F(StatusStream, TheDeadmanIsCarriedEndToEndAndCheckedEveryCycle)
{
  ASSERT_TRUE(drive_to(SupervisorStatus::FAULT_NONE, trusted_state(), held_remote()))
    << received_.back().message;
  EXPECT_TRUE(received_.back().deadman_held);

  RemoteCtrlStates released = held_remote();
  released.button12 = false;
  ASSERT_TRUE(drive_to(SupervisorStatus::FAULT_INTERLOCK, trusted_state(), released))
    << received_.back().message;
  EXPECT_FALSE(received_.back().deadman_held);
  EXPECT_NE(received_.back().message.find("button 12"), std::string::npos)
    << received_.back().message;

  // Pressed again, and it clears itself: an interlock is not a latch.
  ASSERT_TRUE(drive_to(SupervisorStatus::FAULT_NONE, trusted_state(), held_remote()))
    << received_.back().message;
  EXPECT_TRUE(received_.back().deadman_held);

  expect_contract_of_every_report();
}

TEST_F(StatusStream, TheStopLatchesAndOnlyTheAcknowledgementClearsIt)
{
  ASSERT_TRUE(drive_to(SupervisorStatus::FAULT_NONE, trusted_state(), held_remote()))
    << received_.back().message;

  RemoteCtrlStates asserted = held_remote();
  asserted.em_stop = true;
  ASSERT_TRUE(drive_to(SupervisorStatus::FAULT_ESTOP, trusted_state(), asserted))
    << received_.back().message;

  const auto refused = acknowledge(trusted_state(), asserted);
  ASSERT_NE(refused, nullptr);
  EXPECT_FALSE(refused->success);
  EXPECT_FALSE(refused->message.empty());
  EXPECT_NE(refused->message.find("still asserted"), std::string::npos) << refused->message;

  const std::size_t before = received_.size();
  ASSERT_TRUE(
    spin_until(
      [this, before]() {return received_.size() > before + 10;},
      [this]() {publish(trusted_state(), held_remote());}));
  EXPECT_EQ(received_.back().fault, SupervisorStatus::FAULT_ESTOP);
  EXPECT_NE(received_.back().message.find("latched"), std::string::npos)
    << received_.back().message;

  // And the acknowledgement clears it.
  const auto cleared = acknowledge(trusted_state(), held_remote());
  ASSERT_NE(cleared, nullptr);
  EXPECT_TRUE(cleared->success);
  EXPECT_FALSE(cleared->message.empty());
  ASSERT_TRUE(drive_to(SupervisorStatus::FAULT_NONE, trusted_state(), held_remote()))
    << received_.back().message;

  // A cleared latch that re-raises on the next cycle is the correct behaviour.
  ASSERT_TRUE(drive_to(SupervisorStatus::FAULT_ESTOP, trusted_state(), asserted))
    << received_.back().message;

  expect_contract_of_every_report();
}

TEST_F(StatusStream, TheRemoteStoppingIsAssertedRatherThanReleased)
{
  ASSERT_TRUE(drive_to(SupervisorStatus::FAULT_NONE, trusted_state(), held_remote()))
    << received_.back().message;

  ASSERT_TRUE(
    spin_until(
      [this]() {return received_.back().fault == SupervisorStatus::FAULT_ESTOP;},
      [this]() {
        publish_passive(trusted_state());
        publish_controllers();
      }))
    << received_.back().message;
  EXPECT_NE(received_.back().message.find("stopped arriving"), std::string::npos)
    << received_.back().message;
  EXPECT_FALSE(received_.back().deadman_held);

  const std::size_t before = received_.size();
  ASSERT_TRUE(
    spin_until(
      [this, before]() {return received_.size() > before + 10;},
      [this]() {publish(trusted_state(), held_remote());}));
  EXPECT_EQ(received_.back().fault, SupervisorStatus::FAULT_ESTOP);

  const auto cleared = acknowledge(trusted_state(), held_remote());
  ASSERT_NE(cleared, nullptr);
  EXPECT_TRUE(cleared->success);
  ASSERT_TRUE(drive_to(SupervisorStatus::FAULT_NONE, trusted_state(), held_remote()))
    << received_.back().message;

  expect_contract_of_every_report();
}

TEST_F(StatusStream, ThePassiveStateIsCarriedEndToEnd)
{
  ASSERT_TRUE(drive_to(SupervisorStatus::FAULT_NONE, trusted_state()))
    << received_.back().message;

  PassiveState degraded = trusted_state();
  degraded.age = kDeadAge;
  ASSERT_TRUE(drive_to(SupervisorStatus::FAULT_STATE_HEALTH, degraded));
  EXPECT_NE(received_.back().message.find("stopped arriving"), std::string::npos)
    << received_.back().message;

  expect_contract_of_every_report();
}

TEST_F(StatusStream, ASwingingLoadReachesTheStreamAsATypedCauseThatNamesTheCoordinate)
{
  ASSERT_TRUE(drive_to(SupervisorStatus::FAULT_NONE, trusted_state()))
    << received_.back().message;

  PassiveState swinging = trusted_state();
  swinging.velocity[1] = 0.9;
  ASSERT_TRUE(drive_to(SupervisorStatus::FAULT_SWAY, swinging)) << received_.back().message;

  const std::string & message = received_.back().message;
  EXPECT_NE(message.find("theta7_tilt_joint"), std::string::npos) << message;
  EXPECT_EQ(message.find("theta6_tip_joint"), std::string::npos) << message;
  EXPECT_NE(message.find("0.9000"), std::string::npos) << message;
  EXPECT_NE(message.find("0.4000"), std::string::npos) << message;
  EXPECT_NE(message.find("Nothing was stopped"), std::string::npos) << message;

  PassiveState degraded = swinging;
  degraded.age = kDeadAge;
  ASSERT_TRUE(drive_to(SupervisorStatus::FAULT_STATE_HEALTH, degraded))
    << received_.back().message;
  EXPECT_NE(received_.back().message.find("stopped arriving"), std::string::npos)
    << received_.back().message;

  ASSERT_TRUE(drive_to(SupervisorStatus::FAULT_NONE, trusted_state()))
    << received_.back().message;

  expect_contract_of_every_report();
}

TEST_F(StatusStreamWithShortDwell, TheSettledPredicateIsOnEveryReportAndPassesThroughAllThreeStates)
{
  ASSERT_TRUE(spin_until([this]() {return !received_.empty();}));
  EXPECT_NE(
    received_.front().message.find(crane_supervisor::kSettledClausePrefix), std::string::npos)
    << received_.front().message;
  EXPECT_NE(received_.front().message.find("not known"), std::string::npos)
    << received_.front().message;
  EXPECT_EQ(received_.front().fault, SupervisorStatus::FAULT_ESTOP);

  const auto cleared = acknowledge(trusted_state(), held_remote());
  ASSERT_NE(cleared, nullptr);
  EXPECT_TRUE(cleared->success) << cleared->message;

  // A still crane, held long enough for the dwell: settled.
  ASSERT_TRUE(
    spin_until(
      [this]() {
        return !received_.empty() &&
        received_.back().message.find("Sway: settled") != std::string::npos;
      },
      [this]() {publish(trusted_state(), held_remote());}))
    << received_.back().message;
  EXPECT_EQ(received_.back().fault, SupervisorStatus::FAULT_NONE) << received_.back().message;

  PassiveState degraded = trusted_state();
  degraded.age = kDeadAge;
  ASSERT_TRUE(drive_to(SupervisorStatus::FAULT_STATE_HEALTH, degraded))
    << received_.back().message;
  EXPECT_NE(received_.back().message.find("Sway: not known"), std::string::npos)
    << received_.back().message;

  // And every report carried the clause, whatever its fault was.
  for (const SupervisorStatus & status : received_) {
    EXPECT_NE(status.message.find(crane_supervisor::kSettledClausePrefix), std::string::npos)
      << status.message;
  }

  expect_contract_of_every_report();
}

TEST_F(StatusStream, TheSettledPredicateIsAFieldOnAStreamOfItsOwn)
{
  ASSERT_TRUE(
    spin_until(
      [this]() {
        return !observer_->get_publishers_info_by_topic(crane_supervisor::kSwaySettledTopic)
        .empty();
      }));

  const auto endpoints =
    observer_->get_publishers_info_by_topic(crane_supervisor::kSwaySettledTopic);
  ASSERT_EQ(endpoints.size(), 1u);
  EXPECT_EQ(endpoints[0].node_name(), "crane_supervisor");
  EXPECT_EQ(endpoints[0].topic_type(), "crane_msgs/msg/SwaySettled");

  const rclcpp::QoS & qos = endpoints[0].qos_profile();
  EXPECT_EQ(qos.reliability(), rclcpp::ReliabilityPolicy::Reliable);
  EXPECT_EQ(qos.durability(), rclcpp::DurabilityPolicy::Volatile);
  if (qos.depth() != 0) {
    EXPECT_EQ(qos.depth(), 1u);
  }

  ASSERT_TRUE(spin_until([this]() {return !settled_.empty();}));
  EXPECT_EQ(settled_.front().settled, SwaySettledMsg::SETTLED_UNKNOWN);
  EXPECT_EQ(settled_.front().header.frame_id, "");
  EXPECT_NE(rclcpp::Time(settled_.front().header.stamp).nanoseconds(), 0);
  EXPECT_FALSE(settled_.front().message.empty());
  EXPECT_TRUE(std::isnan(settled_.front().velocity[0])) << settled_.front().velocity[0];
  EXPECT_TRUE(std::isnan(settled_.front().velocity[1])) << settled_.front().velocity[1];
}

TEST_F(StatusStreamWithShortDwell, TheFieldAndTheSentenceAreOneDecisionAndCannotDisagree)
{
  ASSERT_TRUE(spin_until([this]() {return !settled_.empty();}));
  EXPECT_EQ(settled_.front().settled, SwaySettledMsg::SETTLED_UNKNOWN);

  const auto cleared = acknowledge(trusted_state(), held_remote());
  ASSERT_NE(cleared, nullptr);
  EXPECT_TRUE(cleared->success) << cleared->message;

  PassiveState still = trusted_state();
  still.velocity[0] = 0.01;
  still.velocity[1] = -0.005;
  ASSERT_TRUE(
    spin_until(
      [this]() {
        return !settled_.empty() && settled_.back().settled == SwaySettledMsg::SETTLED_YES;
      },
      [this, &still]() {publish(still, held_remote());}))
    << settled_.back().message;
  EXPECT_DOUBLE_EQ(settled_.back().velocity[0], 0.01);
  EXPECT_DOUBLE_EQ(settled_.back().velocity[1], -0.005);

  PassiveState swinging = trusted_state();
  swinging.velocity[0] = 0.2;
  ASSERT_TRUE(
    spin_until(
      [this]() {
        return !settled_.empty() && settled_.back().settled == SwaySettledMsg::SETTLED_NO;
      },
      [this, &swinging]() {publish(swinging, held_remote());}))
    << settled_.back().message;
  EXPECT_EQ(received_.back().fault, SupervisorStatus::FAULT_NONE) << received_.back().message;

  PassiveState degraded = still;
  degraded.age = kDeadAge;
  ASSERT_TRUE(
    spin_until(
      [this]() {
        return !settled_.empty() && settled_.back().settled == SwaySettledMsg::SETTLED_UNKNOWN &&
        !received_.empty() && received_.back().fault == SupervisorStatus::FAULT_STATE_HEALTH;
      },
      [this, &degraded]() {publish(degraded, held_remote());}))
    << settled_.back().message;

  // Every cycle that reached both streams carried one decision on two carriers.
  expect_the_field_and_the_sentence_agree();
  expect_contract_of_every_report();
}

TEST_F(StatusStream, TheTrackingErrorIsCarriedEndToEndFromTheControllersOwnState)
{
  ASSERT_TRUE(
    drive_to(
      SupervisorStatus::FAULT_NONE, trusted_state(), held_remote(),
      controller_state_with_error("theta3_arm_joint", -0.25, 0.0)))
    << received_.back().message;
  EXPECT_DOUBLE_EQ(received_.back().tracking_error, 0.25);

  EXPECT_NE(received_.back().message.find("No axis is being compared"), std::string::npos)
    << received_.back().message;
  EXPECT_NE(received_.back().message.find("human-only"), std::string::npos)
    << received_.back().message;

  expect_contract_of_every_report();
}

TEST_F(StatusStream, AControllerThatStopsPublishingIsAFaultAndNotAZeroError)
{
  ASSERT_TRUE(
    drive_to(
      SupervisorStatus::FAULT_NONE, trusted_state(), held_remote(),
      controller_state_with_error("q4_big_telescope", 0.31, 0.0)))
    << received_.back().message;
  EXPECT_DOUBLE_EQ(received_.back().tracking_error, 0.31);

  ASSERT_TRUE(
    spin_until(
      [this]() {
        return received_.back().fault == SupervisorStatus::FAULT_STATE_HEALTH;
      },
      [this]() {
        publish_passive(trusted_state());
        RemoteCtrlStates remote = held_remote();
        remote.header.stamp = observer_->now();
        remote_ctrl_->publish(remote);
        VelocityControllerHealth health = healthy_inner_loop();
        health.header.stamp = observer_->now();
        controller_health_->publish(health);
      }))
    << received_.back().message;
  EXPECT_NE(received_.back().message.find("stopped publishing its own state"), std::string::npos)
    << received_.back().message;
  EXPECT_DOUBLE_EQ(received_.back().tracking_error, 0.0);

  ASSERT_TRUE(drive_to(SupervisorStatus::FAULT_NONE, trusted_state(), held_remote()))
    << received_.back().message;

  expect_contract_of_every_report();
}

TEST_F(StatusStreamWithTolerances, ATrackingExcessIsATypedCauseThatNamesTheAxis)
{
  ASSERT_TRUE(drive_to(SupervisorStatus::FAULT_NONE, trusted_state(), held_remote()))
    << received_.back().message;
  // With every axis inside its tolerance the clear report makes no excuse.
  EXPECT_EQ(received_.back().message.find("No axis is being compared"), std::string::npos)
    << received_.back().message;

  // theta2_boom_joint's fixture tolerance is 0.02 rad/s.
  ASSERT_TRUE(
    drive_to(
      SupervisorStatus::FAULT_TRACKING, trusted_state(), held_remote(),
      controller_state_with_error("theta2_boom_joint", 0.11, 0.09)))
    << received_.back().message;
  EXPECT_NE(received_.back().message.find("theta2_boom_joint"), std::string::npos)
    << received_.back().message;
  EXPECT_NE(received_.back().message.find("rad/s"), std::string::npos)
    << received_.back().message;
  EXPECT_DOUBLE_EQ(received_.back().tracking_error, 0.11);

  ASSERT_TRUE(drive_to(SupervisorStatus::FAULT_NONE, trusted_state(), held_remote()))
    << received_.back().message;

  expect_contract_of_every_report();
}

TEST_F(StatusStreamWithTolerances, TheSixNumbersComeOutOfAFileInTheSharedShape)
{
  const auto & tolerances = supervisor_->config().tracking_tolerance;
  ASSERT_EQ(tolerances.size(), kActuatedJoints.size());
  for (std::size_t i = 0; i < tolerances.size(); ++i) {
    EXPECT_EQ(tolerances[i].joint, kActuatedJoints[i]);
    EXPECT_DOUBLE_EQ(tolerances[i].dq_a, 0.01 * static_cast<double>(i + 1));
  }
  // Every axis has one, so there is nothing left to warn about.
  EXPECT_TRUE(crane_supervisor::tracking_tolerance_notice(supervisor_->config()).empty());
}

TEST_F(StatusStream, WithNoToleranceFileNoTrackingFaultIsRaisedAndTheNodeSaysSo)
{
  const std::string notice =
    crane_supervisor::tracking_tolerance_notice(supervisor_->config());
  EXPECT_FALSE(notice.empty());
  EXPECT_NE(notice.find("theta1_slewing_joint"), std::string::npos) << notice;
  EXPECT_NE(notice.find("(ii-b)"), std::string::npos) << notice;

  ASSERT_TRUE(
    drive_to(
      SupervisorStatus::FAULT_NONE, trusted_state(), held_remote(),
      controller_state_with_error("theta1_slewing_joint", 1.5, 4.0)))
    << received_.back().message;
  EXPECT_DOUBLE_EQ(received_.back().tracking_error, 1.5);
  for (const SupervisorStatus & status : received_) {
    EXPECT_NE(status.fault, SupervisorStatus::FAULT_TRACKING) << status.message;
  }

  expect_contract_of_every_report();
}

TEST_F(StatusStreamWithDistinctDeadlines, EveryInputsDeadlineComesFromItsOwnParameter)
{
  const auto & config = supervisor_->config();
  EXPECT_DOUBLE_EQ(config.deadline(crane_supervisor::Input::PendulumState), 0.11);
  EXPECT_DOUBLE_EQ(config.deadline(crane_supervisor::Input::RemoteCtrl), 0.22);
  EXPECT_DOUBLE_EQ(config.deadline(crane_supervisor::Input::ControllerState), 0.33);
  EXPECT_DOUBLE_EQ(config.deadline(crane_supervisor::Input::ControllerHealth), 0.44);

  // And every input has one: the node would not have been constructed otherwise.
  std::string reason;
  EXPECT_TRUE(crane_supervisor::validate(config, reason)) << reason;
  for (std::size_t i = 0; i < crane_supervisor::kInputCount; ++i) {
    EXPECT_GT(config.freshness_deadline[i], 0.0)
      << crane_supervisor::kInputPolicies[i].topic;
  }
}

TEST_F(StatusStream, AMarginOutsideItsBoundsIsRefusedBeforeTheNodePublishesAnything)
{
  rclcpp::NodeOptions refused;
  refused.parameter_overrides({rclcpp::Parameter("controller_health_timeout", 0.0)});
  EXPECT_THROW(
    std::make_shared<crane_supervisor::SupervisorNode>(refused), std::exception);
}

TEST_F(StatusStream, ADeadlineFiresWithNoMessageArrivingAtAllToNoticeIt)
{
  ASSERT_TRUE(drive_to(SupervisorStatus::FAULT_NONE, trusted_state(), held_remote()))
    << received_.back().message;
  const std::size_t before = received_.size();

  ASSERT_TRUE(
    spin_until([this]() {return received_.back().fault != SupervisorStatus::FAULT_NONE;}))
    << received_.back().message;
  EXPECT_GT(received_.size(), before);
  EXPECT_EQ(received_.back().fault, SupervisorStatus::FAULT_ESTOP);
  EXPECT_NE(received_.back().message.find("stopped arriving"), std::string::npos)
    << received_.back().message;
  EXPECT_NE(received_.back().message.find("Staleness cause"), std::string::npos)
    << received_.back().message;
  EXPECT_NE(
    received_.back().message.find(crane_supervisor::kRemoteCtrlStatesTopic), std::string::npos)
    << received_.back().message;

  expect_contract_of_every_report();
}

TEST_F(StatusStream, TheStreamStoppingBringsTheFaultBackRatherThanLeavingItClear)
{
  ASSERT_TRUE(drive_to(SupervisorStatus::FAULT_NONE, trusted_state()))
    << received_.back().message;

  ASSERT_TRUE(
    spin_until(
      [this]() {return received_.back().fault == SupervisorStatus::FAULT_STATE_HEALTH;},
      [this]() {
        RemoteCtrlStates remote = held_remote();
        remote.header.stamp = observer_->now();
        remote_ctrl_->publish(remote);
        publish_controllers();
      }));
  EXPECT_NE(received_.back().message.find("stopped arriving"), std::string::npos)
    << received_.back().message;

  expect_contract_of_every_report();
}

TEST_F(StatusStream, TheInnerLoopsFaultReachesTheStreamAndNamesTheAxisItIsAbout)
{
  ASSERT_TRUE(
    drive_to(
      SupervisorStatus::FAULT_NONE, trusted_state(), held_remote(),
      tracking_controller_state(), healthy_inner_loop()))
    << received_.back().message;

  ASSERT_TRUE(
    drive_to(
      SupervisorStatus::FAULT_NOT_COMMISSIONED, trusted_state(), held_remote(),
      tracking_controller_state(), uncommissioned_gripper()))
    << received_.back().message;
  EXPECT_NE(received_.back().message.find("q9_left_rail_joint"), std::string::npos)
    << received_.back().message;
  EXPECT_NE(received_.back().message.find("calibration"), std::string::npos)
    << received_.back().message;
  // And no other axis is named: the other five apply the valve inverse normally.
  EXPECT_EQ(received_.back().message.find("theta1_slewing_joint"), std::string::npos)
    << received_.back().message;

  // The supervisor reports it and does nothing else in this slice.
  EXPECT_NE(received_.back().message.find("Nothing was stopped"), std::string::npos)
    << received_.back().message;

  PassiveState degraded = trusted_state();
  degraded.age = kDeadAge;
  ASSERT_TRUE(
    drive_to(
      SupervisorStatus::FAULT_STATE_HEALTH, degraded, held_remote(),
      tracking_controller_state(), uncommissioned_gripper()))
    << received_.back().message;
  EXPECT_NE(received_.back().message.find("stopped arriving"), std::string::npos)
    << received_.back().message;

  ASSERT_TRUE(
    drive_to(
      SupervisorStatus::FAULT_NONE, trusted_state(), held_remote(),
      tracking_controller_state(), healthy_inner_loop()))
    << received_.back().message;

  expect_contract_of_every_report();
}

TEST_F(StatusStream, ARigWithNoHydraulicsReportsNothingAndTheOneSwitchIsTheControllers)
{
  ASSERT_TRUE(
    drive_to(
      SupervisorStatus::FAULT_NONE, trusted_state(), held_remote(),
      tracking_controller_state(), fake_profile_inner_loop()))
    << received_.back().message;

  EXPECT_FALSE(fake_profile_inner_loop().feedforward_applied[kGripperAxis]);
  for (const SupervisorStatus & status : received_) {
    EXPECT_NE(status.fault, SupervisorStatus::FAULT_NOT_COMMISSIONED) << status.message;
  }

  expect_contract_of_every_report();
}

TEST_F(StatusStream, AnInnerLoopThatIsNotReportingIsAFaultRatherThanAClearReport)
{
  ASSERT_TRUE(
    spin_until(
      [this]() {
        return !received_.empty() && received_.back().fault == SupervisorStatus::FAULT_STATE_HEALTH;
      },
      [this]() {
        publish_passive(trusted_state());
        RemoteCtrlStates remote = held_remote();
        remote.header.stamp = observer_->now();
        remote_ctrl_->publish(remote);
        JointTrajectoryControllerState controller = tracking_controller_state();
        controller.header.stamp = observer_->now();
        controller_state_->publish(controller);
      }))
    << received_.back().message;
  EXPECT_NE(
    received_.back().message.find("/crane/velocity_controller/health"), std::string::npos)
    << received_.back().message;
  const std::string never_arrived = received_.back().message;

  // Then the loop comes up and the report clears.
  ASSERT_TRUE(drive_to(SupervisorStatus::FAULT_NONE, trusted_state(), held_remote()))
    << received_.back().message;

  ASSERT_TRUE(
    spin_until(
      [this]() {return received_.back().fault == SupervisorStatus::FAULT_STATE_HEALTH;},
      [this]() {
        publish_passive(trusted_state());
        RemoteCtrlStates remote = held_remote();
        remote.header.stamp = observer_->now();
        remote_ctrl_->publish(remote);
        JointTrajectoryControllerState controller = tracking_controller_state();
        controller.header.stamp = observer_->now();
        controller_state_->publish(controller);
      }))
    << received_.back().message;
  EXPECT_NE(received_.back().message.find("stopped reporting its own health"), std::string::npos)
    << received_.back().message;
  EXPECT_NE(received_.back().message, never_arrived);

  ASSERT_TRUE(drive_to(SupervisorStatus::FAULT_NONE, trusted_state(), held_remote()))
    << received_.back().message;

  expect_contract_of_every_report();
}
