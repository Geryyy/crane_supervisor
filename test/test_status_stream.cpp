// The published stream, against the contract `crane_msgs`' own ROS test already
// asserts for `/crane/supervisor/status`: `crane_msgs/SupervisorStatus`,
// reliable, depth 1, `header.stamp` set, `frame_id` empty, 20 Hz.
//
// In process.  The node object and the observer are two nodes in one executor
// on the isolated domain `ralph/verify.sh` pins, with localhost-only transport:
// no launch, no controller manager, no simulator, no hardware.  Nothing here
// sleeps -- every wait is a bounded spin on a predicate, so a fast machine
// finishes early instead of paying for someone else's margin.

#include <gtest/gtest.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <future>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "crane_msgs/msg/pendulum_state.hpp"
#include "crane_msgs/msg/supervisor_status.hpp"
#include "crane_msgs/msg/velocity_controller_health.hpp"
#include "crane_supervisor/supervisor_node.hpp"
#include "epsilon_crane_msgs/msg/remote_ctrl_states.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace
{

using control_msgs::msg::JointTrajectoryControllerState;
using crane_msgs::msg::PendulumState;
using crane_msgs::msg::SupervisorStatus;
using crane_msgs::msg::VelocityControllerHealth;
using epsilon_crane_msgs::msg::RemoteCtrlStates;
using std_srvs::srv::Trigger;

/// The six actuated joints of ROS 2 Interfaces §3.2, as the trajectory
/// controller publishes them in `joint_names`.
const std::vector<std::string> kActuatedJoints{
  "theta1_slewing_joint", "theta2_boom_joint", "theta3_arm_joint",
  "q4_big_telescope", "theta8_rotator_joint", "q9_left_rail_joint"};

/// The gripper axis: the one the active tool has no valve calibration for
/// (commissioning_prerequisites §1 row 4).  Index 5 of the six, and the axis
/// `crane_control`'s own S5 harness asserts the loop runs PI only.
constexpr std::size_t kGripperAxis = 5;

/// Shorter than the shipped margin so that "the stream stopped" is reachable
/// inside a test budget.  It is an override of the declared parameter, not a
/// second default: the node is built the way a deployment builds it.
constexpr double kTestTimeout = 0.2;

/// How long any one wait may take before the test fails, s.  Generous, because
/// it bounds a failure rather than a success: every wait returns as soon as its
/// predicate holds.
constexpr double kBudget = 10.0;

/// The domain this binary runs on: one step off the one it was given.
/**
 * `verify.sh` already puts each issue's run on a domain of its own, which keeps
 * these nodes off the machine's live graph and off another issue's.  What it
 * cannot do is separate this package's tests from `crane_control`'s, and this
 * issue is the first whose `packages:` names both: colcon runs the two suites at
 * the same time, and they meet on the contract names.  This fixture publishes
 * `/crane/pendulum_state` and `/crane/velocity_controller/health`, and
 * `crane_control`'s harnesses are the real producers of both -- so each suite
 * reads the other's fixture and neither is testing what it thinks it is.
 *
 * The step is taken here rather than in the harness for the same reason the two
 * variables below are cleared here: an isolation a bare `colcon test` does not
 * get is an isolation that comes back the first time somebody runs one.
 * `1 + (given + 50) % 101` lands in 1..101 -- the range ROS 2 keeps clear of the
 * Linux ephemeral ports -- never returns the domain it was given, and sends two
 * different given domains to two different ones, so two issues running at once
 * stay apart here as well.
 */
std::string stepped_domain()
{
  const char * const given = ::getenv("ROS_DOMAIN_ID");
  // A domain that is absent or is not a number at all is domain 0, which is what
  // rclcpp itself would have used.
  const int base = (given == nullptr) ? 0 : std::atoi(given);
  return std::to_string(1 + ((base % 101) + 101) % 101);
}

/// One `rclcpp` context for the whole binary.  Bringing one up and down per
/// test would be five DDS participants' worth of discovery for no assertion.
class RclcppEnvironment : public ::testing::Environment
{
public:
  void SetUp() override
  {
    // The isolation is the test's own, not the harness's.  This workspace ships
    // a Cyclone configuration that pins a unicast peer at the real crane, and a
    // shell that has sourced it would put this node on the machine's live DDS
    // graph -- where duplicate nodes have already been observed.  `verify.sh`
    // already scrubs both variables; doing it here as well means the guarantee
    // survives a bare `colcon test` in somebody's own shell.
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
  /// How the node under test is built. Overridden by the fixture that hands it
  /// a tracking tolerance, so that both fixtures run the deployment's node
  /// rather than a second one written for the test.
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
    pendulum_state_ = observer_->create_publisher<PendulumState>(
      crane_supervisor::kPendulumStateTopic, qos);
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

  /// A message the broadcaster would publish when the estimate is trusted.
  PendulumState trusted_state() const
  {
    PendulumState message;
    message.header.stamp = observer_->now();
    message.valid = true;
    message.status = "complementary filter on the two bracketing IMUs, both healthy";
    return message;
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
  /**
   * Six joints, `error.positions` and `error.velocities` both filled -- which is
   * what the FOLLOW profile produces, since it gives the controller a velocity
   * state interface and a velocity command interface.
   */
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
  /**
   * Six axes named on the wire and the feedforward applied on every one of them.
   * The names are carried rather than assumed because the sixth valve channel
   * drives a different joint per tool, and the controller is the one that knows
   * which tool it is driving.
   */
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
  /**
   * Prerequisite 4 is missing, so the gripper axis runs with the feedforward
   * disabled and PI only, and the loop raises `FAULT_NOT_COMMISSIONED`
   * (commissioning_prerequisites §1 row 4).  `crane_control`'s own S5 harness
   * asserts that this is the message that leaves the controller manager; what is
   * asserted here is that it reaches `/crane/supervisor/status`.
   */
  VelocityControllerHealth uncommissioned_gripper() const
  {
    VelocityControllerHealth message = healthy_inner_loop();
    message.fault = SupervisorStatus::FAULT_NOT_COMMISSIONED;
    message.feedforward_applied[kGripperAxis] = false;
    return message;
  }

  /// What the same loop publishes on `fake`, for the same machine state.
  /**
   * A rig with no hydraulics has nothing to commission (§3), so the loop raises
   * no fault -- while the calibration is no less missing, and the flag that says
   * which axis ran PI only is still false.  The switch between this report and
   * the one above is `crane_velocity_controller`'s own `profile` parameter, and
   * this package has no second one.
   */
  VelocityControllerHealth fake_profile_inner_loop() const
  {
    VelocityControllerHealth message = healthy_inner_loop();
    message.feedforward_applied[kGripperAxis] = false;
    return message;
  }

  /// Publish the trajectory controller's state and the inner loop's health,
  /// both healthy and stamped now.
  /**
   * What a test about one of the *other* two inputs has to keep alive, so that
   * the absence it is about is the only one in the report.  Four streams that
   * all stop at once would still raise a fault, and the test would pass without
   * ever showing which of them produced it.
   */
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
    const PendulumState & state, const RemoteCtrlStates & remote,
    const JointTrajectoryControllerState & controller, const VelocityControllerHealth & health)
  {
    PendulumState fresh_state = state;
    fresh_state.header.stamp = observer_->now();
    pendulum_state_->publish(fresh_state);

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
    const PendulumState & state, const RemoteCtrlStates & remote,
    const JointTrajectoryControllerState & controller)
  {
    publish(state, remote, controller, healthy_inner_loop());
  }

  void publish(const PendulumState & state, const RemoteCtrlStates & remote)
  {
    publish(state, remote, tracking_controller_state());
  }

  /// Spin, publishing every input every pass, until the newest report is `fault`.
  bool drive_to(
    std::uint8_t fault, const PendulumState & state, const RemoteCtrlStates & remote,
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
    std::uint8_t fault, const PendulumState & state, const RemoteCtrlStates & remote,
    const JointTrajectoryControllerState & controller)
  {
    return drive_to(fault, state, remote, controller, healthy_inner_loop());
  }

  bool drive_to(std::uint8_t fault, const PendulumState & state, const RemoteCtrlStates & remote)
  {
    return drive_to(fault, state, remote, tracking_controller_state());
  }

  bool drive_to(std::uint8_t fault, const PendulumState & state)
  {
    return drive_to(fault, state, held_remote());
  }

  /// Call `/crane/clear_fault` and spin until it answers, publishing meanwhile.
  /**
   * The publication has to continue during the call: the acknowledgement is
   * judged against the newest sample the supervisor holds, and a test that went
   * quiet while it waited would be acknowledging against an absent remote.
   */
  Trigger::Response::SharedPtr acknowledge(
    const PendulumState & state, const RemoteCtrlStates & remote)
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
      // ROS 2 Interfaces §1: every new streamed contract with a header stamps
      // it, and status data carries no geometric expression frame.
      EXPECT_NE(rclcpp::Time(status.header.stamp).nanoseconds(), 0);
      EXPECT_EQ(status.header.frame_id, "");
      // There is no controller manager on this domain, which is exactly the
      // case the mode clause exists for: the mode is *not known*, MODE_IDLE is
      // reported because no motion mode can be confirmed, and the words say
      // which of the two it is.  A supervisor that remembered a mode instead
      // would put one here off a request that never happened.  Real switches
      // against a real manager are `test_mode_switch.cpp`.
      EXPECT_EQ(status.mode, SupervisorStatus::MODE_IDLE);
      EXPECT_NE(status.message.find(crane_supervisor::kModeClausePrefix), std::string::npos)
        << status.message;
      EXPECT_NE(status.message.find("not known"), std::string::npos) << status.message;
      // PRD user story 53: every report carries a cause.
      EXPECT_FALSE(status.message.empty()) << static_cast<int>(status.fault);
    }
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<crane_supervisor::SupervisorNode> supervisor_;
  rclcpp::Node::SharedPtr observer_;
  rclcpp::Subscription<SupervisorStatus>::SharedPtr status_;
  rclcpp::Publisher<PendulumState>::SharedPtr pendulum_state_;
  rclcpp::Publisher<RemoteCtrlStates>::SharedPtr remote_ctrl_;
  rclcpp::Publisher<JointTrajectoryControllerState>::SharedPtr controller_state_;
  rclcpp::Publisher<VelocityControllerHealth>::SharedPtr controller_health_;
  rclcpp::Client<Trigger>::SharedPtr clear_fault_;
  std::vector<SupervisorStatus> received_;
};

/// The same node, handed the six numbers out of a file in the shape
/// `crane_control/config/tracking_tolerance.yaml` has.
/**
 * A `--params-file`, not a `parameter_overrides` list, and the fixture keeps the
 * real file's wildcard node key: what is asserted is that this node can read the
 * file the seam clamp and the MPC read, not that it can be handed six doubles.
 * The numbers in the fixture are arbitrary and its own header says so -- the
 * tolerance for any machine comes from merge gate (ii-b) or the identification
 * campaign, both human-only, and no automated test in this workspace can produce
 * one.
 */
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
/**
 * The fixture above gives all four the same number, which is exactly what a node
 * that routed two parameters into one input's slot would look like from the
 * outside: every input would go stale at the same age, which is the one global
 * number the policy of §5.3 exists not to be.  So each of the four gets a value
 * of its own here, and the test below reads them back out of the registry.
 */
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

/// The same node, with the settle dwell shortened to something a stream test can
/// hold continuously.
/**
 * The shipped dwell is 2.0 s -- above the half period of the pendulum, which is
 * the point of it -- and reaching `settled` through this fixture means keeping
 * `/crane/pendulum_state` fresh for that whole time on a machine running
 * parallel colcon jobs.  One scheduling hiccup longer than `kTestTimeout` makes
 * the estimate stale, which correctly resets the dwell, and the test would fail
 * for a reason that is not the contract.
 *
 * So the *margin* is moved rather than the clock, exactly as
 * `crane_control`'s `pendulum_stale_margin.yaml` does for the same problem one
 * layer down.  What the shipped 2.0 s is derived from, and that it clears the
 * pendulum's half period, is asserted in `test_sway_monitor.cpp`, where the
 * dwell is an argument and no wall clock is involved at all.
 */
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
  // Some Humble RMWs report an endpoint's history depth as 0/UNKNOWN even
  // though the entity retained the requested depth, so the graph's depth is
  // checked only when it is exposed -- the same allowance `crane_msgs`' own
  // contract test makes.  The publisher's own profile is checked outright.
  if (qos.depth() != 0) {
    EXPECT_EQ(qos.depth(), 1u);
  }
}

TEST_F(StatusStream, TheStreamRunsAtTheRateRos2InterfacesGivesIt)
{
  // 20 Hz, ROS 2 Interfaces §2.  Asserted on what was delivered rather than on
  // the timer that produced it: the contract is about the stream an operator
  // panel subscribes to.
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
  // A band, not a tolerance: the lower bound catches a stream that free-runs
  // and the upper one a stream that is an order too slow to be a status stream,
  // and neither turns a loaded test machine into a failure.
  EXPECT_GT(gap, 0.025) << "the stream is faster than 40 Hz";
  EXPECT_LT(gap, 0.150) << "the stream is slower than 6.7 Hz";

  expect_contract_of_every_report();
}

TEST_F(StatusStream, AbsenceOfTheStopSignalIsAssertedBeforeAnythingArrives)
{
  // Nothing is published here at all, so the operator remote is among the things
  // that are not arriving.  wiki/control_architecture.md §6.1: absence of the
  // stop signal is treated as asserted rather than as released, so the very
  // first report a freshly started supervisor publishes is the stop.
  ASSERT_TRUE(spin_until([this]() {return !received_.empty();}));

  const SupervisorStatus & status = received_.front();
  EXPECT_EQ(status.fault, SupervisorStatus::FAULT_ESTOP);
  EXPECT_FALSE(status.deadman_held);
  EXPECT_NE(status.message.find("has arrived"), std::string::npos) << status.message;
  // And it says what it is and is not, so nobody reads FAULT_ESTOP off a panel
  // as a safety function this software performed.
  EXPECT_NE(status.message.find("not protection"), std::string::npos) << status.message;
  expect_contract_of_every_report();
}

TEST_F(StatusStream, AbsenceIsNotHealthBeforeTheFirstPendulumStateArrives)
{
  // The remote arrives and the passive state does not, so the state health of
  // §5.3 is what is left to report.
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

  // Released mid-stream, with nothing else changed: §6.2's check runs on every
  // cycle rather than once at the start of a motion, so the release is caught
  // where it happens and not at the next goal.
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

  // Refused while the condition still holds, with an explanation and never an
  // empty success (ROS 2 Interfaces §1).
  const auto refused = acknowledge(trusted_state(), asserted);
  ASSERT_NE(refused, nullptr);
  EXPECT_FALSE(refused->success);
  EXPECT_FALSE(refused->message.empty());
  EXPECT_NE(refused->message.find("still asserted"), std::string::npos) << refused->message;

  // Released, and the fault survives it: the latch is what §6.1 asks for, so
  // that the software comes back in a defined state rather than resuming.
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

  // The passive state keeps arriving and the remote does not.  A dead GPIO
  // reader must not be indistinguishable from a released button, so this is the
  // stop and not the interlock, and it says which of the two it is.
  ASSERT_TRUE(
    spin_until(
      [this]() {return received_.back().fault == SupervisorStatus::FAULT_ESTOP;},
      [this]() {
        PendulumState state = trusted_state();
        state.header.stamp = observer_->now();
        pendulum_state_->publish(state);
        publish_controllers();
      }))
    << received_.back().message;
  EXPECT_NE(received_.back().message.find("stopped arriving"), std::string::npos)
    << received_.back().message;
  EXPECT_FALSE(received_.back().deadman_held);

  // It is latched too: the signal coming back does not clear it by itself, and
  // the acknowledgement is what does.
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

TEST_F(StatusStream, ThePendulumStateIsCarriedEndToEnd)
{
  ASSERT_TRUE(drive_to(SupervisorStatus::FAULT_NONE, trusted_state()))
    << received_.back().message;

  // `valid == false` raises FAULT_STATE_HEALTH, and the broadcaster's own
  // account of why is carried through rather than restated: the estimator
  // distinguishes six causes behind that flag and this must not flatten them.
  PendulumState degraded = trusted_state();
  degraded.valid = false;
  degraded.status =
    "not to be trusted: the upstream IMU on K5 does not report itself healthy";
  ASSERT_TRUE(drive_to(SupervisorStatus::FAULT_STATE_HEALTH, degraded));
  EXPECT_NE(received_.back().message.find(degraded.status), std::string::npos)
    << received_.back().message;

  expect_contract_of_every_report();
}

TEST_F(StatusStream, ASwingingLoadReachesTheStreamAsATypedCauseThatNamesTheCoordinate)
{
  // wiki/control_architecture.md §5 row 7, from the estimate to the panel. The
  // bound is on the passive *rate* -- the published angle is read out on the
  // nominal hinge axes and carries an uncalibrated constant offset that nothing
  // here has measured, and the rate is composed from the two gyro readings and
  // does not.
  ASSERT_TRUE(drive_to(SupervisorStatus::FAULT_NONE, trusted_state()))
    << received_.back().message;

  PendulumState swinging = trusted_state();
  // The tilt coordinate, past the shipped 0.4 rad/s bound; the tip coordinate
  // still. Both of them at once would pass whether or not the report can tell
  // them apart.
  swinging.velocity[1] = 0.9;
  ASSERT_TRUE(drive_to(SupervisorStatus::FAULT_SWAY, swinging)) << received_.back().message;

  const std::string & message = received_.back().message;
  EXPECT_NE(message.find("theta7_tilt_joint"), std::string::npos) << message;
  EXPECT_EQ(message.find("theta6_tip_joint"), std::string::npos) << message;
  EXPECT_NE(message.find("0.9000"), std::string::npos) << message;
  EXPECT_NE(message.find("0.4000"), std::string::npos) << message;
  // Nothing was acted on, and the report says so: the refusal half of §5 row 7
  // needs an authority over a mode this supervisor does not have yet.
  EXPECT_NE(message.find("Nothing was stopped"), std::string::npos) << message;

  // And a degraded estimate carrying the very same rate is FAULT_STATE_HEALTH
  // and not FAULT_SWAY: a sensor that stopped saying anything is a different
  // fact from a load that is swinging.
  PendulumState degraded = swinging;
  degraded.valid = false;
  degraded.status = "not to be trusted: the upstream IMU on K5 stopped refreshing";
  ASSERT_TRUE(drive_to(SupervisorStatus::FAULT_STATE_HEALTH, degraded))
    << received_.back().message;
  EXPECT_NE(received_.back().message.find(degraded.status), std::string::npos)
    << received_.back().message;

  // The load stops swinging and the fault clears itself, with no acknowledgement:
  // sway is not a latched cause.
  ASSERT_TRUE(drive_to(SupervisorStatus::FAULT_NONE, trusted_state()))
    << received_.back().message;

  expect_contract_of_every_report();
}

TEST_F(StatusStreamWithShortDwell, TheSettledPredicateIsOnEveryReportAndPassesThroughAllThreeStates)
{
  // The predicate the task layer gates a grip action on instead of on a timeout.
  // It has no field of its own on `crane_msgs/SupervisorStatus` -- the message is
  // frozen and widening it is a slice of its own (PRD §15) -- so what carries it
  // today is the clause every report ends in, and this is the whole path from
  // the estimate to that clause.
  //
  // Before anything arrives at all it is *unknowable* and says so. That is the
  // state the whole three-valued design exists for: a two-valued predicate would
  // have to call a supervisor that has never seen an estimate either settled or
  // unsettled, and one of those two answers descends onto a swinging block.
  ASSERT_TRUE(spin_until([this]() {return !received_.empty();}));
  EXPECT_NE(
    received_.front().message.find(crane_supervisor::kSettledClausePrefix), std::string::npos)
    << received_.front().message;
  EXPECT_NE(received_.front().message.find("not known"), std::string::npos)
    << received_.front().message;
  // That report is the latched stop of §6.1, since nothing was arriving -- which
  // is the point: the predicate is on a report whose fault is about something
  // else entirely, and it had to be, because it has no field of its own.
  EXPECT_EQ(received_.front().fault, SupervisorStatus::FAULT_ESTOP);

  // The remote comes back and the latch is acknowledged, so that what follows is
  // about the sway rather than about the stop.
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
  // Which is a statement about the sway and not about the fault: the report it
  // arrived on is a clear one, and the two are separate answers.
  EXPECT_EQ(received_.back().fault, SupervisorStatus::FAULT_NONE) << received_.back().message;

  // The estimate goes bad while the crane is demonstrably still. `settled` does
  // not survive it, and it does not become `not settled` either -- it becomes
  // unknowable, which is a third thing.
  PendulumState degraded = trusted_state();
  degraded.valid = false;
  degraded.status = "not to be trusted: no filter state";
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

TEST_F(StatusStream, TheTrackingErrorIsCarriedEndToEndFromTheControllersOwnState)
{
  // The error is not re-derived from `/joint_states`: the controller that
  // computes it publishes it, and this is the whole path from that publication
  // to the field ROS 2 Interfaces §6 fixes -- max over the actuated joints of
  // the absolute position error, rad or m.
  ASSERT_TRUE(
    drive_to(
      SupervisorStatus::FAULT_NONE, trusted_state(), held_remote(),
      controller_state_with_error("theta3_arm_joint", -0.25, 0.0)))
    << received_.back().message;
  EXPECT_DOUBLE_EQ(received_.back().tracking_error, 0.25);

  // Without a tolerance nothing is compared, and the clear report says which of
  // the two absences that is rather than implying a check nobody made.
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

  // The other two inputs keep arriving and the trajectory controller stops.
  // wiki/control_architecture.md §5.3: an input that stops arriving has a
  // defined consequence, and here the consequence is that the error is reported
  // as unmeasured rather than republished or quietly zeroed into a clear report.
  ASSERT_TRUE(
    spin_until(
      [this]() {
        return received_.back().fault == SupervisorStatus::FAULT_STATE_HEALTH;
      },
      [this]() {
        PendulumState state = trusted_state();
        state.header.stamp = observer_->now();
        pendulum_state_->publish(state);
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

  // And it recovers on its own when the controller comes back: a stale input is
  // not a latch.
  ASSERT_TRUE(drive_to(SupervisorStatus::FAULT_NONE, trusted_state(), held_remote()))
    << received_.back().message;

  expect_contract_of_every_report();
}

TEST_F(StatusStreamWithTolerances, ATrackingExcessIsATypedCauseThatNamesTheAxis)
{
  // wiki/control_architecture.md §5.0, end to end: the behaviour tree gets a
  // constant it can branch on and a message that says which axis, instead of a
  // stall inferred from a deliberately tight goal tolerance.
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
  // The reported number stays what §6 says it is -- the position error, rad or
  // m -- while the verdict above it was made per axis on the velocity error.
  EXPECT_DOUBLE_EQ(received_.back().tracking_error, 0.11);

  // It clears itself when the axis comes back inside: tracking is a fact about
  // the machine right now, not a latch.
  ASSERT_TRUE(drive_to(SupervisorStatus::FAULT_NONE, trusted_state(), held_remote()))
    << received_.back().message;

  expect_contract_of_every_report();
}

TEST_F(StatusStreamWithTolerances, TheSixNumbersComeOutOfAFileInTheSharedShape)
{
  // The tolerance is one number with four consumers and it lives in one file
  // (PRD §6). This asserts the loading path rather than the values: the fixture
  // uses the real file's wildcard node key and its `tracking_tolerance.<joint>.
  // dq_a` keys, and it gives each axis a different number so that a node that
  // paired them by index would be caught here.
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
  // The state every profile is in today: the six rows of
  // `crane_control/config/tracking_tolerance.yaml` are negative and nobody loads
  // it. An axis a long way out raises no FAULT_TRACKING, tracking_error is still
  // published as a measurement, and the configuration notice names the axes and
  // the owner of the missing number.
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
  // The second acceptance criterion of issue 023: the deadlines are per input
  // and configured, not one number.  Four different values go in as four
  // parameters and each has to land in its own slot of the registry -- a node
  // that routed two of them into one input would leave that input judged by
  // somebody else's margin, which is invisible from the outside for as long as
  // both streams stay healthy.
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
  // A deadline of zero would report every sample of that input stale, and one
  // that is not a number would report none of them -- which is the input being
  // exempt from §5.3 by arithmetic rather than by omission.  Neither reaches
  // `decide()`: the declared bounds refuse it, and the node throws out of its
  // constructor before it has a publisher at all.
  rclcpp::NodeOptions refused;
  refused.parameter_overrides({rclcpp::Parameter("controller_health_timeout", 0.0)});
  EXPECT_THROW(
    std::make_shared<crane_supervisor::SupervisorNode>(refused), std::exception);
}

TEST_F(StatusStream, ADeadlineFiresWithNoMessageArrivingAtAllToNoticeIt)
{
  // The fifth acceptance criterion of issue 023.  The freshness sweep runs on
  // the status timer and in no subscription callback, because a deadline
  // evaluated only where a message arrives could never fire on the stream that
  // stopped -- and that is the only stream it exists for.
  ASSERT_TRUE(drive_to(SupervisorStatus::FAULT_NONE, trusted_state(), held_remote()))
    << received_.back().message;
  const std::size_t before = received_.size();

  // Nothing is published from here on: not one message, on any of the four
  // inputs.  The report has to change anyway.
  ASSERT_TRUE(
    spin_until([this]() {return received_.back().fault != SupervisorStatus::FAULT_NONE;}))
    << received_.back().message;
  EXPECT_GT(received_.size(), before);
  // The remote is what is reported, because absence of the stop signal outranks
  // everything else this supervisor watches (§6.1) -- and it says which of the
  // staleness causes fired rather than only that something is stale.
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

  // The remote keeps arriving and the passive state stops.
  // wiki/control_architecture.md §5.3: an input that stops arriving has a
  // defined consequence, and a publisher that died must not be
  // indistinguishable from a healthy one.
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
  // The whole point of the issue.  `crane_velocity_controller` computes the
  // fault every control cycle and, until this stream existed, the only way to
  // read it was an accessor for the S5 harness: prerequisite 4 -- the
  // uncalibrated PZS100 gripper axis -- was reported to nobody.  This is the
  // path from the controller's publication to the constant an operator panel
  // renders at 20 Hz (commissioning_prerequisites §2).
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
  // Which axis, by name.  A panel that says "gripper" tells an operator which
  // calibration to run; a count does not.
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

  // commissioning_prerequisites §2's precedence, end to end: with a health cause
  // holding as well, the health code is what the panel is shown.  The missing
  // calibration will still be missing next cycle; the degraded estimate is what
  // an operator has to act on now.
  PendulumState degraded = trusted_state();
  degraded.valid = false;
  degraded.status = "not to be trusted: the upstream IMU on K5 does not report itself healthy";
  ASSERT_TRUE(
    drive_to(
      SupervisorStatus::FAULT_STATE_HEALTH, degraded, held_remote(),
      tracking_controller_state(), uncommissioned_gripper()))
    << received_.back().message;
  EXPECT_NE(received_.back().message.find(degraded.status), std::string::npos)
    << received_.back().message;

  // It is not a latch either: the loop is the owner of the code, so the report
  // follows what the loop is saying right now.
  ASSERT_TRUE(
    drive_to(
      SupervisorStatus::FAULT_NONE, trusted_state(), held_remote(),
      tracking_controller_state(), healthy_inner_loop()))
    << received_.back().message;

  expect_contract_of_every_report();
}

TEST_F(StatusStream, ARigWithNoHydraulicsReportsNothingAndTheOneSwitchIsTheControllers)
{
  // commissioning_prerequisites §3: only the `hardware` profile reports a
  // missing prerequisite, and a fault raised in every developer's session is a
  // fault nobody reads.  What decides it is `crane_velocity_controller`'s own
  // `profile` parameter -- `crane_control`'s S5 harness asserts both sides of
  // that switch -- and this package adds no second one: it reports the code it
  // was sent, and the per-axis flags are not a switch either.
  ASSERT_TRUE(
    drive_to(
      SupervisorStatus::FAULT_NONE, trusted_state(), held_remote(),
      tracking_controller_state(), fake_profile_inner_loop()))
    << received_.back().message;

  // The calibration is no less missing here: the gripper axis still ran PI only
  // and the flag still says which axis it was.  What is absent is the verdict.
  EXPECT_FALSE(fake_profile_inner_loop().feedforward_applied[kGripperAxis]);
  for (const SupervisorStatus & status : received_) {
    EXPECT_NE(status.fault, SupervisorStatus::FAULT_NOT_COMMISSIONED) << status.message;
  }

  expect_contract_of_every_report();
}

TEST_F(StatusStream, AnInnerLoopThatIsNotReportingIsAFaultRatherThanAClearReport)
{
  // §5.3 applied to the fourth input, and the half of the general staleness
  // policy that cannot wait for issue 023: an uncommissioned axis reported to
  // nobody is the state this stream exists to end, so a report that is not
  // arriving must never read as a loop that is saying nothing is wrong.
  //
  // First the absence that is there from the start.  Everything else arrives.
  ASSERT_TRUE(
    spin_until(
      [this]() {
        return !received_.empty() && received_.back().fault == SupervisorStatus::FAULT_STATE_HEALTH;
      },
      [this]() {
        PendulumState state = trusted_state();
        state.header.stamp = observer_->now();
        pendulum_state_->publish(state);
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

  // Then it stops, with the other three still arriving.  "The controller never
  // came up" and "the controller died" are different things to tell an operator,
  // so the two reports do not read the same.
  ASSERT_TRUE(
    spin_until(
      [this]() {return received_.back().fault == SupervisorStatus::FAULT_STATE_HEALTH;},
      [this]() {
        PendulumState state = trusted_state();
        state.header.stamp = observer_->now();
        pendulum_state_->publish(state);
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

  // And it recovers on its own when the loop comes back: a stale input is not a
  // latch.
  ASSERT_TRUE(drive_to(SupervisorStatus::FAULT_NONE, trusted_state(), held_remote()))
    << received_.back().message;

  expect_contract_of_every_report();
}
