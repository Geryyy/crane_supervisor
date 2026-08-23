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
#include <string>
#include <vector>

#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "crane_msgs/msg/pendulum_state.hpp"
#include "crane_msgs/msg/supervisor_status.hpp"
#include "crane_supervisor/supervisor_node.hpp"
#include "epsilon_crane_msgs/msg/remote_ctrl_states.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace
{

using control_msgs::msg::JointTrajectoryControllerState;
using crane_msgs::msg::PendulumState;
using crane_msgs::msg::SupervisorStatus;
using epsilon_crane_msgs::msg::RemoteCtrlStates;
using std_srvs::srv::Trigger;

/// The six actuated joints of ROS 2 Interfaces §3.2, as the trajectory
/// controller publishes them in `joint_names`.
const std::vector<std::string> kActuatedJoints{
  "theta1_slewing_joint", "theta2_boom_joint", "theta3_arm_joint",
  "q4_big_telescope", "theta8_rotator_joint", "q9_left_rail_joint"};

/// Shorter than the shipped margin so that "the stream stopped" is reachable
/// inside a test budget.  It is an override of the declared parameter, not a
/// second default: the node is built the way a deployment builds it.
constexpr double kTestTimeout = 0.2;

/// How long any one wait may take before the test fails, s.  Generous, because
/// it bounds a failure rather than a success: every wait returns as soon as its
/// predicate holds.
constexpr double kBudget = 10.0;

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
        rclcpp::Parameter("controller_state_timeout", kTestTimeout)});
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

  /// Publish all three inputs once, stamped now.
  void publish(
    const PendulumState & state, const RemoteCtrlStates & remote,
    const JointTrajectoryControllerState & controller)
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
  }

  void publish(const PendulumState & state, const RemoteCtrlStates & remote)
  {
    publish(state, remote, tracking_controller_state());
  }

  /// Spin, publishing every input every pass, until the newest report is `fault`.
  bool drive_to(
    std::uint8_t fault, const PendulumState & state, const RemoteCtrlStates & remote,
    const JointTrajectoryControllerState & controller)
  {
    return spin_until(
      [this, fault]() {
        return !received_.empty() && received_.back().fault == fault;
      },
      [this, &state, &remote, &controller]() {publish(state, remote, controller);});
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
      // Mode arbitration is a later issue; the supervisor never reports a mode
      // it has not verified.
      EXPECT_EQ(status.mode, SupervisorStatus::MODE_IDLE);
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
      }));
  EXPECT_NE(received_.back().message.find("stopped arriving"), std::string::npos)
    << received_.back().message;

  expect_contract_of_every_report();
}
