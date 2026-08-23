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

#include "crane_msgs/msg/pendulum_state.hpp"
#include "crane_msgs/msg/supervisor_status.hpp"
#include "crane_supervisor/supervisor_node.hpp"
#include "epsilon_crane_msgs/msg/remote_ctrl_states.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace
{

using crane_msgs::msg::PendulumState;
using crane_msgs::msg::SupervisorStatus;
using epsilon_crane_msgs::msg::RemoteCtrlStates;
using std_srvs::srv::Trigger;

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
  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
      {rclcpp::Parameter("pendulum_state_timeout", kTestTimeout),
        rclcpp::Parameter("remote_ctrl_timeout", kTestTimeout)});
    supervisor_ = std::make_shared<crane_supervisor::SupervisorNode>(options);

    observer_ = std::make_shared<rclcpp::Node>("crane_supervisor_stream_observer");
    const rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    status_ = observer_->create_subscription<SupervisorStatus>(
      crane_supervisor::kStatusTopic, qos,
      [this](SupervisorStatus::ConstSharedPtr message) {received_.push_back(*message);});
    pendulum_state_ = observer_->create_publisher<PendulumState>(
      crane_supervisor::kPendulumStateTopic, qos);
    remote_ctrl_ = observer_->create_publisher<RemoteCtrlStates>(
      crane_supervisor::kRemoteCtrlStatesTopic, qos);
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

  /// Publish both inputs once, stamped now.
  void publish(const PendulumState & state, const RemoteCtrlStates & remote)
  {
    PendulumState fresh_state = state;
    fresh_state.header.stamp = observer_->now();
    pendulum_state_->publish(fresh_state);

    RemoteCtrlStates fresh_remote = remote;
    fresh_remote.header.stamp = observer_->now();
    remote_ctrl_->publish(fresh_remote);
  }

  /// Spin, publishing both inputs every pass, until the newest report is `fault`.
  bool drive_to(std::uint8_t fault, const PendulumState & state, const RemoteCtrlStates & remote)
  {
    return spin_until(
      [this, fault]() {
        return !received_.empty() && received_.back().fault == fault;
      },
      [this, &state, &remote]() {publish(state, remote);});
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
  rclcpp::Client<Trigger>::SharedPtr clear_fault_;
  std::vector<SupervisorStatus> received_;
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
