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
#include <memory>
#include <string>
#include <vector>

#include "crane_msgs/msg/pendulum_state.hpp"
#include "crane_msgs/msg/supervisor_status.hpp"
#include "crane_supervisor/supervisor_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

using crane_msgs::msg::PendulumState;
using crane_msgs::msg::SupervisorStatus;

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
    options.parameter_overrides({rclcpp::Parameter("pendulum_state_timeout", kTestTimeout)});
    supervisor_ = std::make_shared<crane_supervisor::SupervisorNode>(options);

    observer_ = std::make_shared<rclcpp::Node>("crane_supervisor_stream_observer");
    const rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    status_ = observer_->create_subscription<SupervisorStatus>(
      crane_supervisor::kStatusTopic, qos,
      [this](SupervisorStatus::ConstSharedPtr message) {received_.push_back(*message);});
    pendulum_state_ = observer_->create_publisher<PendulumState>(
      crane_supervisor::kPendulumStateTopic, qos);

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

  bool drive_to(std::uint8_t fault, const PendulumState & sample)
  {
    return spin_until(
      [this, fault]() {
        return !received_.empty() && received_.back().fault == fault;
      },
      [this, &sample]() {
        PendulumState fresh = sample;
        fresh.header.stamp = observer_->now();
        pendulum_state_->publish(fresh);
      });
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

TEST_F(StatusStream, AbsenceIsNotHealthBeforeTheFirstPendulumStateArrives)
{
  ASSERT_TRUE(spin_until([this]() {return !received_.empty();}));

  const SupervisorStatus & status = received_.front();
  EXPECT_EQ(status.fault, SupervisorStatus::FAULT_STATE_HEALTH);
  EXPECT_NE(status.fault, SupervisorStatus::FAULT_NONE);
  EXPECT_FALSE(status.message.empty());
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

  // Nothing publishes from here on.  wiki/control_architecture.md §5.3: an
  // input that stops arriving has a defined consequence, and a publisher that
  // died must not be indistinguishable from a healthy one.
  ASSERT_TRUE(
    spin_until(
      [this]() {return received_.back().fault == SupervisorStatus::FAULT_STATE_HEALTH;}));
  EXPECT_NE(received_.back().message.find("stopped arriving"), std::string::npos)
    << received_.back().message;

  expect_contract_of_every_report();
}
