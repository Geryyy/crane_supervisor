// The adapter. It owns the I/O and the timing of the supervisor and nothing
// else: it subscribes, it measures how old what it holds is, it hands plain data
// to `decide()` and it publishes the answer. Every branch that decides anything
// lives in `supervisor_core.hpp`, which no ROS type reaches
// (wiki/implementation/style_guide.md §3).
//
// It has no path to a motion command, and that is deliberate rather than
// unfinished. wiki/control_architecture.md §5.2 gives the supervisor a stop path
// that does not run through the active controller, and
// wiki/implementation/commissioning_prerequisites.md row 3 makes that path
// conditional on a verification only a safety reviewer on the machine can
// perform. Until it is performed the system must not advertise a safety function
// it cannot deliver (PRD user story 54), so this package publishes a status
// stream and holds nothing else -- no publisher on a command topic, no
// controller-manager client, nothing that can deactivate a controller. A static
// guard test asserts it, because the absence is the feature.

#ifndef CRANE_SUPERVISOR__SUPERVISOR_NODE_HPP_
#define CRANE_SUPERVISOR__SUPERVISOR_NODE_HPP_

#include <chrono>
#include <cstdint>

#include "crane_msgs/msg/pendulum_state.hpp"
#include "crane_msgs/msg/supervisor_status.hpp"
#include "crane_supervisor/supervisor_core.hpp"
#include "rclcpp/rclcpp.hpp"

namespace crane_supervisor
{

/// The contract names and the contract rate of ROS 2 Interfaces §2 and §4.
/**
 * Constants and not parameters. A cross-node contract a deployment can rename
 * or re-rate from a YAML file is not a contract, and §1 makes these names
 * absolute for exactly that reason: nothing here relies on a namespace, and
 * nothing above may rely on a remapping.
 */
inline constexpr char kStatusTopic[] = "/crane/supervisor/status";
inline constexpr char kPendulumStateTopic[] = "/crane/pendulum_state";
inline constexpr double kStatusRate = 20.0;

/// The node of ROS 2 Interfaces §2, at the rate that table gives it.
class SupervisorNode : public rclcpp::Node
{
public:
  explicit SupervisorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// One status cycle: sample what arrived, age it, decide, publish.
  /**
   * Public because it is what the timer calls and what the stream test steps.
   * A test that had to wait on a timer to observe one decision would be timing
   * its own scheduler rather than the contract.
   */
  void update();

  [[nodiscard]] const SupervisorConfig & config() const noexcept {return config_;}

private:
  /// The status period, from `kStatusRate`. Named so the timer and the test read
  /// the same number.
  static std::chrono::nanoseconds status_period();

  SupervisorConfig config_;
  /// The newest message on `/crane/pendulum_state`, or null before the first
  /// one. Held rather than consumed: the tracer's whole point is that the gap
  /// between the newest sample and now is itself a signal.
  crane_msgs::msg::PendulumState::ConstSharedPtr pendulum_state_;
  /// What the last published report said, so a transition is logged once
  /// instead of the same line twenty times a second.
  Fault reported_fault_{Fault::None};
  bool ever_reported_{false};

  rclcpp::Subscription<crane_msgs::msg::PendulumState>::SharedPtr pendulum_state_subscription_;
  rclcpp::Publisher<crane_msgs::msg::SupervisorStatus>::SharedPtr status_publisher_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace crane_supervisor

#endif  // CRANE_SUPERVISOR__SUPERVISOR_NODE_HPP_
