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
// stream, serves one acknowledgement, and holds nothing else -- no publisher on
// a command topic, no controller-manager client, nothing that can deactivate a
// controller. A static guard test asserts it, because the absence is the
// feature. That the emergency stop is among the inputs changes none of it: §6.1
// makes the software's relationship to the stop chain supplementary and
// one-directional, so what arrives here is consumed and reported and never acted
// on.
//
// **Every subscription this node holds goes through `subscribe()`, which takes
// an `Input`.** That is where §5.3's rule -- no input may stop arriving without
// a defined consequence -- stops being a checklist. A subscription that named no
// input could not be written; the topic comes off that input's policy row rather
// than off the call site; the input's freshness deadline has to be configured or
// `validate()` refuses the node; and the constructor refuses to finish while any
// enumerator of `Input` is left without a subscription behind it. Adding a fifth
// input is therefore four compile-or-start-time failures, not four things to
// remember.
//
// Everything below runs in one node with the default callback group, so the
// subscriptions, the status timer and the `/crane/clear_fault` service are
// mutually exclusive on any executor. The latch is a plain member for exactly
// that reason, and `crane_supervisor_main.cpp` spins the single-threaded
// executor that makes it true. The freshness sweep is on the timer and not in
// the callbacks, deliberately: a deadline evaluated only where a message arrives
// could not fire on the stream that stopped.

#ifndef CRANE_SUPERVISOR__SUPERVISOR_NODE_HPP_
#define CRANE_SUPERVISOR__SUPERVISOR_NODE_HPP_

#include <array>
#include <chrono>
#include <cstdint>
#include <utility>

#include <memory>

#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "crane_msgs/msg/pendulum_state.hpp"
#include "crane_msgs/msg/supervisor_status.hpp"
#include "crane_msgs/msg/velocity_controller_health.hpp"
#include "crane_supervisor/supervisor_core.hpp"
#include "epsilon_crane_msgs/msg/remote_ctrl_states.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

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
inline constexpr char kClearFaultService[] = "/crane/clear_fault";
inline constexpr double kStatusRate = 20.0;

/// The four input contract names, off the policy rows that already carry them.
/**
 * Aliases and not second copies: an input's topic is written once, in
 * `kInputPolicies`, beside the deadline it is judged against and the fault its
 * absence raises. A name that lived here as well could be changed in one place
 * and not the other, and the stream a report names would stop being the stream
 * the node subscribes to.
 *
 * Two of the four have producers that do not publish on the contract name --
 * the retained `gpio_controller` and the trajectory controller both publish on
 * a *private* name that resolves under whatever the profile called them. §1
 * forbids relying on that, so this node subscribes to the contract name and to
 * nothing else, and lining the two up is a remap that belongs to whoever
 * composes the profile.
 */
inline constexpr const char * kPendulumStateTopic = policy_of(Input::PendulumState).topic;
inline constexpr const char * kRemoteCtrlStatesTopic = policy_of(Input::RemoteCtrl).topic;
inline constexpr const char * kControllerStateTopic = policy_of(Input::ControllerState).topic;
inline constexpr const char * kControllerHealthTopic = policy_of(Input::ControllerHealth).topic;

/// The QoS of every streamed contract this node touches.
/**
 * Reliable, depth 1, volatile -- ROS 2 Interfaces §1 sets the category by the
 * consumer, and all of these are state a controller or a panel acts on. It is
 * the same profile `crane_msgs`' own ROS contract test asserts, so the two agree
 * by construction rather than by inspection. One function and not one per
 * endpoint, so an input cannot arrive on a quietly different profile.
 */
[[nodiscard]] inline rclcpp::QoS contract_qos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
}

/// The configured deadman, out of the twelve booleans the message carries.
/**
 * The one place in this package that turns a button *number* into a button
 * *field*. `epsilon_crane_msgs/RemoteCtrlStates` names them one at a time rather
 * than carrying an array, so the mapping is a switch and cannot be an index; a
 * number outside 1..12 selects nothing and reads as released, which is why
 * `validate()` refuses one before the node is built.
 */
[[nodiscard]] bool deadman_of(
  const epsilon_crane_msgs::msg::RemoteCtrlStates & message, int button);

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

  /// The one place a subscription is created in this package.
  /**
   * It takes an `Input`, so a subscription with no freshness deadline behind it
   * cannot be written: the enumerator is what indexes the deadline
   * `validate()` insists on and the policy row the report is composed from. The
   * topic comes off that row rather than off the call site, and the call marks
   * the input claimed, so the constructor can refuse to finish while any input
   * is left unsubscribed.
   */
  template<typename MessageT, typename CallbackT>
  typename rclcpp::Subscription<MessageT>::SharedPtr subscribe(Input input, CallbackT && callback)
  {
    claimed_[index_of(input)] = true;
    return create_subscription<MessageT>(
      policy_of(input).topic, contract_qos(), std::forward<CallbackT>(callback));
  }

  /// What one cycle observed, from what the node is holding right now.
  /**
   * Shared by the status timer and the acknowledgement, so the two judge the
   * emergency stop from the same sample rather than from two reads a callback
   * apart.
   *
   * The transport half of every input -- whether anything has arrived and how
   * old the newest one is -- is filled in a loop over the registry, from one
   * reading of the clock, so no input can be aged by a rule of its own.
   */
  [[nodiscard]] SupervisorInput observe() const;

  SupervisorConfig config_;
  /// Which inputs a subscription was created for, indexed by `Input`. Checked
  /// once, at construction: an enumerator with no subscription behind it is an
  /// input the freshness sweep would report as never having arrived, for ever,
  /// which is a defect in this node dressed up as a fault in the graph.
  std::array<bool, kInputCount> claimed_{};
  /// The newest message on `/crane/pendulum_state`, or null before the first
  /// one. Held rather than consumed: the tracer's whole point is that the gap
  /// between the newest sample and now is itself a signal.
  crane_msgs::msg::PendulumState::ConstSharedPtr pendulum_state_;
  /// The newest message on `/crane/remote_ctrl_states`, or null before the first
  /// one. Held for the same reason, and for one more: its absence is what §6.1
  /// reads as an asserted stop.
  epsilon_crane_msgs::msg::RemoteCtrlStates::ConstSharedPtr remote_ctrl_;
  /// The newest message on `/crane/controller_state`, or null before the first
  /// one. Held for the same reason as the other two: a trajectory controller
  /// that stopped publishing is not a crane that is tracking perfectly.
  control_msgs::msg::JointTrajectoryControllerState::ConstSharedPtr controller_state_;
  /// The newest message on `/crane/velocity_controller/health`, or null before
  /// the first one. Held for the same reason as the other three: an inner loop
  /// that stopped publishing is not an inner loop with nothing to report, and
  /// an uncommissioned axis reported to nobody is the state this input exists
  /// to end.
  crane_msgs::msg::VelocityControllerHealth::ConstSharedPtr controller_health_;
  /// The emergency-stop latch, carried from one decision into the next. Raised
  /// by `decide()`, lowered only by an acknowledged `/crane/clear_fault`.
  bool estop_latched_{false};
  /// The sway dwell, carried from one decision into the next the same way.
  /**
   * The second and last piece of history this node holds. It is a plain member
   * for the reason the latch is: everything here runs in one node with the
   * default callback group, so the subscriptions, the status timer and the
   * service are mutually exclusive on the single-threaded executor
   * `crane_supervisor_main.cpp` spins.
   */
  SwayState sway_state_;
  /// What the last published report said, so a transition is logged once
  /// instead of the same line twenty times a second.
  Fault reported_fault_{Fault::None};
  bool ever_reported_{false};

  rclcpp::Subscription<crane_msgs::msg::PendulumState>::SharedPtr pendulum_state_subscription_;
  rclcpp::Subscription<epsilon_crane_msgs::msg::RemoteCtrlStates>::SharedPtr
    remote_ctrl_subscription_;
  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr
    controller_state_subscription_;
  rclcpp::Subscription<crane_msgs::msg::VelocityControllerHealth>::SharedPtr
    controller_health_subscription_;
  rclcpp::Publisher<crane_msgs::msg::SupervisorStatus>::SharedPtr status_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_fault_service_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace crane_supervisor

#endif  // CRANE_SUPERVISOR__SUPERVISOR_NODE_HPP_
