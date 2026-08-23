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
// Everything below runs in one node with the default callback group, so the
// subscriptions, the status timer and the `/crane/clear_fault` service are
// mutually exclusive on any executor. The latch is a plain member for exactly
// that reason, and `crane_supervisor_main.cpp` spins the single-threaded
// executor that makes it true.

#ifndef CRANE_SUPERVISOR__SUPERVISOR_NODE_HPP_
#define CRANE_SUPERVISOR__SUPERVISOR_NODE_HPP_

#include <chrono>
#include <cstdint>

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
inline constexpr char kPendulumStateTopic[] = "/crane/pendulum_state";
inline constexpr char kRemoteCtrlStatesTopic[] = "/crane/remote_ctrl_states";
inline constexpr char kClearFaultService[] = "/crane/clear_fault";
inline constexpr double kStatusRate = 20.0;

/// Where the trajectory controller's own state publication is read from.
/**
 * The trajectory controller publishes it on its *private* `~/controller_state`,
 * which resolves under whatever the deployment named that controller
 * (`trajectory_controller_a2b` in the FOLLOW profile). §1 forbids relying on
 * that: cross-node contracts are absolute and live under `/crane/...`, and a
 * subscriber that reached into another node's namespace would break the first
 * time a second trajectory controller was loaded.
 *
 * So this node subscribes to the contract name and to nothing else, exactly as
 * it does for the remote -- whose producer, `gpio_controller`, likewise
 * publishes on a private name. Lining the two up is a remap and belongs to
 * whoever composes the profile. ROS 2 Interfaces §4 does not yet carry a row for
 * this stream; the name follows §1's rule and the row is owed.
 */
inline constexpr char kControllerStateTopic[] = "/crane/controller_state";

/// Where the inner velocity loop's own health is read from.
/**
 * A contract name of ROS 2 Interfaces §4 and, unlike the two above, one whose
 * producer already publishes on it: `crane_velocity_controller` creates the
 * publisher with this absolute name itself, so no profile has to remap anything
 * for this input to arrive.
 *
 * It is the one input this supervisor could not possibly re-derive. Which axes
 * the active tool has an identified valve map for is not in `/joint_states`, not
 * in the trajectory controller's state and not anywhere else on the graph — the
 * controller is the only element that knows, which is why the fault stays where
 * it is computed and travels as a message.
 */
inline constexpr char kControllerHealthTopic[] = "/crane/velocity_controller/health";

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

  /// What one cycle observed, from what the node is holding right now.
  /**
   * Shared by the status timer and the acknowledgement, so the two judge the
   * emergency stop from the same sample rather than from two reads a callback
   * apart.
   */
  [[nodiscard]] SupervisorInput observe() const;

  SupervisorConfig config_;
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
