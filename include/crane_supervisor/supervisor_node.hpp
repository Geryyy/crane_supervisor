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
// stream, serves two services, and holds no publisher on any command topic and
// nothing that writes a setpoint. A static guard test asserts it, because the
// absence is the feature. That the emergency stop is among the inputs changes
// none of it: §6.1 makes the software's relationship to the stop chain
// supplementary and one-directional, so what arrives here is consumed and
// reported and never acted on.
//
// **Since issue 026 there is exactly one exception, and it is named.** This node
// holds two clients on the controller manager -- `list_controllers`, to re-derive
// what is active, and `switch_controller`, to perform a mode change somebody
// asked for on `/crane/set_mode`. ROS 2 Interfaces §5 puts that client here
// deliberately: mode changes go **through the supervisor**, not from the
// behaviour tree, and a stack in which two elements can switch controllers has
// no arbiter at all. The guard test was updated rather than deleted -- it names
// those two service types and no others, and every other identifier that could
// reach a command interface is still banned -- so the exception is a line
// somebody wrote down rather than a hole that opened.
//
// The exception is narrow in a way worth stating: a switch decides *which
// controller holds the claim*, and no message this node sends carries a
// setpoint, a velocity, a trajectory or a goal. Releasing a claim is not a stop
// either, and the report on a `MODE_IDLE` switch says so -- §7.2 measured that a
// manual controller which deactivates leaves its last velocity latched on the
// interface it just released, so a claim released is not a machine at rest.
//
// # Why the two clients live in a callback group of their own
//
// `switch_controller` blocks until the controller manager performs the switch in
// its own control cycle, and the `/crane/set_mode` handler cannot answer until
// it knows the outcome. A client called from inside a service callback on the
// same single-threaded executor would wait for a response that executor is busy
// not processing, which is a deadlock rather than a timeout.
//
// So the two clients are created in a `MutuallyExclusive` callback group that is
// **not** added to the executor with the node, and this node owns a private
// executor that holds only that group. The outer executor never sees the
// clients; the private one is spun in two ways and two only -- `spin_some()` on
// the status cycle, which is non-blocking and is how the polled snapshot is
// taken, and `spin_until_future_complete()` inside the `/crane/set_mode`
// handler, which is the blocking call the handler exists to make. Both run on
// the outer executor's thread, so the private executor is never spun from two
// threads and the node still needs no lock.
//
// # The mode is polled, and the poll is not an `Input`
//
// `mode` on the status stream is re-derived from the controller manager on every
// cycle rather than remembered from the last successful request. The poll is
// asynchronous: the request goes out on the private group and the answer is
// taken on a later cycle, because a status stream that stalled on a slow or
// absent manager would be a 20 Hz contract with a service call in the middle of
// it. `ControllerManagerReport` in the core says why this is not a fifth
// `Input`, and what §5.3's rule looks like for something polled.
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
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <memory>

#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "crane_msgs/msg/pendulum_state.hpp"
#include "crane_msgs/msg/supervisor_status.hpp"
#include "crane_msgs/msg/velocity_controller_health.hpp"
#include "crane_msgs/srv/set_mode.hpp"
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
inline constexpr char kSetModeService[] = "/crane/set_mode";
inline constexpr double kStatusRate = 20.0;

/// The two controller-manager services this node calls, and the only ones.
/**
 * ROS 2 Interfaces §5's row: mode changes go **through the supervisor**, not
 * from the behaviour tree, which makes this the one element in the stack that
 * may hold a switch client at all. Absolute names, because §1 makes cross-node
 * contracts absolute and nothing here may rely on sharing a namespace with the
 * manager.
 */
inline constexpr char kListControllersService[] = "/controller_manager/list_controllers";
inline constexpr char kSwitchControllerService[] = "/controller_manager/switch_controller";

/// How long a *blocking* call on the controller manager may take, s.
/**
 * Budgets and not contracts, so they are constants with a reason rather than
 * parameters: neither is a margin a deployment tunes, and a mode change that
 * hung would hang the one service an operator uses to get out of a mode.
 *
 * `kControllerManagerCallBudget` bounds the `list_controllers` call the
 * `/crane/set_mode` handler makes to read the machine before it decides. It is
 * generous because it bounds a failure rather than a success: the answer is a
 * local service round trip and normally returns in a millisecond.
 *
 * `kSwitchBudget` bounds the wait on `switch_controller` itself, and it is
 * longer than `kSwitchControllerTimeout` on purpose: the inner number is what
 * the *manager* waits for its own control cycle, and the outer one has to
 * outlast it so that a manager which reports a timeout is heard rather than
 * abandoned. A caller that abandoned the call would leave a switch in flight
 * with nobody reading its outcome, which is precisely the half-way state PRD §10
 * step 2 is about.
 *
 * **They are as tight as they are because a mode change costs the status
 * stream.** The `/crane/set_mode` handler and the status timer share this
 * node's default callback group, so while a switch is in flight the 20 Hz
 * stream does not tick and no subscription callback runs. A switch on a healthy
 * manager takes a control cycle -- milliseconds, far inside every input's
 * freshness deadline. A manager that hangs can hold the handler for
 * `kControllerManagerCallBudget + kSwitchBudget + kControllerManagerCallBudget`
 * -- four seconds -- and the next status report will then say inputs went stale,
 * which is *true* (this supervisor was not looking) but names the producer
 * rather than the pause. The README records that, and what would fix it: the
 * arbitration in a callback group of its own on a multi-threaded executor,
 * which costs a lock over the held messages and the latch that are plain
 * members today.
 */
inline constexpr double kControllerManagerCallBudget = 0.5;
inline constexpr double kSwitchControllerTimeout = 1.0;
inline constexpr double kSwitchBudget = 3.0;

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
   * Shared by the status timer, the acknowledgement and the mode arbitration,
   * so all three judge the emergency stop from the same sample rather than from
   * reads a callback apart.
   *
   * The transport half of every input -- whether anything has arrived and how
   * old the newest one is -- is filled in a loop over the registry, from one
   * reading of the clock, so no input can be aged by a rule of its own.
   */
  [[nodiscard]] SupervisorInput observe() const;

  /// The controller manager's newest answer, aged on this node's own clock.
  /**
   * Split out of `observe()` because the `/crane/set_mode` handler reads it
   * twice -- once to check every precondition before anything is deactivated,
   * and once afterwards to report what is *actually* active rather than what was
   * asked for.
   */
  [[nodiscard]] ControllerManagerReport observe_controller_manager() const;

  /// One non-blocking step of the polled snapshot. Called on the status cycle.
  /**
   * Sends a `list_controllers` request when none is in flight and takes the
   * answer when one has come back, and does neither for longer than it takes to
   * ask. The 20 Hz contract of ROS 2 Interfaces §2 is why: a status stream with
   * a synchronous service call in it is a status stream that stops whenever the
   * manager does, which is one of the situations it exists to report.
   *
   * A request that outlives its own deadline is dropped rather than waited on,
   * so a manager that never answers leaves the snapshot ageing -- which is
   * exactly what makes the mode read as *not known* -- instead of leaving one
   * request in flight for ever and never asking again.
   */
  void poll_controller_manager();

  /// A blocking `list_controllers`, for the two points a decision needs truth.
  /**
   * Returns false when the manager did not answer inside `timeout`, which leaves
   * the snapshot as it was: an old answer is aged and reported as old, never
   * replaced by a guess. Any asynchronous poll in flight is abandoned first, so
   * that a stale response cannot land after this one and overwrite a newer
   * answer with an older one.
   */
  bool refresh_controller_manager(double timeout);

  /// One `/crane/set_mode` call: arbitrate, then switch, then read back.
  void set_mode(
    const crane_msgs::srv::SetMode::Request::SharedPtr request,
    crane_msgs::srv::SetMode::Response::SharedPtr response);

  /// One `switch_controller` call. Returns whether the manager performed it and
  /// carries the manager's own account of why not in `account`.
  bool switch_controllers(
    const std::vector<std::string> & activate, const std::vector<std::string> & deactivate,
    std::string & account);

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
  /// The newest `list_controllers` answer, or null before the first one.
  /**
   * Held rather than consumed, for the reason every input above is: the gap
   * between the newest answer and now is itself a signal, and here it is the
   * one that decides whether the mode is known at all.
   */
  controller_manager_msgs::srv::ListControllers::Response::SharedPtr controllers_;
  /// When that answer arrived, on this node's own clock. It is the *arrival*
  /// and not a stamp on the wire: `ListControllers` carries no header, so there
  /// is nothing to age against except when this node heard it.
  rclcpp::Time controllers_at_{0, 0, RCL_ROS_TIME};

  /// One asynchronous `list_controllers` request in flight, or none.
  struct PendingSnapshot
  {
    rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedFuture future;
    std::int64_t request_id{0};
    rclcpp::Time sent_at{0, 0, RCL_ROS_TIME};
  };
  std::optional<PendingSnapshot> pending_snapshot_;

  rclcpp::Publisher<crane_msgs::msg::SupervisorStatus>::SharedPtr status_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_fault_service_;
  rclcpp::Service<crane_msgs::srv::SetMode>::SharedPtr set_mode_service_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  /// The one exception to "no path to a motion command", kept apart from
  /// everything else this node holds.
  /**
   * The group is created with `automatically_add_to_executor_with_node` false,
   * so `rclcpp::spin(node)` in `crane_supervisor_main.cpp` never sees it and the
   * private executor below is the only thing that ever serves these two
   * clients. That is what lets the `/crane/set_mode` handler make a blocking
   * call without deadlocking the executor it is running on.
   */
  rclcpp::CallbackGroup::SharedPtr controller_manager_group_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr controller_manager_executor_;
  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr list_controllers_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_;
};

}  // namespace crane_supervisor

#endif  // CRANE_SUPERVISOR__SUPERVISOR_NODE_HPP_
