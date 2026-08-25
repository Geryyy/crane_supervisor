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
// it cannot deliver (PRD user story 54), so this package publishes two status
// streams, serves two services, and holds no publisher on any command topic and
// nothing that writes a setpoint. A static guard test asserts it, because the
// absence is the feature. That the emergency stop is among the inputs changes
// none of it: §6.1 makes the software's relationship to the stop chain
// supplementary and one-directional, so what arrives here is consumed and
// reported and never acted on.
//
// **Since issue 026 there are exceptions, they are counted, and each is named.**
// This node holds two clients on the controller manager -- `list_controllers`,
// to re-derive what is active, and `switch_controller`, to perform a mode change
// somebody asked for on `/crane/set_mode`. ROS 2 Interfaces §5 puts that client
// here deliberately: mode changes go **through the supervisor**, not from the
// behaviour tree, and a stack in which two elements can switch controllers has
// no arbiter at all. The guard test was updated rather than deleted -- it names
// those service types and no others, and every other identifier that could
// reach a command interface is still banned -- so the exception is a line
// somebody wrote down rather than a hole that opened.
//
// **Issue 054 added a third, on the same terms.** ROS 2 Interfaces §4's "One
// command path" ends by saying that which of the two producers is live is the
// supervisor's decision *alone* and that the two never drive at once. Issue 053
// made that a state of `crane_mpc` -- `shadow`, in which it solves at rate and
// publishes nothing at all on `/crane/mpc/horizon`, and `active`, in which the
// same solve reaches the velocity controller -- and left the transition
// deliberately undriven, as a parameter that is *not* read-only precisely so
// that this node could move it. So there is one `rcl_interfaces/SetParameters`
// client, on the node a profile composed the producer under, and the only
// parameter it ever carries is that producer's `mode`.
//
// The exceptions are narrow in a way worth stating: a switch decides *which
// controller holds the claim*, the parameter call decides *which producer may
// publish*, and no message this node sends carries a setpoint, a velocity, a
// trajectory or a goal. Releasing a claim is not a stop either, and the report
// on a `MODE_IDLE` switch says so -- §7.2 measured that a manual controller
// which deactivates leaves its last velocity latched on the interface it just
// released, so a claim released is not a machine at rest.
//
// # Why the three clients live in a callback group of their own
//
// `switch_controller` blocks until the controller manager performs the switch in
// its own control cycle, and the `/crane/set_mode` handler cannot answer until
// it knows the outcome. A client called from inside a service callback on the
// same single-threaded executor would wait for a response that executor is busy
// not processing, which is a deadlock rather than a timeout. The producer's
// parameter call is blocking for the same reason and is made from the same
// handler.
//
// So the three clients are created in a `MutuallyExclusive` callback group that
// is **not** added to the executor with the node, and this node owns a private
// executor that holds only that group. The outer executor never sees the
// clients; the private one is spun in two ways and two only -- `spin_some()` on
// the status cycle, which is non-blocking and is how the polled snapshot is
// taken, and `spin_until_future_complete()` inside the `/crane/set_mode`
// handler, which is the blocking call the handler exists to make. Both run on
// the outer executor's thread, so the private executor is never spun from two
// threads and the node still needs no lock.
//
// # One handover, in order, and why the order is not symmetric
//
// PRD §10 sequences the change into and out of the MPC path, and
// `/crane/set_mode` performs it in that order rather than in a convenient one:
//
//   1. **Freshness first, before anything is asked of the controller manager.**
//      `mpc_horizon_refusal()` is evaluated on the observation this node
//      already holds, *before* `refresh_controller_manager()` is called at all,
//      so a switch into a dead MPC is refused from the mode the machine is
//      already in and never discovered half-way through one (user story 35).
//      `arbitrate_mode()` runs the same function again at precondition 2, so
//      the early answer and the arbitration cannot disagree.
//   2. **Into `MODE_MPC`, the producer goes active *before* the claim moves.**
//      The MPC then publishes horizons while the trajectory controller is still
//      chained, which cannot steal the command, and the velocity controller has
//      a fitted spline in hand at the instant the follower lets go. The other
//      order would leave the inner loop unchained with no horizon for as long as
//      the switch takes, and an unchained loop with no horizon ramps its command
//      to zero and raises `FAULT_REFERENCE_STALE` -- a hole in the handover
//      rather than a seam.
//   3. **Out of `MODE_MPC`, the producer goes back to shadow *after*.** The same
//      argument read the other way.
//
// The producer is therefore active across the union of the two intervals rather
// than across either one exactly, and what settles it afterwards is the mode
// **read back** off the manager rather than the one that was asked for. A switch
// that failed in either direction therefore leaves the producer where the claim
// actually is, not where the request wanted it to be.
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
#include "crane_msgs/msg/solver_health.hpp"
#include "crane_msgs/msg/supervisor_status.hpp"
#include "crane_msgs/msg/sway_settled.hpp"
#include "crane_msgs/msg/velocity_controller_health.hpp"
#include "crane_msgs/srv/set_mode.hpp"
#include "crane_supervisor/supervisor_core.hpp"
#include "epsilon_crane_msgs/msg/remote_ctrl_states.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
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
/// The second stream of §4, and the reason it is a stream rather than a field.
/**
 * The settled predicate of wiki/control_architecture.md §5 row 7 is what a
 * behaviour tree gates a grip action on instead of on a timeout, and
 * `crane_msgs/SupervisorStatus` has no field for it. Widening that message is a
 * **field add**, which PRD §15 makes a slice of its own; a new message is
 * additive and has no ceremony, so the predicate rides on one of its own.
 *
 * It is published from the same `update()` call as the status, off the same
 * decision, with the same `header.stamp` -- so the two streams cannot describe
 * two different cycles, and the sentence on one is the clause the other ends in.
 */
inline constexpr char kSwaySettledTopic[] = "/crane/sway_settled";
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

/// The horizon producer's own status stream, and the evidence PRD §10 step 2
/// is verified against.
/**
 * A contract name of ROS 2 Interfaces §4 and therefore absolute, like every
 * other name here. It is **not** an alias of a policy row, because this stream
 * is deliberately not an `Input` -- `HorizonReport` in the core says why, and
 * why the evidence is the solve rather than `/crane/mpc/horizon` itself.
 */
inline constexpr char kSolverHealthTopic[] = "/crane/mpc/solver_health";

/// How this node moves the horizon producer between shadow and active.
/**
 * The parameter's name and its two values are `crane_mpc`'s contract and not
 * this node's setting, so they are constants; the *node* the producer was
 * loaded under is a profile's decision and is `SupervisorConfig::mpc_node`.
 * `crane_mpc`'s own declaration validates the string against exactly these two,
 * so a third value would be refused by the producer rather than accepted and
 * quietly ignored.
 *
 * The service name is composed from the configured node name at construction
 * and is the one ROS name in this package that is not written out whole. Every
 * other one is a cross-node contract of ROS 2 Interfaces §1 and is absolute;
 * this one is a node's own parameter service, and which node that is is exactly
 * what a profile decides.
 */
inline constexpr char kSetParametersSuffix[] = "/set_parameters";
inline constexpr char kHorizonProducerModeParameter[] = "mode";
inline constexpr char kHorizonProducerActive[] = "active";
inline constexpr char kHorizonProducerShadow[] = "shadow";

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

/// How long the horizon producer may take to answer a `set_parameters`, s.
/**
 * A budget and not a contract, for the reason the three above are. It is the
 * same number as `kControllerManagerCallBudget` and for the same reason: the
 * call is a local service round trip that normally returns in a millisecond,
 * and the number bounds a failure rather than a success. A producer that will
 * not answer must not hold the one service an operator uses to get out of a
 * mode, and the answer to a call that times out is not "assume it worked" --
 * the report says the claim and the producer disagree, which is the state ROS 2
 * Interfaces §4 forbids and therefore the state worth naming.
 */
inline constexpr double kProducerModeBudget = 0.5;

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

  /// Puts the horizon producer into `active` or into `shadow`, and reports it.
  /**
   * One `rcl_interfaces/SetParameters` call carrying one parameter. It never
   * returns "assume it worked": a producer that refused, that answered nothing,
   * or that nothing is serving is reported in `ProducerSwitch::account`, so a
   * claim that moved while the producer did not is named on the response rather
   * than inferred later from a horizon that never arrives.
   *
   * A deployment that names no producer -- `mpc_node` empty, which `validate()`
   * allows only when `MODE_MPC` has no controllers either -- is not a failure
   * and makes no call: `attempted` stays false and the report says nothing
   * about a producer this composition does not have.
   */
  ProducerSwitch set_horizon_producer_mode(bool active);

  /// The stop half of wiki/control_architecture.md §5 row 3, once it is owed.
  /**
   * Runs on the status cycle when `solver_handback()` says so, and does what a
   * `/crane/set_mode(MODE_IDLE)` request would do: read the machine, build the
   * plan through `arbitrate_mode()` so that every precondition and every
   * exclusion is the same one an operator's request goes through, issue the one
   * strict `switch_controller` call, read the mode back, and put the producer
   * into `shadow` **after** the claim has moved -- PRD §10's order out of
   * `MODE_MPC`, unchanged because it is the same handover.
   *
   * It runs at the **end** of the cycle, after both streams have gone out. The
   * calls it makes block this node's status timer for as long as a mode switch
   * does, and the report an operator is owed is the one that says why -- so the
   * cycle that discovers the escalation publishes `FAULT_SOLVER` and its account
   * first, and the mode reads `MODE_IDLE` on the next one.
   *
   * It is attempted **once** per episode, not retried every 50 ms: a switch the
   * controller manager refused will be refused again next cycle, and retrying it
   * in a loop would take the 20 Hz stream down with it. A failure is logged at
   * `ERROR` with the manager's own account, and `FAULT_SOLVER` goes on standing
   * on the stream, which is the signal the task layer branches on (§5.0).
   * `handback_attempted_` is cleared on the first cycle `solver_handback()` no
   * longer asks for one, so a later episode gets its own attempt.
   *
   * Returns the account, or an empty string when nothing was attempted.
   */
  std::string hand_back_from_mpc(const SolverHandback & handback);

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
  /// The newest message on `/crane/mpc/solver_health`, or null before the first
  /// one. Held for the reason the four above are, and read in three places: PRD
  /// §10 step 2's precondition on a switch into `MODE_MPC`, the `FAULT_SOLVER`
  /// merge on the status stream, and `solver_handback()`.
  /**
   * It is **not** an `Input`, and that is about its *absence* rather than its
   * content: an `Input` is a stream whose silence raises a fault, and an
   * optimizer that is quiet while the machine is in `MODE_FOLLOW` is the
   * ordinary state of this stack rather than a defect. Its `fault` field is
   * merged all the same, unrenumbered, exactly as `VelocityControllerHealth`'s
   * is (ROS 2 Interfaces §4) -- what scopes the merge is the mode, not the
   * stream: `resolve()` reports the code only while `MODE_MPC` is the live
   * command path.
   */
  crane_msgs::msg::SolverHealth::ConstSharedPtr solver_health_;
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
  /// Whether the hand-back of this episode has already been attempted. Cleared
  /// on the first cycle `solver_handback()` no longer asks for one, so a second
  /// escalation gets a second attempt and a standing one gets exactly one.
  bool handback_attempted_{false};

  rclcpp::Subscription<crane_msgs::msg::PendulumState>::SharedPtr pendulum_state_subscription_;
  rclcpp::Subscription<epsilon_crane_msgs::msg::RemoteCtrlStates>::SharedPtr
    remote_ctrl_subscription_;
  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr
    controller_state_subscription_;
  rclcpp::Subscription<crane_msgs::msg::VelocityControllerHealth>::SharedPtr
    controller_health_subscription_;
  /// The horizon producer's own stream, and the **only** subscription in this
  /// package that does not go through `subscribe()`.
  /**
   * That helper takes an `Input`, and an `Input` is by definition a stream whose
   * absence raises a fault on the status. This one's must not, so it cannot be
   * an enumerator and therefore cannot go through the helper. Both static
   * guards were updated to *name* the exception rather than relaxed --
   * `test_every_input_has_a_deadline.py` and `test_no_command_path.py` each
   * pin the two call sites and the two types, so a third subscription is still
   * a line somebody has to write in a test.
   */
  rclcpp::Subscription<crane_msgs::msg::SolverHealth>::SharedPtr solver_health_subscription_;
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
  /// The settled predicate, as a field rather than as a sentence.
  /**
   * The second publisher this node holds and the last one it may hold without
   * `test_no_command_path.py` gaining a line: the guard names every publisher
   * one at a time, because a second publisher is how a status node becomes a
   * command node. This one carries a verdict, two rates and a sentence, and
   * nothing a controller could act on.
   */
  rclcpp::Publisher<crane_msgs::msg::SwaySettled>::SharedPtr sway_settled_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_fault_service_;
  rclcpp::Service<crane_msgs::srv::SetMode>::SharedPtr set_mode_service_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  /// The exceptions to "no path to a motion command", kept apart from
  /// everything else this node holds.
  /**
   * The group is created with `automatically_add_to_executor_with_node` false,
   * so `rclcpp::spin(node)` in `crane_supervisor_main.cpp` never sees it and the
   * private executor below is the only thing that ever serves these three
   * clients. That is what lets the `/crane/set_mode` handler make a blocking
   * call without deadlocking the executor it is running on.
   *
   * It is named for the *shape* of what it holds rather than for the controller
   * manager, because since issue 054 one of the three is on `crane_mpc`: what
   * these have in common is that each is a blocking call on another process
   * made from inside a service callback.
   */
  rclcpp::CallbackGroup::SharedPtr remote_call_group_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr remote_call_executor_;
  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr list_controllers_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_;
  /// The third client, on the node `mpc_node` names. Null when a deployment
  /// composes no horizon producer, which is a composition with no `MODE_MPC`
  /// rather than a misconfiguration.
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr set_horizon_producer_mode_;
};

}  // namespace crane_supervisor

#endif  // CRANE_SUPERVISOR__SUPERVISOR_NODE_HPP_
