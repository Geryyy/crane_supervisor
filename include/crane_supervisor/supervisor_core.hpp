// What the supervisor decides, separated from everything that made the decision
// possible. Plain data in, plain data out: no node handle, no message type, no
// clock, no topic (wiki/implementation/style_guide.md §3). The node above it
// owns the I/O and the timing and nothing else, so every branch below is
// reachable from a unit test with no ROS runtime in the process.
//
// This slice is status and mode only (PRD §2, slice 3). The supervisor's
// distinguishing capability -- an independent stop path -- is withheld until
// commissioning prerequisite 3 is verified, so nothing here returns anything
// that acts: there is no stop, no ramp, no deactivation and no command
// (wiki/control_architecture.md §5.2). What there is, is a *cause* on every
// report (PRD user story 53).
//
// Four inputs are carried end to end, and `Input` below is the registry of
// them: `crane_msgs/PendulumState` on `/crane/pendulum_state`,
// `epsilon_crane_msgs/RemoteCtrlStates` on `/crane/remote_ctrl_states`, the
// trajectory controller's own `control_msgs/JointTrajectoryControllerState`,
// and `crane_msgs/VelocityControllerHealth` on
// `/crane/velocity_controller/health`.
//
// # The freshness policy, and why it is a registry rather than four checks
//
// wiki/control_architecture.md §5.3 names the failure mode this stack is most
// exposed to: not a wrong value, an **absent** one. The rule is that no input
// may simply stop arriving without a defined consequence, and this supervisor is
// the only element that sees every input that reaches it by topic, so it is
// where the rule is enforced for those.
//
// A rule enforced by four hand-written checks is a rule that lapses the first
// time somebody adds a fifth subscription. So the policy is structural:
//
//   * `Input` is the enum of every subscription this package holds, and
//     `kInputCount` is derived from it rather than written beside it.
//   * `kInputPolicies` has one row per enumerator -- a `static_assert` refuses a
//     table that is shorter, longer or out of order -- and the row carries the
//     topic, the type, the producer, what is lost while the input is missing and
//     the `Fault` constant its absence raises.
//   * `SupervisorConfig::freshness_deadline` and `SupervisorInput::streams` are
//     both sized by `kInputCount`, so an input added to the enum gets a slot in
//     each for free and a deadline it must be given.
//   * **There is no default deadline.** A `SupervisorConfig` that has not been
//     given one does not `validate()`, and the node throws rather than
//     publishing. An input added with no margin is therefore a node that refuses
//     to start, not an input nobody watches.
//   * The node creates every subscription through one helper that takes an
//     `Input`, so a subscription that named no input -- and therefore had no
//     deadline -- cannot be written, and the constructor refuses to finish while
//     any enumerator is left unsubscribed.
//
// The deadlines are **per input and configured**, never one global number: the
// passive state at 100 Hz and the remote at 20 Hz do not go stale at the same
// age. The numbers and their derivations live in
// `src/crane_supervisor_parameters.yaml`, once, so this file states no margin of
// its own.
//
// The sweep runs on the status cycle and costs `kInputCount` comparisons. That
// is the point rather than an optimisation: a deadline that were only evaluated
// in a subscription callback could not fire, because the case it exists for is
// the one where no callback runs again.
//
// # The three staleness causes, kept apart
//
// §5.3's "Stale state" row used to describe detection as a timestamp age and
// nothing else. Issues 017 and 019 made that wrong: the passive state now says
// three different things about itself, and they are different in kind --
//
//   flag     the per-IMU hardware-health signal the driver exports. A unit whose
//            driver says it is degraded is degraded whatever its numbers look
//            like, and the numbers are exactly what look fine.
//   age      how long ago the sample was taken, against a margin.
//   refresh  whether the sample still moves. A unit that stopped delivering
//            while its driver reports it healthy and the loop keeps cycling
//            raises neither of the other two.
//
// `Staleness` below keeps them apart at this level, and the split is not
// symmetric, deliberately:
//
//   * **age** is the one this supervisor measures itself, from `header.stamp`
//     against the input's own deadline. Its three values separate an input that
//     never connected, one that was arriving and stopped, and one whose stamp
//     cannot be placed in time at all.
//   * **flag** is the producer's, and arrives as `ProducerUnhealthy`. The
//     producer's own account of the cause is carried through verbatim rather
//     than restated.
//   * **refresh** is the producer's too, and there is no topic-level version of
//     it here on purpose. `pendulum_state_broadcaster` can ask whether seven
//     doubles moved because differenced-gyro noise is thirty times the
//     quantiser; a supervisor asking the same question of
//     `epsilon_crane_msgs/RemoteCtrlStates` would be asking whether twelve
//     booleans moved, and an operator holding a button produces bit-identical
//     payloads for minutes. So the cause reaches this package inside the flag's
//     carried string, and the report says which of the three fired.
//
// # The rest of the inputs
//
// The fourth input is the inner velocity loop's verdict on itself, and it is
// carried rather than re-derived for a reason that is not a preference: **only
// the controller knows which axes the active tool has a valve calibration
// for.** Nothing in `/joint_states` or in the trajectory controller's state says
// it, so a supervisor that tried to work it out would be guessing at the one
// fact the report exists to deliver. What arrives is a
// `crane_msgs/SupervisorStatus` fault constant, so this package merges a code
// rather than translating one, and the per-axis feedforward flags with the joint
// names beside them, so a panel can say which axis is uncommissioned rather than
// that something is.
//
// Where it sits in the order below is `commissioning_prerequisites.md` §2's
// rule and not this file's: when the controller's code and a cause this
// supervisor watches both hold, the **health** code is reported in preference to
// the **commissioning** code. A missing calibration will still be missing next
// cycle; a state that just went stale is the one an operator has to act on now.
//
// The third is what turns the tracking duty of wiki/control_architecture.md §5
// row 1 into a *typed cause* instead of the inferred abort §5.0 describes. The
// behaviour tree's ownership of the retry is correct and untouched; what changes
// is the signal it acts on. The error is not re-derived here from
// `/joint_states`: the controller that computes it publishes it, per joint,
// every control cycle.
//
// Two quantities come off that one message and they are not interchangeable:
//
//   position error  `error.positions`, rad on four axes and m on two. This is
//                   the number `tracking_error` carries, because
//                   wiki/implementation/ros2_interfaces.md §6 fixes that field
//                   as the max over the actuated joints in rad or m. A max
//                   taken across two units is an indicator and not a
//                   comparison, and it is reported as one.
//   velocity error  `error.velocities`, rad/s and m/s. This is what
//                   `FAULT_TRACKING` is decided on, per axis, because the one
//                   tolerance this workspace defines --
//                   `crane_control/config/tracking_tolerance.yaml`, key `dq_a`
//                   -- is a *velocity*-tracking tolerance (PRD §6, gate (ii-b)).
//                   Comparing a position error against it would be rad against
//                   rad/s, which is the hidden mixed unit this file exists to
//                   avoid.
//
// **The tolerance does not exist yet, and no default is invented for it.** All
// six values in that file are negative and the file says in its own header that
// a value which is not finite and positive is not a tolerance. An axis without
// one raises no tracking fault; the supervisor says so once at configuration,
// names the axis and the owner of the missing number, and goes on reporting
// `tracking_error` as a measurement -- the same way the velocity controller's
// seam clamp goes transparent and warns.
//
// The remote carries the emergency stop and the operator deadman, and what this
// package does with them is **diagnosis and recovery, not protection**
// (§6.1). The stop chain is hardware and PLC and acts whether or not this
// process is running; consuming the signal is how the software comes back in a
// defined state instead of resuming from whatever it was doing when the machine
// stopped underneath it. Nothing here stops, ramps, deactivates or commands
// anything, and no safety argument may take credit for it.
//
// Three rules shape the two decisions below, and each is §6.1's or §6.2's and
// not this file's invention:
//
//   asserted-on-absence  a signal that is not arriving is treated as asserted.
//                        A dead GPIO reader, a crashed driver and a released
//                        button must not look alike (§5.3, §6.1), so each gets
//                        its own account of itself and only one of them is a
//                        released button.
//   latched              once the stop is asserted the fault survives the signal
//                        returning to released, and only an explicit
//                        acknowledgement on `/crane/clear_fault` clears it. It
//                        is the *only* latched cause here: every other input
//                        clears its own fault the moment it starts arriving
//                        again inside its deadline, and `InputPolicy::latches`
//                        is where that asymmetry is written down.
//   continuous           the deadman is re-checked every cycle rather than once
//                        at the start of a motion (§6.2).

#ifndef CRANE_SUPERVISOR__SUPERVISOR_CORE_HPP_
#define CRANE_SUPERVISOR__SUPERVISOR_CORE_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace crane_supervisor
{

/// Control mode, numbered as `crane_msgs/SupervisorStatus` numbers it.
/**
 * The values are the wire's, so the adapter is a cast and not a table. The
 * package is ROS-free and cannot include the message to say so; the contract
 * test does, and asserts each pair.
 *
 * Only `Idle` is ever reported in this slice. Mode arbitration and
 * `/crane/set_mode` are a later issue, and a supervisor that announced a mode
 * it had not verified would be advertising an arbitration it does not perform.
 */
enum class Mode : std::uint8_t
{
  Idle = 0,
  Manual = 1,
  Follow = 2,
  Mpc = 3,
};

/// Why the stack is not to be trusted, numbered as the message numbers it.
/**
 * Every constant of `crane_msgs/SupervisorStatus` is listed, including the ones
 * no cause in this slice can raise. Leaving them out would make the enum a
 * subset that has to be widened by later issues, and a value's meaning would
 * then depend on which issue had landed.
 */
enum class Fault : std::uint8_t
{
  None = 0,
  Tracking = 1,
  WorkingCell = 2,
  Solver = 3,
  Sway = 4,
  StateHealth = 5,
  ReferenceStale = 6,
  EStop = 7,
  Interlock = 8,
  NotCommissioned = 9,
};

/// Every input this supervisor subscribes to. The registry, not a list beside
/// one.
/**
 * `SupervisorConfig::freshness_deadline`, `SupervisorInput::streams` and
 * `kInputPolicies` are all sized by this enum, and the node creates every
 * subscription through one helper that takes an `Input`. Adding an enumerator
 * therefore widens all three in one edit and leaves a deadline that must be
 * configured before the node will start; adding a subscription without adding
 * an enumerator does not compile.
 *
 * The values are this package's own and reach no wire, so they are not written
 * out: `Count` follows the last input and is what `kInputCount` reads, so an
 * input added above it widens everything without a second number to keep in
 * step.
 */
enum class Input : std::size_t
{
  PendulumState,
  RemoteCtrl,
  ControllerState,
  ControllerHealth,
  /// Not an input. The bound of everything that is per input.
  Count,
};

/// How many inputs there are, from the enum rather than beside it.
inline constexpr std::size_t kInputCount = static_cast<std::size_t>(Input::Count);

/// One enumerator as its own array index.
[[nodiscard]] inline constexpr std::size_t index_of(Input input) noexcept
{
  return static_cast<std::size_t>(input);
}

/// Which staleness cause fired on one input this cycle.
/**
 * The three causes wiki/control_architecture.md §5.3 now lists -- flag, age and
 * refresh -- do not map one to one onto this enum, and the file header says why.
 * Three of the five values are **age**, which is the cause this supervisor
 * measures itself; `ProducerUnhealthy` is the **flag**, carried from the
 * producer; and **refresh** has no topic-level form here and reaches this
 * package inside the flag's own status string.
 *
 * `NeverArrived` and `StoppedArriving` are two values and not one because "the
 * publisher never came up" and "the publisher died" are different things for an
 * operator to chase. Both are faults, and neither is `Fresh`.
 */
enum class Staleness : std::uint8_t
{
  /// Arriving, inside its deadline, and the producer says nothing is wrong.
  Fresh = 0,
  /// Nothing has arrived on this input since the supervisor started.
  NeverArrived = 1,
  /// The newest sample is older than this input's deadline. It was arriving.
  StoppedArriving = 2,
  /// The newest sample is stamped further in this node's future than the
  /// deadline allows, so its age is not a measurement of anything.
  StampAhead = 3,
  /// It arrived, in time, and the producer marks its own output unusable.
  ProducerUnhealthy = 4,
};

/// One input, and what its absence costs. One row per `Input`.
/**
 * The row is what makes a report say *which input* and *which cause* without a
 * message written per input per cause: the generator below composes the
 * sentence, and this table is the only place a new input has to be described.
 */
struct InputPolicy
{
  /// The enumerator this row is for. Checked against its own index, so a table
  /// out of enum order does not compile.
  Input input;
  /// The input in an operator's words, used mid-sentence.
  const char * label;
  /// The cross-node contract name of ROS 2 Interfaces §4. Absolute: §1 forbids
  /// relying on a namespace, so this is what the node subscribes to.
  const char * topic;
  /// The message type, as an operator would grep for it.
  const char * type;
  /// The node or controller that is supposed to be publishing it.
  const char * producer;
  /// What the report says this stream did when it stops. Per input, because a
  /// broadcaster that stopped arriving and a controller that stopped publishing
  /// its own state are two different sentences to put in front of an operator.
  const char * stopped;
  /// What is lost, and what must not be inferred, while this input is missing.
  const char * consequence;
  /// What to check.
  const char * advice;
  /// The `crane_msgs/SupervisorStatus` constant a report raises while this input
  /// is not fresh. Distinct per input on purpose (PRD user story 52): collapsing
  /// an absent `em_stop` and a stale passive state into one code is the defect
  /// the general policy exists to prevent.
  Fault fault;
  /// Whether a fault this input raises survives the condition going away.
  /**
   * True for the emergency stop alone: §6.1 asks for the software to come back
   * in a defined state rather than resuming from whatever it was doing, so the
   * stop latches and `/crane/clear_fault` is the only thing that lowers it.
   * Every other input recovers on its own the moment it arrives again inside its
   * deadline -- a stale input is not a latch, and a supervisor that made one out
   * of it would need an acknowledgement for every hiccup on the graph.
   */
  bool latches;
};

/// The one row per input, in enum order.
inline constexpr std::array<InputPolicy, kInputCount> kInputPolicies{{
  {Input::PendulumState, "the passive joint state", "/crane/pendulum_state",
    "crane_msgs/PendulumState", "pendulum_state_broadcaster", "stopped arriving",
    "The passive joint state counts as unavailable, so nothing that closes on it may be trusted: "
    "a stale joint velocity makes the inner loop's integrator wind up against a value that is no "
    "longer true.",
    "Check that pendulum_state_broadcaster is loaded and active on the controller manager.",
    Fault::StateHealth, false},
  {Input::RemoteCtrl, "the operator remote", "/crane/remote_ctrl_states",
    "epsilon_crane_msgs/RemoteCtrlStates", "gpio_controller", "stopped arriving",
    "Absence of the stop signal is read as asserted rather than as released, and the deadman is "
    "reported as not held: a dead GPIO reader, a crashed driver and a released button must not "
    "look alike.",
    "Check that gpio_controller is loaded and active on the controller manager and that its "
    "remote_ctrl_states output reaches the contract name.",
    Fault::EStop, true},
  {Input::ControllerState, "the trajectory controller's own state", "/crane/controller_state",
    "control_msgs/JointTrajectoryControllerState", "the trajectory controller",
    "stopped publishing its own state",
    "The tracking error is not being measured, and tracking_error carries 0.0 as the absence of a "
    "measurement rather than as perfect tracking: a controller that died must not look like a "
    "crane that is tracking perfectly.",
    "Check the controller manager's cycle, whether the trajectory controller is still active, and "
    "that its private controller_state output reaches the contract name.",
    Fault::StateHealth, false},
  {Input::ControllerHealth, "the inner velocity loop's health report",
    "/crane/velocity_controller/health", "crane_msgs/VelocityControllerHealth",
    "crane_velocity_controller", "stopped reporting its own health",
    "The inner loop's verdict on itself is unread rather than clear, and a commissioning "
    "prerequisite it is raising would be reaching nobody -- which is the state that stream exists "
    "to end.",
    "Check that crane_velocity_controller is loaded and active on the controller manager.",
    Fault::StateHealth, false},
}};

/// The row of one input.
[[nodiscard]] inline constexpr const InputPolicy & policy_of(Input input) noexcept
{
  return kInputPolicies[index_of(input)];
}

/// True when every row sits at the index of the enumerator it names.
[[nodiscard]] inline constexpr bool policies_are_in_enum_order() noexcept
{
  for (std::size_t i = 0; i < kInputCount; ++i) {
    if (index_of(kInputPolicies[i].input) != i) {
      return false;
    }
  }
  return true;
}

static_assert(
  policies_are_in_enum_order(),
  "kInputPolicies has one row per Input, in enum order: an input added to the enum needs a row "
  "describing what its absence costs, and a row that names a different input than its index would "
  "report the wrong topic for the wrong stream");

/// The buttons `epsilon_crane_msgs/RemoteCtrlStates` carries, by the numbers the
/// message names them with. Twelve, and the wire has no thirteenth.
inline constexpr int kFirstButton = 1;
inline constexpr int kLastButton = 12;

/// One axis's row of `crane_control/config/tracking_tolerance.yaml`.
/**
 * The supervisor holds no tolerance of its own. This struct is the shape the
 * numbers arrive in, not a place to keep them: the file is the one owner (PRD
 * §6 -- one number, three consumers, now four), and a value restated in this
 * package's configuration is a value that can drift away from the clamp's and
 * the MPC's.
 */
struct AxisTolerance
{
  /// URDF joint name, as ROS 2 Interfaces §3.1 spells it and as the trajectory
  /// controller reports it in `joint_names`. Axes are paired by name and never
  /// by index: the controller's order is its own.
  std::string joint;
  /// The per-axis velocity-tracking tolerance, rad/s on the four angles and m/s
  /// on the two lengths. NaN when the file has not been loaded, negative when it
  /// has and the number does not exist yet. Both mean the same thing --
  /// `is_tolerance()` is false, this axis raises no tracking fault, and the
  /// supervisor says which axis and whose number is missing.
  double dq_a{std::numeric_limits<double>::quiet_NaN()};
};

/// The margins the decision is made against. All SI, all named.
struct SupervisorConfig
{
  /// The freshness deadline of every input, s, indexed by `Input`.
  /**
   * Per input and never one global number, because the inputs do not go stale
   * at the same age: `/crane/pendulum_state` is published from inside the
   * manager's 100 Hz cycle and `/crane/remote_ctrl_states` is a 20 Hz contract,
   * so the healthy worst-case age of one is not the healthy worst-case age of
   * the other. A single margin would either report the fast stream late or the
   * slow one falsely.
   *
   * **Value-initialised to zero, and zero is not a deadline.** `validate()`
   * refuses it, and the node throws rather than publishing, so an input added to
   * `Input` and given no margin is a node that refuses to start instead of an
   * input nobody watches. There is deliberately no plausible default here: the
   * numbers and the derivation of each live once, in
   * `src/crane_supervisor_parameters.yaml`, and a struct default beside them
   * would be a second place for them to drift.
   *
   * None of them is a safety timing requirement. By the time the remote's
   * deadline expires the hardware chain has long since acted; what the numbers
   * decide is how far the *diagnosis* lags the event, against how easily a slow
   * link raises a false one.
   */
  std::array<double, kInputCount> freshness_deadline{};

  /// The six rows of `crane_control/config/tracking_tolerance.yaml`, in the
  /// order the actuated joints are configured in.
  /**
   * Empty until the node loads them, and an empty list is not an error: it is
   * the state a deployment that has not been given the file is honestly in, and
   * it raises no tracking fault at all.
   */
  std::vector<AxisTolerance> tracking_tolerance;

  /// Which `epsilon_crane_msgs/RemoteCtrlStates` button is the deadman, 1..12.
  /**
   * Read from the retained stack, not chosen here and not inferred from the
   * number. `epsilon_crane_behavior_tree`'s approval gate tests
   * `RemoteCtrlStates::button12` -- `src/plugins/condition/check_user_approval.cpp`
   * and `include/.../plugins/decorator/get_user_approval.hpp`, whose own comment
   * says "button12 has to be pressed and held". The manual-control TUI defines
   * `KEYCODE_BUTTON12` as `0x7A`, the `z` key
   * (`timber_crane_tui/src/remote_ctrl.cpp`), which is the key
   * wiki/control_architecture.md §6.2 names as the simulated deadman, and the
   * simulation's remote publisher sets `button12` to approve.
   *
   * `gpio_controller` copies the twelve booleans off the state interfaces in
   * order and names none of them: it fixes the wire index, and the approval gate
   * fixes the meaning. It is configuration and not a constant because it is a
   * property of a remote's wiring, which the architecture does not own.
   */
  int deadman_button{12};

  /// The deadline of one input, s.
  [[nodiscard]] double deadline(Input input) const noexcept
  {
    return freshness_deadline[index_of(input)];
  }

  /// The deadline of one input, s, to be written by whoever loads the margins.
  [[nodiscard]] double & deadline(Input input) noexcept
  {
    return freshness_deadline[index_of(input)];
  }
};

/// The transport half of one input: what the node saw of the *topic*.
/**
 * Held identically for every input, in an array `Input` sizes, because the
 * freshness rule is identical for every input. The payload halves below differ
 * per message and stay separate; this half does not, and a per-input copy of it
 * is how one of them ends up exempt.
 */
struct StreamReport
{
  /// False until the first message arrives. It does not become false again: a
  /// stream that stopped is caught by `age`, and the two are different causes
  /// with different messages, because "never connected" and "died" are
  /// different things for an operator to chase.
  bool received{false};
  /// `now - header.stamp` of the newest message, s. Negative when the stamp is
  /// in this node's future, which is a clock fault rather than a fresh sample.
  double age{0.0};
};

/// What `/crane/pendulum_state` said about itself this cycle.
/**
 * The payload half. `pendulum_state_broadcaster` separates six causes behind
 * `valid == false` -- among them the sample that stopped refreshing behind a
 * header that keeps moving -- and says which in `status`, so the string is
 * carried through rather than restated.
 */
struct PendulumStateReport
{
  /// `crane_msgs/PendulumState.valid`.
  bool valid{false};
  /// `crane_msgs/PendulumState.status`, verbatim. The broadcaster's account of
  /// why, carried rather than restated.
  std::string status;
};

/// What `/crane/remote_ctrl_states` said this cycle.
/**
 * The node resolves which of the twelve booleans the configured deadman is; the
 * message carries no such field. Everything else is copied off the message.
 */
struct RemoteCtrlReport
{
  /// The configured deadman button of the newest message. False while nothing is
  /// arriving: a button nobody reported is not a button somebody is holding.
  bool deadman_held{false};
  /// `epsilon_crane_msgs/RemoteCtrlStates.em_stop` of the newest message.
  bool em_stop{false};
};

/// One actuated axis, as the trajectory controller reported it this cycle.
/**
 * Copied off `control_msgs/JointTrajectoryControllerState`, one entry per name
 * in its `joint_names`, in that message's own order. Nothing is derived here:
 * the controller that computes the error is the one that publishes it.
 */
struct AxisError
{
  /// The URDF joint name the controller published this row under.
  std::string joint;
  /// `error.positions`, rad or m. Feeds `tracking_error` and nothing else.
  double position_error{0.0};
  /// `error.velocities`, rad/s or m/s. What `FAULT_TRACKING` is decided on.
  double velocity_error{0.0};
  /// False when the controller published no velocity error for this axis.
  /**
   * The trajectory controller fills `error.velocities` only when it holds a
   * velocity state interface *and* a velocity or effort command interface. The
   * FOLLOW profile gives it both, but a profile that did not would leave the
   * field empty, and an empty field read as a zero error is a crane that tracks
   * perfectly by construction. So the absence is carried rather than defaulted:
   * an axis whose velocity error was not reported is not compared.
   */
  bool velocity_error_reported{false};
};

/// What the trajectory controller's state publication carried this cycle.
struct ControllerStateReport
{
  /// One entry per joint the newest message named.
  std::vector<AxisError> axes;
};

/// What `/crane/velocity_controller/health` carried this cycle.
/**
 * The inner loop's own answer, carried and not restated.
 */
struct ControllerHealthReport
{
  /// `crane_msgs/VelocityControllerHealth.fault`, in the numbering the wire and
  /// this enum share. The inner loop can raise `StateHealth`, `ReferenceStale`
  /// and `NotCommissioned`; a fourth would mean the controller's own
  /// `static_assert` block and the frozen message have drifted apart, so it is
  /// carried through rather than flattened into one of the three.
  Fault fault{Fault::None};
  /// The URDF joints the newest report marked `feedforward_applied == false` --
  /// the axes that ran PI only. Named and not counted: a panel that says
  /// "q9_left_rail_joint" tells an operator which calibration to run, and one
  /// that says "one axis" does not.
  std::vector<std::string> feedforward_free_joints;
};

/// One axis whose velocity error is outside its own tolerance.
struct TrackingBreach
{
  std::string joint;
  /// Signed, rad/s or m/s. Signed because which way an axis is lagging is the
  /// first thing anyone looking at a tracking fault wants to know.
  double velocity_error{0.0};
  /// The `dq_a` this axis was compared against, same unit.
  double tolerance{0.0};
};

/// Everything one decision is made from.
struct SupervisorInput
{
  /// The transport half of every input, indexed by `Input`. Sized by the enum,
  /// so an input added to it is swept for freshness without a line being
  /// written anywhere -- and, having no deadline yet, cannot reach a running
  /// node until one is configured.
  std::array<StreamReport, kInputCount> streams{};

  PendulumStateReport pendulum_state;
  RemoteCtrlReport remote_ctrl;
  ControllerStateReport controller_state;
  ControllerHealthReport controller_health;

  /// The emergency-stop latch as the previous cycle left it.
  /**
   * The latch is state and the core is a pure function, so the state is
   * threaded through rather than held: the node carries `estop_latched` out of
   * one decision and back into the next. That keeps every latched case
   * reachable from a test by writing one bool, instead of by replaying the
   * history that produced it.
   */
  bool estop_latched{false};

  /// The transport half of one input.
  [[nodiscard]] StreamReport & stream(Input input) noexcept {return streams[index_of(input)];}

  /// The transport half of one input.
  [[nodiscard]] const StreamReport & stream(Input input) const noexcept
  {
    return streams[index_of(input)];
  }
};

/// One decision, in the shape `crane_msgs/SupervisorStatus` carries it.
struct SupervisorDecision
{
  Mode mode{Mode::Idle};
  Fault fault{Fault::StateHealth};
  /// Max over the actuated joints of the absolute *position* error, rad or m,
  /// as ROS 2 Interfaces §6 fixes the field.
  /**
   * An indicator, not a comparison. Four of the six axes are angles and two are
   * lengths, so the max is taken across two units and the number that wins says
   * only "this is the largest single-axis deviation anywhere on the crane". The
   * comparison that decides `FAULT_TRACKING` is per axis, against that axis's
   * own tolerance, and it is made on the velocity error rather than on this one
   * -- see the file header.
   *
   * Zero when the trajectory controller's state is not arriving. That case is a
   * fault in its own right, so the zero is never left to read as perfect
   * tracking.
   */
  double tracking_error{0.0};
  /// Not computed in this slice: the virtual working cell of §5.1 needs forward
  /// kinematics that `crane_model`'s production backend does not have yet.
  /// False is the conservative reading -- "not confirmed inside" -- and is the
  /// only value that is not a claim.
  bool inside_working_cell{false};
  /// The configured deadman button, as the newest remote message reported it,
  /// and false whenever that message is not arriving. Reported on every status
  /// whatever the fault is, so a released deadman stays visible even in the
  /// cycles where a more consequential cause owns `fault`.
  bool deadman_held{false};
  /// The emergency-stop latch as this cycle leaves it. The node carries it into
  /// the next call, and `/crane/clear_fault` is the only thing that lowers it.
  bool estop_latched{false};
  /// Why, in words an operator can act on. Never empty (PRD user story 53).
  std::string message;
};

/// The answer to one `/crane/clear_fault` call.
/**
 * `std_srvs/Trigger`'s two fields, in the core's own types. ROS 2 Interfaces §1
 * allows no empty success: a refusal says which condition still holds, and a
 * clear says what it cleared.
 */
struct ClearFaultOutcome
{
  bool cleared{false};
  std::string message;
};

/// Adopts and checks the margins. Returns false and says why, once, on failure.
/**
 * Every input's deadline is checked, in a loop over `Input` rather than against
 * a list written here, so an input added to the enum is checked without this
 * function being touched -- and, until it is given a margin, refused.
 *
 * A missing tracking tolerance is deliberately *not* a failure. It is the state
 * the workspace is actually in, it is reported rather than refused, and a
 * supervisor that declined to start over it would take the whole status stream
 * down for a number that only one of its duties needs. A missing *deadline* is
 * the opposite: it is the one thing that would make an input silently exempt
 * from §5.3.
 */
[[nodiscard]] bool validate(const SupervisorConfig & config, std::string & reason);

/// The freshness of one input, from its deadline and what arrived.
/**
 * Three comparisons and no state. The margin is symmetric: a sample is fresh
 * while its age lies within `[-deadline, deadline]`, so ordinary clock jitter
 * between two hosts is not a fault and a stamp further ahead than the margin --
 * whose age is therefore not a measurement of anything -- is.
 */
[[nodiscard]] Staleness freshness_of(double deadline, const StreamReport & stream) noexcept;

/// The freshness of every input, in one pass.
/**
 * `kInputCount` comparisons, no allocation, no state, and nothing that waits for
 * a message: a deadline evaluated only in a subscription callback could never
 * fire, because the case it exists for is the one where no callback runs again
 * (wiki/control_architecture.md §5.3). The node calls this on the status cycle.
 */
[[nodiscard]] std::array<Staleness, kInputCount> freshness(
  const SupervisorConfig & config, const SupervisorInput & input) noexcept;

/// Which input, and which staleness cause, in words an operator can act on.
/**
 * The whole message for one stale input, composed from that input's policy row:
 * it names the input, the topic, the type, the age it was judged by, the
 * deadline it was judged against, which of the causes fired, and what must not
 * be inferred while it holds. `carried` is the producer's own status string and
 * is read only for `ProducerUnhealthy`.
 */
[[nodiscard]] std::string staleness_message(
  const SupervisorConfig & config, Input input, Staleness cause, const StreamReport & stream,
  const std::string & carried);

/// True when a number is a tolerance: finite and positive.
/**
 * `tracking_tolerance.yaml` states the rule in its own header and writes the
 * absent numbers out as `-1.0` rather than omitting them, so that the gap is
 * visible in the file that will one day carry the value. NaN -- the parameter
 * default, meaning the file was never loaded -- fails the same test.
 */
[[nodiscard]] bool is_tolerance(double dq_a) noexcept;

/// What to say once, at configuration, about the axes that have no tolerance.
/**
 * Empty when every configured axis has one. Otherwise it names the axes, says
 * that they raise no tracking fault, and names the owner of the missing number
 * -- gate (ii-b) or the identification campaign, both human-only (PRD §14).
 * Nothing in this package may fill the gap with a plausible default.
 */
[[nodiscard]] std::string tracking_tolerance_notice(const SupervisorConfig & config);

/// The reduction ROS 2 Interfaces §6 fixes: max over the actuated joints of the
/// absolute position error, rad or m. Zero over an empty report.
[[nodiscard]] double max_position_error(const ControllerStateReport & report) noexcept;

/// The comparison, per axis and in the tolerance's own unit. Worst first.
/**
 * Paired by joint name, never by index: `joint_names` is the controller's
 * ordering and the configuration's is its own. An axis with no tolerance, or
 * one the controller reported no velocity error for, is not compared -- and is
 * therefore absent from the result rather than present with a zero.
 *
 * "Worst" is by how far past its own tolerance an axis is, not by the raw
 * error: over six axes in two units the raw magnitudes are not comparable, and
 * the ratio is.
 */
[[nodiscard]] std::vector<TrackingBreach> tracking_breaches(
  const SupervisorConfig & config, const ControllerStateReport & report);

/// One status cycle. Total: every input produces a decision with a cause.
[[nodiscard]] SupervisorDecision decide(
  const SupervisorConfig & config, const SupervisorInput & input);

/// One acknowledgement of the latched emergency stop, against the newest input.
/**
 * Judged from the same observation `decide()` is judged from, deliberately: an
 * acknowledgement that consulted a second, older view of the stop signal could
 * clear a latch whose condition the very next status cycle still sees.
 *
 * The latch is lowered only when the condition behind it is gone -- `em_stop`
 * released *and* the signal arriving. Lowering it while the condition holds
 * would be a clear the next cycle immediately undoes, reported to the operator
 * as a success.
 */
[[nodiscard]] ClearFaultOutcome clear_fault(
  const SupervisorConfig & config, const SupervisorInput & input);

}  // namespace crane_supervisor

#endif  // CRANE_SUPERVISOR__SUPERVISOR_CORE_HPP_
