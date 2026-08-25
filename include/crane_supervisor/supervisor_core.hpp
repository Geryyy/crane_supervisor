// What the supervisor decides, separated from everything that made the decision
// possible. Plain data in, plain data out: no node handle, no message type, no
// clock, no topic (wiki/implementation/style_guide.md §3). The node above it
// owns the I/O and the timing and nothing else, so every branch below is
// reachable from a unit test with no ROS runtime in the process.
//
// This slice is status and mode only (PRD §2, slice 3), and since issue 026
// this file carries both halves of it. The supervisor's distinguishing
// capability -- an independent stop path -- is still withheld until
// commissioning prerequisite 3 is verified, so nothing here stops, ramps or
// commands anything on its own (wiki/control_architecture.md §5.2). What there
// is, is a *cause* on every report (PRD user story 53), and one narrow
// authority: which controller holds the claim.
//
// # The one authority, and what it is not
//
// §7 says manual and autonomous are already mutually exclusive **by resource
// claim** -- both want the same `velocity` command interfaces, so ros2_control
// refuses to activate one while the other holds it -- and calls that a sound
// foundation to keep. What this package adds is not a second exclusion
// mechanism but a *decision* in front of the existing one: `arbitrate_mode()`
// answers which mode may hold the claim and says why, so that a refusal is a
// sentence an operator reads rather than an activation failure somebody has to
// find in a log.
//
// The difference between that and the stop path of §5.2 is the whole of what is
// still withheld, and it is worth stating rather than implying:
//
//   a mode change  is *asked for* on `/crane/set_mode`, by an operator or by
//                  the task layer, and is answered -- performed, or refused
//                  with a reason. Every precondition is checked before anything
//                  is deactivated (PRD §10 step 2, user story 35), so a switch
//                  that cannot succeed is refused from the mode the machine is
//                  already in rather than discovered half-way through one.
//   a stop         would be this supervisor deactivating a claim *on its own*,
//                  on a fault, and zeroing at the driver boundary. Nothing here
//                  does that, nothing here may, and no function below is
//                  reachable except from a request.
//
// `MODE_IDLE` is a mode and not a stop, and the report it produces says so
// outright: releasing the claim is not the same as bringing the machine to
// rest, because §7.2's measured defect leaves a manual controller's last
// velocity latched on the interface it just released.
//
// # The handover, and why step 2 is first
//
// Since issue 054 `MODE_MPC` is a mode this deployment implements, and PRD §10
// sequences the change into it in four steps. Two of them are this file's:
//
//   step 2  the horizon's freshness is verified **before** the trajectory
//           controller is deactivated. `mpc_horizon_refusal()` is that check,
//           and it is a free function rather than a branch buried in
//           `arbitrate_mode()` because the node runs it *before it asks the
//           controller manager for anything at all* -- so a switch into a dead
//           MPC is refused from the mode the machine is already in, not
//           discovered half-way through one (user story 35). The refusal names
//           freshness, the age, the deadline and what the newest solve said,
//           because an operator left with an unexplained no-op is user story
//           36's failure.
//   the mode of the producer  which path is live is this supervisor's decision
//           alone (ROS 2 Interfaces §4, "One command path"), so the switch also
//           moves `crane_mpc` between the shadow and active states issue 053
//           built. `ModeArbitration::horizon_producer_active` carries that half
//           of the plan and `ProducerSwitch` carries its outcome.
//
// Steps 3 and 4 are **not** here and are not this package's at all. The seam
// clamp and the setpoint the re-entering trajectory is anchored on live in
// `crane_control`, where the command is, and the reason they need no supervisor
// is PRD §10 step 3's own: the same `crane_velocity_controller` instance runs
// both paths, so a mode change deactivates the trajectory controller and leaves
// the inner loop exactly where it was. That is why `MODE_MPC`'s controller list
// overlaps `MODE_FOLLOW`'s rather than replacing it, and why the plan below
// never deactivates a controller the incoming mode also wants.
//
// # Claims, plural, and why the model is not one per machine
//
// §7.3 is the trap this model is shaped around. With the block gripper the tool
// axis is driven by its **own** controller rather than by the arm's, so it can
// move while the arm controller is inactive: a second, independent claim on the
// same pump. A mode model that mapped one machine onto one claim would be wrong
// on the machine this stack runs on, and would be wrong silently -- it would
// deactivate a tool controller it never meant to touch, or leave one holding a
// claim it never accounted for.
//
// So `SupervisorConfig` carries the controllers of the **arm claim** per mode
// and the controllers of the **tool claim** separately, and a mode switch plans
// over the arm claim alone: the tool claim is reported and never switched by a
// mode request. What the deployed `fake` and `hardware` profiles configure today
// is one claim -- `crane_velocity_controller` holds all six `velocity`
// interfaces including the gripper axis -- so the tool list ships empty and the
// configuration file says why. The model carries the second claim because §7.3
// says the machine has one, not because slice 3 can see it.
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
// # The sway duty, and why it is a rate and a predicate
//
// §5 row 7 gives the supervisor one narrow duty about the sway: **refuse to
// start a motion that depends on sway being settled, and report**. It does not
// damp -- that is slice 6 -- and it does not stop a motion already running for
// being swingy. The refusal half needs an authority over a mode that this
// package does not have yet, so what lands here is the report and the signal
// somebody else refuses on.
//
// Two things come off the passive rate and they are not the same thing:
//
//   the bound      `FAULT_SWAY`, raised per coordinate against a configured
//                  rate, naming which of the two exceeded it.
//   the predicate  three-valued -- settled, not settled, unknown -- held over a
//                  dwell and released through a hysteresis, so that one rate
//                  crossing does not chatter the signal at 20 Hz.
//
// **A degraded estimate is neither.** It raises `FAULT_STATE_HEALTH` through the
// paths above, and it leaves the predicate `Unknown`: a sensor that stopped
// saying anything is a different fact from a load that is swinging, and an
// operator does different things about them. Everything about why the test is on
// the rate, and why `velocity_covariance` is not consulted, is in
// `sway_monitor.hpp`.
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

#include "crane_supervisor/sway_monitor.hpp"

namespace crane_supervisor
{

/// Control mode, numbered as `crane_msgs/SupervisorStatus` numbers it.
/**
 * The values are the wire's, so the adapter is a cast and not a table. The
 * package is ROS-free and cannot include the message to say so; the contract
 * test does, and asserts each pair.
 *
 * What is reported is **re-derived from the controller manager on every status
 * cycle**, never remembered from the last successful `/crane/set_mode` call. A
 * supervisor whose model of the world drifts from the world is worse than one
 * that admits it does not know, and a controller that died, was never spawned
 * or was switched by somebody else is exactly the drift a remembered mode would
 * hide.
 */
enum class Mode : std::uint8_t
{
  Idle = 0,
  Manual = 1,
  Follow = 2,
  Mpc = 3,
};

/// How many modes there are. The bound of everything that is per mode.
inline constexpr std::size_t kModeCount = 4;

/// One mode as its own array index.
[[nodiscard]] inline constexpr std::size_t index_of(Mode mode) noexcept
{
  return static_cast<std::size_t>(mode);
}

/// True when a raw wire value names one of the four modes.
/**
 * `crane_msgs/SetMode` carries a `uint8`, so a caller can ask for 200. Casting
 * that to `Mode` and arbitrating on it would be undefined behaviour dressed up
 * as a mode change, and answering it with a plausible mode would be worse: the
 * request is refused and the response says which values exist.
 */
[[nodiscard]] bool is_mode(std::uint8_t value) noexcept;

/// The mode's constant name, as an operator reads it off a panel.
[[nodiscard]] const char * mode_name(Mode mode) noexcept;

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

/// What the horizon producer's newest solve did, numbered as the wire numbers it.
/**
 * The four `SOLVE_*` constants of `crane_msgs/SolverHealth`, in that message's
 * own numbering, so the adapter is a cast and not a table and
 * `test_contract.cpp` asserts each pair. Zero is **unknown** and not a healthy
 * solve, deliberately and for the reason `SwaySettled`'s zero is unknown: a
 * message nobody filled must not read as an optimizer that converged, and
 * `Converged` is the only value PRD §10 step 1's "the MPC runs warm before the
 * switch" is satisfied by.
 */
enum class SolveOutcome : std::uint8_t
{
  Unknown = 0,
  Converged = 1,
  BudgetExceeded = 2,
  Failed = 3,
};

/// The outcome as an operator reads it off a panel.
[[nodiscard]] const char * solve_outcome_name(SolveOutcome outcome) noexcept;

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

  /// How old the controller manager's answer may be and still say what is
  /// active, s.
  /**
   * Zero for the same reason the four above are: `validate()` refuses it and
   * the node throws rather than reporting a mode off a snapshot with no margin
   * on it. It is separate from the array because the thing it bounds is a poll
   * and not a stream -- see `ControllerManagerReport`.
   */
  double controller_manager_deadline{0.0};

  /// How old the horizon producer's newest report may be and still say it
  /// solves, s.
  /**
   * PRD §10 step 2's whole margin, and the only thing this supervisor decides
   * about the MPC. Zero for the reason every other deadline here is zero:
   * `validate()` refuses it and the node throws rather than admitting a switch
   * into a producer it cannot date. It is separate from the array above because
   * this stream is **not** an `Input` -- see `HorizonReport` for why, and for
   * what §5.3's rule looks like for something whose absence is a refused
   * request rather than a fault on the status.
   */
  double horizon_deadline{0.0};

  /// The node the horizon producer was loaded under, whose mode this
  /// supervisor drives.
  /**
   * Configuration for the reason `mode_controllers` is: ROS 2 Interfaces §4
   * fixes that which path is live is the supervisor's decision alone, and a
   * profile fixes which node implements the MPC path and under what name. The
   * `crane_mpc` node's `mode` parameter is what that decision moves (issue
   * 053), and this is the only name in this package that reaches a node rather
   * than a topic or a service of the architecture's own.
   *
   * Empty is admissible only on a deployment that does not implement
   * `MODE_MPC`: `validate()` refuses a configuration that gives `MODE_MPC`
   * controllers and names no producer, because the claim would then move to a
   * path whose producer nothing ever started.
   */
  std::string mpc_node;

  /// Which controllers hold the **arm claim** in each mode, indexed by `Mode`.
  /**
   * The names a deployment loaded its controllers under, so this is
   * configuration and not a constant: the architecture fixes the four modes and
   * a profile fixes what implements them. `mode_controllers[index_of(Idle)]` is
   * empty by definition -- `MODE_IDLE` is the absence of a motion claim, not a
   * controller -- and `validate()` refuses a deployment that gives it one.
   *
   * An empty list for a *motion* mode is not an error either: it is the honest
   * state of a deployment that composes no controller for it, and the request
   * is refused with that as the reason. `MODE_MANUAL` is empty on today's
   * profiles for exactly that reason.
   *
   * **`MODE_FOLLOW` and `MODE_MPC` overlap, and that overlap is the
   * architecture.** PRD §10 step 3 is explicit that the same
   * `crane_velocity_controller` instance carries both paths -- "so the handover
   * is an ordinary seam, not new machinery" -- so `MODE_MPC`'s list is
   * `MODE_FOLLOW`'s without the trajectory controller, and a mode switch
   * between them neither activates nor deactivates the inner loop. What
   * `validate()` enforces is therefore **not** disjointness, which would
   * outlaw exactly that: it is that no two modes name the *same set*, because
   * `active_mode()` matches the active controllers against each mode's set by
   * equality, and two identical sets would make two modes read as active at
   * once.
   */
  std::array<std::vector<std::string>, kModeCount> mode_controllers;

  /// Which controllers hold the **tool claim**, in every mode.
  /**
   * §7.3: with the block gripper the tool axis is on its own controller and can
   * move while the arm controller is inactive, so it is a second claim on the
   * same pump. No mode owns it and no mode switch touches it -- a request for
   * `MODE_FOLLOW` neither activates nor deactivates a gripper controller -- and
   * what the arbitration does with it is *report* it, so that a caller can see
   * that an arm motion and a gripper action are two claims rather than one
   * machine.
   *
   * Empty on the deployed profiles today, and that is the honest state rather
   * than a modelling choice: `crane_velocity_controller` claims all six
   * `velocity` interfaces including `q9_left_rail_joint`, so the CBS stack
   * currently has one claim. `config/crane_supervisor.yaml` says so beside the
   * empty list.
   */
  std::vector<std::string> tool_controllers;

  /// The six rows of `crane_control/config/tracking_tolerance.yaml`, in the
  /// order the actuated joints are configured in.
  /**
   * Empty until the node loads them, and an empty list is not an error: it is
   * the state a deployment that has not been given the file is honestly in, and
   * it raises no tracking fault at all.
   */
  std::vector<AxisTolerance> tracking_tolerance;

  /// The bounds the sway duty of wiki/control_architecture.md §5 row 7 is
  /// decided against, and the dwell its predicate is held over.
  /**
   * Unlike `tracking_tolerance`, every number here is shipped with the package,
   * so a `SupervisorConfig` that has none is a misconfiguration rather than a
   * deployment waiting on a human campaign -- `validate()` refuses it. They are
   * **design** values and the file that carries them says so, beside what would
   * replace each.
   */
  SwayBound sway;

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

/// One controller, as the controller manager described it.
/**
 * The three fields of `controller_manager_msgs/ControllerState` this package
 * reads, and no more. `claimed_interfaces` is here rather than derived because
 * it is the only thing on the graph that says *which* claim a controller holds:
 * §7.3's second claim on the same pump is two controllers holding disjoint sets
 * of command interfaces, and a model that only knew controller names could not
 * tell that from one controller holding both.
 */
struct ControllerReport
{
  /// The name the controller manager loaded it under, which is the name a mode
  /// list names and a switch request carries.
  std::string name;
  /// `unconfigured`, `inactive`, `active` or `finalized`, verbatim. Carried and
  /// not folded into a bool: "not active" covers a controller that is one call
  /// away from active and one that failed to configure, and a switch that could
  /// not succeed has to be refused for the right reason.
  std::string state;
  /// The command interfaces this controller owns right now.
  std::vector<std::string> claimed_interfaces;
};

/// What the controller manager last said about its controllers.
/**
 * This is the one input of this supervisor that is **polled** rather than
 * subscribed, and it is deliberately not an `Input`: the registry above is the
 * enum of every *subscription* this package holds, and every piece of machinery
 * around it -- the `subscribe()` helper, the policy row's `topic` and `stopped`
 * strings, the constructor's claim check -- is shaped for a stream.
 *
 * §5.3's rule still applies and is met differently, because the failure is a
 * different shape. A subscription that stops arriving is invisible without a
 * deadline; a service that stops answering fails at the call site, where the
 * node can see it. So the consequence is defined by construction rather than by
 * a table: no answer, or an answer older than `controller_manager_deadline`,
 * means the active mode **is not known**, and every report says so in the mode
 * clause instead of carrying the last mode the supervisor happened to see.
 *
 * It raises no fault of its own, and that is a decision rather than an
 * omission. A controller manager that stops answering `list_controllers` is not
 * a degraded estimate and is not a stopped machine -- the controllers may be
 * cycling perfectly -- so `FAULT_STATE_HEALTH` would blame the crane for a hole
 * in this supervisor's own view. What it does do is refuse every mode change:
 * `arbitrate_mode()` cannot check a precondition against a view it does not
 * have, and a switch issued blind is exactly the half-way discovery PRD §10
 * step 2 forbids.
 */
struct ControllerManagerReport
{
  /// The transport half, judged by the same `freshness_of()` every input is:
  /// `received` is whether an answer has ever come back, `age` is how long ago
  /// the newest one did, on this node's own clock.
  StreamReport answer;
  /// One entry per loaded controller, in the order the manager listed them.
  std::vector<ControllerReport> controllers;
};

/// What the horizon producer last said about itself, and how old that is.
/**
 * PRD §10 step 2's evidence, and the second thing this supervisor watches that
 * is deliberately **not** an `Input`. The registry above is the enum of the
 * inputs whose absence raises a fault on the status stream, and this one's
 * absence must not: a stack in `MODE_FOLLOW` with no optimizer running is the
 * ordinary state of this machine, not a defect, and
 * wiki/implementation/ros2_interfaces.md §4 records outright that the
 * supervisor merging `FAULT_SOLVER` onto `/crane/supervisor/status` is a slice
 * of its own. A row in `kInputPolicies` would make it a standing fault on every
 * profile the day it was added.
 *
 * §5.3's rule is met and it is met the way `ControllerManagerReport` meets it:
 * the consequence of this stream stopping is **defined, narrow and stated at
 * the point it matters** -- no freshness, no switch. `arbitrate_mode()` refuses
 * `MODE_MPC` and says how old the newest report is and what the deadline was,
 * so an absent optimizer is a refusal an operator reads rather than a switch
 * somebody discovers half-way through (user stories 35 and 36).
 *
 * **It is `/crane/mpc/solver_health` and not `/crane/mpc/horizon`, and the
 * substitution is forced rather than chosen.** In shadow -- which is the state
 * every switch into `MODE_MPC` is made *from* -- `crane_mpc` publishes nothing
 * at all on the contract topic (issue 053), so a freshness check against the
 * horizon itself could never pass and `MODE_MPC` would be unreachable by
 * construction. `solver_health` runs in both modes, carries the solve's own
 * three-valued verdict, and names the mode, which is exactly what PRD §10 step
 * 1 means by "the MPC runs warm before the switch; shadow mode already implies
 * it solves". It also keeps this package clear of `trajectory_msgs`, the
 * package a *commanded* trajectory is typed with, which
 * `test_no_command_path.py` bans outright.
 */
struct HorizonReport
{
  /// The transport half, judged by the same `freshness_of()` every input is.
  StreamReport health;
  /// `crane_msgs/SolverHealth.outcome`, in the numbering the wire and the enum
  /// share.
  SolveOutcome outcome{SolveOutcome::Unknown};
  /// `crane_msgs/SolverHealth.fault`, carried and not translated, exactly as
  /// the inner loop's is. `FAULT_SOLVER` is the only code this producer raises.
  Fault fault{Fault::None};
  /// `solve_time` and `solve_budget`, s. Reported in the refusal so that a
  /// producer that is losing its deadline says so in the sentence that refuses
  /// the switch rather than only in its own stream.
  double solve_time{0.0};
  double solve_budget{0.0};
  /// `applied_previous_solution`: the shifted previous horizon went out instead
  /// of this solve (wiki/mpc.md §6, requirement 3).
  bool applied_previous_solution{false};
  /// `crane_msgs/SolverHealth.message`, verbatim. The producer's own account of
  /// what it did, carried rather than restated, for the reason
  /// `PendulumStateReport::status` is.
  std::string status;
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
  /// `crane_msgs/PendulumState.velocity`, rad/s, `[tip, tilt]`.
  /**
   * The rate and not the angle, and not the covariance either. The angle is read
   * out on the nominal hinge axes and carries an uncalibrated constant offset
   * that nothing in this workspace has measured; the rate is composed from the
   * two gyro readings and does not. `velocity_covariance` is the identified
   * noise of the differenced pair rather than a running statement about this
   * cycle, so weighting a bound with it would be a confidence-weighted test
   * whose confidence never moves -- `sway_monitor.hpp` says the whole of it.
   *
   * NaN until a message fills it, which is the absence of a measurement rather
   * than a measurement of zero: a rate that is not a number leaves the settled
   * predicate `Unknown`.
   */
  std::array<double, kPassiveAxisCount> velocity{
    {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()}};
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

  /// What the controller manager last said, and how old that answer is.
  /**
   * Not one of the `streams` above, because it is polled rather than
   * subscribed; judged by the same `freshness_of()` they are, because the
   * question "is this old enough to stop believing" is the same question.
   */
  ControllerManagerReport controller_manager;

  /// What the horizon producer last said about itself.
  /**
   * Not one of the `streams` above and not a fault of its own: the whole of
   * what it decides is whether `MODE_MPC` may be entered. `HorizonReport` says
   * why that is the right shape and why the evidence is the solve rather than
   * the horizon.
   */
  HorizonReport horizon;

  /// The emergency-stop latch as the previous cycle left it.
  /**
   * The latch is state and the core is a pure function, so the state is
   * threaded through rather than held: the node carries `estop_latched` out of
   * one decision and back into the next. That keeps every latched case
   * reachable from a test by writing one bool, instead of by replaying the
   * history that produced it.
   */
  bool estop_latched{false};

  /// The sway dwell as the previous cycle left it.
  /**
   * Threaded through for the same reason the latch is: the core is a pure
   * function, so the one piece of history the settled predicate needs is carried
   * rather than held. Writing two members reaches any point of the dwell that a
   * test wants, instead of replaying forty cycles to get there.
   */
  SwayState sway;

  /// The instant this observation was taken, on the node's own clock, s.
  /**
   * Read once per cycle and used for one thing: how long the current run of calm
   * cycles has lasted. It is deliberately *not* what `StreamReport::age` is
   * derived from at this level -- the ages arrive already differenced, so a core
   * test can write an age without owning a clock -- and a value that is not
   * finite leaves the predicate `Unknown` rather than completing a dwell.
   */
  double sampled_at{std::numeric_limits<double>::quiet_NaN()};

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
  /// What is **active** right now, re-derived from the controller manager.
  /**
   * Never the last mode a `/crane/set_mode` call succeeded in reaching. When
   * the manager's answer is absent or stale the mode is not known, and this
   * field carries `Idle` -- the value that claims the least, since no motion
   * mode can be confirmed to hold the claim -- while the mode clause on the
   * message says outright that it is not known and why. `Idle` here therefore
   * means "no configured motion mode is known to be active", which is also what
   * it means when the manager answers and nothing is running.
   */
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
  /// The settled predicate, and the dwell as this cycle leaves it.
  /**
   * `sway.settled` is the three-valued answer the task layer gates a grip action
   * on instead of on a timeout, and the node carries the whole struct into the
   * next call the way it carries the latch.
   *
   * **It is decided before any branch of `decide()` returns**, for the reason
   * `deadman_held` and `tracking_error` are filled before any branch returns: it
   * is owed on every cycle and not only on the ones where sway is what went
   * wrong. A dwell that stopped advancing whenever a more consequential cause
   * owned `fault` would restart every time the operator let go of the deadman.
   *
   * It has no field of its own on `crane_msgs/SupervisorStatus`, so what carries
   * it onto the wire today is `message` -- see the README, which names the
   * amendment that would fix that and why it is not in this issue's scope.
   */
  SwayState sway;
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
 *
 * A missing **sway bound** is refused, and the difference from the tolerance is
 * not a preference: the sway numbers are shipped with this package, so a
 * deployment without one has been misconfigured rather than left waiting on a
 * human campaign. `validate_sway()` composes the reason.
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

/// What the controller manager's answer says is active, per claim.
/**
 * Read from the world rather than remembered, every status cycle and again on
 * every `/crane/set_mode` call.
 */
struct ActiveMode
{
  /// The mode whose controllers are **all** active, or `Idle` when none is.
  Mode mode{Mode::Idle};
  /// False when the manager's answer is absent or stale, so `mode` above is the
  /// value that claims the least rather than an observation.
  bool known{false};
  /// Which of the two it is, judged by the same rule every input is judged by.
  Staleness cause{Staleness::NeverArrived};
  /// The controllers of the arm claim that are active right now, named.
  /**
   * Named and not counted, for the reason the uncommissioned axes are: a report
   * that says `crane_velocity_controller` tells somebody what to look at.
   */
  std::vector<std::string> active_controllers;
  /// The controllers of the **tool claim** that are active right now (§7.3).
  /**
   * Reported and never switched. It is a separate list because a gripper action
   * and an arm motion are two claims on the same pump, and a report that added
   * them together would be the one-claim-per-machine model this package does
   * not use.
   */
  std::vector<std::string> active_tool_controllers;
  /// Active controllers this supervisor's model does not account for.
  /**
   * A controller that is active, owns command interfaces, and is in no mode
   * list and not in the tool list. It is **reported and not refused**: the
   * broadcasters own no command interface and would be false alarms, while a
   * controller that does own one is a third claim on the same pump that this
   * configuration never described. §7.3's point is that the machine has more
   * claims than a mode model has modes, so the honest thing is to say which
   * rather than to pretend the list is closed.
   */
  std::vector<std::string> unmodelled_claimants;
  /// True when the active controllers do not form exactly one configured mode.
  /**
   * Either some of a mode's controllers are up and not all of them, or two
   * modes are complete at once. Both mean the world drifted: something was
   * activated or died outside a mode change. `mode` is `Idle`, because no
   * configured motion mode holds the claim on its own, and the clause names the
   * controllers that are up so that the half-state is visible rather than
   * rounded off into a mode nobody can act on.
   */
  bool partial{false};
};

/// How the mode clause every report ends in opens.
/**
 * Exported for the same reason `kSettledClausePrefix` is: a test that has to
 * find the seam should not have to write the sentence out a second time.
 */
inline constexpr char kModeClausePrefix[] = " Mode: ";

/// One `/crane/set_mode` decision, before anything has been switched.
/**
 * Total in the same sense `decide()` is: every request produces an answer with
 * a cause, and `message` is never empty (ROS 2 Interfaces §1, PRD user story
 * 36). A refused switch that leaves an operator with an unexplained no-op is
 * the failure this struct exists against.
 */
struct ModeArbitration
{
  /// Whether the request is admissible. False means refused, and `message`
  /// says by which precondition.
  bool accepted{false};
  /// Whether a switch is owed. False with `accepted` true is the request for
  /// the mode that is already active: nothing is issued and it is reported as
  /// a no-op rather than as a switch.
  bool switch_required{false};
  /// The controllers to activate, in the order they must come up.
  std::vector<std::string> activate;
  /// The controllers to deactivate. Only ever controllers of the arm claim:
  /// no mode request touches the tool claim (§7.3).
  std::vector<std::string> deactivate;
  /// What is active **now**, before anything is switched. This is what the
  /// response carries when the request is refused, because a refusal leaves the
  /// machine in the mode it was already in.
  ActiveMode active;
  /// Which mode the horizon producer has to be in once this plan is carried
  /// out: `true` is `active`, `false` is `shadow`.
  /**
   * True for `MODE_MPC` and false for every other mode, so it is not an extra
   * decision -- it is the same decision, read on the producer's side of the
   * seam. ROS 2 Interfaces §4's "One command path" ends by saying that which
   * path is live is the supervisor's decision **alone** and that the two never
   * drive at once, and this field is where that sentence becomes a call: issue
   * 053 made shadow a state of `crane_mpc` and left the transition to be driven
   * from here.
   *
   * **The order is not symmetric, and it cannot be.** Entering `MODE_MPC` the
   * producer is put into `active` *before* the claim moves, so that a horizon
   * is already fitted when the trajectory controller lets go -- an unchained
   * velocity controller with no horizon ramps its command to zero and raises
   * `FAULT_REFERENCE_STALE`, which is a hole in the handover rather than a
   * seam. Leaving it the producer is put back into `shadow` *after* the claim
   * has moved, for the same reason read the other way. `supervisor_node.hpp`
   * carries the sequence.
   */
  bool horizon_producer_active{false};
  /// Why, in words an operator can act on. Never empty.
  std::string message;
};

/// What this supervisor did to the horizon producer as part of one switch.
/**
 * Reported rather than assumed, exactly as `active_mode` on the response is:
 * `crane_mpc`'s `mode` is a parameter of a node in another process, and a
 * `set_parameters` call can be refused, time out, or find nobody there. A
 * switch whose claim moved and whose producer did not is the state ROS 2
 * Interfaces §4 forbids, so it is named in the report instead of being left for
 * somebody to infer from a silent horizon.
 */
struct ProducerSwitch
{
  /// Whether a call was made at all. False on every request that was refused,
  /// and on a deployment that names no producer.
  bool attempted{false};
  /// Which mode was asked for: `true` is `active`.
  bool active{false};
  /// Whether the producer accepted it.
  bool accepted{false};
  /// The producer's own reason when it refused, or this node's when the call
  /// never went out. Empty when nothing was attempted.
  std::string account;
};

/// What the controller manager's answer says is active.
/**
 * A mode is active when the controllers of the arm claim that are up are
 * **exactly** the ones configured for it -- equality of sets, not containment.
 *
 * Containment was the rule until slice 6 and it stopped being sound the moment
 * `MODE_MPC` was populated. PRD §10 step 3 puts the *same*
 * `crane_velocity_controller` instance on both paths, so `MODE_MPC`'s list is
 * `MODE_FOLLOW`'s minus the trajectory controller: under containment,
 * `MODE_FOLLOW` holding the claim would satisfy `MODE_MPC` as well and the
 * answer would be whichever list this function happened to look at first, while
 * `MODE_MPC` holding it would leave `MODE_FOLLOW` half up and read as drift.
 * Under equality both are answered exactly, and what `validate()` has to insist
 * on is only that no two modes name the same *set*.
 */
[[nodiscard]] ActiveMode active_mode(
  const SupervisorConfig & config, const ControllerManagerReport & report);

/// Whether PRD §10 step 2's precondition holds, and which cause fired if not.
/**
 * Two questions and both have to answer yes. The report has to be **fresh** --
 * judged by the same `freshness_of()` every stream here is judged by, against
 * `horizon_deadline` -- and the newest solve has to have **converged**.
 *
 * The second is not redundant with the first and it is the half PRD §10 step 1
 * is about: a producer that is alive, publishing at rate and failing every
 * solve is precisely the dead MPC user story 35 refuses to switch into, and it
 * is indistinguishable from a healthy one on freshness alone. `BudgetExceeded`
 * is refused with it, and deliberately: wiki/control_architecture.md §5 row 3
 * gives `FAULT_SOLVER` to a deadline miss and to non-convergence alike, and
 * "the MPC runs warm before the switch" is not satisfied by an optimizer that
 * is only keeping up by shipping the horizon it shifted last cycle. A refusal
 * costs an operator one more request; entering on a plan with nothing behind it
 * costs a handover.
 */
struct HorizonPrecondition
{
  /// True when the producer is arriving inside its deadline **and** its newest
  /// solve converged.
  bool fresh{false};
  /// Which staleness cause fired, judged the way every stream is. `Fresh` here
  /// with `fresh` false means the stream is fine and the *solve* is not.
  Staleness cause{Staleness::NeverArrived};
};

[[nodiscard]] HorizonPrecondition horizon_precondition(
  const SupervisorConfig & config, const HorizonReport & report) noexcept;

/// PRD §10 step 2, on its own and ahead of everything else.
/**
 * Empty when nothing about the horizon refuses `requested`: when the request is
 * not `MODE_MPC` at all, or when `horizon_precondition()` holds. Otherwise the
 * whole refusal, ready to be a `message` -- it names freshness as the cause,
 * how old the producer's newest report is, what the deadline was, what the
 * solve said, and that nothing was deactivated.
 *
 * It is exported rather than left inside `arbitrate_mode()` because **the order
 * is the assertion**. `SupervisorNode::set_mode()` runs it before it asks the
 * controller manager for anything at all -- before the poll, let alone before
 * the switch -- and `arbitrate_mode()` runs it again at precondition 2. Two
 * call sites, one function, so the early answer and the arbitration cannot
 * disagree about what was verified or when.
 */
[[nodiscard]] std::string mpc_horizon_refusal(
  const SupervisorConfig & config, std::uint8_t requested, const SupervisorInput & input);

/// The mode clause, for the end of any report. Never empty.
[[nodiscard]] std::string mode_clause(const SupervisorConfig & config, const ActiveMode & active);

/// One `/crane/set_mode` request, arbitrated. Nothing is switched here.
/**
 * Every precondition is evaluated **before** the plan is returned, and the plan
 * is a single activate/deactivate pair for one `switch_controller` call, so a
 * switch that cannot succeed is refused from the mode the machine is already in
 * and is never discovered half-way (PRD §10 step 2, user story 35). In order:
 *
 *   1. the request names one of the four modes;
 *   2. **the horizon is fresh** (PRD §10 step 2, user story 35). Above the
 *      view, above the latch and above every deployment question, because it
 *      depends on none of them and because the node runs the same check before
 *      it asks the controller manager for anything at all -- so a switch into a
 *      dead MPC is refused rather than discovered;
 *   3. the controller manager has answered inside its deadline, since a
 *      precondition cannot be checked against a view this supervisor does not
 *      have;
 *   4. the mode asked for is not the one already active;
 *   5. a latched fault refuses a switch into a *motion* mode, with the latched
 *      cause as the reason. `MODE_IDLE` is not refused by it: releasing the
 *      claim is the one direction a latched stop does not argue against;
 *   6. the deployment configures controllers for the mode;
 *   7. every one of them is loaded on the manager and is in a state a switch
 *      can move.
 *
 * `input` is the same observation `decide()` is judged from, deliberately: an
 * arbitration that consulted a second, older view of the stop signal could
 * admit a switch whose refusal the very next status cycle still stands by.
 */
[[nodiscard]] ModeArbitration arbitrate_mode(
  const SupervisorConfig & config, std::uint8_t requested, const SupervisorInput & input);

/// The account of a switch that was issued, once the outcome is known.
/**
 * Composed here rather than in the adapter so that the sentence an operator
 * reads is written once, beside the arbitration that produced the plan.
 * `reached` is the mode the controller manager reports **after** the call --
 * `active_mode` on the response is what is actually active, never what was
 * asked for. `producer` is what happened to the horizon producer's own mode,
 * reported for the same reason and never assumed: a claim that moved and a
 * producer that did not is the state ROS 2 Interfaces §4 forbids, and the
 * report has to be able to say so.
 */
[[nodiscard]] std::string mode_switch_message(
  const SupervisorConfig & config, Mode requested, const ModeArbitration & arbitration,
  bool switched, const std::string & carried, const ActiveMode & reached,
  const ProducerSwitch & producer);

}  // namespace crane_supervisor

#endif  // CRANE_SUPERVISOR__SUPERVISOR_CORE_HPP_
