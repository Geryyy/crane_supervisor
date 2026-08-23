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
// Four inputs are carried end to end: `crane_msgs/PendulumState` on
// `/crane/pendulum_state`, `epsilon_crane_msgs/RemoteCtrlStates` on
// `/crane/remote_ctrl_states`, the trajectory controller's own
// `control_msgs/JointTrajectoryControllerState`, and
// `crane_msgs/VelocityControllerHealth` on
// `/crane/velocity_controller/health`.
//
// The fourth is the inner velocity loop's verdict on itself, and it is carried
// rather than re-derived for a reason that is not a preference: **only the
// controller knows which axes the active tool has a valve calibration for.**
// Nothing in `/joint_states` or in the trajectory controller's state says it, so
// a supervisor that tried to work it out would be guessing at the one fact the
// report exists to deliver. What arrives is a `crane_msgs/SupervisorStatus`
// fault constant, so this package merges a code rather than translating one, and
// the per-axis feedforward flags with the joint names beside them, so a panel
// can say which axis is uncommissioned rather than that something is.
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
// The passive state is the one input in this stack that reports its own health
// honestly, and the three ways it can fail are the three the tracer has to
// separate:
//
//   absent   nothing has arrived yet, or the stream stopped. Absence is not
//            health -- §5.3's rule is that no input may stop arriving without a
//            defined consequence, so this is a fault and not FAULT_NONE. The
//            *general* staleness policy is a later issue; what is owed here is
//            only that this input cannot be silently missing.
//   ahead    the newest sample is stamped further in the future than the margin
//            allows, so its age cannot be judged at all. A clock that is not
//            synchronised makes every staleness answer meaningless (PRD user
//            story 59), and reporting "fresh" off it would be a guess.
//   invalid  it arrived, in time, and `pendulum_state_broadcaster` marked it
//            unusable. The broadcaster already separates six causes behind that
//            flag and says which in its `status` string, so the string is
//            *carried through* rather than restated here: restating it would
//            flatten a distinction the estimator went to some trouble to make.
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
//                        acknowledgement on `/crane/clear_fault` clears it. A
//                        latch cleared while the condition still holds is
//                        refused with an explanation; one cleared and then
//                        re-raised on the next cycle is working as specified.
//   continuous           the deadman is re-checked every cycle rather than once
//                        at the start of a motion (§6.2).

#ifndef CRANE_SUPERVISOR__SUPERVISOR_CORE_HPP_
#define CRANE_SUPERVISOR__SUPERVISOR_CORE_HPP_

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
  /// Longest age of the newest `crane_msgs/PendulumState` that still counts as
  /// arriving, s.
  /**
   * Derived, not measured, and deliberately loose: it separates *arriving* from
   * *stopped*, which is all this slice owes. The general staleness policy --
   * per input, against the loop period, with the consequence §5.3 tabulates --
   * is a later issue and will want a tighter number than this one.
   *
   * The derivation. `/crane/pendulum_state` is published at 100 Hz by a
   * controller inside the manager's cycle (ROS 2 Interfaces §2, §4) and this
   * node samples it at 20 Hz, so a sample that is perfectly healthy has already
   * been sitting for up to one 50 ms status period when it is read, plus the up
   * to 10 ms it waited for the next control cycle. Against that, the recorded
   * machine data of the workspace CLAUDE.md puts the worst observed control
   * cycle gap at 50.4 ms. 60 + 50 = 110 ms is therefore the worst age a healthy
   * stack can produce, and 150 ms clears it by a margin that is not so wide
   * that a dead publisher goes unnoticed for a human-perceptible time.
   */
  double pendulum_state_timeout{0.15};

  /// Longest age of the newest `epsilon_crane_msgs/RemoteCtrlStates` that still
  /// counts as arriving, s. Past it the signal is absent, and absent is
  /// asserted (§6.1).
  /**
   * `/crane/remote_ctrl_states` is a 20 Hz contract (ROS 2 Interfaces §4) and
   * this node samples it at 20 Hz, so a perfectly healthy sample can be a 50 ms
   * publication period plus a 50 ms status period old when it is read, and the
   * worst control-cycle gap in the recorded machine data adds a further 50.4 ms.
   * 150 ms is therefore the worst age a healthy stack produces and 250 ms clears
   * it without letting a dead reader pass for a held button for longer than a
   * quarter second.
   *
   * It is not a safety timing requirement and must not be read as one: the
   * hardware chain has already stopped the machine by the time this margin
   * expires. What the number trades is a false alarm against how long the
   * *diagnosis* lags the event.
   */
  double remote_ctrl_timeout{0.25};

  /// Longest age of the newest `control_msgs/JointTrajectoryControllerState`
  /// that still counts as arriving, s.
  /**
   * Derived exactly as `pendulum_state_timeout` is, and to the same number,
   * because the two streams have the same shape: the trajectory controller
   * publishes its state from inside the manager's 100 Hz cycle -- ungated, once
   * per `update()` -- and this node samples it at 20 Hz, so a healthy sample is
   * up to one 50 ms status period plus one 10 ms control period old when it is
   * read. The worst observed control-cycle gap in the recorded machine data adds
   * 50.4 ms, and 150 ms clears the resulting 110 ms without letting a controller
   * that stopped publishing pass for a crane that is tracking perfectly.
   */
  double controller_state_timeout{0.15};

  /// Longest age of the newest `crane_msgs/VelocityControllerHealth` that still
  /// counts as arriving, s.
  /**
   * Derived exactly as `remote_ctrl_timeout` is, and to the same number, because
   * the two streams have the same shape rather than because one copied the
   * other: `/crane/velocity_controller/health` is a 20 Hz contract (ROS 2
   * Interfaces §4) — the 100 Hz loop decimates to the rate of its consumers —
   * and this node samples it at 20 Hz, so a healthy sample can be a 50 ms
   * publication period plus a 50 ms status period old when it is read, and the
   * worst control-cycle gap in the recorded machine data adds a further 50.4 ms.
   * 250 ms clears the resulting 150 ms.
   *
   * Only "arriving" against "stopped" is decided here. The general staleness
   * policy of §5.3 is a later issue, and this stream joins it there; what is owed
   * now is only that "no message yet" cannot read as a healthy, commissioned
   * inner loop.
   */
  double controller_health_timeout{0.25};

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
};

/// What the node observed of `/crane/pendulum_state` this cycle.
/**
 * The node computes `age`, because measuring it needs a clock and the core has
 * none. Everything else is copied off the message.
 */
struct PendulumStateReport
{
  /// False until the first message arrives. It does not become false again:
  /// a stream that stopped is caught by `age`, and the two are different
  /// causes with different messages.
  bool received{false};
  /// `crane_msgs/PendulumState.valid`.
  bool valid{false};
  /// `crane_msgs/PendulumState.status`, verbatim. The broadcaster's account of
  /// why, carried rather than restated.
  std::string status;
  /// `now - header.stamp` of the newest message, s. Negative when the stamp is
  /// in this node's future, which is a clock fault rather than a fresh sample.
  double age{0.0};
};

/// What the node observed of `/crane/remote_ctrl_states` this cycle.
/**
 * The node computes `age` and resolves which of the twelve booleans the
 * configured deadman is; the message carries neither. Everything else is copied
 * off the message.
 */
struct RemoteCtrlReport
{
  /// False until the first message arrives. As with the passive state, a stream
  /// that stopped is caught by `age` instead, because "never started" and
  /// "stopped" are different things to tell an operator.
  bool received{false};
  /// The configured deadman button of the newest message. False while nothing is
  /// arriving: a button nobody reported is not a button somebody is holding.
  bool deadman_held{false};
  /// `epsilon_crane_msgs/RemoteCtrlStates.em_stop` of the newest message.
  bool em_stop{false};
  /// `now - header.stamp` of the newest message, s. Negative when the stamp is
  /// in this node's future, which is a clock fault rather than a fresh sample.
  double age{0.0};
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

/// What the node observed of the trajectory controller's state publication.
struct ControllerStateReport
{
  /// False until the first message arrives. As with the other two inputs, a
  /// stream that stopped is caught by `age` instead.
  bool received{false};
  /// `now - header.stamp` of the newest message, s. Negative when the stamp is
  /// in this node's future, which is a clock fault rather than a fresh sample.
  double age{0.0};
  /// One entry per joint the newest message named.
  std::vector<AxisError> axes;
};

/// What the node observed of `/crane/velocity_controller/health` this cycle.
/**
 * The node computes `age` and reads the two parallel arrays off the message;
 * everything else is the inner loop's own answer, carried and not restated.
 */
struct ControllerHealthReport
{
  /// False until the first message arrives. As with the other three inputs, a
  /// stream that stopped is caught by `age` instead: "the controller never came
  /// up" and "the controller died" are different things to tell an operator.
  bool received{false};
  /// `now - header.stamp` of the newest message, s. Negative when the stamp is
  /// in this node's future, which is a clock fault rather than a fresh sample.
  double age{0.0};
  /// `crane_msgs/VelocityControllerHealth.fault`, in the numbering the wire and
  /// this enum share. The inner loop can raise `StateHealth`, `ReferenceStale`
  /// and `NotCommissioned`; a fourth would mean the controller's own
  /// `static_assert` block and the frozen message have drifted apart, so it is
  /// carried through rather than flattened into one of the three.
  Fault fault{Fault::None};
  /// The URDF joints the newest report marked `feedforward_applied == false` —
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

/// Everything one decision is made from. Four inputs so far; the rest of the
/// causes of §5 arrive as further members, one issue each.
struct SupervisorInput
{
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
 * A missing tracking tolerance is deliberately *not* a failure here. It is the
 * state the workspace is actually in, it is reported rather than refused, and a
 * supervisor that declined to start over it would take the whole status stream
 * down for a number that only one of its duties needs.
 */
[[nodiscard]] bool validate(const SupervisorConfig & config, std::string & reason);

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
