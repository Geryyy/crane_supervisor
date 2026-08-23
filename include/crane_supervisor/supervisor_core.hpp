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
// Two inputs are carried end to end: `crane_msgs/PendulumState` on
// `/crane/pendulum_state`, and `epsilon_crane_msgs/RemoteCtrlStates` on
// `/crane/remote_ctrl_states`.
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
#include <string>

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

/// Everything one decision is made from. Two inputs in this slice; the rest of
/// the causes of §5 arrive as further members, one issue each.
struct SupervisorInput
{
  PendulumStateReport pendulum_state;
  RemoteCtrlReport remote_ctrl;
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
  /// Max over the actuated joints, rad or m. Not computed in this slice: no
  /// reference reaches the supervisor yet, so it stays at zero and the message
  /// says the field is not computed rather than letting a zero read as perfect
  /// tracking.
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
[[nodiscard]] bool validate(const SupervisorConfig & config, std::string & reason);

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
