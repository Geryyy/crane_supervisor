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
// One input is carried end to end: `crane_msgs/PendulumState` on
// `/crane/pendulum_state`. It is the one input in this stack that reports its
// own health honestly, and the three ways it can fail are the three the tracer
// has to separate:
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

/// Everything one decision is made from. One input in this slice; the rest of
/// the causes of §5 arrive as further members, one issue each.
struct SupervisorInput
{
  PendulumStateReport pendulum_state;
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
  /// Not observed in this slice: the deadman is on the operator remote, which
  /// is a later issue. False is again the value that claims nothing.
  bool deadman_held{false};
  /// Why, in words an operator can act on. Never empty (PRD user story 53).
  std::string message;
};

/// Adopts and checks the margins. Returns false and says why, once, on failure.
[[nodiscard]] bool validate(const SupervisorConfig & config, std::string & reason);

/// One status cycle. Total: every input produces a decision with a cause.
[[nodiscard]] SupervisorDecision decide(
  const SupervisorConfig & config, const SupervisorInput & input);

}  // namespace crane_supervisor

#endif  // CRANE_SUPERVISOR__SUPERVISOR_CORE_HPP_
