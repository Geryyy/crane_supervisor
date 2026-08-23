#include "crane_supervisor/supervisor_core.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <string>
#include <vector>

namespace crane_supervisor
{
namespace
{

// The parts of a staleness report that do not depend on which input it is
// about. Everything that does comes off that input's `InputPolicy` row, so a
// fifth input gets a report by being described once rather than by having three
// more messages written for it -- which is how the four this package already
// carries came to be judged by four copies of the same three comparisons.
constexpr char kNoStatusGiven[] =
  "(the producer set no status string, which is itself a defect -- the message carries the cause "
  "in that field)";

constexpr char kAbsenceIsNotHealth[] =
  " Absence is not health: wiki/control_architecture.md 5.3 allows no input to stop arriving "
  "without a defined consequence, so this is reported as a fault rather than left at FAULT_NONE. ";

constexpr char kObserving[] =
  "no fault: the passive joint state is arriving inside its margin and pendulum_state_broadcaster "
  "reports it usable, neither passive rate is past its sway bound, the trajectory controller's own "
  "state is arriving and no axis is outside its tolerance, the inner velocity loop is arriving and "
  "reports no fault of its own, the "
  "operator remote is arriving, its emergency stop is released and nothing is latched, and the "
  "deadman is held. Those are the only inputs this supervisor watches -- every one of them has a "
  "freshness deadline of its own and none is exempt, but working cell and solver are not "
  "observed yet, inside_working_cell is not computed and carries the value that claims nothing, "
  "and the supervisor holds no stop authority until the hardware stop input is verified. Nor does "
  "this cover the controllers below it: the manual forwarding controller zeroes 0.5 s after its "
  "own input stops and the proportional controller never times out at all, so their staleness "
  "handling stays ad hoc and no report on this stream says anything about it "
  "(wiki/control_architecture.md 6.3). Read FAULT_NONE as 'nothing this supervisor watches is "
  "wrong', not as 'the machine is safe'.";

// The tracking duty of wiki/control_architecture.md §5 row 1, decided from the
// trajectory controller's own state publication. The typed cause §5.0 exists to
// produce, in place of the stall inferred from a deliberately tight goal
// tolerance.
constexpr char kTrackingExceeded[] =
  "tracking: the velocity-tracking tolerance is exceeded on ";

constexpr char kTrackingExceededTail[] =
  ". This is the typed cause the behaviour tree branches on -- the decision to retry, abandon or "
  "re-approach stays in the task layer, which has the context to make it; what this report "
  "replaces is the stall inferred from a deliberately tight goal tolerance. Nothing was stopped, "
  "ramped or commanded here. tracking_error on this report is the largest absolute position "
  "error over the actuated joints, in rad or m as ROS 2 Interfaces 6 fixes that field; the "
  "comparison above is per axis on the velocity error, because the tolerance in "
  "crane_control/config/tracking_tolerance.yaml is a velocity tolerance.";

constexpr char kNoAxisCompared[] =
  " No axis is being compared against a tolerance, so tracking_error is a measurement and not a "
  "verdict: ";

constexpr char kNoToleranceAtAll[] =
  "no per-axis velocity-tracking tolerance is configured. The number does not exist yet -- it "
  "comes from merge gate (ii-b) or from the identification campaign, both human-only -- and it "
  "belongs in crane_control/config/tracking_tolerance.yaml, which is the one file the seam clamp, "
  "the MPC's constraint margin and this supervisor all read, so that they cannot drift apart.";

constexpr char kNoVelocityErrorReported[] =
  "a tolerance is configured but the trajectory controller reports no velocity error for the axes "
  "it covers. It fills that field only when it holds a velocity state interface and a velocity or "
  "effort command interface; a profile that gives it neither leaves the field empty, and an empty "
  "field must not be read as a zero error.";

// The inner velocity loop's own codes, carried through rather than restated:
// the controller computes them from the state interfaces it claims itself and
// from the identified map of the tool it is driving, and nothing above the
// controller manager can see either.
constexpr char kInnerLoopStateHealth[] =
  "state health: the inner velocity loop reports a measurement of its own as stale or degraded. "
  "It judges that per cycle from the state interfaces it claims itself -- the joint position and "
  "velocity of each actuated axis and the chamber pressures the valve inverse is scheduled on -- "
  "so it is the only element in this stack that can see it, and its code is carried here rather "
  "than re-derived from anything else. The axis is not commanded and its integrator is frozen "
  "while this holds.";

constexpr char kInnerLoopReferenceStale[] =
  "reference: the inner velocity loop reports that the horizon it was executing has run out and "
  "nothing replaced it, so its velocity command is on the ramp to zero rather than following "
  "anything (wiki/control_architecture.md 3.3, 5.3). Every measurement is fine and this is not a "
  "state-health fault: the producer is gone. It is a separate constant for that reason -- an "
  "expired reference and a stale state are two different things to chase -- and the motion is "
  "failed rather than held, because a silent hold must not look like success.";

constexpr char kInnerLoopUnexpectedFault[] =
  "state health: the inner velocity loop reported a fault code it is not supposed to be able to "
  "raise. Its own header names three -- FAULT_STATE_HEALTH, FAULT_REFERENCE_STALE and "
  "FAULT_NOT_COMMISSIONED -- and crane_velocity_controller static_asserts all three against the "
  "frozen crane_msgs/SupervisorStatus constants, so a fourth means the two have drifted apart. "
  "The code is carried through unedited, because inventing a cause here would hide the drift.";

constexpr char kNotCommissionedHead[] =
  "not commissioned: the inner velocity loop is running with the feedforward disabled and PI only "
  "on ";

constexpr char kNotCommissionedTail[] =
  ". That is a missing identified map and not a stale or degraded measurement, which is why it is "
  "its own constant: what it asks for is a calibration, or someone who can run one, and not a "
  "sensor check (wiki/implementation/commissioning_prerequisites.md 1, 2). It is reported below "
  "every health cause for the same reason -- a missing calibration will still be missing next "
  "cycle, while a state that just went stale is the one to act on now -- and only on the hardware "
  "profile, because a rig with no hydraulics has nothing to commission. Nothing was stopped, "
  "ramped or commanded here.";

constexpr char kNotCommissionedNoAxis[] =
  "not commissioned: the inner velocity loop reports a missing commissioning prerequisite but "
  "named no axis for it, which is itself a defect. crane_msgs/VelocityControllerHealth carries a "
  "feedforward flag per axis with the joint name beside it precisely so that this report can say "
  "which calibration is missing rather than that one is.";

constexpr char kNoticeHead[] =
  "no velocity-tracking tolerance for ";

constexpr char kNoticeTail[] =
  ". Those axes raise no FAULT_TRACKING and tracking_error is still reported for them as a "
  "measurement. A value that is not finite and positive is not a tolerance, and this supervisor "
  "does not stand one at a plausible default: the number comes from merge gate (ii-b) or from the "
  "identification campaign (PRD 14), both human-only, and it goes in "
  "crane_control/config/tracking_tolerance.yaml -- read by the seam clamp, by the MPC's "
  "constraint margin and by this supervisor, so that one number cannot become three.";

// The emergency stop and the deadman. Every one of these says the same thing
// about what it is for, because the sentence is the point: this software path is
// diagnosis and clean recovery (wiki/control_architecture.md §6.1), the hardware
// and PLC chain is the protection, and an operator reading FAULT_ESTOP off a
// panel must not take the report for the function.
constexpr char kDiagnosisNotProtection[] =
  " This report is diagnosis and recovery, not protection: the stop chain is hardware and PLC, it "
  "acts whether or not this software is running, and nothing here stopped, ramped or deactivated "
  "anything.";

constexpr char kStopAsserted[] =
  "emergency stop: em_stop is asserted on /crane/remote_ctrl_states. The machine has been stopped "
  "by the chain below this stack; the supervisor latches the fault so that the software comes back "
  "in a defined state rather than resuming from what it was doing. Release the stop, then "
  "acknowledge it on /crane/clear_fault.";

constexpr char kStopLatched[] =
  "emergency stop, latched: em_stop is no longer asserted and /crane/remote_ctrl_states is "
  "arriving again, but the stop has not been acknowledged, so it survives the signal returning to "
  "released. This is the one latched cause on this stream -- every other input clears its own "
  "fault the moment it arrives again inside its deadline. Acknowledge it on /crane/clear_fault "
  "when the machine is where you expect it to be.";

constexpr char kDeadmanReleased[] =
  "interlock: the operator deadman -- button ";

constexpr char kDeadmanReleasedTail[] =
  " of epsilon_crane_msgs/RemoteCtrlStates, per the retained approval gate -- is not held on "
  "/crane/remote_ctrl_states. The check runs on every status cycle rather than once at the start "
  "of a motion, so this clears itself the cycle after the button is pressed and needs no "
  "acknowledgement.";

// The three answers /crane/clear_fault can give. None of them is an empty
// success (ROS 2 Interfaces §1): the refusal names the condition that is still
// true, and the clear says what it lowered and what may happen next.
constexpr char kClearRefusedAsserted[] =
  "refused: em_stop is still asserted on /crane/remote_ctrl_states. The latch is not what is "
  "holding the fault up -- the condition behind it is -- so clearing it now would report a success "
  "the next status cycle undoes. Release the stop on the machine, then acknowledge it here.";

constexpr char kClearRefusedAbsent[] =
  "refused: the operator remote is not arriving on /crane/remote_ctrl_states, and absence of the "
  "stop signal is treated as asserted rather than as released. Restore the signal -- the current "
  "status message says which way it is missing -- and acknowledge it here once it is back.";

constexpr char kClearNothingLatched[] =
  "nothing to acknowledge: no emergency stop is latched. The call is a no-op and is reported as "
  "one rather than as a clear, so that an acknowledgement of nothing is not mistaken for an "
  "acknowledgement of something.";

constexpr char kCleared[] =
  "cleared: the latched emergency stop is acknowledged and lowered. em_stop is released and "
  "/crane/remote_ctrl_states is arriving as of the newest sample this supervisor holds; if either "
  "changes the latch is raised again on the next status cycle, which is the specified behaviour "
  "and not a failed clear. Nothing was started, resumed or commanded by this call.";

/// Seconds as text, to the millisecond.
/**
 * `std::to_string` prints six decimals of a number this stack knows to three,
 * and a status line an operator reads is the wrong place for three digits of
 * false precision.
 */
std::string seconds_text(double seconds)
{
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.3f", seconds);
  return std::string(buffer);
}

/// A joint-space quantity as text. Four decimals, which is a tenth of a
/// milliradian and a tenth of a millimetre -- finer than anything this machine
/// resolves, and coarse enough not to print a double's tail at an operator.
std::string quantity_text(double value)
{
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.4f", value);
  return std::string(buffer);
}

/// The unit of one axis, from its URDF name.
/**
 * ROS 2 Interfaces §3.1 splits the actuated set into four `theta*` revolute
 * axes and two `q*` prismatic ones, and the split is what makes the max in
 * `tracking_error` a mixed-unit number. The unit is read off the name rather
 * than tabulated, because a table would be a second place for the split to be
 * written down and the naming rule is nomenclature §4's, not this file's.
 */
const char * angular_or_linear(const std::string & joint, bool per_second)
{
  const bool angular = !joint.empty() && joint.front() == 't';
  if (per_second) {
    return angular ? "rad/s" : "m/s";
  }
  return angular ? "rad" : "m";
}

/// How a report about one fault opens, so an operator reads the constant in
/// words before reading the observation behind it.
const char * fault_head(Fault fault)
{
  switch (fault) {
    case Fault::EStop:
      return "emergency stop: ";
    case Fault::StateHealth:
      return "state health: ";
    default:
      break;
  }
  return "fault: ";
}

/// `a`, `a and b`, `a, b and c` -- a list an operator reads as a sentence.
std::string joined(const std::vector<std::string> & items)
{
  std::string text;
  for (std::size_t i = 0; i < items.size(); ++i) {
    if (i > 0) {
      text += (i + 1 == items.size()) ? " and " : ", ";
    }
    text += items[i];
  }
  return text;
}

/// The account of the emergency stop, whichever of its causes holds.
/**
 * The absences go through the same generator every other input's do, because
 * §6.1's rule that a dead reader and a released button must not look alike is
 * §5.3's rule with a different constant on it. What is added here is the
 * sentence that must be on every one of them: this is diagnosis, not protection.
 *
 * `Fresh` reaches this only through the latch, which is the case where the
 * condition is gone and the acknowledgement is what is still owed.
 */
std::string stop_message(
  const SupervisorConfig & config, const SupervisorInput & input, Staleness cause)
{
  if (cause != Staleness::Fresh) {
    return staleness_message(
      config, Input::RemoteCtrl, cause, input.stream(Input::RemoteCtrl), {}) +
           kDiagnosisNotProtection;
  }
  if (input.remote_ctrl.em_stop) {
    return std::string(kStopAsserted) + kDiagnosisNotProtection;
  }
  return std::string(kStopLatched) + kDiagnosisNotProtection;
}

/// Why no axis was compared this cycle, or empty when at least one was.
/**
 * Two different absences, and they are not the same thing to chase: nobody has
 * measured the tolerance yet, or the tolerance exists and the controller is not
 * publishing the quantity it applies to.
 */
std::string nothing_compared_reason(
  const SupervisorConfig & config, const ControllerStateReport & report)
{
  bool any_tolerance = false;
  for (const AxisTolerance & tolerance : config.tracking_tolerance) {
    if (!is_tolerance(tolerance.dq_a)) {
      continue;
    }
    any_tolerance = true;
    for (const AxisError & axis : report.axes) {
      if (axis.joint == tolerance.joint && axis.velocity_error_reported) {
        return {};
      }
    }
  }
  return any_tolerance ? kNoVelocityErrorReported : kNoToleranceAtAll;
}

/// The inner loop's own account of one of its codes, in the operator's terms.
/**
 * `NotCommissioned` is deliberately absent: it is answered where the axes it
 * names are available, and it is reported at a different place in the order.
 */
const char * inner_loop_message(Fault fault)
{
  switch (fault) {
    case Fault::StateHealth:
      return kInnerLoopStateHealth;
    case Fault::ReferenceStale:
      return kInnerLoopReferenceStale;
    default:
      break;
  }
  return kInnerLoopUnexpectedFault;
}

/// One axis of a tracking fault, in the operator's terms and with its unit.
std::string breach_text(const TrackingBreach & breach)
{
  return breach.joint + " (velocity error " + quantity_text(breach.velocity_error) + " " +
         angular_or_linear(breach.joint, true) + ", tolerance " + quantity_text(breach.tolerance) +
         " " + angular_or_linear(breach.joint, true) + ")";
}

}  // namespace

bool validate(const SupervisorConfig & config, std::string & reason)
{
  // Over `Input` rather than over a list written here, so that an input added to
  // the enum is checked without this function being touched -- and, until
  // somebody gives it a margin, refused. A deadline that were merely *absent*
  // would make that input exempt from §5.3, which is the one failure a warning
  // could not catch, because there would be nothing to warn about.
  for (std::size_t i = 0; i < kInputCount; ++i) {
    const InputPolicy & policy = kInputPolicies[i];
    const double deadline = config.freshness_deadline[i];
    if (!std::isfinite(deadline) || deadline <= 0.0) {
      reason = std::string("the freshness deadline of ") + policy.label + " (" + policy.type +
        " on " + policy.topic + ") is " + seconds_text(deadline) +
        " s, and a deadline is a finite positive number of seconds: zero or less would report "
        "every sample of it stale, and one that is not a number would report none of them, which "
        "would leave that input with no defined consequence for stopping at all "
        "(wiki/control_architecture.md 5.3). Every input this supervisor holds has a deadline and "
        "none is exempt; the numbers and their derivations are in "
        "src/crane_supervisor_parameters.yaml.";
      return false;
    }
  }
  for (const AxisTolerance & axis : config.tracking_tolerance) {
    if (axis.joint.empty()) {
      reason =
        "every tracking tolerance names the URDF joint it belongs to; an unnamed one can never be "
        "paired with an axis of the trajectory controller's joint_names and would be a tolerance "
        "that silently applies to nothing";
      return false;
    }
  }
  // Refused rather than reported, unlike the tracking tolerance above: these
  // numbers ship with the package, so a deployment without one has been
  // misconfigured and a supervisor that went on publishing would report
  // FAULT_NONE for a duty it had quietly stopped performing.
  if (!validate_sway(config.sway, reason)) {
    return false;
  }
  if (config.deadman_button < kFirstButton || config.deadman_button > kLastButton) {
    reason =
      "deadman_button must name one of the twelve booleans of "
      "epsilon_crane_msgs/RemoteCtrlStates, so it lies between 1 and 12; a number outside them "
      "selects no field, and a deadman read off no field would be released forever";
    return false;
  }
  return true;
}

Staleness freshness_of(double deadline, const StreamReport & stream) noexcept
{
  if (!stream.received) {
    return Staleness::NeverArrived;
  }
  if (stream.age > deadline) {
    return Staleness::StoppedArriving;
  }
  if (stream.age < -deadline) {
    return Staleness::StampAhead;
  }
  return Staleness::Fresh;
}

std::array<Staleness, kInputCount> freshness(
  const SupervisorConfig & config, const SupervisorInput & input) noexcept
{
  std::array<Staleness, kInputCount> causes{};
  for (std::size_t i = 0; i < kInputCount; ++i) {
    causes[i] = freshness_of(config.freshness_deadline[i], input.streams[i]);
  }
  return causes;
}

std::string staleness_message(
  const SupervisorConfig & config, Input input, Staleness cause, const StreamReport & stream,
  const std::string & carried)
{
  const InputPolicy & policy = policy_of(input);
  const std::string deadline = seconds_text(config.deadline(input));
  std::string text = fault_head(policy.fault);

  switch (cause) {
    case Staleness::NeverArrived:
      text += std::string("no ") + policy.type + " has arrived on " + policy.topic +
        " since this supervisor started. Staleness cause: age -- and there is no sample to age, "
        "because this input never connected. An input that never came up and one that was "
        "arriving and died are both faults and are not the same fault to chase. " +
        policy.consequence + kAbsenceIsNotHealth + policy.advice;
      break;
    case Staleness::StoppedArriving:
      text += std::string(policy.label) + " " + policy.stopped +
        ". Staleness cause: age -- the newest " + policy.type + " on " + policy.topic + " is " +
        seconds_text(stream.age) + " s old, past this input's freshness deadline of " + deadline +
        " s. It was arriving before, which is not the same fault as an input that never "
        "connected. " + policy.consequence + " " + policy.advice;
      break;
    case Staleness::StampAhead:
      text += std::string("the age of ") + policy.label +
        " cannot be judged. Staleness cause: age -- the newest " + policy.type + " on " +
        policy.topic + " is stamped " + seconds_text(-stream.age) +
        " s in this supervisor's future, further ahead than this input's freshness deadline of " +
        deadline +
        " s, so its age is not a measurement of anything. Synchronise the clock of the host "
        "publishing it with this one; until then no staleness answer about that stream means "
        "anything (PRD user story 59). " + policy.consequence;
      break;
    case Staleness::ProducerUnhealthy:
      text += std::string(policy.label) + " arrived inside its freshness deadline of " + deadline +
        " s and " + policy.producer +
        " marks it unusable. Staleness cause: the producer's own health flag, which is the one "
        "cause of the three this supervisor cannot measure off a topic. " + policy.producer +
        " separates the flag, a sample that stopped refreshing behind a header that keeps moving, "
        "and the measurement age, and says which in its own status string, so the string is "
        "carried here unedited rather than restated: " +
        (carried.empty() ? std::string(kNoStatusGiven) : carried);
      break;
    case Staleness::Fresh:
      text += std::string(policy.label) + " is arriving inside its freshness deadline of " +
        deadline +
        " s and a staleness report was composed for it anyway. That is a defect in this "
        "supervisor rather than in the stream, and it is reported as one rather than left as a "
        "fault with no cause behind it.";
      break;
  }
  return text;
}

bool is_tolerance(double dq_a) noexcept
{
  return std::isfinite(dq_a) && dq_a > 0.0;
}

std::string tracking_tolerance_notice(const SupervisorConfig & config)
{
  std::vector<std::string> missing;
  for (const AxisTolerance & axis : config.tracking_tolerance) {
    if (!is_tolerance(axis.dq_a)) {
      missing.push_back(axis.joint);
    }
  }
  if (missing.empty()) {
    return {};
  }
  return kNoticeHead + joined(missing) + kNoticeTail;
}

double max_position_error(const ControllerStateReport & report) noexcept
{
  double worst = 0.0;
  for (const AxisError & axis : report.axes) {
    worst = std::max(worst, std::abs(axis.position_error));
  }
  return worst;
}

std::vector<TrackingBreach> tracking_breaches(
  const SupervisorConfig & config, const ControllerStateReport & report)
{
  std::vector<TrackingBreach> breaches;
  for (const AxisTolerance & tolerance : config.tracking_tolerance) {
    if (!is_tolerance(tolerance.dq_a)) {
      continue;
    }
    for (const AxisError & axis : report.axes) {
      if (axis.joint != tolerance.joint || !axis.velocity_error_reported) {
        continue;
      }
      if (std::abs(axis.velocity_error) > tolerance.dq_a) {
        breaches.push_back({tolerance.joint, axis.velocity_error, tolerance.dq_a});
      }
      break;
    }
  }

  std::stable_sort(
    breaches.begin(), breaches.end(),
    [](const TrackingBreach & left, const TrackingBreach & right) {
      return std::abs(left.velocity_error) / left.tolerance >
             std::abs(right.velocity_error) / right.tolerance;
    });
  return breaches;
}

namespace
{

/// The cause of one cycle, with the sway already judged.
/**
 * The precedence chain, and nothing else. It is split out from `decide()` so
 * that the settled clause every report ends in is appended in exactly one place:
 * a clause added at each of the ten `return`s below is a clause the eleventh
 * would be missing.
 */
SupervisorDecision resolve(
  const SupervisorConfig & config, const SupervisorInput & input,
  const std::array<Staleness, kInputCount> & staleness, const SwayVerdict & sway)
{
  SupervisorDecision decision;
  // Mode arbitration is a later issue and this supervisor has verified nothing,
  // so it reports the mode it can defend and no other.
  decision.mode = Mode::Idle;

  const Staleness remote_cause = staleness[index_of(Input::RemoteCtrl)];

  const PendulumStateReport & state = input.pendulum_state;
  const ControllerStateReport & controller = input.controller_state;
  const ControllerHealthReport & inner_loop = input.controller_health;

  // Filled before any branch returns, for the same reason `deadman_held` is: it
  // is a field of its own on every report, so the largest deviation on the crane
  // stays visible in the cycles where a more consequential cause owns `fault`.
  // Zero while the stream is not arriving -- and that case is itself a fault
  // below, so the zero is never the only thing said about it.
  if (staleness[index_of(Input::ControllerState)] == Staleness::Fresh) {
    decision.tracking_error = max_position_error(controller);
  }

  // Filled before any branch returns, because it is owed on *every* status and
  // not only on the ones where the deadman is what went wrong. It is also the
  // reason the interlock can sit below state health below: a released deadman
  // stays visible in this field even in a cycle whose `fault` is something else,
  // where a state-health fault that lost the field would be invisible.
  decision.deadman_held = remote_cause == Staleness::Fresh && input.remote_ctrl.deadman_held;

  // §6.1: asserted latches, and absence is asserted. The latch is raised here
  // and lowered in exactly one place -- an acknowledged `/crane/clear_fault` --
  // and it is the only input whose fault does not clear itself, which is what
  // `InputPolicy::latches` records.
  decision.estop_latched =
    input.estop_latched || remote_cause != Staleness::Fresh || input.remote_ctrl.em_stop;

  // The stop outranks everything else this supervisor watches, and it has to:
  // it is the only cause here with no field of its own, so a cycle that reported
  // something else instead would not report it at all. A stale passive state is
  // a defect in a stream; a stop is the machine having been stopped.
  if (decision.estop_latched) {
    decision.fault = policy_of(Input::RemoteCtrl).fault;
    decision.message = stop_message(config, input, remote_cause);
    return decision;
  }

  // Resolved in the order of how much of the estimate the cause removes, the
  // same ordering pendulum_state_broadcaster resolves its own causes in: an
  // input that is not there leaves nothing to judge, one whose age is unknowable
  // leaves numbers nobody can place in time, and one that arrived leaves numbers
  // the estimator has already had its say about.
  if (staleness[index_of(Input::PendulumState)] != Staleness::Fresh) {
    decision.fault = policy_of(Input::PendulumState).fault;
    decision.message = staleness_message(
      config, Input::PendulumState, staleness[index_of(Input::PendulumState)],
      input.stream(Input::PendulumState), {});
    return decision;
  }

  // The one cause of the three this supervisor cannot measure off a topic: the
  // producer's own flag. Its account is carried through rather than restated,
  // because the broadcaster separates six causes behind that flag -- the sample
  // that stopped refreshing among them -- and restating it here would flatten a
  // distinction the estimator went to some trouble to make.
  if (!state.valid) {
    decision.fault = policy_of(Input::PendulumState).fault;
    decision.message = staleness_message(
      config, Input::PendulumState, Staleness::ProducerUnhealthy,
      input.stream(Input::PendulumState), state.status);
    return decision;
  }

  // The two remaining inputs, judged before anything derived from them is:
  // §5.3 allows no input to stop arriving without a defined consequence, and
  // here the consequences are that the tracking error is unmeasured rather than
  // zero, and that the inner loop's verdict on itself is unread rather than
  // clear. Both are swept in a loop over the registry, so a fifth input judged
  // by the same rule needs no fifth branch.
  for (const Input stream_input : {Input::ControllerState, Input::ControllerHealth}) {
    const Staleness cause = staleness[index_of(stream_input)];
    if (cause != Staleness::Fresh) {
      decision.fault = policy_of(stream_input).fault;
      decision.message =
        staleness_message(config, stream_input, cause, input.stream(stream_input), {});
      return decision;
    }
  }

  // The inner loop's *health* codes, merged as the loop numbered them. Above
  // tracking for the reason the passive state is above it -- a supervisor whose
  // view of the crane is degraded should say so before it says anything derived
  // -- and above the commissioning code for the reason
  // wiki/implementation/commissioning_prerequisites.md §2 gives: the missing
  // calibration will still be missing next cycle, and the state that just went
  // stale is the one an operator has to act on now.
  if (inner_loop.fault != Fault::None && inner_loop.fault != Fault::NotCommissioned) {
    decision.fault = inner_loop.fault;
    decision.message = inner_loop_message(inner_loop.fault);
    return decision;
  }

  // §5 row 7, as a typed cause. Below every health cause, because a supervisor
  // whose own view of the crane is degraded should say that before it says
  // anything derived from it -- and a degraded estimate cannot reach here at all,
  // since `judge_sway()` raises no breach off a rate it was told not to trust.
  // Above tracking, because of the two this is the one with no field of its own:
  // `tracking_error` is filled on every report whatever `fault` says, so a
  // tracking excess stays visible in a cycle sway owns, while a swinging load
  // reported behind a tracking fault would be invisible. It is the same rule
  // that puts the commissioning code above the interlock.
  if (!sway.breaches.empty()) {
    decision.fault = Fault::Sway;
    decision.message = sway_breach_message(sway.breaches);
    return decision;
  }

  // §5 row 1, as a typed cause. Per axis and in the tolerance's own unit,
  // because a max over rad/s and m/s decides nothing; the single number on the
  // wire is the indicator and this is the verdict. The decision about what to do
  // next is not made here and is not made by this supervisor at all (§5.0).
  const std::vector<TrackingBreach> breaches = tracking_breaches(config, controller);
  if (!breaches.empty()) {
    std::vector<std::string> texts;
    texts.reserve(breaches.size());
    for (const TrackingBreach & breach : breaches) {
      texts.push_back(breach_text(breach));
    }
    decision.fault = Fault::Tracking;
    decision.message = kTrackingExceeded + joined(texts) + kTrackingExceededTail;
    return decision;
  }

  // The commissioning code, below every health cause and below tracking, and
  // above the interlock. Above it deliberately: on the hardware profile this
  // condition is *standing* -- prerequisite 4 will hold until someone records a
  // calibration -- while a released deadman is the ordinary resting state of the
  // machine, so putting the interlock first would hide the report this stream
  // exists to deliver behind the routine. Nothing is lost by the ordering,
  // because `deadman_held` is a field of its own on every report and the
  // commissioning code has none.
  if (inner_loop.fault == Fault::NotCommissioned) {
    decision.fault = Fault::NotCommissioned;
    decision.message = inner_loop.feedforward_free_joints.empty()
      ? kNotCommissionedNoAxis
      : kNotCommissionedHead + joined(inner_loop.feedforward_free_joints) + kNotCommissionedTail;
    return decision;
  }

  // Last of the five, and continuous rather than checked once at the start of a
  // motion (§6.2). It is last because it is the only one of them that is a fact
  // about the operator rather than a defect: a released deadman is the ordinary
  // resting state of the machine, and letting it outrank a dead publisher would
  // hide the defect behind the routine.
  if (!decision.deadman_held) {
    decision.fault = Fault::Interlock;
    decision.message =
      kDeadmanReleased + std::to_string(config.deadman_button) + kDeadmanReleasedTail;
    return decision;
  }

  decision.fault = Fault::None;
  decision.message = kObserving;
  // A clear report must not imply a check nobody made. When no axis could be
  // compared, the sentence above is amended rather than left to stand.
  const std::string uncompared = nothing_compared_reason(config, controller);
  if (!uncompared.empty()) {
    decision.message += kNoAxisCompared + uncompared;
  }
  return decision;
}

}  // namespace

SupervisorDecision decide(const SupervisorConfig & config, const SupervisorInput & input)
{
  // One pass over every input, on the status cycle: `kInputCount` comparisons,
  // no allocation, and no dependence on a message arriving. A deadline that were
  // evaluated only in a subscription callback could never fire, because the case
  // it exists for is the one where no callback runs again
  // (wiki/control_architecture.md §5.3).
  const std::array<Staleness, kInputCount> staleness = freshness(config, input);

  // The one place that decides whether the passive estimate is to be believed,
  // and it is the same answer the precedence chain reports on: the stream inside
  // its own deadline *and* the broadcaster marking the sample usable. A second
  // opinion here is how a rate gets judged against a bound in a cycle whose
  // report says the estimate was unusable.
  const bool estimate_trusted =
    staleness[index_of(Input::PendulumState)] == Staleness::Fresh && input.pendulum_state.valid;

  // Judged before the chain runs, not inside it, for the reason `deadman_held`
  // and `tracking_error` are filled before any branch returns: the dwell is owed
  // on every cycle, and one that stopped advancing whenever something more
  // consequential owned `fault` would restart every time the operator let go of
  // the deadman.
  const SwayVerdict sway = judge_sway(
    config.sway, input.pendulum_state.velocity, estimate_trusted, input.sampled_at, input.sway);

  SupervisorDecision decision = resolve(config, input, staleness, sway);
  decision.sway = sway.state;
  // On every report, whatever `fault` says. The predicate has no field of its
  // own on `crane_msgs/SupervisorStatus`, so this clause is the only thing that
  // carries it onto the wire -- which is why it is appended here, once, rather
  // than at each of the branches above.
  decision.message += settled_clause(config.sway, input.pendulum_state.velocity, sway);
  return decision;
}

ClearFaultOutcome clear_fault(const SupervisorConfig & config, const SupervisorInput & input)
{
  ClearFaultOutcome outcome;

  if (!input.estop_latched) {
    outcome.cleared = false;
    outcome.message = kClearNothingLatched;
    return outcome;
  }

  // Judged from the same view of the same signal `decide()` uses, and against
  // the same deadline, so that the acknowledgement and the next status cycle
  // cannot disagree about whether the condition is still there.
  const Staleness cause =
    freshness_of(config.deadline(Input::RemoteCtrl), input.stream(Input::RemoteCtrl));
  if (cause == Staleness::Fresh && input.remote_ctrl.em_stop) {
    outcome.cleared = false;
    outcome.message = kClearRefusedAsserted;
    return outcome;
  }
  if (cause != Staleness::Fresh) {
    outcome.cleared = false;
    outcome.message = kClearRefusedAbsent;
    return outcome;
  }

  outcome.cleared = true;
  outcome.message = kCleared;
  return outcome;
}

}  // namespace crane_supervisor
