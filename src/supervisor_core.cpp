#include "crane_supervisor/supervisor_core.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <string>
#include <vector>

namespace crane_supervisor
{
namespace
{

// The fixed halves of the four messages, as literals. Each one names the cause,
// the observation behind it and the thing an operator is supposed to do next --
// wiki/control_architecture.md §5.0's whole point is that the task layer and the
// panel branch on information rather than on an inferred abort, and a cause with
// no account of itself is the inference again with extra steps.
constexpr char kNothingArrived[] =
  "state health: no crane_msgs/PendulumState has arrived on /crane/pendulum_state since this "
  "supervisor started. Absence is not health -- an input that never arrived is reported as a "
  "fault rather than left at FAULT_NONE -- so the passive joint state counts as unavailable. "
  "Check that pendulum_state_broadcaster is loaded and active on the controller manager.";

constexpr char kStoppedArriving[] =
  "state health: the passive joint state stopped arriving. The newest crane_msgs/PendulumState "
  "on /crane/pendulum_state is ";

constexpr char kStoppedArrivingTail[] =
  " s old, past the configured margin of ";

constexpr char kStoppedArrivingAdvice[] =
  " s. A publisher that died must not be indistinguishable from a healthy one: check the "
  "controller manager's cycle and pendulum_state_broadcaster before trusting anything that "
  "closes on the passive state.";

constexpr char kStampAhead[] =
  "state health: the age of the passive joint state cannot be judged. The newest "
  "crane_msgs/PendulumState on /crane/pendulum_state is stamped ";

constexpr char kStampAheadTail[] =
  " s in this supervisor's future, further ahead than the configured margin of ";

constexpr char kStampAheadAdvice[] =
  " s. Synchronise the clock of the host publishing it with this one; until then no staleness "
  "answer about that stream means anything.";

constexpr char kMarkedUnusable[] =
  "state health: the passive joint state arrived in time and pendulum_state_broadcaster marks "
  "it unusable. Its own account of the cause, unedited: ";

constexpr char kNoStatusGiven[] =
  "(the broadcaster set no status string, which is itself a defect -- "
  "crane_msgs/PendulumState carries the cause in that field)";

constexpr char kObserving[] =
  "no fault: the passive joint state is arriving inside its margin and pendulum_state_broadcaster "
  "reports it usable, the trajectory controller's own state is arriving and no axis is outside "
  "its tolerance, the inner velocity loop is arriving and reports no fault of its own, the "
  "operator remote is arriving, its emergency stop is released and nothing is latched, and the "
  "deadman is held. Those are the only inputs this supervisor watches -- working cell, solver "
  "and sway are not observed yet, inside_working_cell is not computed and carries the value that "
  "claims nothing, and the supervisor holds no stop authority until the hardware stop input is "
  "verified. Read FAULT_NONE as 'nothing this supervisor watches is wrong', not as 'the machine "
  "is safe'.";

// The trajectory controller's own state publication, and the tracking duty of
// wiki/control_architecture.md §5 row 1 that is decided from it. Three of these
// are §5.3's rule applied to a third input -- a controller that stopped
// publishing its error is not a crane that is tracking perfectly -- and the
// fourth is the typed cause §5.0 exists to produce.
constexpr char kControllerStateNeverArrived[] =
  "state health: no control_msgs/JointTrajectoryControllerState has arrived on "
  "/crane/controller_state since this supervisor started, so the tracking error is not being "
  "measured at all and tracking_error carries 0.0 as the absence of a measurement rather than as "
  "perfect tracking. Check that the trajectory controller is loaded and active on the controller "
  "manager and that its private controller_state output reaches the contract name.";

constexpr char kControllerStateStoppedArriving[] =
  "state health: the trajectory controller stopped publishing its own state, so the tracking "
  "error is no longer measured. The newest control_msgs/JointTrajectoryControllerState on "
  "/crane/controller_state is ";

constexpr char kControllerStateStoppedArrivingTail[] =
  " s old, past the configured margin of ";

constexpr char kControllerStateStoppedArrivingAdvice[] =
  " s. A controller that died must not look like one that is tracking perfectly, so this is a "
  "fault and not a tracking_error of zero: check the controller manager's cycle and whether the "
  "trajectory controller is still active.";

constexpr char kControllerStateStampAhead[] =
  "state health: the age of the trajectory controller's state cannot be judged. The newest "
  "control_msgs/JointTrajectoryControllerState on /crane/controller_state is stamped ";

constexpr char kControllerStateStampAheadTail[] =
  " s in this supervisor's future, further ahead than the configured margin of ";

constexpr char kControllerStateStampAheadAdvice[] =
  " s. Synchronise the clock of the host publishing it with this one; until then a tracking error "
  "measured a moment ago and one measured a minute ago are indistinguishable.";

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

// The inner velocity loop's own report. Three of these are §5.3's rule applied
// to a fourth input, and the rest are the loop's three codes carried through
// rather than restated: the controller computes them from the state interfaces
// it claims itself and from the identified map of the tool it is driving, and
// nothing above the controller manager can see either.
constexpr char kControllerHealthNeverArrived[] =
  "state health: no crane_msgs/VelocityControllerHealth has arrived on "
  "/crane/velocity_controller/health since this supervisor started, so the inner velocity loop's "
  "verdict on itself is not being read at all. Absence is not health -- an input that never "
  "arrived is reported as a fault rather than left at FAULT_NONE -- and in particular a "
  "commissioning prerequisite the controller is reporting would be reaching nobody, which is the "
  "state the whole stream exists to end. Check that crane_velocity_controller is loaded and "
  "active on the controller manager.";

constexpr char kControllerHealthStoppedArriving[] =
  "state health: the inner velocity loop stopped reporting its own health. The newest "
  "crane_msgs/VelocityControllerHealth on /crane/velocity_controller/health is ";

constexpr char kControllerHealthStoppedArrivingTail[] =
  " s old, past the configured margin of ";

constexpr char kControllerHealthStoppedArrivingAdvice[] =
  " s. A controller that stopped publishing must not be indistinguishable from one that is "
  "healthy and commissioned: check the controller manager's cycle and whether "
  "crane_velocity_controller is still active.";

constexpr char kControllerHealthStampAhead[] =
  "state health: the age of the inner velocity loop's health report cannot be judged. The newest "
  "crane_msgs/VelocityControllerHealth on /crane/velocity_controller/health is stamped ";

constexpr char kControllerHealthStampAheadTail[] =
  " s in this supervisor's future, further ahead than the configured margin of ";

constexpr char kControllerHealthStampAheadAdvice[] =
  " s. Synchronise the clock of the host publishing it with this one; until then a fault the "
  "inner loop raised a moment ago and one it raised a minute ago are indistinguishable.";

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
  "state-health fault: the producer is gone. The motion is failed rather than held, and a silent "
  "hold must not look like success.";

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

constexpr char kStopSignalNeverArrived[] =
  "emergency stop: no epsilon_crane_msgs/RemoteCtrlStates has arrived on /crane/remote_ctrl_states "
  "since this supervisor started, and absence of the stop signal is treated as asserted rather "
  "than as released. A dead GPIO reader, a crashed driver and a released button must not look "
  "alike. Check that gpio_controller is loaded and active on the controller manager and that its "
  "remote_ctrl_states output reaches the contract name.";

constexpr char kStopSignalStoppedArriving[] =
  "emergency stop: the operator remote stopped arriving, which is treated as asserted rather than "
  "as released. The newest epsilon_crane_msgs/RemoteCtrlStates on /crane/remote_ctrl_states is ";

constexpr char kStopSignalStoppedArrivingTail[] =
  " s old, past the configured margin of ";

constexpr char kStopSignalStoppedArrivingAdvice[] =
  " s. A reader that died must not be indistinguishable from a released button, so this is the "
  "stop and not an interlock: check gpio_controller and the transport before trusting either the "
  "stop or the deadman.";

constexpr char kStopSignalStampAhead[] =
  "emergency stop: the age of the operator remote cannot be judged, which is treated as asserted "
  "rather than as released. The newest epsilon_crane_msgs/RemoteCtrlStates on "
  "/crane/remote_ctrl_states is stamped ";

constexpr char kStopSignalStampAheadTail[] =
  " s in this supervisor's future, further ahead than the configured margin of ";

constexpr char kStopSignalStampAheadAdvice[] =
  " s. Synchronise the clock of the host publishing it with this one; until then a stale stop "
  "signal and a fresh one are indistinguishable, and the safe reading of the two is asserted.";

constexpr char kStopAsserted[] =
  "emergency stop: em_stop is asserted on /crane/remote_ctrl_states. The machine has been stopped "
  "by the chain below this stack; the supervisor latches the fault so that the software comes back "
  "in a defined state rather than resuming from what it was doing. Release the stop, then "
  "acknowledge it on /crane/clear_fault.";

constexpr char kStopLatched[] =
  "emergency stop, latched: em_stop is no longer asserted and /crane/remote_ctrl_states is "
  "arriving again, but the stop has not been acknowledged, so it survives the signal returning to "
  "released. Acknowledge it on /crane/clear_fault when the machine is where you expect it to be.";

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

/// What a stream is doing, with never-started separated from stopped.
/**
 * The same three absences the passive state is judged by, named once so the
 * trajectory controller's state is judged the same way rather than by a second
 * copy of the same three comparisons.
 */
enum class StreamState
{
  NeverArrived,
  StoppedArriving,
  StampAhead,
  Arriving,
};

StreamState stream_state(double timeout, bool received, double age)
{
  if (!received) {
    return StreamState::NeverArrived;
  }
  if (age > timeout) {
    return StreamState::StoppedArriving;
  }
  if (age < -timeout) {
    return StreamState::StampAhead;
  }
  return StreamState::Arriving;
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

/// What the stop signal is doing, with absence separated from assertion.
/**
 * Four of the five are §6.1's "treat absence as asserted" and only `Released`
 * is not, so the enum is the place where a dead reader, a crashed driver, a
 * clock fault, an asserted stop and a released one stop being interchangeable.
 */
enum class StopSignal
{
  NeverArrived,
  StoppedArriving,
  StampAhead,
  Asserted,
  Released,
};

StopSignal stop_signal(const SupervisorConfig & config, const RemoteCtrlReport & remote)
{
  if (!remote.received) {
    return StopSignal::NeverArrived;
  }
  if (remote.age > config.remote_ctrl_timeout) {
    return StopSignal::StoppedArriving;
  }
  if (remote.age < -config.remote_ctrl_timeout) {
    return StopSignal::StampAhead;
  }
  if (remote.em_stop) {
    return StopSignal::Asserted;
  }
  return StopSignal::Released;
}

/// True when the newest sample is one whose booleans mean anything.
bool signal_is_arriving(StopSignal signal)
{
  return signal == StopSignal::Asserted || signal == StopSignal::Released;
}

/// The account of one stop condition, in the operator's terms.
/**
 * `Released` reaches this only through the latch, which is the case where the
 * condition is gone and the acknowledgement is what is still owed.
 */
std::string stop_message(
  const SupervisorConfig & config, StopSignal signal, const RemoteCtrlReport & remote)
{
  switch (signal) {
    case StopSignal::NeverArrived:
      return std::string(kStopSignalNeverArrived) + kDiagnosisNotProtection;
    case StopSignal::StoppedArriving:
      return kStopSignalStoppedArriving + seconds_text(remote.age) +
             kStopSignalStoppedArrivingTail + seconds_text(config.remote_ctrl_timeout) +
             kStopSignalStoppedArrivingAdvice + kDiagnosisNotProtection;
    case StopSignal::StampAhead:
      return kStopSignalStampAhead + seconds_text(-remote.age) + kStopSignalStampAheadTail +
             seconds_text(config.remote_ctrl_timeout) + kStopSignalStampAheadAdvice +
             kDiagnosisNotProtection;
    case StopSignal::Asserted:
      return std::string(kStopAsserted) + kDiagnosisNotProtection;
    case StopSignal::Released:
      break;
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
  if (!std::isfinite(config.pendulum_state_timeout) || config.pendulum_state_timeout <= 0.0) {
    reason =
      "pendulum_state_timeout must be a finite positive number of seconds; a margin of zero or "
      "less would report every sample stale and a margin that is not a number would report none "
      "of them";
    return false;
  }
  if (!std::isfinite(config.remote_ctrl_timeout) || config.remote_ctrl_timeout <= 0.0) {
    reason =
      "remote_ctrl_timeout must be a finite positive number of seconds; a margin of zero or less "
      "would hold the emergency stop asserted against a healthy remote, and one that is not a "
      "number would report a dead reader as a released button";
    return false;
  }
  if (!std::isfinite(config.controller_state_timeout) || config.controller_state_timeout <= 0.0) {
    reason =
      "controller_state_timeout must be a finite positive number of seconds; a margin of zero or "
      "less would report the trajectory controller dead on every cycle, and one that is not a "
      "number would let a controller that stopped publishing pass for a crane that is tracking "
      "perfectly";
    return false;
  }
  if (!std::isfinite(config.controller_health_timeout) || config.controller_health_timeout <= 0.0) {
    reason =
      "controller_health_timeout must be a finite positive number of seconds; a margin of zero or "
      "less would report the inner velocity loop dead on every cycle, and one that is not a "
      "number would let an inner loop that stopped publishing pass for one that is healthy and "
      "commissioned";
    return false;
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
  if (config.deadman_button < kFirstButton || config.deadman_button > kLastButton) {
    reason =
      "deadman_button must name one of the twelve booleans of "
      "epsilon_crane_msgs/RemoteCtrlStates, so it lies between 1 and 12; a number outside them "
      "selects no field, and a deadman read off no field would be released forever";
    return false;
  }
  return true;
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

SupervisorDecision decide(const SupervisorConfig & config, const SupervisorInput & input)
{
  SupervisorDecision decision;
  // Mode arbitration is a later issue and this supervisor has verified nothing,
  // so it reports the mode it can defend and no other.
  decision.mode = Mode::Idle;

  const PendulumStateReport & state = input.pendulum_state;
  const RemoteCtrlReport & remote = input.remote_ctrl;
  const ControllerStateReport & controller = input.controller_state;
  const ControllerHealthReport & inner_loop = input.controller_health;
  const StopSignal signal = stop_signal(config, remote);
  const StreamState tracking_stream =
    stream_state(config.controller_state_timeout, controller.received, controller.age);
  const StreamState inner_loop_stream =
    stream_state(config.controller_health_timeout, inner_loop.received, inner_loop.age);

  // Filled before any branch returns, for the same reason `deadman_held` is: it
  // is a field of its own on every report, so the largest deviation on the crane
  // stays visible in the cycles where a more consequential cause owns `fault`.
  // Zero while the stream is not arriving -- and that case is itself a fault
  // below, so the zero is never the only thing said about it.
  if (tracking_stream == StreamState::Arriving) {
    decision.tracking_error = max_position_error(controller);
  }

  // Filled before any branch returns, because it is owed on *every* status and
  // not only on the ones where the deadman is what went wrong. It is also the
  // reason the interlock can sit below state health below: a released deadman
  // stays visible in this field even in a cycle whose `fault` is something else,
  // where a state-health fault that lost the field would be invisible.
  decision.deadman_held = signal_is_arriving(signal) && remote.deadman_held;

  // §6.1: asserted latches, and absence is asserted. The latch is raised here
  // and lowered in exactly one place -- an acknowledged `/crane/clear_fault`.
  decision.estop_latched = input.estop_latched || signal != StopSignal::Released;

  // The stop outranks everything else this supervisor watches, and it has to:
  // it is the only cause here with no field of its own, so a cycle that reported
  // something else instead would not report it at all. A stale passive state is
  // a defect in a stream; a stop is the machine having been stopped.
  if (decision.estop_latched) {
    decision.fault = Fault::EStop;
    decision.message = stop_message(config, signal, remote);
    return decision;
  }

  // Resolved in the order of how much of the estimate the cause removes, the
  // same ordering pendulum_state_broadcaster resolves its own causes in: an
  // input that is not there leaves nothing to judge, one whose age is unknowable
  // leaves numbers nobody can place in time, and one that arrived leaves numbers
  // the estimator has already had its say about.
  if (!state.received) {
    decision.fault = Fault::StateHealth;
    decision.message = kNothingArrived;
    return decision;
  }

  if (state.age > config.pendulum_state_timeout) {
    decision.fault = Fault::StateHealth;
    decision.message = kStoppedArriving + seconds_text(state.age) + kStoppedArrivingTail +
      seconds_text(config.pendulum_state_timeout) + kStoppedArrivingAdvice;
    return decision;
  }

  if (state.age < -config.pendulum_state_timeout) {
    decision.fault = Fault::StateHealth;
    decision.message = kStampAhead + seconds_text(-state.age) + kStampAheadTail +
      seconds_text(config.pendulum_state_timeout) + kStampAheadAdvice;
    return decision;
  }

  if (!state.valid) {
    decision.fault = Fault::StateHealth;
    decision.message = kMarkedUnusable + (state.status.empty() ? kNoStatusGiven : state.status);
    return decision;
  }

  // The freshness of the input the tracking comparison consumes, judged before
  // the comparison and not after it: §5.3 allows no input to stop arriving
  // without a defined consequence, and the consequence here is that the error
  // is unmeasured rather than zero.
  switch (tracking_stream) {
    case StreamState::NeverArrived:
      decision.fault = Fault::StateHealth;
      decision.message = kControllerStateNeverArrived;
      return decision;
    case StreamState::StoppedArriving:
      decision.fault = Fault::StateHealth;
      decision.message = kControllerStateStoppedArriving + seconds_text(controller.age) +
        kControllerStateStoppedArrivingTail + seconds_text(config.controller_state_timeout) +
        kControllerStateStoppedArrivingAdvice;
      return decision;
    case StreamState::StampAhead:
      decision.fault = Fault::StateHealth;
      decision.message = kControllerStateStampAhead + seconds_text(-controller.age) +
        kControllerStateStampAheadTail + seconds_text(config.controller_state_timeout) +
        kControllerStateStampAheadAdvice;
      return decision;
    case StreamState::Arriving:
      break;
  }

  // The fourth input's own freshness, judged the same way the other three are.
  // §5.3 allows no input to stop arriving without a defined consequence, and the
  // consequence here is that the inner loop's verdict is *unread* rather than
  // clear -- an uncommissioned axis reported to nobody is the state this stream
  // exists to end, so a stream that is not arriving must not read as one that is
  // saying nothing is wrong. The general staleness policy is a later issue; this
  // is the half of it that cannot wait.
  switch (inner_loop_stream) {
    case StreamState::NeverArrived:
      decision.fault = Fault::StateHealth;
      decision.message = kControllerHealthNeverArrived;
      return decision;
    case StreamState::StoppedArriving:
      decision.fault = Fault::StateHealth;
      decision.message = kControllerHealthStoppedArriving + seconds_text(inner_loop.age) +
        kControllerHealthStoppedArrivingTail + seconds_text(config.controller_health_timeout) +
        kControllerHealthStoppedArrivingAdvice;
      return decision;
    case StreamState::StampAhead:
      decision.fault = Fault::StateHealth;
      decision.message = kControllerHealthStampAhead + seconds_text(-inner_loop.age) +
        kControllerHealthStampAheadTail + seconds_text(config.controller_health_timeout) +
        kControllerHealthStampAheadAdvice;
      return decision;
    case StreamState::Arriving:
      break;
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

ClearFaultOutcome clear_fault(const SupervisorConfig & config, const SupervisorInput & input)
{
  ClearFaultOutcome outcome;

  if (!input.estop_latched) {
    outcome.cleared = false;
    outcome.message = kClearNothingLatched;
    return outcome;
  }

  // Judged from the same view of the same signal `decide()` uses, so that the
  // acknowledgement and the next status cycle cannot disagree about whether the
  // condition is still there.
  const StopSignal signal = stop_signal(config, input.remote_ctrl);
  if (signal == StopSignal::Asserted) {
    outcome.cleared = false;
    outcome.message = kClearRefusedAsserted;
    return outcome;
  }
  if (signal != StopSignal::Released) {
    outcome.cleared = false;
    outcome.message = kClearRefusedAbsent;
    return outcome;
  }

  outcome.cleared = true;
  outcome.message = kCleared;
  return outcome;
}

}  // namespace crane_supervisor
