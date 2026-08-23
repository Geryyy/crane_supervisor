#include "crane_supervisor/supervisor_core.hpp"

#include <cmath>
#include <cstdio>
#include <string>

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
  "reports it usable, the operator remote is arriving, its emergency stop is released and nothing "
  "is latched, and the deadman is held. Those are the only inputs this supervisor watches -- "
  "tracking, working cell, solver, sway and reference are not observed yet, tracking_error and "
  "inside_working_cell are not computed and carry the value that claims nothing, and the "
  "supervisor holds no stop authority until the hardware stop input is verified. Read FAULT_NONE "
  "as 'nothing this supervisor watches is wrong', not as 'the machine is safe'.";

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
  if (config.deadman_button < kFirstButton || config.deadman_button > kLastButton) {
    reason =
      "deadman_button must name one of the twelve booleans of "
      "epsilon_crane_msgs/RemoteCtrlStates, so it lies between 1 and 12; a number outside them "
      "selects no field, and a deadman read off no field would be released forever";
    return false;
  }
  return true;
}

SupervisorDecision decide(const SupervisorConfig & config, const SupervisorInput & input)
{
  SupervisorDecision decision;
  // Mode arbitration is a later issue and this supervisor has verified nothing,
  // so it reports the mode it can defend and no other.
  decision.mode = Mode::Idle;

  const PendulumStateReport & state = input.pendulum_state;
  const RemoteCtrlReport & remote = input.remote_ctrl;
  const StopSignal signal = stop_signal(config, remote);

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

  // Last of the three, and continuous rather than checked once at the start of a
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
