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
  "reports it usable. It is the only input this supervisor watches -- tracking, working cell, "
  "solver, sway, reference, emergency stop and interlock are not observed yet, tracking_error and "
  "inside_working_cell are not computed and carry the value that claims nothing, and the "
  "supervisor holds no stop authority until the hardware stop input is verified. Read FAULT_NONE "
  "as 'nothing this supervisor watches is wrong', not as 'the machine is safe'.";

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
  return true;
}

SupervisorDecision decide(const SupervisorConfig & config, const SupervisorInput & input)
{
  SupervisorDecision decision;
  // Mode arbitration is a later issue and this supervisor has verified nothing,
  // so it reports the mode it can defend and no other.
  decision.mode = Mode::Idle;

  const PendulumStateReport & state = input.pendulum_state;

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

  decision.fault = Fault::None;
  decision.message = kObserving;
  return decision;
}

}  // namespace crane_supervisor
