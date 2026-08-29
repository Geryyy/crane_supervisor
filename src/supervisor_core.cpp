#include "crane_supervisor/supervisor_core.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

namespace crane_supervisor
{
namespace
{

constexpr char kAbsenceIsNotHealth[] =
  " Absence is not health: wiki/control_architecture.md 5.3 allows no input to stop arriving "
  "without a defined consequence, so this is reported as a fault rather than left at FAULT_NONE. ";

constexpr char kObserving[] =
  "no fault: the passive joint state is arriving inside its margin, "
  "neither passive rate is past its sway bound, the trajectory controller's own "
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
  "the MPC's constraint margin and this supervisor all read, so that they cannot drift apart. The "
  "same absence reaches further than this report and is worth reading as the whole of it: the seam "
  "clamp in crane_velocity_controller is **transparent** while it holds, so the velocity step at a "
  "handover between the following controller and the MPC is not bounded by anything either. The "
  "mechanism is built and is tested against an injected number; what is missing is the number, and "
  "a plausible one invented anywhere in this stack would be worse than the gap.";

constexpr char kNoVelocityErrorReported[] =
  "a tolerance is configured but the trajectory controller reports no velocity error for the axes "
  "it covers. It fills that field only when it holds a velocity state interface and a velocity or "
  "effort command interface; a profile that gives it neither leaves the field empty, and an empty "
  "field must not be read as a zero error.";

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

constexpr char kActiveState[] = "active";
constexpr char kInactiveState[] = "inactive";

constexpr char kModeNotAValue[] =
  "refused: the requested mode is not one of the four crane_msgs/SupervisorStatus names. They are "
  "MODE_IDLE (0), MODE_MANUAL (1), MODE_FOLLOW (2) and MODE_MPC (3). Nothing was switched, and the "
  "machine is in the mode the response reports.";

constexpr char kHorizonRefusedHead[] =
  "refused: MODE_MPC needs a horizon and the freshness of the one behind it cannot be established. "
  "PRD 10 step 2 requires that to be verified *before* the trajectory controller is deactivated -- "
  "no freshness, no switch -- so this is answered from the mode the machine is already in rather "
  "than discovered half-way through a switch. Nothing was deactivated, and nothing was asked of "
  "the controller manager: this check runs before the view of it is even taken. ";

constexpr char kHorizonNeverArrived[] =
  "No crane_msgs/SolverHealth has arrived on /crane/mpc/solver_health since this supervisor "
  "started, so nothing is known about the optimizer at all -- it has never connected, which on a "
  "deployment that composes no crane_mpc is simply what is true. That stream is what this "
  "supervisor judges the horizon by and not /crane/mpc/horizon itself, because in shadow mode -- "
  "the state every switch into MODE_MPC is made from -- crane_mpc publishes nothing on the "
  "contract topic at all, so a check against the horizon could never pass.";

constexpr char kHorizonStoppedHead[] =
  "The newest crane_msgs/SolverHealth on /crane/mpc/solver_health is ";

constexpr char kHorizonStoppedTail[] =
  " s old, past the freshness deadline of ";

constexpr char kHorizonStoppedEnd[] =
  " s. It was arriving before, so the optimizer has stopped reporting rather than never having "
  "started, and a producer nobody has heard from is exactly the dead MPC this step exists to "
  "refuse a switch into (user story 35).";

constexpr char kHorizonStampAheadHead[] =
  "The newest crane_msgs/SolverHealth on /crane/mpc/solver_health is stamped ";

constexpr char kHorizonStampAheadTail[] =
  " s in this supervisor's future, further ahead than the freshness deadline of ";

constexpr char kHorizonStampAheadEnd[] =
  " s, so its age is not a measurement of anything and no freshness answer about the optimizer "
  "means anything either. crane_mpc runs on a workstation outside the hard real-time cycle and "
  "wiki/control_architecture.md 3.3 makes clock synchronisation between the two a functional "
  "requirement rather than housekeeping; synchronise them.";

constexpr char kHorizonOutcomeHead[] =
  "The optimizer is arriving inside its freshness deadline of ";

constexpr char kHorizonOutcomeMid[] = " s and its newest solve says ";

constexpr char kHorizonOutcomeTail[] =
  ". PRD 10 step 1 wants the MPC *warm* before the switch -- shadow mode already implies it solves "
  "-- and a producer that is alive, publishing at rate and not converging is indistinguishable "
  "from a healthy one on freshness alone, which is why the verdict is checked beside the age. A "
  "budget miss is refused with a failure on purpose: wiki/control_architecture.md 5 row 3 gives "
  "FAULT_SOLVER to a deadline miss and to non-convergence alike, and entering the mode on a plan "
  "with nothing behind it costs a handover, while refusing costs one more request.";

constexpr char kHorizonSolveTiming[] = " The last solve took ";

constexpr char kHorizonSolveBudget[] = " s against a budget of ";

constexpr char kHorizonShifted[] =
  " s, and what went out was the previous horizon shifted rather than this solve "
  "(wiki/mpc.md 6, requirement 3).";

constexpr char kHorizonSolveEnd[] = " s.";

constexpr char kHorizonCarried[] =
  " The producer's own account of the cycle, carried rather than restated: ";

constexpr char kHorizonNoAccount[] =
  " The producer set no message on that report, which is itself worth chasing: "
  "crane_msgs/SolverHealth carries one for exactly this.";

constexpr char kHorizonProducerFault[] =
  " It is also raising FAULT_SOLVER on that stream, which is the one code it raises.";


constexpr char kSolverFaultHead[] =
  "solver: the horizon producer is the live command path and it is reporting a fault on "
  "/crane/mpc/solver_health. The code is merged as crane_mpc numbered it, not translated, exactly "
  "as the inner loop's is (wiki/implementation/ros2_interfaces.md 4). Its newest solve says ";

constexpr char kSolverFaultShifted[] =
  ", and what went out on the horizon was the previous solution shifted by one step rather than "
  "this solve (wiki/mpc.md 6, requirement 3). One such cycle is the defined fallback and not a "
  "reason to leave the mode; what is not a plan any more is a run of them, which crane_mpc counts "
  "and escalates on.";

constexpr char kSolverFaultNothingPublished[] =
  ", and **nothing went out on the horizon at all** -- either wiki/mpc.md 6's repeated-failure "
  "escalation has stopped the publisher, or there was no previous solution to shift. Either way "
  "the receiver is executing the last plan it was given and will run out of it, at which point "
  "crane_velocity_controller ramps the velocity command to zero over its own horizon_expiry_ramp "
  "and reports FAULT_REFERENCE_STALE. That ramp is the fallback of wiki/control_architecture.md 5 "
  "row 3; this supervisor's stop follows it rather than replacing it.";

constexpr char kSolverFaultAccount[] =
  " The producer's own account of the cycle, carried rather than restated: ";

constexpr char kSolverFaultNoAccount[] =
  " The producer set no message on that report, which is itself worth chasing: "
  "crane_msgs/SolverHealth carries one for exactly this.";

constexpr char kHandbackHead[] =
  "the horizon producer has handed control back and the receiver has run out of plan: ";

constexpr char kHandbackMid[] =
  " Nothing is arriving on /crane/mpc/horizon and crane_velocity_controller is reporting "
  "FAULT_REFERENCE_STALE, so MODE_MPC is a mode with no command path behind it. "
  "wiki/control_architecture.md 5 row 3 ends in \"fall back, then stop and report\": the fall back "
  "is the receiver's expiry ramp and it has already engaged, and the stop is this supervisor's -- "
  "5.2 step 1, deactivate the claim. The claim is released to MODE_IDLE and the producer is put "
  "back into shadow.";

constexpr char kHandbackNotFollow[] =
  " It is released to MODE_IDLE and **not** to MODE_FOLLOW, deliberately: choosing to resume a "
  "motion is the task layer's decision and not this one's "
  "(wiki/control_architecture.md 5), and PRD 10 step 4 needs a reference that starts at the "
  "current MPC setpoint, which only the planner can produce. What is owed here is the typed cause, "
  "which is on this stream.";

constexpr char kModeRefusedNoView[] =
  "refused: this supervisor cannot see the controller manager, so it does not know which "
  "controller holds the claim and will not switch blind. A precondition cannot be checked against "
  "a view that is not there, and a switch issued anyway is exactly the half-way discovery PRD 10 "
  "step 2 forbids. Nothing was deactivated. The controller manager's answer: ";

constexpr char kModeRefusedLatched[] =
  "refused: a fault is latched, and a latched fault does not admit a switch into a motion mode. "
  "MODE_IDLE is still reachable -- releasing the claim is the one direction a latched stop does "
  "not argue against -- and clearing the latch goes through /crane/clear_fault, which is the only "
  "thing that lowers it. Nothing was deactivated. The latched cause: ";

constexpr char kModeRefusedNoControllers[] =
  " is configured with no controller on this deployment, so there is nothing to activate for it "
  "and the request is refused rather than answered with a switch that would do nothing. The "
  "controllers of a mode are a property of the profile that composed them, so they are named in "
  "config/crane_supervisor.yaml under mode_controllers; a mode left empty there is a mode this "
  "deployment does not implement. Nothing was deactivated.";

constexpr char kModeRefusedNotLoaded[] =
  " is not loaded on the controller manager, so it cannot be activated. It is checked here, before "
  "anything is deactivated, precisely so that the machine stays in the mode it is in rather than "
  "losing the claim it holds to a switch that was never going to complete (PRD 10 step 2). Check "
  "that the profile spawns it and that it is named identically here and there. Nothing was "
  "deactivated. The controllers the manager does have: ";

constexpr char kModeRefusedNotSwitchableHead[] =
  " is loaded on the controller manager in state ";

constexpr char kModeRefusedNotSwitchableTail[] =
  ", which is not a state a switch can move: only an inactive controller can be activated and only "
  "an active one can be deactivated. A controller that will not configure is a failure to chase on "
  "the manager rather than in this request, and it is reported before anything is deactivated so "
  "that the machine keeps the claim it has. Nothing was deactivated.";

constexpr char kModeAlreadyActiveHead[] = "no switch was issued: ";

constexpr char kModeAlreadyActiveTail[] =
  " is already the mode the controller manager reports as active, so the request is a no-op and is "
  "reported as one rather than as a switch. active_mode on this response is what is actually "
  "active, which is what it would have been either way.";

constexpr char kIdleIsNotAStop[] =
  " MODE_IDLE releases the arm claim; it is not a stop and must not be read as one. This "
  "supervisor holds no stop authority -- the hardware stop input is an unverified commissioning "
  "prerequisite (wiki/control_architecture.md 5.2) -- so nothing is zeroed at the driver boundary "
  "here, and 7.2 records that a manual controller which deactivates leaves its last velocity "
  "latched on the interface it just released. Releasing a claim is a mode change somebody asked "
  "for, not a way to bring the machine to rest.";

constexpr char kSwitchIssuedHead[] = "switched to ";

constexpr char kSwitchIssuedTail[] =
  ". The switch was one call on /controller_manager/switch_controller, strict, with the activation "
  "and the deactivation in the same request, so the machine passed from one mode to the other in "
  "one of the manager's cycles rather than through a state that is neither. Every precondition was "
  "checked before it was issued.";

constexpr char kSwitchFailedHead[] = "the switch to ";

constexpr char kSwitchFailedTail[] =
  " was refused by the controller manager. Every precondition this supervisor can check was "
  "checked before the call, so what failed is on the manager's side of the boundary -- a resource "
  "claim, an activation that returned an error, or a timeout waiting for the control cycle. "
  "active_mode on this response is what the manager reports as active *after* the attempt, read "
  "back rather than assumed. The manager's own account: ";

constexpr char kSwitchNoAccount[] =
  "(the controller manager returned no message, which is itself worth reporting: the service "
  "carries one for exactly this case)";

constexpr char kProducerActive[] = "active";
constexpr char kProducerShadow[] = "shadow";

constexpr char kProducerMovedHead[] = " The horizon producer was put into ";

constexpr char kProducerMovedTail[] =
  " mode, which is the other half of this switch and this supervisor's alone: ROS 2 Interfaces 4 "
  "makes which path is live the supervisor's decision and says the two never drive at once. In "
  "shadow crane_mpc solves at rate and publishes nothing on /crane/mpc/horizon; in active the same "
  "solve reaches the velocity controller. Entering MODE_MPC it is moved before the claim is, so a "
  "horizon is already fitted when the trajectory controller lets go; leaving, after -- an "
  "unchained inner loop with no horizon ramps to zero and raises FAULT_REFERENCE_STALE, which "
  "would be a hole in the handover rather than a seam.";

constexpr char kProducerRefusedHead[] =
  " The horizon producer would not be put into ";

constexpr char kProducerRefusedTail[] =
  " mode, so the claim and the producer disagree about which path is live -- which is the one "
  "state ROS 2 Interfaces 4 forbids, and it is reported rather than left for a silent horizon to "
  "be the evidence of. The producer's own account: ";

constexpr char kProducerNoAccount[] =
  "(none was given, which is itself worth chasing: rcl_interfaces/SetParameters carries a reason "
  "per parameter for exactly this)";

constexpr char kSwitchLandedElsewhereHead[] =
  ". The controller manager accepted the switch and the mode read back afterwards is ";

constexpr char kSwitchLandedElsewhereTail[] =
  " instead, so what this response reports is what is actually active rather than what was asked "
  "for. That gap is the reason the mode is re-derived at all.";

constexpr char kModeClauseUnknownHead[] =
  "not known -- this supervisor cannot see the controller manager. ";

constexpr char kModeClauseUnknownTail[] =
  " MODE_IDLE is reported on this status because no motion mode can be confirmed to hold the "
  "claim, which is the reading that claims the least; it is not an observation that nothing is "
  "running. No mode change is admitted while this holds.";

constexpr char kModeClauseDrift[] =
  ", and the active controllers do not form exactly one configured mode: ";

constexpr char kModeClauseDriftTail[] =
  " is active. Something came up or went down outside a mode change, so no configured motion mode "
  "holds the arm claim on its own and the mode reported is the one that claims the least.";

constexpr char kModeClauseIdle[] =
  ": no controller of any configured mode is active, so the arm claim is free.";

constexpr char kModeClauseActive[] = ", held by ";

constexpr char kToolClauseAbsent[] =
  " No tool claim is configured: crane_velocity_controller holds all six velocity interfaces "
  "including the gripper axis on today's profiles, so this deployment has one claim. "
  "wiki/control_architecture.md 7.3 records that the machine has two -- with the block gripper the "
  "tool axis runs on its own controller and can move while the arm controller is inactive -- so "
  "the model carries the second claim and no mode switch ever touches it.";

constexpr char kToolClauseHeld[] = " Tool claim (7.3), which no mode switch touches, held by ";

constexpr char kToolClauseFree[] =
  " Tool claim (7.3), which no mode switch touches: free. It is a second claim on the same pump, "
  "so a gripper action and an arm motion are two claims and not one machine.";

constexpr char kUnmodelledClaimants[] =
  " Active and owning command interfaces without belonging to any configured mode or to the tool "
  "claim: ";

constexpr char kUnmodelledClaimantsTail[] =
  ". That is a claim this configuration does not describe, and it is reported rather than acted on "
  "-- a mode switch will not deactivate it, and it can refuse one by holding an interface the "
  "incoming controller needs.";

/// Seconds as text, to the millisecond.
std::string seconds_text(double seconds)
{
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.3f", seconds);
  return std::string(buffer);
}

std::string quantity_text(double value)
{
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.4f", value);
  return std::string(buffer);
}

/// The unit of one axis, from its URDF name.
const char * angular_or_linear(const std::string & joint, bool per_second)
{
  const bool angular = !joint.empty() && joint.front() == 't';
  if (per_second) {
    return angular ? "rad/s" : "m/s";
  }
  return angular ? "rad" : "m";
}

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
std::string stop_message(
  const SupervisorConfig & config, const SupervisorInput & input, Staleness cause)
{
  if (cause != Staleness::Fresh) {
    return staleness_message(
      config, Input::RemoteCtrl, cause, input.stream(Input::RemoteCtrl)) +
           kDiagnosisNotProtection;
  }
  if (input.remote_ctrl.em_stop) {
    return std::string(kStopAsserted) + kDiagnosisNotProtection;
  }
  return std::string(kStopLatched) + kDiagnosisNotProtection;
}

/// Whether the emergency stop is latched as of this observation.
bool stop_is_latched(const SupervisorConfig & config, const SupervisorInput & input)
{
  const Staleness cause =
    freshness_of(config.deadline(Input::RemoteCtrl), input.stream(Input::RemoteCtrl));
  return input.estop_latched || cause != Staleness::Fresh || input.remote_ctrl.em_stop;
}

/// Why no axis was compared this cycle, or empty when at least one was.
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

bool is_mode(std::uint8_t value) noexcept
{
  return value < static_cast<std::uint8_t>(kModeCount);
}

const char * solve_outcome_name(SolveOutcome outcome) noexcept
{
  switch (outcome) {
    case SolveOutcome::Unknown:
      return "SOLVE_UNKNOWN -- nothing filled the verdict on that report";
    case SolveOutcome::Converged:
      return "SOLVE_CONVERGED";
    case SolveOutcome::BudgetExceeded:
      return "SOLVE_BUDGET_EXCEEDED -- the solve hit its deadline without converging";
    case SolveOutcome::Failed:
      return "SOLVE_FAILED";
  }
  return "a verdict crane_msgs/SolverHealth does not define";
}

const char * mode_name(Mode mode) noexcept
{
  // The wire's own constant names, so a report and a panel say the same word.
  switch (mode) {
    case Mode::Idle:
      return "MODE_IDLE";
    case Mode::Manual:
      return "MODE_MANUAL";
    case Mode::Follow:
      return "MODE_FOLLOW";
    case Mode::Mpc:
      return "MODE_MPC";
  }
  return "MODE_IDLE";
}

bool validate(const SupervisorConfig & config, std::string & reason)
{
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
  if (
    !std::isfinite(config.controller_manager_deadline) ||
    config.controller_manager_deadline <= 0.0)
  {
    reason =
      "the controller manager's answer has a freshness deadline of " +
      seconds_text(config.controller_manager_deadline) +
      " s, and a deadline is a finite positive number of seconds. Without one the mode reported on "
      "the status stream would either always be unknown or never be, and the mode is re-derived "
      "from that answer rather than remembered. The number and its derivation are in "
      "src/crane_supervisor_parameters.yaml.";
    return false;
  }
  if (!std::isfinite(config.horizon_deadline) || config.horizon_deadline <= 0.0) {
    reason =
      "the horizon producer's report has a freshness deadline of " +
      seconds_text(config.horizon_deadline) +
      " s, and a deadline is a finite positive number of seconds. Without one PRD 10 step 2's "
      "check has nothing to compare an age against, so MODE_MPC would be admitted or refused by a "
      "number nobody chose. The number and its derivation are in "
      "src/crane_supervisor_parameters.yaml.";
    return false;
  }
  if (!config.mode_controllers[index_of(Mode::Idle)].empty()) {
    reason =
      "MODE_IDLE is configured with controllers, and it is the absence of a motion claim rather "
      "than a controller: a mode whose controllers had to be activated to be idle would make "
      "releasing the claim into a claim of its own. Leave mode_controllers.idle unset.";
    return false;
  }
  std::vector<std::string> in_a_mode;
  for (std::size_t i = 0; i < kModeCount; ++i) {
    const std::vector<std::string> & controllers = config.mode_controllers[i];
    for (std::size_t j = 0; j < controllers.size(); ++j) {
      const std::string & name = controllers[j];
      if (name.empty()) {
        reason = std::string("a controller of ") + mode_name(static_cast<Mode>(i)) +
          " is configured with an empty name, and an unnamed controller can never be paired with "
          "anything the controller manager loaded";
        return false;
      }
      if (std::find(controllers.begin(), controllers.begin() + static_cast<std::ptrdiff_t>(j),
        name) != controllers.begin() + static_cast<std::ptrdiff_t>(j))
      {
        reason = name + " is configured twice for " + mode_name(static_cast<Mode>(i)) +
          ". A mode is read as active by comparing the controllers that are up against the set "
          "this list names, and a name that appears twice makes the list's length stop counting "
          "the controllers in it, so the comparison would never match.";
        return false;
      }
      if (std::find(in_a_mode.begin(), in_a_mode.end(), name) == in_a_mode.end()) {
        in_a_mode.push_back(name);
      }
    }
    for (std::size_t k = 0; k < i; ++k) {
      if (controllers.empty() || config.mode_controllers[k].size() != controllers.size()) {
        continue;
      }
      bool same = true;
      for (const std::string & name : controllers) {
        const std::vector<std::string> & other = config.mode_controllers[k];
        if (std::find(other.begin(), other.end(), name) == other.end()) {
          same = false;
          break;
        }
      }
      if (same) {
        reason = std::string(mode_name(static_cast<Mode>(i))) + " and " +
          mode_name(static_cast<Mode>(k)) +
          " are configured with the same set of controllers, so both would read as active at once "
          "and which one this supervisor reported would depend on the order it happened to check "
          "them in. Two modes may share a controller -- the same inner loop carries both paths of "
          "PRD 10 step 3 -- but they may not be the same claim under two names.";
        return false;
      }
    }
  }
  for (const std::string & name : config.tool_controllers) {
    if (name.empty()) {
      reason =
        "a controller of the tool claim is configured with an empty name, and an unnamed "
        "controller can never be paired with anything the controller manager loaded";
      return false;
    }
    if (std::find(in_a_mode.begin(), in_a_mode.end(), name) != in_a_mode.end()) {
      reason = name +
        " is configured both as a mode's controller and as the tool claim's. The tool axis is a "
        "second, independent claim on the same pump (wiki/control_architecture.md 7.3) and no mode "
        "switch may touch it, so a controller that were both would be deactivated by an arm mode "
        "change that is supposed to leave the gripper alone.";
      return false;
    }
  }
  if (!config.mode_controllers[index_of(Mode::Mpc)].empty() && config.mpc_node.empty()) {
    reason =
      "MODE_MPC is configured with controllers and no horizon producer is named. Which path is "
      "live is this supervisor's decision alone (wiki/implementation/ros2_interfaces.md 4, 'One "
      "command path') and the switch has to move crane_mpc between shadow and active as well as "
      "moving the claim, so a deployment that implements the mode names the node it composed the "
      "producer under in mpc_node. Without it the claim would move to a path whose producer "
      "publishes nothing.";
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
  const SupervisorConfig & config, Input input, Staleness cause, const StreamReport & stream)
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
SupervisorDecision resolve(
  const SupervisorConfig & config, const SupervisorInput & input,
  const std::array<Staleness, kInputCount> & staleness, const SwayVerdict & sway,
  const ActiveMode & active)
{
  SupervisorDecision decision;

  const Staleness remote_cause = staleness[index_of(Input::RemoteCtrl)];

  const ControllerStateReport & controller = input.controller_state;
  const ControllerHealthReport & inner_loop = input.controller_health;

  if (staleness[index_of(Input::ControllerState)] == Staleness::Fresh) {
    decision.tracking_error = max_position_error(controller);
  }

  decision.deadman_held = remote_cause == Staleness::Fresh && input.remote_ctrl.deadman_held;

  decision.estop_latched = stop_is_latched(config, input);

  if (decision.estop_latched) {
    decision.fault = policy_of(Input::RemoteCtrl).fault;
    decision.message = stop_message(config, input, remote_cause);
    return decision;
  }

  if (staleness[index_of(Input::PendulumState)] != Staleness::Fresh) {
    decision.fault = policy_of(Input::PendulumState).fault;
    decision.message = staleness_message(
      config, Input::PendulumState, staleness[index_of(Input::PendulumState)],
      input.stream(Input::PendulumState));
    return decision;
  }

  for (const Input stream_input : {Input::ControllerState, Input::ControllerHealth}) {
    const Staleness cause = staleness[index_of(stream_input)];
    if (cause != Staleness::Fresh) {
      decision.fault = policy_of(stream_input).fault;
      decision.message =
        staleness_message(config, stream_input, cause, input.stream(stream_input));
      return decision;
    }
  }

  const HorizonReport & horizon = input.horizon;
  const bool horizon_fresh =
    freshness_of(config.horizon_deadline, horizon.health) == Staleness::Fresh;
  if (active.known && active.mode == Mode::Mpc && horizon_fresh && horizon.fault != Fault::None) {
    decision.fault = horizon.fault;
    decision.message = std::string(kSolverFaultHead) + solve_outcome_name(horizon.outcome) +
      (horizon.applied_previous_solution ? kSolverFaultShifted : kSolverFaultNothingPublished);
    decision.message += horizon.status.empty()
      ? std::string(kSolverFaultNoAccount)
      : kSolverFaultAccount + horizon.status;
    return decision;
  }

  if (inner_loop.fault != Fault::None && inner_loop.fault != Fault::NotCommissioned) {
    decision.fault = inner_loop.fault;
    decision.message = inner_loop_message(inner_loop.fault);
    return decision;
  }

  if (!sway.breaches.empty()) {
    decision.fault = Fault::Sway;
    decision.message = sway_breach_message(sway.breaches);
    return decision;
  }

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

  if (inner_loop.fault == Fault::NotCommissioned) {
    decision.fault = Fault::NotCommissioned;
    decision.message = inner_loop.feedforward_free_joints.empty()
      ? kNotCommissionedNoAxis
      : kNotCommissionedHead + joined(inner_loop.feedforward_free_joints) + kNotCommissionedTail;
    return decision;
  }

  if (!decision.deadman_held) {
    decision.fault = Fault::Interlock;
    decision.message =
      kDeadmanReleased + std::to_string(config.deadman_button) + kDeadmanReleasedTail;
    return decision;
  }

  decision.fault = Fault::None;
  decision.message = kObserving;
  const std::string uncompared = nothing_compared_reason(config, controller);
  if (!uncompared.empty()) {
    decision.message += kNoAxisCompared + uncompared;
  }
  return decision;
}

}  // namespace

SupervisorDecision decide(const SupervisorConfig & config, const SupervisorInput & input)
{
  const std::array<Staleness, kInputCount> staleness = freshness(config, input);

  const bool estimate_trusted = staleness[index_of(Input::PendulumState)] == Staleness::Fresh;

  const SwayVerdict sway = judge_sway(
    config.sway, input.pendulum_state.velocity, estimate_trusted, input.sampled_at, input.sway);

  const ActiveMode active = active_mode(config, input.controller_manager);

  SupervisorDecision decision = resolve(config, input, staleness, sway, active);
  decision.mode = active.mode;
  decision.sway = sway.state;
  decision.message += mode_clause(config, active);
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

namespace
{

/// The controller manager's row for one controller, or null when it has none.
const ControllerReport * controller_named(
  const ControllerManagerReport & report, const std::string & name)
{
  for (const ControllerReport & controller : report.controllers) {
    if (controller.name == name) {
      return &controller;
    }
  }
  return nullptr;
}

/// True when the manager reports this controller active right now.
bool is_controller_active(const ControllerManagerReport & report, const std::string & name)
{
  const ControllerReport * const controller = controller_named(report, name);
  return controller != nullptr && controller->state == kActiveState;
}

bool is_modelled(const SupervisorConfig & config, const std::string & name)
{
  for (std::size_t i = 0; i < kModeCount; ++i) {
    const auto & controllers = config.mode_controllers[i];
    if (std::find(controllers.begin(), controllers.end(), name) != controllers.end()) {
      return true;
    }
  }
  return std::find(config.tool_controllers.begin(), config.tool_controllers.end(), name) !=
         config.tool_controllers.end();
}

/// The manager's answer, as the reason a refusal can print.
std::string controller_manager_account(const SupervisorConfig & config, Staleness cause)
{
  const std::string deadline = seconds_text(config.controller_manager_deadline);
  switch (cause) {
    case Staleness::NeverArrived:
      return "it has not answered since this supervisor started. Check that a controller manager "
             "is running and that its list_controllers service is reachable from this node.";
    case Staleness::StoppedArriving:
      return "its newest answer is older than the " + deadline +
             " s this supervisor allows it, so the mode it named cannot be assumed to still hold. "
             "It was answering before, which is not the same fault as a manager that never came "
             "up.";
    case Staleness::StampAhead:
      return "its newest answer is dated further in this node's future than the " + deadline +
             " s margin allows, so its age is not a measurement of anything.";
    default:
      break;
  }
  return "it answered inside its " + deadline +
         " s margin and an account of its absence was composed anyway, which is a defect in this "
         "supervisor rather than in the manager.";
}

}  // namespace

ActiveMode active_mode(const SupervisorConfig & config, const ControllerManagerReport & report)
{
  ActiveMode active;
  active.cause = freshness_of(config.controller_manager_deadline, report.answer);
  if (active.cause != Staleness::Fresh) {
    return active;
  }
  active.known = true;

  for (std::size_t i = 0; i < kModeCount; ++i) {
    for (const std::string & name : config.mode_controllers[i]) {
      if (
        is_controller_active(report, name) &&
        std::find(active.active_controllers.begin(), active.active_controllers.end(), name) ==
        active.active_controllers.end())
      {
        active.active_controllers.push_back(name);
      }
    }
  }

  std::size_t matched = 0;
  for (std::size_t i = 0; i < kModeCount; ++i) {
    const std::vector<std::string> & controllers = config.mode_controllers[i];
    if (controllers.empty() || controllers.size() != active.active_controllers.size()) {
      continue;
    }
    bool all_up = true;
    for (const std::string & name : controllers) {
      if (!is_controller_active(report, name)) {
        all_up = false;
        break;
      }
    }
    if (all_up) {
      ++matched;
      active.mode = static_cast<Mode>(i);
    }
  }
  if (matched != 1) {
    active.mode = Mode::Idle;
    active.partial = !active.active_controllers.empty();
  }

  for (const std::string & name : config.tool_controllers) {
    if (is_controller_active(report, name)) {
      active.active_tool_controllers.push_back(name);
    }
  }

  for (const ControllerReport & controller : report.controllers) {
    if (
      controller.state == kActiveState && !controller.claimed_interfaces.empty() &&
      !is_modelled(config, controller.name))
    {
      active.unmodelled_claimants.push_back(controller.name);
    }
  }
  return active;
}

std::string mode_clause(const SupervisorConfig & config, const ActiveMode & active)
{
  std::string text = kModeClausePrefix;
  if (!active.known) {
    return text + kModeClauseUnknownHead + controller_manager_account(config, active.cause) +
           kModeClauseUnknownTail;
  }

  text += mode_name(active.mode);
  if (active.partial) {
    text += std::string(kModeClauseDrift) + joined(active.active_controllers) +
      kModeClauseDriftTail;
  } else if (active.active_controllers.empty()) {
    text += kModeClauseIdle;
  } else {
    text += std::string(kModeClauseActive) + joined(active.active_controllers) + ".";
  }

  if (config.tool_controllers.empty()) {
    text += kToolClauseAbsent;
  } else if (active.active_tool_controllers.empty()) {
    text += kToolClauseFree;
  } else {
    text += std::string(kToolClauseHeld) + joined(active.active_tool_controllers) + ".";
  }

  if (!active.unmodelled_claimants.empty()) {
    text += std::string(kUnmodelledClaimants) + joined(active.unmodelled_claimants) +
      kUnmodelledClaimantsTail;
  }
  return text;
}

HorizonPrecondition horizon_precondition(
  const SupervisorConfig & config, const HorizonReport & report) noexcept
{
  HorizonPrecondition precondition;
  precondition.cause = freshness_of(config.horizon_deadline, report.health);
  precondition.fresh =
    precondition.cause == Staleness::Fresh && report.outcome == SolveOutcome::Converged;
  return precondition;
}

std::string mpc_horizon_refusal(
  const SupervisorConfig & config, std::uint8_t requested, const SupervisorInput & input)
{
  if (!is_mode(requested) || static_cast<Mode>(requested) != Mode::Mpc) {
    return {};
  }

  const HorizonReport & horizon = input.horizon;
  const HorizonPrecondition precondition = horizon_precondition(config, horizon);
  if (precondition.fresh) {
    return {};
  }

  const std::string deadline = seconds_text(config.horizon_deadline);
  std::string text = kHorizonRefusedHead;
  switch (precondition.cause) {
    case Staleness::NeverArrived:
      text += kHorizonNeverArrived;
      break;
    case Staleness::StoppedArriving:
      text += kHorizonStoppedHead + seconds_text(horizon.health.age) + kHorizonStoppedTail +
        deadline + kHorizonStoppedEnd;
      break;
    case Staleness::StampAhead:
      text += kHorizonStampAheadHead + seconds_text(-horizon.health.age) + kHorizonStampAheadTail +
        deadline + kHorizonStampAheadEnd;
      break;
    case Staleness::Fresh:
      text += kHorizonOutcomeHead + deadline + kHorizonOutcomeMid +
        solve_outcome_name(horizon.outcome) + kHorizonOutcomeTail;
      break;
  }

  if (horizon.health.received && horizon.solve_budget > 0.0) {
    text += kHorizonSolveTiming + seconds_text(horizon.solve_time) + kHorizonSolveBudget +
      seconds_text(horizon.solve_budget) +
      (horizon.applied_previous_solution ? kHorizonShifted : kHorizonSolveEnd);
  }
  if (horizon.fault == Fault::Solver) {
    text += kHorizonProducerFault;
  }
  if (horizon.health.received) {
    text += horizon.status.empty()
      ? std::string(kHorizonNoAccount)
      : kHorizonCarried + horizon.status;
  }
  return text;
}

SolverHandback solver_handback(
  const SupervisorConfig & config, const SupervisorInput & input, const ActiveMode & active)
{
  SolverHandback handback;

  if (!active.known || active.partial || active.mode != Mode::Mpc) {
    return handback;
  }

  const HorizonReport & horizon = input.horizon;
  if (freshness_of(config.horizon_deadline, horizon.health) != Staleness::Fresh) {
    return handback;
  }

  if (horizon.fault != Fault::Solver || horizon.applied_previous_solution) {
    return handback;
  }

  if (
    freshness_of(config.deadline(Input::ControllerHealth), input.stream(Input::ControllerHealth)) !=
    Staleness::Fresh || input.controller_health.fault != Fault::ReferenceStale)
  {
    return handback;
  }

  handback.required = true;
  handback.message = std::string(kHandbackHead) +
    (horizon.status.empty() ? std::string("the producer set no message on that report") :
    horizon.status) + "." + kHandbackMid + kHandbackNotFollow;
  return handback;
}

ModeArbitration arbitrate_mode(
  const SupervisorConfig & config, std::uint8_t requested, const SupervisorInput & input)
{
  ModeArbitration arbitration;
  arbitration.active = active_mode(config, input.controller_manager);

  // 1. A uint8 off the wire is not a mode until it has been checked.
  if (!is_mode(requested)) {
    arbitration.message = kModeNotAValue;
    return arbitration;
  }
  const Mode mode = static_cast<Mode>(requested);

  {
    const std::string refusal = mpc_horizon_refusal(config, requested, input);
    if (!refusal.empty()) {
      arbitration.message = refusal;
      return arbitration;
    }
  }

  // 3. Nothing below can be checked without a view of the machine.
  if (!arbitration.active.known) {
    arbitration.message =
      kModeRefusedNoView + controller_manager_account(config, arbitration.active.cause);
    return arbitration;
  }

  if (mode == arbitration.active.mode && !arbitration.active.partial) {
    arbitration.accepted = true;
    arbitration.message =
      kModeAlreadyActiveHead + std::string(mode_name(mode)) + kModeAlreadyActiveTail;
    return arbitration;
  }

  if (mode != Mode::Idle && stop_is_latched(config, input)) {
    const Staleness remote_cause =
      freshness_of(config.deadline(Input::RemoteCtrl), input.stream(Input::RemoteCtrl));
    arbitration.message = kModeRefusedLatched + stop_message(config, input, remote_cause);
    return arbitration;
  }

  const std::vector<std::string> & wanted = config.mode_controllers[index_of(mode)];
  if (mode != Mode::Idle && wanted.empty()) {
    arbitration.message = std::string(mode_name(mode)) + kModeRefusedNoControllers;
    return arbitration;
  }

  for (const std::string & name : wanted) {
    const ControllerReport * const controller =
      controller_named(input.controller_manager, name);
    if (controller == nullptr) {
      std::vector<std::string> loaded;
      loaded.reserve(input.controller_manager.controllers.size());
      for (const ControllerReport & known : input.controller_manager.controllers) {
        loaded.push_back(known.name);
      }
      arbitration.message = name + kModeRefusedNotLoaded +
        (loaded.empty() ? std::string("none at all") : joined(loaded)) + ".";
      return arbitration;
    }
    if (controller->state != kActiveState && controller->state != kInactiveState) {
      arbitration.message = name + kModeRefusedNotSwitchableHead + controller->state +
        kModeRefusedNotSwitchableTail;
      return arbitration;
    }
  }

  arbitration.accepted = true;
  arbitration.horizon_producer_active = mode == Mode::Mpc;
  for (const std::string & name : wanted) {
    if (!is_controller_active(input.controller_manager, name)) {
      arbitration.activate.push_back(name);
    }
  }
  for (std::size_t i = 0; i < kModeCount; ++i) {
    if (i == index_of(mode)) {
      continue;
    }
    for (const std::string & name : config.mode_controllers[i]) {
      if (
        is_controller_active(input.controller_manager, name) &&
        std::find(wanted.begin(), wanted.end(), name) == wanted.end() &&
        std::find(arbitration.deactivate.begin(), arbitration.deactivate.end(), name) ==
        arbitration.deactivate.end())
      {
        arbitration.deactivate.push_back(name);
      }
    }
  }
  arbitration.switch_required = !arbitration.activate.empty() || !arbitration.deactivate.empty();

  if (!arbitration.switch_required) {
    arbitration.message =
      kModeAlreadyActiveHead + std::string(mode_name(mode)) + kModeAlreadyActiveTail;
    return arbitration;
  }

  arbitration.message = kSwitchIssuedHead + std::string(mode_name(mode)) + kSwitchIssuedTail;
  if (mode == Mode::Idle) {
    arbitration.message += kIdleIsNotAStop;
  }
  return arbitration;
}

std::string mode_switch_message(
  const SupervisorConfig & config, Mode requested, const ModeArbitration & arbitration,
  bool switched, const std::string & carried, const ActiveMode & reached,
  const ProducerSwitch & producer)
{
  std::string text;
  if (!switched) {
    text = kSwitchFailedHead + std::string(mode_name(requested)) + kSwitchFailedTail +
      (carried.empty() ? std::string(kSwitchNoAccount) : carried);
  } else {
    text = arbitration.message;
    if (reached.mode != requested || !reached.known || reached.partial) {
      text += kSwitchLandedElsewhereHead + std::string(mode_name(reached.mode)) +
        kSwitchLandedElsewhereTail;
    }
  }
  if (producer.attempted) {
    const char * const wanted = producer.active ? kProducerActive : kProducerShadow;
    if (producer.accepted) {
      text += kProducerMovedHead + std::string(wanted) + kProducerMovedTail;
    } else {
      text += kProducerRefusedHead + std::string(wanted) + kProducerRefusedTail +
        (producer.account.empty() ? std::string(kProducerNoAccount) : producer.account);
    }
  }
  return text + mode_clause(config, reached);
}

}  // namespace crane_supervisor
