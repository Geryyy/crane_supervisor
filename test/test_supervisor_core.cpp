// The decision core, offline. No ROS, no DDS, no clock is linked into this
// binary and none is reachable from it, so every cause can be produced exactly
// and one at a time -- which is the point of the core being ROS-free at all.

#include <gtest/gtest.h>

#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include "crane_supervisor/supervisor_core.hpp"

namespace
{

/// The six actuated joints of ROS 2 Interfaces §3.2, in configuration order.
/// Four `theta*` angles and two `q*` lengths -- the split that makes the max in
/// `tracking_error` a number in two units.
const std::vector<std::string> & actuated_joints()
{
  static const std::vector<std::string> joints{
    "theta1_slewing_joint", "theta2_boom_joint", "theta3_arm_joint",
    "q4_big_telescope", "theta8_rotator_joint", "q9_left_rail_joint"};
  return joints;
}

/// The configuration a deployment actually gets today: no tolerance loaded.
crane_supervisor::SupervisorConfig default_config()
{
  crane_supervisor::SupervisorConfig config;
  for (const std::string & joint : actuated_joints()) {
    // One member given, so `dq_a` takes its own default: NaN, the state a
    // deployment that was never handed the tolerance file is honestly in.
    config.tracking_tolerance.push_back(crane_supervisor::AxisTolerance{joint});
  }
  return config;
}

/// The same configuration with the six numbers a human has not measured yet.
/**
 * Distinct per axis on purpose: a comparison that paired the two lists by index
 * rather than by name would still pass with six equal numbers.
 */
crane_supervisor::SupervisorConfig config_with_tolerances()
{
  crane_supervisor::SupervisorConfig config;
  double dq_a = 0.01;
  for (const std::string & joint : actuated_joints()) {
    config.tracking_tolerance.push_back(crane_supervisor::AxisTolerance{joint, dq_a});
    dq_a += 0.01;
  }
  return config;
}

/// A trajectory controller tracking exactly, reporting all six axes.
crane_supervisor::ControllerStateReport tracking_controller()
{
  crane_supervisor::ControllerStateReport report;
  report.received = true;
  report.age = 0.01;
  for (const std::string & joint : actuated_joints()) {
    crane_supervisor::AxisError axis;
    axis.joint = joint;
    axis.velocity_error_reported = true;
    report.axes.push_back(axis);
  }
  return report;
}

/// The report with one axis set, by name.
crane_supervisor::ControllerStateReport with_error(
  const std::string & joint, double position_error, double velocity_error)
{
  crane_supervisor::ControllerStateReport report = tracking_controller();
  for (crane_supervisor::AxisError & axis : report.axes) {
    if (axis.joint == joint) {
      axis.position_error = position_error;
      axis.velocity_error = velocity_error;
    }
  }
  return report;
}

/// A remote that is arriving, with the stop released and the deadman held.
crane_supervisor::RemoteCtrlReport held_remote()
{
  crane_supervisor::RemoteCtrlReport remote;
  remote.received = true;
  remote.deadman_held = true;
  remote.em_stop = false;
  remote.age = 0.01;
  return remote;
}

/// Both inputs arriving, in time, trusted, with the operator holding the button.
crane_supervisor::SupervisorInput healthy_input()
{
  crane_supervisor::SupervisorInput input;
  input.pendulum_state.received = true;
  input.pendulum_state.valid = true;
  input.pendulum_state.age = 0.01;
  input.pendulum_state.status = "complementary filter on the two bracketing IMUs";
  input.remote_ctrl = held_remote();
  input.controller_state = tracking_controller();
  return input;
}

/// One decision, with the latch carried into the next input the way the node
/// carries it. The latch is the only thing a cycle hands to its successor.
crane_supervisor::SupervisorDecision step(
  const crane_supervisor::SupervisorConfig & config, crane_supervisor::SupervisorInput & input)
{
  const auto decision = crane_supervisor::decide(config, input);
  input.estop_latched = decision.estop_latched;
  return decision;
}

}  // namespace

TEST(SupervisorCore, RejectsAMarginThatCannotSeparateArrivingFromStopped)
{
  std::string reason;
  crane_supervisor::SupervisorConfig config;

  config.pendulum_state_timeout = 0.0;
  EXPECT_FALSE(crane_supervisor::validate(config, reason));
  EXPECT_FALSE(reason.empty());

  config.pendulum_state_timeout = -1.0;
  EXPECT_FALSE(crane_supervisor::validate(config, reason));

  config.pendulum_state_timeout = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(crane_supervisor::validate(config, reason));

  config = crane_supervisor::SupervisorConfig{};
  config.remote_ctrl_timeout = 0.0;
  EXPECT_FALSE(crane_supervisor::validate(config, reason));
  EXPECT_FALSE(reason.empty());

  config.remote_ctrl_timeout = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(crane_supervisor::validate(config, reason));

  config = crane_supervisor::SupervisorConfig{};
  config.controller_state_timeout = 0.0;
  EXPECT_FALSE(crane_supervisor::validate(config, reason));
  EXPECT_FALSE(reason.empty());

  config.controller_state_timeout = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(crane_supervisor::validate(config, reason));

  EXPECT_TRUE(crane_supervisor::validate(default_config(), reason));
}

TEST(SupervisorCore, AMissingToleranceIsReportedRatherThanRefused)
{
  // The number does not exist yet and is human-owned.  A supervisor that
  // refused to start over it would withhold the emergency stop, the state
  // health and the interlock to report one duty it cannot perform, so the
  // absence is a warning at configuration and not a validation failure.
  std::string reason;
  EXPECT_TRUE(crane_supervisor::validate(default_config(), reason));
  EXPECT_TRUE(crane_supervisor::validate(config_with_tolerances(), reason));

  // ... and it names every axis that has none, plus who owns the number.
  const std::string notice = crane_supervisor::tracking_tolerance_notice(default_config());
  EXPECT_FALSE(notice.empty());
  for (const std::string & joint : actuated_joints()) {
    EXPECT_NE(notice.find(joint), std::string::npos) << notice;
  }
  EXPECT_NE(notice.find("(ii-b)"), std::string::npos) << notice;
  EXPECT_NE(notice.find("tracking_tolerance.yaml"), std::string::npos) << notice;

  // Nothing to say once every axis has one.
  EXPECT_TRUE(crane_supervisor::tracking_tolerance_notice(config_with_tolerances()).empty());

  // Only one axis missing, and only that axis is named.
  crane_supervisor::SupervisorConfig partial = config_with_tolerances();
  partial.tracking_tolerance[1].dq_a = -1.0;
  const std::string one = crane_supervisor::tracking_tolerance_notice(partial);
  EXPECT_NE(one.find("theta2_boom_joint"), std::string::npos) << one;
  EXPECT_EQ(one.find("theta1_slewing_joint"), std::string::npos) << one;
}

TEST(SupervisorCore, ANumberIsAToleranceOnlyIfItIsFiniteAndPositive)
{
  // The rule `tracking_tolerance.yaml` states in its own header, and the reason
  // its six rows are written out as -1.0 rather than omitted: the absence has to
  // be visible in the file that will one day carry the value.
  EXPECT_TRUE(crane_supervisor::is_tolerance(0.02));
  EXPECT_FALSE(crane_supervisor::is_tolerance(-1.0));
  EXPECT_FALSE(crane_supervisor::is_tolerance(0.0));
  EXPECT_FALSE(crane_supervisor::is_tolerance(std::numeric_limits<double>::quiet_NaN()));
  EXPECT_FALSE(crane_supervisor::is_tolerance(std::numeric_limits<double>::infinity()));
}

TEST(SupervisorCore, ATrackingToleranceWithNoAxisNameIsRefused)
{
  // A tolerance is paired with an axis by name.  One with no name can never be
  // paired with anything, so it is a tolerance that silently applies to nothing
  // -- which is the one failure mode a warning would not catch, because there
  // would be nothing to name in it.
  std::string reason;
  crane_supervisor::SupervisorConfig config = config_with_tolerances();
  config.tracking_tolerance[3].joint.clear();
  EXPECT_FALSE(crane_supervisor::validate(config, reason));
  EXPECT_FALSE(reason.empty());
}

TEST(SupervisorCore, RejectsADeadmanButtonTheMessageDoesNotHave)
{
  // The message names twelve booleans and no thirteenth.  A number outside them
  // would select no field at all, and a deadman read off no field is one that is
  // released forever -- which would raise FAULT_INTERLOCK for a reason that has
  // nothing to do with the operator.
  std::string reason;
  crane_supervisor::SupervisorConfig config;

  config.deadman_button = 0;
  EXPECT_FALSE(crane_supervisor::validate(config, reason));
  EXPECT_FALSE(reason.empty());

  config.deadman_button = 13;
  EXPECT_FALSE(crane_supervisor::validate(config, reason));

  for (int button = crane_supervisor::kFirstButton; button <= crane_supervisor::kLastButton;
    ++button)
  {
    config.deadman_button = button;
    EXPECT_TRUE(crane_supervisor::validate(config, reason)) << button;
  }

  // The default is button 12, read from the retained stack: the behaviour tree's
  // approval gate returns `msg.button12`, and the TUI's keycode table maps that
  // button to `z`, which is the key wiki/control_architecture.md §6.2 names as
  // the simulated deadman.  It is asserted here so that a later edit of the
  // default is a test failure rather than a silent re-wiring.
  EXPECT_EQ(default_config().deadman_button, 12);
}

TEST(SupervisorCore, AbsenceIsNotHealthBeforeTheFirstMessage)
{
  // wiki/control_architecture.md §5.3: no input may stop arriving without a
  // defined consequence, and "has not started arriving" is the same absence.
  // The remote is healthy here so that the passive state is what is being
  // judged; with nothing arriving at all the stop of §6.1 owns the report.
  crane_supervisor::SupervisorInput input;
  input.remote_ctrl = held_remote();

  const auto decision = crane_supervisor::decide(default_config(), input);
  EXPECT_EQ(decision.fault, crane_supervisor::Fault::StateHealth);
  EXPECT_NE(decision.fault, crane_supervisor::Fault::None);
  EXPECT_FALSE(decision.message.empty());
}

TEST(SupervisorCore, AbsenceIsNotHealthAfterTheStreamStops)
{
  const auto config = default_config();
  auto input = healthy_input();

  // One margin's worth of age is still arriving; a hair past it is not.
  input.pendulum_state.age = config.pendulum_state_timeout;
  EXPECT_EQ(crane_supervisor::decide(config, input).fault, crane_supervisor::Fault::None);

  input.pendulum_state.age = 0.42;
  const auto stopped = crane_supervisor::decide(config, input);
  EXPECT_EQ(stopped.fault, crane_supervisor::Fault::StateHealth);

  // The cause names both numbers, because an operator cannot judge a margin
  // crossing without knowing what was crossed by how much.
  EXPECT_NE(stopped.message.find("0.420"), std::string::npos) << stopped.message;
  EXPECT_NE(stopped.message.find("0.150"), std::string::npos) << stopped.message;
}

TEST(SupervisorCore, AStampAheadOfTheClockIsAFaultRatherThanAFreshSample)
{
  // A stamp slightly ahead is ordinary clock jitter between two hosts and stays
  // inside the margin.  Further ahead than the margin, the age is not a
  // measurement of anything, and reporting the sample fresh off it would hide
  // exactly the absence this input exists to detect (PRD user story 59).
  const auto config = default_config();
  auto input = healthy_input();

  input.pendulum_state.age = -config.pendulum_state_timeout;
  EXPECT_EQ(crane_supervisor::decide(config, input).fault, crane_supervisor::Fault::None);

  input.pendulum_state.age = -2.0 * config.pendulum_state_timeout;
  const auto ahead = crane_supervisor::decide(config, input);
  EXPECT_EQ(ahead.fault, crane_supervisor::Fault::StateHealth);
  EXPECT_NE(ahead.message.find("future"), std::string::npos) << ahead.message;
}

TEST(SupervisorCore, AnInvalidStateCarriesTheBroadcastersOwnCauseThrough)
{
  // The estimator distinguishes six causes behind `valid == false` and says
  // which in `status`.  Restating it here would flatten that distinction, so
  // the string is carried verbatim.
  auto input = healthy_input();
  input.pendulum_state.valid = false;
  input.pendulum_state.status =
    "no estimate: the upstream IMU on K5 is missing or unreadable. Both covariance blocks are "
    "not estimated (-1)";

  const auto decision = crane_supervisor::decide(default_config(), input);
  EXPECT_EQ(decision.fault, crane_supervisor::Fault::StateHealth);
  EXPECT_NE(decision.message.find(input.pendulum_state.status), std::string::npos)
    << decision.message;
}

TEST(SupervisorCore, AnInvalidStateWithNoStatusStillReportsACause)
{
  auto input = healthy_input();
  input.pendulum_state.valid = false;
  input.pendulum_state.status.clear();

  const auto decision = crane_supervisor::decide(default_config(), input);
  EXPECT_EQ(decision.fault, crane_supervisor::Fault::StateHealth);
  EXPECT_FALSE(decision.message.empty());
  EXPECT_NE(decision.message.find("no status string"), std::string::npos) << decision.message;
}

TEST(SupervisorCore, AbsenceOutranksInvalidity)
{
  // A stream that stopped and whose last sample was already bad has to report
  // the absence: it is the cause that removes more of the estimate, and it is
  // the one that is still true right now.
  const auto config = default_config();
  auto input = healthy_input();
  input.pendulum_state.valid = false;
  input.pendulum_state.age = 10.0;

  const auto decision = crane_supervisor::decide(config, input);
  EXPECT_NE(decision.message.find("stopped arriving"), std::string::npos) << decision.message;
}

TEST(SupervisorCore, AHealthyTracerClearsTheFaultAndStillSaysWhatIsNotWatched)
{
  const auto decision = crane_supervisor::decide(default_config(), healthy_input());
  EXPECT_EQ(decision.fault, crane_supervisor::Fault::None);
  EXPECT_FALSE(decision.message.empty());
  // FAULT_NONE from a supervisor that watches one input is not a statement
  // about the machine, and the report says so rather than letting a panel read
  // it as one.
  EXPECT_NE(decision.message.find("only input"), std::string::npos) << decision.message;
}

TEST(SupervisorCore, AReleasedDeadmanIsAnInterlockAndIsCheckedEveryCycle)
{
  // wiki/control_architecture.md §6.2: the check runs continuously and not once
  // at the start of a motion, so the same input decided twice decides the same
  // way both times and the release is caught in whichever cycle it happens in.
  const auto config = default_config();
  auto input = healthy_input();

  EXPECT_EQ(step(config, input).fault, crane_supervisor::Fault::None);

  input.remote_ctrl.deadman_held = false;
  for (int cycle = 0; cycle < 3; ++cycle) {
    const auto released = step(config, input);
    EXPECT_EQ(released.fault, crane_supervisor::Fault::Interlock) << cycle;
    EXPECT_FALSE(released.deadman_held);
    // The cause names the button, because "the interlock is open" is not
    // something an operator can act on and "button 12 is not held" is.
    EXPECT_NE(released.message.find("button 12"), std::string::npos) << released.message;
  }

  // Pressed again, and the interlock clears itself on the next cycle: it is a
  // fact about the operator, not a latch.
  input.remote_ctrl.deadman_held = true;
  const auto pressed = step(config, input);
  EXPECT_EQ(pressed.fault, crane_supervisor::Fault::None);
  EXPECT_TRUE(pressed.deadman_held);
}

TEST(SupervisorCore, TheDeadmanIsReportedOnEveryDecisionWhateverTheFaultIs)
{
  // `deadman_held` is a field of its own on every status, so the button stays
  // visible in the cycles where a more consequential cause owns `fault`.
  const auto config = default_config();
  auto input = healthy_input();
  input.pendulum_state.valid = false;
  input.pendulum_state.status = "the upstream IMU on K5 does not report itself healthy";

  const auto degraded = crane_supervisor::decide(config, input);
  EXPECT_EQ(degraded.fault, crane_supervisor::Fault::StateHealth);
  EXPECT_TRUE(degraded.deadman_held);

  // ... and it is false whenever the remote is not arriving, because a button
  // nobody reported is not a button somebody is holding.
  input.remote_ctrl.received = false;
  EXPECT_FALSE(crane_supervisor::decide(config, input).deadman_held);
}

TEST(SupervisorCore, AnAssertedStopLatchesAndSurvivesTheSignalGoingBackToReleased)
{
  // §6.1: the supervisor consumes the stop so the software comes back in a
  // defined state instead of resuming from whatever it was doing.  A fault that
  // cleared itself when the button was released would resume silently.
  const auto config = default_config();
  auto input = healthy_input();

  input.remote_ctrl.em_stop = true;
  const auto asserted = step(config, input);
  EXPECT_EQ(asserted.fault, crane_supervisor::Fault::EStop);
  EXPECT_TRUE(asserted.estop_latched);

  input.remote_ctrl.em_stop = false;
  for (int cycle = 0; cycle < 5; ++cycle) {
    const auto latched = step(config, input);
    EXPECT_EQ(latched.fault, crane_supervisor::Fault::EStop) << cycle;
    EXPECT_TRUE(latched.estop_latched) << cycle;
    EXPECT_NE(latched.message.find("latched"), std::string::npos) << latched.message;
  }
}

TEST(SupervisorCore, AbsenceOfTheStopSignalIsAssertedAndTheThreeDoNotLookAlike)
{
  // §5.3 and §6.1: a dead GPIO reader, a crashed driver and a released button
  // must not look alike.  All three of the absences raise the stop; the released
  // button raises the interlock; and no two of the four say the same thing.
  const auto config = default_config();

  crane_supervisor::SupervisorInput never_arrived = healthy_input();
  never_arrived.remote_ctrl.received = false;

  crane_supervisor::SupervisorInput stopped_arriving = healthy_input();
  stopped_arriving.remote_ctrl.age = 10.0;

  crane_supervisor::SupervisorInput stamp_ahead = healthy_input();
  stamp_ahead.remote_ctrl.age = -10.0;

  crane_supervisor::SupervisorInput asserted = healthy_input();
  asserted.remote_ctrl.em_stop = true;

  crane_supervisor::SupervisorInput released_button = healthy_input();
  released_button.remote_ctrl.deadman_held = false;

  std::vector<std::string> messages;
  for (const auto & input :
    {never_arrived, stopped_arriving, stamp_ahead, asserted})
  {
    const auto decision = crane_supervisor::decide(config, input);
    EXPECT_EQ(decision.fault, crane_supervisor::Fault::EStop);
    EXPECT_TRUE(decision.estop_latched);
    EXPECT_FALSE(decision.message.empty());
    messages.push_back(decision.message);
  }

  const auto released = crane_supervisor::decide(config, released_button);
  EXPECT_EQ(released.fault, crane_supervisor::Fault::Interlock);
  EXPECT_FALSE(released.estop_latched);
  messages.push_back(released.message);

  for (std::size_t i = 0; i < messages.size(); ++i) {
    for (std::size_t j = i + 1; j < messages.size(); ++j) {
      EXPECT_NE(messages[i], messages[j]) << i << " and " << j << ": " << messages[i];
    }
  }

  // A margin's worth of age is still arriving, on both sides of now, exactly as
  // the passive state's margin is read.
  crane_supervisor::SupervisorInput inside = healthy_input();
  inside.remote_ctrl.age = config.remote_ctrl_timeout;
  EXPECT_EQ(crane_supervisor::decide(config, inside).fault, crane_supervisor::Fault::None);
  inside.remote_ctrl.age = -config.remote_ctrl_timeout;
  EXPECT_EQ(crane_supervisor::decide(config, inside).fault, crane_supervisor::Fault::None);
}

TEST(SupervisorCore, TheStopOutranksTheOtherTwoCausesAndTheInterlockOutranksNothing)
{
  // Precedence, stated once: the stop has no field of its own, so a cycle that
  // reported something else instead would not report it at all.  The deadman
  // does have one, so the interlock can sit below a defect in a stream without
  // becoming invisible.
  const auto config = default_config();

  crane_supervisor::SupervisorInput everything_wrong = healthy_input();
  everything_wrong.pendulum_state.received = false;
  everything_wrong.remote_ctrl.em_stop = true;
  everything_wrong.remote_ctrl.deadman_held = false;
  EXPECT_EQ(crane_supervisor::decide(config, everything_wrong).fault,
    crane_supervisor::Fault::EStop);

  crane_supervisor::SupervisorInput state_and_interlock = healthy_input();
  state_and_interlock.pendulum_state.received = false;
  state_and_interlock.remote_ctrl.deadman_held = false;
  const auto decision = crane_supervisor::decide(config, state_and_interlock);
  EXPECT_EQ(decision.fault, crane_supervisor::Fault::StateHealth);
  EXPECT_FALSE(decision.deadman_held);
}

TEST(SupervisorCore, ClearingIsRefusedWithAnExplanationWhileTheConditionHolds)
{
  const auto config = default_config();

  crane_supervisor::SupervisorInput asserted = healthy_input();
  asserted.remote_ctrl.em_stop = true;
  asserted.estop_latched = true;
  const auto refused = crane_supervisor::clear_fault(config, asserted);
  EXPECT_FALSE(refused.cleared);
  EXPECT_NE(refused.message.find("still asserted"), std::string::npos) << refused.message;

  crane_supervisor::SupervisorInput absent = healthy_input();
  absent.remote_ctrl.received = false;
  absent.estop_latched = true;
  const auto refused_absent = crane_supervisor::clear_fault(config, absent);
  EXPECT_FALSE(refused_absent.cleared);
  EXPECT_NE(refused_absent.message.find("not arriving"), std::string::npos)
    << refused_absent.message;

  // Never an empty success (ROS 2 Interfaces §1): an acknowledgement of nothing
  // is reported as one rather than as a clear.
  crane_supervisor::SupervisorInput nothing_latched = healthy_input();
  const auto nothing = crane_supervisor::clear_fault(config, nothing_latched);
  EXPECT_FALSE(nothing.cleared);
  EXPECT_FALSE(nothing.message.empty());
  EXPECT_NE(nothing.message.find("nothing to acknowledge"), std::string::npos) << nothing.message;
}

TEST(SupervisorCore, AnAcknowledgedStopClearsAndReRaisesOnTheNextCycleIfItRecurs)
{
  const auto config = default_config();
  auto input = healthy_input();

  input.remote_ctrl.em_stop = true;
  ASSERT_TRUE(step(config, input).estop_latched);

  input.remote_ctrl.em_stop = false;
  ASSERT_EQ(step(config, input).fault, crane_supervisor::Fault::EStop);

  // The acknowledgement, judged against the same input the cycle above judged.
  const auto cleared = crane_supervisor::clear_fault(config, input);
  EXPECT_TRUE(cleared.cleared);
  EXPECT_FALSE(cleared.message.empty());
  input.estop_latched = false;

  const auto after = step(config, input);
  EXPECT_EQ(after.fault, crane_supervisor::Fault::None);
  EXPECT_FALSE(after.estop_latched);

  // Pressed again: the latch comes straight back, which is the specified
  // behaviour and not a failed clear.
  input.remote_ctrl.em_stop = true;
  const auto again = step(config, input);
  EXPECT_EQ(again.fault, crane_supervisor::Fault::EStop);
  EXPECT_TRUE(again.estop_latched);
}

TEST(SupervisorCore, TheReportedErrorIsTheMaxOverTheActuatedJointsOfThePositionError)
{
  // ROS 2 Interfaces §6 fixes the field as the max over the actuated joints in
  // rad or m, so it is the *position* error that is reduced and the reduction is
  // over the absolute value: an axis lagging by 0.3 rad is as far out as one
  // leading by 0.3 rad.
  const auto config = default_config();
  auto input = healthy_input();

  input.controller_state = with_error("theta3_arm_joint", -0.30, 0.0);
  EXPECT_DOUBLE_EQ(crane_supervisor::decide(config, input).tracking_error, 0.30);

  // A second, larger deviation on a *length* axis wins, and that it wins across
  // a change of unit is exactly why the number is an indicator and the verdict
  // is per axis.
  for (auto & axis : input.controller_state.axes) {
    if (axis.joint == "q4_big_telescope") {
      axis.position_error = 0.44;
    }
  }
  EXPECT_DOUBLE_EQ(crane_supervisor::decide(config, input).tracking_error, 0.44);

  // The reduction on its own, over an empty report: nothing measured is zero,
  // and the stream's absence is what says so -- not this number.
  EXPECT_DOUBLE_EQ(crane_supervisor::max_position_error({}), 0.0);
}

TEST(SupervisorCore, TheReportedErrorIsCarriedWhateverTheFaultIs)
{
  // Like `deadman_held`, and for the same reason: a field that vanished in the
  // cycles where something more consequential owned `fault` would hide the
  // deviation exactly when someone was looking for it.
  const auto config = default_config();
  auto input = healthy_input();
  input.controller_state = with_error("theta1_slewing_joint", 0.12, 0.0);
  input.remote_ctrl.deadman_held = false;

  const auto interlocked = crane_supervisor::decide(config, input);
  EXPECT_EQ(interlocked.fault, crane_supervisor::Fault::Interlock);
  EXPECT_DOUBLE_EQ(interlocked.tracking_error, 0.12);
}

TEST(SupervisorCore, WithNoToleranceNoTrackingFaultIsRaisedAndTheReportSaysSo)
{
  // The whole point of the fourth acceptance criterion: an axis that is wildly
  // out raises no FAULT_TRACKING when nobody has measured what "out" means, and
  // the clear report does not get to imply a check that was never made.
  const auto config = default_config();
  auto input = healthy_input();
  input.controller_state = with_error("theta2_boom_joint", 5.0, 5.0);

  const auto decision = crane_supervisor::decide(config, input);
  EXPECT_EQ(decision.fault, crane_supervisor::Fault::None);
  EXPECT_DOUBLE_EQ(decision.tracking_error, 5.0);
  EXPECT_NE(decision.message.find("No axis is being compared"), std::string::npos)
    << decision.message;
  EXPECT_NE(decision.message.find("human-only"), std::string::npos) << decision.message;

  // A tolerance that is present and negative -- the state the shipped file is
  // actually in -- reads exactly the same way.
  crane_supervisor::SupervisorConfig negative = config_with_tolerances();
  for (auto & axis : negative.tracking_tolerance) {
    axis.dq_a = -1.0;
  }
  const auto against_negative = crane_supervisor::decide(negative, input);
  EXPECT_EQ(against_negative.fault, crane_supervisor::Fault::None);
  EXPECT_TRUE(crane_supervisor::tracking_breaches(negative, input.controller_state).empty());
}

TEST(SupervisorCore, AVelocityErrorPastItsOwnToleranceIsATypedCauseThatNamesTheAxis)
{
  // wiki/control_architecture.md §5 row 1 and §5.0: the tree branches on this
  // instead of inferring a stall from a deliberately tight goal tolerance.  The
  // comparison is per axis and in the tolerance's own unit -- `dq_a`, rad/s and
  // m/s -- because a max over two units decides nothing.
  const auto config = config_with_tolerances();
  auto input = healthy_input();

  // theta2_boom_joint's tolerance is 0.02 rad/s here.  One hair under is not a
  // fault; one hair over is.
  input.controller_state = with_error("theta2_boom_joint", 0.0, 0.02);
  EXPECT_EQ(crane_supervisor::decide(config, input).fault, crane_supervisor::Fault::None);

  input.controller_state = with_error("theta2_boom_joint", 0.0, -0.05);
  const auto decision = crane_supervisor::decide(config, input);
  EXPECT_EQ(decision.fault, crane_supervisor::Fault::Tracking);
  EXPECT_NE(decision.message.find("theta2_boom_joint"), std::string::npos) << decision.message;
  // The number, the tolerance it was compared against, and the unit of both.
  EXPECT_NE(decision.message.find("-0.0500"), std::string::npos) << decision.message;
  EXPECT_NE(decision.message.find("0.0200"), std::string::npos) << decision.message;
  EXPECT_NE(decision.message.find("rad/s"), std::string::npos) << decision.message;
  // And no other axis is named, because five axes that are fine are not a cause.
  EXPECT_EQ(decision.message.find("theta1_slewing_joint"), std::string::npos) << decision.message;
}

TEST(SupervisorCore, ALengthAxisIsComparedInItsOwnUnitAndSaysWhichItIs)
{
  // Four axes are angles and two are lengths.  Reporting m/s as rad/s is the
  // hidden mixed unit the issue exists to remove, so the unit is read off the
  // axis and printed with the number.
  const auto config = config_with_tolerances();
  auto input = healthy_input();
  input.controller_state = with_error("q9_left_rail_joint", 0.0, 0.5);

  const auto decision = crane_supervisor::decide(config, input);
  EXPECT_EQ(decision.fault, crane_supervisor::Fault::Tracking);
  EXPECT_NE(decision.message.find("q9_left_rail_joint"), std::string::npos) << decision.message;
  EXPECT_NE(decision.message.find("m/s"), std::string::npos) << decision.message;
  EXPECT_EQ(decision.message.find("rad/s"), std::string::npos) << decision.message;
}

TEST(SupervisorCore, AxesArePairedByNameAndTheWorstOffenderIsNamedFirst)
{
  // The controller's `joint_names` ordering is its own, and it is not obliged to
  // match the configuration's.  Pairing by index would compare an angle against
  // a length's tolerance and never say so.
  const auto config = config_with_tolerances();
  auto input = healthy_input();

  // Reversed order, and two axes out: theta1 by 3x its 0.01 tolerance,
  // theta8_rotator_joint by 2x its 0.05 one.
  crane_supervisor::ControllerStateReport reversed;
  reversed.received = true;
  reversed.age = 0.01;
  for (auto it = actuated_joints().rbegin(); it != actuated_joints().rend(); ++it) {
    crane_supervisor::AxisError axis;
    axis.joint = *it;
    axis.velocity_error_reported = true;
    if (*it == "theta1_slewing_joint") {
      axis.velocity_error = 0.03;
    }
    if (*it == "theta8_rotator_joint") {
      axis.velocity_error = 0.10;
    }
    reversed.axes.push_back(axis);
  }
  input.controller_state = reversed;

  const auto breaches = crane_supervisor::tracking_breaches(config, reversed);
  ASSERT_EQ(breaches.size(), 2u);
  // Worst by how far past its own tolerance it is, not by the raw number: the
  // rotator's 0.10 is the larger error and the slewing axis is the further out.
  EXPECT_EQ(breaches[0].joint, "theta1_slewing_joint");
  EXPECT_DOUBLE_EQ(breaches[0].tolerance, 0.01);
  EXPECT_EQ(breaches[1].joint, "theta8_rotator_joint");
  EXPECT_DOUBLE_EQ(breaches[1].tolerance, 0.05);

  const auto decision = crane_supervisor::decide(config, input);
  EXPECT_EQ(decision.fault, crane_supervisor::Fault::Tracking);
  EXPECT_LT(
    decision.message.find("theta1_slewing_joint"),
    decision.message.find("theta8_rotator_joint")) << decision.message;
}

TEST(SupervisorCore, AnAxisTheControllerReportsNoVelocityErrorForIsNotCompared)
{
  // The trajectory controller fills `error.velocities` only when it holds a
  // velocity state interface and a velocity or effort command interface.  An
  // empty field read as a zero error would be a crane that tracks perfectly by
  // construction, so the absence is carried and the report says which absence
  // it is.
  const auto config = config_with_tolerances();
  auto input = healthy_input();
  for (auto & axis : input.controller_state.axes) {
    axis.velocity_error_reported = false;
    axis.velocity_error = 9.0;
    axis.position_error = 0.7;
  }

  const auto decision = crane_supervisor::decide(config, input);
  EXPECT_EQ(decision.fault, crane_supervisor::Fault::None);
  EXPECT_DOUBLE_EQ(decision.tracking_error, 0.7);
  EXPECT_TRUE(crane_supervisor::tracking_breaches(config, input.controller_state).empty());
  EXPECT_NE(decision.message.find("velocity state interface"), std::string::npos)
    << decision.message;
}

TEST(SupervisorCore, AControllerThatStoppedPublishingIsNotACraneTrackingPerfectly)
{
  // §5.3 applied to the third input.  All three absences are faults, none of
  // them is a tracking_error of zero standing on its own, and no two of them
  // say the same thing.
  const auto config = default_config();

  auto never = healthy_input();
  never.controller_state = crane_supervisor::ControllerStateReport{};

  auto stopped = healthy_input();
  stopped.controller_state = with_error("theta1_slewing_joint", 0.9, 0.0);
  stopped.controller_state.age = 10.0;

  auto ahead = healthy_input();
  ahead.controller_state.age = -10.0;

  std::vector<std::string> messages;
  for (const auto & input : {never, stopped, ahead}) {
    const auto decision = crane_supervisor::decide(config, input);
    EXPECT_EQ(decision.fault, crane_supervisor::Fault::StateHealth);
    // The error the last sample carried is not republished as if it were
    // current, and the zero is never the only thing said about the stream.
    EXPECT_DOUBLE_EQ(decision.tracking_error, 0.0);
    EXPECT_FALSE(decision.message.empty());
    messages.push_back(decision.message);
  }
  EXPECT_NE(messages[0], messages[1]);
  EXPECT_NE(messages[1], messages[2]);
  EXPECT_NE(messages[0], messages[2]);
  EXPECT_NE(messages[1].find("10.000"), std::string::npos) << messages[1];
  EXPECT_NE(messages[2].find("future"), std::string::npos) << messages[2];

  // A margin's worth of age is still arriving, on both sides of now, exactly as
  // the other two inputs are read.
  auto inside = healthy_input();
  inside.controller_state.age = config.controller_state_timeout;
  EXPECT_EQ(crane_supervisor::decide(config, inside).fault, crane_supervisor::Fault::None);
  inside.controller_state.age = -config.controller_state_timeout;
  EXPECT_EQ(crane_supervisor::decide(config, inside).fault, crane_supervisor::Fault::None);
}

TEST(SupervisorCore, TheStopAndTheStateOutrankTrackingAndTrackingOutranksTheInterlock)
{
  // Precedence, stated once.  A tracking excess is a defect and a released
  // deadman is the ordinary resting state of the machine, so tracking sits
  // above the interlock; the passive state sits above tracking because a
  // supervisor whose own view of the crane is stale should say that first.
  const auto config = config_with_tolerances();

  auto everything = healthy_input();
  everything.controller_state = with_error("theta1_slewing_joint", 0.0, 1.0);
  everything.remote_ctrl.deadman_held = false;
  EXPECT_EQ(crane_supervisor::decide(config, everything).fault, crane_supervisor::Fault::Tracking);

  everything.pendulum_state.received = false;
  EXPECT_EQ(
    crane_supervisor::decide(config, everything).fault, crane_supervisor::Fault::StateHealth);

  everything.remote_ctrl.em_stop = true;
  EXPECT_EQ(crane_supervisor::decide(config, everything).fault, crane_supervisor::Fault::EStop);
}

TEST(SupervisorCore, NothingHereActs)
{
  // The whole action of this supervisor is to report.  There is no stop, no
  // ramp, no deactivation and no command in the decision it returns -- on a
  // tracking fault least of all, which is the whole of §5.0: the signal is
  // fixed and the decision stays in the task layer.
  const auto config = config_with_tolerances();
  auto input = healthy_input();
  input.controller_state = with_error("theta3_arm_joint", 0.2, 1.0);

  const auto tracking = crane_supervisor::decide(config, input);
  EXPECT_EQ(tracking.fault, crane_supervisor::Fault::Tracking);
  EXPECT_NE(tracking.message.find("stays in the task layer"), std::string::npos)
    << tracking.message;
  EXPECT_NE(tracking.message.find("Nothing was stopped"), std::string::npos) << tracking.message;

  input.remote_ctrl.em_stop = true;
  const auto decision = crane_supervisor::decide(config, input);
  EXPECT_EQ(decision.mode, crane_supervisor::Mode::Idle);
  EXPECT_FALSE(decision.inside_working_cell);
  EXPECT_NE(decision.message.find("not protection"), std::string::npos) << decision.message;
}

/// The whole reachable input space of this slice, one struct per combination.
/**
 * Swept rather than enumerated by hand, because the cases that go wrong are the
 * ones nobody thought to name. The tolerance list is a parameter of the sweep
 * too: with tolerances and without are two different decision surfaces, and both
 * are reachable from a deployment.
 */
std::vector<crane_supervisor::SupervisorInput> every_input()
{
  std::vector<crane_supervisor::SupervisorInput> inputs;
  for (const bool received : {false, true}) {
    for (const bool valid : {false, true}) {
      for (const double age : {-100.0, -0.05, 0.0, 0.05, 100.0}) {
        for (const char * status : {"", "a cause the estimator distinguished"}) {
          for (const bool remote_received : {false, true}) {
            for (const bool em_stop : {false, true}) {
              for (const bool deadman : {false, true}) {
                for (const bool latched : {false, true}) {
                  for (const bool controller_received : {false, true}) {
                    for (const double velocity_error : {0.0, 100.0}) {
                      crane_supervisor::SupervisorInput input;
                      input.pendulum_state.received = received;
                      input.pendulum_state.valid = valid;
                      input.pendulum_state.age = age;
                      input.pendulum_state.status = status;
                      input.remote_ctrl.received = remote_received;
                      input.remote_ctrl.em_stop = em_stop;
                      input.remote_ctrl.deadman_held = deadman;
                      input.remote_ctrl.age = age;
                      input.estop_latched = latched;
                      input.controller_state = tracking_controller();
                      input.controller_state.received = controller_received;
                      input.controller_state.age = age;
                      for (auto & axis : input.controller_state.axes) {
                        axis.velocity_error = velocity_error;
                        axis.position_error = velocity_error;
                      }
                      inputs.push_back(input);
                    }
                  }
                }
              }
            }
          }
        }
      }
    }
  }
  return inputs;
}

TEST(SupervisorCore, EveryReachableDecisionCarriesACause)
{
  // PRD user story 53, asserted over the whole reachable input space rather
  // than over the cases the tests above happen to name: a report with a fault
  // and no words is the inferred abort §5.0 removes, back again.
  for (const auto & config : {default_config(), config_with_tolerances()}) {
    for (const auto & input : every_input()) {
      const auto decision = crane_supervisor::decide(config, input);
      EXPECT_FALSE(decision.message.empty());
      // Mode arbitration is a later issue; until then the supervisor reports the
      // one mode it has verified, whatever it was told.
      EXPECT_EQ(decision.mode, crane_supervisor::Mode::Idle);
      // The working cell is still not computed in this slice, and carries the
      // value that claims nothing rather than a stub that claims something.
      EXPECT_FALSE(decision.inside_working_cell);
      // The tracking error is a measurement of what arrived, and it is zero in
      // exactly the case where nothing usable did.
      if (decision.tracking_error != 0.0) {
        EXPECT_TRUE(input.controller_state.received);
      }
      // A latch is never lowered by a status cycle, whatever else it decides.
      if (input.estop_latched) {
        EXPECT_TRUE(decision.estop_latched);
        EXPECT_EQ(decision.fault, crane_supervisor::Fault::EStop);
      }
      // And the acknowledgement is total too: it answers every input with words.
      EXPECT_FALSE(crane_supervisor::clear_fault(config, input).message.empty());
    }
  }
}

TEST(SupervisorCore, NoTrackingFaultIsReachableWithoutATolerance)
{
  // The fourth acceptance criterion, over the whole input space rather than
  // over one case: with no number to compare against, FAULT_TRACKING is not
  // reachable at all -- however far out any axis is.
  const auto config = default_config();
  for (const auto & input : every_input()) {
    EXPECT_NE(crane_supervisor::decide(config, input).fault, crane_supervisor::Fault::Tracking);
  }
}

TEST(SupervisorCore, TheFaultsThisSliceRaisesAreStateHealthTrackingEStopAndInterlock)
{
  // Every other cause of the §5 table belongs to a later issue.  A supervisor
  // that raised one of them from an input it does not have would be reporting a
  // check it never made.
  for (const auto & config : {default_config(), config_with_tolerances()}) {
    for (const auto & input : every_input()) {
      const auto fault = crane_supervisor::decide(config, input).fault;
      EXPECT_TRUE(
        fault == crane_supervisor::Fault::None ||
        fault == crane_supervisor::Fault::StateHealth ||
        fault == crane_supervisor::Fault::Tracking ||
        fault == crane_supervisor::Fault::EStop ||
        fault == crane_supervisor::Fault::Interlock)
        << static_cast<int>(fault);
    }
  }
}
