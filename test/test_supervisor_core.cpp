// The decision core, offline. No ROS, no DDS, no clock is linked into this
// binary and none is reachable from it, so every cause can be produced exactly
// and one at a time -- which is the point of the core being ROS-free at all.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "crane_supervisor/supervisor_core.hpp"

namespace
{

using crane_supervisor::Input;
using crane_supervisor::Staleness;
using crane_supervisor::kInputCount;
using crane_supervisor::index_of;
using crane_supervisor::policy_of;

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

/// The four sway numbers `config/crane_supervisor.yaml` ships.
/**
 * Written out here for the same reason the deadlines below are: `SwayBound` is
 * value-initialised to zero, zero is not a bound and `validate()` refuses one,
 * so a test of the decision has to say which bounds it is deciding against
 * exactly as a deployment does. Their derivations, and what would replace each,
 * are in `src/crane_supervisor_parameters.yaml`; the properties those
 * derivations claim are asserted in `test_sway_monitor.cpp`.
 */
crane_supervisor::SwayBound shipped_sway_bound()
{
  crane_supervisor::SwayBound bound;
  bound.dq_u_max = {{0.4, 0.4}};
  bound.dq_u_settled = {{0.04, 0.04}};
  bound.settled_release_factor = 1.5;
  bound.settle_dwell = 2.0;
  return bound;
}

/// The status period of ROS 2 Interfaces §2, as the step a sequence advances by.
constexpr double kCycle = 1.0 / 20.0;

/// The clock a fixture starts at, s. Epoch-scale rather than zero, because that
/// is what a node's clock reads and the dwell arithmetic runs on it.
constexpr double kFirstCycle = 1'700'000'000.0;

/// The part of a report the fault composed, without the settled clause.
/**
 * Every report ends in the settled clause, and the clause names both passive
 * coordinates in rad/s. A test asserting that a *length* axis is not reported in
 * rad/s therefore has to look at the sentence the fault wrote and not at the
 * whole string. The seam comes off the core rather than being written out here,
 * so the two cannot drift apart.
 */
std::string without_settled_clause(const std::string & message)
{
  return message.substr(0, message.find(crane_supervisor::kSettledClausePrefix));
}

/// The four shipped freshness deadlines, s, in `Input` order.
/**
 * Written out here rather than read off a struct default, because there is no
 * struct default: the numbers and the derivation of each live once, in
 * `src/crane_supervisor_parameters.yaml`, and a `SupervisorConfig` that was
 * handed none is one `validate()` refuses. That is the point -- an input with no
 * deadline is a node that does not start -- so a test of the decision has to say
 * which margins it is deciding against, exactly as a deployment does.
 */
crane_supervisor::SupervisorConfig with_shipped_deadlines(
  crane_supervisor::SupervisorConfig config)
{
  config.deadline(Input::PendulumState) = 0.15;
  config.deadline(Input::RemoteCtrl) = 0.25;
  config.deadline(Input::ControllerState) = 0.15;
  config.deadline(Input::ControllerHealth) = 0.25;
  // The polled view of the controller manager. Not one of the four -- it is a
  // service and not a stream -- and refused by `validate()` all the same when
  // it is missing, so a fixture that omitted it would not be a configuration a
  // node could start from.
  config.controller_manager_deadline = 0.25;
  // The horizon producer's margin, refused by `validate()` when it is missing
  // for the same reason: PRD §10 step 2's check has to compare an age against
  // something, and a fixture that omitted it would not be a configuration a node
  // could start from either.
  config.horizon_deadline = 0.3;
  // The arm claim's controllers, as `config/crane_supervisor.yaml` ships them:
  // the FOLLOW pair in the cascade's activation order, MODE_MPC as that pair
  // without the trajectory controller (PRD §10 step 3 -- the same inner loop
  // carries both paths), and nothing for MODE_MANUAL, which is the deployment's
  // own state and not a convenience since no CBS profile composes one.
  config.mode_controllers[crane_supervisor::index_of(crane_supervisor::Mode::Follow)] = {
    "crane_velocity_controller", "trajectory_controller_a2b"};
  config.mode_controllers[crane_supervisor::index_of(crane_supervisor::Mode::Mpc)] = {
    "crane_velocity_controller"};
  config.mpc_node = "crane_mpc";
  config.sway = shipped_sway_bound();
  return config;
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
  return with_shipped_deadlines(config);
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
  return with_shipped_deadlines(config);
}

/// A trajectory controller tracking exactly, reporting all six axes.
crane_supervisor::ControllerStateReport tracking_controller()
{
  crane_supervisor::ControllerStateReport report;
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

/// A remote with the stop released and the deadman held.
crane_supervisor::RemoteCtrlReport held_remote()
{
  crane_supervisor::RemoteCtrlReport remote;
  remote.deadman_held = true;
  remote.em_stop = false;
  return remote;
}

/// The inner velocity loop with nothing to report.
crane_supervisor::ControllerHealthReport healthy_inner_loop()
{
  crane_supervisor::ControllerHealthReport inner_loop;
  inner_loop.fault = crane_supervisor::Fault::None;
  return inner_loop;
}

/// The report `crane_velocity_controller` publishes on the `hardware` profile
/// today: prerequisite 4 is missing, so the gripper axis ran PI only and the
/// loop raises the commissioning code and names the axis.
crane_supervisor::ControllerHealthReport uncommissioned_gripper()
{
  crane_supervisor::ControllerHealthReport inner_loop = healthy_inner_loop();
  inner_loop.fault = crane_supervisor::Fault::NotCommissioned;
  inner_loop.feedforward_free_joints = {"q9_left_rail_joint"};
  return inner_loop;
}

/// The same loop with one of its own codes, and no axis to name for it.
crane_supervisor::ControllerHealthReport inner_loop_fault(crane_supervisor::Fault fault)
{
  crane_supervisor::ControllerHealthReport inner_loop = healthy_inner_loop();
  inner_loop.fault = fault;
  return inner_loop;
}

/// All four inputs arriving, in time, trusted, with the operator holding the
/// button and the inner loop reporting nothing wrong with itself.
/**
 * The arriving half is set over the whole registry rather than input by input,
 * deliberately: a fixture that named its four streams by hand would leave a
 * fifth input default-constructed -- which is to say never arrived -- and every
 * test below would then be asserting about that fifth input's absence instead of
 * about what it was written for.
 */
crane_supervisor::SupervisorInput healthy_input()
{
  crane_supervisor::SupervisorInput input;
  for (std::size_t i = 0; i < kInputCount; ++i) {
    input.streams[i].received = true;
    input.streams[i].age = 0.01;
  }
  input.pendulum_state.valid = true;
  input.pendulum_state.status = "complementary filter on the two bracketing IMUs";
  // A still crane. The rate is filled because a `PendulumStateReport` that was
  // never given one carries NaN -- the absence of a measurement -- and a fixture
  // built on the absence would be asserting about the sway being unknowable
  // rather than about whatever the test was written for.
  input.pendulum_state.velocity = {{0.0, 0.0}};
  input.sampled_at = kFirstCycle;
  input.remote_ctrl = held_remote();
  input.controller_state = tracking_controller();
  input.controller_health = healthy_inner_loop();
  return input;
}

/// The same input with both passive rates set.
crane_supervisor::SupervisorInput swinging(double dq_u)
{
  crane_supervisor::SupervisorInput input = healthy_input();
  input.pendulum_state.velocity = {{dq_u, dq_u}};
  return input;
}

/// The same input with one named stream made stale in one named way.
crane_supervisor::SupervisorInput stale(
  const crane_supervisor::SupervisorConfig & config, Input which, Staleness cause)
{
  crane_supervisor::SupervisorInput input = healthy_input();
  switch (cause) {
    case Staleness::NeverArrived:
      input.stream(which).received = false;
      break;
    case Staleness::StoppedArriving:
      input.stream(which).age = 10.0 * config.deadline(which) + 1.0;
      break;
    case Staleness::StampAhead:
      input.stream(which).age = -10.0 * config.deadline(which) - 1.0;
      break;
    default:
      break;
  }
  return input;
}

/// One decision, with everything a cycle hands to its successor carried the way
/// the node carries it: the emergency-stop latch, the sway dwell, and the clock.
crane_supervisor::SupervisorDecision step(
  const crane_supervisor::SupervisorConfig & config, crane_supervisor::SupervisorInput & input)
{
  const auto decision = crane_supervisor::decide(config, input);
  input.estop_latched = decision.estop_latched;
  input.sway = decision.sway;
  input.sampled_at += kCycle;
  return decision;
}

/// The three ways a stream can fail to be fresh from the transport alone.
const std::array<Staleness, 3> & transport_causes()
{
  static const std::array<Staleness, 3> causes{
    Staleness::NeverArrived, Staleness::StoppedArriving, Staleness::StampAhead};
  return causes;
}

}  // namespace

TEST(SupervisorCore, EveryInputHasADeadlineAndNoneIsExempt)
{
  // The first acceptance criterion of issue 023, and the reason the deadlines
  // live in an array `Input` sizes rather than in four named doubles: an input
  // added to the enum and given no margin has a slot value-initialised to zero,
  // and zero is not a deadline.  So the node refuses to start rather than
  // publishing a status about a stream nobody is watching -- the guard is
  // structural, and this loop is over the registry rather than over a list.
  std::string reason;
  EXPECT_TRUE(crane_supervisor::validate(default_config(), reason)) << reason;

  // A `SupervisorConfig` nobody handed a margin to is refused outright, which is
  // what makes "no default deadline" a rule and not a comment.
  EXPECT_FALSE(crane_supervisor::validate(crane_supervisor::SupervisorConfig{}, reason));
  EXPECT_FALSE(reason.empty());

  for (std::size_t i = 0; i < kInputCount; ++i) {
    const Input which = static_cast<Input>(i);
    for (const double refused : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN()}) {
      crane_supervisor::SupervisorConfig config = default_config();
      config.deadline(which) = refused;
      EXPECT_FALSE(crane_supervisor::validate(config, reason)) << policy_of(which).topic;
      // And it says which input, because "a margin is wrong" is not something an
      // operator or an integrator can act on and "this topic has none" is.
      EXPECT_NE(reason.find(policy_of(which).topic), std::string::npos) << reason;
    }
  }
}

TEST(SupervisorCore, TheRegistryIsTheOneListAndItsRowsMatchTheirInputs)
{
  // The table is what makes a report name its own stream, so a row sitting at
  // the wrong index would report the wrong topic for the wrong input.  A
  // `static_assert` in the header refuses that at compile time; this asserts the
  // properties a compiler cannot -- that no two inputs share a topic and that
  // every row is filled in.
  EXPECT_EQ(crane_supervisor::kInputPolicies.size(), kInputCount);
  for (std::size_t i = 0; i < kInputCount; ++i) {
    const crane_supervisor::InputPolicy & policy = crane_supervisor::kInputPolicies[i];
    EXPECT_EQ(index_of(policy.input), i);
    EXPECT_NE(std::string(policy.topic), "");
    EXPECT_NE(std::string(policy.type), "");
    EXPECT_NE(std::string(policy.label), "");
    EXPECT_NE(std::string(policy.consequence), "");
    EXPECT_NE(std::string(policy.advice), "");
    for (std::size_t j = i + 1; j < kInputCount; ++j) {
      EXPECT_NE(std::string(policy.topic), crane_supervisor::kInputPolicies[j].topic);
    }
  }
}

TEST(SupervisorCore, EveryInputIsSweptAndReportsItsOwnInputAndItsOwnCause)
{
  // wiki/control_architecture.md §5.3, over the registry rather than over the
  // four inputs somebody remembered: every input that stops arriving ends in the
  // fault its policy row names, and the message says which input it was about
  // and which staleness cause fired.  An input added to `Input` is swept by this
  // loop the day it is added, and fails here until `decide()` consults it.
  const auto config = default_config();

  for (std::size_t i = 0; i < kInputCount; ++i) {
    const Input which = static_cast<Input>(i);
    std::vector<std::string> messages;

    for (const Staleness cause : transport_causes()) {
      const auto decision = crane_supervisor::decide(config, stale(config, which, cause));
      EXPECT_EQ(decision.fault, policy_of(which).fault) << policy_of(which).topic;
      EXPECT_NE(decision.fault, crane_supervisor::Fault::None) << policy_of(which).topic;
      // Which input.
      EXPECT_NE(decision.message.find(policy_of(which).topic), std::string::npos)
        << decision.message;
      EXPECT_NE(decision.message.find(policy_of(which).type), std::string::npos)
        << decision.message;
      // Which cause.
      EXPECT_NE(decision.message.find("Staleness cause"), std::string::npos) << decision.message;
      messages.push_back(decision.message);
    }

    // An input that never arrived and one that stopped arriving are both faults
    // and are not the same fault to chase: "never connected" sends an integrator
    // after a launch file and "died" sends them after a process.
    for (std::size_t a = 0; a < messages.size(); ++a) {
      for (std::size_t b = a + 1; b < messages.size(); ++b) {
        EXPECT_NE(messages[a], messages[b]) << policy_of(which).topic << ": " << messages[a];
      }
    }
  }
}

TEST(SupervisorCore, TheDeadlinesArePerInputAndNotOneNumber)
{
  // §5.3's margins are properties of the streams they judge: the passive state
  // comes off the manager's 100 Hz cycle and the remote is a 20 Hz contract, so
  // an age that is healthy on one is a dead publisher on the other.  A single
  // global margin would report the fast stream late or the slow one falsely, and
  // this asserts the two are actually judged apart.
  auto config = default_config();
  config.deadline(Input::PendulumState) = 0.05;
  config.deadline(Input::RemoteCtrl) = 1.0;

  auto input = healthy_input();
  input.stream(Input::PendulumState).age = 0.3;
  input.stream(Input::RemoteCtrl).age = 0.3;

  // The same age, the same cycle: stale on one input and fresh on the other.
  const auto causes = crane_supervisor::freshness(config, input);
  EXPECT_EQ(causes[index_of(Input::PendulumState)], Staleness::StoppedArriving);
  EXPECT_EQ(causes[index_of(Input::RemoteCtrl)], Staleness::Fresh);

  const auto decision = crane_supervisor::decide(config, input);
  EXPECT_EQ(decision.fault, crane_supervisor::Fault::StateHealth);
  EXPECT_NE(decision.message.find("0.050"), std::string::npos) << decision.message;
}

TEST(SupervisorCore, TheFreshnessSweepIsTotalBoundedAndNeedsNoMessageToNoticeAStop)
{
  // The sweep answers for every input on every call, from the state of the
  // input struct alone: no history, no counters, nothing that has to be fed by a
  // message arriving.  That is what makes a deadline able to fire on the one
  // stream it exists for -- the one where no callback will ever run again.
  const auto config = default_config();
  const auto input = stale(config, Input::ControllerHealth, Staleness::StoppedArriving);

  const auto first = crane_supervisor::freshness(config, input);
  const auto second = crane_supervisor::freshness(config, input);
  EXPECT_EQ(first, second);
  EXPECT_EQ(first.size(), kInputCount);
  EXPECT_EQ(first[index_of(Input::ControllerHealth)], Staleness::StoppedArriving);
  for (std::size_t i = 0; i < kInputCount; ++i) {
    if (i != index_of(Input::ControllerHealth)) {
      EXPECT_EQ(first[i], Staleness::Fresh) << crane_supervisor::kInputPolicies[i].topic;
    }
  }

  // The margin is symmetric and closed: exactly a deadline's worth of age is
  // still arriving, on both sides of now, so ordinary clock jitter between two
  // hosts is not a fault.
  for (std::size_t i = 0; i < kInputCount; ++i) {
    const Input which = static_cast<Input>(i);
    EXPECT_EQ(
      crane_supervisor::freshness_of(config.deadline(which), {true, config.deadline(which)}),
      Staleness::Fresh) << policy_of(which).topic;
    EXPECT_EQ(
      crane_supervisor::freshness_of(config.deadline(which), {true, -config.deadline(which)}),
      Staleness::Fresh) << policy_of(which).topic;
    EXPECT_EQ(
      crane_supervisor::freshness_of(config.deadline(which), {false, 0.0}),
      Staleness::NeverArrived) << policy_of(which).topic;
  }
}

TEST(SupervisorCore, EveryInputRecoversOnItsOwnExceptTheOneThatLatches)
{
  // §5.3's rule is symmetric: an input that starts arriving again inside its
  // deadline clears its own fault, with no acknowledgement.  The emergency stop
  // is the single exception and `InputPolicy::latches` is where that is written
  // down -- §6.1 asks for the software to come back in a defined state, so the
  // stop survives the signal returning and only `/crane/clear_fault` lowers it.
  const auto config = default_config();

  for (std::size_t i = 0; i < kInputCount; ++i) {
    const Input which = static_cast<Input>(i);
    for (const Staleness cause : transport_causes()) {
      auto input = stale(config, which, cause);
      ASSERT_EQ(step(config, input).fault, policy_of(which).fault) << policy_of(which).topic;

      // The stream comes back, inside its deadline.
      input.stream(which) = crane_supervisor::StreamReport{true, 0.01};
      const auto recovered = step(config, input);

      if (policy_of(which).latches) {
        EXPECT_EQ(recovered.fault, policy_of(which).fault) << policy_of(which).topic;
        EXPECT_TRUE(recovered.estop_latched);
        const auto cleared = crane_supervisor::clear_fault(config, input);
        EXPECT_TRUE(cleared.cleared) << cleared.message;
        input.estop_latched = false;
        EXPECT_EQ(step(config, input).fault, crane_supervisor::Fault::None);
      } else {
        EXPECT_EQ(recovered.fault, crane_supervisor::Fault::None)
          << policy_of(which).topic << ": " << recovered.message;
      }
    }
  }
}

TEST(SupervisorCore, TheThreeConstantsStayDistinctRatherThanCollapsingIntoOne)
{
  // PRD user story 52 and the third acceptance criterion of issue 023.  A stale
  // state, an expired reference and an absent stop are three different things to
  // do next, so they are three constants -- collapsing them into one is the
  // defect the policy exists to prevent, and it is the kind of defect that is
  // invisible until an operator is standing in front of the panel.
  const auto config = default_config();

  const auto state = crane_supervisor::decide(
    config, stale(config, Input::PendulumState, Staleness::StoppedArriving));
  EXPECT_EQ(state.fault, crane_supervisor::Fault::StateHealth);

  auto reference_input = healthy_input();
  reference_input.controller_health = inner_loop_fault(crane_supervisor::Fault::ReferenceStale);
  const auto reference = crane_supervisor::decide(config, reference_input);
  EXPECT_EQ(reference.fault, crane_supervisor::Fault::ReferenceStale);

  const auto stop = crane_supervisor::decide(
    config, stale(config, Input::RemoteCtrl, Staleness::NeverArrived));
  EXPECT_EQ(stop.fault, crane_supervisor::Fault::EStop);

  EXPECT_NE(state.fault, reference.fault);
  EXPECT_NE(state.fault, stop.fault);
  EXPECT_NE(reference.fault, stop.fault);
  EXPECT_NE(state.message, reference.message);
  EXPECT_NE(state.message, stop.message);
  EXPECT_NE(reference.message, stop.message);
}

TEST(SupervisorCore, AMissingToleranceIsReportedRatherThanRefused)
{
  // The number does not exist yet and is human-owned.  A supervisor that
  // refused to start over it would withhold the emergency stop, the state
  // health and the interlock to report one duty it cannot perform, so the
  // absence is a warning at configuration and not a validation failure.  A
  // missing *deadline* is the opposite and is refused -- see the first test.
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
  crane_supervisor::SupervisorConfig config = default_config();

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
  const auto config = default_config();
  auto input = healthy_input();
  input.stream(Input::PendulumState).received = false;

  const auto decision = crane_supervisor::decide(config, input);
  EXPECT_EQ(decision.fault, crane_supervisor::Fault::StateHealth);
  EXPECT_NE(decision.fault, crane_supervisor::Fault::None);
  EXPECT_FALSE(decision.message.empty());
}

TEST(SupervisorCore, AbsenceIsNotHealthAfterTheStreamStops)
{
  const auto config = default_config();
  auto input = healthy_input();

  // One deadline's worth of age is still arriving; a hair past it is not.
  input.stream(Input::PendulumState).age = config.deadline(Input::PendulumState);
  EXPECT_EQ(crane_supervisor::decide(config, input).fault, crane_supervisor::Fault::None);

  input.stream(Input::PendulumState).age = 0.42;
  const auto stopped = crane_supervisor::decide(config, input);
  EXPECT_EQ(stopped.fault, crane_supervisor::Fault::StateHealth);

  // The cause names both numbers, because an operator cannot judge a margin
  // crossing without knowing what was crossed by how much.
  EXPECT_NE(stopped.message.find("0.420"), std::string::npos) << stopped.message;
  EXPECT_NE(stopped.message.find("0.150"), std::string::npos) << stopped.message;
  EXPECT_NE(stopped.message.find("stopped arriving"), std::string::npos) << stopped.message;
}

TEST(SupervisorCore, AStampAheadOfTheClockIsAFaultRatherThanAFreshSample)
{
  // A stamp slightly ahead is ordinary clock jitter between two hosts and stays
  // inside the margin.  Further ahead than the margin, the age is not a
  // measurement of anything, and reporting the sample fresh off it would hide
  // exactly the absence this input exists to detect (PRD user story 59).
  const auto config = default_config();
  auto input = healthy_input();

  input.stream(Input::PendulumState).age = -config.deadline(Input::PendulumState);
  EXPECT_EQ(crane_supervisor::decide(config, input).fault, crane_supervisor::Fault::None);

  input.stream(Input::PendulumState).age = -2.0 * config.deadline(Input::PendulumState);
  const auto ahead = crane_supervisor::decide(config, input);
  EXPECT_EQ(ahead.fault, crane_supervisor::Fault::StateHealth);
  EXPECT_NE(ahead.message.find("future"), std::string::npos) << ahead.message;
}

TEST(SupervisorCore, TheProducersOwnFlagIsTheThirdCauseAndItsAccountIsCarriedThrough)
{
  // The three causes §5.3 now lists are the flag, the age and the sample that
  // stopped refreshing behind a header that keeps moving.  The age is the one
  // this supervisor measures itself; the other two are the broadcaster's, and
  // they arrive here behind `valid == false` with the broadcaster's own account
  // of which one fired.  Restating it would flatten a distinction the estimator
  // went to some trouble to make.
  auto input = healthy_input();
  input.pendulum_state.valid = false;
  input.pendulum_state.status =
    "not to be trusted: the upstream IMU on K5 answered with the same seven values for 10 cycles";

  const auto decision = crane_supervisor::decide(default_config(), input);
  EXPECT_EQ(decision.fault, crane_supervisor::Fault::StateHealth);
  EXPECT_NE(decision.message.find(input.pendulum_state.status), std::string::npos)
    << decision.message;
  // And it says which of the three fired, and that this one is not measured
  // here: a supervisor cannot ask whether twelve booleans moved.
  EXPECT_NE(decision.message.find("health flag"), std::string::npos) << decision.message;
  EXPECT_NE(decision.message.find("refreshing"), std::string::npos) << decision.message;
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
  input.stream(Input::PendulumState).age = 10.0;

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
  // Including the gap it does *not* close: §6.3's controller-side timeouts are
  // still ad hoc, and the supervisor reporting that honestly is the correct
  // interim answer rather than papering over it.
  EXPECT_NE(decision.message.find("never times out"), std::string::npos) << decision.message;
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
  input.stream(Input::RemoteCtrl).received = false;
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

  std::vector<std::string> messages;
  for (const Staleness cause : transport_causes()) {
    const auto decision = crane_supervisor::decide(config, stale(config, Input::RemoteCtrl, cause));
    EXPECT_EQ(decision.fault, crane_supervisor::Fault::EStop);
    EXPECT_TRUE(decision.estop_latched);
    EXPECT_FALSE(decision.message.empty());
    // Every one of them says it is diagnosis and not the protection.
    EXPECT_NE(decision.message.find("not protection"), std::string::npos) << decision.message;
    messages.push_back(decision.message);
  }

  crane_supervisor::SupervisorInput asserted = healthy_input();
  asserted.remote_ctrl.em_stop = true;
  const auto stopped = crane_supervisor::decide(config, asserted);
  EXPECT_EQ(stopped.fault, crane_supervisor::Fault::EStop);
  messages.push_back(stopped.message);

  crane_supervisor::SupervisorInput released_button = healthy_input();
  released_button.remote_ctrl.deadman_held = false;
  const auto released = crane_supervisor::decide(config, released_button);
  EXPECT_EQ(released.fault, crane_supervisor::Fault::Interlock);
  EXPECT_FALSE(released.estop_latched);
  messages.push_back(released.message);

  for (std::size_t i = 0; i < messages.size(); ++i) {
    for (std::size_t j = i + 1; j < messages.size(); ++j) {
      EXPECT_NE(messages[i], messages[j]) << i << " and " << j << ": " << messages[i];
    }
  }
}

TEST(SupervisorCore, TheStopOutranksTheOtherTwoCausesAndTheInterlockOutranksNothing)
{
  // Precedence, stated once: the stop has no field of its own, so a cycle that
  // reported something else instead would not report it at all.  The deadman
  // does have one, so the interlock can sit below a defect in a stream without
  // becoming invisible.
  const auto config = default_config();

  crane_supervisor::SupervisorInput everything_wrong = healthy_input();
  everything_wrong.stream(Input::PendulumState).received = false;
  everything_wrong.remote_ctrl.em_stop = true;
  everything_wrong.remote_ctrl.deadman_held = false;
  EXPECT_EQ(crane_supervisor::decide(config, everything_wrong).fault,
    crane_supervisor::Fault::EStop);

  crane_supervisor::SupervisorInput state_and_interlock = healthy_input();
  state_and_interlock.stream(Input::PendulumState).received = false;
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
  absent.stream(Input::RemoteCtrl).received = false;
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
  // An axis that is wildly out raises no FAULT_TRACKING when nobody has measured
  // what "out" means, and the clear report does not get to imply a check that
  // was never made.
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
  // The sentence the fault composed, without the settled clause every report
  // ends in: that clause reports the two passive rates and is in rad/s by
  // construction, so the mixed-unit assertion is about the tracking half.
  const std::string tracking = without_settled_clause(decision.message);
  EXPECT_NE(tracking.find("q9_left_rail_joint"), std::string::npos) << tracking;
  EXPECT_NE(tracking.find("m/s"), std::string::npos) << tracking;
  EXPECT_EQ(tracking.find("rad/s"), std::string::npos) << tracking;
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

  auto stopped = stale(config, Input::ControllerState, Staleness::StoppedArriving);
  stopped.controller_state = with_error("theta1_slewing_joint", 0.9, 0.0);

  std::vector<std::string> messages;
  for (const auto & input :
    {stale(config, Input::ControllerState, Staleness::NeverArrived), stopped,
      stale(config, Input::ControllerState, Staleness::StampAhead)})
  {
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
  EXPECT_NE(messages[1].find("stopped publishing its own state"), std::string::npos) << messages[1];
  EXPECT_NE(messages[2].find("future"), std::string::npos) << messages[2];
}

TEST(SupervisorCore, AnInnerLoopThatStoppedReportingIsNotAnInnerLoopWithNothingToReport)
{
  // §5.3 applied to the fourth input.  It is the one absence with a consequence
  // the other three do not have: an uncommissioned axis reported to nobody is
  // the state this whole stream exists to end, so a report that is not arriving
  // must not read as a loop that is saying nothing is wrong.
  const auto config = default_config();

  auto stopped = stale(config, Input::ControllerHealth, Staleness::StoppedArriving);
  stopped.controller_health = uncommissioned_gripper();

  std::vector<std::string> messages;
  for (const auto & input :
    {stale(config, Input::ControllerHealth, Staleness::NeverArrived), stopped,
      stale(config, Input::ControllerHealth, Staleness::StampAhead)})
  {
    const auto decision = crane_supervisor::decide(config, input);
    EXPECT_EQ(decision.fault, crane_supervisor::Fault::StateHealth);
    EXPECT_FALSE(decision.message.empty());
    messages.push_back(decision.message);
  }
  EXPECT_NE(messages[0], messages[1]);
  EXPECT_NE(messages[1], messages[2]);
  EXPECT_NE(messages[0], messages[2]);
  // The stale report's own code is not republished as if it were current: what
  // is reported is that nobody knows, not what the loop last said.
  EXPECT_EQ(messages[1].find("not commissioned"), std::string::npos) << messages[1];
  EXPECT_NE(messages[1].find("stopped reporting its own health"), std::string::npos)
    << messages[1];
  EXPECT_NE(messages[2].find("future"), std::string::npos) << messages[2];
}

TEST(SupervisorCore, TheInnerLoopsOwnCodesAreCarriedRatherThanTranslated)
{
  // The loop computes its verdict from the state interfaces it claims itself
  // and from the identified map of the tool it is driving, and nothing above
  // the controller manager can see either.  So the code arrives as a
  // `crane_msgs/SupervisorStatus` constant and is merged, not translated.
  const auto config = default_config();
  auto input = healthy_input();

  input.controller_health = inner_loop_fault(crane_supervisor::Fault::StateHealth);
  const auto degraded = crane_supervisor::decide(config, input);
  EXPECT_EQ(degraded.fault, crane_supervisor::Fault::StateHealth);
  EXPECT_NE(degraded.message.find("inner velocity loop"), std::string::npos) << degraded.message;

  input.controller_health = inner_loop_fault(crane_supervisor::Fault::ReferenceStale);
  const auto expired = crane_supervisor::decide(config, input);
  EXPECT_EQ(expired.fault, crane_supervisor::Fault::ReferenceStale);
  EXPECT_NE(expired.message.find("horizon"), std::string::npos) << expired.message;
  // A horizon that ran out is not a measurement that went bad, and the two are
  // separate constants for the same reason the commissioning code is its own.
  EXPECT_NE(degraded.message, expired.message);

  // A fourth code would mean the controller's own `static_assert` block and the
  // frozen message have drifted apart.  Inventing a cause here would hide the
  // drift, so the value is carried through unedited and the report says so.
  input.controller_health = inner_loop_fault(crane_supervisor::Fault::Sway);
  const auto drifted = crane_supervisor::decide(config, input);
  EXPECT_EQ(drifted.fault, crane_supervisor::Fault::Sway);
  EXPECT_NE(drifted.message.find("not supposed to be able to raise"), std::string::npos)
    << drifted.message;
}

TEST(SupervisorCore, AMissingCalibrationNamesTheAxisItIsMissingFor)
{
  // The point of carrying the flags per axis with the joint names beside them:
  // a panel that says "q9_left_rail_joint" tells an operator which calibration
  // to run, and one that says "one axis" does not.
  const auto config = default_config();
  auto input = healthy_input();
  input.controller_health = uncommissioned_gripper();

  const auto decision = crane_supervisor::decide(config, input);
  EXPECT_EQ(decision.fault, crane_supervisor::Fault::NotCommissioned);
  EXPECT_NE(decision.message.find("q9_left_rail_joint"), std::string::npos) << decision.message;
  // And it says what it is asking for, because a calibration and a sensor check
  // are the two different things this constant exists to separate
  // (wiki/implementation/commissioning_prerequisites.md §2).
  EXPECT_NE(decision.message.find("calibration"), std::string::npos) << decision.message;
  // Nothing was acted on here either.
  EXPECT_NE(decision.message.find("Nothing was stopped"), std::string::npos) << decision.message;

  // Two axes, both named: a tool whose sixth valve channel is uncalibrated and
  // an axis that lost its map is one report, not a count of two.
  input.controller_health.feedforward_free_joints = {"theta8_rotator_joint", "q9_left_rail_joint"};
  const auto both = crane_supervisor::decide(config, input);
  EXPECT_NE(both.message.find("theta8_rotator_joint"), std::string::npos) << both.message;
  EXPECT_NE(both.message.find("q9_left_rail_joint"), std::string::npos) << both.message;

  // A commissioning code with no axis behind it is a defect in the report, and
  // is reported as one rather than as a fault with nothing to act on.
  input.controller_health.feedforward_free_joints.clear();
  const auto unnamed = crane_supervisor::decide(config, input);
  EXPECT_EQ(unnamed.fault, crane_supervisor::Fault::NotCommissioned);
  EXPECT_NE(unnamed.message.find("named no axis"), std::string::npos) << unnamed.message;
}

TEST(SupervisorCore, TheHealthCodeIsReportedInPreferenceToTheCommissioningCode)
{
  // wiki/implementation/commissioning_prerequisites.md §2, which is what this
  // ordering is and not this package's preference: the missing calibration will
  // still be missing next cycle, while a state that just went stale is the one
  // an operator has to act on now.
  const auto config = default_config();

  // On its own the commissioning code is what is reported.
  auto only_commissioning = healthy_input();
  only_commissioning.controller_health = uncommissioned_gripper();
  EXPECT_EQ(
    crane_supervisor::decide(config, only_commissioning).fault,
    crane_supervisor::Fault::NotCommissioned);

  // Beside a health cause of the supervisor's own it is not.  Every one of the
  // health causes wins, whichever input raised it.
  auto degraded_state = only_commissioning;
  degraded_state.pendulum_state.valid = false;
  degraded_state.pendulum_state.status = "the upstream IMU on K5 does not report itself healthy";
  const auto against_state = crane_supervisor::decide(config, degraded_state);
  EXPECT_EQ(against_state.fault, crane_supervisor::Fault::StateHealth);
  EXPECT_NE(against_state.message.find(degraded_state.pendulum_state.status), std::string::npos)
    << against_state.message;

  auto dead_controller = only_commissioning;
  dead_controller.stream(Input::ControllerState).age = 10.0;
  EXPECT_EQ(
    crane_supervisor::decide(config, dead_controller).fault, crane_supervisor::Fault::StateHealth);

  // Including the health code the *same* loop raises: a report can carry only
  // one code, so this is the cycle where the loop found a measurement of its
  // own bad while an axis was still uncalibrated.  It reports the measurement.
  auto inner_health = only_commissioning;
  inner_health.controller_health.fault = crane_supervisor::Fault::StateHealth;
  const auto against_inner = crane_supervisor::decide(config, inner_health);
  EXPECT_EQ(against_inner.fault, crane_supervisor::Fault::StateHealth);
  EXPECT_NE(against_inner.message.find("inner velocity loop"), std::string::npos)
    << against_inner.message;

  // And a horizon that expired is a health cause in this ordering too: it is
  // the producer that is gone, and that is also this cycle's news.
  auto stale_reference = only_commissioning;
  stale_reference.controller_health.fault = crane_supervisor::Fault::ReferenceStale;
  EXPECT_EQ(
    crane_supervisor::decide(config, stale_reference).fault,
    crane_supervisor::Fault::ReferenceStale);

  // Below it sits only the interlock, and deliberately: on the `hardware`
  // profile the commissioning condition is *standing*, while a released deadman
  // is the ordinary resting state of the machine.  Nothing is lost by it --
  // `deadman_held` is a field of its own on every report and the commissioning
  // code has none.
  auto released = only_commissioning;
  released.remote_ctrl.deadman_held = false;
  const auto over_interlock = crane_supervisor::decide(config, released);
  EXPECT_EQ(over_interlock.fault, crane_supervisor::Fault::NotCommissioned);
  EXPECT_FALSE(over_interlock.deadman_held);
}

TEST(SupervisorCore, TheProfileSwitchIsTheControllersAndThisPackageAddsNoSecondOne)
{
  // wiki/implementation/commissioning_prerequisites.md §3: only the `hardware`
  // profile reports a missing calibration, and the thing that decides it is
  // `crane_velocity_controller`'s own `profile` parameter.  This package holds
  // no profile, no rig name and no second switch -- it reports the code it was
  // sent -- so the two rigs are two different reports on the wire and nothing
  // else.
  const auto config = default_config();

  // What the `hardware` profile publishes today.
  auto hardware = healthy_input();
  hardware.controller_health = uncommissioned_gripper();
  const auto reported = crane_supervisor::decide(config, hardware);
  EXPECT_EQ(reported.fault, crane_supervisor::Fault::NotCommissioned);

  // What the `fake` profile publishes for the very same machine state: the
  // gripper axis still ran PI only and the flag still says which axis, and the
  // loop still raises no commissioning fault because a rig with no hydraulics
  // has nothing to commission.
  auto fake = healthy_input();
  fake.controller_health = healthy_inner_loop();
  fake.controller_health.feedforward_free_joints = {"q9_left_rail_joint"};
  const auto quiet = crane_supervisor::decide(config, fake);
  EXPECT_EQ(quiet.fault, crane_supervisor::Fault::None);
  // And the clear report does not go looking for the axis and raise the fault
  // on its own: the flags are not a second switch either.
  EXPECT_EQ(quiet.message.find("not commissioned"), std::string::npos) << quiet.message;
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

  everything.stream(Input::PendulumState).received = false;
  EXPECT_EQ(
    crane_supervisor::decide(config, everything).fault, crane_supervisor::Fault::StateHealth);

  everything.remote_ctrl.em_stop = true;
  EXPECT_EQ(crane_supervisor::decide(config, everything).fault, crane_supervisor::Fault::EStop);
}

TEST(SupervisorCore, APassiveRatePastItsBoundIsATypedCauseThatNamesTheCoordinate)
{
  // wiki/control_architecture.md §5 row 7.  Per coordinate and on the *rate*,
  // because the published angle carries an uncalibrated constant offset that
  // nothing in this workspace has measured and the rate does not.
  const auto config = default_config();

  // One hair under the bound is not a fault; one hair over is.
  auto input = swinging(config.sway.dq_u_max[0]);
  EXPECT_EQ(crane_supervisor::decide(config, input).fault, crane_supervisor::Fault::None);

  input = healthy_input();
  input.pendulum_state.velocity = {{0.0, 0.9}};
  const auto decision = crane_supervisor::decide(config, input);
  EXPECT_EQ(decision.fault, crane_supervisor::Fault::Sway);
  // Which of the two coordinates crossed it, and which did not.
  EXPECT_NE(decision.message.find("theta7_tilt_joint"), std::string::npos) << decision.message;
  EXPECT_EQ(decision.message.find("theta6_tip_joint"), std::string::npos) << decision.message;
  EXPECT_NE(decision.message.find("0.9000"), std::string::npos) << decision.message;
  EXPECT_NE(decision.message.find("0.4000"), std::string::npos) << decision.message;
  // Nothing was acted on, on this cause least of all: refusing a motion needs an
  // authority over a mode that this supervisor does not have yet, and damping is
  // a later slice.
  EXPECT_NE(decision.message.find("Nothing was stopped"), std::string::npos) << decision.message;
  // And a rate past the fault bound is certainly not settled.
  EXPECT_EQ(decision.sway.settled, crane_supervisor::SwaySettled::NotSettled);
}

TEST(SupervisorCore, ADegradedEstimateIsStateHealthAndNeverSway)
{
  // The distinction the whole duty rests on.  A sensor that stopped saying
  // anything is a different fact from a load that is swinging, and an operator
  // does different things about them: one is a check of the graph and the other
  // is a wait.  So a degraded estimate carrying a wild rate is reported as
  // FAULT_STATE_HEALTH and the predicate goes to unknown, rather than the rate
  // being taken at face value and reported as a swing.
  const auto config = default_config();

  auto invalid = swinging(9.0);
  invalid.pendulum_state.valid = false;
  invalid.pendulum_state.status = "the upstream IMU on K5 does not report itself healthy";
  const auto degraded = crane_supervisor::decide(config, invalid);
  EXPECT_EQ(degraded.fault, crane_supervisor::Fault::StateHealth);
  EXPECT_EQ(degraded.sway.settled, crane_supervisor::SwaySettled::Unknown);
  EXPECT_NE(degraded.message.find(invalid.pendulum_state.status), std::string::npos)
    << degraded.message;

  // The same for every way the stream itself can fail to be fresh.
  for (const Staleness cause : transport_causes()) {
    auto absent = stale(config, Input::PendulumState, cause);
    absent.pendulum_state.velocity = {{9.0, 9.0}};
    const auto reported = crane_supervisor::decide(config, absent);
    EXPECT_EQ(reported.fault, crane_supervisor::Fault::StateHealth);
    EXPECT_EQ(reported.sway.settled, crane_supervisor::SwaySettled::Unknown);
  }
}

TEST(SupervisorCore, TheSwayFaultSitsBelowEveryHealthCauseAndAboveTracking)
{
  // Precedence, stated once.  Below the health causes because a supervisor whose
  // own view of the crane is degraded should say that before it says anything
  // derived from it -- and above tracking because of the two, sway is the one
  // with no field of its own: `tracking_error` is filled on every report, so a
  // tracking excess stays visible in a cycle sway owns, while a swinging load
  // reported behind a tracking fault would be invisible.
  const auto config = config_with_tolerances();

  auto both_wrong = swinging(0.9);
  both_wrong.controller_state = with_error("theta1_slewing_joint", 0.0, 1.0);
  EXPECT_EQ(crane_supervisor::decide(config, both_wrong).fault, crane_supervisor::Fault::Sway);
  // The tracking measurement is not lost by the ordering.
  EXPECT_DOUBLE_EQ(crane_supervisor::decide(config, both_wrong).tracking_error, 0.0);

  // Every health cause outranks it, whichever input raised it.
  auto inner_health = both_wrong;
  inner_health.controller_health = inner_loop_fault(crane_supervisor::Fault::StateHealth);
  EXPECT_EQ(
    crane_supervisor::decide(config, inner_health).fault, crane_supervisor::Fault::StateHealth);

  auto dead_controller = both_wrong;
  dead_controller.stream(Input::ControllerState).age = 10.0;
  EXPECT_EQ(
    crane_supervisor::decide(config, dead_controller).fault, crane_supervisor::Fault::StateHealth);

  // And the commissioning code and the interlock sit below it, because both of
  // them are conditions that will still be there next cycle.
  auto uncommissioned = both_wrong;
  uncommissioned.controller_health = uncommissioned_gripper();
  uncommissioned.remote_ctrl.deadman_held = false;
  const auto over = crane_supervisor::decide(config, uncommissioned);
  EXPECT_EQ(over.fault, crane_supervisor::Fault::Sway);
  EXPECT_FALSE(over.deadman_held);

  // The stop still outranks everything.
  auto stopped = both_wrong;
  stopped.remote_ctrl.em_stop = true;
  EXPECT_EQ(crane_supervisor::decide(config, stopped).fault, crane_supervisor::Fault::EStop);
}

TEST(SupervisorCore, TheDwellRunsThroughCyclesThatReportSomethingElseEntirely)
{
  // The dwell is judged before the precedence chain and not inside it, for the
  // reason `deadman_held` and `tracking_error` are filled before any branch
  // returns.  A dwell that only advanced on the cycles where sway was what went
  // wrong would restart every time the operator let go of the deadman -- and the
  // predicate exists precisely so that a grip can be gated on it without the
  // task layer having to keep its own timer.
  const auto config = default_config();
  auto input = healthy_input();
  input.remote_ctrl.deadman_held = false;

  // Enough cycles to cover the dwell twice over, every one of them reporting the
  // released deadman rather than anything about the sway.
  const int cycles = 2 * static_cast<int>(config.sway.settle_dwell / kCycle);
  crane_supervisor::SwaySettled settled = crane_supervisor::SwaySettled::Unknown;
  for (int cycle = 0; cycle < cycles; ++cycle) {
    const auto decision = step(config, input);
    ASSERT_EQ(decision.fault, crane_supervisor::Fault::Interlock) << cycle;
    settled = decision.sway.settled;
  }
  EXPECT_EQ(settled, crane_supervisor::SwaySettled::Settled);

  // And the crane starting to swing drops it again, in a cycle whose report is
  // still about the deadman.
  input.pendulum_state.velocity = {{0.2, 0.0}};
  const auto moving = step(config, input);
  EXPECT_EQ(moving.fault, crane_supervisor::Fault::Interlock);
  EXPECT_EQ(moving.sway.settled, crane_supervisor::SwaySettled::NotSettled);
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

namespace
{

/// Every shape the fourth input's payload can arrive in.
std::vector<crane_supervisor::ControllerHealthReport> every_inner_loop_report()
{
  std::vector<crane_supervisor::ControllerHealthReport> reports{
    healthy_inner_loop(),
    inner_loop_fault(crane_supervisor::Fault::StateHealth),
    inner_loop_fault(crane_supervisor::Fault::ReferenceStale),
    uncommissioned_gripper()};
  return reports;
}

/// The whole reachable input space of this slice, one struct per combination.
/**
 * Swept rather than enumerated by hand, because the cases that go wrong are the
 * ones nobody thought to name. The transport half is swept over the registry --
 * every input arriving or not, at every age -- so an input added to `Input` is
 * swept the day it is added rather than the day somebody remembers it.
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
                      for (const auto & inner_loop : every_inner_loop_report()) {
                        crane_supervisor::SupervisorInput input;
                        input.sampled_at = kFirstCycle;
                        for (std::size_t i = 0; i < kInputCount; ++i) {
                          input.streams[i].received = received;
                          input.streams[i].age = age;
                        }
                        input.stream(Input::RemoteCtrl).received = remote_received;
                        input.stream(Input::ControllerState).received = controller_received;
                        input.pendulum_state.valid = valid;
                        input.pendulum_state.status = status;
                        input.remote_ctrl.em_stop = em_stop;
                        input.remote_ctrl.deadman_held = deadman;
                        input.estop_latched = latched;
                        input.controller_state = tracking_controller();
                        for (auto & axis : input.controller_state.axes) {
                          axis.velocity_error = velocity_error;
                          axis.position_error = velocity_error;
                        }
                        input.controller_health = inner_loop;
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
  }

  // The passive rate, applied over the whole set rather than as a twelfth nested
  // loop. Three values and not two: still, between the settle bound and the
  // fault bound, and past the fault bound -- which is the reachable space of the
  // sway duty in a single cycle. The dwell needs a sequence and is swept in
  // `test_sway_monitor.cpp`, where the clock is an argument.
  std::vector<crane_supervisor::SupervisorInput> swept;
  swept.reserve(inputs.size() * 3);
  for (const double dq_u : {0.0, 0.1, 9.0}) {
    for (crane_supervisor::SupervisorInput input : inputs) {
      input.pendulum_state.velocity = {{dq_u, dq_u}};
      swept.push_back(std::move(input));
    }
  }
  return swept;
}

}  // namespace

TEST(SupervisorCore, EveryReachableDecisionCarriesACause)
{
  // PRD user story 53, asserted over the whole reachable input space rather
  // than over the cases the tests above happen to name: a report with a fault
  // and no words is the inferred abort §5.0 removes, back again.
  for (const auto & config : {default_config(), config_with_tolerances()}) {
    for (const auto & input : every_input()) {
      const auto decision = crane_supervisor::decide(config, input);
      EXPECT_FALSE(decision.message.empty());
      // The mode is re-derived from the controller manager's own answer on
      // every cycle, so what is asserted here is that `decide()` reports what
      // `active_mode()` read rather than anything of its own. None of these
      // inputs carries an answer, which is the state a supervisor with no
      // manager on the graph is in: the mode is not known and MODE_IDLE -- the
      // reading that claims the least -- is what goes on the wire.
      const auto active = crane_supervisor::active_mode(config, input.controller_manager);
      EXPECT_EQ(decision.mode, active.mode);
      EXPECT_EQ(decision.mode, crane_supervisor::Mode::Idle);
      EXPECT_FALSE(active.known);
      // And the clause that says so is on every one of them, whatever the fault
      // is: MODE_IDLE on a report whose manager is silent and MODE_IDLE on one
      // whose arm claim is free are the same byte and different facts.
      EXPECT_NE(decision.message.find(crane_supervisor::kModeClausePrefix), std::string::npos)
        << decision.message;
      // The working cell is still not computed in this slice, and carries the
      // value that claims nothing rather than a stub that claims something.
      EXPECT_FALSE(decision.inside_working_cell);
      // The tracking error is a measurement of what arrived, and it is zero in
      // exactly the case where nothing usable did.
      if (decision.tracking_error != 0.0) {
        EXPECT_TRUE(input.stream(Input::ControllerState).received);
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

TEST(SupervisorCore, NoInputThatIsNotFreshIsEverReportedAsFaultFree)
{
  // §5.3's rule, over the whole reachable input space: an input outside its own
  // deadline never ends in FAULT_NONE, whichever input it is and however healthy
  // everything else is.  This is the assertion that would fail first if a fifth
  // subscription were added and left out of the precedence chain.
  for (const auto & config : {default_config(), config_with_tolerances()}) {
    for (const auto & input : every_input()) {
      const auto causes = crane_supervisor::freshness(config, input);
      const bool any_stale = std::any_of(
        causes.begin(), causes.end(), [](Staleness cause) {return cause != Staleness::Fresh;});
      if (any_stale) {
        EXPECT_NE(crane_supervisor::decide(config, input).fault, crane_supervisor::Fault::None);
      }
    }
  }
}

TEST(SupervisorCore, TheSettledPredicateIsOnEveryDecisionAndIsNeverSettledWhileUnknowable)
{
  // Three states and not two, over the whole reachable input space: an estimate
  // that is absent, stale or marked unusable makes "settled" unanswerable, and a
  // grip action gated on a two-valued predicate would descend onto a swinging
  // block the moment the bracketing IMU stopped answering.
  for (const auto & config : {default_config(), config_with_tolerances()}) {
    for (const auto & input : every_input()) {
      const auto decision = crane_supervisor::decide(config, input);
      const auto causes = crane_supervisor::freshness(config, input);
      const bool trusted = causes[index_of(Input::PendulumState)] == Staleness::Fresh &&
        input.pendulum_state.valid;
      if (trusted) {
        EXPECT_NE(decision.sway.settled, crane_supervisor::SwaySettled::Unknown);
      } else {
        EXPECT_EQ(decision.sway.settled, crane_supervisor::SwaySettled::Unknown);
      }
      // And it is said out loud on every report, whatever `fault` is: the
      // predicate has no field of its own, so a cycle that dropped the clause
      // would drop it exactly when a more consequential cause was in the way.
      EXPECT_NE(decision.message.find(crane_supervisor::kSettledClausePrefix), std::string::npos)
        << decision.message;
    }
  }
}

TEST(SupervisorCore, NoTrackingFaultIsReachableWithoutATolerance)
{
  // With no number to compare against, FAULT_TRACKING is not reachable at all --
  // however far out any axis is.
  const auto config = default_config();
  for (const auto & input : every_input()) {
    EXPECT_NE(crane_supervisor::decide(config, input).fault, crane_supervisor::Fault::Tracking);
  }
}

TEST(SupervisorCore, TheFaultsThisSliceRaisesAreItsOwnFiveAndTheInnerLoopsThree)
{
  // Every other cause of the §5 table belongs to a later issue.  A supervisor
  // that raised one of them from an input it does not have would be reporting a
  // check it never made.
  //
  // Two of the eight are not this package's verdicts at all: FAULT_REFERENCE_STALE
  // and FAULT_NOT_COMMISSIONED are the inner velocity loop's, merged as the loop
  // numbered them.  Working cell and solver stay unreachable -- nothing on any of
  // the four inputs can produce them, and neither can this supervisor.  FAULT_SWAY
  // is reachable now, and from exactly one place: the passive rate past its own
  // configured bound.
  for (const auto & config : {default_config(), config_with_tolerances()}) {
    for (const auto & input : every_input()) {
      const auto fault = crane_supervisor::decide(config, input).fault;
      EXPECT_TRUE(
        fault == crane_supervisor::Fault::None ||
        fault == crane_supervisor::Fault::StateHealth ||
        fault == crane_supervisor::Fault::Tracking ||
        fault == crane_supervisor::Fault::Sway ||
        fault == crane_supervisor::Fault::ReferenceStale ||
        fault == crane_supervisor::Fault::EStop ||
        fault == crane_supervisor::Fault::Interlock ||
        fault == crane_supervisor::Fault::NotCommissioned)
        << static_cast<int>(fault);
      // And the two that are not this package's are reported only when the loop
      // sent them: neither is derivable from anything else this supervisor holds.
      if (fault == crane_supervisor::Fault::ReferenceStale ||
        fault == crane_supervisor::Fault::NotCommissioned)
      {
        EXPECT_EQ(fault, input.controller_health.fault);
        EXPECT_TRUE(input.stream(Input::ControllerHealth).received);
      }
      // And FAULT_SWAY is reachable only from a rate this supervisor was
      // entitled to believe.  A degraded estimate is FAULT_STATE_HEALTH, never
      // this: a sensor that stopped saying anything is a different fact from a
      // load that is swinging.
      if (fault == crane_supervisor::Fault::Sway) {
        EXPECT_TRUE(input.stream(Input::PendulumState).received);
        EXPECT_TRUE(input.pendulum_state.valid);
      }
    }
  }
}

// --------------------------------------------------------------------------
// The mode, and the one authority this supervisor has.  Everything below is
// offline: `arbitrate_mode()` decides and returns a plan, and nothing in this
// binary can switch anything.  What a real switch does against a real
// controller manager is `test_mode_switch.cpp`.
// --------------------------------------------------------------------------

namespace
{

using crane_supervisor::ActiveMode;
using crane_supervisor::ControllerManagerReport;
using crane_supervisor::ControllerReport;
using crane_supervisor::Mode;

constexpr char kVelocityController[] = "crane_velocity_controller";
constexpr char kFollower[] = "trajectory_controller_a2b";
constexpr char kManualController[] = "manual_velocity_controller";
constexpr char kToolController[] = "tool_velocity_controller";

/// The shipped configuration with a manual controller and a tool claim added.
/**
 * Today's profiles have neither -- no CBS profile composes a manual controller,
 * and `crane_velocity_controller` holds all six `velocity` interfaces including
 * the gripper axis, so the deployment has one claim.  A test of the arbitration
 * has to have both, because what is being asserted is that the *model* carries
 * a second claim and a mode it does not implement, which is exactly what a
 * deployment that only ever had one would never exercise
 * (wiki/control_architecture.md §7.3).
 */
crane_supervisor::SupervisorConfig config_with_modes()
{
  crane_supervisor::SupervisorConfig config = default_config();
  config.mode_controllers[crane_supervisor::index_of(Mode::Manual)] = {kManualController};
  config.tool_controllers = {kToolController};
  return config;
}

/// One row of what the controller manager would have answered.
ControllerReport controller(
  const std::string & name, const std::string & state,
  const std::vector<std::string> & claimed = {})
{
  ControllerReport report;
  report.name = name;
  report.state = state;
  report.claimed_interfaces = claimed;
  return report;
}

/// An answer that came back just now, listing `controllers`.
ControllerManagerReport answered(std::vector<ControllerReport> controllers)
{
  ControllerManagerReport report;
  report.answer.received = true;
  report.answer.age = 0.01;
  report.controllers = std::move(controllers);
  return report;
}

/// Everything loaded and inactive: the state after a manager comes up with the
/// controllers configured and nothing spawned.
ControllerManagerReport all_inactive()
{
  return answered(
    {controller(kVelocityController, "inactive"), controller(kFollower, "inactive"),
      controller(kManualController, "inactive"), controller(kToolController, "inactive")});
}

/// The same, with the FOLLOW pair active and holding the arm claim.
ControllerManagerReport following()
{
  return answered(
    {controller(kVelocityController, "active", {"theta1_slewing_joint/velocity"}),
      controller(kFollower, "active", {"crane_velocity_controller/theta1_slewing_joint/velocity"}),
      controller(kManualController, "inactive"), controller(kToolController, "inactive")});
}

/// The state after a switch into MODE_MPC: the inner loop alone holds the claim.
ControllerManagerReport unchained()
{
  return answered(
    {controller(kVelocityController, "active", {"theta1_slewing_joint/velocity"}),
      controller(kFollower, "inactive"), controller(kManualController, "inactive"),
      controller(kToolController, "inactive")});
}

/// What the horizon producer would have reported, `age` seconds ago.
/**
 * The evidence PRD §10 step 2 is verified against, and it is
 * `crane_msgs/SolverHealth` rather than the horizon itself for a reason the
 * core's `HorizonReport` states: in shadow -- the state every switch into
 * `MODE_MPC` is made from -- `crane_mpc` publishes nothing at all on
 * `/crane/mpc/horizon`, so a check against the horizon could never pass.
 */
crane_supervisor::HorizonReport solving(
  double age = 0.01,
  crane_supervisor::SolveOutcome outcome = crane_supervisor::SolveOutcome::Converged)
{
  crane_supervisor::HorizonReport report;
  report.health.received = true;
  report.health.age = age;
  report.outcome = outcome;
  report.solve_time = 0.012;
  report.solve_budget = 0.03;
  report.status = "solved in 12 ms of a 30 ms budget";
  return report;
}

/// A healthy observation with the controller manager answering.
/**
 * The horizon producer is left as it is on a stack that never composed one:
 * nothing has arrived, which is what makes a `MODE_MPC` request refused unless
 * a test says otherwise. `warm()` is how a test says otherwise.
 */
crane_supervisor::SupervisorInput input_with(ControllerManagerReport manager)
{
  crane_supervisor::SupervisorInput input = healthy_input();
  input.controller_manager = std::move(manager);
  return input;
}

/// The same observation with the horizon producer solving inside its deadline.
crane_supervisor::SupervisorInput warm(
  crane_supervisor::SupervisorInput input, crane_supervisor::HorizonReport horizon = solving())
{
  input.horizon = std::move(horizon);
  return input;
}

/// One request, arbitrated against a healthy machine in a named state.
crane_supervisor::ModeArbitration ask(
  const crane_supervisor::SupervisorConfig & config, Mode mode, ControllerManagerReport manager)
{
  return crane_supervisor::arbitrate_mode(
    config, static_cast<std::uint8_t>(mode), input_with(std::move(manager)));
}

}  // namespace

TEST(SupervisorMode, TheActiveModeIsReadOffTheManagerAndNotRemembered)
{
  const auto config = config_with_modes();

  // Nothing active: the arm claim is free, and that is MODE_IDLE as an
  // observation rather than as a default.
  const ActiveMode idle = crane_supervisor::active_mode(config, all_inactive());
  EXPECT_TRUE(idle.known);
  EXPECT_EQ(idle.mode, Mode::Idle);
  EXPECT_FALSE(idle.partial);
  EXPECT_TRUE(idle.active_controllers.empty());

  // Both of FOLLOW's controllers active: MODE_FOLLOW, and the report names them.
  const ActiveMode follow = crane_supervisor::active_mode(config, following());
  EXPECT_TRUE(follow.known);
  EXPECT_EQ(follow.mode, Mode::Follow);
  EXPECT_FALSE(follow.partial);
  EXPECT_EQ(
    follow.active_controllers, (std::vector<std::string>{kVelocityController, kFollower}));

  // The inner loop alone: **MODE_MPC**, and not a half-state of MODE_FOLLOW.
  // That is the whole of what slice 6 changed about this function.  PRD §10
  // step 3 puts the same `crane_velocity_controller` instance on both paths, so
  // MODE_MPC's claim *is* MODE_FOLLOW's minus the trajectory controller, and
  // matching by containment would have called this drift while calling
  // MODE_FOLLOW ambiguous.  Matching by set equality answers both exactly.
  ControllerManagerReport unchained = following();
  unchained.controllers[1].state = "inactive";
  const ActiveMode mpc = crane_supervisor::active_mode(config, unchained);
  EXPECT_TRUE(mpc.known);
  EXPECT_FALSE(mpc.partial);
  EXPECT_EQ(mpc.mode, Mode::Mpc);
  EXPECT_EQ(mpc.active_controllers, (std::vector<std::string>{kVelocityController}));

  // And MODE_FOLLOW is still answered exactly rather than being satisfied by
  // MODE_MPC's list as well: one mode matches, not two.
  EXPECT_EQ(crane_supervisor::active_mode(config, following()).mode, Mode::Follow);

  // The trajectory controller alone, with the inner loop it chains onto down:
  // *that* is a half-state, and a half-state is not a mode.  MODE_IDLE is what
  // claims the least, and the clause says which controller is up rather than
  // rounding the drift off into a mode nobody can act on.
  ControllerManagerReport half = following();
  half.controllers[0].state = "inactive";
  const ActiveMode drifted = crane_supervisor::active_mode(config, half);
  EXPECT_TRUE(drifted.known);
  EXPECT_TRUE(drifted.partial);
  EXPECT_EQ(drifted.mode, Mode::Idle);
  const std::string clause = crane_supervisor::mode_clause(config, drifted);
  EXPECT_NE(clause.find(kFollower), std::string::npos) << clause;
}

TEST(SupervisorMode, AManagerThatIsNotAnsweringLeavesTheModeNotKnown)
{
  const auto config = config_with_modes();

  // Never answered.  MODE_IDLE goes on the wire because no motion mode can be
  // confirmed, and the clause says outright that this is not an observation
  // that nothing is running.
  const ActiveMode silent = crane_supervisor::active_mode(config, {});
  EXPECT_FALSE(silent.known);
  EXPECT_EQ(silent.mode, Mode::Idle);
  EXPECT_EQ(silent.cause, Staleness::NeverArrived);
  EXPECT_NE(
    crane_supervisor::mode_clause(config, silent).find("not known"), std::string::npos);

  // Answered once and then stopped.  A remembered mode would stand on the wire
  // for ever; this ages out, and the two absences do not read the same.
  ControllerManagerReport stale = following();
  stale.answer.age = 10.0 * config.controller_manager_deadline;
  const ActiveMode aged = crane_supervisor::active_mode(config, stale);
  EXPECT_FALSE(aged.known);
  EXPECT_EQ(aged.mode, Mode::Idle);
  EXPECT_EQ(aged.cause, Staleness::StoppedArriving);
  EXPECT_NE(
    crane_supervisor::mode_clause(config, silent), crane_supervisor::mode_clause(config, aged));
}

TEST(SupervisorMode, TheStatusModeIsWhatTheManagerSaysAndNothingElse)
{
  // End of the path the acceptance criterion is about: `decide()` reports what
  // was read, and a clear report is not a claim about the mode.
  const auto config = config_with_modes();
  auto input = input_with(following());
  const auto decision = crane_supervisor::decide(config, input);
  EXPECT_EQ(decision.fault, crane_supervisor::Fault::None);
  EXPECT_EQ(decision.mode, Mode::Follow);
  EXPECT_NE(decision.message.find("MODE_FOLLOW"), std::string::npos) << decision.message;
  EXPECT_NE(decision.message.find(kVelocityController), std::string::npos) << decision.message;
}

TEST(SupervisorMode, AValueThatIsNotAModeIsRefusedRatherThanCast)
{
  const auto config = config_with_modes();
  const auto refused = crane_supervisor::arbitrate_mode(config, 200, input_with(all_inactive()));
  EXPECT_FALSE(refused.accepted);
  EXPECT_FALSE(refused.switch_required);
  EXPECT_TRUE(refused.activate.empty());
  EXPECT_TRUE(refused.deactivate.empty());
  EXPECT_NE(refused.message.find("MODE_MPC"), std::string::npos) << refused.message;

  // And the four that are modes are not refused for being unreadable.
  for (std::uint8_t value = 0; value < 4; ++value) {
    EXPECT_TRUE(crane_supervisor::is_mode(value)) << static_cast<int>(value);
  }
  EXPECT_FALSE(crane_supervisor::is_mode(4));
}

TEST(SupervisorMode, AStaleHorizonRefusesMpcAndTheTrajectoryControllerIsNotPlannedAway)
{
  // PRD §10 step 2 and user story 35.  The freshness of the horizon is verified
  // *before* the trajectory controller is deactivated -- no freshness, no
  // switch -- and what makes that an assertion rather than a claim is that the
  // plan comes back **empty**: nothing was deactivated, because nothing was
  // planned to be.
  const auto config = config_with_modes();

  // Nothing has ever arrived, which is also what a deployment that composes no
  // producer at all looks like from here.
  const auto absent = ask(config, Mode::Mpc, following());
  EXPECT_FALSE(absent.accepted);
  EXPECT_TRUE(absent.deactivate.empty()) << "the trajectory controller was planned away";
  EXPECT_TRUE(absent.activate.empty());
  EXPECT_NE(absent.message.find("freshness"), std::string::npos) << absent.message;
  EXPECT_NE(absent.message.find("Nothing was deactivated"), std::string::npos) << absent.message;
  // The machine is reported in the mode it is still in, not in the one that was
  // asked for (user story 36).
  EXPECT_EQ(absent.active.mode, Mode::Follow);

  // It was arriving and stopped, which is a different thing to chase and reads
  // as one: the age and the deadline are both in the sentence.
  crane_supervisor::HorizonReport late = solving(10.0 * config.horizon_deadline);
  const auto stale = crane_supervisor::arbitrate_mode(
    config, static_cast<std::uint8_t>(Mode::Mpc), warm(input_with(following()), late));
  EXPECT_FALSE(stale.accepted);
  EXPECT_TRUE(stale.deactivate.empty());
  EXPECT_NE(stale.message.find("3.000 s old"), std::string::npos) << stale.message;
  EXPECT_NE(stale.message.find("0.300 s"), std::string::npos) << stale.message;
  EXPECT_NE(stale.message, absent.message);

  // A stamp further in this node's future than the margin allows is not a fresh
  // sample either, and it names the clock rather than the optimizer.
  crane_supervisor::HorizonReport ahead = solving(-10.0 * config.horizon_deadline);
  const auto skewed = crane_supervisor::arbitrate_mode(
    config, static_cast<std::uint8_t>(Mode::Mpc), warm(input_with(following()), ahead));
  EXPECT_FALSE(skewed.accepted);
  EXPECT_TRUE(skewed.deactivate.empty());
  EXPECT_NE(skewed.message.find("future"), std::string::npos) << skewed.message;
}

TEST(SupervisorMode, AProducerThatIsAliveAndNotConvergingIsTheDeadMpcStepTwoRefuses)
{
  // The half freshness alone cannot see.  An optimizer that publishes at rate
  // and fails every solve reads as fresh, and it is exactly the dead MPC user
  // story 35 refuses to switch into: PRD §10 step 1 wants it *warm*, and shadow
  // mode already implies it solves.
  const auto config = config_with_modes();

  for (const crane_supervisor::SolveOutcome outcome :
    {crane_supervisor::SolveOutcome::Unknown, crane_supervisor::SolveOutcome::BudgetExceeded,
      crane_supervisor::SolveOutcome::Failed})
  {
    const crane_supervisor::HorizonReport report = solving(0.01, outcome);
    EXPECT_EQ(
      crane_supervisor::horizon_precondition(config, report).cause, Staleness::Fresh)
      << "the stream is fine; it is the solve that is not";
    EXPECT_FALSE(crane_supervisor::horizon_precondition(config, report).fresh);

    const auto refused = crane_supervisor::arbitrate_mode(
      config, static_cast<std::uint8_t>(Mode::Mpc), warm(input_with(following()), report));
    EXPECT_FALSE(refused.accepted);
    EXPECT_TRUE(refused.deactivate.empty());
    EXPECT_NE(
      refused.message.find(crane_supervisor::solve_outcome_name(outcome)), std::string::npos)
      << refused.message;
    // The producer's own account of the cycle is carried rather than restated,
    // the way the broadcaster's status string is.
    EXPECT_NE(refused.message.find("30 ms budget"), std::string::npos) << refused.message;
  }

  // And the converged one is the only one that passes.
  EXPECT_TRUE(crane_supervisor::horizon_precondition(config, solving()).fresh);
}

TEST(SupervisorMode, TheFreshnessCheckIsAFunctionOfItsOwnAndItAnswersOnlyAboutMpc)
{
  // The order is the assertion, and it is what makes the node able to run this
  // check *before it asks the controller manager for anything at all*.  A
  // request for any other mode has to come back empty from it, or the node's
  // early exit would refuse switches that have nothing to do with the horizon.
  const auto config = config_with_modes();
  const crane_supervisor::SupervisorInput cold = input_with(following());

  for (const Mode mode : {Mode::Idle, Mode::Manual, Mode::Follow}) {
    EXPECT_TRUE(
      crane_supervisor::mpc_horizon_refusal(config, static_cast<std::uint8_t>(mode), cold).empty())
      << crane_supervisor::mode_name(mode);
  }
  // A value that is not a mode is not this check's to refuse either:
  // `arbitrate_mode()` answers that one and says which values exist.
  EXPECT_TRUE(crane_supervisor::mpc_horizon_refusal(config, 200, cold).empty());

  // MODE_MPC with nothing arriving is the one case it answers, and the sentence
  // it returns is the one the arbitration puts on the response -- one function,
  // two call sites, so the early answer and the arbitration cannot disagree.
  const std::string refusal =
    crane_supervisor::mpc_horizon_refusal(config, static_cast<std::uint8_t>(Mode::Mpc), cold);
  EXPECT_FALSE(refusal.empty());
  EXPECT_EQ(refusal, ask(config, Mode::Mpc, following()).message);

  // And it is checked *above* the view of the controller manager: a supervisor
  // that cannot see the manager still refuses MODE_MPC for the horizon, which
  // is the reason that is actually true and the one an operator can act on.
  crane_supervisor::SupervisorInput blind = healthy_input();
  EXPECT_EQ(
    crane_supervisor::arbitrate_mode(config, static_cast<std::uint8_t>(Mode::Mpc), blind).message,
    refusal);
}

TEST(SupervisorMode, AWarmHorizonAdmitsMpcAndThePlanNeverNamesTheInnerLoop)
{
  // PRD §10 step 3 as a plan: "Same controller instance across both paths, so
  // the handover is an ordinary seam, not new machinery."  The switch out of
  // MODE_FOLLOW names the trajectory controller and **nothing else** -- a plan
  // that named the inner loop would be asking for the sole claimant of the six
  // velocity command interfaces to be released and re-claimed, which would
  // destroy the very thing step 3 clamps the first B-spline to.
  const auto config = config_with_modes();
  const auto accepted = crane_supervisor::arbitrate_mode(
    config, static_cast<std::uint8_t>(Mode::Mpc), warm(input_with(following())));
  ASSERT_TRUE(accepted.accepted) << accepted.message;
  EXPECT_TRUE(accepted.switch_required);
  EXPECT_EQ(accepted.deactivate, (std::vector<std::string>{kFollower}));
  EXPECT_TRUE(accepted.activate.empty()) << "the inner loop was planned to be reactivated";
  // The other half of the switch, and this supervisor's alone.
  EXPECT_TRUE(accepted.horizon_producer_active);

  // And back.  Only the trajectory controller is activated, nothing is
  // deactivated at all, and the producer goes back to shadow.
  const auto back = crane_supervisor::arbitrate_mode(
    config, static_cast<std::uint8_t>(Mode::Follow), warm(input_with(unchained())));
  ASSERT_TRUE(back.accepted) << back.message;
  EXPECT_EQ(back.activate, (std::vector<std::string>{kFollower}));
  EXPECT_TRUE(back.deactivate.empty()) << "the inner loop was planned away across the handover";
  EXPECT_FALSE(back.horizon_producer_active);

  // Every other mode leaves the producer in shadow, so the flag is the mode and
  // not a second decision.
  for (const Mode mode : {Mode::Idle, Mode::Manual}) {
    EXPECT_FALSE(
      crane_supervisor::arbitrate_mode(
        config, static_cast<std::uint8_t>(mode), warm(input_with(following())))
      .horizon_producer_active)
      << crane_supervisor::mode_name(mode);
  }
}

TEST(SupervisorMode, TheHorizonProducerHasADeadlineAndItIsNotAnyOfTheInputDeadlines)
{
  // The producer's stream is deliberately not an `Input` -- an optimizer that is
  // quiet while the machine is in MODE_FOLLOW is the ordinary state of this
  // stack, and ROS 2 Interfaces §4 makes merging FAULT_SOLVER onto the status a
  // slice of its own.  §5.3's rule is met the way the polled controller-manager
  // view meets it: its own margin, refused when absent, and a defined
  // consequence at the point it matters.
  std::string reason;
  auto undated = config_with_modes();
  undated.horizon_deadline = 0.0;
  EXPECT_FALSE(crane_supervisor::validate(undated, reason));
  EXPECT_NE(reason.find("horizon"), std::string::npos) << reason;

  // And a producer that stopped raises no fault on the status stream at all:
  // the decision is unchanged by it, which is the property the registry's
  // absence is there to give.
  const auto config = config_with_modes();
  crane_supervisor::SupervisorInput quiet = input_with(following());
  const auto without = crane_supervisor::decide(config, quiet);
  const auto with = crane_supervisor::decide(config, warm(quiet));
  EXPECT_EQ(without.fault, with.fault);
  EXPECT_EQ(without.message, with.message);
}

TEST(SupervisorMode, NoViewOfTheManagerRefusesEverySwitchRatherThanSwitchingBlind)
{
  const auto config = config_with_modes();
  crane_supervisor::SupervisorInput input = healthy_input();
  const auto refused =
    crane_supervisor::arbitrate_mode(config, static_cast<std::uint8_t>(Mode::Follow), input);
  EXPECT_FALSE(refused.accepted);
  EXPECT_TRUE(refused.activate.empty());
  EXPECT_TRUE(refused.deactivate.empty());
  EXPECT_NE(refused.message.find("cannot see the controller manager"), std::string::npos)
    << refused.message;
}

TEST(SupervisorMode, ALatchedFaultRefusesAMotionModeAndCarriesTheLatchedCause)
{
  const auto config = config_with_modes();
  crane_supervisor::SupervisorInput input = input_with(all_inactive());
  input.estop_latched = true;

  const auto refused =
    crane_supervisor::arbitrate_mode(config, static_cast<std::uint8_t>(Mode::Follow), input);
  EXPECT_FALSE(refused.accepted);
  EXPECT_TRUE(refused.deactivate.empty());
  // The latched cause is the reason, carried rather than restated, and it names
  // where the acknowledgement goes.
  EXPECT_NE(refused.message.find("emergency stop"), std::string::npos) << refused.message;
  EXPECT_NE(refused.message.find("/crane/clear_fault"), std::string::npos) << refused.message;

  // Absence of the stop signal is asserted, so it latches the same way and
  // refuses the same switch (§6.1).
  crane_supervisor::SupervisorInput absent = input_with(all_inactive());
  absent.stream(Input::RemoteCtrl).received = false;
  EXPECT_FALSE(
    crane_supervisor::arbitrate_mode(config, static_cast<std::uint8_t>(Mode::Manual), absent)
    .accepted);

  // MODE_IDLE is not refused by it: releasing the claim is the one direction a
  // latched stop does not argue against.
  crane_supervisor::SupervisorInput latched_and_following = input_with(following());
  latched_and_following.estop_latched = true;
  const auto to_idle = crane_supervisor::arbitrate_mode(
    config, static_cast<std::uint8_t>(Mode::Idle), latched_and_following);
  EXPECT_TRUE(to_idle.accepted) << to_idle.message;
  EXPECT_TRUE(to_idle.switch_required);
}

TEST(SupervisorMode, EveryPreconditionIsCheckedBeforeAnythingIsPlannedAwayFromTheActiveMode)
{
  // PRD §10 step 2 and user story 35, as a property of the plan rather than of
  // the timing: on every refusal there is nothing to deactivate, so a switch
  // that cannot succeed leaves the machine in the mode it is already in and is
  // never discovered half-way.
  const auto config = config_with_modes();

  // A mode this deployment configures no controller for.
  auto empty_manual = config;
  empty_manual.mode_controllers[crane_supervisor::index_of(Mode::Manual)].clear();
  const auto unconfigured = ask(empty_manual, Mode::Manual, following());
  EXPECT_FALSE(unconfigured.accepted);
  EXPECT_TRUE(unconfigured.deactivate.empty());
  EXPECT_NE(unconfigured.message.find("no controller"), std::string::npos)
    << unconfigured.message;

  // A controller the manager has never heard of.
  ControllerManagerReport without_manual = following();
  without_manual.controllers.erase(without_manual.controllers.begin() + 2);
  const auto missing = ask(config, Mode::Manual, without_manual);
  EXPECT_FALSE(missing.accepted);
  EXPECT_TRUE(missing.deactivate.empty()) << "the FOLLOW pair was planned away for a switch "
                                             "that could never have completed";
  EXPECT_NE(missing.message.find("not loaded"), std::string::npos) << missing.message;

  // A controller that is loaded and will not configure.
  ControllerManagerReport unconfigurable = following();
  unconfigurable.controllers[2].state = "unconfigured";
  const auto stuck = ask(config, Mode::Manual, unconfigurable);
  EXPECT_FALSE(stuck.accepted);
  EXPECT_TRUE(stuck.deactivate.empty());
  EXPECT_NE(stuck.message.find("unconfigured"), std::string::npos) << stuck.message;

  // Every one of them says what was *not* done, so an operator is never left
  // with an unexplained no-op (PRD user story 36).
  for (const auto & refusal : {unconfigured, missing, stuck}) {
    EXPECT_FALSE(refusal.message.empty());
    EXPECT_NE(refusal.message.find("Nothing was deactivated"), std::string::npos)
      << refusal.message;
    EXPECT_EQ(refusal.active.mode, Mode::Follow);
  }
}

TEST(SupervisorMode, TheModesAreMutuallyExclusiveAndTheSwitchIsOneCall)
{
  // §7: manual and autonomous already exclude each other by resource claim, and
  // what this adds is the arbitrated decision in front of it.  The plan carries
  // the deactivation of the outgoing mode and the activation of the incoming
  // one together, so the machine passes from one to the other inside one of the
  // manager's cycles rather than through a state that is neither.
  const auto config = config_with_modes();
  const auto to_manual = ask(config, Mode::Manual, following());
  ASSERT_TRUE(to_manual.accepted) << to_manual.message;
  EXPECT_TRUE(to_manual.switch_required);
  EXPECT_EQ(to_manual.activate, (std::vector<std::string>{kManualController}));
  EXPECT_EQ(to_manual.deactivate, (std::vector<std::string>{kVelocityController, kFollower}));

  // The order of the activation is the deployment's own and is not sorted:
  // PRD §5's cascade brings the inner loop up before anything chains onto its
  // reference interfaces.
  const auto to_follow = ask(config, Mode::Follow, all_inactive());
  ASSERT_TRUE(to_follow.accepted) << to_follow.message;
  EXPECT_EQ(to_follow.activate, (std::vector<std::string>{kVelocityController, kFollower}));
  EXPECT_TRUE(to_follow.deactivate.empty());
}

TEST(SupervisorMode, TheToolClaimIsNeverSwitchedByAModeRequest)
{
  // §7.3: with the block gripper the tool axis is on its own controller and can
  // move while the arm controller is inactive -- a second, independent claim on
  // the same pump.  A mode model that assumed one claim per machine would
  // deactivate it here, silently, on every arm mode change.
  const auto config = config_with_modes();
  ControllerManagerReport with_tool = following();
  with_tool.controllers[3].state = "active";
  with_tool.controllers[3].claimed_interfaces = {"q9_left_rail_joint/velocity"};

  const auto to_manual = ask(config, Mode::Manual, with_tool);
  ASSERT_TRUE(to_manual.accepted) << to_manual.message;
  EXPECT_EQ(
    std::find(to_manual.deactivate.begin(), to_manual.deactivate.end(), kToolController),
    to_manual.deactivate.end())
    << "an arm mode change deactivated the tool claim";
  EXPECT_EQ(
    std::find(to_manual.activate.begin(), to_manual.activate.end(), kToolController),
    to_manual.activate.end());

  // Releasing the arm claim does not release it either: MODE_IDLE is a mode of
  // the arm and not of the machine.
  const auto to_idle = ask(config, Mode::Idle, with_tool);
  ASSERT_TRUE(to_idle.accepted) << to_idle.message;
  EXPECT_EQ(to_idle.deactivate, (std::vector<std::string>{kVelocityController, kFollower}));

  // And it is reported, so a caller can see that a gripper action and an arm
  // motion are two claims rather than one machine.
  const ActiveMode active = crane_supervisor::active_mode(config, with_tool);
  EXPECT_EQ(active.active_tool_controllers, (std::vector<std::string>{kToolController}));
  const std::string clause = crane_supervisor::mode_clause(config, active);
  EXPECT_NE(clause.find("Tool claim"), std::string::npos) << clause;
  EXPECT_NE(clause.find(kToolController), std::string::npos) << clause;
}

TEST(SupervisorMode, ADeploymentWithNoToolClaimSaysSoRatherThanImplyingOneClaim)
{
  // The shipped state: `crane_velocity_controller` holds all six `velocity`
  // interfaces including `q9_left_rail_joint`, so this composition has one
  // claim.  The clause reports the absence rather than staying quiet, because a
  // report that mentioned the tool only when it was held would be a
  // one-claim-per-machine model with an exception in it.
  const auto config = default_config();
  const std::string clause =
    crane_supervisor::mode_clause(config, crane_supervisor::active_mode(config, following()));
  EXPECT_NE(clause.find("No tool claim is configured"), std::string::npos) << clause;
  EXPECT_NE(clause.find("7.3"), std::string::npos) << clause;
}

TEST(SupervisorMode, AClaimTheConfigurationDoesNotDescribeIsReportedRatherThanIgnored)
{
  // The other half of "not one claim per machine": something active that owns a
  // command interface and belongs to no configured mode and to no tool claim.
  // It is reported and not refused on -- a broadcaster owns none and would be a
  // false alarm -- and it can still block a switch by holding an interface the
  // incoming controller needs.
  const auto config = config_with_modes();
  ControllerManagerReport report = following();
  report.controllers.push_back(controller("joint_state_broadcaster", "active"));
  report.controllers.push_back(
    controller("someone_elses_controller", "active", {"theta2_boom_joint/velocity"}));

  const ActiveMode active = crane_supervisor::active_mode(config, report);
  EXPECT_EQ(
    active.unmodelled_claimants, (std::vector<std::string>{"someone_elses_controller"}));
  const std::string clause = crane_supervisor::mode_clause(config, active);
  EXPECT_NE(clause.find("someone_elses_controller"), std::string::npos) << clause;
  EXPECT_EQ(clause.find("joint_state_broadcaster"), std::string::npos) << clause;
}

TEST(SupervisorMode, TheModeAlreadyActiveIsANoOpAndIsReportedAsOne)
{
  const auto config = config_with_modes();
  const auto again = ask(config, Mode::Follow, following());
  EXPECT_TRUE(again.accepted);
  EXPECT_FALSE(again.switch_required);
  EXPECT_TRUE(again.activate.empty());
  EXPECT_TRUE(again.deactivate.empty());
  EXPECT_NE(again.message.find("no switch was issued"), std::string::npos) << again.message;
  EXPECT_EQ(again.active.mode, Mode::Follow);
}

TEST(SupervisorMode, ReleasingTheClaimIsAModeChangeAndSaysItIsNotAStop)
{
  // The one place this issue comes closest to §5.2's withheld stop path, and
  // the report is where the difference is stated: nothing is zeroed at the
  // driver boundary, and §7.2 measured that a manual controller which
  // deactivates leaves its last velocity latched on the interface it released.
  const auto config = config_with_modes();
  const auto to_idle = ask(config, Mode::Idle, following());
  ASSERT_TRUE(to_idle.accepted) << to_idle.message;
  EXPECT_TRUE(to_idle.activate.empty());
  EXPECT_EQ(to_idle.deactivate, (std::vector<std::string>{kVelocityController, kFollower}));
  EXPECT_NE(to_idle.message.find("not a stop"), std::string::npos) << to_idle.message;
  EXPECT_NE(to_idle.message.find("latched on the interface"), std::string::npos)
    << to_idle.message;
}

TEST(SupervisorMode, EveryRequestIsAnsweredWithACauseAndNeverWithAnEmptyResult)
{
  // ROS 2 Interfaces §1 and PRD user story 36, over every mode against every
  // state of the machine this core can be handed.  A refused switch that leaves
  // an operator with an unexplained no-op is the failure this sweep is written
  // against.
  const std::vector<ControllerManagerReport> machines{
    ControllerManagerReport{}, all_inactive(), following()};
  for (const auto & config : {default_config(), config_with_modes()}) {
    for (const auto & machine : machines) {
      for (std::uint8_t value = 0; value < 6; ++value) {
        crane_supervisor::SupervisorInput input = input_with(machine);
        for (const bool latched : {false, true}) {
          input.estop_latched = latched;
          const auto arbitration = crane_supervisor::arbitrate_mode(config, value, input);
          EXPECT_FALSE(arbitration.message.empty()) << static_cast<int>(value);
          // A refusal never plans anything away, and an accepted no-op never
          // plans anything either.
          if (!arbitration.accepted || !arbitration.switch_required) {
            EXPECT_TRUE(arbitration.activate.empty()) << arbitration.message;
            EXPECT_TRUE(arbitration.deactivate.empty()) << arbitration.message;
          }
          // Nothing is ever planned that is not a controller of a mode: the
          // tool claim is out of reach of a mode request by construction.
          for (const auto & name : arbitration.deactivate) {
            EXPECT_NE(name, kToolController) << arbitration.message;
          }
        }
      }
    }
  }
}

TEST(SupervisorMode, ValidateRefusesAConfigurationTheArbitrationCouldNotAnswerFrom)
{
  std::string reason;

  // Two modes with the *same set* of controllers: both would read as active at
  // once and the answer would depend on which list was checked first.  This is
  // what replaced the old pairwise-disjoint rule, and it keeps the defect
  // that rule was aimed at while letting through the overlap PRD §10 step 3
  // requires.
  auto indistinguishable = config_with_modes();
  indistinguishable.mode_controllers[crane_supervisor::index_of(Mode::Manual)] = {
    kVelocityController};
  EXPECT_FALSE(crane_supervisor::validate(indistinguishable, reason));
  EXPECT_NE(reason.find("same set"), std::string::npos) << reason;

  // The overlap itself is admitted, and it has to be: `MODE_MPC` is
  // `MODE_FOLLOW` without the trajectory controller because the same inner loop
  // carries both paths, and a rule that outlawed that would outlaw the
  // architecture.  This is the shipped configuration.
  EXPECT_TRUE(crane_supervisor::validate(config_with_modes(), reason)) << reason;

  // A name twice in one list: the list's length stops counting its controllers,
  // so the set comparison could never match and the mode would be unreachable.
  auto repeated = config_with_modes();
  repeated.mode_controllers[crane_supervisor::index_of(Mode::Manual)] = {
    kManualController, kManualController};
  EXPECT_FALSE(crane_supervisor::validate(repeated, reason));
  EXPECT_NE(reason.find("twice"), std::string::npos) << reason;

  // MODE_MPC implemented with no producer named: the claim would move to a path
  // whose producer publishes nothing, and the inner loop would be unchained
  // with no horizon.
  auto unnamed = config_with_modes();
  unnamed.mpc_node.clear();
  EXPECT_FALSE(crane_supervisor::validate(unnamed, reason));
  EXPECT_NE(reason.find("mpc_node"), std::string::npos) << reason;

  // ... and a deployment that implements neither is fine without one.
  auto without_mpc = unnamed;
  without_mpc.mode_controllers[crane_supervisor::index_of(Mode::Mpc)].clear();
  EXPECT_TRUE(crane_supervisor::validate(without_mpc, reason)) << reason;

  // A controller that is both a mode's and the tool claim's: an arm mode change
  // would deactivate the gripper it is supposed to leave alone (§7.3).
  auto shared_tool = config_with_modes();
  shared_tool.tool_controllers = {kFollower};
  EXPECT_FALSE(crane_supervisor::validate(shared_tool, reason));
  EXPECT_NE(reason.find("7.3"), std::string::npos) << reason;

  // MODE_IDLE with controllers: releasing the claim would become a claim.
  auto busy_idle = config_with_modes();
  busy_idle.mode_controllers[crane_supervisor::index_of(Mode::Idle)] = {kManualController};
  EXPECT_FALSE(crane_supervisor::validate(busy_idle, reason));
  EXPECT_NE(reason.find("MODE_IDLE"), std::string::npos) << reason;

  // And the polled view has a deadline like everything else, or the mode would
  // either always be unknown or never be.
  auto undated = config_with_modes();
  undated.controller_manager_deadline = 0.0;
  EXPECT_FALSE(crane_supervisor::validate(undated, reason));
  EXPECT_NE(reason.find("controller manager"), std::string::npos) << reason;

  EXPECT_TRUE(crane_supervisor::validate(config_with_modes(), reason)) << reason;
}
