
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

const std::vector<std::string> & actuated_joints()
{
  static const std::vector<std::string> joints{
    "theta1_slewing_joint", "theta2_boom_joint", "theta3_arm_joint",
    "q4_big_telescope", "theta8_rotator_joint", "q9_left_rail_joint"};
  return joints;
}

/// The four sway numbers `config/crane_supervisor.yaml` ships.
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

constexpr double kFirstCycle = 1'700'000'000.0;

/// The part of a report the fault composed, without the settled clause.
std::string without_settled_clause(const std::string & message)
{
  return message.substr(0, message.find(crane_supervisor::kSettledClausePrefix));
}

/// The four shipped freshness deadlines, s, in `Input` order.
crane_supervisor::SupervisorConfig with_shipped_deadlines(
  crane_supervisor::SupervisorConfig config)
{
  config.deadline(Input::PendulumState) = 0.15;
  config.deadline(Input::RemoteCtrl) = 0.25;
  config.deadline(Input::ControllerState) = 0.15;
  config.deadline(Input::ControllerHealth) = 0.25;
  config.controller_manager_deadline = 0.25;
  config.horizon_deadline = 0.3;
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
    config.tracking_tolerance.push_back(crane_supervisor::AxisTolerance{joint});
  }
  return with_shipped_deadlines(config);
}

/// The same configuration with the six numbers a human has not measured yet.
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

crane_supervisor::SupervisorInput healthy_input()
{
  crane_supervisor::SupervisorInput input;
  for (std::size_t i = 0; i < kInputCount; ++i) {
    input.streams[i].received = true;
    input.streams[i].age = 0.01;
  }
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
  std::string reason;
  EXPECT_TRUE(crane_supervisor::validate(default_config(), reason)) << reason;

  EXPECT_FALSE(crane_supervisor::validate(crane_supervisor::SupervisorConfig{}, reason));
  EXPECT_FALSE(reason.empty());

  for (std::size_t i = 0; i < kInputCount; ++i) {
    const Input which = static_cast<Input>(i);
    for (const double refused : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN()}) {
      crane_supervisor::SupervisorConfig config = default_config();
      config.deadline(which) = refused;
      EXPECT_FALSE(crane_supervisor::validate(config, reason)) << policy_of(which).topic;
      EXPECT_NE(reason.find(policy_of(which).topic), std::string::npos) << reason;
    }
  }
}

TEST(SupervisorCore, TheRegistryIsTheOneListAndItsRowsMatchTheirInputs)
{
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

    for (std::size_t a = 0; a < messages.size(); ++a) {
      for (std::size_t b = a + 1; b < messages.size(); ++b) {
        EXPECT_NE(messages[a], messages[b]) << policy_of(which).topic << ": " << messages[a];
      }
    }
  }
}

TEST(SupervisorCore, TheDeadlinesArePerInputAndNotOneNumber)
{
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
  EXPECT_TRUE(crane_supervisor::is_tolerance(0.02));
  EXPECT_FALSE(crane_supervisor::is_tolerance(-1.0));
  EXPECT_FALSE(crane_supervisor::is_tolerance(0.0));
  EXPECT_FALSE(crane_supervisor::is_tolerance(std::numeric_limits<double>::quiet_NaN()));
  EXPECT_FALSE(crane_supervisor::is_tolerance(std::numeric_limits<double>::infinity()));
}

TEST(SupervisorCore, ATrackingToleranceWithNoAxisNameIsRefused)
{
  std::string reason;
  crane_supervisor::SupervisorConfig config = config_with_tolerances();
  config.tracking_tolerance[3].joint.clear();
  EXPECT_FALSE(crane_supervisor::validate(config, reason));
  EXPECT_FALSE(reason.empty());
}

TEST(SupervisorCore, RejectsADeadmanButtonTheMessageDoesNotHave)
{
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

  EXPECT_EQ(default_config().deadman_button, 12);
}

TEST(SupervisorCore, AbsenceIsNotHealthBeforeTheFirstMessage)
{
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

  EXPECT_NE(stopped.message.find("0.420"), std::string::npos) << stopped.message;
  EXPECT_NE(stopped.message.find("0.150"), std::string::npos) << stopped.message;
  EXPECT_NE(stopped.message.find("stopped arriving"), std::string::npos) << stopped.message;
}

TEST(SupervisorCore, AStampAheadOfTheClockIsAFaultRatherThanAFreshSample)
{
  const auto config = default_config();
  auto input = healthy_input();

  input.stream(Input::PendulumState).age = -config.deadline(Input::PendulumState);
  EXPECT_EQ(crane_supervisor::decide(config, input).fault, crane_supervisor::Fault::None);

  input.stream(Input::PendulumState).age = -2.0 * config.deadline(Input::PendulumState);
  const auto ahead = crane_supervisor::decide(config, input);
  EXPECT_EQ(ahead.fault, crane_supervisor::Fault::StateHealth);
  EXPECT_NE(ahead.message.find("future"), std::string::npos) << ahead.message;
}

TEST(SupervisorCore, AHealthyTracerClearsTheFaultAndStillSaysWhatIsNotWatched)
{
  const auto decision = crane_supervisor::decide(default_config(), healthy_input());
  EXPECT_EQ(decision.fault, crane_supervisor::Fault::None);
  EXPECT_FALSE(decision.message.empty());
  EXPECT_NE(decision.message.find("only input"), std::string::npos) << decision.message;
  EXPECT_NE(decision.message.find("never times out"), std::string::npos) << decision.message;
}

TEST(SupervisorCore, AReleasedDeadmanIsAnInterlockAndIsCheckedEveryCycle)
{
  const auto config = default_config();
  auto input = healthy_input();

  EXPECT_EQ(step(config, input).fault, crane_supervisor::Fault::None);

  input.remote_ctrl.deadman_held = false;
  for (int cycle = 0; cycle < 3; ++cycle) {
    const auto released = step(config, input);
    EXPECT_EQ(released.fault, crane_supervisor::Fault::Interlock) << cycle;
    EXPECT_FALSE(released.deadman_held);
    EXPECT_NE(released.message.find("button 12"), std::string::npos) << released.message;
  }

  input.remote_ctrl.deadman_held = true;
  const auto pressed = step(config, input);
  EXPECT_EQ(pressed.fault, crane_supervisor::Fault::None);
  EXPECT_TRUE(pressed.deadman_held);
}

TEST(SupervisorCore, TheDeadmanIsReportedOnEveryDecisionWhateverTheFaultIs)
{
  const auto config = default_config();
  auto input = healthy_input();
  input.stream(Input::PendulumState).age = 10.0;

  const auto degraded = crane_supervisor::decide(config, input);
  EXPECT_EQ(degraded.fault, crane_supervisor::Fault::StateHealth);
  EXPECT_TRUE(degraded.deadman_held);

  input.stream(Input::RemoteCtrl).received = false;
  EXPECT_FALSE(crane_supervisor::decide(config, input).deadman_held);
}

TEST(SupervisorCore, AnAssertedStopLatchesAndSurvivesTheSignalGoingBackToReleased)
{
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

  input.remote_ctrl.em_stop = true;
  const auto again = step(config, input);
  EXPECT_EQ(again.fault, crane_supervisor::Fault::EStop);
  EXPECT_TRUE(again.estop_latched);
}

TEST(SupervisorCore, TheReportedErrorIsTheMaxOverTheActuatedJointsOfThePositionError)
{
  const auto config = default_config();
  auto input = healthy_input();

  input.controller_state = with_error("theta3_arm_joint", -0.30, 0.0);
  EXPECT_DOUBLE_EQ(crane_supervisor::decide(config, input).tracking_error, 0.30);

  for (auto & axis : input.controller_state.axes) {
    if (axis.joint == "q4_big_telescope") {
      axis.position_error = 0.44;
    }
  }
  EXPECT_DOUBLE_EQ(crane_supervisor::decide(config, input).tracking_error, 0.44);

  EXPECT_DOUBLE_EQ(crane_supervisor::max_position_error({}), 0.0);
}

TEST(SupervisorCore, TheReportedErrorIsCarriedWhateverTheFaultIs)
{
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
  const auto config = default_config();
  auto input = healthy_input();
  input.controller_state = with_error("theta2_boom_joint", 5.0, 5.0);

  const auto decision = crane_supervisor::decide(config, input);
  EXPECT_EQ(decision.fault, crane_supervisor::Fault::None);
  EXPECT_DOUBLE_EQ(decision.tracking_error, 5.0);
  EXPECT_NE(decision.message.find("No axis is being compared"), std::string::npos)
    << decision.message;
  EXPECT_NE(decision.message.find("human-only"), std::string::npos) << decision.message;

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
  const auto config = config_with_tolerances();
  auto input = healthy_input();

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
  const auto config = config_with_tolerances();
  auto input = healthy_input();
  input.controller_state = with_error("q9_left_rail_joint", 0.0, 0.5);

  const auto decision = crane_supervisor::decide(config, input);
  EXPECT_EQ(decision.fault, crane_supervisor::Fault::Tracking);
  const std::string tracking = without_settled_clause(decision.message);
  EXPECT_NE(tracking.find("q9_left_rail_joint"), std::string::npos) << tracking;
  EXPECT_NE(tracking.find("m/s"), std::string::npos) << tracking;
  EXPECT_EQ(tracking.find("rad/s"), std::string::npos) << tracking;
}

TEST(SupervisorCore, AxesArePairedByNameAndTheWorstOffenderIsNamedFirst)
{
  const auto config = config_with_tolerances();
  auto input = healthy_input();

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
  EXPECT_EQ(messages[1].find("not commissioned"), std::string::npos) << messages[1];
  EXPECT_NE(messages[1].find("stopped reporting its own health"), std::string::npos)
    << messages[1];
  EXPECT_NE(messages[2].find("future"), std::string::npos) << messages[2];
}

TEST(SupervisorCore, TheInnerLoopsOwnCodesAreCarriedRatherThanTranslated)
{
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
  EXPECT_NE(degraded.message, expired.message);

  input.controller_health = inner_loop_fault(crane_supervisor::Fault::Sway);
  const auto drifted = crane_supervisor::decide(config, input);
  EXPECT_EQ(drifted.fault, crane_supervisor::Fault::Sway);
  EXPECT_NE(drifted.message.find("not supposed to be able to raise"), std::string::npos)
    << drifted.message;
}

TEST(SupervisorCore, AMissingCalibrationNamesTheAxisItIsMissingFor)
{
  const auto config = default_config();
  auto input = healthy_input();
  input.controller_health = uncommissioned_gripper();

  const auto decision = crane_supervisor::decide(config, input);
  EXPECT_EQ(decision.fault, crane_supervisor::Fault::NotCommissioned);
  EXPECT_NE(decision.message.find("q9_left_rail_joint"), std::string::npos) << decision.message;
  EXPECT_NE(decision.message.find("calibration"), std::string::npos) << decision.message;
  // Nothing was acted on here either.
  EXPECT_NE(decision.message.find("Nothing was stopped"), std::string::npos) << decision.message;

  input.controller_health.feedforward_free_joints = {"theta8_rotator_joint", "q9_left_rail_joint"};
  const auto both = crane_supervisor::decide(config, input);
  EXPECT_NE(both.message.find("theta8_rotator_joint"), std::string::npos) << both.message;
  EXPECT_NE(both.message.find("q9_left_rail_joint"), std::string::npos) << both.message;

  input.controller_health.feedforward_free_joints.clear();
  const auto unnamed = crane_supervisor::decide(config, input);
  EXPECT_EQ(unnamed.fault, crane_supervisor::Fault::NotCommissioned);
  EXPECT_NE(unnamed.message.find("named no axis"), std::string::npos) << unnamed.message;
}

TEST(SupervisorCore, TheHealthCodeIsReportedInPreferenceToTheCommissioningCode)
{
  const auto config = default_config();

  // On its own the commissioning code is what is reported.
  auto only_commissioning = healthy_input();
  only_commissioning.controller_health = uncommissioned_gripper();
  EXPECT_EQ(
    crane_supervisor::decide(config, only_commissioning).fault,
    crane_supervisor::Fault::NotCommissioned);

  auto degraded_state = only_commissioning;
  degraded_state.stream(Input::PendulumState).age = 10.0;
  const auto against_state = crane_supervisor::decide(config, degraded_state);
  EXPECT_EQ(against_state.fault, crane_supervisor::Fault::StateHealth);
  EXPECT_NE(against_state.message.find("stopped arriving"), std::string::npos)
    << against_state.message;

  auto dead_controller = only_commissioning;
  dead_controller.stream(Input::ControllerState).age = 10.0;
  EXPECT_EQ(
    crane_supervisor::decide(config, dead_controller).fault, crane_supervisor::Fault::StateHealth);

  auto inner_health = only_commissioning;
  inner_health.controller_health.fault = crane_supervisor::Fault::StateHealth;
  const auto against_inner = crane_supervisor::decide(config, inner_health);
  EXPECT_EQ(against_inner.fault, crane_supervisor::Fault::StateHealth);
  EXPECT_NE(against_inner.message.find("inner velocity loop"), std::string::npos)
    << against_inner.message;

  auto stale_reference = only_commissioning;
  stale_reference.controller_health.fault = crane_supervisor::Fault::ReferenceStale;
  EXPECT_EQ(
    crane_supervisor::decide(config, stale_reference).fault,
    crane_supervisor::Fault::ReferenceStale);

  auto released = only_commissioning;
  released.remote_ctrl.deadman_held = false;
  const auto over_interlock = crane_supervisor::decide(config, released);
  EXPECT_EQ(over_interlock.fault, crane_supervisor::Fault::NotCommissioned);
  EXPECT_FALSE(over_interlock.deadman_held);
}

TEST(SupervisorCore, TheProfileSwitchIsTheControllersAndThisPackageAddsNoSecondOne)
{
  const auto config = default_config();

  // What the `hardware` profile publishes today.
  auto hardware = healthy_input();
  hardware.controller_health = uncommissioned_gripper();
  const auto reported = crane_supervisor::decide(config, hardware);
  EXPECT_EQ(reported.fault, crane_supervisor::Fault::NotCommissioned);

  auto fake = healthy_input();
  fake.controller_health = healthy_inner_loop();
  fake.controller_health.feedforward_free_joints = {"q9_left_rail_joint"};
  const auto quiet = crane_supervisor::decide(config, fake);
  EXPECT_EQ(quiet.fault, crane_supervisor::Fault::None);
  EXPECT_EQ(quiet.message.find("not commissioned"), std::string::npos) << quiet.message;
}

TEST(SupervisorCore, TheStopAndTheStateOutrankTrackingAndTrackingOutranksTheInterlock)
{
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
  EXPECT_NE(decision.message.find("Nothing was stopped"), std::string::npos) << decision.message;
  // And a rate past the fault bound is certainly not settled.
  EXPECT_EQ(decision.sway.settled, crane_supervisor::SwaySettled::NotSettled);
}

TEST(SupervisorCore, ADegradedEstimateIsStateHealthAndNeverSway)
{
  const auto config = default_config();

  auto invalid = swinging(9.0);
  invalid.stream(Input::PendulumState).age = 10.0;
  const auto degraded = crane_supervisor::decide(config, invalid);
  EXPECT_EQ(degraded.fault, crane_supervisor::Fault::StateHealth);
  EXPECT_EQ(degraded.sway.settled, crane_supervisor::SwaySettled::Unknown);
  EXPECT_NE(degraded.message.find("stopped arriving"), std::string::npos)
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
  const auto config = default_config();
  auto input = healthy_input();
  input.remote_ctrl.deadman_held = false;

  const int cycles = 2 * static_cast<int>(config.sway.settle_dwell / kCycle);
  crane_supervisor::SwaySettled settled = crane_supervisor::SwaySettled::Unknown;
  for (int cycle = 0; cycle < cycles; ++cycle) {
    const auto decision = step(config, input);
    ASSERT_EQ(decision.fault, crane_supervisor::Fault::Interlock) << cycle;
    settled = decision.sway.settled;
  }
  EXPECT_EQ(settled, crane_supervisor::SwaySettled::Settled);

  input.pendulum_state.velocity = {{0.2, 0.0}};
  const auto moving = step(config, input);
  EXPECT_EQ(moving.fault, crane_supervisor::Fault::Interlock);
  EXPECT_EQ(moving.sway.settled, crane_supervisor::SwaySettled::NotSettled);
}

TEST(SupervisorCore, NothingHereActs)
{
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

/// The two things outside the four inputs that the solver branch reads.
struct HorizonContext
{
  crane_supervisor::ControllerManagerReport manager;
  crane_supervisor::HorizonReport horizon;
};

/// One row of what the controller manager would have answered.
crane_supervisor::ControllerManagerReport claim_held_by(
  const std::vector<std::string> & active_controllers)
{
  crane_supervisor::ControllerManagerReport report;
  report.answer.received = true;
  report.answer.age = 0.01;
  for (const char * name : {"crane_velocity_controller", "trajectory_controller_a2b"}) {
    crane_supervisor::ControllerReport entry;
    entry.name = name;
    const bool up = std::find(active_controllers.begin(), active_controllers.end(), name) !=
      active_controllers.end();
    entry.state = up ? "active" : "inactive";
    if (up) {
      entry.claimed_interfaces = {"theta1_slewing_joint/velocity"};
    }
    report.controllers.push_back(std::move(entry));
  }
  return report;
}

/// What the horizon producer last said, `age` seconds ago.
crane_supervisor::HorizonReport reported(
  double age, crane_supervisor::SolveOutcome outcome, crane_supervisor::Fault fault,
  bool applied_previous_solution)
{
  crane_supervisor::HorizonReport report;
  report.health.received = true;
  report.health.age = age;
  report.outcome = outcome;
  report.fault = fault;
  report.solve_time = 0.012;
  report.solve_budget = 0.03;
  report.applied_previous_solution = applied_previous_solution;
  report.status = "the producer's own account of the cycle";
  return report;
}

/// The five combinations of mode and producer the solver branch turns on.
std::vector<HorizonContext> every_horizon_context()
{
  const std::vector<std::string> follow{
    "crane_velocity_controller", "trajectory_controller_a2b"};
  const std::vector<std::string> mpc{"crane_velocity_controller"};
  return {
    HorizonContext{},
    HorizonContext{
      claim_held_by(follow),
      reported(0.01, crane_supervisor::SolveOutcome::Failed, crane_supervisor::Fault::Solver,
      false)},
    // MODE_MPC with the optimizer converging: the mode this slice makes real.
    HorizonContext{
      claim_held_by(mpc),
      reported(0.01, crane_supervisor::SolveOutcome::Converged, crane_supervisor::Fault::None,
      false)},
    HorizonContext{
      claim_held_by(mpc),
      reported(0.01, crane_supervisor::SolveOutcome::Failed, crane_supervisor::Fault::Solver,
      false)},
    HorizonContext{
      claim_held_by(mpc),
      reported(100.0, crane_supervisor::SolveOutcome::Failed, crane_supervisor::Fault::Solver,
      false)}};
}

/// The whole reachable input space of this slice, one struct per combination.
std::vector<crane_supervisor::SupervisorInput> every_input()
{
  std::vector<crane_supervisor::SupervisorInput> inputs;
  for (const bool received : {false, true}) {
    {
      for (const double age : {-100.0, -0.05, 0.0, 0.05, 100.0}) {
        {
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

  std::vector<crane_supervisor::SupervisorInput> swept;
  swept.reserve(inputs.size() * 3);
  for (const double dq_u : {0.0, 0.1, 9.0}) {
    for (crane_supervisor::SupervisorInput input : inputs) {
      input.pendulum_state.velocity = {{dq_u, dq_u}};
      swept.push_back(std::move(input));
    }
  }

  std::vector<crane_supervisor::SupervisorInput> composed;
  const std::vector<HorizonContext> contexts = every_horizon_context();
  composed.reserve(swept.size() * contexts.size());
  for (const HorizonContext & context : contexts) {
    for (crane_supervisor::SupervisorInput input : swept) {
      input.controller_manager = context.manager;
      input.horizon = context.horizon;
      composed.push_back(std::move(input));
    }
  }
  return composed;
}

}  // namespace

TEST(SupervisorCore, EveryReachableDecisionCarriesACause)
{
  for (const auto & config : {default_config(), config_with_tolerances()}) {
    for (const auto & input : every_input()) {
      const auto decision = crane_supervisor::decide(config, input);
      EXPECT_FALSE(decision.message.empty());
      const auto active = crane_supervisor::active_mode(config, input.controller_manager);
      EXPECT_EQ(decision.mode, active.mode);
      if (!active.known) {
        EXPECT_EQ(decision.mode, crane_supervisor::Mode::Idle);
      }
      EXPECT_NE(decision.message.find(crane_supervisor::kModeClausePrefix), std::string::npos)
        << decision.message;
      EXPECT_FALSE(decision.inside_working_cell);
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
  for (const auto & config : {default_config(), config_with_tolerances()}) {
    for (const auto & input : every_input()) {
      const auto decision = crane_supervisor::decide(config, input);
      const auto causes = crane_supervisor::freshness(config, input);
      const bool trusted = causes[index_of(Input::PendulumState)] == Staleness::Fresh;
      if (trusted) {
        EXPECT_NE(decision.sway.settled, crane_supervisor::SwaySettled::Unknown);
      } else {
        EXPECT_EQ(decision.sway.settled, crane_supervisor::SwaySettled::Unknown);
      }
      EXPECT_NE(decision.message.find(crane_supervisor::kSettledClausePrefix), std::string::npos)
        << decision.message;
    }
  }
}

TEST(SupervisorCore, NoTrackingFaultIsReachableWithoutATolerance)
{
  const auto config = default_config();
  for (const auto & input : every_input()) {
    EXPECT_NE(crane_supervisor::decide(config, input).fault, crane_supervisor::Fault::Tracking);
  }
}

TEST(SupervisorCore, TheFaultsThisSliceRaisesAreItsOwnFiveAndTheOtherLoopsFour)
{
  for (const auto & config : {default_config(), config_with_tolerances()}) {
    for (const auto & input : every_input()) {
      const auto fault = crane_supervisor::decide(config, input).fault;
      EXPECT_TRUE(
        fault == crane_supervisor::Fault::None ||
        fault == crane_supervisor::Fault::StateHealth ||
        fault == crane_supervisor::Fault::Tracking ||
        fault == crane_supervisor::Fault::Sway ||
        fault == crane_supervisor::Fault::Solver ||
        fault == crane_supervisor::Fault::ReferenceStale ||
        fault == crane_supervisor::Fault::EStop ||
        fault == crane_supervisor::Fault::Interlock ||
        fault == crane_supervisor::Fault::NotCommissioned)
        << static_cast<int>(fault);
      if (fault == crane_supervisor::Fault::ReferenceStale ||
        fault == crane_supervisor::Fault::NotCommissioned)
      {
        EXPECT_EQ(fault, input.controller_health.fault);
        EXPECT_TRUE(input.stream(Input::ControllerHealth).received);
      }
      if (fault == crane_supervisor::Fault::Solver) {
        EXPECT_EQ(fault, input.horizon.fault);
        const auto active = crane_supervisor::active_mode(config, input.controller_manager);
        EXPECT_TRUE(active.known);
        EXPECT_EQ(active.mode, crane_supervisor::Mode::Mpc);
        EXPECT_EQ(
          crane_supervisor::freshness_of(config.horizon_deadline, input.horizon.health),
          Staleness::Fresh);
      }
      if (fault == crane_supervisor::Fault::Sway) {
        EXPECT_TRUE(input.stream(Input::PendulumState).received);
      }
    }
  }
}


namespace
{

/// A healthy machine with a named mode holding the claim and a named producer.
crane_supervisor::SupervisorInput driving(
  const std::vector<std::string> & active_controllers, crane_supervisor::HorizonReport horizon)
{
  crane_supervisor::SupervisorInput input = healthy_input();
  input.controller_manager = claim_held_by(active_controllers);
  input.horizon = std::move(horizon);
  return input;
}

const std::vector<std::string> & mpc_claim()
{
  static const std::vector<std::string> claim{"crane_velocity_controller"};
  return claim;
}

const std::vector<std::string> & follow_claim()
{
  static const std::vector<std::string> claim{
    "crane_velocity_controller", "trajectory_controller_a2b"};
  return claim;
}

crane_supervisor::HorizonReport escalated()
{
  return reported(
    0.01, crane_supervisor::SolveOutcome::Failed, crane_supervisor::Fault::Solver, false);
}

crane_supervisor::HorizonReport shifted()
{
  return reported(
    0.01, crane_supervisor::SolveOutcome::BudgetExceeded, crane_supervisor::Fault::Solver, true);
}

/// The active mode of an observation, read the way `decide()` reads it.
crane_supervisor::ActiveMode mode_of(
  const crane_supervisor::SupervisorConfig & config,
  const crane_supervisor::SupervisorInput & input)
{
  return crane_supervisor::active_mode(config, input.controller_manager);
}

}  // namespace

TEST(SupervisorSolver, TheProducersCodeIsCarriedRatherThanTranslated)
{
  const auto config = default_config();
  const auto decision = crane_supervisor::decide(config, driving(mpc_claim(), escalated()));

  EXPECT_EQ(decision.fault, crane_supervisor::Fault::Solver);
  EXPECT_EQ(static_cast<std::uint8_t>(decision.fault), 3U) << "the code was renumbered on the way";
  EXPECT_NE(decision.message.find("the producer's own account"), std::string::npos)
    << decision.message;
  EXPECT_NE(decision.message.find("nothing went out on the horizon"), std::string::npos)
    << decision.message;
  EXPECT_EQ(decision.mode, crane_supervisor::Mode::Mpc);
}

TEST(SupervisorSolver, AShiftedPreviousSolutionSaysSoRatherThanClaimingSilence)
{
  const auto config = default_config();
  const auto decision = crane_supervisor::decide(config, driving(mpc_claim(), shifted()));

  EXPECT_EQ(decision.fault, crane_supervisor::Fault::Solver);
  EXPECT_NE(decision.message.find("shifted by one step"), std::string::npos) << decision.message;
  EXPECT_EQ(decision.message.find("nothing went out on the horizon"), std::string::npos)
    << decision.message;
}

TEST(SupervisorSolver, AFailedShadowSolveDroveNothingAndIsNotReported)
{
  const auto config = default_config();
  const auto decision = crane_supervisor::decide(config, driving(follow_claim(), escalated()));

  EXPECT_EQ(decision.fault, crane_supervisor::Fault::None);
  EXPECT_EQ(decision.mode, crane_supervisor::Mode::Follow);
  EXPECT_FALSE(crane_supervisor::solver_handback(
      config, driving(follow_claim(), escalated()), mode_of(config, driving(follow_claim(),
      escalated()))).required);
}

TEST(SupervisorSolver, AVerdictNobodyHasHeardSinceIsNotAnObservation)
{
  const auto config = default_config();
  crane_supervisor::HorizonReport old = escalated();
  old.health.age = 10.0 * config.horizon_deadline;
  const auto input = driving(mpc_claim(), old);

  EXPECT_EQ(crane_supervisor::decide(config, input).fault, crane_supervisor::Fault::None);
  EXPECT_FALSE(crane_supervisor::solver_handback(config, input, mode_of(config, input)).required);
}

TEST(SupervisorSolver, TheProducersCauseOutranksTheReceiversSymptom)
{
  const auto config = default_config();
  auto input = driving(mpc_claim(), escalated());
  input.controller_health = inner_loop_fault(crane_supervisor::Fault::ReferenceStale);

  EXPECT_EQ(crane_supervisor::decide(config, input).fault, crane_supervisor::Fault::Solver);

  auto stale_only = driving(
    mpc_claim(),
    reported(
      0.01, crane_supervisor::SolveOutcome::Converged, crane_supervisor::Fault::None, false));
  stale_only.controller_health = inner_loop_fault(crane_supervisor::Fault::ReferenceStale);
  EXPECT_EQ(
    crane_supervisor::decide(config, stale_only).fault, crane_supervisor::Fault::ReferenceStale);
}

TEST(SupervisorSolver, TheStopAndTheDegradedEstimateStillOutrankIt)
{
  const auto config = default_config();

  auto stopped = driving(mpc_claim(), escalated());
  stopped.remote_ctrl.em_stop = true;
  EXPECT_EQ(crane_supervisor::decide(config, stopped).fault, crane_supervisor::Fault::EStop);

  auto blind = driving(mpc_claim(), escalated());
  blind.stream(Input::PendulumState).received = false;
  EXPECT_EQ(crane_supervisor::decide(config, blind).fault, crane_supervisor::Fault::StateHealth);
}

TEST(SupervisorSolver, TheHandBackWaitsForTheReceiverToRunOutOfPlan)
{
  const auto config = default_config();

  const auto still_executing = driving(mpc_claim(), escalated());
  EXPECT_FALSE(
    crane_supervisor::solver_handback(
      config, still_executing, mode_of(config, still_executing)).required)
    << "the claim was released while the receiver still had plan to run";

  auto ran_out = still_executing;
  ran_out.controller_health = inner_loop_fault(crane_supervisor::Fault::ReferenceStale);
  const auto handback =
    crane_supervisor::solver_handback(config, ran_out, mode_of(config, ran_out));
  ASSERT_TRUE(handback.required);
  EXPECT_NE(handback.message.find("handed control back"), std::string::npos) << handback.message;
  EXPECT_NE(handback.message.find("MODE_IDLE"), std::string::npos) << handback.message;
  EXPECT_NE(handback.message.find("task layer"), std::string::npos) << handback.message;
}

TEST(SupervisorSolver, AShiftedFallbackIsNotAHandBack)
{
  const auto config = default_config();
  auto input = driving(mpc_claim(), shifted());
  input.controller_health = inner_loop_fault(crane_supervisor::Fault::ReferenceStale);

  EXPECT_FALSE(crane_supervisor::solver_handback(config, input, mode_of(config, input)).required);
}

TEST(SupervisorSolver, AnInnerLoopThatStoppedReportingAsksForNoHandBack)
{
  const auto config = default_config();
  auto input = driving(mpc_claim(), escalated());
  input.controller_health = inner_loop_fault(crane_supervisor::Fault::ReferenceStale);
  input.stream(Input::ControllerHealth).received = false;

  EXPECT_FALSE(crane_supervisor::solver_handback(config, input, mode_of(config, input)).required);
}

TEST(SupervisorSolver, NoHandBackFromAModeThatIsNotMpcOrFromAViewNobodyHas)
{
  const auto config = default_config();

  auto following = driving(follow_claim(), escalated());
  following.controller_health = inner_loop_fault(crane_supervisor::Fault::ReferenceStale);
  EXPECT_FALSE(
    crane_supervisor::solver_handback(config, following, mode_of(config, following)).required);

  auto unseen = healthy_input();
  unseen.horizon = escalated();
  unseen.controller_health = inner_loop_fault(crane_supervisor::Fault::ReferenceStale);
  EXPECT_FALSE(
    crane_supervisor::solver_handback(config, unseen, mode_of(config, unseen)).required);
}

TEST(SupervisorSolver, TheHandBackIsAPredicateAndActsOnNothing)
{
  const auto config = default_config();
  auto input = driving(mpc_claim(), escalated());
  input.controller_health = inner_loop_fault(crane_supervisor::Fault::ReferenceStale);

  const auto first = crane_supervisor::solver_handback(config, input, mode_of(config, input));
  const auto second = crane_supervisor::solver_handback(config, input, mode_of(config, input));
  EXPECT_TRUE(first.required);
  EXPECT_EQ(first.required, second.required);
  EXPECT_EQ(first.message, second.message);
}


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

  ControllerManagerReport unchained = following();
  unchained.controllers[1].state = "inactive";
  const ActiveMode mpc = crane_supervisor::active_mode(config, unchained);
  EXPECT_TRUE(mpc.known);
  EXPECT_FALSE(mpc.partial);
  EXPECT_EQ(mpc.mode, Mode::Mpc);
  EXPECT_EQ(mpc.active_controllers, (std::vector<std::string>{kVelocityController}));

  EXPECT_EQ(crane_supervisor::active_mode(config, following()).mode, Mode::Follow);

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

  const ActiveMode silent = crane_supervisor::active_mode(config, {});
  EXPECT_FALSE(silent.known);
  EXPECT_EQ(silent.mode, Mode::Idle);
  EXPECT_EQ(silent.cause, Staleness::NeverArrived);
  EXPECT_NE(
    crane_supervisor::mode_clause(config, silent).find("not known"), std::string::npos);

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
  const auto config = config_with_modes();

  const auto absent = ask(config, Mode::Mpc, following());
  EXPECT_FALSE(absent.accepted);
  EXPECT_TRUE(absent.deactivate.empty()) << "the trajectory controller was planned away";
  EXPECT_TRUE(absent.activate.empty());
  EXPECT_NE(absent.message.find("freshness"), std::string::npos) << absent.message;
  EXPECT_NE(absent.message.find("Nothing was deactivated"), std::string::npos) << absent.message;
  EXPECT_EQ(absent.active.mode, Mode::Follow);

  crane_supervisor::HorizonReport late = solving(10.0 * config.horizon_deadline);
  const auto stale = crane_supervisor::arbitrate_mode(
    config, static_cast<std::uint8_t>(Mode::Mpc), warm(input_with(following()), late));
  EXPECT_FALSE(stale.accepted);
  EXPECT_TRUE(stale.deactivate.empty());
  EXPECT_NE(stale.message.find("3.000 s old"), std::string::npos) << stale.message;
  EXPECT_NE(stale.message.find("0.300 s"), std::string::npos) << stale.message;
  EXPECT_NE(stale.message, absent.message);

  crane_supervisor::HorizonReport ahead = solving(-10.0 * config.horizon_deadline);
  const auto skewed = crane_supervisor::arbitrate_mode(
    config, static_cast<std::uint8_t>(Mode::Mpc), warm(input_with(following()), ahead));
  EXPECT_FALSE(skewed.accepted);
  EXPECT_TRUE(skewed.deactivate.empty());
  EXPECT_NE(skewed.message.find("future"), std::string::npos) << skewed.message;
}

TEST(SupervisorMode, AProducerThatIsAliveAndNotConvergingIsTheDeadMpcStepTwoRefuses)
{
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
    EXPECT_NE(refused.message.find("30 ms budget"), std::string::npos) << refused.message;
  }

  // And the converged one is the only one that passes.
  EXPECT_TRUE(crane_supervisor::horizon_precondition(config, solving()).fresh);
}

TEST(SupervisorMode, TheFreshnessCheckIsAFunctionOfItsOwnAndItAnswersOnlyAboutMpc)
{
  const auto config = config_with_modes();
  const crane_supervisor::SupervisorInput cold = input_with(following());

  for (const Mode mode : {Mode::Idle, Mode::Manual, Mode::Follow}) {
    EXPECT_TRUE(
      crane_supervisor::mpc_horizon_refusal(config, static_cast<std::uint8_t>(mode), cold).empty())
      << crane_supervisor::mode_name(mode);
  }
  EXPECT_TRUE(crane_supervisor::mpc_horizon_refusal(config, 200, cold).empty());

  const std::string refusal =
    crane_supervisor::mpc_horizon_refusal(config, static_cast<std::uint8_t>(Mode::Mpc), cold);
  EXPECT_FALSE(refusal.empty());
  EXPECT_EQ(refusal, ask(config, Mode::Mpc, following()).message);

  crane_supervisor::SupervisorInput blind = healthy_input();
  EXPECT_EQ(
    crane_supervisor::arbitrate_mode(config, static_cast<std::uint8_t>(Mode::Mpc), blind).message,
    refusal);
}

TEST(SupervisorMode, AWarmHorizonAdmitsMpcAndThePlanNeverNamesTheInnerLoop)
{
  const auto config = config_with_modes();
  const auto accepted = crane_supervisor::arbitrate_mode(
    config, static_cast<std::uint8_t>(Mode::Mpc), warm(input_with(following())));
  ASSERT_TRUE(accepted.accepted) << accepted.message;
  EXPECT_TRUE(accepted.switch_required);
  EXPECT_EQ(accepted.deactivate, (std::vector<std::string>{kFollower}));
  EXPECT_TRUE(accepted.activate.empty()) << "the inner loop was planned to be reactivated";
  // The other half of the switch, and this supervisor's alone.
  EXPECT_TRUE(accepted.horizon_producer_active);

  const auto back = crane_supervisor::arbitrate_mode(
    config, static_cast<std::uint8_t>(Mode::Follow), warm(input_with(unchained())));
  ASSERT_TRUE(back.accepted) << back.message;
  EXPECT_EQ(back.activate, (std::vector<std::string>{kFollower}));
  EXPECT_TRUE(back.deactivate.empty()) << "the inner loop was planned away across the handover";
  EXPECT_FALSE(back.horizon_producer_active);

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
  std::string reason;
  auto undated = config_with_modes();
  undated.horizon_deadline = 0.0;
  EXPECT_FALSE(crane_supervisor::validate(undated, reason));
  EXPECT_NE(reason.find("horizon"), std::string::npos) << reason;

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
  EXPECT_NE(refused.message.find("emergency stop"), std::string::npos) << refused.message;
  EXPECT_NE(refused.message.find("/crane/clear_fault"), std::string::npos) << refused.message;

  crane_supervisor::SupervisorInput absent = input_with(all_inactive());
  absent.stream(Input::RemoteCtrl).received = false;
  EXPECT_FALSE(
    crane_supervisor::arbitrate_mode(config, static_cast<std::uint8_t>(Mode::Manual), absent)
    .accepted);

  crane_supervisor::SupervisorInput latched_and_following = input_with(following());
  latched_and_following.estop_latched = true;
  const auto to_idle = crane_supervisor::arbitrate_mode(
    config, static_cast<std::uint8_t>(Mode::Idle), latched_and_following);
  EXPECT_TRUE(to_idle.accepted) << to_idle.message;
  EXPECT_TRUE(to_idle.switch_required);
}

TEST(SupervisorMode, EveryPreconditionIsCheckedBeforeAnythingIsPlannedAwayFromTheActiveMode)
{
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

  for (const auto & refusal : {unconfigured, missing, stuck}) {
    EXPECT_FALSE(refusal.message.empty());
    EXPECT_NE(refusal.message.find("Nothing was deactivated"), std::string::npos)
      << refusal.message;
    EXPECT_EQ(refusal.active.mode, Mode::Follow);
  }
}

TEST(SupervisorMode, TheModesAreMutuallyExclusiveAndTheSwitchIsOneCall)
{
  const auto config = config_with_modes();
  const auto to_manual = ask(config, Mode::Manual, following());
  ASSERT_TRUE(to_manual.accepted) << to_manual.message;
  EXPECT_TRUE(to_manual.switch_required);
  EXPECT_EQ(to_manual.activate, (std::vector<std::string>{kManualController}));
  EXPECT_EQ(to_manual.deactivate, (std::vector<std::string>{kVelocityController, kFollower}));

  const auto to_follow = ask(config, Mode::Follow, all_inactive());
  ASSERT_TRUE(to_follow.accepted) << to_follow.message;
  EXPECT_EQ(to_follow.activate, (std::vector<std::string>{kVelocityController, kFollower}));
  EXPECT_TRUE(to_follow.deactivate.empty());
}

TEST(SupervisorMode, TheToolClaimIsNeverSwitchedByAModeRequest)
{
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

  const auto to_idle = ask(config, Mode::Idle, with_tool);
  ASSERT_TRUE(to_idle.accepted) << to_idle.message;
  EXPECT_EQ(to_idle.deactivate, (std::vector<std::string>{kVelocityController, kFollower}));

  const ActiveMode active = crane_supervisor::active_mode(config, with_tool);
  EXPECT_EQ(active.active_tool_controllers, (std::vector<std::string>{kToolController}));
  const std::string clause = crane_supervisor::mode_clause(config, active);
  EXPECT_NE(clause.find("Tool claim"), std::string::npos) << clause;
  EXPECT_NE(clause.find(kToolController), std::string::npos) << clause;
}

TEST(SupervisorMode, ADeploymentWithNoToolClaimSaysSoRatherThanImplyingOneClaim)
{
  const auto config = default_config();
  const std::string clause =
    crane_supervisor::mode_clause(config, crane_supervisor::active_mode(config, following()));
  EXPECT_NE(clause.find("No tool claim is configured"), std::string::npos) << clause;
  EXPECT_NE(clause.find("7.3"), std::string::npos) << clause;
}

TEST(SupervisorMode, AClaimTheConfigurationDoesNotDescribeIsReportedRatherThanIgnored)
{
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
          if (!arbitration.accepted || !arbitration.switch_required) {
            EXPECT_TRUE(arbitration.activate.empty()) << arbitration.message;
            EXPECT_TRUE(arbitration.deactivate.empty()) << arbitration.message;
          }
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

  auto indistinguishable = config_with_modes();
  indistinguishable.mode_controllers[crane_supervisor::index_of(Mode::Manual)] = {
    kVelocityController};
  EXPECT_FALSE(crane_supervisor::validate(indistinguishable, reason));
  EXPECT_NE(reason.find("same set"), std::string::npos) << reason;

  EXPECT_TRUE(crane_supervisor::validate(config_with_modes(), reason)) << reason;

  auto repeated = config_with_modes();
  repeated.mode_controllers[crane_supervisor::index_of(Mode::Manual)] = {
    kManualController, kManualController};
  EXPECT_FALSE(crane_supervisor::validate(repeated, reason));
  EXPECT_NE(reason.find("twice"), std::string::npos) << reason;

  auto unnamed = config_with_modes();
  unnamed.mpc_node.clear();
  EXPECT_FALSE(crane_supervisor::validate(unnamed, reason));
  EXPECT_NE(reason.find("mpc_node"), std::string::npos) << reason;

  // ... and a deployment that implements neither is fine without one.
  auto without_mpc = unnamed;
  without_mpc.mode_controllers[crane_supervisor::index_of(Mode::Mpc)].clear();
  EXPECT_TRUE(crane_supervisor::validate(without_mpc, reason)) << reason;

  auto shared_tool = config_with_modes();
  shared_tool.tool_controllers = {kFollower};
  EXPECT_FALSE(crane_supervisor::validate(shared_tool, reason));
  EXPECT_NE(reason.find("7.3"), std::string::npos) << reason;

  // MODE_IDLE with controllers: releasing the claim would become a claim.
  auto busy_idle = config_with_modes();
  busy_idle.mode_controllers[crane_supervisor::index_of(Mode::Idle)] = {kManualController};
  EXPECT_FALSE(crane_supervisor::validate(busy_idle, reason));
  EXPECT_NE(reason.find("MODE_IDLE"), std::string::npos) << reason;

  auto undated = config_with_modes();
  undated.controller_manager_deadline = 0.0;
  EXPECT_FALSE(crane_supervisor::validate(undated, reason));
  EXPECT_NE(reason.find("controller manager"), std::string::npos) << reason;

  EXPECT_TRUE(crane_supervisor::validate(config_with_modes(), reason)) << reason;
}
