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

crane_supervisor::SupervisorConfig default_config()
{
  return crane_supervisor::SupervisorConfig{};
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

  EXPECT_TRUE(crane_supervisor::validate(default_config(), reason));
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

TEST(SupervisorCore, NothingHereActs)
{
  // The whole action of this supervisor is to report.  There is no stop, no
  // ramp, no deactivation and no command in the decision it returns, and the
  // three fields it does not compute still carry the value that claims nothing.
  const auto config = default_config();
  auto input = healthy_input();
  input.remote_ctrl.em_stop = true;

  const auto decision = crane_supervisor::decide(config, input);
  EXPECT_EQ(decision.mode, crane_supervisor::Mode::Idle);
  EXPECT_EQ(decision.tracking_error, 0.0);
  EXPECT_FALSE(decision.inside_working_cell);
  EXPECT_NE(decision.message.find("not protection"), std::string::npos) << decision.message;
}

TEST(SupervisorCore, EveryReachableDecisionCarriesACause)
{
  // PRD user story 53, asserted over the whole reachable input space rather
  // than over the cases the tests above happen to name: a report with a fault
  // and no words is the inferred abort §5.0 removes, back again.
  const auto config = default_config();
  std::vector<crane_supervisor::SupervisorInput> inputs;
  for (const bool received : {false, true}) {
    for (const bool valid : {false, true}) {
      for (const double age : {-100.0, -0.05, 0.0, 0.05, 100.0}) {
        for (const char * status : {"", "a cause the estimator distinguished"}) {
          for (const bool remote_received : {false, true}) {
            for (const bool em_stop : {false, true}) {
              for (const bool deadman : {false, true}) {
                for (const bool latched : {false, true}) {
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
                  inputs.push_back(input);
                }
              }
            }
          }
        }
      }
    }
  }

  for (const auto & input : inputs) {
    const auto decision = crane_supervisor::decide(config, input);
    EXPECT_FALSE(decision.message.empty());
    // Mode arbitration is a later issue; until then the supervisor reports the
    // one mode it has verified, whatever it was told.
    EXPECT_EQ(decision.mode, crane_supervisor::Mode::Idle);
    // Nothing in this slice computes these two, and each carries the value
    // that claims nothing rather than a stub that claims something.
    EXPECT_EQ(decision.tracking_error, 0.0);
    EXPECT_FALSE(decision.inside_working_cell);
    // A latch is never lowered by a status cycle, whatever else it decides.
    if (input.estop_latched) {
      EXPECT_TRUE(decision.estop_latched);
      EXPECT_EQ(decision.fault, crane_supervisor::Fault::EStop);
    }
    // And the acknowledgement is total too: it answers every input with words.
    EXPECT_FALSE(crane_supervisor::clear_fault(config, input).message.empty());
  }
}

TEST(SupervisorCore, TheFaultsThisSliceRaisesAreStateHealthEStopAndInterlock)
{
  // Every other cause of the §5 table belongs to a later issue.  A supervisor
  // that raised one of them from an input it does not have would be reporting a
  // check it never made.
  const auto config = default_config();
  for (const bool received : {false, true}) {
    for (const bool valid : {false, true}) {
      for (const double age : {-100.0, 0.0, 100.0}) {
        for (const bool remote_received : {false, true}) {
          for (const bool em_stop : {false, true}) {
            for (const bool deadman : {false, true}) {
              crane_supervisor::SupervisorInput input;
              input.pendulum_state.received = received;
              input.pendulum_state.valid = valid;
              input.pendulum_state.age = age;
              input.remote_ctrl.received = remote_received;
              input.remote_ctrl.em_stop = em_stop;
              input.remote_ctrl.deadman_held = deadman;
              input.remote_ctrl.age = age;
              const auto fault = crane_supervisor::decide(config, input).fault;
              EXPECT_TRUE(
                fault == crane_supervisor::Fault::None ||
                fault == crane_supervisor::Fault::StateHealth ||
                fault == crane_supervisor::Fault::EStop ||
                fault == crane_supervisor::Fault::Interlock)
                << static_cast<int>(fault);
            }
          }
        }
      }
    }
  }
}
