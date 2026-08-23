// The decision core, offline. No ROS, no DDS, no clock is linked into this
// binary and none is reachable from it, so every cause can be produced exactly
// and one at a time -- which is the point of the core being ROS-free at all.

#include <gtest/gtest.h>

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

/// A report that is arriving, in time, and trusted by the broadcaster.
crane_supervisor::SupervisorInput healthy_input()
{
  crane_supervisor::SupervisorInput input;
  input.pendulum_state.received = true;
  input.pendulum_state.valid = true;
  input.pendulum_state.age = 0.01;
  input.pendulum_state.status = "complementary filter on the two bracketing IMUs";
  return input;
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

  EXPECT_TRUE(crane_supervisor::validate(default_config(), reason));
}

TEST(SupervisorCore, AbsenceIsNotHealthBeforeTheFirstMessage)
{
  // wiki/control_architecture.md §5.3: no input may stop arriving without a
  // defined consequence, and "has not started arriving" is the same absence.
  const auto decision = crane_supervisor::decide(default_config(), {});
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
          crane_supervisor::SupervisorInput input;
          input.pendulum_state.received = received;
          input.pendulum_state.valid = valid;
          input.pendulum_state.age = age;
          input.pendulum_state.status = status;
          inputs.push_back(input);
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
    // Nothing in this slice computes these three, and each carries the value
    // that claims nothing rather than a stub that claims something.
    EXPECT_EQ(decision.tracking_error, 0.0);
    EXPECT_FALSE(decision.inside_working_cell);
    EXPECT_FALSE(decision.deadman_held);
  }
}

TEST(SupervisorCore, TheOnlyFaultThisSliceRaisesIsStateHealth)
{
  // Every other cause of the §5 table belongs to a later issue.  A supervisor
  // that raised one of them from an input it does not have would be reporting a
  // check it never made.
  const auto config = default_config();
  for (const bool received : {false, true}) {
    for (const bool valid : {false, true}) {
      for (const double age : {-100.0, 0.0, 100.0}) {
        crane_supervisor::SupervisorInput input;
        input.pendulum_state.received = received;
        input.pendulum_state.valid = valid;
        input.pendulum_state.age = age;
        const auto fault = crane_supervisor::decide(config, input).fault;
        EXPECT_TRUE(
          fault == crane_supervisor::Fault::None ||
          fault == crane_supervisor::Fault::StateHealth)
          << static_cast<int>(fault);
      }
    }
  }
}
