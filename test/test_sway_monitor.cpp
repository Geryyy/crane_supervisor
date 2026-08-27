
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include "crane_supervisor/sway_monitor.hpp"

namespace
{

using crane_supervisor::PassiveAxis;
using crane_supervisor::SwayBound;
using crane_supervisor::SwaySettled;
using crane_supervisor::SwayState;
using crane_supervisor::SwayVerdict;
using crane_supervisor::index_of;
using crane_supervisor::judge_sway;
using crane_supervisor::kPassiveAxisCount;

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

/// The status rate of ROS 2 Interfaces §2, as the period the node steps at.
constexpr double kCycle = 1.0 / 20.0;

/// The four numbers `config/crane_supervisor.yaml` ships, written out here.
SwayBound shipped_bound()
{
  SwayBound bound;
  bound.dq_u_max = {{0.4, 0.4}};
  bound.dq_u_settled = {{0.04, 0.04}};
  bound.settled_release_factor = 1.5;
  bound.settle_dwell = 2.0;
  return bound;
}

/// Both coordinates at one rate.
std::array<double, kPassiveAxisCount> both(double rate)
{
  return {{rate, rate}};
}

/// One coordinate at `rate`, the other still.
std::array<double, kPassiveAxisCount> only(PassiveAxis axis, double rate)
{
  std::array<double, kPassiveAxisCount> rates{{0.0, 0.0}};
  rates[index_of(axis)] = rate;
  return rates;
}

/// A clock the tests advance by hand, so the dwell is an argument and not a wait.
class Sequence
{
public:
  explicit Sequence(SwayBound bound)
  : bound_(bound) {}

  /// One cycle at `rate` on both coordinates, the estimate trusted.
  SwaySettled step(double rate, int cycles = 1)
  {
    return step(both(rate), true, cycles);
  }

  /// One cycle of a named rate pair, trusted or not.
  SwaySettled step(
    const std::array<double, kPassiveAxisCount> & rates, bool trusted, int cycles = 1)
  {
    for (int i = 0; i < cycles; ++i) {
      now_ += kCycle;
      last_ = judge_sway(bound_, rates, trusted, now_, last_.state);
    }
    return last_.state.settled;
  }

  /// Drive the sequence until the predicate reads settled, or give up.
  bool settle(double rate = 0.0)
  {
    const int budget = 2 * static_cast<int>(bound_.settle_dwell / kCycle) + 4;
    for (int i = 0; i < budget; ++i) {
      if (step(rate) == SwaySettled::Settled) {
        return true;
      }
    }
    return false;
  }

  void skip(double seconds) {now_ += seconds;}

  [[nodiscard]] const SwayVerdict & verdict() const {return last_;}
  [[nodiscard]] double now() const {return now_;}

private:
  SwayBound bound_;
  double now_{1'700'000'000.0};
  SwayVerdict last_;
};

}  // namespace

TEST(SwayMonitor, TheShippedBoundsValidateAndTheAbsentOnesAreRefused)
{
  std::string reason;
  EXPECT_TRUE(crane_supervisor::validate_sway(shipped_bound(), reason)) << reason;

  EXPECT_FALSE(crane_supervisor::validate_sway(SwayBound{}, reason));
  EXPECT_FALSE(reason.empty());

  for (std::size_t i = 0; i < kPassiveAxisCount; ++i) {
    for (const double refused : {0.0, -1.0, kNaN, std::numeric_limits<double>::infinity()}) {
      SwayBound bound = shipped_bound();
      bound.dq_u_max[i] = refused;
      EXPECT_FALSE(crane_supervisor::validate_sway(bound, reason))
        << crane_supervisor::kPassiveAxisNames[i].joint;
      EXPECT_NE(reason.find(crane_supervisor::kPassiveAxisNames[i].joint), std::string::npos)
        << reason;

      bound = shipped_bound();
      bound.dq_u_settled[i] = refused;
      EXPECT_FALSE(crane_supervisor::validate_sway(bound, reason))
        << crane_supervisor::kPassiveAxisNames[i].joint;
    }

    SwayBound crossed = shipped_bound();
    crossed.dq_u_settled[i] = crossed.dq_u_max[i];
    EXPECT_FALSE(crane_supervisor::validate_sway(crossed, reason));
    crossed.dq_u_settled[i] = crossed.dq_u_max[i] * 2.0;
    EXPECT_FALSE(crane_supervisor::validate_sway(crossed, reason));
  }

  // Hysteresis the wrong way round chatters instead of damping.
  SwayBound inverted = shipped_bound();
  inverted.settled_release_factor = 0.9;
  EXPECT_FALSE(crane_supervisor::validate_sway(inverted, reason));
  inverted.settled_release_factor = 1.0;
  EXPECT_TRUE(crane_supervisor::validate_sway(inverted, reason)) << reason;

  SwayBound no_dwell = shipped_bound();
  no_dwell.settle_dwell = 0.0;
  EXPECT_FALSE(crane_supervisor::validate_sway(no_dwell, reason));
  no_dwell.settle_dwell = kNaN;
  EXPECT_FALSE(crane_supervisor::validate_sway(no_dwell, reason));
}

TEST(SwayMonitor, TheShippedSettleBoundClearsTheIdentifiedRateNoise)
{
  const std::array<double, kPassiveAxisCount> sigma{{std::sqrt(3.0e-5), std::sqrt(2.0e-5)}};
  const SwayBound bound = shipped_bound();
  for (std::size_t i = 0; i < kPassiveAxisCount; ++i) {
    EXPECT_GT(bound.dq_u_settled[i], 5.0 * sigma[i])
      << crane_supervisor::kPassiveAxisNames[i].label;
    EXPECT_GT(bound.dq_u_settled[i] * bound.settled_release_factor, 8.0 * sigma[i]);
  }

  const double half_period = M_PI / 2.0;
  EXPECT_GT(bound.settle_dwell, half_period);
}

TEST(SwayMonitor, ARatePastItsBoundIsAFaultThatNamesWhichCoordinateCrossedIt)
{
  const SwayBound bound = shipped_bound();

  // One hair under is not a fault; one hair over is.
  Sequence at_bound{bound};
  at_bound.step(only(PassiveAxis::Tip, bound.dq_u_max[index_of(PassiveAxis::Tip)]), true);
  EXPECT_TRUE(at_bound.verdict().breaches.empty());

  Sequence over{bound};
  over.step(only(PassiveAxis::Tip, 0.9), true);
  ASSERT_EQ(over.verdict().breaches.size(), 1u);
  EXPECT_EQ(over.verdict().breaches[0].axis, PassiveAxis::Tip);
  EXPECT_DOUBLE_EQ(over.verdict().breaches[0].dq_u, 0.9);
  EXPECT_DOUBLE_EQ(over.verdict().breaches[0].bound, bound.dq_u_max[index_of(PassiveAxis::Tip)]);

  const std::string message = crane_supervisor::sway_breach_message(over.verdict().breaches);
  EXPECT_NE(message.find("theta6_tip_joint"), std::string::npos) << message;
  EXPECT_NE(message.find("tip"), std::string::npos) << message;
  // The number, the bound it was compared against and the unit of both.
  EXPECT_NE(message.find("0.9000"), std::string::npos) << message;
  EXPECT_NE(message.find("0.4000"), std::string::npos) << message;
  EXPECT_NE(message.find("rad/s"), std::string::npos) << message;
  // And no other coordinate is named, because one that is fine is not a cause.
  EXPECT_EQ(message.find("theta7_tilt_joint"), std::string::npos) << message;
  EXPECT_NE(message.find("Nothing was stopped"), std::string::npos) << message;

  Sequence backwards{bound};
  backwards.step(only(PassiveAxis::Tilt, -0.7), true);
  ASSERT_EQ(backwards.verdict().breaches.size(), 1u);
  EXPECT_EQ(backwards.verdict().breaches[0].axis, PassiveAxis::Tilt);
  EXPECT_DOUBLE_EQ(backwards.verdict().breaches[0].dq_u, -0.7);
  EXPECT_NE(
    crane_supervisor::sway_breach_message(backwards.verdict().breaches).find("-0.7000"),
    std::string::npos);

  SwayBound uneven = bound;
  uneven.dq_u_max = {{0.4, 0.1}};
  Sequence pair{uneven};
  pair.step({{0.5, 0.5}}, true);
  ASSERT_EQ(pair.verdict().breaches.size(), 2u);
  EXPECT_EQ(pair.verdict().breaches[0].axis, PassiveAxis::Tilt);
  EXPECT_EQ(pair.verdict().breaches[1].axis, PassiveAxis::Tip);

  EXPECT_TRUE(crane_supervisor::sway_breach_message({}).empty());
}

TEST(SwayMonitor, TheSettledPredicateNeedsTheWholeDwellAndNotOneQuietCycle)
{
  const SwayBound bound = shipped_bound();
  Sequence still{bound};

  const int cycles_in_dwell = static_cast<int>(bound.settle_dwell / kCycle);
  for (int i = 0; i < cycles_in_dwell; ++i) {
    EXPECT_EQ(still.step(0.0), SwaySettled::NotSettled) << i;
  }
  still.step(0.0);
  EXPECT_EQ(still.step(0.0), SwaySettled::Settled);
  // And it stays settled while the crane stays still.
  EXPECT_EQ(still.step(0.0, 20), SwaySettled::Settled);
}

TEST(SwayMonitor, ASingleCrossingDoesNotChatterTheSignalAtTheStatusRate)
{
  const SwayBound bound = shipped_bound();
  const double settle = bound.dq_u_settled[0];

  Sequence run{bound};
  ASSERT_TRUE(run.settle());

  EXPECT_EQ(run.step(settle * 1.2), SwaySettled::Settled);
  EXPECT_EQ(run.step(0.0), SwaySettled::Settled);

  EXPECT_EQ(run.step(settle * 2.0), SwaySettled::NotSettled);
  const int half_the_dwell = static_cast<int>(bound.settle_dwell / kCycle) / 2;
  EXPECT_EQ(run.step(0.0, half_the_dwell), SwaySettled::NotSettled);
  EXPECT_TRUE(run.settle());

  Sequence at_bound{bound};
  EXPECT_TRUE(at_bound.settle(settle));

  Sequence just_over{bound};
  EXPECT_FALSE(just_over.settle(settle * 1.001));
}

TEST(SwayMonitor, ADistrustedEstimateIsUnknownAndNeverSettledAndRaisesNoSwayFault)
{
  const SwayBound bound = shipped_bound();

  Sequence run{bound};
  ASSERT_TRUE(run.settle());
  ASSERT_EQ(run.verdict().state.settled, SwaySettled::Settled);

  EXPECT_EQ(run.step(both(0.0), false), SwaySettled::Unknown);
  EXPECT_TRUE(run.verdict().breaches.empty());
  EXPECT_FALSE(run.verdict().estimate_read);

  EXPECT_EQ(run.step(both(9.0), false), SwaySettled::Unknown);
  EXPECT_TRUE(run.verdict().breaches.empty());

  // Coming back does not restore the old dwell: it starts a new one.
  EXPECT_EQ(run.step(0.0), SwaySettled::NotSettled);
  EXPECT_TRUE(run.settle());

  Sequence not_a_number{bound};
  ASSERT_TRUE(not_a_number.settle());
  EXPECT_EQ(not_a_number.step(both(kNaN), true), SwaySettled::Unknown);
  EXPECT_TRUE(not_a_number.verdict().breaches.empty());
  EXPECT_FALSE(not_a_number.verdict().estimate_read);

  Sequence half{bound};
  EXPECT_EQ(half.step({{0.0, kNaN}}, true), SwaySettled::Unknown);

  // And a caller with no clock reading cannot complete a dwell by accident.
  const SwayVerdict no_clock = judge_sway(bound, both(0.0), true, kNaN, SwayState{});
  EXPECT_EQ(no_clock.state.settled, SwaySettled::Unknown);
}

TEST(SwayMonitor, WithNoSettleBoundThePredicateIsUnknownRatherThanFalse)
{
  SwayBound bound = shipped_bound();
  bound.dq_u_settled = {{0.0, 0.0}};

  const SwayVerdict verdict = judge_sway(bound, both(0.0), true, 1.0, SwayState{});
  EXPECT_EQ(verdict.state.settled, SwaySettled::Unknown);
  EXPECT_TRUE(verdict.estimate_read);

  const std::string clause = crane_supervisor::settled_clause(bound, both(0.0), verdict);
  EXPECT_NE(clause.find("no settle bound is configured"), std::string::npos) << clause;
  EXPECT_EQ(clause.find("not usable"), std::string::npos) << clause;

  const SwayVerdict over = judge_sway(bound, both(9.0), true, 1.0, SwayState{});
  EXPECT_EQ(over.breaches.size(), kPassiveAxisCount);
}

TEST(SwayMonitor, AClockThatSteppedBackwardsRestartsTheDwellRatherThanCompletingIt)
{
  const SwayBound bound = shipped_bound();

  SwayState state;
  SwayVerdict verdict = judge_sway(bound, both(0.0), true, 100.0, state);
  EXPECT_EQ(verdict.state.settled, SwaySettled::NotSettled);
  EXPECT_DOUBLE_EQ(verdict.state.calm_since, 100.0);

  // Backwards, past the start of the run in progress.
  verdict = judge_sway(bound, both(0.0), true, 10.0, verdict.state);
  EXPECT_EQ(verdict.state.settled, SwaySettled::NotSettled);
  EXPECT_DOUBLE_EQ(verdict.state.calm_since, 10.0);

  verdict = judge_sway(bound, both(0.0), true, 10.0 + bound.settle_dwell, verdict.state);
  EXPECT_EQ(verdict.state.settled, SwaySettled::Settled);
}

TEST(SwayMonitor, TheSettledClauseSaysThePredicateAndTheNumbersBehindIt)
{
  const SwayBound bound = shipped_bound();

  Sequence run{bound};
  ASSERT_TRUE(run.settle());
  const std::string settled = crane_supervisor::settled_clause(bound, both(0.0), run.verdict());
  EXPECT_EQ(settled.rfind(crane_supervisor::kSettledClausePrefix, 0), 0u) << settled;
  EXPECT_NE(settled.find("settled"), std::string::npos) << settled;
  EXPECT_NE(settled.find("0.0400"), std::string::npos) << settled;
  EXPECT_NE(settled.find("2.000"), std::string::npos) << settled;

  run.step(0.5);
  const std::string moving = crane_supervisor::settled_clause(bound, both(0.5), run.verdict());
  EXPECT_NE(moving.find("not settled"), std::string::npos) << moving;
  EXPECT_NE(moving.find("0.5000"), std::string::npos) << moving;

  run.step(both(0.0), false);
  const std::string unknown = crane_supervisor::settled_clause(bound, both(0.0), run.verdict());
  EXPECT_NE(unknown.find("not known"), std::string::npos) << unknown;
  EXPECT_NE(settled, moving);
  EXPECT_NE(moving, unknown);
  EXPECT_NE(settled, unknown);

  for (const SwaySettled value :
    {SwaySettled::Unknown, SwaySettled::NotSettled, SwaySettled::Settled})
  {
    EXPECT_FALSE(std::string(crane_supervisor::settled_word(value)).empty());
  }
}

TEST(SwayMonitor, TheNamesAreIndexedByTheSameOrderTheEstimatePublishes)
{
  EXPECT_EQ(crane_supervisor::kPassiveAxisNames.size(), kPassiveAxisCount);
  EXPECT_EQ(index_of(PassiveAxis::Tip), 0u);
  EXPECT_EQ(index_of(PassiveAxis::Tilt), 1u);
  for (std::size_t i = 0; i < kPassiveAxisCount; ++i) {
    const auto & named = crane_supervisor::kPassiveAxisNames[i];
    EXPECT_EQ(index_of(named.axis), i);
    EXPECT_NE(std::string(named.label), "");
    EXPECT_NE(std::string(named.joint), "");
    for (std::size_t j = i + 1; j < kPassiveAxisCount; ++j) {
      EXPECT_NE(std::string(named.label), crane_supervisor::kPassiveAxisNames[j].label);
      EXPECT_NE(std::string(named.joint), crane_supervisor::kPassiveAxisNames[j].joint);
    }
  }
  EXPECT_EQ(std::string(crane_supervisor::name_of(PassiveAxis::Tip).joint), "theta6_tip_joint");
  EXPECT_EQ(std::string(crane_supervisor::name_of(PassiveAxis::Tilt).joint), "theta7_tilt_joint");
}
