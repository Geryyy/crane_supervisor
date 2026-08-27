#include "crane_supervisor/sway_monitor.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <string>
#include <vector>

namespace crane_supervisor
{
namespace
{

constexpr char kSwayHead[] = "sway: the passive joint rate is past its bound on ";

constexpr char kSwayTail[] =
  ". The bound is on the *rate* and not on the angle, deliberately: the published angle is read "
  "out on the nominal hinge axes and carries an uncalibrated constant offset, while the rate is a "
  "composition of the two gyro readings through the known chain and does not "
  "(wiki/system.md Measurement). Nothing was stopped, ramped, damped or commanded here -- active "
  "damping is a later slice, and this supervisor has no authority to refuse a motion until it owns "
  "a mode. What this report is for is the task layer's decision not to *start* one that depends on "
  "the load hanging still.";

constexpr char kSettledExplained[] =
  " -- both passive rates have stayed inside the settle bound for the whole dwell, which is longer "
  "than half the pendulum's own period, so a swing cannot have been caught at a turning point.";

constexpr char kNotSettledExplained[] =
  " -- the dwell that proves a swing was not caught at a turning point has not elapsed with both "
  "rates inside the settle bound.";

constexpr char kUnknownExplained[] =
  " -- the passive state estimate was not usable this cycle, so whether the load is settled is not "
  "answerable and is deliberately not reported as settled. The cause is the state-health report "
  "above or on an earlier cycle; a sensor that stopped saying anything is a different fact from a "
  "load that is swinging.";

constexpr char kNoBoundExplained[] =
  " -- the estimate was usable and no settle bound is configured to judge it against, which is a "
  "defect in this supervisor's configuration and not in the estimate. A node started through its "
  "own parameters cannot reach this: validate_sway() refuses a supervisor with no bound before it "
  "publishes anything.";

constexpr char kRatesHead[] = " Rates this cycle: ";

constexpr char kUnboundedRatesHead[] = " Rates this cycle, judged against nothing: ";

constexpr char kNoRates[] = " No rate this cycle could be placed against a bound.";

std::string quantity_text(double value)
{
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.4f", value);
  return std::string(buffer);
}

std::string seconds_text(double seconds)
{
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.3f", seconds);
  return std::string(buffer);
}

/// The rate a coordinate has to stay inside to count as calm this cycle.
double calm_threshold(const SwayBound & bound, std::size_t axis, bool was_settled)
{
  const double entering = bound.dq_u_settled[axis];
  return was_settled ? entering * bound.settled_release_factor : entering;
}

/// True when every element of a block is a bound.
bool every_element_is_a_bound(const std::array<double, kPassiveAxisCount> & block)
{
  for (std::size_t i = 0; i < kPassiveAxisCount; ++i) {
    if (!is_bound(block[i])) {
      return false;
    }
  }
  return true;
}

/// One breached coordinate, in the operator's terms and with its unit.
std::string breach_text(const SwayBreach & breach)
{
  const PassiveAxisName & named = name_of(breach.axis);
  return std::string(named.label) + " (" + named.joint + ", rate " + quantity_text(breach.dq_u) +
         " rad/s, bound " + quantity_text(breach.bound) + " rad/s)";
}

}  // namespace

const char * settled_word(SwaySettled settled) noexcept
{
  switch (settled) {
    case SwaySettled::Settled:
      return "settled";
    case SwaySettled::NotSettled:
      return "not settled";
    case SwaySettled::Unknown:
      break;
  }
  return "not known";
}

bool is_bound(double value) noexcept
{
  return std::isfinite(value) && value > 0.0;
}

bool validate_sway(const SwayBound & bound, std::string & reason)
{
  for (std::size_t i = 0; i < kPassiveAxisCount; ++i) {
    const PassiveAxisName & named = kPassiveAxisNames[i];
    if (!is_bound(bound.dq_u_max[i])) {
      reason = std::string("the sway rate bound of the ") + named.label + " coordinate (" +
        named.joint + ") is " + quantity_text(bound.dq_u_max[i]) +
        " rad/s, and a bound is a finite positive rate: zero would report every cycle as sway and "
        "one that is not a number would report none of them, which would leave the sway duty of "
        "wiki/control_architecture.md 5 with nothing to judge against at all. The numbers and "
        "their derivations are in src/crane_supervisor_parameters.yaml.";
      return false;
    }
    if (!is_bound(bound.dq_u_settled[i])) {
      reason = std::string("the settle bound of the ") + named.label + " coordinate (" +
        named.joint + ") is " + quantity_text(bound.dq_u_settled[i]) +
        " rad/s, and a bound is a finite positive rate. Without one the settled predicate can "
        "never be anything but unknown, and a grip action gated on it would never run.";
      return false;
    }
    if (bound.dq_u_settled[i] >= bound.dq_u_max[i]) {
      reason = std::string("the settle bound of the ") + named.label + " coordinate is " +
        quantity_text(bound.dq_u_settled[i]) + " rad/s and its fault bound is " +
        quantity_text(bound.dq_u_max[i]) +
        " rad/s, so the settle bound is not below the fault bound. It has to be: otherwise the "
        "same cycle would report the load as swinging past its bound and as settled.";
      return false;
    }
  }
  if (!std::isfinite(bound.settled_release_factor) || bound.settled_release_factor < 1.0) {
    reason = "the settled release factor is " + quantity_text(bound.settled_release_factor) +
      ", and it is a factor on the settle bound that has to be at least 1.0: below one the "
      "predicate would be harder to hold than to reach, which is hysteresis the wrong way round "
      "and chatters instead of damping the signal.";
    return false;
  }
  if (!std::isfinite(bound.settle_dwell) || bound.settle_dwell <= 0.0) {
    reason = "the settle dwell is " + seconds_text(bound.settle_dwell) +
      " s, and it is a finite positive time. A dwell of zero would call the load settled in the "
      "single cycle its rate happened to cross zero, which for a pendulum is every half period.";
    return false;
  }
  return true;
}

SwayVerdict judge_sway(
  const SwayBound & bound, const std::array<double, kPassiveAxisCount> & dq_u,
  bool estimate_trusted, double sampled_at, const SwayState & previous)
{
  SwayVerdict verdict;

  const bool readable = estimate_trusted && std::isfinite(sampled_at) &&
    std::isfinite(dq_u[index_of(PassiveAxis::Tip)]) &&
    std::isfinite(dq_u[index_of(PassiveAxis::Tilt)]);
  if (!readable) {
    return verdict;
  }
  verdict.estimate_read = true;

  for (std::size_t i = 0; i < kPassiveAxisCount; ++i) {
    if (is_bound(bound.dq_u_max[i]) && std::abs(dq_u[i]) > bound.dq_u_max[i]) {
      verdict.breaches.push_back({static_cast<PassiveAxis>(i), dq_u[i], bound.dq_u_max[i]});
    }
  }
  std::stable_sort(
    verdict.breaches.begin(), verdict.breaches.end(),
    [](const SwayBreach & left, const SwayBreach & right) {
      return std::abs(left.dq_u) / left.bound > std::abs(right.dq_u) / right.bound;
    });

  if (!every_element_is_a_bound(bound.dq_u_settled) ||
    !std::isfinite(bound.settled_release_factor) || bound.settled_release_factor < 1.0 ||
    !std::isfinite(bound.settle_dwell) || bound.settle_dwell < 0.0)
  {
    return verdict;
  }

  const bool was_settled = previous.settled == SwaySettled::Settled;
  bool calm = true;
  for (std::size_t i = 0; i < kPassiveAxisCount; ++i) {
    if (std::abs(dq_u[i]) > calm_threshold(bound, i, was_settled)) {
      calm = false;
    }
  }

  if (!calm) {
    verdict.state.settled = SwaySettled::NotSettled;
    return verdict;
  }

  double calm_since = previous.calm_since;
  if (!std::isfinite(calm_since) || sampled_at < calm_since) {
    calm_since = sampled_at;
  }
  verdict.state.calm_since = calm_since;
  verdict.state.settled = (was_settled || sampled_at - calm_since >= bound.settle_dwell)
    ? SwaySettled::Settled
    : SwaySettled::NotSettled;
  return verdict;
}

std::string sway_breach_message(const std::vector<SwayBreach> & breaches)
{
  if (breaches.empty()) {
    return {};
  }
  std::string text = kSwayHead;
  for (std::size_t i = 0; i < breaches.size(); ++i) {
    if (i > 0) {
      text += (i + 1 == breaches.size()) ? " and " : ", ";
    }
    text += breach_text(breaches[i]);
  }
  return text + kSwayTail;
}

std::string settled_clause(
  const SwayBound & bound, const std::array<double, kPassiveAxisCount> & dq_u,
  const SwayVerdict & verdict)
{
  std::string text = std::string(kSettledClausePrefix) + settled_word(verdict.state.settled);

  if (verdict.state.settled == SwaySettled::Unknown) {
    if (!verdict.estimate_read) {
      return text + kUnknownExplained + kNoRates;
    }
    text += kNoBoundExplained;
    text += kUnboundedRatesHead;
    for (std::size_t i = 0; i < kPassiveAxisCount; ++i) {
      if (i > 0) {
        text += ", ";
      }
      text += std::string(kPassiveAxisNames[i].label) + " " + quantity_text(dq_u[i]);
    }
    return text + " rad/s.";
  }

  const bool settled = verdict.state.settled == SwaySettled::Settled;
  text += settled ? kSettledExplained : kNotSettledExplained;
  text += kRatesHead;
  for (std::size_t i = 0; i < kPassiveAxisCount; ++i) {
    if (i > 0) {
      text += ", ";
    }
    text += std::string(kPassiveAxisNames[i].label) + " " + quantity_text(dq_u[i]) +
      " against a settle bound of " + quantity_text(bound.dq_u_settled[i]);
  }
  return text + " rad/s, over a dwell of " + seconds_text(bound.settle_dwell) + " s.";
}

}  // namespace crane_supervisor
