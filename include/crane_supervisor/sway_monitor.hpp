
#ifndef CRANE_SUPERVISOR__SWAY_MONITOR_HPP_
#define CRANE_SUPERVISOR__SWAY_MONITOR_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace crane_supervisor
{

/// The two passive coordinates, in the index order the estimate publishes them.
enum class PassiveAxis : std::uint8_t
{
  Tip = 0,
  Tilt = 1,
  /// Not an axis. The bound of everything that is per passive coordinate.
  Count = 2,
};

/// How many passive coordinates there are, from the enum rather than beside it.
inline constexpr std::size_t kPassiveAxisCount = static_cast<std::size_t>(PassiveAxis::Count);

/// One passive enumerator as its own array index.
[[nodiscard]] inline constexpr std::size_t index_of(PassiveAxis axis) noexcept
{
  return static_cast<std::size_t>(axis);
}

/// One passive coordinate, as the machine names it.
struct PassiveAxisName
{
  PassiveAxis axis;
  const char * label;
  const char * joint;
};

/// The one row per passive coordinate, in enum order.
inline constexpr std::array<PassiveAxisName, kPassiveAxisCount> kPassiveAxisNames{{
  {PassiveAxis::Tip, "tip", "theta6_tip_joint"},
  {PassiveAxis::Tilt, "tilt", "theta7_tilt_joint"},
}};

/// True when every row sits at the index of the coordinate it names.
[[nodiscard]] inline constexpr bool passive_names_are_in_enum_order() noexcept
{
  for (std::size_t i = 0; i < kPassiveAxisCount; ++i) {
    if (index_of(kPassiveAxisNames[i].axis) != i) {
      return false;
    }
  }
  return true;
}

static_assert(
  passive_names_are_in_enum_order(),
  "kPassiveAxisNames is indexed by PassiveAxis, and the index is also the index into the published "
  "position and velocity arrays: a row at the wrong index would name the tilt coordinate while "
  "reporting the tip coordinate's rate");

/// The row of one passive coordinate.
[[nodiscard]] inline constexpr const PassiveAxisName & name_of(PassiveAxis axis) noexcept
{
  return kPassiveAxisNames[index_of(axis)];
}

/// Whether the passive state is settled, or whether that cannot be said at all.
enum class SwaySettled : std::uint8_t
{
  Unknown = 0,
  NotSettled = 1,
  Settled = 2,
};

/// The predicate in an operator's word. Never empty.
[[nodiscard]] const char * settled_word(SwaySettled settled) noexcept;

/// The bounds the sway duty is decided against. All SI, all per coordinate.
struct SwayBound
{
  /// The rate past which `FAULT_SWAY` is raised, rad/s, indexed by `PassiveAxis`.
  std::array<double, kPassiveAxisCount> dq_u_max{};
  std::array<double, kPassiveAxisCount> dq_u_settled{};
  double settled_release_factor{0.0};
  double settle_dwell{0.0};
};

/// One passive coordinate whose rate is past `dq_u_max`.
struct SwayBreach
{
  PassiveAxis axis{PassiveAxis::Tip};
  double dq_u{0.0};
  /// The `dq_u_max` this coordinate was compared against, rad/s.
  double bound{0.0};
};

/// The dwell, as one cycle hands it to the next.
struct SwayState
{
  /// The predicate as the previous cycle left it.
  SwaySettled settled{SwaySettled::Unknown};
  double calm_since{std::numeric_limits<double>::quiet_NaN()};
};

/// One cycle's answer about the sway.
struct SwayVerdict
{
  std::vector<SwayBreach> breaches;
  /// The dwell as this cycle leaves it. The caller carries it into the next.
  SwayState state;
  /// Whether two finite rates were actually read this cycle.
  bool estimate_read{false};
};

/// True when a number is a bound: finite and positive.
[[nodiscard]] bool is_bound(double value) noexcept;

/// Adopts and checks the sway bounds. Returns false and says why, once.
[[nodiscard]] bool validate_sway(const SwayBound & bound, std::string & reason);

/// One cycle of the sway duty. Total: every input produces a verdict.
[[nodiscard]] SwayVerdict judge_sway(
  const SwayBound & bound, const std::array<double, kPassiveAxisCount> & dq_u,
  bool estimate_trusted, double sampled_at, const SwayState & previous);

/// The whole `FAULT_SWAY` report, naming which coordinate exceeded which bound.
[[nodiscard]] std::string sway_breach_message(const std::vector<SwayBreach> & breaches);

/// How the settled clause every report ends in opens.
inline constexpr char kSettledClausePrefix[] = " Sway: ";

/// The settled clause, for the end of any report.
[[nodiscard]] std::string settled_clause(
  const SwayBound & bound, const std::array<double, kPassiveAxisCount> & dq_u,
  const SwayVerdict & verdict);

}  // namespace crane_supervisor

#endif  // CRANE_SUPERVISOR__SWAY_MONITOR_HPP_
