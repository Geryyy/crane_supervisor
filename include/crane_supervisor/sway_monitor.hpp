// The sway duty of wiki/control_architecture.md §5 row 7, separated from
// everything that made it observable. Two rates in, a fault and a predicate out:
// no node handle, no message type, no clock, no topic
// (wiki/implementation/style_guide.md §3).
//
// The duty is stated narrowly in §5 and stays that way here. The supervisor
// **refuses to start a motion that depends on sway being settled, and reports**.
// Nothing below damps anything, nothing below stops a motion already running for
// being swingy, and nothing below refuses anything either -- this supervisor has
// no authority over a mode until `/crane/set_mode` exists, and active damping is
// slice 6. What is produced is a typed cause and a predicate somebody else acts
// on.
//
// # Why the test is on the rate and on nothing else
//
// Two independent reasons, and either one alone would be enough:
//
//   * **The angle carries an uncalibrated constant offset and the rate does
//     not.** `pendulum_state_broadcaster` reads the two passive coordinates out
//     on the *nominal* hinge axes, because the calibrated 2-D spline that
//     corrects the real double hinge's non-orthogonality needs calibration data
//     this workspace does not carry, and `theta7_tilt_joint`'s limits are not
//     centred on zero either. The rate is a composition of two gyro readings
//     through the known chain rather than a derivative of the angle, so a
//     constant offset in the angle does not reach it. A settled test built on
//     the angle would be a test against an offset nobody has measured.
//   * **It is the quantity the estimate is best conditioned on.** The published
//     rate is de-biased by a filter state that is *bounded* at
//     `filter.max_gyro_bias`, so at most that much real rate can ever be removed
//     from it, and the measured run-to-run bias of the differenced pair is an
//     order below that bound.
//
// # Why the covariance is not consulted
//
// `crane_msgs/PendulumState` carries a `velocity_covariance`, and this file
// deliberately does not read it. The block is the *identified noise of the
// differenced gyro pair* -- a property of the sensors, published whenever the
// estimate is trusted and withdrawn to -1 in every element when it was never
// identified for the hardware in question. It is therefore not a running
// statement about how uncertain *this cycle's* rate is, and weighting a bound
// with it would produce a confidence-weighted test whose confidence never moves.
// What the number is legitimately good for is fixing the floor a *design*
// threshold has to clear, and that use is in `crane_supervisor_parameters.yaml`,
// once, as a derivation rather than as a runtime read.
//
// # Three states, and the third one is the point
//
// An estimate that is absent, stale or marked unusable makes "settled"
// **unknowable**, and unknowable must not read as settled: a grip action gated
// on a two-valued predicate would descend onto a swinging block the moment the
// bracketing IMU stopped answering. So `SwaySettled` has three values, `Unknown`
// is what a distrusted estimate produces, and a distrusted estimate raises
// `FAULT_STATE_HEALTH` rather than `FAULT_SWAY` -- a sensor that stopped saying
// anything is a different fact from a load that is swinging, and an operator
// does different things about them.

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
/**
 * `crane_msgs/PendulumState`'s `position` and `velocity` are both `[tip, tilt]`
 * -- q5 then q6 of wiki/nomenclature.md §4.1, which is the order
 * `pendulum_state_broadcaster` configures its `joints` in. The enumerator's
 * value *is* the index into those arrays, so nothing here reorders anything.
 */
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
/**
 * Two names and not one: `label` is what an operator reads mid-sentence, and
 * `joint` is the URDF name of ROS 2 Interfaces §3.1, which is what somebody
 * chasing the report greps for. Both are properties of the machine -- the
 * passive set is fixed at $n_\text{u} = 2$ -- so they are constants here for the
 * same reason the actuated set is a fixed-size parameter and not a free list.
 */
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
/**
 * **Three values, and the third is not a convenience.** wiki/control_architecture.md
 * §5.3's "Degraded sway estimate" row asks for a motion that depends on sway
 * feedback to be refused when the estimate is degraded, and a grip action gated
 * on a boolean cannot express that: an absent, stale or unusable estimate would
 * have to be flattened into one of the two, and "not settled" blocks a grip
 * forever while "settled" descends onto a swinging block. `Unknown` is the state
 * that says the question was not answerable this cycle.
 *
 * The numbering is this package's own and reaches no wire, because there is no
 * wire for it yet -- see the README.
 */
enum class SwaySettled : std::uint8_t
{
  /// The estimate was not trusted this cycle, or no bound is configured to judge
  /// it against. Not settled, not unsettled: unknowable.
  Unknown = 0,
  /// Trusted, and at least one coordinate's rate is outside the settle bound --
  /// or the dwell that proves a swing was not caught at a turning point has not
  /// elapsed yet.
  NotSettled = 1,
  /// Trusted, and both rates have stayed inside the settle bound for the whole
  /// dwell.
  Settled = 2,
};

/// The predicate in an operator's word. Never empty.
[[nodiscard]] const char * settled_word(SwaySettled settled) noexcept;

/// The bounds the sway duty is decided against. All SI, all per coordinate.
/**
 * **Value-initialised to zero, and zero is not a bound.** `validate_sway()`
 * refuses it and the node throws rather than publishing, for the same reason a
 * freshness deadline has no plausible default: the numbers and the derivation of
 * every one of them live once, in `src/crane_supervisor_parameters.yaml`, and a
 * struct default beside them would be a second place for them to drift.
 *
 * Every one of them is a **design** value. What would replace each is named in
 * that file, beside the number.
 */
struct SwayBound
{
  /// The rate past which `FAULT_SWAY` is raised, rad/s, indexed by `PassiveAxis`.
  std::array<double, kPassiveAxisCount> dq_u_max{};
  /// The rate at or below which a coordinate counts as calm, rad/s, indexed by
  /// `PassiveAxis`. Strictly below `dq_u_max`, which `validate_sway()` enforces:
  /// a settle bound at or above the fault bound would call a crane settled in
  /// the same cycle it reported the swing.
  std::array<double, kPassiveAxisCount> dq_u_settled{};
  /// How much higher the rate has to climb to *leave* the settled state than to
  /// enter it, as a factor on `dq_u_settled`. Dimensionless and at least 1.0.
  /**
   * The hysteresis half of the anti-chatter rule. The dwell below stops the
   * predicate flickering on the way in; this stops one noise sample past the
   * bound flickering it on the way out, which would otherwise cost a whole dwell
   * to recover from.
   */
  double settled_release_factor{0.0};
  /// How long every coordinate has to stay calm before the predicate reads
  /// `Settled`, s.
  double settle_dwell{0.0};
};

/// One passive coordinate whose rate is past `dq_u_max`.
struct SwayBreach
{
  PassiveAxis axis{PassiveAxis::Tip};
  /// Signed, rad/s. Signed because which way the load is going is the first
  /// thing anyone looking at a sway fault wants to know.
  double dq_u{0.0};
  /// The `dq_u_max` this coordinate was compared against, rad/s.
  double bound{0.0};
};

/// The dwell, as one cycle hands it to the next.
/**
 * The core is a pure function and the dwell is state, so the state is threaded
 * through rather than held -- exactly as the emergency-stop latch is. That keeps
 * every case reachable from a test by writing two members, instead of by
 * replaying the history that produced them.
 */
struct SwayState
{
  /// The predicate as the previous cycle left it.
  SwaySettled settled{SwaySettled::Unknown};
  /// The instant the current unbroken run of calm cycles began, on the caller's
  /// own clock, s. NaN when no run is in progress, which is also what a cycle
  /// that broke one leaves behind.
  double calm_since{std::numeric_limits<double>::quiet_NaN()};
};

/// One cycle's answer about the sway.
struct SwayVerdict
{
  /// The coordinates past `dq_u_max`, worst first. Empty when none is, and
  /// always empty while the estimate is not trusted: a rate nobody can place is
  /// not evidence of a swinging load.
  std::vector<SwayBreach> breaches;
  /// The dwell as this cycle leaves it. The caller carries it into the next.
  SwayState state;
  /// Whether two finite rates were actually read this cycle.
  /**
   * `Unknown` has two causes and they are not the same thing to chase: the
   * estimate was not trusted, or it was and no bound was configured to judge it
   * against. This is what tells them apart in the report, so a misconfigured
   * supervisor does not blame the sensor.
   */
  bool estimate_read{false};
};

/// True when a number is a bound: finite and positive.
/**
 * The same rule `is_tolerance()` applies to `dq_a`, and for the same reason: a
 * zero bound would report every cycle over it and a bound that is not a number
 * would report none of them, and neither is a comparison.
 */
[[nodiscard]] bool is_bound(double value) noexcept;

/// Adopts and checks the sway bounds. Returns false and says why, once.
/**
 * Unlike the tracking tolerance -- which does not exist yet, is human-owned and
 * is therefore *reported* rather than refused -- every number here is shipped
 * with the package. A deployment that is missing one has been misconfigured, not
 * left waiting on a campaign, so it is refused: a supervisor that quietly stopped
 * judging the sway would report `FAULT_NONE` for a duty it was not performing.
 */
[[nodiscard]] bool validate_sway(const SwayBound & bound, std::string & reason);

/// One cycle of the sway duty. Total: every input produces a verdict.
/**
 * `estimate_trusted` is the caller's answer to whether `/crane/pendulum_state`
 * arrived inside its deadline *and* the broadcaster marks the sample usable. It
 * is passed in rather than re-derived because the freshness policy belongs to
 * the registry in `supervisor_core.hpp`, and there must be exactly one place
 * that decides whether an input is to be believed.
 *
 * `sampled_at` is the caller's clock, s, read once for the whole cycle. A value
 * earlier than the run in progress -- a clock that stepped backwards -- restarts
 * the dwell rather than completing it early.
 */
[[nodiscard]] SwayVerdict judge_sway(
  const SwayBound & bound, const std::array<double, kPassiveAxisCount> & dq_u,
  bool estimate_trusted, double sampled_at, const SwayState & previous);

/// The whole `FAULT_SWAY` report, naming which coordinate exceeded which bound.
/**
 * Empty over an empty breach list, so a caller cannot compose a fault message
 * for a cycle that has no fault in it.
 */
[[nodiscard]] std::string sway_breach_message(const std::vector<SwayBreach> & breaches);

/// How the settled clause every report ends in opens.
/**
 * Exported so that a test which has to look at the sentence *before* it can find
 * the seam without writing the literal out a second time. Every report carries
 * the clause, whatever `fault` says, for the reason `deadman_held` is filled on
 * every report: the predicate has no field of its own, so a cycle that dropped
 * it would drop it exactly when a more consequential cause was in the way.
 */
inline constexpr char kSettledClausePrefix[] = " Sway: ";

/// The settled clause, for the end of any report.
/**
 * Names the predicate, the two rates it was decided from and the bound they were
 * compared against, so that a bag carries the numbers and not only the verdict.
 */
[[nodiscard]] std::string settled_clause(
  const SwayBound & bound, const std::array<double, kPassiveAxisCount> & dq_u,
  const SwayVerdict & verdict);

}  // namespace crane_supervisor

#endif  // CRANE_SUPERVISOR__SWAY_MONITOR_HPP_
