
#ifndef CRANE_SUPERVISOR__SUPERVISOR_CORE_HPP_
#define CRANE_SUPERVISOR__SUPERVISOR_CORE_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "crane_supervisor/sway_monitor.hpp"

namespace crane_supervisor
{

/// Control mode, numbered as `crane_msgs/SupervisorStatus` numbers it.
enum class Mode : std::uint8_t
{
  Idle = 0,
  Manual = 1,
  Follow = 2,
  Mpc = 3,
};

/// How many modes there are. The bound of everything that is per mode.
inline constexpr std::size_t kModeCount = 4;

/// One mode as its own array index.
[[nodiscard]] inline constexpr std::size_t index_of(Mode mode) noexcept
{
  return static_cast<std::size_t>(mode);
}

/// True when a raw wire value names one of the four modes.
[[nodiscard]] bool is_mode(std::uint8_t value) noexcept;

/// The mode's constant name, as an operator reads it off a panel.
[[nodiscard]] const char * mode_name(Mode mode) noexcept;

/// Why the stack is not to be trusted, numbered as the message numbers it.
enum class Fault : std::uint8_t
{
  None = 0,
  Tracking = 1,
  WorkingCell = 2,
  Solver = 3,
  Sway = 4,
  StateHealth = 5,
  ReferenceStale = 6,
  EStop = 7,
  Interlock = 8,
  NotCommissioned = 9,
};

enum class Input : std::size_t
{
  PendulumState,
  RemoteCtrl,
  ControllerState,
  ControllerHealth,
  /// Not an input. The bound of everything that is per input.
  Count,
};

/// How many inputs there are, from the enum rather than beside it.
inline constexpr std::size_t kInputCount = static_cast<std::size_t>(Input::Count);

/// One enumerator as its own array index.
[[nodiscard]] inline constexpr std::size_t index_of(Input input) noexcept
{
  return static_cast<std::size_t>(input);
}

/// What the horizon producer's newest solve did, numbered as the wire numbers it.
enum class SolveOutcome : std::uint8_t
{
  Unknown = 0,
  Converged = 1,
  BudgetExceeded = 2,
  Failed = 3,
};

/// The outcome as an operator reads it off a panel.
[[nodiscard]] const char * solve_outcome_name(SolveOutcome outcome) noexcept;

/// Which staleness cause fired on one input this cycle.
enum class Staleness : std::uint8_t
{
  /// Arriving, inside its deadline, and the producer says nothing is wrong.
  Fresh = 0,
  /// Nothing has arrived on this input since the supervisor started.
  NeverArrived = 1,
  /// The newest sample is older than this input's deadline. It was arriving.
  StoppedArriving = 2,
  StampAhead = 3,
};

/// One input, and what its absence costs. One row per `Input`.
struct InputPolicy
{
  Input input;
  /// The input in an operator's words, used mid-sentence.
  const char * label;
  const char * topic;
  /// The message type, as an operator would grep for it.
  const char * type;
  /// The node or controller that is supposed to be publishing it.
  const char * producer;
  const char * stopped;
  /// What is lost, and what must not be inferred, while this input is missing.
  const char * consequence;
  /// What to check.
  const char * advice;
  Fault fault;
  /// Whether a fault this input raises survives the condition going away.
  bool latches;
};

/// The one row per input, in enum order.
inline constexpr std::array<InputPolicy, kInputCount> kInputPolicies{{
  {Input::PendulumState, "the passive joint state", "/joint_states",
    "sensor_msgs/JointState", "joint_state_broadcaster", "stopped arriving",
    "The passive joint state counts as unavailable, so nothing that closes on it may be trusted: "
    "a stale joint velocity makes the inner loop's integrator wind up against a value that is no "
    "longer true.",
    "Check that joint_state_broadcaster is loaded and active on the controller manager.",
    Fault::StateHealth, false},
  {Input::RemoteCtrl, "the operator remote", "/crane/remote_ctrl_states",
    "epsilon_crane_msgs/RemoteCtrlStates", "gpio_controller", "stopped arriving",
    "Absence of the stop signal is read as asserted rather than as released, and the deadman is "
    "reported as not held: a dead GPIO reader, a crashed driver and a released button must not "
    "look alike.",
    "Check that gpio_controller is loaded and active on the controller manager and that its "
    "remote_ctrl_states output reaches the contract name.",
    Fault::EStop, true},
  {Input::ControllerState, "the trajectory controller's own state", "/crane/controller_state",
    "control_msgs/JointTrajectoryControllerState", "the trajectory controller",
    "stopped publishing its own state",
    "The tracking error is not being measured, and tracking_error carries 0.0 as the absence of a "
    "measurement rather than as perfect tracking: a controller that died must not look like a "
    "crane that is tracking perfectly.",
    "Check the controller manager's cycle, whether the trajectory controller is still active, and "
    "that its private controller_state output reaches the contract name.",
    Fault::StateHealth, false},
  {Input::ControllerHealth, "the inner velocity loop's health report",
    "/crane/velocity_controller/health", "crane_msgs/VelocityControllerHealth",
    "crane_velocity_controller", "stopped reporting its own health",
    "The inner loop's verdict on itself is unread rather than clear, and a commissioning "
    "prerequisite it is raising would be reaching nobody -- which is the state that stream exists "
    "to end.",
    "Check that crane_velocity_controller is loaded and active on the controller manager.",
    Fault::StateHealth, false},
}};

/// The row of one input.
[[nodiscard]] inline constexpr const InputPolicy & policy_of(Input input) noexcept
{
  return kInputPolicies[index_of(input)];
}

/// True when every row sits at the index of the enumerator it names.
[[nodiscard]] inline constexpr bool policies_are_in_enum_order() noexcept
{
  for (std::size_t i = 0; i < kInputCount; ++i) {
    if (index_of(kInputPolicies[i].input) != i) {
      return false;
    }
  }
  return true;
}

static_assert(
  policies_are_in_enum_order(),
  "kInputPolicies has one row per Input, in enum order: an input added to the enum needs a row "
  "describing what its absence costs, and a row that names a different input than its index would "
  "report the wrong topic for the wrong stream");

inline constexpr int kFirstButton = 1;
inline constexpr int kLastButton = 12;

/// One axis's row of `crane_control/config/tracking_tolerance.yaml`.
struct AxisTolerance
{
  std::string joint;
  double dq_a{std::numeric_limits<double>::quiet_NaN()};
};

/// The margins the decision is made against. All SI, all named.
struct SupervisorConfig
{
  /// The freshness deadline of every input, s, indexed by `Input`.
  std::array<double, kInputCount> freshness_deadline{};

  double controller_manager_deadline{0.0};

  double horizon_deadline{0.0};

  std::string mpc_node;

  /// Which controllers hold the **arm claim** in each mode, indexed by `Mode`.
  std::array<std::vector<std::string>, kModeCount> mode_controllers;

  /// Which controllers hold the **tool claim**, in every mode.
  std::vector<std::string> tool_controllers;

  std::vector<AxisTolerance> tracking_tolerance;

  SwayBound sway;

  /// Which `epsilon_crane_msgs/RemoteCtrlStates` button is the deadman, 1..12.
  int deadman_button{12};

  /// The deadline of one input, s.
  [[nodiscard]] double deadline(Input input) const noexcept
  {
    return freshness_deadline[index_of(input)];
  }

  /// The deadline of one input, s, to be written by whoever loads the margins.
  [[nodiscard]] double & deadline(Input input) noexcept
  {
    return freshness_deadline[index_of(input)];
  }
};

/// The transport half of one input: what the node saw of the *topic*.
struct StreamReport
{
  bool received{false};
  double age{0.0};
};

/// One controller, as the controller manager described it.
struct ControllerReport
{
  std::string name;
  std::string state;
  /// The command interfaces this controller owns right now.
  std::vector<std::string> claimed_interfaces;
};

/// What the controller manager last said about its controllers.
struct ControllerManagerReport
{
  StreamReport answer;
  /// One entry per loaded controller, in the order the manager listed them.
  std::vector<ControllerReport> controllers;
};

/// What the horizon producer last said about itself, and how old that is.
struct HorizonReport
{
  /// The transport half, judged by the same `freshness_of()` every input is.
  StreamReport health;
  SolveOutcome outcome{SolveOutcome::Unknown};
  Fault fault{Fault::None};
  double solve_time{0.0};
  double solve_budget{0.0};
  bool applied_previous_solution{false};
  std::string status;
};

/// The passive pair off `/joint_states` this cycle.
/**
 * Freshness only, and no health: `sensor_msgs/JointState` carries no validity
 * flag, so a producer that has stopped publishing is the whole of what this
 * source can report about itself.
 */
struct PendulumStateReport
{
  /// The passive velocities, rad/s, `[tip, tilt]`.
  std::array<double, kPassiveAxisCount> velocity{
    {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()}};
};

/// What `/crane/remote_ctrl_states` said this cycle.
struct RemoteCtrlReport
{
  bool deadman_held{false};
  /// `epsilon_crane_msgs/RemoteCtrlStates.em_stop` of the newest message.
  bool em_stop{false};
};

/// One actuated axis, as the trajectory controller reported it this cycle.
struct AxisError
{
  /// The URDF joint name the controller published this row under.
  std::string joint;
  /// `error.positions`, rad or m. Feeds `tracking_error` and nothing else.
  double position_error{0.0};
  /// `error.velocities`, rad/s or m/s. What `FAULT_TRACKING` is decided on.
  double velocity_error{0.0};
  /// False when the controller published no velocity error for this axis.
  bool velocity_error_reported{false};
};

/// What the trajectory controller's state publication carried this cycle.
struct ControllerStateReport
{
  /// One entry per joint the newest message named.
  std::vector<AxisError> axes;
};

/// What `/crane/velocity_controller/health` carried this cycle.
struct ControllerHealthReport
{
  Fault fault{Fault::None};
  std::vector<std::string> feedforward_free_joints;
};

/// One axis whose velocity error is outside its own tolerance.
struct TrackingBreach
{
  std::string joint;
  double velocity_error{0.0};
  /// The `dq_a` this axis was compared against, same unit.
  double tolerance{0.0};
};

/// Everything one decision is made from.
struct SupervisorInput
{
  std::array<StreamReport, kInputCount> streams{};

  PendulumStateReport pendulum_state;
  RemoteCtrlReport remote_ctrl;
  ControllerStateReport controller_state;
  ControllerHealthReport controller_health;

  /// What the controller manager last said, and how old that answer is.
  ControllerManagerReport controller_manager;

  /// What the horizon producer last said about itself.
  HorizonReport horizon;

  /// The emergency-stop latch as the previous cycle left it.
  bool estop_latched{false};

  /// The sway dwell as the previous cycle left it.
  SwayState sway;

  /// The instant this observation was taken, on the node's own clock, s.
  double sampled_at{std::numeric_limits<double>::quiet_NaN()};

  /// The transport half of one input.
  [[nodiscard]] StreamReport & stream(Input input) noexcept {return streams[index_of(input)];}

  /// The transport half of one input.
  [[nodiscard]] const StreamReport & stream(Input input) const noexcept
  {
    return streams[index_of(input)];
  }
};

/// One decision, in the shape `crane_msgs/SupervisorStatus` carries it.
struct SupervisorDecision
{
  /// What is **active** right now, re-derived from the controller manager.
  Mode mode{Mode::Idle};
  Fault fault{Fault::StateHealth};
  double tracking_error{0.0};
  bool inside_working_cell{false};
  bool deadman_held{false};
  bool estop_latched{false};
  /// The settled predicate, and the dwell as this cycle leaves it.
  SwayState sway;
  /// Why, in words an operator can act on. Never empty (PRD user story 53).
  std::string message;
};

/// The answer to one `/crane/clear_fault` call.
struct ClearFaultOutcome
{
  bool cleared{false};
  std::string message;
};

/// Adopts and checks the margins. Returns false and says why, once, on failure.
[[nodiscard]] bool validate(const SupervisorConfig & config, std::string & reason);

/// The freshness of one input, from its deadline and what arrived.
[[nodiscard]] Staleness freshness_of(double deadline, const StreamReport & stream) noexcept;

/// The freshness of every input, in one pass.
[[nodiscard]] std::array<Staleness, kInputCount> freshness(
  const SupervisorConfig & config, const SupervisorInput & input) noexcept;

/// Which input, and which staleness cause, in words an operator can act on.
[[nodiscard]] std::string staleness_message(
  const SupervisorConfig & config, Input input, Staleness cause, const StreamReport & stream);

/// True when a number is a tolerance: finite and positive.
[[nodiscard]] bool is_tolerance(double dq_a) noexcept;

/// What to say once, at configuration, about the axes that have no tolerance.
[[nodiscard]] std::string tracking_tolerance_notice(const SupervisorConfig & config);

[[nodiscard]] double max_position_error(const ControllerStateReport & report) noexcept;

/// The comparison, per axis and in the tolerance's own unit. Worst first.
[[nodiscard]] std::vector<TrackingBreach> tracking_breaches(
  const SupervisorConfig & config, const ControllerStateReport & report);

/// One status cycle. Total: every input produces a decision with a cause.
[[nodiscard]] SupervisorDecision decide(
  const SupervisorConfig & config, const SupervisorInput & input);

/// One acknowledgement of the latched emergency stop, against the newest input.
[[nodiscard]] ClearFaultOutcome clear_fault(
  const SupervisorConfig & config, const SupervisorInput & input);

/// What the controller manager's answer says is active, per claim.
struct ActiveMode
{
  /// The mode whose controllers are **all** active, or `Idle` when none is.
  Mode mode{Mode::Idle};
  bool known{false};
  /// Which of the two it is, judged by the same rule every input is judged by.
  Staleness cause{Staleness::NeverArrived};
  /// The controllers of the arm claim that are active right now, named.
  std::vector<std::string> active_controllers;
  /// The controllers of the **tool claim** that are active right now (§7.3).
  std::vector<std::string> active_tool_controllers;
  /// Active controllers this supervisor's model does not account for.
  std::vector<std::string> unmodelled_claimants;
  /// True when the active controllers do not form exactly one configured mode.
  bool partial{false};
};

/// How the mode clause every report ends in opens.
inline constexpr char kModeClausePrefix[] = " Mode: ";

/// One `/crane/set_mode` decision, before anything has been switched.
struct ModeArbitration
{
  bool accepted{false};
  bool switch_required{false};
  /// The controllers to activate, in the order they must come up.
  std::vector<std::string> activate;
  std::vector<std::string> deactivate;
  ActiveMode active;
  bool horizon_producer_active{false};
  /// Why, in words an operator can act on. Never empty.
  std::string message;
};

/// What this supervisor did to the horizon producer as part of one switch.
struct ProducerSwitch
{
  bool attempted{false};
  /// Which mode was asked for: `true` is `active`.
  bool active{false};
  /// Whether the producer accepted it.
  bool accepted{false};
  std::string account;
};

/// What the controller manager's answer says is active.
[[nodiscard]] ActiveMode active_mode(
  const SupervisorConfig & config, const ControllerManagerReport & report);

/// Whether PRD §10 step 2's precondition holds, and which cause fired if not.
struct HorizonPrecondition
{
  bool fresh{false};
  Staleness cause{Staleness::NeverArrived};
};

[[nodiscard]] HorizonPrecondition horizon_precondition(
  const SupervisorConfig & config, const HorizonReport & report) noexcept;

/// PRD §10 step 2, on its own and ahead of everything else.
[[nodiscard]] std::string mpc_horizon_refusal(
  const SupervisorConfig & config, std::uint8_t requested, const SupervisorInput & input);

/// wiki/mpc.md §6's repeated-failure escalation, as the supervisor's own duty.
struct SolverHandback
{
  /// The claim is to be released, now.
  bool required{false};
  /// The whole account, ready to be a `message`. Empty when `required` is false.
  std::string message;
};

[[nodiscard]] SolverHandback solver_handback(
  const SupervisorConfig & config, const SupervisorInput & input, const ActiveMode & active);

/// The mode clause, for the end of any report. Never empty.
[[nodiscard]] std::string mode_clause(const SupervisorConfig & config, const ActiveMode & active);

/// One `/crane/set_mode` request, arbitrated. Nothing is switched here.
[[nodiscard]] ModeArbitration arbitrate_mode(
  const SupervisorConfig & config, std::uint8_t requested, const SupervisorInput & input);

/// The account of a switch that was issued, once the outcome is known.
[[nodiscard]] std::string mode_switch_message(
  const SupervisorConfig & config, Mode requested, const ModeArbitration & arbitration,
  bool switched, const std::string & carried, const ActiveMode & reached,
  const ProducerSwitch & producer);

}  // namespace crane_supervisor

#endif  // CRANE_SUPERVISOR__SUPERVISOR_CORE_HPP_
