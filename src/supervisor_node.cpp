#include "crane_supervisor/supervisor_node.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "crane_supervisor/crane_supervisor_parameters.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"

namespace crane_supervisor
{
namespace
{

/// Seconds as text, to the millisecond, for the one line logged at startup.
std::string seconds_text(double seconds)
{
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.3f", seconds);
  return std::string(buffer);
}

/// Every input and the deadline it is judged against, as one sentence.
/**
 * Built from the registry rather than written out, so the line an operator finds
 * in the log names the inputs the node actually holds -- including one added
 * after this line was written.
 */
std::string deadline_summary(const SupervisorConfig & config)
{
  std::string text;
  for (std::size_t i = 0; i < kInputCount; ++i) {
    if (i > 0) {
      text += (i + 1 == kInputCount) ? " and " : ", ";
    }
    text += std::string(kInputPolicies[i].topic) + " within " +
      seconds_text(config.freshness_deadline[i]) + " s";
  }
  return text;
}

/// The settled clause of one report, which is the sentence that report ends in.
/**
 * The seam between the two streams, and the whole of why they cannot disagree:
 * the clause on `/crane/sway_settled` is not composed a second time here, it is
 * the bytes `decide()` already appended to the status report. Nothing about the
 * predicate is recomputed in this adapter -- the verdict, the dwell and the
 * sentence are all the core's.
 *
 * `rfind` and not `find`: a cause message is free to mention the sway in its own
 * words, and the clause is appended last, so the last occurrence is the clause
 * and it runs to the end of the report.
 */
std::string settled_clause_of(const std::string & report)
{
  const std::size_t start = report.rfind(kSettledClausePrefix);
  if (start == std::string::npos) {
    // Unreachable through `decide()`, which appends the clause to every report
    // whatever its fault is. The report itself is the fallback rather than an
    // empty string, because ROS 2 Interfaces §1 admits no empty explanation.
    return report;
  }
  // The clause opens with the space that separated it from the sentence before
  // it. That space belongs to the report, not to the clause.
  return report.substr(start + 1);
}

/// Seconds as a `std::chrono` duration, for the two blocking waits.
std::chrono::nanoseconds budget(double seconds)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(seconds));
}

}  // namespace

bool deadman_of(const epsilon_crane_msgs::msg::RemoteCtrlStates & message, int button)
{
  // Twelve named booleans and no array, so this is the switch the wire forces.
  // The default is `false` rather than an exception: `validate()` has already
  // refused a number outside the range at construction, and a deadman that read
  // as released would be the safe answer if it ever got past it.
  switch (button) {
    case 1: return message.button1;
    case 2: return message.button2;
    case 3: return message.button3;
    case 4: return message.button4;
    case 5: return message.button5;
    case 6: return message.button6;
    case 7: return message.button7;
    case 8: return message.button8;
    case 9: return message.button9;
    case 10: return message.button10;
    case 11: return message.button11;
    case 12: return message.button12;
    default: return false;
  }
}

std::chrono::nanoseconds SupervisorNode::status_period()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / kStatusRate));
}

SupervisorNode::SupervisorNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("crane_supervisor", options)
{
  const ParamListener listener(this);
  const auto parameters = listener.get_params();

  // One margin onto the input it belongs to. This is the one hand-written
  // mapping in the freshness path and it is guarded from both sides: an input
  // added to `Input` with no line here keeps the zero its slot is
  // value-initialised with, `validate()` refuses a deadline of zero, and the
  // node throws below rather than publishing a status about a stream nobody is
  // watching (wiki/control_architecture.md §5.3).
  config_.deadline(Input::PendulumState) = parameters.pendulum_state_timeout;
  config_.deadline(Input::RemoteCtrl) = parameters.remote_ctrl_timeout;
  config_.deadline(Input::ControllerState) = parameters.controller_state_timeout;
  config_.deadline(Input::ControllerHealth) = parameters.controller_health_timeout;
  config_.controller_manager_deadline = parameters.controller_manager_timeout;
  // The horizon producer's margin, and the node whose mode this supervisor
  // drives. Both are the mode arbitration's and neither is swept on the status
  // cycle: `HorizonReport` in the core says why this stream is not an `Input`.
  config_.horizon_deadline = parameters.horizon_timeout;
  config_.mpc_node = parameters.mpc_node;
  config_.deadman_button = static_cast<int>(parameters.deadman_button);

  // Which controller holds the claim in each mode, and which holds the tool
  // claim. Names and not types: the controller manager loads by name, a profile
  // chooses the names, and this supervisor arbitrates between whatever it was
  // told. `MODE_IDLE` gets no list -- it is the absence of a motion claim -- and
  // `validate()` refuses one that has been given controllers.
  config_.mode_controllers[index_of(Mode::Manual)] = parameters.mode_controllers.manual;
  config_.mode_controllers[index_of(Mode::Follow)] = parameters.mode_controllers.follow;
  config_.mode_controllers[index_of(Mode::Mpc)] = parameters.mode_controllers.mpc;
  config_.tool_controllers = parameters.tool_controllers;

  // The sway duty's bounds, in the order `PassiveAxis` fixes -- which is also
  // the order `crane_msgs/PendulumState` publishes its two arrays in, so nothing
  // on this path reorders anything. Every one of them is a design value and
  // `src/crane_supervisor_parameters.yaml` says so beside each, with what would
  // replace it; `validate()` below refuses a deployment that is missing one.
  // A block that arrived short leaves the slots it did not fill at the zero they
  // are value-initialised with, and zero is not a bound: the declared
  // `fixed_size<>` refuses that first, and if it ever did not, `validate()` names
  // the coordinate rather than this loop reading past the end of a vector.
  for (std::size_t i = 0; i < kPassiveAxisCount; ++i) {
    if (i < parameters.sway.dq_u_max.size()) {
      config_.sway.dq_u_max[i] = parameters.sway.dq_u_max[i];
    }
    if (i < parameters.sway.dq_u_settled.size()) {
      config_.sway.dq_u_settled[i] = parameters.sway.dq_u_settled[i];
    }
  }
  config_.sway.settled_release_factor = parameters.sway.settled_release_factor;
  config_.sway.settle_dwell = parameters.sway.settle_dwell;

  // The six numbers arrive from `crane_control/config/tracking_tolerance.yaml`
  // -- the file's node key is the wildcard on purpose, so the seam clamp, the
  // MPC's constraint margin and this supervisor read the same rows out of the
  // same file. Nothing is restated here: the declared default is NaN, which is
  // the absence of a number rather than a number that happens to be small, and
  // a profile that loads the file for one consumer and not another is a visible
  // omission rather than a silent disagreement.
  config_.tracking_tolerance.clear();
  config_.tracking_tolerance.reserve(parameters.joints.size());
  for (const std::string & joint : parameters.joints) {
    config_.tracking_tolerance.push_back(
      AxisTolerance{joint, parameters.tracking_tolerance.joints_map.at(joint).dq_a});
  }

  // generate_parameter_library has already rejected a value outside the
  // declared bounds. The core is checked against its own rule anyway: it is the
  // ROS-free half, it is what a later configuration path will be checked
  // against, and a margin that reached `decide()` unchecked would be a silent
  // failure of exactly the kind Style Guide §4 exists to stop. It is also what
  // refuses an input that was given no deadline at all.
  std::string reason;
  if (!validate(config_, reason)) {
    throw std::runtime_error("crane_supervisor: " + reason);
  }

  // Once, at configuration, and never again: an axis with no tolerance raises no
  // tracking fault, and a supervisor that stayed quiet about it would be
  // reporting FAULT_NONE for a check it is not making. It is a warning and not a
  // refusal -- the number is human-owned and does not exist yet, and taking the
  // whole status stream down over it would withhold four duties to report one.
  const std::string notice = tracking_tolerance_notice(config_);
  if (!notice.empty()) {
    RCLCPP_WARN(get_logger(), "crane_supervisor: %s", notice.c_str());
  }

  status_publisher_ =
    create_publisher<crane_msgs::msg::SupervisorStatus>(kStatusTopic, contract_qos());

  // The settled predicate, on a stream of its own because `SupervisorStatus` has
  // no field for it and widening a frozen message is a slice of its own (PRD
  // §15). Same QoS as the status and published from the same cycle: a task layer
  // that branches on the field and an operator reading the sentence are looking
  // at one decision.
  sway_settled_publisher_ =
    create_publisher<crane_msgs::msg::SwaySettled>(kSwaySettledTopic, contract_qos());

  // Every subscription below goes through `subscribe()` and names its `Input`,
  // which is what gives it a freshness deadline and a policy row to be reported
  // from. None of them takes a topic name of its own.
  pendulum_state_subscription_ = subscribe<crane_msgs::msg::PendulumState>(
    Input::PendulumState,
    [this](crane_msgs::msg::PendulumState::ConstSharedPtr message) {
      pendulum_state_ = std::move(message);
    });

  remote_ctrl_subscription_ = subscribe<epsilon_crane_msgs::msg::RemoteCtrlStates>(
    Input::RemoteCtrl,
    [this](epsilon_crane_msgs::msg::RemoteCtrlStates::ConstSharedPtr message) {
      remote_ctrl_ = std::move(message);
    });

  // The third input, and the one that makes the tracking duty of §5 row 1 a
  // typed cause. The controller that computes the error is the one that
  // publishes it, so nothing is re-derived here from `/joint_states`.
  controller_state_subscription_ = subscribe<control_msgs::msg::JointTrajectoryControllerState>(
    Input::ControllerState,
    [this](control_msgs::msg::JointTrajectoryControllerState::ConstSharedPtr message) {
      controller_state_ = std::move(message);
    });

  // The fourth input, and the only one this supervisor could not re-derive from
  // anything else on the graph: which axes the active tool has an identified
  // valve map for is known to the inner velocity loop alone. It computes the
  // fault every cycle and publishes it at the rate of the consumers, of which
  // this node is one.
  controller_health_subscription_ = subscribe<crane_msgs::msg::VelocityControllerHealth>(
    Input::ControllerHealth,
    [this](crane_msgs::msg::VelocityControllerHealth::ConstSharedPtr message) {
      controller_health_ = std::move(message);
    });

  // The horizon producer's own stream, and the one subscription in this package
  // that does not go through `subscribe()`. It cannot: that helper takes an
  // `Input`, and an `Input` is a stream whose absence raises a fault on the
  // status. This one's must not -- an optimizer that is quiet while the machine
  // is in MODE_FOLLOW is the ordinary state of this stack, and ROS 2 Interfaces
  // §4 makes merging FAULT_SOLVER onto the status a slice of its own -- so it is
  // held to §5.3's rule the way the polled controller-manager view is: a defined
  // consequence at the point it matters, which is that PRD §10 step 2 refuses
  // MODE_MPC and says how old the newest report is and what the deadline was.
  solver_health_subscription_ = create_subscription<crane_msgs::msg::SolverHealth>(
    kSolverHealthTopic, contract_qos(),
    [this](crane_msgs::msg::SolverHealth::ConstSharedPtr message) {
      solver_health_ = std::move(message);
    });

  // The other half of the structural guard. `validate()` refuses an input with
  // no deadline; this refuses one with no subscription, which would otherwise be
  // reported as never having arrived for the lifetime of the process -- a defect
  // in this node dressed up as a fault in the graph.
  for (std::size_t i = 0; i < kInputCount; ++i) {
    if (!claimed_[i]) {
      throw std::runtime_error(
        std::string("crane_supervisor: ") + kInputPolicies[i].label + " (" +
        kInputPolicies[i].topic +
        ") is an input of this supervisor with no subscription behind it. Every enumerator of "
        "Input is subscribed through subscribe(), or the node does not start: an input nobody "
        "reads would be reported as never having arrived for ever.");
    }
  }

  // The acknowledgement of ROS 2 Interfaces §5, and the only thing in this
  // package a caller can ask for. It lowers a latch and does nothing else: it
  // starts nothing, resumes nothing and commands nothing, and refusing new goals
  // while the stop is latched is a mode decision this supervisor does not yet
  // have the authority to make.
  clear_fault_service_ = create_service<std_srvs::srv::Trigger>(
    kClearFaultService,
    [this](
      const std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response) {
      const ClearFaultOutcome outcome = clear_fault(config_, observe());
      if (outcome.cleared) {
        estop_latched_ = false;
      }
      // ROS 2 Interfaces §1: `success` and an explanation, and never an empty
      // success. `success` is true only when a latch existed and is now down,
      // so a caller cannot read an acknowledgement of nothing as a recovery.
      response->success = outcome.cleared;
      response->message = outcome.message;
      RCLCPP_INFO(get_logger(), "%s: %s", kClearFaultService, outcome.message.c_str());
    });

  // The arbitration of ROS 2 Interfaces §5, and the only way a mode changes.
  // It is served on the default callback group, so the outer executor answers
  // it; the blocking calls it makes go out on the private group below.
  set_mode_service_ = create_service<crane_msgs::srv::SetMode>(
    kSetModeService,
    [this](
      const crane_msgs::srv::SetMode::Request::SharedPtr request,
      crane_msgs::srv::SetMode::Response::SharedPtr response) {
      set_mode(request, response);
    });

  // The exceptions to this package's absence of a path to the machine, and the
  // reason they are safe to have: the group is never added to the executor that
  // spins this node, so the clients below are served only by the private
  // executor, only from the two places that spin it.
  remote_call_group_ =
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  remote_call_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  remote_call_executor_->add_callback_group(remote_call_group_, get_node_base_interface());
  list_controllers_ = create_client<controller_manager_msgs::srv::ListControllers>(
    kListControllersService, rmw_qos_profile_services_default, remote_call_group_);
  switch_controller_ = create_client<controller_manager_msgs::srv::SwitchController>(
    kSwitchControllerService, rmw_qos_profile_services_default, remote_call_group_);
  // The third, and only on a deployment that named a producer. Which node that
  // is is a profile's decision, so the service name is composed here rather
  // than written out; the parameter it carries and the two values it may take
  // are `crane_mpc`'s own contract and are constants (ROS 2 Interfaces §4, "One
  // command path": which path is live is this supervisor's decision alone).
  if (!config_.mpc_node.empty()) {
    set_horizon_producer_mode_ = create_client<rcl_interfaces::srv::SetParameters>(
      "/" + config_.mpc_node + kSetParametersSuffix, rmw_qos_profile_services_default,
      remote_call_group_);
  }

  // The freshness sweep runs from here and from nowhere else. A deadline
  // evaluated in a subscription callback could not fire on the stream that
  // stopped, which is the only stream it exists for.
  status_timer_ = create_wall_timer(status_period(), [this]() {update();});

  RCLCPP_INFO(
    get_logger(),
    "crane_supervisor: publishing %s and %s at %.1f Hz -- the second carries the settled predicate "
    "as a field, because the first is frozen and has no room for one. Every input it holds has a "
    "freshness deadline of its own and none is exempt -- %s -- and an input that stops arriving is "
    "reported as a fault "
    "rather than left at FAULT_NONE. Button %d of the remote is the deadman. It serves %s and %s. "
    "The mode it reports is re-derived from %s every cycle and never remembered, and %s is the one "
    "call this package makes that reaches the machine -- narrowly: it decides which controller "
    "holds the claim and carries no setpoint. Beside it, and only on a deployment that names a "
    "horizon producer, one parameter call moves that producer between shadow and active, because "
    "which of the two paths is live is this supervisor's decision alone and the two never drive at "
    "once. It holds no stop authority: the hardware stop input "
    "is an unverified commissioning prerequisite, so releasing a claim on MODE_IDLE is a mode "
    "change somebody asked for and not a way to bring the machine to rest, and what it does with "
    "the emergency stop is diagnosis and recovery rather than protection.",
    kStatusTopic, kSwaySettledTopic, kStatusRate, deadline_summary(config_).c_str(),
    config_.deadman_button,
    kClearFaultService, kSetModeService, kListControllersService, kSwitchControllerService);
}

SupervisorInput SupervisorNode::observe() const
{
  SupervisorInput input;
  input.estop_latched = estop_latched_;
  input.sway = sway_state_;

  // One reading of the clock for all of them, so two inputs sampled in the same
  // cycle are aged against the same instant. `now() - header.stamp` is the age
  // the publisher of that header intends a consumer to compute: every producer
  // here stamps from the control cycle's own time on the same clock, so the two
  // sides agree about what time it is without either saying so on the wire.
  const rclcpp::Time sampled_at = now();
  // The same reading, in the seconds the ROS-free core measures its dwell in. It
  // is this node's clock and not a message stamp, deliberately: the dwell is how
  // long *this supervisor* has been watching a calm crane, and a producer whose
  // stamps stopped advancing must not be able to complete one.
  input.sampled_at = sampled_at.seconds();
  const auto age_of = [&sampled_at](const auto & message) {
      return (sampled_at - rclcpp::Time(message->header.stamp)).seconds();
    };

  // The transport half of every input, in one loop over the registry, so no
  // input is aged by a rule of its own. An input added to `Input` without a row
  // here reads as never having arrived -- a standing fault, which is the safe
  // direction for the omission to fail in, and the constructor's claim check
  // catches the omission that produces it.
  const std::array<bool, kInputCount> arrived{{
    static_cast<bool>(pendulum_state_), static_cast<bool>(remote_ctrl_),
    static_cast<bool>(controller_state_), static_cast<bool>(controller_health_)}};
  const std::array<double, kInputCount> age{{
    pendulum_state_ ? age_of(pendulum_state_) : 0.0,
    remote_ctrl_ ? age_of(remote_ctrl_) : 0.0,
    controller_state_ ? age_of(controller_state_) : 0.0,
    controller_health_ ? age_of(controller_health_) : 0.0}};
  for (std::size_t i = 0; i < kInputCount; ++i) {
    input.streams[i].received = arrived[i];
    input.streams[i].age = age[i];
  }

  if (pendulum_state_) {
    input.pendulum_state.valid = pendulum_state_->valid;
    input.pendulum_state.status = pendulum_state_->status;
    // The rate, in the `[tip, tilt]` order both ends already agree on. Neither
    // `position` nor `velocity_covariance` is read: the angle carries an
    // uncalibrated constant offset and the covariance is the identified noise of
    // the sensor pair rather than a statement about this cycle
    // (crane_supervisor/sway_monitor.hpp).
    input.pendulum_state.velocity = pendulum_state_->velocity;
  }

  if (remote_ctrl_) {
    input.remote_ctrl.deadman_held = deadman_of(*remote_ctrl_, config_.deadman_button);
    input.remote_ctrl.em_stop = remote_ctrl_->em_stop;
  }

  if (controller_state_) {
    const auto & message = *controller_state_;
    input.controller_state.axes.reserve(message.joint_names.size());
    for (std::size_t i = 0; i < message.joint_names.size(); ++i) {
      AxisError axis;
      axis.joint = message.joint_names[i];
      if (i < message.error.positions.size()) {
        axis.position_error = message.error.positions[i];
      }
      // Only when the controller actually published one. It fills
      // `error.velocities` only with a velocity state interface and a velocity
      // or effort command interface, and an absent field read as a zero error
      // would be a crane that tracks perfectly by construction.
      if (i < message.error.velocities.size()) {
        axis.velocity_error = message.error.velocities[i];
        axis.velocity_error_reported = true;
      }
      input.controller_state.axes.push_back(std::move(axis));
    }
  }

  input.controller_manager = observe_controller_manager();

  // The horizon producer, aged on the same reading of the clock the four inputs
  // are, against its own deadline rather than any of theirs. It is read here
  // and consulted in exactly one place -- PRD §10 step 2 -- so a request
  // arbitrated from this observation is judged against the same producer the
  // status cycle saw.
  if (solver_health_) {
    const auto & message = *solver_health_;
    input.horizon.health.received = true;
    input.horizon.health.age = age_of(solver_health_);
    // A cast and not a lookup table, the way the mode and the fault are: the
    // core's `SolveOutcome` and the message's `SOLVE_*` constants share one
    // numbering, and `test_contract.cpp` asserts every pair of them.
    input.horizon.outcome = static_cast<SolveOutcome>(message.outcome);
    input.horizon.fault = static_cast<Fault>(message.fault);
    input.horizon.solve_time = message.solve_time;
    input.horizon.solve_budget = message.solve_budget;
    input.horizon.applied_previous_solution = message.applied_previous_solution;
    input.horizon.status = message.message;
  }

  if (controller_health_) {
    const auto & message = *controller_health_;
    // A cast and not a lookup table: the message and the core's enum share one
    // numbering, and `test_contract.cpp` asserts every pair of them. A value
    // outside the ten constants stays what the controller sent rather than being
    // folded into one of them -- see `ControllerHealthReport::fault`.
    input.controller_health.fault = static_cast<Fault>(message.fault);
    // An axis the controller named and did not apply the feedforward to. The
    // name is taken off the wire rather than off this node's own `joints`: the
    // sixth valve channel drives a different joint per tool, and the controller
    // is the one that knows which tool it is driving.
    for (std::size_t i = 0; i < message.joint_names.size(); ++i) {
      if (!message.joint_names[i].empty() && !message.feedforward_applied[i]) {
        input.controller_health.feedforward_free_joints.push_back(message.joint_names[i]);
      }
    }
  }

  return input;
}

ControllerManagerReport SupervisorNode::observe_controller_manager() const
{
  ControllerManagerReport report;
  if (!controllers_) {
    return report;
  }
  report.answer.received = true;
  // Arrival and not a stamp: `ListControllers` carries no header, so the only
  // thing this answer can be aged against is when this node heard it. That is a
  // weaker measurement than the four streams' and it is weaker in the safe
  // direction -- it cannot be fooled by a producer whose clock ran ahead.
  report.answer.age = (now() - controllers_at_).seconds();
  report.controllers.reserve(controllers_->controller.size());
  for (const auto & controller : controllers_->controller) {
    ControllerReport entry;
    entry.name = controller.name;
    entry.state = controller.state;
    entry.claimed_interfaces = controller.claimed_interfaces;
    report.controllers.push_back(std::move(entry));
  }
  return report;
}

void SupervisorNode::poll_controller_manager()
{
  // The private group's only non-blocking spin. Anything the two clients have
  // waiting is dispatched here, on the status cycle, on the outer executor's
  // own thread.
  remote_call_executor_->spin_some();

  if (pending_snapshot_.has_value()) {
    if (
      pending_snapshot_->future.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
      controllers_ = pending_snapshot_->future.get();
      controllers_at_ = now();
      pending_snapshot_.reset();
      return;
    }
    // A request that outlived its own deadline is dropped rather than waited
    // on. Left in flight it would block every later poll, and the snapshot
    // would then age out and read as "not known" for a reason that is this
    // node's rather than the manager's.
    if ((now() - pending_snapshot_->sent_at).seconds() > config_.controller_manager_deadline) {
      list_controllers_->remove_pending_request(pending_snapshot_->request_id);
      pending_snapshot_.reset();
    }
    return;
  }

  if (!list_controllers_->service_is_ready()) {
    return;
  }
  auto pending = list_controllers_->async_send_request(
    std::make_shared<controller_manager_msgs::srv::ListControllers::Request>());
  const std::int64_t request_id = pending.request_id;
  pending_snapshot_ = PendingSnapshot{pending.future.share(), request_id, now()};
}

bool SupervisorNode::refresh_controller_manager(double timeout)
{
  // A poll in flight is abandoned first: its answer is older than the one about
  // to be asked for, and letting it land afterwards would replace a fresh view
  // of the machine with a stale one at the worst possible moment.
  if (pending_snapshot_.has_value()) {
    list_controllers_->remove_pending_request(pending_snapshot_->request_id);
    pending_snapshot_.reset();
  }
  if (!list_controllers_->service_is_ready()) {
    return false;
  }

  auto pending = list_controllers_->async_send_request(
    std::make_shared<controller_manager_msgs::srv::ListControllers::Request>());
  const std::int64_t request_id = pending.request_id;
  auto future = pending.future.share();
  if (
    remote_call_executor_->spin_until_future_complete(future, budget(timeout)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    list_controllers_->remove_pending_request(request_id);
    // The snapshot is left exactly as it was. An old answer is aged and
    // reported as old; it is never replaced by a guess.
    return false;
  }
  controllers_ = future.get();
  controllers_at_ = now();
  return true;
}

bool SupervisorNode::switch_controllers(
  const std::vector<std::string> & activate, const std::vector<std::string> & deactivate,
  std::string & account)
{
  if (!switch_controller_->service_is_ready()) {
    account =
      "the controller manager's switch service is not reachable from this node, so the switch was "
      "never issued and nothing was deactivated";
    return false;
  }

  auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  request->activate_controllers = activate;
  request->deactivate_controllers = deactivate;
  // Strict, and both lists in one request: a best-effort switch that activated
  // half of a mode would leave the machine in a state no mode names, and two
  // requests would put it through one. §7's exclusion by resource claim is what
  // makes the single strict call the right shape -- the manager refuses the
  // whole thing rather than letting two claimants meet.
  request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
  request->activate_asap = true;
  request->timeout = rclcpp::Duration::from_seconds(kSwitchControllerTimeout);

  auto pending = switch_controller_->async_send_request(request);
  const std::int64_t request_id = pending.request_id;
  auto future = pending.future.share();
  if (
    remote_call_executor_->spin_until_future_complete(future, budget(kSwitchBudget)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    switch_controller_->remove_pending_request(request_id);
    account =
      "the call did not come back inside this node's own budget, so whether the manager performed "
      "the switch is unknown here -- which is why active_mode below is read back off the manager "
      "rather than assumed";
    return false;
  }
  const auto response = future.get();
  account = response->message;
  return response->ok;
}

ProducerSwitch SupervisorNode::set_horizon_producer_mode(bool active)
{
  ProducerSwitch outcome;
  outcome.active = active;
  if (!set_horizon_producer_mode_) {
    // A composition with no horizon producer. `validate()` has already refused
    // one that implements MODE_MPC and names none, so this is a deployment
    // where the mode is unreachable anyway and there is nothing to report.
    return outcome;
  }
  outcome.attempted = true;

  if (!set_horizon_producer_mode_->service_is_ready()) {
    outcome.account =
      "the producer's parameter service is not reachable from this node, so its mode was never "
      "moved -- check that the profile composes it under the name mpc_node gives";
    return outcome;
  }

  auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
  rcl_interfaces::msg::Parameter parameter;
  parameter.name = kHorizonProducerModeParameter;
  parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
  parameter.value.string_value = active ? kHorizonProducerActive : kHorizonProducerShadow;
  // One parameter and never more. The producer's own declaration validates the
  // string against its two values, so a third would be refused there rather
  // than accepted and quietly ignored here.
  request->parameters.push_back(std::move(parameter));

  auto pending = set_horizon_producer_mode_->async_send_request(request);
  const std::int64_t request_id = pending.request_id;
  auto future = pending.future.share();
  if (
    remote_call_executor_->spin_until_future_complete(future, budget(kProducerModeBudget)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    set_horizon_producer_mode_->remove_pending_request(request_id);
    outcome.account =
      "the call did not come back inside this node's own budget, so which mode the producer is in "
      "is unknown here";
    return outcome;
  }

  const auto answer = future.get();
  if (answer->results.empty()) {
    outcome.account =
      "the producer answered with no result for the one parameter that was sent, which is itself a "
      "defect: rcl_interfaces/SetParameters carries one per parameter";
    return outcome;
  }
  outcome.accepted = answer->results.front().successful;
  outcome.account = answer->results.front().reason;
  return outcome;
}

void SupervisorNode::set_mode(
  const crane_msgs::srv::SetMode::Request::SharedPtr request,
  crane_msgs::srv::SetMode::Response::SharedPtr response)
{
  // PRD §10 step 2, and **the order is the assertion** (user story 35). The
  // horizon's freshness is verified before this node asks the controller
  // manager for anything at all -- not before the switch, before the *poll* --
  // so a switch into a dead MPC is refused from the mode the machine is already
  // in and costs the manager nothing. `arbitrate_mode()` runs the same function
  // again at precondition 2, off the same observation, so the early answer here
  // and the arbitration below cannot disagree about what was verified.
  //
  // The mode reported on a refusal comes off the polled snapshot rather than a
  // fresh one, deliberately: it is judged by the same deadline every other
  // answer is, so a snapshot too old to believe reads as *not known* instead of
  // being replaced by a call this branch exists not to make.
  const std::string refusal = mpc_horizon_refusal(config_, request->mode, observe());
  if (!refusal.empty()) {
    const ActiveMode active = active_mode(config_, observe_controller_manager());
    response->success = false;
    response->message = refusal + mode_clause(config_, active);
    response->active_mode = static_cast<std::uint8_t>(active.mode);
    RCLCPP_WARN(get_logger(), "%s: %s", kSetModeService, response->message.c_str());
    return;
  }

  // Truth about the machine, taken now. Every remaining precondition is checked
  // against this snapshot, before anything is deactivated: a switch that cannot
  // succeed is refused from the mode the machine is already in. A stale
  // snapshot would make `arbitrate_mode()` refuse for want of a view, which is
  // the safe direction for this failure to fall in.
  refresh_controller_manager(kControllerManagerCallBudget);

  const ModeArbitration arbitration = arbitrate_mode(config_, request->mode, observe());
  if (!arbitration.accepted || !arbitration.switch_required) {
    // ROS 2 Interfaces §1: `success` and an explanation, never an empty
    // success. `active_mode` is what is active, which after a refusal is the
    // mode the machine was already in.
    response->success = arbitration.accepted;
    response->message = arbitration.message + mode_clause(config_, arbitration.active);
    response->active_mode = static_cast<std::uint8_t>(arbitration.active.mode);
    if (arbitration.accepted) {
      RCLCPP_INFO(get_logger(), "%s: %s", kSetModeService, response->message.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "%s: %s", kSetModeService, response->message.c_str());
    }
    return;
  }

  const Mode requested = static_cast<Mode>(request->mode);

  // Into MODE_MPC the producer goes active **before** the claim moves, so that
  // a horizon is already fitted when the trajectory controller lets go. It
  // cannot steal the command in the meantime: the velocity controller takes the
  // chained path whenever a trajectory controller is chained onto it, and the
  // freshness this switch was admitted on is the producer's own statement that
  // it is solving.
  ProducerSwitch producer;
  if (arbitration.horizon_producer_active) {
    producer = set_horizon_producer_mode(true);
  }

  std::string account;
  const bool switched = switch_controllers(arbitration.activate, arbitration.deactivate, account);

  // Read back, never assumed. `active_mode` on the response is what is actually
  // active after the call -- that is the whole reason the mode is re-derived at
  // all, and a switch the manager accepted can still land somewhere else.
  refresh_controller_manager(kControllerManagerCallBudget);
  const ActiveMode reached = active_mode(config_, observe_controller_manager());

  // Out of MODE_MPC the producer goes back to shadow **after** the claim has
  // moved, for the same reason it goes active before: an inner loop that is
  // unchained with no horizon ramps its command to zero.
  //
  // It is settled from the mode that was **read back** and not from the one
  // that was asked for, which is what makes this self-correcting: a switch into
  // MODE_MPC that the manager refused leaves the producer active over a claim
  // that never moved, and this puts it back. The two branches are exclusive --
  // the one above only fires for a request into MODE_MPC, this one only when
  // the machine did not end up there -- so no request makes two calls.
  const bool mpc_holds_the_claim = reached.known && !reached.partial && reached.mode == Mode::Mpc;
  if (!mpc_holds_the_claim && (arbitration.horizon_producer_active || switched)) {
    producer = set_horizon_producer_mode(false);
  }

  response->success = switched && reached.known && !reached.partial && reached.mode == requested;
  response->message =
    mode_switch_message(config_, requested, arbitration, switched, account, reached, producer);
  response->active_mode = static_cast<std::uint8_t>(reached.mode);
  if (response->success) {
    RCLCPP_INFO(get_logger(), "%s: %s", kSetModeService, response->message.c_str());
  } else {
    RCLCPP_WARN(get_logger(), "%s: %s", kSetModeService, response->message.c_str());
  }
}

void SupervisorNode::update()
{
  // Before the decision, so that the mode on this report is re-derived from the
  // newest answer the manager has given rather than from the one before it.
  poll_controller_manager();

  // The observation is held rather than handed straight to `decide()`: the
  // settled stream carries the two rates the predicate was decided from, and
  // they have to be the rates *this* decision saw. Reading the held message a
  // second time would publish a rate that arrived after the verdict.
  const SupervisorInput observed = observe();
  const SupervisorDecision decision = decide(config_, observed);
  // The two things a cycle carries into the next one. `decide()` raises the
  // latch and only an acknowledged `/crane/clear_fault` lowers it; the sway dwell
  // is advanced on every cycle whatever the report said, so that a fault
  // somewhere else does not restart it.
  estop_latched_ = decision.estop_latched;
  sway_state_ = decision.sway;

  // One reading of the clock for both publications. They are two streams of one
  // decision, so a consumer pairs them by stamp; two readings would put the same
  // cycle on the wire under two different times.
  const rclcpp::Time stamp = now();

  crane_msgs::msg::SupervisorStatus status;
  status.header.stamp = stamp;
  // Status data has no geometric expression frame, so the header value is the
  // empty string rather than an invented one (ROS 2 Interfaces §1, §4).
  status.header.frame_id = "";
  status.mode = static_cast<std::uint8_t>(decision.mode);
  status.fault = static_cast<std::uint8_t>(decision.fault);
  status.tracking_error = decision.tracking_error;
  status.inside_working_cell = decision.inside_working_cell;
  status.deadman_held = decision.deadman_held;
  status.message = decision.message;
  status_publisher_->publish(status);

  // The same verdict, as a field a behaviour tree can branch on rather than an
  // English clause it would have to parse (wiki/control_architecture.md §5 row
  // 7). Four assignments and no decision: `settled` is the value the core
  // decided, `velocity` is what it decided it from, and `message` is the clause
  // the report above already carries -- so the stream and the sentence agree by
  // construction rather than by two functions being kept in step.
  crane_msgs::msg::SwaySettled settled;
  settled.header.stamp = stamp;
  settled.header.frame_id = "";
  // A cast and not a lookup table, the way the mode and the fault are: the core's
  // `SwaySettled` and the message's `SETTLED_*` constants share one numbering,
  // and `test_contract.cpp` asserts every pair of them.
  settled.settled = static_cast<std::uint8_t>(decision.sway.settled);
  settled.velocity = observed.pendulum_state.velocity;
  settled.message = settled_clause_of(decision.message);
  sway_settled_publisher_->publish(settled);

  // On the transition only. The stream already carries the cause twenty times a
  // second; what a log adds is the moment it changed, and repeating it every
  // cycle would bury that moment in its own copies.
  if (!ever_reported_ || decision.fault != reported_fault_) {
    ever_reported_ = true;
    reported_fault_ = decision.fault;
    if (decision.fault == Fault::None) {
      RCLCPP_INFO(get_logger(), "%s", decision.message.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "%s", decision.message.c_str());
    }
  }

  // wiki/control_architecture.md §5 row 3's second half, and the last thing this
  // cycle does. Both streams have already gone out carrying FAULT_SOLVER and the
  // account of why, so the report an operator reads arrives *before* the claim
  // moves rather than after it; the calls below block this timer for as long as
  // a `/crane/set_mode` request does, and the mode reads MODE_IDLE on the next
  // cycle.
  //
  // The active mode is re-derived from the same held snapshot `decide()` used --
  // no service call, no second view -- so the predicate and the report cannot
  // disagree about which mode the machine was in when the escalation was seen.
  const SolverHandback handback =
    solver_handback(config_, observed, active_mode(config_, observe_controller_manager()));
  if (!handback.required) {
    handback_attempted_ = false;
    return;
  }
  if (handback_attempted_) {
    return;
  }
  handback_attempted_ = true;
  hand_back_from_mpc(handback);
}

std::string SupervisorNode::hand_back_from_mpc(const SolverHandback & handback)
{
  // Truth about the machine, taken now, exactly as `set_mode()` takes it before
  // it arbitrates. The held snapshot was good enough to decide *that* the claim
  // must be released; it is not good enough to plan the release from, because a
  // plan is built against controller states and those may have moved since the
  // last poll.
  refresh_controller_manager(kControllerManagerCallBudget);

  // Through `arbitrate_mode()` and not around it. Every precondition an
  // operator's MODE_IDLE request passes is one this release passes too -- that
  // the view is known, that the controllers are loaded and switchable, that the
  // plan never names a controller the incoming mode also wants -- so there is
  // one set of rules for the claim and not a second, quieter one for the
  // supervisor's own stop. The latch does not refuse this direction: releasing
  // the claim is the one thing a latched stop does not argue against.
  const ModeArbitration arbitration =
    arbitrate_mode(config_, static_cast<std::uint8_t>(Mode::Idle), observe());
  if (!arbitration.accepted || !arbitration.switch_required) {
    const std::string account = handback.message + " " + arbitration.message;
    RCLCPP_ERROR(get_logger(), "%s", account.c_str());
    return account;
  }

  std::string carried;
  const bool switched = switch_controllers(arbitration.activate, arbitration.deactivate, carried);

  // Read back, never assumed, for the reason `set_mode()` reads it back: a
  // switch the manager accepted can still land somewhere else, and the producer
  // is settled from where the claim actually is.
  refresh_controller_manager(kControllerManagerCallBudget);
  const ActiveMode reached = active_mode(config_, observe_controller_manager());

  // Out of MODE_MPC the producer goes back to shadow **after** the claim has
  // moved (PRD §10). The condition is the same one `set_mode()` uses and for the
  // same reason: it is settled off the mode that was read back, so a release the
  // manager refused leaves the producer where the claim still is.
  ProducerSwitch producer;
  const bool mpc_holds_the_claim = reached.known && !reached.partial && reached.mode == Mode::Mpc;
  if (!mpc_holds_the_claim) {
    producer = set_horizon_producer_mode(false);
  }

  const std::string account = handback.message + " " +
    mode_switch_message(config_, Mode::Idle, arbitration, switched, carried, reached, producer);
  if (switched && !mpc_holds_the_claim) {
    RCLCPP_WARN(get_logger(), "%s", account.c_str());
  } else {
    RCLCPP_ERROR(get_logger(), "%s", account.c_str());
  }
  return account;
}

}  // namespace crane_supervisor
