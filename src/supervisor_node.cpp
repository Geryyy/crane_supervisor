#include "crane_supervisor/supervisor_node.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <iterator>
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
std::string settled_clause_of(const std::string & report)
{
  const std::size_t start = report.rfind(kSettledClausePrefix);
  if (start == std::string::npos) {
    return report;
  }
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

  config_.deadline(Input::PendulumState) = parameters.pendulum_state_timeout;
  config_.deadline(Input::RemoteCtrl) = parameters.remote_ctrl_timeout;
  config_.deadline(Input::ControllerState) = parameters.controller_state_timeout;
  config_.deadline(Input::ControllerHealth) = parameters.controller_health_timeout;
  config_.controller_manager_deadline = parameters.controller_manager_timeout;
  config_.horizon_deadline = parameters.horizon_timeout;
  config_.mpc_node = parameters.mpc_node;
  config_.deadman_button = static_cast<int>(parameters.deadman_button);

  config_.mode_controllers[index_of(Mode::Manual)] = parameters.mode_controllers.manual;
  config_.mode_controllers[index_of(Mode::Follow)] = parameters.mode_controllers.follow;
  config_.mode_controllers[index_of(Mode::Mpc)] = parameters.mode_controllers.mpc;
  config_.tool_controllers = parameters.tool_controllers;

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

  config_.tracking_tolerance.clear();
  config_.tracking_tolerance.reserve(parameters.joints.size());
  for (const std::string & joint : parameters.joints) {
    config_.tracking_tolerance.push_back(
      AxisTolerance{joint, parameters.tracking_tolerance.joints_map.at(joint).dq_a});
  }

  std::string reason;
  if (!validate(config_, reason)) {
    throw std::runtime_error("crane_supervisor: " + reason);
  }

  const std::string notice = tracking_tolerance_notice(config_);
  if (!notice.empty()) {
    RCLCPP_WARN(get_logger(), "crane_supervisor: %s", notice.c_str());
  }

  status_publisher_ =
    create_publisher<crane_msgs::msg::SupervisorStatus>(kStatusTopic, contract_qos());

  sway_settled_publisher_ =
    create_publisher<crane_msgs::msg::SwaySettled>(kSwaySettledTopic, contract_qos());

  // The passive pair, cached by joint name off the shared `/joint_states`: the topic carries one
  // partial message per broadcaster, so a message that names neither passive joint is the actuated
  // half and says nothing about this input's freshness.
  pendulum_state_subscription_ = subscribe<sensor_msgs::msg::JointState>(
    Input::PendulumState,
    [this](sensor_msgs::msg::JointState::ConstSharedPtr message) {
      PassivePair pair;
      for (std::size_t axis = 0; axis < kPassiveAxisCount; ++axis) {
        const auto found = std::find(
          message->name.begin(), message->name.end(), kPassiveAxisNames[axis].joint);
        if (found == message->name.end()) {
          return;
        }
        const std::size_t index =
          static_cast<std::size_t>(std::distance(message->name.begin(), found));
        if (index >= message->velocity.size()) {
          return;
        }
        pair.velocity[axis] = message->velocity[index];
      }
      pair.received = true;
      pair.stamp = rclcpp::Time(message->header.stamp, RCL_ROS_TIME);
      passive_pair_ = pair;
    });

  remote_ctrl_subscription_ = subscribe<epsilon_crane_msgs::msg::RemoteCtrlStates>(
    Input::RemoteCtrl,
    [this](epsilon_crane_msgs::msg::RemoteCtrlStates::ConstSharedPtr message) {
      remote_ctrl_ = std::move(message);
    });

  controller_state_subscription_ = subscribe<control_msgs::msg::JointTrajectoryControllerState>(
    Input::ControllerState,
    [this](control_msgs::msg::JointTrajectoryControllerState::ConstSharedPtr message) {
      controller_state_ = std::move(message);
    });

  controller_health_subscription_ = subscribe<crane_msgs::msg::VelocityControllerHealth>(
    Input::ControllerHealth,
    [this](crane_msgs::msg::VelocityControllerHealth::ConstSharedPtr message) {
      controller_health_ = std::move(message);
    });

  solver_health_subscription_ = create_subscription<crane_msgs::msg::SolverHealth>(
    kSolverHealthTopic, contract_qos(),
    [this](crane_msgs::msg::SolverHealth::ConstSharedPtr message) {
      solver_health_ = std::move(message);
    });

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

  clear_fault_service_ = create_service<std_srvs::srv::Trigger>(
    kClearFaultService,
    [this](
      const std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response) {
      const ClearFaultOutcome outcome = clear_fault(config_, observe());
      if (outcome.cleared) {
        estop_latched_ = false;
      }
      response->success = outcome.cleared;
      response->message = outcome.message;
      RCLCPP_INFO(get_logger(), "%s: %s", kClearFaultService, outcome.message.c_str());
    });

  set_mode_service_ = create_service<crane_msgs::srv::SetMode>(
    kSetModeService,
    [this](
      const crane_msgs::srv::SetMode::Request::SharedPtr request,
      crane_msgs::srv::SetMode::Response::SharedPtr response) {
      set_mode(request, response);
    });

  remote_call_group_ =
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  remote_call_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  remote_call_executor_->add_callback_group(remote_call_group_, get_node_base_interface());
  list_controllers_ = create_client<controller_manager_msgs::srv::ListControllers>(
    kListControllersService, rmw_qos_profile_services_default, remote_call_group_);
  switch_controller_ = create_client<controller_manager_msgs::srv::SwitchController>(
    kSwitchControllerService, rmw_qos_profile_services_default, remote_call_group_);
  if (!config_.mpc_node.empty()) {
    set_horizon_producer_mode_ = create_client<rcl_interfaces::srv::SetParameters>(
      "/" + config_.mpc_node + kSetParametersSuffix, rmw_qos_profile_services_default,
      remote_call_group_);
  }

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

  const rclcpp::Time sampled_at = now();
  input.sampled_at = sampled_at.seconds();
  const auto age_of = [&sampled_at](const auto & message) {
      return (sampled_at - rclcpp::Time(message->header.stamp)).seconds();
    };

  const std::array<bool, kInputCount> arrived{{
    passive_pair_.received, static_cast<bool>(remote_ctrl_),
    static_cast<bool>(controller_state_), static_cast<bool>(controller_health_)}};
  const std::array<double, kInputCount> age{{
    passive_pair_.received ? (sampled_at - passive_pair_.stamp).seconds() : 0.0,
    remote_ctrl_ ? age_of(remote_ctrl_) : 0.0,
    controller_state_ ? age_of(controller_state_) : 0.0,
    controller_health_ ? age_of(controller_health_) : 0.0}};
  for (std::size_t i = 0; i < kInputCount; ++i) {
    input.streams[i].received = arrived[i];
    input.streams[i].age = age[i];
  }

  if (passive_pair_.received) {
    input.pendulum_state.velocity = passive_pair_.velocity;
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
      if (i < message.error.velocities.size()) {
        axis.velocity_error = message.error.velocities[i];
        axis.velocity_error_reported = true;
      }
      input.controller_state.axes.push_back(std::move(axis));
    }
  }

  input.controller_manager = observe_controller_manager();

  if (solver_health_) {
    const auto & message = *solver_health_;
    input.horizon.health.received = true;
    input.horizon.health.age = age_of(solver_health_);
    input.horizon.outcome = static_cast<SolveOutcome>(message.outcome);
    input.horizon.fault = static_cast<Fault>(message.fault);
    input.horizon.solve_time = message.solve_time;
    input.horizon.solve_budget = message.solve_budget;
    input.horizon.applied_previous_solution = message.applied_previous_solution;
    input.horizon.status = message.message;
  }

  if (controller_health_) {
    const auto & message = *controller_health_;
    input.controller_health.fault = static_cast<Fault>(message.fault);
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
  const std::string refusal = mpc_horizon_refusal(config_, request->mode, observe());
  if (!refusal.empty()) {
    const ActiveMode active = active_mode(config_, observe_controller_manager());
    response->success = false;
    response->message = refusal + mode_clause(config_, active);
    response->active_mode = static_cast<std::uint8_t>(active.mode);
    RCLCPP_WARN(get_logger(), "%s: %s", kSetModeService, response->message.c_str());
    return;
  }

  refresh_controller_manager(kControllerManagerCallBudget);

  const ModeArbitration arbitration = arbitrate_mode(config_, request->mode, observe());
  if (!arbitration.accepted || !arbitration.switch_required) {
    // A mode that needs no controller switch still owns the producer. Which
    // controllers are up and which mode the producer is in are two facts, and
    // only the first one is what `switch_required` is about: a profile that
    // never spawned `trajectory_controller_a2b` is already in the mpc set the
    // moment it starts, so MODE_MPC moves nothing -- and before this, returned
    // accepted with `crane_mpc` still in shadow. Nothing then published
    // `/crane/mpc/horizon`, the inner loop ran with no producer at all, and the
    // caller had been told the mode was reached. The producer is set to what
    // the accepted mode implies here for the same reason the switching path
    // sets it: MODE_MPC means active, every other mode means shadow.
    ProducerSwitch producer;
    if (arbitration.accepted) {
      producer = set_horizon_producer_mode(arbitration.horizon_producer_active);
    }
    // Only a producer that was asked and refused unmakes the mode. Not asking
    // at all is the composition's answer -- a deployment with no `crane_mpc`
    // reaches every mode that does not need one -- and is left as it was.
    const bool producer_holds = !producer.attempted || producer.accepted;
    response->success = arbitration.accepted && producer_holds;
    response->message = arbitration.message + mode_clause(config_, arbitration.active);
    if (!producer_holds) {
      response->message += " The horizon producer did not take that mode: " + producer.account;
    }
    response->active_mode = static_cast<std::uint8_t>(arbitration.active.mode);
    if (response->success) {
      RCLCPP_INFO(get_logger(), "%s: %s", kSetModeService, response->message.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "%s: %s", kSetModeService, response->message.c_str());
    }
    return;
  }

  const Mode requested = static_cast<Mode>(request->mode);

  ProducerSwitch producer;
  if (arbitration.horizon_producer_active) {
    producer = set_horizon_producer_mode(true);
  }

  std::string account;
  const bool switched = switch_controllers(arbitration.activate, arbitration.deactivate, account);

  refresh_controller_manager(kControllerManagerCallBudget);
  const ActiveMode reached = active_mode(config_, observe_controller_manager());

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
  poll_controller_manager();

  const SupervisorInput observed = observe();
  const SupervisorDecision decision = decide(config_, observed);
  estop_latched_ = decision.estop_latched;
  sway_state_ = decision.sway;

  const rclcpp::Time stamp = now();

  crane_msgs::msg::SupervisorStatus status;
  status.header.stamp = stamp;
  status.header.frame_id = "";
  status.mode = static_cast<std::uint8_t>(decision.mode);
  status.fault = static_cast<std::uint8_t>(decision.fault);
  status.tracking_error = decision.tracking_error;
  status.inside_working_cell = decision.inside_working_cell;
  status.deadman_held = decision.deadman_held;
  status.message = decision.message;
  status_publisher_->publish(status);

  crane_msgs::msg::SwaySettled settled;
  settled.header.stamp = stamp;
  settled.header.frame_id = "";
  settled.settled = static_cast<std::uint8_t>(decision.sway.settled);
  settled.velocity = observed.pendulum_state.velocity;
  settled.message = settled_clause_of(decision.message);
  sway_settled_publisher_->publish(settled);

  if (!ever_reported_ || decision.fault != reported_fault_) {
    ever_reported_ = true;
    reported_fault_ = decision.fault;
    if (decision.fault == Fault::None) {
      RCLCPP_INFO(get_logger(), "%s", decision.message.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "%s", decision.message.c_str());
    }
  }

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
  refresh_controller_manager(kControllerManagerCallBudget);

  const ModeArbitration arbitration =
    arbitrate_mode(config_, static_cast<std::uint8_t>(Mode::Idle), observe());
  if (!arbitration.accepted || !arbitration.switch_required) {
    const std::string account = handback.message + " " + arbitration.message;
    RCLCPP_ERROR(get_logger(), "%s", account.c_str());
    return account;
  }

  std::string carried;
  const bool switched = switch_controllers(arbitration.activate, arbitration.deactivate, carried);

  refresh_controller_manager(kControllerManagerCallBudget);
  const ActiveMode reached = active_mode(config_, observe_controller_manager());

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
