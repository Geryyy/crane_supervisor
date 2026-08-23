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

#include "crane_supervisor/crane_supervisor_parameters.hpp"

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
  config_.deadman_button = static_cast<int>(parameters.deadman_button);

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

  // The freshness sweep runs from here and from nowhere else. A deadline
  // evaluated in a subscription callback could not fire on the stream that
  // stopped, which is the only stream it exists for.
  status_timer_ = create_wall_timer(status_period(), [this]() {update();});

  RCLCPP_INFO(
    get_logger(),
    "crane_supervisor: publishing %s at %.1f Hz. Every input it holds has a freshness deadline of "
    "its own and none is exempt -- %s -- and an input that stops arriving is reported as a fault "
    "rather than left at FAULT_NONE. Button %d of the remote is the deadman. It serves %s and "
    "does nothing else. It holds no stop authority: the hardware stop input is an unverified "
    "commissioning prerequisite, so the package has no path to a motion command by construction, "
    "and what it does with the emergency stop is diagnosis and recovery rather than protection.",
    kStatusTopic, kStatusRate, deadline_summary(config_).c_str(), config_.deadman_button,
    kClearFaultService);
}

SupervisorInput SupervisorNode::observe() const
{
  SupervisorInput input;
  input.estop_latched = estop_latched_;

  // One reading of the clock for all of them, so two inputs sampled in the same
  // cycle are aged against the same instant. `now() - header.stamp` is the age
  // the publisher of that header intends a consumer to compute: every producer
  // here stamps from the control cycle's own time on the same clock, so the two
  // sides agree about what time it is without either saying so on the wire.
  const rclcpp::Time sampled_at = now();
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

void SupervisorNode::update()
{
  const SupervisorDecision decision = decide(config_, observe());
  // The latch is the one thing a cycle carries into the next one. `decide()`
  // raises it; only an acknowledged `/crane/clear_fault` lowers it.
  estop_latched_ = decision.estop_latched;

  crane_msgs::msg::SupervisorStatus status;
  status.header.stamp = now();
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
}

}  // namespace crane_supervisor
