#include "crane_supervisor/supervisor_node.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "crane_supervisor/crane_supervisor_parameters.hpp"

namespace crane_supervisor
{
namespace
{

/// The QoS of every streamed contract this node touches.
/**
 * Reliable, depth 1, volatile -- ROS 2 Interfaces §1 sets the category by the
 * consumer, and both of these are state a controller acts on. It is the same
 * profile `crane_msgs`' own ROS contract test asserts on both topics, so the
 * two agree by construction rather than by inspection.
 */
rclcpp::QoS contract_qos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
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
  config_.pendulum_state_timeout = parameters.pendulum_state_timeout;
  config_.remote_ctrl_timeout = parameters.remote_ctrl_timeout;
  config_.controller_state_timeout = parameters.controller_state_timeout;
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
  // failure of exactly the kind Style Guide §4 exists to stop.
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

  pendulum_state_subscription_ = create_subscription<crane_msgs::msg::PendulumState>(
    kPendulumStateTopic, contract_qos(),
    [this](crane_msgs::msg::PendulumState::ConstSharedPtr message) {
      pendulum_state_ = std::move(message);
    });

  remote_ctrl_subscription_ = create_subscription<epsilon_crane_msgs::msg::RemoteCtrlStates>(
    kRemoteCtrlStatesTopic, contract_qos(),
    [this](epsilon_crane_msgs::msg::RemoteCtrlStates::ConstSharedPtr message) {
      remote_ctrl_ = std::move(message);
    });

  // The third input, and the one that makes the tracking duty of §5 row 1 a
  // typed cause. The controller that computes the error is the one that
  // publishes it, so nothing is re-derived here from `/joint_states`.
  controller_state_subscription_ =
    create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
    kControllerStateTopic, contract_qos(),
    [this](control_msgs::msg::JointTrajectoryControllerState::ConstSharedPtr message) {
      controller_state_ = std::move(message);
    });

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

  status_timer_ = create_wall_timer(status_period(), [this]() {update();});

  RCLCPP_INFO(
    get_logger(),
    "crane_supervisor: publishing %s at %.1f Hz. It observes %s, %s -- button %d of it is the "
    "deadman -- and %s, serves %s, and does nothing else. It holds no stop authority: the "
    "hardware stop input is an unverified commissioning prerequisite, so the package has no path "
    "to a motion command by construction, and what it does with the emergency stop is diagnosis "
    "and recovery rather than protection.",
    kStatusTopic, kStatusRate, kPendulumStateTopic, kRemoteCtrlStatesTopic,
    config_.deadman_button, kControllerStateTopic, kClearFaultService);
}

SupervisorInput SupervisorNode::observe() const
{
  SupervisorInput input;
  input.estop_latched = estop_latched_;

  if (pendulum_state_) {
    input.pendulum_state.received = true;
    input.pendulum_state.valid = pendulum_state_->valid;
    input.pendulum_state.status = pendulum_state_->status;
    // `now() - header.stamp` is the age the publisher of that header intends a
    // consumer to compute: pendulum_state_broadcaster measures its own
    // staleness on the same clock it stamps with, so the two agree about what
    // time it is without either of them saying so on the wire.
    input.pendulum_state.age = (now() - rclcpp::Time(pendulum_state_->header.stamp)).seconds();
  }

  if (remote_ctrl_) {
    input.remote_ctrl.received = true;
    input.remote_ctrl.deadman_held = deadman_of(*remote_ctrl_, config_.deadman_button);
    input.remote_ctrl.em_stop = remote_ctrl_->em_stop;
    // gpio_controller stamps the message from the control cycle's own time, so
    // the age is measured the same way as the passive state's and means the
    // same thing.
    input.remote_ctrl.age = (now() - rclcpp::Time(remote_ctrl_->header.stamp)).seconds();
  }

  if (controller_state_) {
    const auto & message = *controller_state_;
    input.controller_state.received = true;
    // The trajectory controller stamps its state with the control cycle's own
    // time, the same clock the other two inputs are aged against.
    input.controller_state.age = (now() - rclcpp::Time(message.header.stamp)).seconds();
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
