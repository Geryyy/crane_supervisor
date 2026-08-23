#include "crane_supervisor/supervisor_node.hpp"

#include <chrono>
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

std::chrono::nanoseconds SupervisorNode::status_period()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / kStatusRate));
}

SupervisorNode::SupervisorNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("crane_supervisor", options)
{
  const ParamListener listener(this);
  config_.pendulum_state_timeout = listener.get_params().pendulum_state_timeout;

  // generate_parameter_library has already rejected a value outside the
  // declared bounds. The core is checked against its own rule anyway: it is the
  // ROS-free half, it is what a later configuration path will be checked
  // against, and a margin that reached `decide()` unchecked would be a silent
  // failure of exactly the kind Style Guide §4 exists to stop.
  std::string reason;
  if (!validate(config_, reason)) {
    throw std::runtime_error("crane_supervisor: " + reason);
  }

  status_publisher_ =
    create_publisher<crane_msgs::msg::SupervisorStatus>(kStatusTopic, contract_qos());

  pendulum_state_subscription_ = create_subscription<crane_msgs::msg::PendulumState>(
    kPendulumStateTopic, contract_qos(),
    [this](crane_msgs::msg::PendulumState::ConstSharedPtr message) {
      pendulum_state_ = std::move(message);
    });

  status_timer_ = create_wall_timer(status_period(), [this]() {update();});

  RCLCPP_INFO(
    get_logger(),
    "crane_supervisor: publishing %s at %.1f Hz. It observes %s and nothing else, and it holds no "
    "stop authority: the hardware stop input is an unverified commissioning prerequisite, so the "
    "package has no path to a motion command by construction.",
    kStatusTopic, kStatusRate, kPendulumStateTopic);
}

void SupervisorNode::update()
{
  SupervisorInput input;
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

  const SupervisorDecision decision = decide(config_, input);

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
