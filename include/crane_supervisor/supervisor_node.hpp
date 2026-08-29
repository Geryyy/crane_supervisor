
#ifndef CRANE_SUPERVISOR__SUPERVISOR_NODE_HPP_
#define CRANE_SUPERVISOR__SUPERVISOR_NODE_HPP_

#include <array>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <memory>

#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "crane_msgs/msg/solver_health.hpp"
#include "crane_msgs/msg/supervisor_status.hpp"
#include "crane_msgs/msg/sway_settled.hpp"
#include "crane_msgs/msg/velocity_controller_health.hpp"
#include "crane_msgs/srv/set_mode.hpp"
#include "crane_supervisor/supervisor_core.hpp"
#include "epsilon_crane_msgs/msg/remote_ctrl_states.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace crane_supervisor
{

/// The contract names and the contract rate of ROS 2 Interfaces §2 and §4.
inline constexpr char kStatusTopic[] = "/crane/supervisor/status";
/// The second stream of §4, and the reason it is a stream rather than a field.
inline constexpr char kSwaySettledTopic[] = "/crane/sway_settled";
inline constexpr char kClearFaultService[] = "/crane/clear_fault";
inline constexpr char kSetModeService[] = "/crane/set_mode";
inline constexpr double kStatusRate = 20.0;

/// The two controller-manager services this node calls, and the only ones.
inline constexpr char kListControllersService[] = "/controller_manager/list_controllers";
inline constexpr char kSwitchControllerService[] = "/controller_manager/switch_controller";

inline constexpr char kSolverHealthTopic[] = "/crane/mpc/solver_health";

/// How this node moves the horizon producer between shadow and active.
inline constexpr char kSetParametersSuffix[] = "/set_parameters";
inline constexpr char kHorizonProducerModeParameter[] = "mode";
inline constexpr char kHorizonProducerActive[] = "active";
inline constexpr char kHorizonProducerShadow[] = "shadow";

/// How long a *blocking* call on the controller manager may take, s.
inline constexpr double kControllerManagerCallBudget = 0.5;
inline constexpr double kSwitchControllerTimeout = 1.0;
inline constexpr double kSwitchBudget = 3.0;

/// How long the horizon producer may take to answer a `set_parameters`, s.
inline constexpr double kProducerModeBudget = 0.5;

/// The four input contract names, off the policy rows that already carry them.
/// The passive pair arrives on `/joint_states` beside the actuated six, in its own partial
/// message, so this name is shared with `joint_state_broadcaster` rather than owned.
inline constexpr const char * kPassiveStateTopic = policy_of(Input::PendulumState).topic;
inline constexpr const char * kRemoteCtrlStatesTopic = policy_of(Input::RemoteCtrl).topic;
inline constexpr const char * kControllerStateTopic = policy_of(Input::ControllerState).topic;
inline constexpr const char * kControllerHealthTopic = policy_of(Input::ControllerHealth).topic;

/// The QoS of every streamed contract this node touches.
[[nodiscard]] inline rclcpp::QoS contract_qos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
}

/// The configured deadman, out of the twelve booleans the message carries.
[[nodiscard]] bool deadman_of(
  const epsilon_crane_msgs::msg::RemoteCtrlStates & message, int button);

/// The node of ROS 2 Interfaces §2, at the rate that table gives it.
class SupervisorNode : public rclcpp::Node
{
public:
  explicit SupervisorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// One status cycle: sample what arrived, age it, decide, publish.
  void update();

  [[nodiscard]] const SupervisorConfig & config() const noexcept {return config_;}

private:
  static std::chrono::nanoseconds status_period();

  /// The one place a subscription is created in this package.
  template<typename MessageT, typename CallbackT>
  typename rclcpp::Subscription<MessageT>::SharedPtr subscribe(Input input, CallbackT && callback)
  {
    claimed_[index_of(input)] = true;
    return create_subscription<MessageT>(
      policy_of(input).topic, contract_qos(), std::forward<CallbackT>(callback));
  }

  /// What one cycle observed, from what the node is holding right now.
  [[nodiscard]] SupervisorInput observe() const;

  /// The controller manager's newest answer, aged on this node's own clock.
  [[nodiscard]] ControllerManagerReport observe_controller_manager() const;

  /// One non-blocking step of the polled snapshot. Called on the status cycle.
  void poll_controller_manager();

  /// A blocking `list_controllers`, for the two points a decision needs truth.
  bool refresh_controller_manager(double timeout);

  /// One `/crane/set_mode` call: arbitrate, then switch, then read back.
  void set_mode(
    const crane_msgs::srv::SetMode::Request::SharedPtr request,
    crane_msgs::srv::SetMode::Response::SharedPtr response);

  bool switch_controllers(
    const std::vector<std::string> & activate, const std::vector<std::string> & deactivate,
    std::string & account);

  /// Puts the horizon producer into `active` or into `shadow`, and reports it.
  ProducerSwitch set_horizon_producer_mode(bool active);

  /// The stop half of wiki/control_architecture.md §5 row 3, once it is owed.
  std::string hand_back_from_mpc(const SolverHandback & handback);

  SupervisorConfig config_;
  std::array<bool, kInputCount> claimed_{};
  /// The newest passive pair off `/joint_states`, cached by joint name.
  /**
   * The topic carries partial messages from two broadcasters, so the pair is kept as values and
   * a stamp rather than as the message: the newest message on the topic is usually the actuated
   * six, and aging this input against that stamp would report a freshness that is not its own.
   */
  struct PassivePair
  {
    bool received{false};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    std::array<double, kPassiveAxisCount> velocity{{0.0, 0.0}};
  };
  PassivePair passive_pair_;
  epsilon_crane_msgs::msg::RemoteCtrlStates::ConstSharedPtr remote_ctrl_;
  control_msgs::msg::JointTrajectoryControllerState::ConstSharedPtr controller_state_;
  crane_msgs::msg::VelocityControllerHealth::ConstSharedPtr controller_health_;
  crane_msgs::msg::SolverHealth::ConstSharedPtr solver_health_;
  bool estop_latched_{false};
  /// The sway dwell, carried from one decision into the next the same way.
  SwayState sway_state_;
  Fault reported_fault_{Fault::None};
  bool ever_reported_{false};
  bool handback_attempted_{false};

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr pendulum_state_subscription_;
  rclcpp::Subscription<epsilon_crane_msgs::msg::RemoteCtrlStates>::SharedPtr
    remote_ctrl_subscription_;
  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr
    controller_state_subscription_;
  rclcpp::Subscription<crane_msgs::msg::VelocityControllerHealth>::SharedPtr
    controller_health_subscription_;
  rclcpp::Subscription<crane_msgs::msg::SolverHealth>::SharedPtr solver_health_subscription_;
  /// The newest `list_controllers` answer, or null before the first one.
  controller_manager_msgs::srv::ListControllers::Response::SharedPtr controllers_;
  rclcpp::Time controllers_at_{0, 0, RCL_ROS_TIME};

  /// One asynchronous `list_controllers` request in flight, or none.
  struct PendingSnapshot
  {
    rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedFuture future;
    std::int64_t request_id{0};
    rclcpp::Time sent_at{0, 0, RCL_ROS_TIME};
  };
  std::optional<PendingSnapshot> pending_snapshot_;

  rclcpp::Publisher<crane_msgs::msg::SupervisorStatus>::SharedPtr status_publisher_;
  /// The settled predicate, as a field rather than as a sentence.
  rclcpp::Publisher<crane_msgs::msg::SwaySettled>::SharedPtr sway_settled_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_fault_service_;
  rclcpp::Service<crane_msgs::srv::SetMode>::SharedPtr set_mode_service_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  rclcpp::CallbackGroup::SharedPtr remote_call_group_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr remote_call_executor_;
  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr list_controllers_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_;
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr set_horizon_producer_mode_;
};

}  // namespace crane_supervisor

#endif  // CRANE_SUPERVISOR__SUPERVISOR_NODE_HPP_
