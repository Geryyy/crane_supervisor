// `/crane/set_mode`, against a real controller manager, in process.
//
// The S5 shape `crane_control` already uses (PRD "Seams", S5): a real
// `controller_manager::ControllerManager` with mock hardware, real controllers
// loaded by name out of a parameter file, and the control loop driven by hand.
// No launch, no simulator, no spawner, no live graph.
//
// **A switch is the one call that must never be issued against the machine by
// accident**, which is why the assertion is here and not against a bringup.
// This workspace ships a Cyclone configuration that pins a unicast peer at the
// real crane, and a shell that has sourced it would put these nodes on the
// machine's live DDS graph -- where duplicate nodes have already been observed.
// So the isolation is the test's own as well as `ralph/verify.sh`'s: the two
// variables are scrubbed here, localhost-only transport is forced, and the
// domain is stepped again off the one this run was given.
//
// What is asserted is the whole path an operator's request takes: the service
// call, the arbitration, the real `switch_controller` call, the claim moving on
// the manager, and the mode coming back on `/crane/supervisor/status` -- read
// off the manager rather than remembered.
//
// # Threads, and why there are three kinds of them
//
// `switch_controller` blocks until the manager performs the switch in its own
// control cycle, and the supervisor's `/crane/set_mode` handler blocks until it
// knows the outcome. Three things therefore have to make progress at the same
// time, and none of them can be the same thread:
//
//   the control loop  `read`/`update`/`write`, driven by the test's own thread.
//                     The manager's switch will not complete without it.
//   the manager       its `switch_controller` service handler, which waits on
//                     that loop. It cannot run on the thread driving it.
//   the supervisor    its `/crane/set_mode` handler, which waits on the manager.
//
// So the executor is multi-threaded and holds all three nodes, a spinner thread
// runs it, and the test thread does nothing but turn the control loop and watch
// for an outcome. That is a property of the *harness*: the deployed node spins
// the single-threaded executor `crane_supervisor_main.cpp` creates, and what the
// arrangement here costs it is written down in `supervisor_node.hpp`.

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <future>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "controller_manager/controller_manager.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "crane_msgs/msg/supervisor_status.hpp"
#include "crane_msgs/srv/set_mode.hpp"
#include "crane_supervisor/supervisor_node.hpp"
#include "epsilon_crane_msgs/msg/remote_ctrl_states.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace
{

using crane_msgs::msg::SupervisorStatus;
using crane_msgs::srv::SetMode;
using epsilon_crane_msgs::msg::RemoteCtrlStates;
using std_srvs::srv::Trigger;

constexpr char kFollowController[] = "follow_velocity_controller";
constexpr char kManualController[] = "manual_velocity_controller";
constexpr char kToolController[] = "tool_velocity_controller";

/// The five arm joints and the one tool joint, split the way the two claims of
/// wiki/control_architecture.md §7.3 split them.
const std::vector<std::string> & arm_joints()
{
  static const std::vector<std::string> joints{
    "theta1_slewing_joint", "theta2_boom_joint", "theta3_arm_joint", "q4_big_telescope",
    "theta8_rotator_joint"};
  return joints;
}

constexpr char kToolJoint[] = "q9_left_rail_joint";

/// The manager's rate, and the period the harness drives it at.
constexpr double kRate = 100.0;

/// How long any one wait may take before the test fails, s. It bounds a failure
/// rather than a success: every wait returns as soon as its predicate holds.
constexpr double kBudget = 20.0;

/// How long the harness pauses between control cycles.
/**
 * The loop's period and not a wait: every *wait* below is on an observable
 * outcome. A tight loop would burn a core for the whole run on a machine that
 * is already running the executor's threads beside it, and the manager needs
 * only that the cycle turns, not that it turns as fast as it can.
 */
constexpr std::chrono::milliseconds kCyclePeriod{1};

/// The domain this binary runs on: one step off the one it was given, and a
/// different step from `test_status_stream.cpp`'s.
/**
 * `verify.sh` puts each issue's run on a domain of its own, which keeps these
 * nodes off the machine's live graph and off another issue's. What it cannot do
 * is separate this binary from `test_status_stream.cpp`, which colcon runs at
 * the same time: both bring up a node called `crane_supervisor` publishing
 * `/crane/supervisor/status`, so on one domain each would be reading the
 * other's stream and neither would be testing what it thinks it is.
 *
 * The offset differs from that file's by 50, which is never zero modulo 101, so
 * the two binaries can never land on the same domain whatever they were given.
 * The range 1..101 is the one ROS 2 keeps clear of the Linux ephemeral ports.
 */
std::string stepped_domain()
{
  const char * const given = ::getenv("ROS_DOMAIN_ID");
  const int base = (given == nullptr) ? 0 : std::atoi(given);
  return std::to_string(1 + ((base + 50) % 101 + 101) % 101);
}

class RclcppEnvironment : public ::testing::Environment
{
public:
  void SetUp() override
  {
    ::unsetenv("CYCLONEDDS_URI");
    ::unsetenv("FASTRTPS_DEFAULT_PROFILES_FILE");
    ::setenv("ROS_LOCALHOST_ONLY", "1", 1);
    ::setenv("ROS_DOMAIN_ID", stepped_domain().c_str(), 1);
    rclcpp::init(0, nullptr);
  }
  void TearDown() override {rclcpp::shutdown();}
};

[[maybe_unused]] const ::testing::Environment * const kEnvironment =
  ::testing::AddGlobalTestEnvironment(new RclcppEnvironment);

/// Six actuated joints on mock hardware, each with a `velocity` command.
/**
 * The smallest description that carries the claim this issue is about. There is
 * no hydraulics block and no passive pair: nothing here evaluates a model, and a
 * fixture that carried one would be asserting `crane_model`'s contract in a
 * binary written about mode arbitration.
 */
std::string make_urdf()
{
  std::ostringstream urdf;
  urdf << "<?xml version=\"1.0\"?>\n<robot name=\"crane_mode_switch_harness\">\n"
       << "  <link name=\"base_link\"/>\n";
  std::vector<std::string> joints = arm_joints();
  joints.push_back(kToolJoint);
  for (std::size_t i = 0; i < joints.size(); ++i) {
    urdf << "  <link name=\"l" << i + 1 << "\"/>\n";
  }
  for (std::size_t i = 0; i < joints.size(); ++i) {
    const std::string parent = (i == 0) ? std::string("base_link") : "l" + std::to_string(i);
    urdf << "  <joint name=\"" << joints[i] << "\" type=\"revolute\">\n"
         << "    <parent link=\"" << parent << "\"/><child link=\"l" << i + 1 << "\"/>\n"
         << "    <origin xyz=\"1 0 0\"/><axis xyz=\"0 1 0\"/>\n"
         << "    <limit lower=\"-3.0\" upper=\"3.0\" effort=\"1000\" velocity=\"2.0\"/>\n"
         << "  </joint>\n";
  }
  urdf << "  <ros2_control name=\"CraneModeSwitchHardware\" type=\"system\">\n"
       << "    <hardware>\n"
       << "      <plugin>mock_components/GenericSystem</plugin>\n"
       << "      <param name=\"calculate_dynamics\">false</param>\n"
       << "    </hardware>\n";
  for (const std::string & joint : joints) {
    urdf << "    <joint name=\"" << joint << "\">\n"
         << "      <command_interface name=\"velocity\"/>\n"
         << "      <state_interface name=\"position\">"
         << "<param name=\"initial_value\">0.0</param></state_interface>\n"
         << "      <state_interface name=\"velocity\">"
         << "<param name=\"initial_value\">0.0</param></state_interface>\n"
         << "    </joint>\n";
  }
  urdf << "  </ros2_control>\n</robot>\n";
  return urdf.str();
}

class ModeSwitch : public ::testing::Test
{
protected:
  void SetUp() override
  {
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
      rclcpp::ExecutorOptions(), 4);

    auto options = controller_manager::get_cm_node_options();
    auto arguments = options.arguments();
    arguments.push_back("--ros-args");
    arguments.push_back("--params-file");
    arguments.push_back(
      std::string(CRANE_SUPERVISOR_TEST_CONFIG_DIR) + "/mode_switch_controllers.yaml");
    options.arguments(arguments);

    cm_ = std::make_shared<controller_manager::ControllerManager>(
      executor_, make_urdf(), true, "controller_manager", "", options);

    // Loaded by name only: the type comes from the parameter file, so a file
    // that never reached the node fails here rather than leaving three
    // controllers at a default that happens to look plausible.
    for (const char * name : {kFollowController, kManualController, kToolController}) {
      ASSERT_TRUE(cm_->load_controller(name)) << name;
      ASSERT_EQ(cm_->configure_controller(name), controller_interface::return_type::OK) << name;
    }
    ASSERT_EQ(cm_->get_update_rate(), static_cast<unsigned int>(kRate));

    supervisor_ = std::make_shared<crane_supervisor::SupervisorNode>(supervisor_options());

    observer_ = std::make_shared<rclcpp::Node>("crane_supervisor_mode_observer");
    const rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    status_ = observer_->create_subscription<SupervisorStatus>(
      crane_supervisor::kStatusTopic, qos,
      [this](SupervisorStatus::ConstSharedPtr message) {latest_ = *message;});
    remote_ctrl_ = observer_->create_publisher<RemoteCtrlStates>(
      crane_supervisor::kRemoteCtrlStatesTopic, qos);
    set_mode_ = observer_->create_client<SetMode>(crane_supervisor::kSetModeService);
    clear_fault_ = observer_->create_client<Trigger>(crane_supervisor::kClearFaultService);

    executor_->add_node(cm_);
    executor_->add_node(supervisor_);
    executor_->add_node(observer_);
    spinner_ = std::thread([this]() {executor_->spin();});
  }

  void TearDown() override
  {
    executor_->cancel();
    if (spinner_.joinable()) {
      spinner_.join();
    }
    executor_->remove_node(observer_);
    executor_->remove_node(supervisor_);
    executor_->remove_node(cm_);
    set_mode_.reset();
    clear_fault_.reset();
    status_.reset();
    remote_ctrl_.reset();
    observer_.reset();
    supervisor_.reset();
    cm_.reset();
    executor_.reset();
  }

  /// How the node under test is built: the deployment's node, told which
  /// controller holds the claim in each mode and which holds the tool claim.
  static rclcpp::NodeOptions supervisor_options()
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
      {rclcpp::Parameter("pendulum_state_timeout", 0.2),
        rclcpp::Parameter("remote_ctrl_timeout", 0.5),
        rclcpp::Parameter("controller_state_timeout", 0.2),
        rclcpp::Parameter("controller_health_timeout", 0.2),
        // Generous, because the poll shares a callback group with a status
        // timer that a mode switch blocks: a margin tuned for a quiet node
        // would report the manager stale for a reason the harness created.
        rclcpp::Parameter("controller_manager_timeout", 2.0),
        rclcpp::Parameter(
          "mode_controllers.follow", std::vector<std::string>{kFollowController}),
        rclcpp::Parameter(
          "mode_controllers.manual", std::vector<std::string>{kManualController}),
        rclcpp::Parameter("tool_controllers", std::vector<std::string>{kToolController}),
        // No horizon producer, which is what this harness actually composes:
        // `mode_controllers.mpc` is left empty, so `validate()` allows the name
        // to be empty too and no request here moves anything outside the
        // controller manager. What a switch into MODE_MPC does to the producer
        // needs a producer to watch and is `test_handover_sequence.cpp`.
        rclcpp::Parameter("mpc_node", std::string{})});
    return options;
  }

  /// One control cycle, on the harness's own thread.
  void step()
  {
    cm_->read(now_, period_);
    cm_->update(now_, period_);
    cm_->write(now_, period_);
    now_ = now_ + period_;
  }

  /// Turn the control loop until `ready` holds. Returns false on the budget.
  bool drive_until(const std::function<bool()> & ready)
  {
    const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::duration<double>(kBudget);
    while (std::chrono::steady_clock::now() < deadline) {
      if (ready()) {
        return true;
      }
      step();
      std::this_thread::sleep_for(kCyclePeriod);
    }
    return ready();
  }

  /// One `/crane/set_mode` call, with the control loop turning underneath it.
  SetMode::Response::SharedPtr request(std::uint8_t mode)
  {
    if (!drive_until([this]() {return set_mode_->service_is_ready();})) {
      return nullptr;
    }
    auto message = std::make_shared<SetMode::Request>();
    message->mode = mode;
    auto future = set_mode_->async_send_request(message).future.share();
    const bool answered = drive_until([&future]() {
        return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
      });
    return answered ? future.get() : nullptr;
  }

  /// The manager's own view, read directly rather than through the supervisor.
  /**
   * Only ever called once a request has come back. `get_loaded_controllers()`
   * takes the manager's controller lock, which `switch_controller()` holds for
   * the whole switch while it waits on this thread to turn the control loop --
   * a reader that walked the list mid-switch would deadlock against itself.
   */
  bool is_active(const std::string & name) const
  {
    for (const auto & spec : cm_->get_loaded_controllers()) {
      if (spec.info.name == name) {
        return spec.c->get_lifecycle_state().id() ==
               lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
      }
    }
    return false;
  }

  /// A remote with the stop released and the deadman held, stamped now.
  void publish_remote()
  {
    RemoteCtrlStates message;
    message.header.stamp = observer_->now();
    message.button12 = true;
    message.em_stop = false;
    remote_ctrl_->publish(message);
  }

  /// Keep the remote alive until `ready` holds, so that the latched stop of
  /// §6.1 -- which every freshly started supervisor is in -- can be cleared.
  bool drive_with_remote_until(const std::function<bool()> & ready)
  {
    return drive_until([this, &ready]() {
        publish_remote();
        return ready();
      });
  }

  /// A switch taken outside the supervisor entirely, the way a spawner or a
  /// crashing controller would take one.
  /**
   * Off the harness's own thread, because `switch_controller()` blocks until
   * the control loop performs the switch and this thread is what turns that
   * loop -- the same reason `crane_control`'s S5 harness runs it asynchronously.
   */
  controller_interface::return_type switch_outside_the_supervisor(
    const std::vector<std::string> & activate, const std::vector<std::string> & deactivate)
  {
    auto switched = std::async(std::launch::async, [this, activate, deactivate]() {
        return cm_->switch_controller(
          activate, deactivate, controller_manager_msgs::srv::SwitchController::Request::STRICT,
          true, rclcpp::Duration::from_seconds(kBudget));
      });
    drive_until([&switched]() {
      return switched.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
    });
    return switched.get();
  }

  /// Bring the supervisor out of the stop it may start latched in, so that what
  /// a test asserts afterwards is about the mode and not about the stop.
  void clear_the_starting_latch()
  {
    ASSERT_TRUE(drive_with_remote_until([this]() {return clear_fault_->service_is_ready();}));
    // Whether a latch was ever raised is a race this harness does not control:
    // a supervisor whose first status cycle ran before the first remote message
    // is in the latched stop of §6.1, and one whose did not never entered it.
    // Both are correct behaviour, so the acknowledgement is issued and *either*
    // answer accepted -- what is waited on is the outcome, which is a stream no
    // longer reporting the stop.
    ASSERT_TRUE(drive_with_remote_until([this]() {return latest_.has_value();}));
    auto future = clear_fault_->async_send_request(std::make_shared<Trigger::Request>())
      .future.share();
    ASSERT_TRUE(
      drive_with_remote_until([&future]() {
        return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
      }));
    ASSERT_TRUE(
      drive_with_remote_until([this]() {
        return latest_->fault != SupervisorStatus::FAULT_ESTOP;
      }))
      << latest_->message;
  }

  /// The newest status report, or none yet.
  bool saw_mode(std::uint8_t mode)
  {
    return latest_.has_value() && latest_->mode == mode;
  }

  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::shared_ptr<controller_manager::ControllerManager> cm_;
  std::shared_ptr<crane_supervisor::SupervisorNode> supervisor_;
  rclcpp::Node::SharedPtr observer_;
  rclcpp::Subscription<SupervisorStatus>::SharedPtr status_;
  rclcpp::Publisher<RemoteCtrlStates>::SharedPtr remote_ctrl_;
  rclcpp::Client<SetMode>::SharedPtr set_mode_;
  rclcpp::Client<Trigger>::SharedPtr clear_fault_;
  std::optional<SupervisorStatus> latest_;
  std::thread spinner_;
  const rclcpp::Duration period_ = rclcpp::Duration::from_seconds(1.0 / kRate);
  rclcpp::Time now_{0, 0, RCL_ROS_TIME};
};

}  // namespace

TEST_F(ModeSwitch, TheServiceIsTheOnlyWayAModeChangesAndTheSwitchIsReal)
{
  clear_the_starting_latch();

  // Nothing is active, and the supervisor says so from the manager's own answer
  // rather than from a default.
  ASSERT_TRUE(drive_with_remote_until([this]() {return saw_mode(SupervisorStatus::MODE_IDLE);}));
  EXPECT_FALSE(is_active(kFollowController));

  const auto to_follow = request(SupervisorStatus::MODE_FOLLOW);
  ASSERT_NE(to_follow, nullptr);
  EXPECT_TRUE(to_follow->success) << to_follow->message;
  EXPECT_FALSE(to_follow->message.empty());
  // `active_mode` is what is actually active after the call, and the manager
  // agrees: the claim moved.
  EXPECT_EQ(to_follow->active_mode, SupervisorStatus::MODE_FOLLOW) << to_follow->message;
  EXPECT_TRUE(is_active(kFollowController));
  EXPECT_FALSE(is_active(kManualController));

  // And it reaches the stream, re-derived rather than remembered.
  ASSERT_TRUE(drive_with_remote_until([this]() {return saw_mode(SupervisorStatus::MODE_FOLLOW);}))
    << latest_->message;
  EXPECT_NE(latest_->message.find(kFollowController), std::string::npos) << latest_->message;
}

TEST_F(ModeSwitch, ManualAndAutonomousExcludeEachOtherAndTheSupervisorArbitratesIt)
{
  // wiki/control_architecture.md §7: the two want the same five `velocity`
  // command interfaces, so ros2_control would refuse to activate one while the
  // other holds it. What this asserts is that nobody ever reaches that refusal:
  // the supervisor deactivates the outgoing mode and activates the incoming one
  // in the same request, so the exclusion is an arbitrated decision with a
  // stated reason instead of an activation failure in a log.
  clear_the_starting_latch();
  ASSERT_NE(request(SupervisorStatus::MODE_FOLLOW), nullptr);
  ASSERT_TRUE(is_active(kFollowController));

  const auto to_manual = request(SupervisorStatus::MODE_MANUAL);
  ASSERT_NE(to_manual, nullptr);
  EXPECT_TRUE(to_manual->success) << to_manual->message;
  EXPECT_EQ(to_manual->active_mode, SupervisorStatus::MODE_MANUAL) << to_manual->message;
  EXPECT_TRUE(is_active(kManualController));
  EXPECT_FALSE(is_active(kFollowController)) << "both claimants are active at once";

  // Releasing the claim is a mode of its own, and the report says it is not a
  // stop: nothing was zeroed at the driver boundary (§5.2, §7.2).
  const auto to_idle = request(SupervisorStatus::MODE_IDLE);
  ASSERT_NE(to_idle, nullptr);
  EXPECT_TRUE(to_idle->success) << to_idle->message;
  EXPECT_EQ(to_idle->active_mode, SupervisorStatus::MODE_IDLE) << to_idle->message;
  EXPECT_FALSE(is_active(kManualController));
  EXPECT_NE(to_idle->message.find("not a stop"), std::string::npos) << to_idle->message;

  // Asking again for the mode that is active is a no-op, reported as one.
  const auto again = request(SupervisorStatus::MODE_IDLE);
  ASSERT_NE(again, nullptr);
  EXPECT_TRUE(again->success) << again->message;
  EXPECT_NE(again->message.find("no switch was issued"), std::string::npos) << again->message;
}

TEST_F(ModeSwitch, AnArmModeChangeLeavesTheToolClaimExactlyWhereItWas)
{
  // §7.3, against a manager that really has two claims: the tool axis is on its
  // own controller and can move while either arm controller is inactive. A mode
  // model that assumed one claim per machine would take it down here, silently,
  // on every arm mode change.
  clear_the_starting_latch();

  // The tool claim is taken outside the supervisor entirely, because no mode
  // owns it: a gripper action is not an arm mode.
  ASSERT_EQ(
    switch_outside_the_supervisor({kToolController}, {}), controller_interface::return_type::OK);
  ASSERT_TRUE(is_active(kToolController));

  ASSERT_NE(request(SupervisorStatus::MODE_FOLLOW), nullptr);
  EXPECT_TRUE(is_active(kToolController)) << "activating an arm mode disturbed the tool claim";

  const auto to_manual = request(SupervisorStatus::MODE_MANUAL);
  ASSERT_NE(to_manual, nullptr);
  EXPECT_TRUE(to_manual->success) << to_manual->message;
  EXPECT_TRUE(is_active(kToolController)) << "an arm mode change deactivated the tool claim";

  const auto to_idle = request(SupervisorStatus::MODE_IDLE);
  ASSERT_NE(to_idle, nullptr);
  EXPECT_TRUE(is_active(kToolController)) << "releasing the arm claim released the tool claim";

  // And it is reported, so a caller can see that an arm motion and a gripper
  // action are two claims on the same pump rather than one machine.
  EXPECT_NE(to_idle->message.find("Tool claim"), std::string::npos) << to_idle->message;
  EXPECT_NE(to_idle->message.find(kToolController), std::string::npos) << to_idle->message;
}

TEST_F(ModeSwitch, MpcIsRefusedBeforeTheTrajectoryControllerIsDeactivated)
{
  // PRD §10 step 2 and user story 35, against a real manager: freshness is
  // verified before anything is deactivated, nothing is publishing
  // `/crane/mpc/solver_health` in this harness, and the refusal therefore
  // leaves the machine in the mode it was already in.
  //
  // What is asserted here is the *outcome* against a real controller manager.
  // That the switch service was never reached at all, and that the freshness
  // check ran before this node consulted the manager for anything, needs a
  // manager that can be watched and is `test_handover_sequence.cpp`.
  clear_the_starting_latch();
  ASSERT_NE(request(SupervisorStatus::MODE_FOLLOW), nullptr);
  ASSERT_TRUE(is_active(kFollowController));

  const auto refused = request(SupervisorStatus::MODE_MPC);
  ASSERT_NE(refused, nullptr);
  EXPECT_FALSE(refused->success);
  EXPECT_NE(refused->message.find("freshness"), std::string::npos) << refused->message;
  EXPECT_NE(refused->message.find("Nothing was deactivated"), std::string::npos)
    << refused->message;
  EXPECT_EQ(refused->active_mode, SupervisorStatus::MODE_FOLLOW) << refused->message;
  EXPECT_TRUE(is_active(kFollowController)) << "a refused switch deactivated the active mode";
}

TEST_F(ModeSwitch, ALatchedFaultRefusesAMotionModeAndIdleStaysReachable)
{
  // A freshly started supervisor is in the latched stop of §6.1: nothing is
  // arriving, and absence of the stop signal is asserted rather than released.
  // No remote is published here at all, so the latch stands.
  ASSERT_TRUE(drive_until([this]() {return set_mode_->service_is_ready();}));

  const auto refused = request(SupervisorStatus::MODE_FOLLOW);
  ASSERT_NE(refused, nullptr);
  EXPECT_FALSE(refused->success);
  EXPECT_NE(refused->message.find("latched"), std::string::npos) << refused->message;
  EXPECT_NE(refused->message.find("/crane/clear_fault"), std::string::npos) << refused->message;
  EXPECT_FALSE(is_active(kFollowController));

  // Releasing the claim is still reachable, which is the one direction a
  // latched stop does not argue against.
  const auto to_idle = request(SupervisorStatus::MODE_IDLE);
  ASSERT_NE(to_idle, nullptr);
  EXPECT_TRUE(to_idle->success) << to_idle->message;

  // And once the stop is acknowledged the same request goes through, so the
  // latch was the refusal and not a broken deployment.
  clear_the_starting_latch();
  const auto accepted = request(SupervisorStatus::MODE_FOLLOW);
  ASSERT_NE(accepted, nullptr);
  EXPECT_TRUE(accepted->success) << accepted->message;
  EXPECT_TRUE(is_active(kFollowController));
}

TEST_F(ModeSwitch, TheModeFollowsTheWorldWhenTheWorldChangesBehindTheSupervisor)
{
  // The acceptance criterion this issue writes against a supervisor whose model
  // drifts: `mode` is re-derived from the manager, so a controller that goes
  // down without a mode change is visible on the next status cycle rather than
  // hidden behind the last successful request.
  clear_the_starting_latch();
  ASSERT_NE(request(SupervisorStatus::MODE_FOLLOW), nullptr);
  ASSERT_TRUE(drive_with_remote_until([this]() {return saw_mode(SupervisorStatus::MODE_FOLLOW);}));

  // Taken down outside the supervisor entirely -- which is exactly what a
  // controller crashing, or somebody else's spawner, looks like from here.
  ASSERT_EQ(
    switch_outside_the_supervisor({}, {kFollowController}),
    controller_interface::return_type::OK);

  ASSERT_TRUE(drive_with_remote_until([this]() {return saw_mode(SupervisorStatus::MODE_IDLE);}))
    << latest_->message;
}

TEST_F(ModeSwitch, EveryAnswerCarriesACauseAndNeverAnEmptySuccess)
{
  // ROS 2 Interfaces §1 and PRD user story 36, on the wire: a refused switch
  // that leaves an operator with an unexplained no-op is the failure this
  // criterion is written against.
  clear_the_starting_latch();
  for (const std::uint8_t mode : {SupervisorStatus::MODE_IDLE, SupervisorStatus::MODE_MANUAL,
      SupervisorStatus::MODE_FOLLOW, SupervisorStatus::MODE_MPC, std::uint8_t{200}})
  {
    const auto answer = request(mode);
    ASSERT_NE(answer, nullptr) << static_cast<int>(mode);
    EXPECT_FALSE(answer->message.empty()) << static_cast<int>(mode);
    // And the mode it reports is the one the manager is in, whichever way the
    // request went.
    const std::uint8_t truth = is_active(kFollowController)
      ? SupervisorStatus::MODE_FOLLOW
      : (is_active(kManualController) ? SupervisorStatus::MODE_MANUAL
                                      : SupervisorStatus::MODE_IDLE);
    EXPECT_EQ(answer->active_mode, truth) << answer->message;
  }
}
