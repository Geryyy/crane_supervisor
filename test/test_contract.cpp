// The contracts this package has with packages outside it: the installed
// `crane_model` dynamics seam, the numbering of `crane_msgs/SupervisorStatus`,
// and which of the twelve booleans of `epsilon_crane_msgs/RemoteCtrlStates` the
// configured deadman selects.
//
// Message structs only -- no node is constructed, no clock read, no graph
// joined, and the adapter is linked for one free function.  The mode/fault fixture
// that used to live in this file as a private test double is gone: the real
// decision core replaces it, and it is exercised in `test_supervisor_core.cpp`.
// A double beside the implementation would be a second answer to the same
// question, and the whole reason the core is ROS-free is that the real one is
// reachable from a test without a runtime.

#include <gtest/gtest.h>

#include <cstdint>

#include "crane_model/testing/mock_model.hpp"
#include "crane_msgs/msg/supervisor_status.hpp"
#include "crane_msgs/msg/sway_settled.hpp"
#include "crane_supervisor/supervisor_core.hpp"
#include "crane_supervisor/supervisor_node.hpp"
#include "epsilon_crane_msgs/msg/remote_ctrl_states.hpp"

TEST(CraneSupervisorContract, W08UsesInstalledModelDynamicsContract)
{
  const auto model = crane_model::testing::MockModel::create(crane_model::Tool::Pzs100);
  ASSERT_TRUE(model.ok());
  const auto result = model.value().inverse_dynamics(
    crane_model::Q::Zero(), crane_model::DQ::Zero(), crane_model::DQ::Zero(),
    crane_model::Payload{});
  EXPECT_EQ(result.status().code, crane_model::ErrorCode::InvalidPayload);
}

TEST(CraneSupervisorContract, TheCoreIsNumberedAsTheMessageIsNumbered)
{
  // The core is ROS-free and cannot say this about itself.  Here is where it is
  // said, so that the adapter above it stays a cast rather than a lookup table
  // that can disagree with the wire.
  using Status = crane_msgs::msg::SupervisorStatus;
  using crane_supervisor::Fault;
  using crane_supervisor::Mode;

  EXPECT_EQ(static_cast<std::uint8_t>(Mode::Idle), Status::MODE_IDLE);
  EXPECT_EQ(static_cast<std::uint8_t>(Mode::Manual), Status::MODE_MANUAL);
  EXPECT_EQ(static_cast<std::uint8_t>(Mode::Follow), Status::MODE_FOLLOW);
  EXPECT_EQ(static_cast<std::uint8_t>(Mode::Mpc), Status::MODE_MPC);

  EXPECT_EQ(static_cast<std::uint8_t>(Fault::None), Status::FAULT_NONE);
  EXPECT_EQ(static_cast<std::uint8_t>(Fault::Tracking), Status::FAULT_TRACKING);
  EXPECT_EQ(static_cast<std::uint8_t>(Fault::WorkingCell), Status::FAULT_WORKING_CELL);
  EXPECT_EQ(static_cast<std::uint8_t>(Fault::Solver), Status::FAULT_SOLVER);
  EXPECT_EQ(static_cast<std::uint8_t>(Fault::Sway), Status::FAULT_SWAY);
  EXPECT_EQ(static_cast<std::uint8_t>(Fault::StateHealth), Status::FAULT_STATE_HEALTH);
  EXPECT_EQ(static_cast<std::uint8_t>(Fault::ReferenceStale), Status::FAULT_REFERENCE_STALE);
  EXPECT_EQ(static_cast<std::uint8_t>(Fault::EStop), Status::FAULT_ESTOP);
  EXPECT_EQ(static_cast<std::uint8_t>(Fault::Interlock), Status::FAULT_INTERLOCK);
  EXPECT_EQ(static_cast<std::uint8_t>(Fault::NotCommissioned), Status::FAULT_NOT_COMMISSIONED);
}

TEST(CraneSupervisorContract, TheSettledPredicateIsNumberedAsItsOwnMessageNumbersIt)
{
  // Same argument as the modes and the faults above, for the second stream: the
  // core is ROS-free and cannot say this about itself, so the adapter's cast is
  // held to the wire here rather than becoming a table that can drift from it.
  using Wire = crane_msgs::msg::SwaySettled;
  using Predicate = crane_supervisor::SwaySettled;

  EXPECT_EQ(static_cast<std::uint8_t>(Predicate::Unknown), Wire::SETTLED_UNKNOWN);
  EXPECT_EQ(static_cast<std::uint8_t>(Predicate::NotSettled), Wire::SETTLED_NO);
  EXPECT_EQ(static_cast<std::uint8_t>(Predicate::Settled), Wire::SETTLED_YES);

  // And the value both sides start from is the unknown one.  A core whose dwell
  // began at `Settled`, or a message whose zero meant it, would answer "the load
  // is hanging still" before anything had been observed at all -- which is the
  // one answer the three-valued design exists to refuse.
  EXPECT_EQ(
    static_cast<std::uint8_t>(crane_supervisor::SwayState{}.settled), Wire::SETTLED_UNKNOWN);
  EXPECT_EQ(Wire{}.settled, Wire::SETTLED_UNKNOWN);
}

TEST(CraneSupervisorContract, TheRealCoreStartsFromAbsenceRatherThanFromHealth)
{
  // The decision a supervisor makes before anything has told it anything, in
  // the wire's own constants as well as in the enum `test_supervisor_core.cpp`
  // uses.  It is FAULT_ESTOP rather than the FAULT_STATE_HEALTH this asserted
  // while the remote was not yet an input: with nothing arriving at all, the
  // stop signal is among the things that are not arriving, and
  // wiki/control_architecture.md §6.1 reads absence of the stop signal as
  // asserted rather than as released.  A supervisor that started from a stale
  // estimate instead would have started from the lesser of the two absences.
  const auto decision = crane_supervisor::decide(crane_supervisor::SupervisorConfig{}, {});
  EXPECT_EQ(
    static_cast<std::uint8_t>(decision.fault), crane_msgs::msg::SupervisorStatus::FAULT_ESTOP);
  EXPECT_TRUE(decision.estop_latched);
  EXPECT_FALSE(decision.deadman_held);
  EXPECT_EQ(
    static_cast<std::uint8_t>(decision.mode), crane_msgs::msg::SupervisorStatus::MODE_IDLE);
  EXPECT_FALSE(decision.message.empty());
}

TEST(CraneSupervisorContract, TheDeadmanIsReadOffTheFieldTheRetainedStackNames)
{
  // The number in the configuration is not the assertion; which *field* it
  // selects is.  `epsilon_crane_msgs/RemoteCtrlStates` is the retained
  // machine-telemetry boundary (ROS 2 Interfaces §9), and the retained approval
  // gate reads `button12` off it -- so the default must select `button12` and
  // nothing else.  This is the one place the ROS-free core's button number and
  // the wire's twelve booleans are checked against each other.
  epsilon_crane_msgs::msg::RemoteCtrlStates message;
  message.button12 = true;
  EXPECT_TRUE(
    crane_supervisor::deadman_of(message, crane_supervisor::SupervisorConfig{}.deadman_button));

  for (int button = crane_supervisor::kFirstButton; button < crane_supervisor::kLastButton;
    ++button)
  {
    EXPECT_FALSE(crane_supervisor::deadman_of(message, button)) << button;
  }

  message.button12 = false;
  EXPECT_FALSE(
    crane_supervisor::deadman_of(message, crane_supervisor::SupervisorConfig{}.deadman_button));
}
