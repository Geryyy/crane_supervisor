// The two contracts this package has with packages outside it: the installed
// `crane_model` dynamics seam, and the numbering of `crane_msgs/SupervisorStatus`.
//
// Message structs only -- no node, no clock, no graph.  The mode/fault fixture
// that used to live in this file as a private test double is gone: the real
// decision core replaces it, and it is exercised in `test_supervisor_core.cpp`.
// A double beside the implementation would be a second answer to the same
// question, and the whole reason the core is ROS-free is that the real one is
// reachable from a test without a runtime.

#include <gtest/gtest.h>

#include <cstdint>

#include "crane_model/testing/mock_model.hpp"
#include "crane_msgs/msg/supervisor_status.hpp"
#include "crane_supervisor/supervisor_core.hpp"

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

TEST(CraneSupervisorContract, TheRealCoreStartsFromAbsenceRatherThanFromHealth)
{
  // The decision a supervisor makes before anything has told it anything.  It
  // is the one the replaced double got right and the one every later cause is
  // added on top of, so it is asserted against the wire's own constants here as
  // well as against the enum in `test_supervisor_core.cpp`.
  const auto decision = crane_supervisor::decide(crane_supervisor::SupervisorConfig{}, {});
  EXPECT_EQ(
    static_cast<std::uint8_t>(decision.fault),
    crane_msgs::msg::SupervisorStatus::FAULT_STATE_HEALTH);
  EXPECT_EQ(
    static_cast<std::uint8_t>(decision.mode), crane_msgs::msg::SupervisorStatus::MODE_IDLE);
  EXPECT_FALSE(decision.message.empty());
}
