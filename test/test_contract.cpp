
#include <gtest/gtest.h>

#include <cstdint>

#include "crane_model/testing/mock_model.hpp"
#include "crane_msgs/msg/solver_health.hpp"
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

TEST(CraneSupervisorContract, TheSolveVerdictIsNumberedAsTheProducersOwnMessageNumbersIt)
{
  using Wire = crane_msgs::msg::SolverHealth;
  using Outcome = crane_supervisor::SolveOutcome;

  EXPECT_EQ(static_cast<std::uint8_t>(Outcome::Unknown), Wire::SOLVE_UNKNOWN);
  EXPECT_EQ(static_cast<std::uint8_t>(Outcome::Converged), Wire::SOLVE_CONVERGED);
  EXPECT_EQ(static_cast<std::uint8_t>(Outcome::BudgetExceeded), Wire::SOLVE_BUDGET_EXCEEDED);
  EXPECT_EQ(static_cast<std::uint8_t>(Outcome::Failed), Wire::SOLVE_FAILED);

  EXPECT_EQ(static_cast<std::uint8_t>(crane_supervisor::HorizonReport{}.outcome),
    Wire::SOLVE_UNKNOWN);
  EXPECT_EQ(Wire{}.outcome, Wire::SOLVE_UNKNOWN);

  EXPECT_EQ(
    static_cast<std::uint8_t>(crane_supervisor::Fault::Solver),
    crane_msgs::msg::SupervisorStatus::FAULT_SOLVER);
}

TEST(CraneSupervisorContract, TheSettledPredicateIsNumberedAsItsOwnMessageNumbersIt)
{
  using Wire = crane_msgs::msg::SwaySettled;
  using Predicate = crane_supervisor::SwaySettled;

  EXPECT_EQ(static_cast<std::uint8_t>(Predicate::Unknown), Wire::SETTLED_UNKNOWN);
  EXPECT_EQ(static_cast<std::uint8_t>(Predicate::NotSettled), Wire::SETTLED_NO);
  EXPECT_EQ(static_cast<std::uint8_t>(Predicate::Settled), Wire::SETTLED_YES);

  EXPECT_EQ(
    static_cast<std::uint8_t>(crane_supervisor::SwayState{}.settled), Wire::SETTLED_UNKNOWN);
  EXPECT_EQ(Wire{}.settled, Wire::SETTLED_UNKNOWN);
}

TEST(CraneSupervisorContract, TheRealCoreStartsFromAbsenceRatherThanFromHealth)
{
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
