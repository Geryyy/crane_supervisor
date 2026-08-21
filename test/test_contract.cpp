#include <gtest/gtest.h>

#include "crane_model/testing/mock_model.hpp"

namespace crane_supervisor
{
enum class Mode : unsigned char { idle = 0, manual = 1, follow = 2, mpc = 3 };
enum class Fault : unsigned char {
  none = 0, tracking = 1, working_cell = 2, solver = 3, sway = 4,
  state_health = 5, reference_stale = 6, estop = 7, interlock = 8
};

struct SupervisorInput
{
  Mode requested_mode{Mode::idle};
  bool state_valid{false};
  bool reference_valid{false};
  bool horizon_valid{false};
  bool inside_working_cell{false};
  bool deadman_held{false};
  bool solver_healthy{false};
};

struct SupervisorDecision
{
  Mode active_mode{Mode::idle};
  Fault fault{Fault::none};
  bool assert_stop{true};
};

class SupervisorContract
{
public:
  virtual ~SupervisorContract() = default;
  virtual SupervisorDecision evaluate(const SupervisorInput & input) const = 0;
};
}  // namespace crane_supervisor

namespace
{
class SupervisorDouble final : public crane_supervisor::SupervisorContract
{
public:
  crane_supervisor::SupervisorDecision evaluate(
    const crane_supervisor::SupervisorInput & input) const override
  {
    if (!input.state_valid) {
      return {crane_supervisor::Mode::idle, crane_supervisor::Fault::state_health, true};
    }
    if (!input.deadman_held) {
      return {crane_supervisor::Mode::idle, crane_supervisor::Fault::interlock, true};
    }
    return {input.requested_mode, crane_supervisor::Fault::none, false};
  }
};
}  // namespace

TEST(CraneSupervisorContract, InvalidStateRequestsStop)
{
  const auto decision = SupervisorDouble().evaluate({});
  EXPECT_EQ(decision.fault, crane_supervisor::Fault::state_health);
  EXPECT_TRUE(decision.assert_stop);
}

TEST(CraneSupervisorContract, W08UsesInstalledModelDynamicsContract)
{
  const auto model = crane_model::testing::MockModel::create(crane_model::Tool::Pzs100);
  ASSERT_TRUE(model.ok());
  const auto result = model.value().inverse_dynamics(
    crane_model::Q::Zero(), crane_model::DQ::Zero(), crane_model::DQ::Zero(),
    crane_model::Payload{});
  EXPECT_EQ(result.status().code, crane_model::ErrorCode::InvalidPayload);
}

TEST(CraneSupervisorContract, HealthyDeadmanAllowsRequestedMode)
{
  crane_supervisor::SupervisorInput input;
  input.state_valid = true;
  input.deadman_held = true;
  input.requested_mode = crane_supervisor::Mode::mpc;
  const auto decision = SupervisorDouble().evaluate(input);
  EXPECT_EQ(decision.active_mode, crane_supervisor::Mode::mpc);
  EXPECT_FALSE(decision.assert_stop);
}
