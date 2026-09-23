// State_RLBase -- the RL FSM state.
//
// Structural counterpart of unitree_rl_mjlab/deploy/include/FSM/State_RLBase.h,
// and deliberately as thin as that one:
//
//   enter()  verify the policy is ready, reset it, start the policy thread
//   run()    take the newest processed RobotCommand, check its age, publish it
//   exit()   stop and join the policy thread, reset the policy runtime
//
// There is NO observation packing, NO history handling, NO ONNX call, NO action
// scale and NO joint remapping in this file. All of that lives below
// PolicyRunner, in rl::PolicyEnvironment and its managers, and is driven by the
// policy package's deploy.yaml. Adding a new policy should touch this file not
// at all.
//
// The gains are not here either: they come from the policy package
// (deploy.yaml `stiffness`/`damping`) and arrive already written into the
// RobotCommand that PolicyEnvironment produced. `bail_kd` below is the only
// gain this state owns, and it is not an RL gain -- it is the damping used
// while refusing or bailing out.
#pragma once

#include "FSM/FSMState.h"

namespace mini_pi
{

class State_RLBase : public FSMState
{
public:
    State_RLBase(int state, std::string state_string, ControlContext* ctx);

    void enter() override;
    void run() override;
    void exit() override;

private:
    /// Damping applied while refusing or bailing out. NOT an RL gain.
    JointArray bail_kd_{};

    /// A Target older than this is not trusted. Defaults to 3 policy periods,
    /// computed in enter() once step_dt is known, so the value tracks the
    /// policy rate instead of being a magic constant. `action_timeout_s` in
    /// fsm.yaml overrides it.
    double action_timeout_s_ = 0.0;
    bool action_timeout_from_config_ = false;

    /// Set whenever RL cannot continue; drives the guard back to Passive.
    bool bail_ = false;
    /// Suppresses repeated "first target" logging.
    bool got_first_target_ = false;
    std::uint64_t last_seq_ = 0;
};

MINI_PI_REGISTER_FSM(State_RLBase)

} // namespace mini_pi
