// State_RLBase -- the RL FSM state (the `Velocity` state in fsm.yaml).
//
// Structural counterpart of unitree_rl_mjlab/deploy/include/FSM/State_RLBase.h,
// and deliberately as thin as that one:
//
//   enter()  verify the policy is ready, reset it, start the policy thread
//   run()    take RLPolicyRunner::latest(), check its age, publish its command --
//            the same target is re-published every 1 kHz tick until the
//            50 Hz policy thread replaces it (zero-order hold)
//   exit()   stop and join the policy thread, reset the policy
//
// There is NO observation packing, NO history handling, NO ONNX call, NO action
// scale and NO joint remapping in this file. All of that lives behind
// RLPolicyRunner and is driven by the policy package's deploy.yaml. Adding a new
// policy should touch this file not at all.
//
// The gains are not here either: they come from the policy package and arrive
// already written into PolicyOutput::cmd. `bail_kd` below is the only
// gain this state owns, and it is not an RL gain -- it is the damping used
// while refusing or bailing out.
#pragma once

#include "FSMState.h"

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

    /// The policy must publish its FIRST output within this long after
    /// enter(); otherwise the state damps and returns to Passive. Catches a
    /// policy thread that is alive but stuck (or refusing) before its first
    /// step. fsm.yaml `first_output_timeout_s`, default kDefaultFirstOutputS.
    static constexpr double kDefaultFirstOutputS = 0.5;
    double first_output_timeout_s_ = kDefaultFirstOutputS;
    TimePoint entered_at_{};

    /// Set whenever RL cannot continue; drives the guard back to Passive.
    bool bail_ = false;
    /// Suppresses repeated "first target" logging.
    bool got_first_target_ = false;
};

MINI_PI_REGISTER_FSM(State_RLBase)

} // namespace mini_pi
