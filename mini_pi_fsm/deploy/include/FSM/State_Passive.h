// Adapted from unitree_rl_mjlab/deploy/include/FSM/State_Passive.h.
//
// Same intent (zero stiffness + damping, track the measured position), but the
// gains are NOT Unitree's. Mini-Pi uses kp = 0, kd = 1 by default because that
// is exactly what the HighTorque SDK's own protect path does --
// hightorque::HtdwMotor::protectMotor() in sim2real_sdk/src/motor/htdw_motor.cpp
// calls setMotor(0, 0, 0, kp=0, ki=0, kd=1, ...). See docs/source_mapping.md.
#pragma once

#include "FSM/FSMState.h"

namespace mini_pi
{

class State_Passive : public FSMState
{
public:
    State_Passive(int state, std::string state_string, ControlContext* ctx);

    void enter() override;
    void run() override;

private:
    JointArray kd_{};
};

MINI_PI_REGISTER_FSM(State_Passive)

} // namespace mini_pi
