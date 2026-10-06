// ControlContext -- the shared, non-owning view that FSM states get.
// Ownership lives in main.cpp; the context only hands out references.
// This replaces Unitree's approach of static FSMState::lowcmd / lowstate
// globals (unitree_rl_mjlab/deploy/include/FSM/FSMState.h), which is the one Unitree design
// decision this package deliberately does not copy.
#pragma once

#include "common/JointMapper.h"
#include "common/Safety.h"
#include "common/Types.h"
#include "hardware/HardwareInterface.h"
#include "input/InputManager.h"
#include "policy/RLPolicyRunner.h"

#include <mutex>
#include <string>

namespace deploy
{

struct ControlContext
{
    /// Whichever backend the robot's factory built: its real backend (e.g.
    /// Mini-Pi HighTorqueHardware), or SimHardware for the MuJoCo bridge. FSM states must only ever
    /// use the HardwareInterface surface, so that the same state code drives
    /// both -- this is the one place the choice is visible at all.
    HardwareInterface*  hardware = nullptr;
    JointMapper*        mapper   = nullptr;
    /// Every runtime safety rule, the fault latch, and the single exit from
    /// the FSM to the low-level bus (Safety::publish).
    Safety*             safety   = nullptr;
    InputManager*       input    = nullptr;
    RLPolicyRunner*     policy   = nullptr;

    /// Latest fused state, written once per control cycle by CtrlFSM.
    RobotState* state = nullptr;
    /// Command being built this cycle; states write here and nowhere else.
    RobotCommand* command = nullptr;

    /// Guards `state` for readers on the policy thread.
    std::mutex* state_mutex = nullptr;

    double control_dt = 0.001;

    /// One hardware read + motor->robot mapping into `*state`, under
    /// `state_mutex`. The only way RobotState is refreshed: startup readiness
    /// polls it, and CtrlFSM calls it at the top of every tick, so both judge
    /// the identical kind of snapshot.
    void readState()
    {
        RobotState& s = *state;
        std::lock_guard<std::mutex> lk(*state_mutex);
        hardware->read(s);
        mapper->motorToRobot(s.motor_q, s.motor_dq, s.motor_tau,
                             s.robot_q, s.robot_dq, s.robot_tau);
    }

    bool valid() const
    {
        return hardware && mapper && safety && input && policy &&
               state && command && state_mutex;
    }
};

} // namespace deploy
