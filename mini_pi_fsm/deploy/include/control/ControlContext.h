// ControlContext -- the shared, non-owning view that FSM states get.
// Ownership lives in main.cpp; the context only hands out references.
// This replaces Unitree's approach of static FSMState::lowcmd / lowstate
// globals (deploy/include/FSM/FSMState.h), which is the one Unitree design
// decision this package deliberately does not copy.
#pragma once

#include "control/JointMapper.h"
#include "control/SafetyGate.h"
#include "control/SafetyManager.h"
#include "control/Types.h"
#include "hardware/HardwareInterface.h"
#include "input/InputManager.h"
#include "policy/PolicyRunner.h"

#include <mutex>
#include <string>

namespace mini_pi
{

struct ControlContext
{
    /// Whichever backend main() selected: HighTorqueHardware for the real
    /// Mini-Pi, SimHardware for the MuJoCo bridge. FSM states must only ever
    /// use the HardwareInterface surface, so that the same state code drives
    /// both -- this is the one place the choice is visible at all.
    HardwareInterface*  hardware = nullptr;
    JointMapper*        mapper   = nullptr;
    SafetyManager*      safety   = nullptr;
    InputManager*       input    = nullptr;
    PolicyRunner*       policy   = nullptr;

    /// The single exit from the FSM to the low-level bus, and the owner of the
    /// runtime fault latch. Replaces the former `last_fault` SafetyReport and
    /// the State_Fault FSM state: detection still lives in SafetyManager, the
    /// RESPONSE now lives below the FSM rather than inside it.
    SafetyGate*         gate     = nullptr;

    /// Latest fused state, written once per control cycle by CtrlFSM.
    RobotState* state = nullptr;
    /// Command being built this cycle; states write here and nowhere else.
    RobotCommand* command = nullptr;

    /// Guards `state` for readers on the policy thread.
    std::mutex* state_mutex = nullptr;

    double control_dt = 0.001;

    bool valid() const
    {
        return hardware && mapper && safety && input && policy && gate &&
               state && command && state_mutex;
    }
};

} // namespace mini_pi
