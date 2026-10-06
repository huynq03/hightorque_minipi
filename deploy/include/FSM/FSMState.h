// Adapted from unitree_rl_mjlab/deploy/include/FSM/FSMState.h.
//
// Kept: the constructor reads `FSM.<name>.transitions` from the config and
// turns each entry into a registered guard; pre_run() refreshes inputs;
// post_run() flushes the command; a global "go safe" guard is registered for
// every state.
// Replaced: Unitree's joystick DSL (`LT + up.on_pressed`) is not ported. The
// transitions name an InputManager request instead, which is readable
// straight from fsm.yaml and needs no parser. See doc/architecture.md.
#pragma once

#include "FSM/BaseState.h"

#include <yaml-cpp/yaml.h>

namespace deploy
{

class FSMState : public BaseState
{
public:
    FSMState(int state, std::string state_string, ControlContext* ctx);

    /// Refresh operator input. The state snapshot itself is refreshed by
    /// CtrlFSM before the state runs, so every state sees the same sample.
    void pre_run() override;

    /// Hand the command to Safety::publish, which validates, clamps, maps and
    /// transmits it -- or overrides it while a runtime fault is latched.
    void post_run() override;

protected:
    /// Damping-only: kp = 0, per-joint kd, target = measured q, dq = tau = 0.
    /// This is the safe fallback every state uses; it is the same shape as
    /// hightorque::HtdwMotor::protectMotor() but with configurable kd.
    void holdDamping(RobotCommand& cmd, const JointArray& kd) const;

    /// Hold the measured pose with an explicit kp/kd pair.
    void holdCurrentPosition(RobotCommand& cmd, double kp, double kd) const;

    /// Stamps the command as produced now and bumps its sequence number.
    /// Every state must call this on every cycle it intends to command;
    /// a command that is not restamped goes stale and trips CommandTimeout.
    void stampValid(RobotCommand& cmd) const;

    /// Per-state config node (`FSM.<name>` in fsm.yaml). May be undefined.
    YAML::Node cfg_;

private:
    void registerTransitions();
};

/// The fsm.yaml document, loaded once by main() before the FSM is built.
namespace fsm_config
{
extern YAML::Node node;
} // namespace fsm_config

} // namespace deploy
