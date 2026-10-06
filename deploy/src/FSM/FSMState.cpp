#include "FSM/FSMState.h"

#include <ros/ros.h>

namespace deploy
{

namespace fsm_config { YAML::Node node; }

FSMState::FSMState(int state, std::string state_string, ControlContext* ctx)
    : BaseState(state, state_string, ctx)
{
    ROS_INFO("Initializing State_%s ...", state_string.c_str());
    if (fsm_config::node["FSM"]) cfg_ = fsm_config::node["FSM"][state_string];
    registerTransitions();
}

void FSMState::registerTransitions()
{
    // Named triggers, resolved against InputManager. Deliberately a small
    // closed set rather than a DSL.
    const auto trigger = [this](const std::string& name) -> std::function<bool()> {
        InputManager* in = ctx_->input;
        if (name == "passive") return [in] { return in->requestPassive(); };
        if (name == "stand")   return [in] { return in->requestStand(); };
        if (name == "rl")      return [in] { return in->requestRL(); };
        if (name == "reset")   return [in] { return in->requestReset(); };
        if (name == "always")  return [] { return true; };
        return nullptr;
    };

    if (cfg_ && cfg_["transitions"])
    {
        for (const auto& it : cfg_["transitions"])
        {
            const std::string target = it.first.as<std::string>();
            const std::string cond   = it.second.as<std::string>();

            if (!FSMStateRegistry::instance().hasName(target))
            {
                ROS_WARN("FSM State_'%s' is not enabled; ignoring transition from %s",
                         target.c_str(), getStateString().c_str());
                continue;
            }
            auto fn = trigger(cond);
            if (!fn)
            {
                ROS_ERROR("FSM: unknown transition trigger '%s' (state %s -> %s). "
                          "Valid: passive, stand, rl, reset, always.",
                          cond.c_str(), getStateString().c_str(), target.c_str());
                continue;
            }
            registered_checks.emplace_back(std::move(fn),
                                           FSMStateRegistry::instance().id(target));
        }
    }

    // Registered for every state, and the direct counterpart of Unitree's
    // unconditional `lowstate->isTimeout() -> Passive` guard
    // (unitree_rl_mjlab/deploy/include/FSM/FSMState.h:48-53): a latched runtime
    // fault drives the FSM to the safe OPERATING mode.
    //
    // This is only half the reaction. Safety::publish independently overrides the
    // transmitted command while the latch is held, so reaching Passive is not
    // what makes the robot safe -- it is what makes sure that, after the
    // operator clears the latch, what resumes is Passive and never the
    // previous RL action.
    if (FSMStateRegistry::instance().hasName("Passive") && getStateString() != "Passive")
    {
        auto* safety = ctx_->safety;
        registered_checks.emplace_back([safety] { return safety->faulted(); },
                                       FSMStateRegistry::instance().id("Passive"));
    }
}

void FSMState::pre_run()
{
    ctx_->input->update();
}

void FSMState::post_run()
{
    // The FSM's whole output contract is this one line: hand the RobotCommand
    // to Safety::publish. Clamping, the command checks, the fault latch, the safe
    // override and JointMapper all live below the FSM now, so no state -- this
    // one or any added later -- has a path to the hardware that skips them.
    ctx_->safety->publish(*ctx_->command, *ctx_);
}

void FSMState::holdCurrentPosition(RobotCommand& cmd, double kp, double kd) const
{
    const RobotState& s = *ctx_->state;
    for (std::size_t j = 0; j < NUM_DOF; ++j)
    {
        cmd.q[j]      = s.robot_q[j];
        cmd.dq[j]     = 0.0;
        cmd.torque[j] = 0.0;
        cmd.kp[j]     = kp;
        cmd.kd[j]     = kd;
    }
}

void FSMState::holdDamping(RobotCommand& cmd, const JointArray& kd) const
{
    const RobotState& s = *ctx_->state;
    for (std::size_t j = 0; j < NUM_DOF; ++j)
    {
        // The motor runs POS_VEL_TQE_KP_KD2, so a position field is always
        // transmitted. With kp == 0 it has no effect, but we still send the
        // measured pose so a later transition starts from a consistent target.
        cmd.q[j]      = s.robot_q[j];
        cmd.dq[j]     = 0.0;
        cmd.torque[j] = 0.0;
        cmd.kp[j]     = 0.0;
        cmd.kd[j]     = kd[j];
    }
}

void FSMState::stampValid(RobotCommand& cmd) const
{
    cmd.valid = true;
    cmd.stamp = Clock::now();
    ++cmd.seq;
}

} // namespace deploy
