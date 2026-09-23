#include "FSM/State_RLBase.h"

#include <ros/ros.h>

namespace mini_pi
{

State_RLBase::State_RLBase(int state, std::string state_string, ControlContext* ctx)
    : FSMState(state, state_string, ctx)
{
    bail_kd_.fill(1.0);

    if (!cfg_) throw std::runtime_error("State_RLBase: FSM." + state_string + " block missing");

    if (cfg_["action_timeout_s"])
    {
        action_timeout_s_ = cfg_["action_timeout_s"].as<double>();
        action_timeout_from_config_ = true;
        if (!(action_timeout_s_ > 0.0))
            throw std::runtime_error("State_RLBase: action_timeout_s must be > 0");
    }
    if (cfg_["bail_kd"])
    {
        const auto v = cfg_["bail_kd"].as<std::vector<double>>();
        if (v.size() != MINI_PI_DOF)
            throw std::runtime_error("State_RLBase: FSM." + getStateString() +
                                     ".bail_kd has the wrong length");
        for (std::size_t i = 0; i < MINI_PI_DOF; ++i) bail_kd_[i] = v[i];
    }

    // Highest-priority guard: leave immediately once RL has bailed. Inserted
    // at the front so it wins over the transitions declared in fsm.yaml.
    registered_checks.emplace(
        registered_checks.begin(),
        std::make_pair([this] { return bail_; },
                       FSMStateRegistry::instance().id("Passive")));
}

void State_RLBase::enter()
{
    bail_ = false;
    got_first_target_ = false;
    last_seq_ = 0;

    PolicyRunner* policy = ctx_->policy;

    if (!policy->ready())
    {
        // CtrlFSM normally rejects the transition before we get here; this is
        // the backstop for a direct entry.
        bail_ = true;
        ROS_ERROR("State_RLBase: refusing to enter RL -- %s. Returning to Passive.",
                  policy->unavailableReason().c_str());
        return;
    }

    if (!action_timeout_from_config_)
    {
        // Three policy periods: one missed step is tolerated, two are not.
        action_timeout_s_ = 3.0 * static_cast<double>(policy->stepDt());
    }

    // The policy thread reads the FSM's RobotState, which the FSM thread
    // rewrites every millisecond. The provider takes the same mutex CtrlFSM
    // uses, so one policy step always sees one consistent snapshot.
    ControlContext* ctx = ctx_;
    policy->setStateProvider([ctx](RobotState& s, VelocityCommand& c) {
        {
            std::lock_guard<std::mutex> lk(*ctx->state_mutex);
            s = *ctx->state;
        }
        c = ctx->input->velocityCommand();
    });

    policy->resetEnvironment();

    if (!policy->start())
    {
        bail_ = true;
        ROS_ERROR("State_RLBase: policy thread did not start. Returning to Passive.");
        return;
    }

    ROS_INFO("State_RLBase: policy thread started at %.1f Hz (step_dt %.4f s), "
             "action timeout %.3f s. Package: %s",
             policy->stepDt() > 0.0f ? 1.0 / policy->stepDt() : 0.0,
             static_cast<double>(policy->stepDt()), action_timeout_s_,
             policy->package().dir.c_str());
}

void State_RLBase::run()
{
    RobotCommand& cmd = *ctx_->command;

    if (bail_)
    {
        // Damping-only hold for the cycle(s) before the guard moves us out.
        // Deliberately not a position target: the last policy target may be
        // exactly what went wrong.
        holdDamping(cmd, bail_kd_);
        stampValid(cmd);
        return;
    }

    PolicyRunner* policy = ctx_->policy;
    const PolicyRunner::Target t = policy->latest();

    if (!t.valid)
    {
        // Normal for the first few milliseconds: the FSM runs at 1 kHz and the
        // policy at 50 Hz, so up to 20 cycles pass before the first target.
        // Damping-hold rather than commanding anything invented.
        holdDamping(cmd, bail_kd_);
        stampValid(cmd);

        if (policy->failed())
        {
            bail_ = true;
            ROS_ERROR("State_RLBase: policy thread failed before producing a command (%s). "
                      "Returning to Passive.", policy->failureReason().c_str());
        }
        return;
    }

    const double age = age_seconds(t.stamp);
    if (age > action_timeout_s_)
    {
        bail_ = true;
        // Built as a named string: a .c_str() on a temporary inside the
        // argument list would dangle before ROS_ERROR read it.
        const std::string why = policy->failed()
            ? " -- policy thread aborted: " + policy->failureReason()
            : std::string();
        ROS_ERROR("State_RLBase: policy command is %.4f s old (timeout %.4f s)%s. "
                  "Returning to Passive.", age, action_timeout_s_, why.c_str());
        holdDamping(cmd, bail_kd_);
        stampValid(cmd);
        return;
    }

    if (!got_first_target_)
    {
        got_first_target_ = true;
        ROS_INFO("State_RLBase: first policy command received (seq %lu, age %.4f s)",
                 static_cast<unsigned long>(t.seq), age);
    }
    last_seq_ = t.seq;

    // The command is used EXACTLY as the policy layer produced it -- q, kp and
    // kd from the policy package, dq and tau zero. It is stamped here, on the
    // FSM thread, and then goes through FSMState::post_run(), i.e. through
    // SafetyManager::clampCommand + checkCommand and JointMapper, like every
    // other state's command. RL has no separate path to the hardware.
    cmd.q      = t.cmd.q;
    cmd.dq     = t.cmd.dq;
    cmd.torque = t.cmd.torque;
    cmd.kp     = t.cmd.kp;
    cmd.kd     = t.cmd.kd;
    stampValid(cmd);
}

void State_RLBase::exit()
{
    PolicyRunner* policy = ctx_->policy;
    const std::uint64_t steps = policy->steps();
    const double period = policy->measuredPeriod();

    policy->stop();
    policy->resetEnvironment();

    ROS_INFO("State_RLBase: policy thread stopped after %lu steps (mean period %.4f s, "
             "%.1f Hz)", static_cast<unsigned long>(steps), period,
             period > 0.0 ? 1.0 / period : 0.0);
}

} // namespace mini_pi
