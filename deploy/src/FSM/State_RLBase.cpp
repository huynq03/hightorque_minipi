#include "FSM/State_RLBase.h"

#include <ros/ros.h>

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace deploy
{

State_RLBase::State_RLBase(int state, std::string state_string, ControlContext* ctx)
    : FSMState(state, state_string, ctx)
{
    bail_kd_.fill(1.0);

    if (!cfg_) throw std::runtime_error("State_RLBase: FSM." + state_string + " block missing");
    policy_ = ctx_->policyFor(state_string);
    if (!policy_) throw std::runtime_error("State_RLBase: no policy runner for " + state_string);

    if (cfg_["action_timeout_s"])
    {
        action_timeout_s_ = cfg_["action_timeout_s"].as<double>();
        action_timeout_from_config_ = true;
        if (!(action_timeout_s_ > 0.0))
            throw std::runtime_error("State_RLBase: action_timeout_s must be > 0");
    }
    if (cfg_["first_output_timeout_s"])
    {
        first_output_timeout_s_ = cfg_["first_output_timeout_s"].as<double>();
        if (!(first_output_timeout_s_ > 0.0))
            throw std::runtime_error("State_RLBase: first_output_timeout_s must be > 0");
    }
    if (cfg_["bail_kd"])
    {
        const auto v = cfg_["bail_kd"].as<std::vector<double>>();
        if (v.size() != NUM_DOF)
            throw std::runtime_error("State_RLBase: FSM." + getStateString() +
                                     ".bail_kd has the wrong length");
        for (std::size_t i = 0; i < NUM_DOF; ++i) bail_kd_[i] = v[i];
    }

    const auto num = [](const YAML::Node& n, const char* k, double& dst) {
        if (n && n[k]) dst = n[k].as<double>();
    };
    num(cfg_["still"], "max_gyro", lim_.still_gyro);
    num(cfg_["still"], "max_joint_speed", lim_.still_dq);
    num(cfg_["guard"], "max_excursion", lim_.guard_excursion);
    num(cfg_["guard"], "max_joint_speed", lim_.guard_dq);
    num(cfg_["episode"], "timeout_s", lim_.timeout_s);
    num(cfg_["episode"], "upright_gravity_z", lim_.upright_gz);
    num(cfg_["episode"], "hold_s", lim_.hold_s);
    if (cfg_["episode"] && cfg_["episode"]["on_success"])
    {
        const std::string to = cfg_["episode"]["on_success"].as<std::string>();
        if (!FSMStateRegistry::instance().hasName(to))
            throw std::runtime_error("State_RLBase: FSM." + state_string +
                                     ".episode.on_success names unknown state " + to);
        lim_.on_success = FSMStateRegistry::instance().id(to);
    }
    supervised_ = std::isfinite(lim_.guard_excursion) || std::isfinite(lim_.guard_dq) ||
                  std::isfinite(lim_.timeout_s) || lim_.on_success != 0;

    // Highest-priority guard: leave immediately once RL has bailed. Inserted
    // at the front so it wins over the transitions declared in fsm.yaml.
    registered_checks.emplace(
        registered_checks.begin(),
        std::make_pair([this] { return bail_; },
                       FSMStateRegistry::instance().id("Passive")));
    if (lim_.on_success != 0)
        registered_checks.emplace(registered_checks.begin() + 1,
                                  std::make_pair([this] { return success_; }, lim_.on_success));
}

namespace
{
template <class A>
double maxAbs(const A& a)
{
    double m = 0.0;
    for (double v : a) m = std::max(m, std::abs(v));
    return m;
}
} // namespace

bool State_RLBase::still_(const RobotState& s) const
{
    return maxAbs(s.angular_velocity) < lim_.still_gyro && maxAbs(s.robot_dq) < lim_.still_dq;
}

std::string State_RLBase::entryRefusal() const
{
    const RobotState& s = *ctx_->state;
    if (still_(s)) return {};
    char buf[128];
    std::snprintf(buf, sizeof(buf), "robot not still: |gyro| %.2f, |dq| %.2f (still: %.2f / %.2f)",
                  maxAbs(s.angular_velocity), maxAbs(s.robot_dq), lim_.still_gyro, lim_.still_dq);
    return buf;
}

void State_RLBase::supervise_()
{
    const RobotState& s = *ctx_->state;
    const JointArray& lo = policy_->targetLo();
    const JointArray& hi = policy_->targetHi();
    for (std::size_t i = 0; i < NUM_DOF && !bail_; ++i)
    {
        const double out = std::max(lo[i] - s.robot_q[i], s.robot_q[i] - hi[i]);
        if (out > lim_.guard_excursion || std::abs(s.robot_dq[i]) > lim_.guard_dq)
        {
            bail_ = true;
            ROS_ERROR("State_RLBase: joint %zu at %.3f rad, %.2f rad/s: %.3f rad outside the "
                      "target clip [%.3f, %.3f] (guard: %.2f rad, %.2f rad/s). Returning to Passive.",
                      i, s.robot_q[i], s.robot_dq[i], out, lo[i], hi[i], lim_.guard_excursion,
                      lim_.guard_dq);
        }
    }

    std::array<double, 3> g{};
    const bool settled = s.imu_valid && projected_gravity_xyzw(s.orientation, g) &&
                         g[2] < lim_.upright_gz && still_(s);
    if (!settled) settled_since_ = {};
    else if (settled_since_ == TimePoint{}) settled_since_ = Clock::now();
    if (settled && lim_.on_success != 0 && !success_ && age_seconds(settled_since_) >= lim_.hold_s)
    {
        success_ = true;
        ROS_INFO("State_RLBase: upright and still for %.2f s (%.2f s after entry) -> %s.",
                 lim_.hold_s, age_seconds(entered_at_),
                 FSMStateRegistry::instance().name(lim_.on_success).c_str());
    }
    // A robot already upright and still is in its hold, not late: let hold_s finish.
    if (!success_ && !bail_ && !settled && age_seconds(entered_at_) > lim_.timeout_s)
    {
        bail_ = true;
        ROS_ERROR("State_RLBase: not upright and still %.1f s after entry (episode.timeout_s). "
                  "Returning to Passive.", lim_.timeout_s);
    }
}

void State_RLBase::enter()
{
    bail_ = false;
    success_ = false;
    settled_since_ = {};
    got_first_target_ = false;
    entered_at_ = Clock::now();

    RLPolicyRunner* policy = policy_;

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

    policy->reset();

    if (!policy->start())
    {
        bail_ = true;
        ROS_ERROR("State_RLBase: policy thread did not start. Returning to Passive.");
        return;
    }

    ROS_INFO("State_RLBase: policy thread started at %.1f Hz (step_dt %.4f s), "
             "action timeout %.3f s, first output within %.3f s. Package: %s",
             policy->stepDt() > 0.0f ? 1.0 / policy->stepDt() : 0.0,
             static_cast<double>(policy->stepDt()), action_timeout_s_,
             first_output_timeout_s_, policy->package().dir.c_str());
}

void State_RLBase::run()
{
    RobotCommand& cmd = *ctx_->command;
    if (supervised_ && !bail_) supervise_();

    if (bail_)
    {
        // Damping-only hold for the cycle(s) before the guard moves us out.
        // Deliberately not a position target: the last policy target may be
        // exactly what went wrong.
        holdDamping(cmd, bail_kd_);
        stampValid(cmd);
        return;
    }

    RLPolicyRunner* policy = policy_;
    const PolicyOutput t = policy->latest();

    if (!t.valid)
    {
        // Normal for the first few milliseconds: the FSM runs at 1 kHz and the
        // policy at 50 Hz, so up to 20 cycles pass before the first target.
        // Damping-hold rather than commanding anything invented. Also what a
        // FAILED policy looks like: RLPolicyRunner withdraws its output on failure.
        holdDamping(cmd, bail_kd_);
        stampValid(cmd);

        if (policy->failed())
        {
            bail_ = true;
            ROS_ERROR("State_RLBase: policy thread failed %s(%s). Returning to Passive.",
                      got_first_target_ ? "" : "before producing a command ",
                      policy->failureReason().c_str());
        }
        else if (!got_first_target_ && age_seconds(entered_at_) > first_output_timeout_s_)
        {
            // Alive but silent: no inference result at all since entry.
            bail_ = true;
            ROS_ERROR("State_RLBase: no policy output %.3f s after entering RL (limit %.3f s, "
                      "fsm.yaml first_output_timeout_s). Returning to Passive.",
                      age_seconds(entered_at_), first_output_timeout_s_);
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

    // The command is used EXACTLY as the policy layer produced it -- q, kp and
    // kd from the policy package, dq and tau zero. It is stamped here, on the
    // FSM thread, and then goes through FSMState::post_run(), i.e. through
    // Safety::clampCommand + checkCommand and JointMapper, like every
    // other state's command. RL has no separate path to the hardware.
    // A relative action is re-based on the measured q every tick (see joint_actions.h).
    if (policy->relative())
        for (std::size_t i = 0; i < NUM_DOF; ++i)
            cmd.q[i] = std::clamp(ctx_->state->robot_q[i] + t.delta[i], policy->targetLo()[i],
                                  policy->targetHi()[i]);
    else
        cmd.q = t.cmd.q;
    cmd.dq     = t.cmd.dq;
    cmd.torque = t.cmd.torque;
    cmd.kp     = t.cmd.kp;
    cmd.kd     = t.cmd.kd;
    stampValid(cmd);
}

void State_RLBase::exit()
{
    RLPolicyRunner* policy = policy_;
    const std::uint64_t steps = policy->steps();
    const double period = policy->measuredPeriod();

    policy->stop();
    policy->reset();

    ROS_INFO("State_RLBase: policy thread stopped after %lu steps (mean period %.4f s, "
             "%.1f Hz)", static_cast<unsigned long>(steps), period,
             period > 0.0 ? 1.0 / period : 0.0);
}

} // namespace deploy
