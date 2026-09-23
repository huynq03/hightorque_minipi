#include "FSM/State_FixStand.h"

#include "control/LinearInterpolator.h"

#include <ros/ros.h>

#include <cstdio>
#include <sstream>

namespace mini_pi
{

State_FixStand::State_FixStand(int state, std::string state_string, ControlContext* ctx)
    : FSMState(state, state_string, ctx)
{
    kp_.fill(0.0);
    kd_.fill(1.0);

    if (!cfg_) throw std::runtime_error("State_FixStand: FSM.FixStand block missing from fsm.yaml");

    auto load_gain = [&](const char* key, JointArray& dst) {
        if (!cfg_[key])
            throw std::runtime_error(std::string("State_FixStand: FSM.FixStand.") + key +
                                     " missing");
        const auto v = cfg_[key].as<std::vector<double>>();
        if (v.size() != MINI_PI_DOF)
            throw std::runtime_error(std::string("State_FixStand: FSM.FixStand.") + key +
                                     " has the wrong length");
        for (std::size_t i = 0; i < MINI_PI_DOF; ++i) dst[i] = v[i];
    };
    load_gain("kp", kp_);
    load_gain("kd", kd_);

    // Configurable duration. `duration_s` is the simple form; `ts`/`qs` remain
    // available for a multi-keyframe move.
    if (cfg_["ts"])
    {
        ts_ = cfg_["ts"].as<std::vector<double>>();
    }
    else
    {
        const double d = cfg_["duration_s"] ? cfg_["duration_s"].as<double>() : 3.0;
        if (!(d > 0.0))
            throw std::runtime_error("State_FixStand: duration_s must be > 0");
        ts_ = {0.0, d};
    }

    if (ts_.size() < 2)
        throw std::runtime_error("State_FixStand: need at least two keyframe times");
    for (std::size_t k = 1; k < ts_.size(); ++k)
    {
        if (!(ts_[k] > ts_[k - 1]))
            throw std::runtime_error("State_FixStand: ts must be strictly increasing");
    }

    if (cfg_["qs"])
    {
        qs_ = cfg_["qs"].as<std::vector<std::vector<double>>>();
        if (qs_.size() != ts_.size())
            throw std::runtime_error("State_FixStand: ts and qs have different lengths");
        for (std::size_t k = 1; k < qs_.size(); ++k)
        {
            if (qs_[k].size() != MINI_PI_DOF)
                throw std::runtime_error("State_FixStand: qs[" + std::to_string(k) +
                                         "] has the wrong length");
        }
    }
    else
    {
        // Default target: robot_q == 0 == the nominal pose in this convention.
        // No motor-space offset appears here; JointMapper turns robot zero into
        // direction[j] * joint_offset[j] on the wire.
        qs_.assign(ts_.size(), std::vector<double>(MINI_PI_DOF, 0.0));
    }

    // Placeholder for the start pose; really filled in enter().
    qs_[0].assign(MINI_PI_DOF, 0.0);
}

void State_FixStand::enter()
{
    // Keyframe 0 is always "wherever we are right now", read from the freshly
    // sampled robot-space state. This is what guarantees no jump on entry.
    const RobotState& s = *ctx_->state;
    qs_[0].assign(s.robot_q.begin(), s.robot_q.end());
    t0_ = Clock::now();
    entered_ = true;

    std::ostringstream os;
    os << "State_FixStand: interpolating over " << ts_.back() << " s from measured pose [";
    for (std::size_t j = 0; j < MINI_PI_DOF; ++j) os << (j ? " " : "") << qs_[0][j];
    os << "] to [";
    for (std::size_t j = 0; j < MINI_PI_DOF; ++j) os << (j ? " " : "") << qs_.back()[j];
    os << "] (robot space)";
    ROS_INFO_STREAM(os.str());
}

bool State_FixStand::finished() const
{
    return entered_ && age_seconds(t0_) >= ts_.back();
}

std::string State_FixStand::notReadyForRLReason() const
{
    if (finished()) return {};
    if (!entered_) return "FixStand has not started";
    char buf[128];
    std::snprintf(buf, sizeof(buf),
                  "FixStand is still interpolating (%.2f s of %.2f s)",
                  age_seconds(t0_), ts_.back());
    return buf;
}

void State_FixStand::run()
{
    if (!entered_)
    {
        // Defensive: never command a trajectory we have no start pose for.
        holdDamping(*ctx_->command, kd_);
        stampValid(*ctx_->command);
        return;
    }

    const double t = age_seconds(t0_);
    const std::vector<double> q = linear_interpolate(t, ts_, qs_);
    if (q.size() != MINI_PI_DOF)
    {
        ROS_ERROR_THROTTLE(1.0, "State_FixStand: interpolation returned %zu values", q.size());
        holdDamping(*ctx_->command, kd_);
        stampValid(*ctx_->command);
        return;
    }

    RobotCommand& cmd = *ctx_->command;
    for (std::size_t j = 0; j < MINI_PI_DOF; ++j)
    {
        cmd.q[j]      = q[j];
        cmd.dq[j]     = 0.0;
        cmd.torque[j] = 0.0;
        cmd.kp[j]     = kp_[j];
        cmd.kd[j]     = kd_[j];
    }
    stampValid(cmd);
}

} // namespace mini_pi
