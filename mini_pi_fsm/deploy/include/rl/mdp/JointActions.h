// rl::mdp::JointPositionAction -- raw network output -> joint position target.
//
// Structural counterpart of unitree_rl_mjlab/deploy/include/isaaclab/envs/mdp/
// actions/joint_actions.h.
//
// ======================== PIPELINE ORDER (task §13) =========================
//     raw                         the network output, policy order
//  -> clip        [lo, hi]        on the RAW action
//  -> scale       elementwise
//  -> offset      elementwise     normally the policy's default pose
//  == q_target                    policy order
//  -> clip_output [lo_i, hi_i]    OPTIONAL, per joint, on the TARGET
//
// Unitree clips the PROCESSED value and has no raw clip. The order above is
// what the task specifies and what humanoid-gym does (`clip_actions` bounds the
// network output; `clip_output_lower/upper` in HighTorque's
// config/walk/humanoidgym.yaml is the separate post-offset bound, exposed here
// as clip_output).
//
// dq_target and tau_ff are ZERO by construction: this term commands position
// only. The PD gains come from deploy.yaml `stiffness`/`damping` and are
// applied by PolicyEnvironment, not here.
// ============================================================================
#pragma once

#include "rl/ActionManager.h"
#include "rl/PolicyEnvironment.h"
#include "rl/RobotView.h"

#include <algorithm>
#include <sstream>
#include <stdexcept>

namespace mini_pi
{
namespace rl
{
namespace mdp
{

class JointPositionAction : public ActionTerm
{
public:
    JointPositionAction(const YAML::Node& cfg, PolicyEnvironment* env)
        : ActionTerm(cfg, env)
    {
        dim_ = static_cast<int>(env->robot->numJoints());
        raw_.assign(static_cast<std::size_t>(dim_), 0.0f);
        processed_.assign(static_cast<std::size_t>(dim_), 0.0f);

        const std::size_t n = static_cast<std::size_t>(dim_);
        scale_  = loadPerJoint_(cfg["scale"], n, "scale", 1.0f);
        // Default offset is the policy's default pose, which is what makes
        // `action == 0` mean `hold the default pose`. An explicit `offset:`
        // overrides it; `offset: [0,...]` gives an absolute-target policy.
        if (cfg["offset"] && !cfg["offset"].IsNull())
            offset_ = loadPerJoint_(cfg["offset"], n, "offset", 0.0f);
        else
            offset_ = env->robot->data.default_joint_pos;

        if (cfg["clip"] && !cfg["clip"].IsNull())
        {
            const auto v = cfg["clip"].as<std::vector<float>>();
            if (v.size() != 2)
                throw std::runtime_error("JointPositionAction: `clip` must be [lo, hi] "
                                         "(it bounds the RAW action)");
            clip_lo_ = v[0];
            clip_hi_ = v[1];
            clip_ = true;
            if (!(clip_lo_ < clip_hi_))
                throw std::runtime_error("JointPositionAction: clip lo must be < hi");
        }

        if (cfg["clip_output"] && !cfg["clip_output"].IsNull())
        {
            const YAML::Node c = cfg["clip_output"];
            out_lo_ = loadPerJoint_(c["lower"], n, "clip_output.lower", 0.0f);
            out_hi_ = loadPerJoint_(c["upper"], n, "clip_output.upper", 0.0f);
            if (out_lo_.empty() || out_hi_.empty())
                throw std::runtime_error("JointPositionAction: clip_output needs both "
                                         "`lower` and `upper`");
            for (std::size_t i = 0; i < n; ++i)
                if (!(out_lo_[i] < out_hi_[i]))
                    throw std::runtime_error("JointPositionAction: clip_output.lower[" +
                                             std::to_string(i) + "] must be < upper");
            clip_output_ = true;
        }
    }

    int action_dim() const override { return dim_; }
    const std::vector<float>& raw_actions() const override { return raw_; }
    const std::vector<float>& processed_actions() const override { return processed_; }

    void process_actions(const std::vector<float>& actions) override
    {
        // raw_ holds the CLIPPED network output. That is what humanoid-gym
        // stores as `self.actions` (LeggedRobot.step: clip(actions)) and feeds
        // back as the last-action observation, so last_action must read it.
        raw_ = actions;
        for (std::size_t i = 0; i < raw_.size(); ++i)
        {
            if (clip_) raw_[i] = std::min(std::max(raw_[i], clip_lo_), clip_hi_);
            float a = raw_[i];
            a *= scale_[i];
            a += offset_[i];
            if (clip_output_) a = std::min(std::max(a, out_lo_[i]), out_hi_[i]);
            processed_[i] = a;
        }
    }

    void reset() override
    {
        raw_.assign(static_cast<std::size_t>(dim_), 0.0f);
        // Resetting to the offset, not to zero: with a zero raw action the
        // pipeline yields exactly `offset`, so the two agree and the first
        // command after a reset is the default pose rather than joint zero.
        processed_ = offset_;
    }

    std::string describe() const override
    {
        std::ostringstream os;
        os << "    JointPositionAction: dim " << dim_;
        if (clip_) os << ", raw clip [" << clip_lo_ << ", " << clip_hi_ << "]";
        os << "\n      scale =[";
        for (std::size_t i = 0; i < scale_.size(); ++i) os << (i ? " " : "") << scale_[i];
        os << "]\n      offset=[";
        for (std::size_t i = 0; i < offset_.size(); ++i) os << (i ? " " : "") << offset_[i];
        os << "]\n";
        if (clip_output_) os << "      clip_output: enabled\n";
        return os.str();
    }

private:
    static std::vector<float> loadPerJoint_(const YAML::Node& n, std::size_t dim,
                                            const char* what, float fallback)
    {
        if (!n || n.IsNull()) return std::vector<float>(dim, fallback);
        if (n.IsScalar()) return std::vector<float>(dim, n.as<float>());
        const auto v = n.as<std::vector<float>>();
        if (v.size() != dim)
            throw std::runtime_error(std::string("JointPositionAction: `") + what + "` has " +
                                     std::to_string(v.size()) + " values but the policy has " +
                                     std::to_string(dim) + " joints");
        return v;
    }

    int dim_ = 0;
    std::vector<float> raw_, processed_;
    std::vector<float> scale_, offset_;
    std::vector<float> out_lo_, out_hi_;
    bool clip_ = false, clip_output_ = false;
    float clip_lo_ = 0.0f, clip_hi_ = 0.0f;
};

MINI_PI_REGISTER_ACTION(JointPositionAction)

} // namespace mdp
} // namespace rl
} // namespace mini_pi
