// Joint action terms. Same as unitree_rl_mjlab/deploy/include/isaaclab/envs/
// mdp/actions/joint_actions.h, every key optional:
//
//   raw_clip  [lo, hi]           addition to Unitree: bounds the NETWORK OUTPUT
//                                (humanoid-gym clip_actions); this clipped
//                                value is what last_action feeds back
//   scale     scalar | [n]       default 1
//   offset    scalar | [n]       default deploy.yaml default_joint_pos
//                                (IsaacLab use_default_offset)
//   clip      [[lo, hi] x n]     Unitree: bounds the processed target
//   joint_ids null               every policy joint (subsets unsupported)
//
//   target = clip(clamp(raw, raw_clip) * scale + offset)
#pragma once

#include "isaaclab/envs/manager_based_rl_env.h"
#include "isaaclab/manager/action_manager.h"

#include <algorithm>
#include <sstream>
#include <stdexcept>

namespace isaaclab
{

class JointAction : public ActionTerm
{
public:
    JointAction(const YAML::Node& cfg, ManagerBasedRLEnv* env) : ActionTerm(cfg, env)
    {
        const auto& d = env->robot->data;
        dim_ = static_cast<int>(d.joint_ids_map.size());
        if (cfg["joint_ids"] && !cfg["joint_ids"].IsNull())
            throw std::runtime_error("JointAction: joint_ids subsets are not supported; the "
                                     "action covers every policy joint");
        scale_ = per_joint(cfg["scale"], "scale", std::vector<float>(n(), 1.0f));
        offset_ = per_joint(cfg["offset"], "offset", d.default_joint_pos);
        if (cfg["raw_clip"] && !cfg["raw_clip"].IsNull())
        {
            raw_clip_ = cfg["raw_clip"].as<std::vector<float>>();
            if (raw_clip_.size() != 2 || !(raw_clip_[0] < raw_clip_[1]))
                throw std::runtime_error("JointAction: raw_clip must be [lo, hi], lo < hi");
        }
        if (cfg["clip"] && !cfg["clip"].IsNull())
        {
            clip_ = cfg["clip"].as<std::vector<std::vector<float>>>();
            bool ok = clip_.size() == n();
            for (const auto& c : clip_) ok = ok && c.size() == 2 && c[0] < c[1];
            if (!ok)
                throw std::runtime_error("JointAction: clip must be " + std::to_string(n()) +
                                         " pairs [lo, hi] with lo < hi");
        }
        reset();
    }

    int action_dim() const override { return dim_; }
    const std::vector<float>& raw_actions() const override { return raw_; }
    const std::vector<float>& processed_actions() const override { return processed_; }

    void process_actions(const std::vector<float>& actions) override
    {
        for (std::size_t i = 0; i < n(); ++i)
        {
            raw_[i] = raw_clip_.empty() ? actions[i]
                                        : std::clamp(actions[i], raw_clip_[0], raw_clip_[1]);
            float a = raw_[i] * scale_[i];
            a += offset_[i];
            if (!clip_.empty()) a = std::clamp(a, clip_[i][0], clip_[i][1]);
            processed_[i] = a;
        }
    }

    void reset() override
    {
        raw_.assign(n(), 0.0f);
        processed_.assign(n(), 0.0f);
    }

    std::string describe() const override
    {
        std::ostringstream os;
        os << "    JointAction dim " << dim_;
        if (!raw_clip_.empty()) os << ", raw clip [" << raw_clip_[0] << ", " << raw_clip_[1] << "]";
        if (!clip_.empty()) os << ", target clip per joint";
        os << "\n      scale =[";
        for (std::size_t i = 0; i < n(); ++i) os << (i ? " " : "") << scale_[i];
        os << "]\n      offset=[";
        for (std::size_t i = 0; i < n(); ++i) os << (i ? " " : "") << offset_[i];
        os << "]\n";
        return os.str();
    }

protected:
    std::size_t n() const { return static_cast<std::size_t>(dim_); }

    std::vector<float> per_joint(const YAML::Node& v, const char* what,
                                 const std::vector<float>& fallback) const
    {
        if (!v || v.IsNull()) return fallback;
        return scalar_or_list(v, n(), std::string("JointAction: `") + what + "` (one per policy joint)");
    }

    int dim_ = 0;
    std::vector<float> scale_, offset_, raw_clip_;
    std::vector<std::vector<float>> clip_;
    std::vector<float> raw_, processed_;
};

class JointPositionAction : public JointAction
{
public:
    using JointAction::JointAction;
};

REGISTER_ACTION(JointPositionAction)

} // namespace isaaclab
