// Observation term configuration and its history buffer.
//
// Same as unitree_rl_mjlab/deploy/include/isaaclab/manager/manager_term_cfg.h
// (per-term clip / scale / history_length, scale_first, oldest entry first),
// plus two additions:
//   check    optional availability check: why the term cannot be computed
//            from the ArticulationData (source missing / stale), or empty
//   fill()   zero-fills the history (`history_init: zero`, humanoid-gym),
//            instead of Unitree's repeat-the-current-observation reset()
#pragma once

#include "isaaclab/assets/articulation/articulation.h"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <deque>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

namespace isaaclab
{

class ManagerBasedRLEnv;

/// deploy.yaml value given as a scalar (broadcast to `n`) or a list of exactly
/// `n`. The caller handles a missing node; `what` names it in the error.
inline std::vector<float> scalar_or_list(const YAML::Node& v, std::size_t n, const std::string& what)
{
    if (v.IsScalar()) return std::vector<float>(n, v.as<float>());
    auto out = v.as<std::vector<float>>();
    if (out.size() != n)
        throw std::runtime_error(what + " has " + std::to_string(out.size()) + " values, expected " +
                                 std::to_string(n));
    return out;
}

/// Unscaled, unclipped values in the term's natural units, policy joint order.
using ObsFunc = std::function<std::vector<float>(ManagerBasedRLEnv*, const YAML::Node&)>;
using ObsCheck = std::function<std::string(const ArticulationData&, const YAML::Node&)>;

struct ObservationTermCfg
{
    std::string name;
    YAML::Node params;
    ObsFunc func;
    ObsCheck check;              ///< may be empty: always available
    std::vector<float> clip;     ///< [lo, hi] or empty
    std::vector<float> scale;    ///< one per value or empty
    int history_length = 1;
    bool scale_first = false;    ///< IsaacLab default: clip, then scale
    std::size_t dim = 0;         ///< width, measured once at load

    /// Unitree: the history repeats `obs`.
    void reset(const std::vector<float>& obs)
    {
        buff_.clear();
        for (int i = 0; i < history_length; ++i) add(obs);
    }
    /// The history holds zeros (already processed: no clip / scale applied).
    void fill(float v)
    {
        buff_.assign(static_cast<std::size_t>(history_length), std::vector<float>(dim, v));
    }

    void add(std::vector<float> obs)
    {
        for (std::size_t j = 0; j < obs.size(); ++j)
        {
            if (scale_first)
            {
                if (!scale.empty()) obs[j] *= scale[j];
                if (!clip.empty()) obs[j] = std::clamp(obs[j], clip[0], clip[1]);
            }
            else
            {
                if (!clip.empty()) obs[j] = std::clamp(obs[j], clip[0], clip[1]);
                if (!scale.empty()) obs[j] *= scale[j];
            }
        }
        buff_.push_back(std::move(obs));
        while (buff_.size() > static_cast<std::size_t>(history_length)) buff_.pop_front();
    }

    /// Entry n, 0 = oldest.
    const std::vector<float>& get(int n) const { return buff_[static_cast<std::size_t>(n)]; }

    /// Whole history, oldest first.
    std::vector<float> get() const
    {
        std::vector<float> out;
        for (const auto& e : buff_) out.insert(out.end(), e.begin(), e.end());
        return out;
    }

private:
    std::deque<std::vector<float>> buff_;
};

} // namespace isaaclab
