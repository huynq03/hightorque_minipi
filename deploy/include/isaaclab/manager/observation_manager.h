// ObservationManager -- deploy.yaml `observations:` -> one flat vector per group.
//
// Same model as unitree_rl_mjlab/deploy/include/isaaclab/manager/
// observation_manager.h: terms selected BY NAME from a registry
// (REGISTER_OBSERVATION, see envs/mdp/observations/observations.h), in YAML
// order; per-term params / scale / clip / history_length; group keys
// `scale_first` and `use_gym_history`. Either one group (term names directly
// under `observations:`, group name "obs") or several (each key a group named
// after the ONNX input it feeds).
//
//   use_gym_history: false   [term0 oldest..newest][term1 oldest..newest]...
//   use_gym_history: true    [frame oldest: term0 term1 ...]...[frame newest]
//
// Additions (all off by default, i.e. Unitree behaviour):
//   history_init: zero   reset() zero-fills the history (humanoid-gym frame
//                        stack) instead of repeating the current observation
//   per-term `check`     unavailable() names the first term whose source is
//                        missing; the env refuses to step
//   scalar scale         broadcast to the term's width
#pragma once

#include "isaaclab/manager/manager_term_cfg.h"

#include <map>
#include <string>
#include <vector>

namespace isaaclab
{

struct ObsEntry
{
    ObsFunc func;
    ObsCheck check;
};
using ObsMap = std::map<std::string, ObsEntry>;

inline ObsMap& observations_map()
{
    static ObsMap instance;
    return instance;
}

#define REGISTER_OBSERVATION(name)                                                       \
    inline std::vector<float> name(ManagerBasedRLEnv* env, const YAML::Node& params);    \
    inline struct name##_registrar                                                       \
    {                                                                                    \
        name##_registrar() { observations_map()[#name].func = name; }                    \
    } name##_registrar_instance;                                                         \
    inline std::vector<float> name(ManagerBasedRLEnv* env, const YAML::Node& params)

/// Availability check for an already registered term.
#define REGISTER_OBSERVATION_CHECK(name)                                                 \
    inline std::string name##_check(const ArticulationData& data,                        \
                                    const YAML::Node& params);                           \
    inline struct name##_check_registrar                                                 \
    {                                                                                    \
        name##_check_registrar() { observations_map()[#name].check = name##_check; }     \
    } name##_check_registrar_instance;                                                   \
    inline std::string name##_check(const ArticulationData& data, const YAML::Node& params)

class ObservationManager
{
public:
    struct Group
    {
        std::string name;
        std::vector<ObservationTermCfg> terms;
        bool use_gym_history = false;
        bool zero_init = false;
        std::size_t dim = 0;            ///< flattened width
        std::vector<float> obs;         ///< last computed value
    };

    /// Throws std::runtime_error on an unregistered term or a malformed entry.
    ObservationManager(const YAML::Node& cfg, ManagerBasedRLEnv* env);

    void reset();
    /// One entry per group, keyed by group name. Throws if a term is not finite.
    std::map<std::string, std::vector<float>> compute();

    /// "term '<name>': <why>" for the first term that cannot be computed from
    /// `data`, else empty. Pure; any thread.
    std::string unavailable(const ArticulationData& data) const;

    const std::vector<Group>& groups() const { return groups_; }
    const Group& group(const std::string& name) const;
    /// The newest single frame of `group` (each term's newest entry).
    std::vector<float> last_frame(const std::string& group) const;

    std::string describe() const;
    /// `frame` (a last_frame() of the first group) sliced by term, labelled.
    std::string describe_frame(const std::vector<float>& frame) const;

private:
    Group parse_group_(const std::string& name, const YAML::Node& node);
    void compute_group_(Group& group);
    /// Rebuilds group.obs from the term histories (gym or term-major order).
    static void flatten_(Group& group);

    ManagerBasedRLEnv* env_;
    std::vector<Group> groups_;
};

} // namespace isaaclab
