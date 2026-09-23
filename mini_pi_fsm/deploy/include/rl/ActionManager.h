// rl::ActionManager -- config-driven action processing.
//
// Structural counterpart of unitree_rl_mjlab/deploy/include/isaaclab/manager/
// action_manager.h. Same ActionTerm interface, same registry, same split of a
// flat action vector across terms by action_dim().
//
// Differences from Unitree, both required by the task:
//   - `actions:` is a YAML SEQUENCE of {type: ...} entries, not a map keyed by
//     class name, because concatenation order matters and YAML does not
//     guarantee map order.
//   - the pipeline order is clip -> scale -> offset (task §13). Unitree does
//     scale -> offset -> clip. An optional `clip_output` covers Unitree's
//     post-offset clip as well, so a policy trained either way is expressible.
//
// A new action type is added by writing a class and one REGISTER_ACTION line.
// No FSM state and no other manager changes. See mdp/JointActions.h.
#pragma once

#include <yaml-cpp/yaml.h>

#include <functional>
#include <map>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

namespace mini_pi
{
namespace rl
{

class PolicyEnvironment;

class ActionTerm
{
public:
    ActionTerm(const YAML::Node& cfg, PolicyEnvironment* env) : cfg_(cfg), env_(env) {}
    virtual ~ActionTerm() = default;

    virtual int action_dim() const = 0;
    virtual const std::vector<float>& raw_actions() const = 0;
    virtual const std::vector<float>& processed_actions() const = 0;
    virtual void process_actions(const std::vector<float>& actions) = 0;
    virtual void reset() {}
    virtual std::string describe() const { return {}; }

protected:
    YAML::Node cfg_;
    PolicyEnvironment* env_ = nullptr;
};

using ActionFactory = std::function<std::unique_ptr<ActionTerm>(const YAML::Node&,
                                                                PolicyEnvironment*)>;

inline std::map<std::string, ActionFactory>& actions_map()
{
    static std::map<std::string, ActionFactory> instance;
    return instance;
}

#define MINI_PI_REGISTER_ACTION(name)                                                    \
    inline struct name##_registrar                                                       \
    {                                                                                    \
        name##_registrar()                                                               \
        {                                                                                \
            ::mini_pi::rl::actions_map()[#name] =                                        \
                [](const YAML::Node& cfg, ::mini_pi::rl::PolicyEnvironment* env)         \
                    -> std::unique_ptr<::mini_pi::rl::ActionTerm> {                      \
                    return std::make_unique<name>(cfg, env);                             \
                };                                                                       \
        }                                                                                \
    } name##_registrar_instance;

class ActionManager
{
public:
    /// `cfg` is the deploy.yaml `actions:` node (a sequence).
    ActionManager(const YAML::Node& cfg, PolicyEnvironment* env);

    /// Zeroes the raw action and resets every term. The `last_action`
    /// observation therefore reads all zeros on the first inference after a
    /// reset, which is what task §11's "15 zero frames" implies.
    void reset();

    /// The network output of the last step after each term's RAW clip and
    /// before scale/offset. This is what the `last_action` observation term
    /// reports: humanoid-gym feeds back `self.actions = clip(actions)`.
    const std::vector<float>& action() const { return action_; }

    /// The unmodified network output of the last step (diagnostics only).
    const std::vector<float>& networkOutput() const { return network_output_; }

    /// Concatenated processed output of every term.
    const std::vector<float>& processed_actions() const { return processed_; }

    /// Splits `action` across the terms and runs each term's pipeline.
    /// Throws if `action.size()` != total_action_dim().
    void process_action(const std::vector<float>& action);

    int total_action_dim() const { return total_dim_; }
    std::string describe() const;

private:
    std::vector<std::unique_ptr<ActionTerm>> terms_;
    std::vector<float> action_;          ///< clipped, unscaled
    std::vector<float> network_output_;  ///< as returned by the model
    std::vector<float> processed_;
    int total_dim_ = 0;
};

} // namespace rl
} // namespace mini_pi
