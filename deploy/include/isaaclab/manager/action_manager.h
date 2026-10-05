// ActionManager -- deploy.yaml `actions:` -> processed targets, policy order.
//
// Same as unitree_rl_mjlab/deploy/include/isaaclab/manager/action_manager.h:
// `actions:` is a map {<registered ActionTerm>: cfg}; terms consume
// consecutive slices of the network output in YAML order.
//
// One deliberate difference: action() is the concatenation of every term's
// raw_actions(), i.e. AFTER a term's optional raw clip -- what the
// `last_action` observation feeds back. Without a raw clip it is the network
// output, exactly Unitree's `_action`.
#pragma once

#include <yaml-cpp/yaml.h>

#include <functional>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace isaaclab
{

class ManagerBasedRLEnv;

class ActionTerm
{
public:
    ActionTerm(const YAML::Node& cfg, ManagerBasedRLEnv* env) : cfg(cfg), env(env) {}
    virtual ~ActionTerm() = default;

    virtual int action_dim() const = 0;
    virtual const std::vector<float>& raw_actions() const = 0;
    virtual const std::vector<float>& processed_actions() const = 0;
    virtual void process_actions(const std::vector<float>& actions) = 0;
    virtual void reset() {}
    virtual std::string describe() const { return {}; }

protected:
    YAML::Node cfg;
    ManagerBasedRLEnv* env;
};

using ActionFactory = std::function<std::unique_ptr<ActionTerm>(const YAML::Node&, ManagerBasedRLEnv*)>;

inline std::map<std::string, ActionFactory>& actions_map()
{
    static std::map<std::string, ActionFactory> instance;
    return instance;
}

#define REGISTER_ACTION(name)                                                                   \
    inline struct name##_registrar                                                              \
    {                                                                                           \
        name##_registrar()                                                                      \
        {                                                                                       \
            actions_map()[#name] = [](const YAML::Node& cfg, ManagerBasedRLEnv* env) {          \
                return std::unique_ptr<ActionTerm>(std::make_unique<name>(cfg, env));           \
            };                                                                                  \
        }                                                                                       \
    } name##_registrar_instance;

class ActionManager
{
public:
    ActionManager(const YAML::Node& cfg, ManagerBasedRLEnv* env)
    {
        if (!cfg || !cfg.IsMap() || cfg.size() == 0)
            throw std::runtime_error("deploy.yaml: `actions` must be a non-empty map "
                                     "{<ActionTerm>: cfg}");
        for (auto it = cfg.begin(); it != cfg.end(); ++it)
        {
            const std::string name = it->first.as<std::string>();
            const auto f = actions_map().find(name);
            if (f == actions_map().end())
            {
                std::string known;
                for (const auto& kv : actions_map()) known += " " + kv.first;
                throw std::runtime_error("deploy.yaml: action term '" + name +
                                         "' is not registered. Known:" + known);
            }
            terms_.push_back(f->second(it->second, env));
        }
        reset();
    }

    void reset()
    {
        for (auto& t : terms_) t->reset();
        gather_();
    }

    /// Fed back by `last_action`: every term's raw_actions().
    const std::vector<float>& action() const { return action_; }
    /// The targets, policy order.
    const std::vector<float>& processed_actions() const { return processed_; }

    /// Throws if `action` does not have total_action_dim() values.
    void process_action(const std::vector<float>& action)
    {
        if (static_cast<int>(action.size()) != total_action_dim())
            throw std::runtime_error("ActionManager: model produced " +
                                     std::to_string(action.size()) + " values but the "
                                     "configured action terms expect " +
                                     std::to_string(total_action_dim()));
        auto it = action.begin();
        for (auto& t : terms_)
        {
            t->process_actions(std::vector<float>(it, it + t->action_dim()));
            it += t->action_dim();
        }
        gather_();
    }

    int total_action_dim() const
    {
        int n = 0;
        for (const auto& t : terms_) n += t->action_dim();
        return n;
    }

    std::string describe() const
    {
        std::ostringstream os;
        os << "  action dim " << total_action_dim() << "\n";
        for (const auto& t : terms_) os << t->describe();
        return os.str();
    }

private:
    void gather_()
    {
        action_.clear();
        processed_.clear();
        for (const auto& t : terms_)
        {
            action_.insert(action_.end(), t->raw_actions().begin(), t->raw_actions().end());
            processed_.insert(processed_.end(), t->processed_actions().begin(),
                              t->processed_actions().end());
        }
    }

    std::vector<std::unique_ptr<ActionTerm>> terms_;
    std::vector<float> action_, processed_;
};

} // namespace isaaclab
