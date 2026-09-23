#include "rl/ActionManager.h"

#include "rl/PolicyEnvironment.h"

#include <sstream>
#include <stdexcept>

namespace mini_pi
{
namespace rl
{

ActionManager::ActionManager(const YAML::Node& cfg, PolicyEnvironment* env)
{
    if (!cfg || !cfg.IsSequence() || cfg.size() == 0)
        throw std::runtime_error("deploy.yaml: `actions` must be a non-empty SEQUENCE of "
                                 "{type: <ActionTerm>} entries. A sequence is used rather "
                                 "than a map because terms are concatenated in order.");

    for (std::size_t i = 0; i < cfg.size(); ++i)
    {
        const YAML::Node n = cfg[i];
        const std::string where = "deploy.yaml actions[" + std::to_string(i) + "]";
        if (!n["type"]) throw std::runtime_error(where + " has no `type`");

        const std::string type = n["type"].as<std::string>();
        auto reg = actions_map().find(type);
        if (reg == actions_map().end())
        {
            std::ostringstream os;
            os << where << ": action term '" << type << "' is not registered. Known:";
            for (const auto& kv : actions_map()) os << " " << kv.first;
            throw std::runtime_error(os.str());
        }
        terms_.push_back(reg->second(n, env));
        total_dim_ += terms_.back()->action_dim();
    }

    action_.assign(static_cast<std::size_t>(total_dim_), 0.0f);
    network_output_.assign(static_cast<std::size_t>(total_dim_), 0.0f);
    processed_.assign(static_cast<std::size_t>(total_dim_), 0.0f);
}

void ActionManager::reset()
{
    network_output_.assign(static_cast<std::size_t>(total_dim_), 0.0f);
    action_.assign(static_cast<std::size_t>(total_dim_), 0.0f);
    processed_.assign(static_cast<std::size_t>(total_dim_), 0.0f);
    for (auto& t : terms_) t->reset();
}

void ActionManager::process_action(const std::vector<float>& action)
{
    if (action.size() != static_cast<std::size_t>(total_dim_))
        throw std::runtime_error("ActionManager: model produced " +
                                 std::to_string(action.size()) + " values but the configured "
                                 "action terms expect " + std::to_string(total_dim_));
    network_output_ = action;

    action_.clear();
    action_.reserve(static_cast<std::size_t>(total_dim_));
    processed_.clear();
    processed_.reserve(static_cast<std::size_t>(total_dim_));
    std::size_t idx = 0;
    for (auto& t : terms_)
    {
        const auto n = static_cast<std::size_t>(t->action_dim());
        t->process_actions(std::vector<float>(action.begin() + static_cast<std::ptrdiff_t>(idx),
                                              action.begin() + static_cast<std::ptrdiff_t>(idx + n)));
        const auto& p = t->processed_actions();
        processed_.insert(processed_.end(), p.begin(), p.end());
        const auto& a = t->raw_actions();
        action_.insert(action_.end(), a.begin(), a.end());
        idx += n;
    }
}

std::string ActionManager::describe() const
{
    std::ostringstream os;
    os << "  action dim " << total_dim_ << "\n";
    for (const auto& t : terms_) os << t->describe();
    return os.str();
}

} // namespace rl
} // namespace mini_pi
