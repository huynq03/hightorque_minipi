#include "isaaclab/manager/observation_manager.h"

#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace isaaclab
{
namespace
{
bool is_registered(const std::string& name)
{
    const auto it = observations_map().find(name);
    return it != observations_map().end() && it->second.func;
}

std::vector<float> load_scale(const YAML::Node& n, std::size_t dim, const std::string& where)
{
    if (!n || n.IsNull()) return {};
    return scalar_or_list(n, dim, where + ": scale (one per term value)");
}

std::vector<float> load_clip(const YAML::Node& n, const std::string& where)
{
    if (!n || n.IsNull()) return {};
    auto v = n.as<std::vector<float>>();
    if (v.size() != 2 || !(v[0] < v[1]))
        throw std::runtime_error(where + ": clip must be [lo, hi] with lo < hi");
    return v;
}
} // namespace

ObservationManager::ObservationManager(const YAML::Node& cfg, ManagerBasedRLEnv* env) : env_(env)
{
    if (!cfg || !cfg.IsMap() || cfg.size() == 0)
        throw std::runtime_error("deploy.yaml: `observations` must be a non-empty map");

    // Unitree: terms directly under `observations:` form the single group "obs".
    if (is_registered(cfg.begin()->first.as<std::string>()))
        groups_.push_back(parse_group_("obs", cfg));
    else
        for (auto it = cfg.begin(); it != cfg.end(); ++it)
            groups_.push_back(parse_group_(it->first.as<std::string>(), it->second));
    reset();
}

ObservationManager::Group ObservationManager::parse_group_(const std::string& name,
                                                           const YAML::Node& node)
{
    const std::string where = "deploy.yaml observations." + name;
    if (!node.IsMap() || node.size() == 0) throw std::runtime_error(where + " must be a non-empty map");

    Group g;
    g.name = name;
    bool scale_first = false;
    for (auto it = node.begin(); it != node.end(); ++it)
    {
        const std::string key = it->first.as<std::string>();
        if (key == "scale_first") { scale_first = it->second.as<bool>(); continue; }
        if (key == "use_gym_history") { g.use_gym_history = it->second.as<bool>(); continue; }
        if (key == "history_init")
        {
            const std::string v = it->second.as<std::string>();
            if (v != "zero" && v != "current")
                throw std::runtime_error(where + ".history_init must be 'zero' or 'current'");
            g.zero_init = v == "zero";
            continue;
        }

        const std::string tw = where + "." + key;
        if (!is_registered(key))
        {
            std::ostringstream os;
            os << tw << ": observation term '" << key << "' is not registered. Known:";
            for (const auto& kv : observations_map())
                if (kv.second.func) os << " " << kv.first;
            throw std::runtime_error(os.str());
        }
        const YAML::Node t = it->second.IsMap() ? it->second : YAML::Node(YAML::NodeType::Map);
        ObservationTermCfg tc;
        tc.name = key;
        tc.params = t["params"] && !t["params"].IsNull() ? t["params"]
                                                         : YAML::Node(YAML::NodeType::Map);
        tc.func = observations_map().at(key).func;
        tc.check = observations_map().at(key).check;
        tc.history_length = t["history_length"] ? t["history_length"].as<int>() : 1;
        if (tc.history_length < 1) throw std::runtime_error(tw + ": history_length must be >= 1");
        // Width is discovered by calling the term once, as Unitree does.
        tc.dim = tc.func(env_, tc.params).size();
        if (tc.dim == 0) throw std::runtime_error(tw + ": the term produced 0 values");
        tc.scale = load_scale(t["scale"], tc.dim, tw);
        tc.clip = load_clip(t["clip"], tw);
        g.terms.push_back(std::move(tc));
    }
    if (g.terms.empty()) throw std::runtime_error(where + " has no observation terms");

    for (auto& t : g.terms)
    {
        t.scale_first = scale_first;
        g.dim += t.dim * static_cast<std::size_t>(t.history_length);
        if (g.use_gym_history && t.history_length != g.terms[0].history_length)
            throw std::runtime_error(where + ": use_gym_history needs one history_length "
                                     "for every term");
    }
    return g;
}

void ObservationManager::reset()
{
    for (auto& g : groups_)
    {
        for (auto& t : g.terms)
        {
            if (g.zero_init) { t.fill(0.0f); continue; }
            auto v = t.func(env_, t.params);
            bool finite = true;
            for (float x : v) finite = finite && std::isfinite(x);
            if (finite) t.reset(v);
            else        t.fill(0.0f);   // no valid sample yet: nothing invented
        }
        flatten_(g);
    }
}

void ObservationManager::flatten_(Group& g)
{
    g.obs.clear();
    if (g.use_gym_history)
    {
        for (int h = 0; h < g.terms[0].history_length; ++h)
            for (const auto& t : g.terms)
            {
                const auto& e = t.get(h);
                g.obs.insert(g.obs.end(), e.begin(), e.end());
            }
    }
    else
    {
        for (const auto& t : g.terms)
        {
            const auto e = t.get();
            g.obs.insert(g.obs.end(), e.begin(), e.end());
        }
    }
}

void ObservationManager::compute_group_(Group& g)
{
    for (auto& t : g.terms)
    {
        auto v = t.func(env_, t.params);
        if (v.size() != t.dim)
            throw std::runtime_error("observation term '" + t.name + "' returned " +
                                     std::to_string(v.size()) + " values, " +
                                     std::to_string(t.dim) + " at load time");
        for (float x : v)
            if (!std::isfinite(x))
                throw std::runtime_error("observation term '" + t.name +
                                         "' is not finite -- inference not run");
        t.add(std::move(v));
    }
    flatten_(g);
}

std::map<std::string, std::vector<float>> ObservationManager::compute()
{
    std::map<std::string, std::vector<float>> out;
    for (auto& g : groups_)
    {
        compute_group_(g);
        out[g.name] = g.obs;
    }
    return out;
}

std::string ObservationManager::unavailable(const ArticulationData& data) const
{
    for (const auto& g : groups_)
        for (const auto& t : g.terms)
            if (t.check)
                if (std::string why = t.check(data, t.params); !why.empty())
                    return "term '" + t.name + "': " + why;
    return {};
}

const ObservationManager::Group& ObservationManager::group(const std::string& name) const
{
    for (const auto& g : groups_)
        if (g.name == name) return g;
    throw std::runtime_error("no observation group '" + name + "'");
}

std::vector<float> ObservationManager::last_frame(const std::string& name) const
{
    std::vector<float> f;
    for (const auto& t : group(name).terms)
    {
        const auto& e = t.get(t.history_length - 1);
        f.insert(f.end(), e.begin(), e.end());
    }
    return f;
}

std::string ObservationManager::describe() const
{
    std::ostringstream os;
    for (const auto& g : groups_)
    {
        os << "  observation group '" << g.name << "': " << g.dim << " values"
           << (g.use_gym_history ? ", gym (frame-major) history" : "")
           << (g.zero_init ? ", zero-initialized" : "") << "\n";
        for (const auto& t : g.terms)
        {
            os << "    " << t.name << "  dim " << t.dim << " x history " << t.history_length;
            if (!t.scale.empty())
            {
                os << "  scale=[";
                for (std::size_t i = 0; i < t.scale.size(); ++i) os << (i ? " " : "") << t.scale[i];
                os << "]";
            }
            if (!t.clip.empty()) os << "  clip=[" << t.clip[0] << ", " << t.clip[1] << "]";
            os << "\n";
        }
    }
    return os.str();
}

std::string ObservationManager::describe_frame(const std::vector<float>& frame) const
{
    // The short labels are the ones the status log has always printed;
    // doc/deployment.md (dry run) refers to them. A term without one is
    // printed under its own name.
    static const std::map<std::string, const char*> kLabel = {
        {"gait_phase", "phase "},  {"velocity_commands", "cmd   "},
        {"joint_pos_rel", "q     "}, {"joint_pos", "q     "},
        {"joint_vel", "dq    "},   {"joint_vel_rel", "dq    "},
        {"last_action", "act   "}, {"base_ang_vel", "gyro  "},
        {"base_euler", "euler "},  {"base_lin_vel", "linvel"},
        {"projected_gravity", "grav  "}, {"gait_joint_reference", "ref   "}};

    const Group& g = groups_.front();
    std::size_t frame_dim = 0;
    for (const auto& t : g.terms) frame_dim += t.dim;
    if (frame.size() != frame_dim) return {};

    std::ostringstream os;
    os << std::fixed << std::setprecision(3);
    std::size_t a = 0;
    for (const auto& t : g.terms)
    {
        const std::size_t b = a + t.dim;
        const auto it = kLabel.find(t.name);
        os << "\n  obs " << (it != kLabel.end() ? it->second : t.name.c_str()) << " [" << a
           << ":" << b << ")";
        for (std::size_t i = a; i < b; ++i) os << " " << frame[i];
        a = b;
    }
    return os.str();
}

} // namespace isaaclab
