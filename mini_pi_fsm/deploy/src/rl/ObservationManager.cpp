#include "rl/ObservationManager.h"

#include "rl/PolicyEnvironment.h"

#include <algorithm>
#include <sstream>
#include <stdexcept>

namespace mini_pi
{
namespace rl
{
namespace
{
/// A YAML scale/clip may be a scalar (broadcast) or a sequence of exactly
/// `dim` values. Anything else is a configuration error.
std::vector<float> loadVector(const YAML::Node& n, std::size_t dim, const char* what,
                              const std::string& where)
{
    std::vector<float> v;
    if (!n || n.IsNull()) return v;

    if (n.IsScalar())
    {
        v.assign(dim, n.as<float>());
        return v;
    }
    if (!n.IsSequence())
        throw std::runtime_error(where + ": " + what + " must be a scalar or a sequence");

    v = n.as<std::vector<float>>();
    if (v.size() != dim)
        throw std::runtime_error(where + ": " + what + " has " + std::to_string(v.size()) +
                                 " values but the term produces " + std::to_string(dim));
    return v;
}
} // namespace

ObservationManager::ObservationManager(const YAML::Node& cfg, PolicyEnvironment* env)
    : env_(env)
{
    if (!cfg || !cfg.IsMap() || cfg.size() == 0)
        throw std::runtime_error("deploy.yaml: `observations` must be a non-empty map of groups");

    for (auto it = cfg.begin(); it != cfg.end(); ++it)
    {
        const std::string group_name = it->first.as<std::string>();
        groups_.push_back(parseGroup_(group_name, it->second));
        index_[group_name] = groups_.size() - 1;
    }
    state_.resize(groups_.size());
    reset();
}

ObservationGroupCfg ObservationManager::parseGroup_(const std::string& name,
                                                    const YAML::Node& node)
{
    const std::string where = "deploy.yaml observations." + name;
    ObservationGroupCfg g;
    g.name = name;

    const YAML::Node terms = node["terms"];
    if (!terms || !terms.IsSequence() || terms.size() == 0)
        throw std::runtime_error(where + ".terms must be a non-empty SEQUENCE. A sequence is "
                                "used rather than a map because the term order is the wire "
                                "format and YAML does not guarantee map order.");

    for (std::size_t i = 0; i < terms.size(); ++i)
    {
        const YAML::Node t = terms[i];
        const std::string tw = where + ".terms[" + std::to_string(i) + "]";
        if (!t["name"]) throw std::runtime_error(tw + " has no `name`");

        ObservationTermCfg tc;
        tc.name   = t["name"].as<std::string>();
        tc.params = t["params"] ? t["params"] : YAML::Node(YAML::NodeType::Map);

        auto reg = observations_map().find(tc.name);
        if (reg == observations_map().end())
        {
            std::ostringstream os;
            os << tw << ": observation term '" << tc.name << "' is not registered. Known:";
            for (const auto& kv : observations_map()) os << " " << kv.first;
            throw std::runtime_error(os.str());
        }
        tc.func = reg->second;

        // Width is discovered by calling the term once, exactly as Unitree
        // does. Terms must therefore be callable before reset(), which is why
        // RobotView is fully sized by its constructor.
        tc.dim = tc.func(env_, tc.params).size();
        if (tc.dim == 0) throw std::runtime_error(tw + ": term '" + tc.name + "' produced 0 values");

        tc.scale = loadVector(t["scale"], tc.dim, "scale", tw);
        tc.clip  = loadVector(t["clip"], 2, "clip", tw);
        if (!tc.clip.empty() && tc.clip.size() != 2)
            throw std::runtime_error(tw + ": clip must be [lo, hi]");

        g.frame_dim += tc.dim;
        g.terms.push_back(std::move(tc));
    }

    // ---- group clip ------------------------------------------------------
    if (const YAML::Node c = node["clip"])
    {
        const auto v = c.as<std::vector<float>>();
        if (v.size() != 2) throw std::runtime_error(where + ".clip must be [lo, hi]");
        g.clip_enabled = true;
        g.clip_lo = v[0];
        g.clip_hi = v[1];
        if (!(g.clip_lo < g.clip_hi))
            throw std::runtime_error(where + ".clip: lo must be < hi");
    }

    // ---- history ---------------------------------------------------------
    if (const YAML::Node h = node["history"])
    {
        const std::string type = h["type"] ? h["type"].as<std::string>() : std::string("frame_stack");
        if (type != "frame_stack")
            throw std::runtime_error(where + ".history.type: only 'frame_stack' is implemented "
                                     "(got '" + type + "')");
        g.history_length = h["length"] ? h["length"].as<std::size_t>() : 1;
        if (g.history_length == 0)
            throw std::runtime_error(where + ".history.length must be >= 1");

        if (const YAML::Node iv = h["initial_value"])
        {
            const std::string s = iv.as<std::string>();
            if (s != "zero")
                throw std::runtime_error(where + ".history.initial_value: only 'zero' is "
                                         "implemented (got '" + s + "')");
        }
        // Declared dimensions are ASSERTED, never used to size anything. A
        // mismatch means the config and the term list disagree, and silently
        // trusting either one produces a wrong observation.
        if (const YAML::Node fd = h["frame_dim"])
        {
            const auto want = fd.as<std::size_t>();
            if (want != g.frame_dim)
                throw std::runtime_error(where + ".history.frame_dim says " +
                                         std::to_string(want) + " but the configured terms "
                                         "produce " + std::to_string(g.frame_dim));
        }
        if (const YAML::Node td = h["total_dim"])
        {
            const auto want = td.as<std::size_t>();
            if (want != g.frame_dim * g.history_length)
                throw std::runtime_error(where + ".history.total_dim says " +
                                         std::to_string(want) + " but frame_dim(" +
                                         std::to_string(g.frame_dim) + ") * length(" +
                                         std::to_string(g.history_length) + ") = " +
                                         std::to_string(g.frame_dim * g.history_length));
        }
    }
    return g;
}

void ObservationManager::reset()
{
    for (std::size_t gi = 0; gi < groups_.size(); ++gi)
    {
        const auto& g = groups_[gi];
        auto& st = state_[gi];
        st.frames.assign(g.history_length, std::vector<float>(g.frame_dim, 0.0f));
        st.flat.assign(g.totalDim(), 0.0f);
        st.last_frame.assign(g.frame_dim, 0.0f);
    }
}

std::vector<float> ObservationManager::buildFrame_(const ObservationGroupCfg& g)
{
    std::vector<float> frame;
    frame.reserve(g.frame_dim);

    for (const auto& t : g.terms)
    {
        std::vector<float> v = t.func(env_, t.params);
        if (v.size() != t.dim)
            throw std::runtime_error("observation term '" + t.name + "' returned " +
                                     std::to_string(v.size()) + " values but produced " +
                                     std::to_string(t.dim) + " at load time");

        // Per-term clip is the Unitree order: clip first, then scale.
        if (!t.clip.empty())
            for (auto& x : v) x = std::min(std::max(x, t.clip[0]), t.clip[1]);
        if (!t.scale.empty())
            for (std::size_t i = 0; i < v.size(); ++i) v[i] *= t.scale[i];

        frame.insert(frame.end(), v.begin(), v.end());
    }

    // Group clip: applied to the ASSEMBLED, SCALED frame. This is the
    // humanoid-gym `clip_obs` semantics.
    if (g.clip_enabled)
        for (auto& x : frame) x = std::min(std::max(x, g.clip_lo), g.clip_hi);

    return frame;
}

void ObservationManager::push_(std::size_t gi, std::vector<float> frame)
{
    const auto& g = groups_[gi];
    auto& st = state_[gi];

    st.last_frame = frame;
    st.frames.push_back(std::move(frame));
    while (st.frames.size() > g.history_length) st.frames.pop_front();

    // Flatten frame-major: [oldest][...][newest], each frame contiguous.
    st.flat.resize(g.totalDim());
    for (std::size_t h = 0; h < st.frames.size(); ++h)
    {
        std::copy(st.frames[h].begin(), st.frames[h].end(),
                  st.flat.begin() + static_cast<std::ptrdiff_t>(h * g.frame_dim));
    }
}

std::map<std::string, std::vector<float>> ObservationManager::compute()
{
    std::map<std::string, std::vector<float>> out;
    for (std::size_t gi = 0; gi < groups_.size(); ++gi)
    {
        push_(gi, buildFrame_(groups_[gi]));
        out[groups_[gi].name] = state_[gi].flat;
    }
    return out;
}

const std::vector<float>& ObservationManager::flat(const std::string& group) const
{
    return state_.at(index_.at(group)).flat;
}

const std::vector<float>& ObservationManager::lastFrame(const std::string& group) const
{
    return state_.at(index_.at(group)).last_frame;
}

std::string ObservationManager::describe() const
{
    std::ostringstream os;
    for (const auto& g : groups_)
    {
        os << "  observation group '" << g.name << "': frame " << g.frame_dim
           << " x history " << g.history_length << " = " << g.totalDim();
        if (g.clip_enabled) os << ", clip [" << g.clip_lo << ", " << g.clip_hi << "]";
        os << "\n";
        std::size_t off = 0;
        for (const auto& t : g.terms)
        {
            os << "    [" << off << ":" << (off + t.dim) << ") " << t.name;
            if (!t.scale.empty())
            {
                os << "  scale=[";
                for (std::size_t i = 0; i < t.scale.size(); ++i)
                    os << (i ? " " : "") << t.scale[i];
                os << "]";
            }
            os << "\n";
            off += t.dim;
        }
    }
    return os.str();
}

} // namespace rl
} // namespace mini_pi
