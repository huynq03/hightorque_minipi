#include "isaaclab/envs/manager_based_rl_env.h"

// The registries are filled by these headers' static registrars; including
// them here links them into every program that builds an env.
#include "isaaclab/envs/mdp/actions/joint_actions.h"
#include "isaaclab/envs/mdp/observations/observations.h"

#include <cmath>
#include <map>
#include <sstream>
#include <stdexcept>

namespace isaaclab
{
namespace
{
std::vector<float> per_joint(const YAML::Node& n, std::size_t dim, const char* what)
{
    if (!n || n.IsNull()) throw std::runtime_error(std::string("deploy.yaml: `") + what + "` is required");
    return scalar_or_list(n, dim, std::string("deploy.yaml: `") + what + "` (one per joint_names entry)");
}

std::array<float, 2> range(const YAML::Node& n, const char* what, std::array<float, 2> fallback)
{
    if (!n || n.IsNull()) return fallback;
    const auto v = n.as<std::vector<float>>();
    if (v.size() != 2 || !(v[0] <= v[1]))
        throw std::runtime_error(std::string("deploy.yaml commands.base_velocity.ranges.") + what +
                                 " must be [min, max] with min <= max");
    return {v[0], v[1]};
}

/// deploy.yaml joint_names (policy order) -> robot index, by name.
std::vector<int> resolve_joints(const std::vector<std::string>& robot,
                                const std::vector<std::string>& policy)
{
    if (policy.size() != robot.size())
        throw std::runtime_error("deploy.yaml joint_names has " + std::to_string(policy.size()) +
                                 " joints but the robot has " + std::to_string(robot.size()) +
                                 ". Partial-DOF policies are not supported.");
    std::map<std::string, int> index;
    for (std::size_t r = 0; r < robot.size(); ++r) index[robot[r]] = static_cast<int>(r);
    std::vector<int> ids;
    std::map<std::string, int> seen;
    for (const auto& n : policy)
    {
        if (!seen.emplace(n, 0).second)
            throw std::runtime_error("deploy.yaml joint_names repeats '" + n + "'");
        const auto it = index.find(n);
        if (it == index.end())
        {
            std::string known;
            for (const auto& r : robot) known += " " + r;
            throw std::runtime_error("deploy.yaml joint '" + n +
                                     "' is not a robot joint. Known robot joints:" + known);
        }
        ids.push_back(it->second);
    }
    return ids;
}
} // namespace

ManagerBasedRLEnv::ManagerBasedRLEnv(const YAML::Node& cfg_in, std::shared_ptr<Articulation> robot_in)
    : cfg(cfg_in), robot(std::move(robot_in))
{
    if (!cfg || !cfg.IsMap()) throw std::runtime_error("deploy.yaml is empty or is not a map");
    if (cfg["policy_type"])
        throw std::runtime_error("deploy.yaml: `policy_type` is gone -- every package is "
                                 "described by observations / actions alone");
    if (!cfg["step_dt"]) throw std::runtime_error("deploy.yaml: `step_dt` is required");
    step_dt = cfg["step_dt"].as<double>();
    if (!(step_dt > 0.0)) throw std::runtime_error("deploy.yaml: `step_dt` must be > 0");
    if (!cfg["joint_names"])
        throw std::runtime_error("deploy.yaml: `joint_names` is required (POLICY joint order, "
                                 "resolved by name against the robot's joint names)");

    auto& d = robot->data;
    d.joint_ids_map = resolve_joints(robot->robot_joint_names,
                                     cfg["joint_names"].as<std::vector<std::string>>());
    const std::size_t n = d.joint_ids_map.size();
    d.joint_pos_shift.assign(n, 0.0);
    if (const YAML::Node c = cfg["calibration"])
    {
        // The policy's joint coordinate is defined by another motor->joint
        // calibration: express the difference to the robot's own as a shift.
        const auto off = c["joint_offset"] ? c["joint_offset"].as<std::vector<double>>()
                                           : std::vector<double>{};
        if (off.size() != n)
            throw std::runtime_error("deploy.yaml calibration.joint_offset needs " +
                                     std::to_string(n) + " values (policy joint order)");
        if (robot->robot_joint_offsets.size() != robot->robot_joint_names.size())
            throw std::runtime_error("deploy.yaml declares `calibration` but the robot supplied "
                                     "no joint offsets to the runtime");
        for (std::size_t i = 0; i < n; ++i)
            d.joint_pos_shift[i] = robot->robot_joint_offsets[static_cast<std::size_t>(d.joint_ids_map[i])] - off[i];
    }
    d.default_joint_pos = per_joint(cfg["default_joint_pos"], n, "default_joint_pos");
    d.joint_stiffness = per_joint(cfg["stiffness"], n, "stiffness");
    d.joint_damping = per_joint(cfg["damping"], n, "damping");
    for (std::size_t i = 0; i < n; ++i)
        if (!(d.joint_stiffness[i] >= 0.0f) || !(d.joint_damping[i] >= 0.0f))
            throw std::runtime_error("deploy.yaml: stiffness/damping must be >= 0");

    if (const YAML::Node c = cfg["commands"])
        if (const YAML::Node r = c["base_velocity"] ? c["base_velocity"]["ranges"] : YAML::Node())
        {
            command_ranges.lin_vel_x = range(r["lin_vel_x"], "lin_vel_x", command_ranges.lin_vel_x);
            command_ranges.lin_vel_y = range(r["lin_vel_y"], "lin_vel_y", command_ranges.lin_vel_y);
            command_ranges.ang_vel_z = range(r["ang_vel_z"], "ang_vel_z", command_ranges.ang_vel_z);
        }

    robot->update();
    // Actions first: last_action reads the action manager while the
    // observation manager measures term widths.
    action_manager = std::make_unique<ActionManager>(cfg["actions"], this);
    observation_manager = std::make_unique<ObservationManager>(cfg["observations"], this);
    // Measuring widths advanced the gait clock / yaw origin: start clean.
    reset();
}

void ManagerBasedRLEnv::set_algorithm(std::unique_ptr<Algorithms> a)
{
    if (!a) throw std::runtime_error("set_algorithm: null model");
    const auto& groups = observation_manager->groups();
    const auto& in_names = a->inputNames();

    if (in_names.size() == 1 && groups.size() == 1 && in_names[0] != groups[0].name)
        throw std::runtime_error("the model's input is named '" + in_names[0] +
                                 "' but the only observation group is named '" + groups[0].name +
                                 "'. Rename the group in deploy.yaml to match the model -- the "
                                 "group name IS the binding.");
    for (std::size_t i = 0; i < in_names.size(); ++i)
    {
        const ObservationManager::Group* g = nullptr;
        for (const auto& c : groups)
            if (c.name == in_names[i]) g = &c;
        if (!g)
        {
            std::ostringstream os;
            os << "model input '" << in_names[i] << "' has no observation group. Configured:";
            for (const auto& c : groups) os << " " << c.name;
            throw std::runtime_error(os.str());
        }
        if (const std::string bad = checkTensorContract(a->inputSpec(i), "input", g->dim); !bad.empty())
            throw std::runtime_error(bad + " (observation group '" + g->name + "')");
        if (a->inputSize(i) != g->dim)
            throw std::runtime_error("model input '" + in_names[i] + "' expects " +
                                     std::to_string(a->inputSize(i)) + " values but observation "
                                     "group '" + g->name + "' produces " + std::to_string(g->dim));
    }
    // OrtRunner returns output 0 only; a second output would be silently ignored.
    if (a->outputCount() != 1)
        throw std::runtime_error("the model has " + std::to_string(a->outputCount()) +
                                 " outputs; exactly 1 (the action) is supported");
    const auto act_dim = static_cast<std::size_t>(action_manager->total_action_dim());
    if (const std::string bad = checkTensorContract(a->outputSpec(0), "output", act_dim); !bad.empty())
        throw std::runtime_error(bad + " (configured action terms)");
    if (a->outputSize() != act_dim)
        throw std::runtime_error("the model outputs " + std::to_string(a->outputSize()) +
                                 " values but the configured action terms expect " +
                                 std::to_string(act_dim));
    alg = std::move(a);
}

void ManagerBasedRLEnv::reset()
{
    robot->update();
    action_manager->reset();
    observation_manager->reset();
    // After the observation reset, which may sample the terms (history_init:
    // current): the first step() still sees phase 0 and captures the yaw origin.
    episode_length = 0;
    global_phase = 0.0f;
    yaw_origin = 0.0f;
    yaw_origin_set = false;
}

void ManagerBasedRLEnv::step()
{
    if (!alg) throw std::runtime_error("ManagerBasedRLEnv::step: no model attached");
    robot->update();
    if (const std::string why = observation_manager->unavailable(robot->data); !why.empty())
        throw std::runtime_error("OBSERVATION_UNAVAILABLE: " + why + " -- inference not run");
    const auto obs = observation_manager->compute();
    const auto& action = alg->act(obs);
    // Refused before the action manager stores it as the next last_action.
    for (std::size_t i = 0; i < action.size(); ++i)
        if (!std::isfinite(action[i]))
            throw std::runtime_error("model output[" + std::to_string(i) + "] is not finite");
    action_manager->process_action(action);
    ++episode_length;
}

std::string ManagerBasedRLEnv::describe() const
{
    std::ostringstream os;
    const auto& d = robot->data;
    os << "ManagerBasedRLEnv\n"
       << "  step_dt " << step_dt << " s (" << 1.0 / step_dt << " Hz)\n"
       << "  command ranges: vx [" << command_ranges.lin_vel_x[0] << ", "
       << command_ranges.lin_vel_x[1] << "]  vy [" << command_ranges.lin_vel_y[0] << ", "
       << command_ranges.lin_vel_y[1] << "]  wz [" << command_ranges.ang_vel_z[0] << ", "
       << command_ranges.ang_vel_z[1] << "]\n"
       << observation_manager->describe() << action_manager->describe();
    os << (alg ? alg->describe() : std::string("  model: NONE ATTACHED\n"));
    os << "  policy joint -> robot joint, default q, kp, kd\n";
    for (std::size_t i = 0; i < d.joint_ids_map.size(); ++i)
        os << "    [" << i << "] " << robot->robot_joint_names[d.joint_ids_map[i]] << " -> robot["
           << d.joint_ids_map[i] << "]  shift=" << d.joint_pos_shift[i] << "  q0="
           << d.default_joint_pos[i] << "  kp="
           << d.joint_stiffness[i] << "  kd=" << d.joint_damping[i] << "\n";
    return os.str();
}

} // namespace isaaclab
