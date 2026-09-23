#include "rl/PolicyEnvironment.h"

// Including the term headers is what REGISTERS them: each MINI_PI_REGISTER_*
// macro defines an inline registrar object whose constructor inserts into the
// global map. If this include disappears the maps are empty and every
// deploy.yaml fails with "term is not registered", which is the intended
// failure mode -- it can never silently produce a wrong observation.
#include "rl/mdp/JointActions.h"
#include "rl/mdp/Observations.h"

#include <chrono>
#include <sstream>
#include <stdexcept>

namespace mini_pi
{
namespace rl
{
namespace
{
std::array<float, 2> loadRange(const YAML::Node& n, const char* what,
                               const std::array<float, 2>& fallback)
{
    if (!n || n.IsNull()) return fallback;
    const auto v = n.as<std::vector<float>>();
    if (v.size() != 2)
        throw std::runtime_error(std::string("deploy.yaml commands.base_velocity.ranges.") +
                                 what + " must be [min, max]");
    if (!(v[0] <= v[1]))
        throw std::runtime_error(std::string("deploy.yaml commands.base_velocity.ranges.") +
                                 what + ": min must be <= max");
    return {v[0], v[1]};
}

std::vector<float> loadPerJoint(const YAML::Node& n, std::size_t dim, const char* what,
                                bool required)
{
    if (!n || n.IsNull())
    {
        if (required)
            throw std::runtime_error(std::string("deploy.yaml: `") + what + "` is required");
        return std::vector<float>(dim, 0.0f);
    }
    if (n.IsScalar()) return std::vector<float>(dim, n.as<float>());
    const auto v = n.as<std::vector<float>>();
    if (v.size() != dim)
        throw std::runtime_error(std::string("deploy.yaml: `") + what + "` has " +
                                 std::to_string(v.size()) + " values but joint_names has " +
                                 std::to_string(dim));
    return v;
}
} // namespace

PolicyEnvironment::PolicyEnvironment(const YAML::Node& cfg_in,
                                     const std::vector<std::string>& robot_joint_names)
    : cfg(cfg_in)
{
    if (!cfg || !cfg.IsMap()) throw std::runtime_error("deploy.yaml is empty or is not a map");

    if (!cfg["step_dt"]) throw std::runtime_error("deploy.yaml: `step_dt` is required");
    step_dt = cfg["step_dt"].as<float>();
    if (!(step_dt > 0.0f)) throw std::runtime_error("deploy.yaml: `step_dt` must be > 0");

    if (!cfg["joint_names"])
        throw std::runtime_error("deploy.yaml: `joint_names` is required. It is the POLICY "
                                 "joint order and is resolved by name against mapping.yaml.");
    const auto policy_joint_names = cfg["joint_names"].as<std::vector<std::string>>();

    robot = std::make_shared<RobotView>(robot_joint_names, policy_joint_names);
    const std::size_t n = robot->numJoints();

    robot->data.default_joint_pos =
        loadPerJoint(cfg["default_joint_pos"], n, "default_joint_pos", true);

    stiffness = loadPerJoint(cfg["stiffness"], n, "stiffness", true);
    damping   = loadPerJoint(cfg["damping"], n, "damping", true);
    for (std::size_t i = 0; i < n; ++i)
    {
        if (stiffness[i] < 0.0f || damping[i] < 0.0f)
            throw std::runtime_error("deploy.yaml: stiffness/damping must be >= 0 (joint " +
                                     policy_joint_names[i] + ")");
    }

    // Precompute the robot-order gain arrays: they never change at runtime,
    // and doing the scatter once keeps step() free of per-tick index work.
    kp_robot_.fill(0.0);
    kd_robot_.fill(0.0);
    robot->policyToRobotArray(stiffness, kp_robot_);
    robot->policyToRobotArray(damping, kd_robot_);
    last_target_.fill(0.0);

    if (const YAML::Node c = cfg["commands"])
    {
        if (const YAML::Node r = c["base_velocity"] ? c["base_velocity"]["ranges"] : YAML::Node())
        {
            command_ranges.lin_vel_x = loadRange(r["lin_vel_x"], "lin_vel_x",
                                                 command_ranges.lin_vel_x);
            command_ranges.lin_vel_y = loadRange(r["lin_vel_y"], "lin_vel_y",
                                                 command_ranges.lin_vel_y);
            // Accept both spellings: IsaacLab/Unitree say ang_vel_z, the task
            // brief says ang_vel_yaw. They are the same quantity.
            command_ranges.ang_vel_z = loadRange(r["ang_vel_z"] ? r["ang_vel_z"]
                                                                : r["ang_vel_yaw"],
                                                 "ang_vel_z", command_ranges.ang_vel_z);
        }
    }

    // Managers last: the observation terms call back into this object (for
    // step_dt, command_ranges, robot and action_manager) while measuring their
    // own width, so everything they read must already be in place.
    //
    // ActionManager before ObservationManager, because the `last_action` term
    // reads action_manager->action() during that width measurement.
    action_manager      = std::make_unique<ActionManager>(cfg["actions"], this);
    observation_manager = std::make_unique<ObservationManager>(cfg["observations"], this);

    if (action_manager->total_action_dim() != static_cast<int>(n))
        throw std::runtime_error("deploy.yaml: the action terms produce " +
                                 std::to_string(action_manager->total_action_dim()) +
                                 " values but the policy has " + std::to_string(n) + " joints");

    // Measuring term widths advanced the gait phase and left the history
    // holding those probe frames. Discard both.
    reset();
}

void PolicyEnvironment::setModel(std::unique_ptr<ModelRunner> model)
{
    if (!model) throw std::runtime_error("PolicyEnvironment::setModel: null model");

    const auto& groups = observation_manager->groups();
    const auto& in_names = model->inputNames();

    // Bind by name when the names agree. Fall back to positional binding only
    // in the unambiguous 1-input / 1-group case, and say so, because a silent
    // positional bind on a multi-input model would pair the wrong tensors.
    if (in_names.size() == 1 && groups.size() == 1 && in_names[0] != groups[0].name)
    {
        throw std::runtime_error(
            "the model's input is named '" + in_names[0] + "' but the only observation group "
            "is named '" + groups[0].name + "'. Rename the group in deploy.yaml to match the "
            "model -- the group name IS the binding.");
    }

    for (std::size_t i = 0; i < in_names.size(); ++i)
    {
        const ObservationGroupCfg* g = nullptr;
        for (const auto& c : groups)
            if (c.name == in_names[i]) { g = &c; break; }

        if (!g)
        {
            std::ostringstream os;
            os << "model input '" << in_names[i] << "' has no observation group. Configured:";
            for (const auto& c : groups) os << " " << c.name;
            throw std::runtime_error(os.str());
        }
        if (g->totalDim() != model->inputSize(i))
            throw std::runtime_error("model input '" + in_names[i] + "' expects " +
                                     std::to_string(model->inputSize(i)) +
                                     " values but observation group '" + g->name +
                                     "' produces " + std::to_string(g->frame_dim) + " x " +
                                     std::to_string(g->history_length) + " = " +
                                     std::to_string(g->totalDim()));
    }

    if (model->outputSize() != static_cast<std::size_t>(action_manager->total_action_dim()))
        throw std::runtime_error("the model outputs " + std::to_string(model->outputSize()) +
                                 " values but the configured action terms expect " +
                                 std::to_string(action_manager->total_action_dim()));

    model_ = std::move(model);
}

void PolicyEnvironment::reset()
{
    global_phase   = 0.0f;
    episode_length = 0;
    yaw_origin     = 0.0f;
    yaw_origin_set = false;
    action_manager->reset();
    observation_manager->reset();
    inference_count_ = 0;
    inference_total_s_ = 0.0;
    mean_inference_s_ = 0.0;
    max_inference_s_ = 0.0;
    // The default pose, in robot order: what step() would produce from a zero
    // action. A consumer reading lastTargetRobot() before the first step gets
    // a sane pose rather than joint zero.
    last_target_.fill(0.0);
    robot->policyToRobotArray(action_manager->processed_actions(), last_target_);
}

void PolicyEnvironment::step(const RobotState& state, const VelocityCommand& cmd,
                             RobotCommand& out)
{
    if (!model_) throw std::runtime_error("PolicyEnvironment::step: no model attached");

    ++episode_length;
    robot->update(state, cmd);

    const auto obs = observation_manager->compute();
    const auto inference_start = std::chrono::steady_clock::now();
    const auto& action = model_->act(obs);
    const double inference_s = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - inference_start).count();
    inference_total_s_ += inference_s;
    ++inference_count_;
    mean_inference_s_ = inference_total_s_ / static_cast<double>(inference_count_);
    double previous_max = max_inference_s_.load();
    while (inference_s > previous_max &&
           !max_inference_s_.compare_exchange_weak(previous_max, inference_s)) {}
    action_manager->process_action(action);

    robot->policyToRobotArray(action_manager->processed_actions(), last_target_);

    out.q  = last_target_;
    out.kp = kp_robot_;
    out.kd = kd_robot_;
    // Position control only. JointPositionAction commands no velocity and no
    // feedforward torque, and neither does any other term today.
    out.dq.fill(0.0);
    out.torque.fill(0.0);
    // Deliberately NOT stamping out.valid / out.stamp / out.seq. See the
    // header: freshness is asserted by the FSM thread, not by the producer.
}

std::string PolicyEnvironment::describe() const
{
    std::ostringstream os;
    os << "rl::PolicyEnvironment\n"
       << "  step_dt " << step_dt << " s (" << (1.0f / step_dt) << " Hz)\n"
       << "  command ranges: vx [" << command_ranges.lin_vel_x[0] << ", "
       << command_ranges.lin_vel_x[1] << "]  vy [" << command_ranges.lin_vel_y[0] << ", "
       << command_ranges.lin_vel_y[1] << "]  wz [" << command_ranges.ang_vel_z[0] << ", "
       << command_ranges.ang_vel_z[1] << "]\n"
       << observation_manager->describe()
       << action_manager->describe();
    if (model_) os << model_->describe();
    else        os << "  model: NONE ATTACHED\n";
    os << robot->describe();
    os << "  stiffness (policy order): ";
    for (std::size_t i = 0; i < stiffness.size(); ++i) os << (i ? " " : "") << stiffness[i];
    os << "\n  damping   (policy order): ";
    for (std::size_t i = 0; i < damping.size(); ++i) os << (i ? " " : "") << damping[i];
    os << "\n";
    return os.str();
}

} // namespace rl
} // namespace mini_pi
