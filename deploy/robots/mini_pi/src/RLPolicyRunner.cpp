#include "RLPolicyRunner.h"

#include <ros/ros.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <sstream>
#include <utility>

namespace mini_pi
{
namespace
{
namespace fs = std::filesystem;

std::string join(const fs::path& a, const char* b, const char* c) { return (a / b / c).string(); }

/// True when `dir` has both required files; `why` receives the first missing one.
bool isPolicyPackage(const std::string& dir, std::string* why = nullptr)
{
    std::error_code ec;
    if (!fs::is_directory(dir, ec))
    {
        if (why) *why = dir + " is not a directory";
        return false;
    }
    // Both are required. Accepting on `exported/` alone -- as Unitree does --
    // turns a half-copied package into a crash deep inside construction.
    for (const auto& f : {join(dir, kParamsSubdir, kDeployYamlName),
                          join(dir, kExportedSubdir, kModelName)})
        if (!fs::is_regular_file(f, ec))
        {
            if (why) *why = "missing " + f;
            return false;
        }
    return true;
}

PolicyPackage accept(const fs::path& dir)
{
    PolicyPackage pkg;
    pkg.dir = dir.string();
    pkg.deploy_yaml = join(dir, kParamsSubdir, kDeployYamlName);
    pkg.model = join(dir, kExportedSubdir, kModelName);
    pkg.version = dir.filename().string();
    pkg.valid = true;
    return pkg;
}

PolicyPackage reject(std::string why)
{
    PolicyPackage pkg;
    pkg.error = std::move(why);
    return pkg;
}

/// Sorted names of the valid packages directly under `root`, or "none".
std::string validPackageNames(const fs::path& root)
{
    std::vector<std::string> names;
    std::error_code ec;
    for (const auto& e : fs::directory_iterator(root, ec))
        if (e.is_directory(ec) && isPolicyPackage(e.path().string()))
            names.push_back(e.path().filename().string());
    std::sort(names.begin(), names.end());
    std::string out;
    for (const auto& n : names) out += (out.empty() ? "" : " ") + n;
    return out.empty() ? "none" : out;
}

CommandLimits limitsFrom(const isaaclab::CommandRanges& r)
{
    CommandLimits l;
    l.vx = {r.lin_vel_x[0], r.lin_vel_x[1]};
    l.vy = {r.lin_vel_y[0], r.lin_vel_y[1]};
    l.wz = {r.ang_vel_z[0], r.ang_vel_z[1]};
    return l;
}

/// deploy.yaml operator.keyboard + the env's command ranges -> envelope.
std::string readOperatorEnvelope(const YAML::Node& deploy, const isaaclab::CommandRanges& r,
                                 OperatorEnvelope& env)
{
    env.limits = limitsFrom(r);
    const YAML::Node kb = deploy["operator"] ? deploy["operator"]["keyboard"] : YAML::Node();
    for (auto [key, dst] : {std::pair<const char*, double*>{"linear_step", &env.key_linear_step},
                            {"yaw_step", &env.key_yaw_step}})
    {
        if (!kb || !kb[key] || !kb[key].IsScalar())
            return std::string("operator.keyboard.") + key + " is required";
        *dst = kb[key].as<double>();
        if (!(*dst > 0.0) || !std::isfinite(*dst))
            return std::string("operator.keyboard.") + key + " must be a finite value > 0";
    }
    for (const auto* a : {&env.limits.vx, &env.limits.vy, &env.limits.wz})
        if ((*a)[0] > 0.0 || (*a)[1] < 0.0)
            return "commands.base_velocity.ranges must contain 0";
    return {};
}
} // namespace

// ============================== package on disk ==============================

PolicyPackage resolvePolicyPackage(const std::string& policy_dir, const std::string& base_dir,
                                   const std::string& version)
{
    if (policy_dir.empty()) return reject("policy_dir is empty");

    fs::path root(policy_dir);
    if (root.is_relative() && !base_dir.empty()) root = fs::path(base_dir) / root;

    std::error_code ec;
    if (!fs::is_directory(root, ec)) return reject("policy_dir '" + root.string() + "' does not exist");

    if (!version.empty())
    {
        const fs::path cand = root / version;
        if (!fs::is_directory(cand, ec))
            return reject("Unknown policy package: " + version + " (no directory '" +
                          cand.string() + "'; available: " + validPackageNames(root) + ")");
        std::string why;
        if (!isPolicyPackage(cand.string(), &why))
            return reject("policy version '" + version + "' is not a valid package: " + why);
        return accept(cand);
    }

    if (isPolicyPackage(root.string())) return accept(root);

    std::vector<fs::path> candidates;
    for (const auto& e : fs::directory_iterator(root, ec))
        if (e.is_directory(ec)) candidates.push_back(e.path());
    if (candidates.empty())
        return reject("policy_dir '" + root.string() +
                      "' is not a policy package and contains no version directories (expected " +
                      kParamsSubdir + "/" + kDeployYamlName + " and " + kExportedSubdir + "/" +
                      kModelName + ")");

    // Lexicographic, as Unitree's parser_policy_dir: the last valid name wins.
    // A STRING sort, so v10 < v9 -- zero-pad beyond v9.
    std::sort(candidates.begin(), candidates.end());
    std::string rejected;
    for (auto it = candidates.rbegin(); it != candidates.rend(); ++it)
    {
        std::string why;
        if (isPolicyPackage(it->string(), &why)) return accept(*it);
        rejected += (rejected.empty() ? "" : "; ") + why;
    }
    return reject("no valid policy package under '" + root.string() + "' (" + rejected + ")");
}

// ================================ runner =====================================

const char* to_string(PolicyStatus s)
{
    switch (s)
    {
        case PolicyStatus::NotConfigured: return "POLICY_NOT_CONFIGURED";
        case PolicyStatus::Ready:         return "POLICY_READY";
    }
    return "UNKNOWN";
}

RLPolicyRunner::RLPolicyRunner() = default;
RLPolicyRunner::~RLPolicyRunner() { stop(); }

std::unique_ptr<isaaclab::ManagerBasedRLEnv> RLPolicyRunner::buildEnv(
    const YAML::Node& deploy, const std::string& deploy_path, const std::string& model_path,
    const std::vector<std::string>& robot_joint_names, std::string& error,
    std::unique_ptr<isaaclab::Algorithms> alg, const std::vector<double>& robot_joint_offsets)
{
    std::unique_ptr<isaaclab::ManagerBasedRLEnv> env;
    try
    {
        env = std::make_unique<isaaclab::ManagerBasedRLEnv>(
            deploy, std::make_shared<MiniPiArticulation>(robot_joint_names, robot_joint_offsets));
    }
    catch (const std::exception& e)
    {
        error = "invalid " + deploy_path + ": " + e.what();
        return nullptr;
    }
    // Configuration first, so a malformed deploy.yaml is reported as such
    // even when the model is also missing.
    if (!alg) alg = isaaclab::createOrtRunner(model_path, error);
    if (!alg) return nullptr;
    try
    {
        env->set_algorithm(std::move(alg));
    }
    catch (const std::exception& e)
    {
        error = "model does not match " + deploy_path + ": " + e.what();
        return nullptr;
    }
    return env;
}

bool RLPolicyRunner::initialize(const std::string& policy_yaml, const Options& opt)
{
    stop();
    status_ = PolicyStatus::NotConfigured;
    reason_ = "POLICY_NOT_CONFIGURED";
    env_.reset();
    robot_ = nullptr;
    envelope_ = OperatorEnvelope{};
    package_ = PolicyPackage{};

    if (opt.robot_joint_names.empty())
    {
        ROS_FATAL("RLPolicyRunner: no robot joint names supplied");
        return false;
    }
    auto refuse = [this](const std::string& why) {
        reason_ = "POLICY_NOT_CONFIGURED (" + why + ")";
        ROS_ERROR("RLPolicyRunner: %s", reason_.c_str());
        return true;
    };

    YAML::Node root;
    try { root = YAML::LoadFile(policy_yaml); }
    catch (const YAML::Exception& e) { return refuse("cannot read " + policy_yaml + ": " + e.what()); }

    const YAML::Node p = root["policy"];
    const std::string state_name = (p && p["state"]) ? p["state"].as<std::string>() : "Velocity";
    std::string version = (p && p["version"]) ? p["version"].as<std::string>() : std::string();
    if (!opt.version_override.empty()) version = opt.version_override;
    const bool allow_test = opt.allow_test_policy_override ||
                            (p && p["allow_test_policy"] && p["allow_test_policy"].as<bool>());

    std::string policy_dir;
    if (p && p["policy_dir"]) policy_dir = p["policy_dir"].as<std::string>();
    if (opt.fsm && opt.fsm[state_name] && opt.fsm[state_name]["policy_dir"])
        policy_dir = opt.fsm[state_name]["policy_dir"].as<std::string>();
    if (!opt.policy_dir_override.empty()) policy_dir = opt.policy_dir_override;
    if (policy_dir.empty()) return refuse("no policy_dir: set FSM." + state_name + ".policy_dir in fsm.yaml");

    package_ = resolvePolicyPackage(policy_dir, opt.base_dir, version);
    if (!package_.valid) return refuse(package_.error);
    ROS_INFO("RLPolicyRunner: selected policy package '%s' (version '%s')", package_.dir.c_str(),
             package_.version.c_str());

    YAML::Node deploy;
    try { deploy = YAML::LoadFile(package_.deploy_yaml); }
    catch (const YAML::Exception& e) { return refuse("malformed " + package_.deploy_yaml + ": " + e.what()); }

    // The real-hardware lock, BEFORE the env is built.
    if (deploy["simulation_only"] && deploy["simulation_only"].as<bool>())
    {
        if (!allow_test)
            return refuse("package '" + package_.dir + "' declares simulation_only: true, but "
                          "policy.yaml allow_test_policy is false");
        if (!opt.backend_is_simulation)
            return refuse("package '" + package_.dir + "' declares simulation_only: true and the "
                          "hardware backend is '" + opt.backend + "', which is not a simulation");
        ROS_WARN("RLPolicyRunner: SIMULATION-ONLY TEST POLICY %s on backend %s",
                 package_.dir.c_str(), opt.backend.c_str());
    }

    std::string error;
    auto env = buildEnv(deploy, package_.deploy_yaml, package_.model, opt.robot_joint_names, error,
                        nullptr, opt.robot_joint_offsets);
    if (!env) return refuse(error);

    OperatorEnvelope envelope;
    if (const std::string e = readOperatorEnvelope(deploy, env->command_ranges, envelope); !e.empty())
        return refuse(package_.deploy_yaml + ": " + e);

    adopt_(std::move(env));
    envelope_ = envelope;
    ROS_INFO_STREAM("\n" << describe());
    return true;
}

void RLPolicyRunner::adopt_(std::unique_ptr<isaaclab::ManagerBasedRLEnv> env)
{
    env_ = std::move(env);
    robot_ = static_cast<MiniPiArticulation*>(env_->robot.get());   // buildEnv() made it
    const auto& d = env_->robot->data;
    kp_.fill(0.0);
    kd_.fill(0.0);
    for (std::size_t i = 0; i < d.joint_ids_map.size(); ++i)
    {
        kp_[static_cast<std::size_t>(d.joint_ids_map[i])] = d.joint_stiffness[i];
        kd_[static_cast<std::size_t>(d.joint_ids_map[i])] = d.joint_damping[i];
    }
    status_ = PolicyStatus::Ready;
    reason_.clear();
    reset();
}

void RLPolicyRunner::setStateProvider(StateProvider fn)
{
    if (robot_) robot_->source = std::move(fn);
}

void RLPolicyRunner::reset()
{
    if (env_) env_->reset();
    steps_ = 0;
    mean_period_ = 0.0;
    {
        std::lock_guard<std::mutex> lk(timing_mutex_);
        timing_ = PolicyTiming{};
    }
    failed_ = false;
    {
        std::lock_guard<std::mutex> lk(failure_mutex_);
        failure_.clear();
    }
    std::lock_guard<std::mutex> lk(output_mutex_);
    output_ = PolicyOutput{};
    last_frame_.clear();
}

float RLPolicyRunner::stepDt() const
{
    return env_ ? static_cast<float>(env_->step_dt) : 0.0f;
}

std::string RLPolicyRunner::entryBlocker(const RobotState& state) const
{
    if (!env_) return {};
    // Availability only: a fresh ArticulationData (no joint map) filled from
    // `state`, so this never touches the data the policy thread is writing.
    isaaclab::ArticulationData d;
    MiniPiArticulation::fill(state, VelocityCommand{}, d);
    const std::string why = env_->observation_manager->unavailable(d);
    return why.empty() ? why : "observation " + why;
}

std::string RLPolicyRunner::describeFrame(const std::vector<float>& frame) const
{
    return env_ ? env_->observation_manager->describe_frame(frame) : std::string();
}

PolicyTiming RLPolicyRunner::timing() const
{
    std::lock_guard<std::mutex> lk(timing_mutex_);
    return timing_;
}

bool RLPolicyRunner::start()
{
    if (running_) return true;
    if (!ready())
    {
        ROS_ERROR("RLPolicyRunner::start() refused: %s", reason_.c_str());
        return false;
    }
    if (!robot_->source)
    {
        ROS_ERROR("RLPolicyRunner::start() refused: no state provider installed");
        return false;
    }
    running_ = true;
    thread_ = std::thread(&RLPolicyRunner::threadBody_, this);
    return true;
}

void RLPolicyRunner::stop()
{
    running_ = false;
    if (thread_.joinable()) thread_.join();
}

void RLPolicyRunner::threadBody_()
{
    // sleep_until on a fixed grid, like Unitree's policy thread, re-anchored
    // after an overrun.
    const auto period = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(env_->step_dt));
    auto next = Clock::now() + period;
    const auto t_start = Clock::now();
    TimePoint prev_start{};
    PolicyOutput out;
    const std::string first_group = env_->observation_manager->groups().front().name;

    while (running_)
    {
        const TimePoint step_start = Clock::now();
        try
        {
            env_->step();
        }
        catch (const std::exception& e)
        {
            fail_(e.what());
            return;
        }

        robot_->to_robot(env_->action_manager->processed_actions(), out.cmd.q);
        out.cmd.kp = kp_;
        out.cmd.kd = kd_;
        out.cmd.dq.fill(0.0);
        out.cmd.torque.fill(0.0);
        // Policy-independent backstop: a non-finite target never leaves here.
        if (!is_finite(out.cmd.q))
        {
            fail_("the policy produced a non-finite joint target");
            return;
        }
        const auto& a = env_->action_manager->action();
        for (std::size_t i = 0; i < out.action.size() && i < a.size(); ++i) out.action[i] = a[i];
        out.observation_min = std::numeric_limits<float>::infinity();
        out.observation_max = -std::numeric_limits<float>::infinity();
        out.observation_finite = true;
        for (const auto& g : env_->observation_manager->groups())
            for (float v : g.obs)
            {
                out.observation_finite = out.observation_finite && std::isfinite(v);
                out.observation_min = std::min(out.observation_min, v);
                out.observation_max = std::max(out.observation_max, v);
            }
        std::vector<float> frame = env_->observation_manager->last_frame(first_group);

        const auto now = Clock::now();
        {
            std::lock_guard<std::mutex> lk(output_mutex_);
            const std::uint64_t seq = output_.seq;
            output_ = out;
            last_frame_.swap(frame);
            output_.sample_stamp = step_start;
            output_.stamp = now;
            output_.seq = seq + 1;
            output_.valid = true;
        }

        const auto n = steps_.fetch_add(1) + 1;
        mean_period_ = std::chrono::duration<double>(now - t_start).count() / static_cast<double>(n);

        bool overran = false;
        next = nextPolicyDeadline(next, now, period, overran);
        {
            std::lock_guard<std::mutex> lk(timing_mutex_);
            const double update_s = std::chrono::duration<double>(now - step_start).count();
            timing_.last_update = update_s;
            timing_.max_update = std::max(timing_.max_update, update_s);
            timing_.mean_update += (update_s - timing_.mean_update) / static_cast<double>(n);
            if (prev_start != TimePoint{})
            {
                const double p = std::chrono::duration<double>(step_start - prev_start).count();
                timing_.last_period = p;
                timing_.min_period = timing_.min_period > 0.0 ? std::min(timing_.min_period, p) : p;
                timing_.max_period = std::max(timing_.max_period, p);
            }
            if (overran) ++timing_.overruns;
        }
        prev_start = step_start;

        std::this_thread::sleep_until(next);
        next += period;
    }
}

void RLPolicyRunner::fail_(const std::string& why)
{
    {
        std::lock_guard<std::mutex> lk(failure_mutex_);
        failure_ = why;
    }
    {
        // A failed policy vouches for nothing: withdraw the last target.
        std::lock_guard<std::mutex> lk(output_mutex_);
        output_.valid = false;
    }
    failed_ = true;
    running_ = false;
    ROS_ERROR("RLPolicyRunner: policy thread ABORTED: %s. The published command is withdrawn; "
              "State_RLBase damps and leaves RL.", why.c_str());
}

PolicyOutput RLPolicyRunner::latest() const
{
    std::lock_guard<std::mutex> lk(output_mutex_);
    return output_;
}

std::vector<float> RLPolicyRunner::latestFrame() const
{
    std::lock_guard<std::mutex> lk(output_mutex_);
    return last_frame_;
}

std::string RLPolicyRunner::failureReason() const
{
    std::lock_guard<std::mutex> lk(failure_mutex_);
    return failure_;
}

std::string RLPolicyRunner::describe() const
{
    std::ostringstream os;
    os << "RLPolicyRunner: status=" << to_string(status_) << "\n";
    if (!reason_.empty()) os << "  reason : " << reason_ << "\n";
    if (package_.valid)
        os << "  package: " << package_.dir << "  (version '" << package_.version << "')\n"
           << "  config : " << package_.deploy_yaml << "\n"
           << "  model  : " << package_.model << "\n";
    if (!env_) return os.str() + "  policy: NONE\n";
    const auto& e = envelope_;
    os << "  operator envelope: vx [" << e.limits.vx[0] << ", " << e.limits.vx[1] << "]  vy ["
       << e.limits.vy[0] << ", " << e.limits.vy[1] << "]  wz [" << e.limits.wz[0] << ", "
       << e.limits.wz[1] << "]  key step linear " << e.key_linear_step << " yaw "
       << e.key_yaw_step << "\n"
       << env_->describe();
    return os.str();
}

} // namespace mini_pi
