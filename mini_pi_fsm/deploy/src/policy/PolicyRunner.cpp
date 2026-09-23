#include "policy/PolicyRunner.h"

#include <cmath>

#include <ros/ros.h>

#include <algorithm>
#include <limits>
#include <sstream>

namespace mini_pi
{

const char* to_string(PolicyStatus s)
{
    switch (s)
    {
        case PolicyStatus::NotConfigured: return "POLICY_NOT_CONFIGURED";
        case PolicyStatus::Ready:         return "POLICY_READY";
    }
    return "UNKNOWN";
}

PolicyRunner::~PolicyRunner() { stop(); }

bool PolicyRunner::initialize(const std::string& policy_yaml, const Options& opt)
{
    status_ = PolicyStatus::NotConfigured;
    reason_ = "POLICY_NOT_CONFIGURED";
    env_.reset();
    package_ = rl::PolicyPackage{};

    if (opt.robot_joint_names.empty())
    {
        ROS_FATAL("PolicyRunner: no robot joint names supplied");
        return false;
    }

    // ---- runtime policy settings -----------------------------------------
    YAML::Node root;
    try { root = YAML::LoadFile(policy_yaml); }
    catch (const YAML::Exception& e)
    {
        reason_ = std::string("POLICY_NOT_CONFIGURED (cannot read ") + policy_yaml + ": " +
                  e.what() + ")";
        ROS_WARN("PolicyRunner: %s", reason_.c_str());
        return true;
    }

    const YAML::Node p = root["policy"];
    const std::string state_name = (p && p["state"]) ? p["state"].as<std::string>()
                                                     : std::string("Velocity");
    std::string version = (p && p["version"]) ? p["version"].as<std::string>()
                                              : std::string();
    if (!opt.version_override.empty()) version = opt.version_override;

    const bool allow_test = opt.allow_test_policy_override ||
                            (p && p["allow_test_policy"] && p["allow_test_policy"].as<bool>());

    // ---- where the package lives (task §4: fsm.yaml FSM.<state>.policy_dir)
    std::string policy_dir;
    if (p && p["policy_dir"]) policy_dir = p["policy_dir"].as<std::string>();
    if (opt.fsm && opt.fsm[state_name] && opt.fsm[state_name]["policy_dir"])
        policy_dir = opt.fsm[state_name]["policy_dir"].as<std::string>();
    if (!opt.policy_dir_override.empty()) policy_dir = opt.policy_dir_override;
    if (policy_dir.empty())
    {
        reason_ = "POLICY_NOT_CONFIGURED (no policy_dir: set FSM." + state_name +
                  ".policy_dir in fsm.yaml)";
        ROS_WARN("PolicyRunner: %s", reason_.c_str());
        return true;
    }

    package_ = rl::resolvePolicyPackage(policy_dir, opt.base_dir, version);
    if (!package_.valid)
    {
        reason_ = "POLICY_NOT_CONFIGURED (" + package_.error + ")";
        ROS_WARN("PolicyRunner: %s", reason_.c_str());
        ROS_WARN("PolicyRunner: this is the EXPECTED state until a trained policy is "
                 "installed. Drop a package at <policy_dir>/<version>/{params/deploy.yaml,"
                 "exported/policy.onnx} and restart.");
        return true;
    }
    ROS_INFO("PolicyRunner: selected policy package '%s' (version '%s')",
             package_.dir.c_str(), package_.version.c_str());

    // ---- deploy.yaml ------------------------------------------------------
    YAML::Node deploy;
    try { deploy = YAML::LoadFile(package_.deploy_yaml); }
    catch (const YAML::Exception& e)
    {
        reason_ = std::string("POLICY_NOT_CONFIGURED (malformed ") + package_.deploy_yaml +
                  ": " + e.what() + ")";
        ROS_ERROR("PolicyRunner: %s", reason_.c_str());
        return true;
    }

    // ---- the real-hardware lock (task §33) --------------------------------
    // Checked BEFORE the environment is built, so a test package is never
    // even instantiated on the robot.
    const bool sim_only = deploy["simulation_only"] && deploy["simulation_only"].as<bool>();
    if (sim_only)
    {
        if (!allow_test)
        {
            reason_ = "POLICY_NOT_CONFIGURED (package '" + package_.dir +
                      "' declares simulation_only: true, but policy.yaml allow_test_policy "
                      "is false)";
            ROS_ERROR("PolicyRunner: %s", reason_.c_str());
            return true;
        }
        if (!opt.backend_is_simulation)
        {
            reason_ = "POLICY_NOT_CONFIGURED (package '" + package_.dir +
                      "' declares simulation_only: true and the hardware backend is '" +
                      opt.backend + "', which is not a simulation)";
            ROS_ERROR("PolicyRunner: %s -- REFUSING. A simulation-only policy must never "
                      "drive real motors.", reason_.c_str());
            return true;
        }
        ROS_WARN("PolicyRunner: ======================================================");
        ROS_WARN("PolicyRunner: THIS IS A SIMULATION-ONLY TEST POLICY, NOT A TRAINED ONE.");
        ROS_WARN("PolicyRunner: package: %s", package_.dir.c_str());
        ROS_WARN("PolicyRunner: backend: %s", opt.backend.c_str());
        ROS_WARN("PolicyRunner: ======================================================");
    }

    // ---- environment ------------------------------------------------------
    try
    {
        env_ = std::make_unique<rl::PolicyEnvironment>(deploy, opt.robot_joint_names);
    }
    catch (const std::exception& e)
    {
        env_.reset();
        reason_ = std::string("POLICY_NOT_CONFIGURED (invalid ") + package_.deploy_yaml + ": " +
                  e.what() + ")";
        ROS_ERROR("PolicyRunner: %s", reason_.c_str());
        return true;
    }

    // ---- model ------------------------------------------------------------
    std::string model_error;
    auto model = rl::createOrtRunner(package_.model, model_error);
    if (!model)
    {
        env_.reset();
        reason_ = "POLICY_NOT_CONFIGURED (" + model_error + ")";
        ROS_ERROR("PolicyRunner: %s", reason_.c_str());
        return true;
    }
    try
    {
        env_->setModel(std::move(model));
    }
    catch (const std::exception& e)
    {
        env_.reset();
        reason_ = std::string("POLICY_NOT_CONFIGURED (model does not match ") +
                  package_.deploy_yaml + ": " + e.what() + ")";
        ROS_ERROR("PolicyRunner: %s", reason_.c_str());
        return true;
    }

    env_->reset();
    status_ = PolicyStatus::Ready;
    reason_.clear();
    ROS_INFO_STREAM("\n" << describe());
    return true;
}

void PolicyRunner::resetEnvironment()
{
    if (env_) env_->reset();
    steps_ = 0;
    mean_period_ = 0.0;
    failed_ = false;
    {
        std::lock_guard<std::mutex> lk(failure_mutex_);
        failure_.clear();
    }
    std::lock_guard<std::mutex> lk(target_mutex_);
    target_ = Target{};
}

bool PolicyRunner::start()
{
    if (running_) return true;
    if (!ready())
    {
        ROS_ERROR("PolicyRunner::start() refused: %s", reason_.c_str());
        return false;
    }
    if (!provider_)
    {
        ROS_ERROR("PolicyRunner::start() refused: no state provider installed");
        return false;
    }
    running_ = true;
    thread_ = std::thread(&PolicyRunner::threadBody_, this);
    return true;
}

void PolicyRunner::stop()
{
    running_ = false;
    if (thread_.joinable()) thread_.join();
}

void PolicyRunner::threadBody_()
{
    // Paced with sleep_until on a fixed grid, like Unitree's policy thread.
    const auto period = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(env_->step_dt));
    auto next = Clock::now() + period;
    const auto t_start = Clock::now();

    RobotState state;
    VelocityCommand cmd;
    RobotCommand out;

    while (running_)
    {
        provider_(state, cmd);

        try
        {
            env_->step(state, cmd, out);
        }
        catch (const std::exception& e)
        {
            // Never publish after a failure: State_RLBase sees the Target go
            // stale and bails. Publishing a half-built command instead would
            // be exactly the failure mode the action timeout exists to catch.
            fail_(e.what());
            return;
        }

        const auto now = Clock::now();

        JointArray raw_action{};
        const auto& raw = env_->action_manager->action();
        std::copy(raw.begin(), raw.end(), raw_action.begin());

        float observation_min = std::numeric_limits<float>::infinity();
        float observation_max = -std::numeric_limits<float>::infinity();
        bool observation_finite = true;
        for (const auto& group : env_->observation_manager->groups())
        {
            const auto& values = env_->observation_manager->flat(group.name);
            for (float value : values)
            {
                if (!std::isfinite(value)) observation_finite = false;
                observation_min = std::min(observation_min, value);
                observation_max = std::max(observation_max, value);
            }
        }
        std::vector<float> last_frame;
        if (!env_->observation_manager->groups().empty())
            last_frame = env_->observation_manager->lastFrame(
                env_->observation_manager->groups().front().name);
        {
            std::lock_guard<std::mutex> lk(target_mutex_);
            target_.cmd             = out;
            target_.raw_action      = raw_action;
            target_.observation_min = observation_min;
            target_.observation_max = observation_max;
            target_.last_frame      = std::move(last_frame);
            target_.observation_finite = observation_finite;
            target_.stamp           = now;
            target_.seq            += 1;
            target_.valid           = true;
        }

        const auto n = steps_.fetch_add(1) + 1;
        mean_period_ = std::chrono::duration<double>(now - t_start).count() /
                       static_cast<double>(n);

        if (now > next)
        {
            // Overran. Resynchronize rather than trying to catch up, so a
            // hiccup cannot turn into a burst of back-to-back inferences.
            next = now + period;
        }
        else
        {
            std::this_thread::sleep_until(next);
            next += period;
        }
    }
}

void PolicyRunner::fail_(const std::string& why)
{
    {
        std::lock_guard<std::mutex> lk(failure_mutex_);
        failure_ = why;
    }
    failed_  = true;
    running_ = false;
    ROS_ERROR("PolicyRunner: policy thread ABORTED: %s. No further commands will be "
              "published; State_RLBase will time out and leave RL.", why.c_str());
}

PolicyRunner::Target PolicyRunner::latest() const
{
    std::lock_guard<std::mutex> lk(target_mutex_);
    return target_;
}

std::string PolicyRunner::failureReason() const
{
    std::lock_guard<std::mutex> lk(failure_mutex_);
    return failure_;
}

double PolicyRunner::measuredPeriod() const { return mean_period_; }

std::string PolicyRunner::describe() const
{
    std::ostringstream os;
    os << "PolicyRunner: status=" << to_string(status_) << "\n";
    if (!reason_.empty()) os << "  reason : " << reason_ << "\n";
    if (package_.valid)
    {
        os << "  package: " << package_.dir << "  (version '" << package_.version << "')\n"
           << "  config : " << package_.deploy_yaml << "\n"
           << "  model  : " << package_.model << "\n";
    }
    if (env_) os << env_->describe();
    else      os << "  environment: NONE\n";
    return os.str();
}

} // namespace mini_pi
