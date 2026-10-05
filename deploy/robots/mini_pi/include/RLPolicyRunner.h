// RLPolicyRunner -- runs ONE ManagerBasedRLEnv on its own thread at step_dt.
//
// Unitree runs its policy thread inside State_RLBase::enter(). Mini-Pi keeps
// that thread here, owned by main(), for three Mini-Pi reasons: the policy
// package is resolved and validated once at startup (CtrlFSM refuses to enter
// RL while it is not ready), the operator envelope handed to InputManager
// comes from the package, and Safety stops the thread on a fault. There is no
// per-policy code: every package is a ManagerBasedRLEnv built from
//     <policy_dir>/<version>/params/deploy.yaml + exported/policy.onnx
//
// What the FSM sees (State_RLBase, the Velocity state):
//
//     runner.reset(); runner.start();          on entry
//     PolicyOutput t = runner.latest();        every 1 kHz tick
//     runner.stop();                           on exit
//
// ============================== THREADING ===================================
//   policy thread   step_dt (deploy.yaml), owned here
//   FSM thread      1 kHz, reads latest()
//
//   policy   0 ms      20 ms      40 ms
//              A0        A1         A2
//   FSM      0 1 2 ... 19 20 21 ...
//            A0 A0 ... A0 A1 A1 ...           zero-order hold
//
// The FSM re-applies the newest completed PolicyOutput until a newer one is
// published; it never waits for inference. The handoff is a mutex-protected
// copy of one PolicyOutput. Staleness is judged by the consumer (State_RLBase
// action_timeout_s / first_output_timeout_s). A failed step stops the thread
// AND withdraws the last output (valid = false), so the FSM bails at once.
// A step that overruns its deadline re-anchors the grid (nextPolicyDeadline):
// never a back-to-back inference.
// ============================================================================
//
// ======================= THE REAL-HARDWARE LOCK =============================
// A package may declare `simulation_only: true` in its deploy.yaml (the test
// fixtures). It is REFUSED unless BOTH policy.yaml `allow_test_policy` is true
// AND the hardware backend is a simulation. Checked before the env is built.
// ============================================================================
#pragma once

#include "MiniPiArticulation.h"
#include "Types.h"
#include "isaaclab/envs/manager_based_rl_env.h"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace mini_pi
{

// ---- policy package on disk -------------------------------------------------

/// One resolved policy package: <dir>/params/deploy.yaml + <dir>/exported/policy.onnx.
struct PolicyPackage
{
    std::string dir;
    std::string deploy_yaml;
    std::string model;
    std::string version;      ///< the directory's own name, e.g. "v0"
    bool valid = false;
    std::string error;        ///< why resolution failed; empty when valid
};

constexpr const char* kParamsSubdir   = "params";
constexpr const char* kDeployYamlName = "deploy.yaml";
constexpr const char* kExportedSubdir = "exported";
constexpr const char* kModelName      = "policy.onnx";

/// Counterpart of Unitree's param::parser_policy_dir(), plus: an explicit
/// version may be named, a directory is a package only with BOTH files, and
/// every rejection carries a reason.
///   relative policy_dir     -> resolved against base_dir
///   policy_dir IS a package -> used directly
///   otherwise               -> the lexicographically last VALID subdirectory
///   version non-empty       -> only <policy_dir>/<version>, which must be valid
/// Never throws.
PolicyPackage resolvePolicyPackage(const std::string& policy_dir, const std::string& base_dir,
                                   const std::string& version = std::string());

// ---- what the runner produces -----------------------------------------------

/// [min, max] per axis of the velocity command the policy accepts
/// (deploy.yaml commands.base_velocity.ranges). Handed to InputManager.
struct CommandLimits
{
    std::array<double, 2> vx{{0.0, 0.0}};
    std::array<double, 2> vy{{0.0, 0.0}};
    std::array<double, 2> wz{{0.0, 0.0}};
};

/// The command limits plus deploy.yaml operator.keyboard.{linear_step, yaw_step}.
struct OperatorEnvelope
{
    CommandLimits limits;
    double key_linear_step = 0.0;
    double key_yaw_step = 0.0;
};

/// One policy step's result, and the snapshot the FSM reads with latest().
struct PolicyOutput
{
    /// ROBOT joint order; q/kp/kd filled, dq = torque = 0. Its own
    /// valid/stamp/seq are set by the FSM thread (FSMState::stampValid).
    RobotCommand cmd{};

    TimePoint stamp{};       ///< when the policy thread produced it
    std::uint64_t seq = 0;
    bool valid = false;

    // ---- diagnostics only ----------------------------------------------------
    TimePoint sample_stamp{};       ///< when the state it was built from was sampled
    JointArray action{};            ///< ActionManager::action(), policy order
    float observation_min = 0.0f;
    float observation_max = 0.0f;
    bool observation_finite = true;
};

enum class PolicyStatus
{
    NotConfigured = 0,  ///< no usable policy package; RL is unavailable
    Ready               ///< package loaded, model validated, thread may run
};

const char* to_string(PolicyStatus s);

/// Policy-thread timing, seconds.
struct PolicyTiming
{
    double last_period = 0.0;    ///< start-to-start of the last two steps
    double min_period = 0.0;
    double max_period = 0.0;
    double last_update = 0.0;    ///< duration of the last env step (obs + ONNX + action)
    double mean_update = 0.0;
    double max_update = 0.0;
    std::uint64_t overruns = 0;
};

/// on time (now <= due) -> due; overrun -> now + period (re-anchor).
inline TimePoint nextPolicyDeadline(TimePoint due, TimePoint now, Clock::duration period,
                                    bool& overran)
{
    overran = now > due;
    return overran ? now + period : due;
}

// ---- the runner -------------------------------------------------------------

class RLPolicyRunner
{
public:
    struct Options
    {
        /// The whole fsm.yaml `FSM` node; policy_dir is FSM.<state>.policy_dir.
        YAML::Node fsm;
        /// What a relative policy_dir resolves against (package share dir).
        std::string base_dir;
        /// mapping.yaml joint_names, i.e. ROBOT joint order.
        std::vector<std::string> robot_joint_names;
        /// mapping.yaml joint_offset, robot order (zeros in Identity mode):
        /// lets a package's `calibration` follow the live mapping.
        std::vector<double> robot_joint_offsets;
        std::string backend;
        bool backend_is_simulation = false;

        // Launch overrides; empty = fsm.yaml / policy.yaml value.
        std::string policy_dir_override;
        std::string version_override;
        /// Can turn the test gate ON, never OFF; a sim backend is still required.
        bool allow_test_policy_override = false;
    };

    RLPolicyRunner();
    ~RLPolicyRunner();
    RLPolicyRunner(const RLPolicyRunner&) = delete;
    RLPolicyRunner& operator=(const RLPolicyRunner&) = delete;

    /// Resolves and builds the selected package. Returns TRUE even when no
    /// policy is available (check ready()); false only if `opt` is unusable.
    bool initialize(const std::string& policy_yaml, const Options& opt);

    /// Builds the env of one package on a MiniPiArticulation and attaches its
    /// model: `alg` when given, else ONNX Runtime on `model_path`. nullptr +
    /// `error` on any configuration or model-contract problem.
    static std::unique_ptr<isaaclab::ManagerBasedRLEnv> buildEnv(
        const YAML::Node& deploy, const std::string& deploy_path, const std::string& model_path,
        const std::vector<std::string>& robot_joint_names, std::string& error,
        std::unique_ptr<isaaclab::Algorithms> alg = nullptr,
        const std::vector<double>& robot_joint_offsets = {});

    PolicyStatus status() const { return status_; }
    bool ready() const { return status_ == PolicyStatus::Ready; }
    const std::string& unavailableReason() const { return reason_; }

    /// Installed as the MiniPiArticulation::Source: called from the policy
    /// thread; must take whatever lock guards the FSM's RobotState.
    using StateProvider = MiniPiArticulation::Source;
    void setStateProvider(StateProvider fn);

    /// Why RL must not be entered on `state` (a configured observation has no
    /// source), empty when it may. Pure; any thread.
    std::string entryBlocker(const RobotState& state) const;

    /// Refuses unless ready() and a state provider is installed. Idempotent.
    bool start();
    /// Stops and joins the thread. Safe when not running.
    void stop();
    bool running() const { return running_; }

    PolicyOutput latest() const;
    /// Newest frame of the first observation group (status log; kept out of
    /// PolicyOutput so the 1 kHz latest() copy does not allocate).
    std::vector<float> latestFrame() const;
    bool failed() const { return failed_; }
    std::string failureReason() const;

    /// Back to a fresh env (history, last action, gait clock) and no output.
    /// Call before start() and after stop(), from the FSM thread.
    void reset();

    float stepDt() const;
    std::uint64_t steps() const { return steps_; }
    double measuredPeriod() const { return mean_period_; }
    PolicyTiming timing() const;

    const PolicyPackage& package() const { return package_; }
    const OperatorEnvelope& operatorEnvelope() const { return envelope_; }
    /// The newest frame sliced into named terms (status log).
    std::string describeFrame(const std::vector<float>& frame) const;
    std::string describe() const;

private:
    void adopt_(std::unique_ptr<isaaclab::ManagerBasedRLEnv> env);
    void threadBody_();
    void fail_(const std::string& why);

    PolicyStatus status_ = PolicyStatus::NotConfigured;
    std::string reason_ = "POLICY_NOT_CONFIGURED";

    PolicyPackage package_;
    std::unique_ptr<isaaclab::ManagerBasedRLEnv> env_;
    MiniPiArticulation* robot_ = nullptr;   ///< env_->robot, as built by buildEnv()
    JointArray kp_{}, kd_{};   ///< robot order, from the env's gains
    OperatorEnvelope envelope_;

    std::thread thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> failed_{false};
    std::atomic<std::uint64_t> steps_{0};

    mutable std::mutex output_mutex_;
    PolicyOutput output_;
    std::vector<float> last_frame_;

    mutable std::mutex failure_mutex_;
    std::string failure_;

    std::atomic<double> mean_period_{0.0};
    mutable std::mutex timing_mutex_;
    PolicyTiming timing_;
};

} // namespace mini_pi
