// PolicyRunner -- owns the policy package, the RL environment and the policy
// thread. The FSM's single point of contact with the RL layer.
//
// Unitree puts all of this inside State_RLBase. It lives here instead for one
// concrete reason: CtrlFSM already refuses a transition into any `type: RLBase`
// state while `ctx_->policy->ready()` is false (src/FSM/CtrlFSM.cpp). Keeping
// the readiness answer outside the state is what lets the FSM reject the
// transition BEFORE entering, instead of entering and bouncing straight out --
// and it is what makes the real-hardware lock in §33 enforceable at startup
// rather than at the moment somebody presses the button.
//
// ======================= THE REAL-HARDWARE LOCK =============================
// A policy package may declare `simulation_only: true` in its deploy.yaml.
// Such a package is REFUSED unless BOTH of these hold:
//
//   1. policy.yaml `allow_test_policy` is true, AND
//   2. the selected hardware backend is a simulation backend.
//
// The two are independent on purpose. (1) alone is a config flag somebody can
// copy onto the robot; (2) cannot be satisfied by any amount of configuration
// while livelybot_serial is the backend. Both must agree.
//
// Production packages live below config/policy/velocity/. Test fixtures remain
// separate and keep the stricter simulation-only gate described above.
// ============================================================================
//
// ============================== THREADING ===================================
// Producer: the policy thread, at `step_dt` (50 Hz for the baseline).
// Consumer: the CtrlFSM thread, at 1 kHz.
//
// The handoff is a mutex-protected snapshot of one Target. A lock-free
// exchange was considered and rejected: the critical section is a copy of
// ~5 * 12 doubles with no allocation and no syscall, i.e. tens of nanoseconds
// against a 1 ms budget, while a correct double-buffer needs acquire/release
// reasoning that is far easier to get subtly wrong. One FSM iteration always
// observes one internally consistent Target.
// ============================================================================
#pragma once

#include "control/JointMapper.h"
#include "control/Types.h"
#include "rl/PolicyEnvironment.h"
#include "rl/PolicyPackage.h"

#include <yaml-cpp/yaml.h>

#include <atomic>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace mini_pi
{

enum class PolicyStatus
{
    NotConfigured = 0,  ///< no usable policy package; RL is unavailable
    Ready               ///< package loaded, model validated, thread may run
};

const char* to_string(PolicyStatus s);

class PolicyRunner
{
public:
    /// The snapshot the policy thread publishes and State_RLBase consumes.
    struct Target
    {
        RobotCommand cmd{};      ///< robot joint order; q/kp/kd filled, dq=tau=0
        JointArray raw_action{}; ///< last raw ONNX output, policy joint order
        float observation_min = 0.0f;
        float observation_max = 0.0f;
        std::vector<float> last_frame;  ///< newest single frame of the first group
        bool observation_finite = true;
        TimePoint stamp{};       ///< when the policy thread produced it
        std::uint64_t seq = 0;
        bool valid = false;
    };

    struct Options
    {
        /// The whole fsm.yaml `FSM` node. `policy_dir` is read from
        /// FSM.<state>.policy_dir, per task §4.
        YAML::Node fsm;
        /// Directory that a relative policy_dir resolves against -- normally
        /// the installed package share directory.
        std::string base_dir;
        /// mapping.yaml joint_names, i.e. ROBOT joint order.
        std::vector<std::string> robot_joint_names;
        /// HardwareInterface::backendName() of the selected backend.
        std::string backend;
        /// Backends that count as a simulation for the lock above.
        bool backend_is_simulation = false;

        // ---- launch-file / parameter-server overrides ---------------------
        // Empty means "use the value from fsm.yaml / policy.yaml".
        std::string policy_dir_override;
        std::string version_override;
        /// OR-ed with policy.yaml `allow_test_policy`: a launch file can turn
        /// the test gate ON, never OFF. Turning it on is still not sufficient
        /// -- the backend must also be a simulation.
        bool allow_test_policy_override = false;
    };

    PolicyRunner() = default;
    ~PolicyRunner();

    PolicyRunner(const PolicyRunner&) = delete;
    PolicyRunner& operator=(const PolicyRunner&) = delete;

    /// Reads `policy_yaml`, resolves the policy package, builds the
    /// environment, loads the model and validates every dimension.
    ///
    /// Returns TRUE even when no policy is available: a missing policy is a
    /// normal, expected state of this package, not a startup failure. The
    /// caller checks ready(). It returns false only if `opt` is unusable.
    bool initialize(const std::string& policy_yaml, const Options& opt);

    PolicyStatus status() const { return status_; }
    bool ready() const { return status_ == PolicyStatus::Ready; }
    const std::string& unavailableReason() const { return reason_; }

    /// Supplies the policy thread with a consistent state + command sample.
    /// Called from the policy thread; the implementation must take whatever
    /// lock guards the FSM's RobotState.
    using StateProvider = std::function<void(RobotState&, VelocityCommand&)>;
    void setStateProvider(StateProvider fn) { provider_ = std::move(fn); }

    /// Starts the policy thread. Refuses (and logs) unless ready() and a state
    /// provider is installed. Idempotent.
    bool start();
    /// Stops and JOINS the policy thread. Safe to call when not running.
    void stop();
    bool running() const { return running_; }

    /// The newest complete Target. Cheap; safe from the FSM thread.
    Target latest() const;

    /// Set when the policy thread aborted (an inference or dimension error).
    /// The thread stops publishing; State_RLBase bails on the stale Target.
    bool failed() const { return failed_; }
    std::string failureReason() const;

    /// Zeroes the environment. Call before start(), from the FSM thread.
    void resetEnvironment();

    float stepDt() const { return env_ ? env_->step_dt : 0.0f; }
    /// Policy steps completed since the last resetEnvironment().
    std::uint64_t steps() const { return steps_; }
    /// Measured mean policy period, seconds. 0 before the second step.
    double measuredPeriod() const;
    double meanInferenceSeconds() const
    {
        return env_ ? env_->meanInferenceSeconds() : 0.0;
    }
    double maxInferenceSeconds() const
    {
        return env_ ? env_->maxInferenceSeconds() : 0.0;
    }

    const rl::PolicyPackage& package() const { return package_; }
    rl::PolicyEnvironment* environment() { return env_.get(); }

    std::string describe() const;

private:
    void threadBody_();
    void fail_(const std::string& why);

    PolicyStatus status_ = PolicyStatus::NotConfigured;
    std::string reason_ = "POLICY_NOT_CONFIGURED";

    rl::PolicyPackage package_;
    std::unique_ptr<rl::PolicyEnvironment> env_;
    StateProvider provider_;

    std::thread thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> failed_{false};
    std::atomic<std::uint64_t> steps_{0};

    mutable std::mutex target_mutex_;
    Target target_;

    mutable std::mutex failure_mutex_;
    std::string failure_;

    // Timing, written only by the policy thread, read for diagnostics.
    std::atomic<double> mean_period_{0.0};
};

} // namespace mini_pi
