// Adapted from unitree_rl_mjlab/deploy/include/FSM/CtrlFSM.h.
//
// Kept: construction from a YAML `_` block listing the enabled states with
// id/type, duplicate-id rejection, the pre_run/run/post_run tick and the
// first-guard-wins transition scan.
// Replaced: unitree::common::RecurrentThread with a std::thread paced by
// std::chrono::steady_clock + sleep_until, and the control period comes from
// config rather than a hard-coded 0.001.
#pragma once

#include "FSM/BaseState.h"
#include "control/ControlContext.h"

#include <yaml-cpp/yaml.h>

#include <atomic>
#include <memory>
#include <mutex>
#include <set>
#include <thread>
#include <vector>

namespace mini_pi
{

class CtrlFSM
{
public:
    CtrlFSM(const YAML::Node& cfg, ControlContext* ctx, double control_dt);
    ~CtrlFSM();

    CtrlFSM(const CtrlFSM&) = delete;
    CtrlFSM& operator=(const CtrlFSM&) = delete;

    void add(const std::shared_ptr<BaseState>& state);

    /// Starts the control thread in `initial_state`, which must be one of the
    /// enabled states. Unitree does the same thing implicitly with
    /// `currentState = states[0]` plus the comment "Start From State_Passive"
    /// (CtrlFSM.h:48-57); naming it removes the dependence on YAML ordering.
    ///
    /// Readiness is NOT checked here. The system is expected to be ready
    /// already -- see control/SystemReadiness.h and main().
    void start(const std::string& initial_state = "Passive");
    void stop();

    /// One control cycle: refresh state, run the current state, evaluate the
    /// transition guards. The control thread calls this in a loop; it is public
    /// so that a test can drive the FSM deterministically, with no thread and
    /// no ROS master. Not for production callers.
    void step() { tick_(); }

    std::string currentStateName() const;

    /// Real achieved period of the last cycle, for the debug dump.
    double lastCycleSeconds() const { return last_cycle_s_; }
    /// Number of cycles that overran the configured period.
    std::uint64_t overruns() const { return overruns_; }

    /// Consistent copy of the state read and the command published by the
    /// most recent completed tick. For diagnostics on another thread: the live
    /// ctx->state / ctx->command are rewritten by the FSM thread every cycle
    /// and must not be read concurrently.
    /// Returns false (and leaves the outputs alone) before the first tick.
    bool snapshot(RobotState& state, RobotCommand& command) const
    {
        std::lock_guard<std::mutex> lk(snap_mutex_);
        if (!snap_valid_) return false;
        state = snap_state_;
        command = snap_command_;
        return true;
    }

    std::vector<std::shared_ptr<BaseState>> states;

private:
    void run_();
    void tick_();
    /// Read hardware, map to robot order, run the state-side safety checks.
    void refreshState_();
    /// Operator recovery from a latched runtime fault (the `reset` input).
    /// This is what the removed `Fault -> Passive: reset` transition used to do.
    void serviceFaultReset_();

    ControlContext* ctx_ = nullptr;
    double dt_ = 0.001;

    std::shared_ptr<BaseState> currentState_;
    std::thread thread_;
    std::atomic<bool> running_{false};

    double last_cycle_s_ = 0.0;
    std::uint64_t overruns_ = 0;

    mutable std::mutex snap_mutex_;
    RobotState snap_state_{};
    RobotCommand snap_command_{};
    bool snap_valid_ = false;

    /// FSM ids whose type is RLBase. A transition into one of these is
    /// rejected while PolicyRunner is not ready.
    std::set<int> rl_state_ids_;
};

} // namespace mini_pi
