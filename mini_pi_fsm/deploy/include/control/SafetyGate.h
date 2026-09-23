// SafetyGate -- the single exit from the FSM to the low-level bus, and the
// owner of the runtime fault latch.
//
//     FSM (Passive / FixStand / Velocity)
//            │  RobotCommand
//            ▼
//       SafetyGate.publish()
//            ├── HEALTHY  → clamp, check, JointMapper, hardware->write()
//            └── FAULTED  → the FSM command is DISCARDED and a damping-only
//                           safe override is written instead
//            ▼
//          motors
//
// ====================== WHY THIS IS NOT AN FSM STATE ========================
// This replaces State_Fault. The response it performs is byte-for-byte the one
// State_Fault performed -- damping hold recomputed from the measured pose every
// cycle, PolicyRunner stopped on entry, optional firmware stop -- but "faulted"
// is now a SYSTEM CONDITION orthogonal to the FSM, not a robot operating mode.
// Consequences:
//   * the FSM only ever reports Passive / FixStand / Velocity;
//   * the override happens BELOW the FSM, so no state, present or future, can
//     route around it;
//   * a fault no longer needs a state id, a transition table entry, or a
//     "Fault -> Passive" edge in fsm.yaml.
//
// Unitree's equivalent is the unconditional `lowstate->isTimeout() -> Passive`
// guard registered on every state (deploy/include/FSM/FSMState.h:48-53): the
// reaction to a runtime fault is to reach the safe operating mode, not to enter
// a dedicated fault mode. Mini-Pi does both -- it drives the FSM to Passive AND
// latches the output override -- because unlike Unitree's DDS link, the
// HighTorque bus gives no independent guarantee that Passive's own command will
// be transmitted correctly once feedback is untrustworthy.
//
// ============================== LATCHING ====================================
// The latch is preserved from State_Fault, deliberately. A fault that clears
// itself (a motor packet arrives again, the robot is set back upright) must NOT
// silently restore commanding -- least of all RL, whose last target may be
// exactly what went wrong. Recovery requires the operator `reset` input
// (RB + Y), exactly as `Fault -> Passive: reset` used to, and the FSM is
// separately driven to Passive while the latch is held, so what resumes after a
// reset is Passive, never the previous RL action.
// ===========================================================================
#pragma once

#include "control/SafetyManager.h"
#include "control/Types.h"

#include <atomic>
#include <mutex>
#include <string>

namespace mini_pi
{

struct ControlContext;

class SafetyGate
{
public:
    struct Config
    {
        /// Damping used by the safe override. Default 1.0 reproduces
        /// hightorque::HtdwMotor::protectMotor() (kp = 0, kd = 1), the SDK's
        /// own reflex. Same default and same meaning State_Fault had.
        JointArray kd{};
        /// false => damping hold. true => the firmware motor stop
        /// (hardware->stop()), which makes a standing robot fall. Same switch
        /// as fsm.yaml `Fault.hard_stop`, with the same default of false.
        bool hard_stop = false;

        Config() { kd.fill(1.0); }
    };

    explicit SafetyGate(const Config& cfg = Config()) : cfg_(cfg) {}

    /// Loads the `fault_response:` block of safety.yaml. Missing keys keep the
    /// defaults. Returns false only if a present key is malformed.
    bool loadFromYaml(const std::string& path);

    const Config& config() const { return cfg_; }

    // --- lifecycle ---------------------------------------------------------

    /// Called exactly once, by main(), after startup readiness has succeeded.
    /// Before this the gate is DISARMED: raise() is ignored and publish()
    /// refuses to transmit. That is what keeps "no motor packet yet" at startup
    /// from being recorded as a runtime fault.
    void arm();
    bool armed() const { return armed_.load(std::memory_order_acquire); }

    // --- fault state -------------------------------------------------------

    /// Latch a critical fault. The FIRST report wins, so the diagnostic names
    /// the original cause rather than whatever it cascaded into. A no-op while
    /// disarmed, and a no-op for an ok() report.
    void raise(const SafetyReport& rep);

    bool faulted() const { return faulted_.load(std::memory_order_acquire); }
    /// The latched report, or a default (ok()) one when healthy.
    SafetyReport fault() const;
    /// Seconds since the latch closed; 0 while healthy.
    double faultAge() const;

    /// Explicit operator recovery -- the ONLY way the latch opens. Called by
    /// CtrlFSM when InputManager reports the `reset` input, which is the same
    /// trigger the old `Fault -> Passive: reset` transition used.
    /// Returns true if a latch was actually cleared.
    bool clear();

    // --- the one exit to hardware -----------------------------------------

    /// Validates and transmits `cmd`, or transmits the safe override instead.
    ///
    /// Returns true when the FSM's own command reached the hardware. Every
    /// other outcome (disarmed, faulted, invalid, rejected by SafetyManager)
    /// returns false, and in every armed case a damping-only command is
    /// written so the bus never simply goes quiet.
    bool publish(RobotCommand& cmd, ControlContext& ctx);

private:
    /// kp = 0, per-joint kd, target = measured pose, dq = tau = 0. The same
    /// shape FSMState::holdDamping() produces, recomputed every cycle so it can
    /// never go stale.
    void writeSafeOverride(ControlContext& ctx);

    Config cfg_;

    std::atomic<bool> armed_{false};
    std::atomic<bool> faulted_{false};

    mutable std::mutex mutex_;   ///< guards report_ / since_
    SafetyReport report_;
    TimePoint since_{};

    /// hard_stop / PolicyRunner::stop() are one-shot actions on the edge into
    /// the faulted condition, not per-cycle ones.
    bool reacted_ = false;
};

} // namespace mini_pi
