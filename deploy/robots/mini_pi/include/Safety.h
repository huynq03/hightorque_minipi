// Safety -- every runtime safety rule, and the single exit from the FSM to the
// hardware.
//
//     HardwareInterface::read -> JointMapper -> RobotState
//                                                  |
//                              Safety::checkState()   (every tick, CtrlFSM)
//                                                  |  a fault LATCHES
//                                                  v
//                                    FSM state builds a RobotCommand
//                                                  |
//                              Safety::publish()      (every tick, post_run)
//            HEALTHY  -> clampCommand, checkCommand, JointMapper, hardware->write()
//            FAULTED  -> the FSM command is DISCARDED; a damping-only override
//                        (kp = 0, kd = fault_response.kd, q = measured) is
//                        written instead
//
// The two halves stay separate methods because they answer different
// questions: checkState() judges what the robot IS doing (sensor freshness,
// finiteness, hardware health, tilt); checkCommand()/clampCommand() judge what
// the FSM WANTS it to do. Neither depends on the other, and neither depends on
// which FSM state or which policy produced the command.
//
// ========================= WHY THIS IS NOT AN FSM STATE =====================
// "Faulted" is a SYSTEM CONDITION orthogonal to the FSM, not a robot operating
// mode:
//   * the FSM only ever reports Passive / FixStand / Velocity;
//   * the override happens BELOW the FSM, in publish(), so no state -- present
//     or future -- can route around it;
//   * a latched fault also drives the FSM to Passive (FSMState registers that
//     guard for every state), so what resumes after a reset is Passive, never
//     the previous RL action.
// Unitree's equivalent is the unconditional `lowstate->isTimeout() -> Passive`
// guard on every state (deploy/robots/mini_pi/include/FSMState.h:48-53). Mini-Pi does
// that AND overrides the output, because unlike Unitree's DDS link the
// HighTorque bus gives no independent guarantee that Passive's own command
// will be transmitted correctly once feedback is untrustworthy.
//
// ============================== ARMING / LATCHING ===========================
// Safety starts DISARMED. Until main() calls arm() (after startup readiness
// succeeded, see FSM/SystemReadiness.h) raise() is ignored and publish()
// transmits nothing -- "no motor packet yet" at startup is not a fault.
// Once armed, the FIRST fault wins and stays latched until the operator
// `reset` input (RB + Y / key 0) calls clear(). A fault that clears itself
// (a motor packet arrives again, the robot is set upright) must NOT silently
// restore commanding.
//
// Checks whose limits are UNKNOWN ship disabled rather than with an invented
// bound; see config/safety.yaml.
// ============================================================================
#pragma once

#include "Types.h"

#include <atomic>
#include <mutex>
#include <string>
#include <vector>

namespace mini_pi
{

struct ControlContext;

/// Exact fault reason. The string form is the stable token used in logs and
/// diagnostics; see to_string().
enum class SafetyFault
{
    None = 0,
    MotorTimeout,        ///< MOTOR_TIMEOUT
    ImuTimeout,          ///< IMU_TIMEOUT
    NanState,            ///< NAN_STATE
    NanCommand,          ///< NAN_COMMAND
    JointPositionLimit,  ///< JOINT_POSITION_LIMIT
    JointVelocityLimit,  ///< JOINT_VELOCITY_LIMIT
    JointTorqueLimit,    ///< JOINT_TORQUE_LIMIT
    CommandTimeout,      ///< COMMAND_TIMEOUT
    CommandInvalid,      ///< COMMAND_INVALID
    HardwareFault,       ///< HARDWARE_FAULT
    BadOrientation       ///< BAD_ORIENTATION
};

const char* to_string(SafetyFault f);

struct SafetyReport
{
    SafetyFault fault = SafetyFault::None;
    int joint = -1;              ///< offending robot-order joint, -1 if N/A
    double value = 0.0;
    std::string detail;

    bool ok() const { return fault == SafetyFault::None; }
};

class Safety
{
public:
    struct Config
    {
        // Every check can be disabled independently.
        bool enable_motor_timeout  = true;
        bool enable_imu_timeout    = true;
        bool enable_finite_check   = true;
        bool enable_position_limit = true;
        bool enable_velocity_limit = false;
        bool enable_torque_limit   = false;
        bool enable_command_timeout = true;
        bool enable_orientation    = true;

        double motor_timeout_s   = 0.1;
        double imu_timeout_s     = 0.2;
        double command_timeout_s = 0.05;

        /// All limit arrays are in ROBOT JOINT ORDER (mapping.yaml
        /// `joint_names`), NOT motor order.
        std::vector<double> q_lower;
        std::vector<double> q_upper;
        std::vector<double> dq_max;
        std::vector<double> tau_max;

        /// |roll| or |pitch| beyond this is treated as a fall.
        double max_tilt_rad = 1.0;

        /// safety.yaml `fault_response:` -- what publish() does while faulted.
        struct FaultResponse
        {
            /// Damping of the override. Default 1.0 reproduces
            /// hightorque::HtdwMotor::protectMotor() (kp = 0, kd = 1), the
            /// SDK's own reflex.
            JointArray kd{};
            /// false => damping hold. true => additionally the firmware motor
            /// stop (hardware->stop()), which makes a standing robot fall.
            bool hard_stop = false;

            FaultResponse() { kd.fill(1.0); }
        } fault_response;
    };

    /// Loads safety.yaml: the checks from its top level, the response from
    /// its `fault_response:` block. Missing keys keep the defaults; a missing
    /// or malformed limit vector DISABLES that check rather than inventing a
    /// bound. Returns false on an unreadable file, q_lower >= q_upper, or a
    /// malformed fault_response.kd.
    bool loadFromYaml(const std::string& path);
    void setConfig(const Config& c) { cfg_ = c; }
    const Config& config() const { return cfg_; }

    // --- state side --------------------------------------------------------

    /// Sensor plausibility, freshness and hardware health. Independent of any
    /// command and of any policy. Pure: raising is the caller's decision.
    SafetyReport checkState(const RobotState& state, HardwareStatus hw) const;

    // --- command side ------------------------------------------------------

    /// Checks a command before it is mapped to motor order.
    SafetyReport checkCommand(const RobotCommand& cmd) const;

    /// Clamps q/dq/torque into the configured envelope, in place.
    /// Returns the number of values that had to be clamped. `mask`, when
    /// given, receives which ones (see lastClampMask()).
    int clampCommand(RobotCommand& cmd, std::uint64_t* mask = nullptr) const;

    /// Which values the most recent publish() had to clamp, robot joint order:
    /// bit j = q[j], bit 12 + j = dq[j], bit 24 + j = torque[j]. 0 when nothing
    /// saturated. Any thread; diagnostics only.
    std::uint64_t lastClampMask() const { return last_clamp_mask_.load(std::memory_order_relaxed); }
    /// publish() calls that clamped at least one value, since construction.
    std::uint64_t clampedCommands() const { return clamped_commands_.load(std::memory_order_relaxed); }

    // --- lifecycle / latch -------------------------------------------------

    /// Called exactly once, by main(), after startup readiness has succeeded.
    void arm();
    bool armed() const { return armed_.load(std::memory_order_acquire); }

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
    /// CtrlFSM on the `reset` input. Returns true if a latch was cleared.
    bool clear();

    // --- the one exit to hardware -----------------------------------------

    /// Validates and transmits `cmd`, or transmits the safe override instead.
    ///
    /// Returns true when the FSM's own command reached the hardware. Every
    /// other outcome (disarmed, faulted, invalid, rejected by checkCommand)
    /// returns false, and in every armed case a damping-only command is
    /// written so the bus never simply goes quiet.
    bool publish(RobotCommand& cmd, ControlContext& ctx);

private:
    /// kp = 0, per-joint kd, target = measured pose, dq = tau = 0. Recomputed
    /// every cycle so it can never go stale.
    void writeSafeOverride(ControlContext& ctx);

    Config cfg_;

    std::atomic<bool> armed_{false};
    std::atomic<bool> faulted_{false};
    std::atomic<std::uint64_t> last_clamp_mask_{0};
    std::atomic<std::uint64_t> clamped_commands_{0};

    mutable std::mutex mutex_;   ///< guards report_ / since_
    SafetyReport report_;
    TimePoint since_{};

    /// hard_stop / RLPolicyRunner::stop() are one-shot actions on the edge into the
    /// faulted condition, not per-cycle ones.
    bool reacted_ = false;
};

} // namespace mini_pi
