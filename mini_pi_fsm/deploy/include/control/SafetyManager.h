// SafetyManager -- DETECTS unsafe conditions. It never decides what to do
// about them: that is SafetyGate's job (control/SafetyGate.h), which latches
// the fault and overrides the outgoing command. See docs/architecture.md.
#pragma once

#include "control/Types.h"

#include <string>
#include <vector>

namespace mini_pi
{

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

class SafetyManager
{
public:
    struct Config
    {
        // Every check can be disabled independently. A check whose limits are
        // UNKNOWN ships disabled rather than with an invented bound; the
        // framework stays in place either way.
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
        /// `joint_names`), NOT motor order. See config/safety.yaml.
        std::vector<double> q_lower;
        std::vector<double> q_upper;
        std::vector<double> dq_max;
        std::vector<double> tau_max;

        /// |roll| or |pitch| beyond this is treated as a fall.
        double max_tilt_rad = 1.0;
    };

    bool loadFromYaml(const std::string& path);
    void setConfig(const Config& c) { cfg_ = c; }
    const Config& config() const { return cfg_; }

    /// Checks sensor plausibility, freshness and hardware health.
    /// Independent of any command and of any policy.
    SafetyReport checkState(const RobotState& state, HardwareStatus hw) const;

    /// Checks a command before it is mapped to motor order.
    SafetyReport checkCommand(const RobotCommand& cmd) const;

    /// Clamps q/dq/torque into the configured envelope, in place.
    /// Returns the number of values that had to be clamped.
    int clampCommand(RobotCommand& cmd) const;

private:
    Config cfg_;
};

} // namespace mini_pi
