// Simulated low-level bus -- wire format.
//
// This header is the counterpart of Unitree's DDS IDL pair
// (`unitree_go::msg::dds_::LowCmd_` / `LowState_`). It is included by BOTH
// sides of the simulated bus and is the single definition of the wire layout:
//
//     deploy/src/hardware/SimHardware.cpp            (controller side)
//     simulate/robots/<robot>/src/*_bridge.cpp       (simulator side)
//
// It is shared by every robot. The only robot-specific input is the motor
// count, taken from the robot's RobotSpec.h, so both ends must be built
// against the same robot.
//
// ========================= WHAT THIS IS AND IS NOT =========================
// These packets carry LOW-LEVEL HARDWARE SEMANTICS ONLY: per-motor q / dq /
// tau / kp / kd, per-motor feedback, and the IMU. That is exactly the set of
// quantities that crosses the motor bus and the IMU serial link on the real
// robot.
//
// They are in MOTOR SPACE -- the global order of livelybot_serial::robot::
// Motors (canboard -> canport -> motor id), i.e. the same index space that
// HighTorqueHardware::write() writes into `rb_->Motors[i]`. The robot<->motor
// transform (map_index / direction / joint_offset) has ALREADY been applied by
// JointMapper before a command is put on this bus, and has NOT yet been
// applied to a state taken off it. See doc/simulation.md "Avoiding double
// mapping".
//
// Deliberately NOT transmitted: FSM state, policy observations, policy
// actions, safety verdicts, joystick intent. Those live above the hardware
// boundary and travel through the same paths they use on the real robot.
//
// ONE LABELLED EXCEPTION (v2): LowState::sim_base_lin_vel, the simulator's
// ground-truth base velocity, which SimLowLevelClient puts into
// RobotState::base_lin_vel. The real robot has no such measurement, so a
// policy that observes base_lin_vel is tested here with a PERFECT estimate.
// ===========================================================================
//
// ------------------------------- ENCODING ---------------------------------
// Fixed-size POD, native byte order, no padding. This bus is loopback-only
// between two processes on one machine (the simulator and the controller), so
// there is no endianness or ABI negotiation to do -- exactly like the shared
// memory / local DDS transport Unitree's `interface: "lo"` default sets up.
// A `static_assert` pins each struct size so an accidental layout change is a
// compile error rather than a field-shear bug at runtime.
//
// `float` is used for every joint quantity, not `double`, because that is the
// precision the real bus carries in both directions:
//   - commands: livelybot_serial `motor::pos_vel_tqe_kp_kd2(float, ...)`
//   - feedback: `motor_back_t { float position, velocity, torque; }`
// Using double here would make the simulation quieter than the hardware.
// ---------------------------------------------------------------------------
#pragma once

#include "RobotSpec.h"

#include <cstddef>
#include <cstdint>

namespace deploy
{
namespace lowlevel
{

/// 'M' 'P' 'L' 'L'. Rejects stray traffic on the port.
constexpr std::uint32_t kMagic = 0x4D504C4Cu;

/// Bump on ANY layout change. Both sides refuse a mismatch loudly instead of
/// misinterpreting fields.
constexpr std::uint16_t kVersion = 2;   // v2: + sim_base_lin_vel

/// The robot's dof count (RobotSpec.h), the same value as deploy::NUM_DOF.
/// Taken from RobotSpec.h rather than Types.h so the header stays minimal for
/// the simulator.
constexpr std::size_t kNumMotor = robot_spec::kNumDof;

/// sizeof() of a struct whose members occupy `bytes`, padded to the 8-byte
/// alignment Header imposes.
constexpr std::size_t padded8(std::size_t bytes) { return (bytes + 7) / 8 * 8; }

enum : std::uint16_t
{
    kTypeLowCmd   = 1,
    kTypeLowState = 2
};

/// Command mode. Mirrors the three things the real hardware layer can do.
enum : std::uint8_t
{
    /// Normal servo command: apply the PD law below with the payload gains.
    kModeServo = 0,
    /// HighTorqueHardware::protect() -- kp = 0, kd = 1, zero targets. Sent as
    /// its own mode rather than as a servo packet so the simulator logs it the
    /// same way the robot's damping reflex is visible in a HighTorque log.
    kModeProtect = 1,
    /// HighTorqueHardware::stop() -- firmware motor stop. All torque cut; a
    /// standing robot falls. Latched until the next servo packet.
    kModeStop = 2
};

struct Header
{
    std::uint32_t magic;
    std::uint16_t version;
    std::uint16_t type;
    /// Monotonic per-direction counter. The RECEIVING side uses it purely as a
    /// change detector to stamp freshness with its own steady_clock -- exactly
    /// how HighTorqueHardware treats `motor_back_t::time`, and for the same
    /// reason: the sender's clock is not the receiver's.
    std::uint64_t seq;
    /// Sender's wall time [s]. Diagnostics only. Never used for freshness.
    double stamp;
};
static_assert(sizeof(Header) == 24, "LowLevelBus Header layout changed");

/// Controller -> simulator. The analogue of Unitree `LowCmd_`.
struct LowCmd
{
    Header h;

    float q[kNumMotor];    ///< target position   [rad], MOTOR space
    float dq[kNumMotor];   ///< target velocity   [rad/s], MOTOR space
    float tau[kNumMotor];  ///< feed-forward torque [Nm], MOTOR space
    float kp[kNumMotor];   ///< position gain, magnitude (never signed)
    float kd[kNumMotor];   ///< velocity gain, magnitude (never signed)

    std::uint8_t mode;     ///< kModeServo / kModeProtect / kModeStop
    std::uint8_t pad[7];
};
static_assert(sizeof(LowCmd) == padded8(24 + 5 * kNumMotor * 4 + 8), "LowCmd layout changed");

/// Simulator -> controller. The analogue of Unitree `LowState_`.
///
/// Carries the IMU in the same packet, as Unitree's LowState_ does. A real
/// robot may get its IMU from a separate ROS node, but on the simulated bus the
/// simulator IS the whole robot, so one packet keeps a single timestamp domain
/// and one freshness rule.
struct LowState
{
    Header h;

    float q[kNumMotor];    ///< measured position [rad], MOTOR space
    float dq[kNumMotor];   ///< measured velocity [rad/s], MOTOR space
    float tau[kNumMotor];  ///< torque estimate   [Nm], MOTOR space

    /// IMU orientation as (x, y, z, w) -- the sensor_msgs/Imu convention, i.e.
    /// exactly what the yesense node publishes on the real robot and what
    /// RobotState::orientation expects. MuJoCo's framequat sensor is
    /// (w, x, y, z); the BRIDGE does that reordering, so that the controller
    /// side never sees a MuJoCo convention.
    float quat[4];
    float gyro[3];         ///< body-frame angular velocity [rad/s]
    float acc[3];          ///< body-frame linear acceleration [m/s^2]
    /// SIMULATION ORACLE, not hardware: MuJoCo velocimeter on the imu site,
    /// i.e. base linear velocity in the base frame [m/s]. Meaningful only
    /// when sim_base_lin_vel_valid != 0 (the scene has the sensor).
    float sim_base_lin_vel[3];

    /// Per-motor fault code, same meaning as motor_back_t::fault (0 healthy).
    /// Always 0 today; present so a future simulated motor fault can be
    /// injected without a wire change.
    std::uint8_t motor_fault[kNumMotor];
    std::uint8_t sim_base_lin_vel_valid;
    std::uint8_t pad[3];

    /// MuJoCo `d->time` [s]. Diagnostics and real-vs-sim log alignment only.
    double sim_time;
};
// header + q/dq/tau + quat/gyro/acc/sim_base_lin_vel + motor_fault + valid/pad,
// padded to 8 bytes, + sim_time.
static_assert(sizeof(LowState) ==
                  padded8(24 + 3 * kNumMotor * 4 + 13 * 4 + kNumMotor + 4) + 8,
              "LowState layout changed");

inline void fill_header(Header& h, std::uint16_t type, std::uint64_t seq, double stamp)
{
    h.magic   = kMagic;
    h.version = kVersion;
    h.type    = type;
    h.seq     = seq;
    h.stamp   = stamp;
}

/// True if the header is one of ours, the right version and the right type.
inline bool header_ok(const Header& h, std::uint16_t expect_type)
{
    return h.magic == kMagic && h.version == kVersion && h.type == expect_type;
}

} // namespace lowlevel
} // namespace deploy
