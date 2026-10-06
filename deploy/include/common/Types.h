// Shared deploy framework -- SDK-independent runtime types.
//
// Structural reference: unitree_rl_mjlab/deploy/robots/g1/include/Types.h
// (which aliases Unitree DDS LowCmd_t/LowState_t). This framework deliberately
// does NOT alias an SDK type here: vendor SDKs expose Eigen-based and
// ROS-based structures that must not leak into the FSM, so this header
// defines the plain-C++ exchange types instead.
//
// The only robot-specific input is the dof count, taken from the robot's own
// RobotSpec.h (deploy/robots/<robot>/include).
#pragma once

#include "RobotSpec.h"

#include <array>
#include <chrono>
#include <cstdint>
#include <limits>
#include <cmath>
#include <cstddef>
#include <string>

namespace deploy
{

/// Actuated joints of the robot being built (RobotSpec.h).
constexpr std::size_t NUM_DOF = robot_spec::kNumDof;

using Clock     = std::chrono::steady_clock;
using TimePoint = Clock::time_point;
using JointArray = std::array<double, NUM_DOF>;

inline double age_seconds(const TimePoint& stamp, const TimePoint& now = Clock::now())
{
    if (stamp.time_since_epoch().count() == 0) return std::numeric_limits<double>::infinity();
    return std::chrono::duration<double>(now - stamp).count();
}

/// Health of the hardware transport, reported by the hardware backend.
enum class HardwareStatus
{
    Uninitialized = 0,  ///< initialize() not called or failed
    Ok,                 ///< all motors discovered and every feedback fresh
    Stale,              ///< initialized but >=1 motor's feedback exceeds the timeout
    Fault               ///< motor fault code / SDK limit flag raised
};

inline const char* to_string(HardwareStatus s)
{
    switch (s)
    {
        case HardwareStatus::Uninitialized: return "Uninitialized";
        case HardwareStatus::Ok:            return "Ok";
        case HardwareStatus::Stale:         return "Stale";
        case HardwareStatus::Fault:         return "Fault";
    }
    return "Unknown";
}

/// Base linear velocity, filled by the hardware backend like every other
/// sensor field:
///   SimHardware         the simulator's ground truth (MuJoCo velocimeter on
///                       the imu site) -- an ORACLE no real robot has;
///   real backends       nothing so far: there is no verified estimator, so `valid`
///                       stays false.
/// A consumer must not use `value` unless `valid` and fresh; the
/// `base_lin_vel` observation term enforces that. Never substitute zeros, the
/// command, or an integrated accelerometer.
struct BaseLinearVelocity
{
    /// vx, vy, vz [m/s] of the base, expressed in the BASE frame (the frame
    /// RobotState::orientation / angular_velocity use, i.e. after the IMU
    /// mount transform).
    std::array<double, 3> value{};
    /// True only while the source vouches for `value`.
    bool valid = false;
    /// When the source produced `value`. Consumers judge freshness from this.
    TimePoint stamp{};
};

/// One snapshot of everything the controller knows about the robot.
///
/// `motor_*` is in HARDWARE MOTOR ORDER (the global order of
/// the robot's motor bus, as the backend enumerates it).
/// `robot_*` is in ROBOT/URDF JOINT ORDER (config/mapping.yaml `joint_names`).
/// The permutation and sign between the two is owned by JointMapper.
struct RobotState
{
    JointArray motor_q{};
    JointArray motor_dq{};
    JointArray motor_tau{};

    JointArray robot_q{};
    JointArray robot_dq{};
    JointArray robot_tau{};

    std::array<double, 3> angular_velocity{};  ///< body-frame gyro [rad/s]
    std::array<double, 4> orientation{};       ///< quaternion, (x, y, z, w)
    std::array<double, 3> rpy{};               ///< ZYX euler, (roll, pitch, yaw)

    /// See BaseLinearVelocity: valid only on the simulator today.
    BaseLinearVelocity base_lin_vel{};

    /// The IMU exactly as the sensor reported it, BEFORE the sensor-to-base
    /// mount transform (robot.yaml `imu.mount_rpy`). Diagnostics only; the
    /// three fields above are the base-frame values every consumer uses.
    std::array<double, 3> imu_raw_angular_velocity{};
    std::array<double, 4> imu_raw_orientation{{0, 0, 0, 1}};  ///< (x, y, z, w)

    /// Per-motor fault code straight from motor_back_t::fault (0 == healthy).
    std::array<int, NUM_DOF> motor_fault{};

    // --- freshness, tracked PER MOTOR ------------------------------------
    // Successfully dereferencing a motor object proves nothing about whether
    // its feedback is current, so every motor carries its own age. The
    // timeout these are compared against comes from config/safety.yaml.
    std::array<double, NUM_DOF> motor_age{};   ///< seconds since last frame
    std::array<bool, NUM_DOF> motor_fresh{};   ///< age <= configured timeout
    std::size_t stale_motor_count = NUM_DOF;   ///< how many are not fresh

    /// True when every motor has produced at least one finite frame.
    bool motor_valid = false;
    bool imu_valid   = false;

    /// Oldest motor stamp across all 12 -- the conservative one to test.
    TimePoint motor_stamp{};
    TimePoint imu_stamp{};
    double imu_age = std::numeric_limits<double>::infinity();
};

/// One control command, always expressed in ROBOT/URDF JOINT ORDER.
/// FSM states only ever fill this; the conversion to motor order happens
/// downstream in JointMapper + the hardware backend.
struct RobotCommand
{
    JointArray q{};
    JointArray dq{};
    JointArray torque{};

    JointArray kp{};
    JointArray kd{};

    bool valid = false;

    /// Set every time a state publishes a command. Safety rejects a
    /// command older than safety.yaml `command_timeout_s`, so a state that
    /// stops refreshing cannot have a stale command reused indefinitely.
    TimePoint stamp{};
    std::uint64_t seq = 0;

    /// Clears the payload but preserves `seq`, so the monotonic counter keeps
    /// exposing whether the current state actually refreshed this cycle.
    void reset()
    {
        q.fill(0.0); dq.fill(0.0); torque.fill(0.0); kp.fill(0.0); kd.fill(0.0);
        valid = false;
        stamp = TimePoint{};
    }
};

/// Same payload as RobotCommand but in hardware motor order. Produced only by
/// JointMapper, consumed only by a HardwareInterface backend.
struct MotorCommand
{
    JointArray q{};
    JointArray dq{};
    JointArray torque{};
    JointArray kp{};
    JointArray kd{};
    bool valid = false;
};

/// Normalized operator intent, produced by InputManager.
struct VelocityCommand
{
    double vx   = 0.0;
    double vy   = 0.0;
    double dyaw = 0.0;
};

/// Quaternion (x, y, z, w) -> ZYX euler (roll, pitch, yaw).
///
/// Shared by every backend so that the real and the
/// simulated backend cannot drift apart in how they derive `RobotState::rpy`.
/// Safety's orientation check reads that field, so a difference here
/// would silently change when a fall is declared in sim versus on the robot.
/// The arithmetic is unchanged, including the clamp on the pitch term.
inline void quat_xyzw_to_rpy(const std::array<double, 4>& q, std::array<double, 3>& rpy)
{
    const double x = q[0], y = q[1], z = q[2], w = q[3];
    rpy[0] = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    const double sp = 2.0 * (w * y - z * x);
    rpy[1] = std::asin(sp < -1.0 ? -1.0 : (sp > 1.0 ? 1.0 : sp));
    rpy[2] = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

/// Hamilton product a*b, both (x, y, z, w).
inline std::array<double, 4> quat_mul_xyzw(const std::array<double, 4>& a,
                                           const std::array<double, 4>& b)
{
    return {a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
            a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
            a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
            a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2]};
}

/// ZYX euler (roll, pitch, yaw) -> quaternion (x, y, z, w); R = Rz*Ry*Rx.
/// Exact inverse of quat_xyzw_to_rpy() away from pitch = +-pi/2.
inline std::array<double, 4> rpy_to_quat_xyzw(const std::array<double, 3>& rpy)
{
    const double cr = std::cos(rpy[0] * 0.5), sr = std::sin(rpy[0] * 0.5);
    const double cp = std::cos(rpy[1] * 0.5), sp = std::sin(rpy[1] * 0.5);
    const double cy = std::cos(rpy[2] * 0.5), sy = std::sin(rpy[2] * 0.5);
    return {sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy};
}

/// Rotate v by unit quaternion q (x, y, z, w).
inline std::array<double, 3> quat_rotate_xyzw(const std::array<double, 4>& q,
                                              const std::array<double, 3>& v)
{
    const std::array<double, 4> p{v[0], v[1], v[2], 0.0};
    const std::array<double, 4> qc{-q[0], -q[1], -q[2], q[3]};
    const auto r = quat_mul_xyzw(quat_mul_xyzw(q, p), qc);
    return {r[0], r[1], r[2]};
}

/// Sensor-to-base IMU mount transform.
///
/// `mount_rpy` is the orientation of the SENSOR frame expressed in the BASE
/// frame (R_bs), as ZYX euler. With the sensor reporting its own world
/// orientation q_ws and body rate w_s:
///     q_wb = q_ws * conj(q_bs)        w_b = R_bs * w_s
/// mount_rpy = {0, 0, 0} is the identity and leaves both unchanged, which is
/// the only value that is correct until the real mounting has been MEASURED on
/// the robot (dry-run checklist). Examples: sensor upside down about x ->
/// {pi, 0, 0}; sensor rotated 90 deg about z -> {0, 0, pi/2}.
inline void apply_imu_mount(const std::array<double, 3>& mount_rpy,
                            std::array<double, 4>& q_xyzw, std::array<double, 3>& gyro)
{
    if (mount_rpy[0] == 0.0 && mount_rpy[1] == 0.0 && mount_rpy[2] == 0.0) return;
    const auto q_bs = rpy_to_quat_xyzw(mount_rpy);
    const std::array<double, 4> q_sb{-q_bs[0], -q_bs[1], -q_bs[2], q_bs[3]};
    q_xyzw = quat_mul_xyzw(q_xyzw, q_sb);
    gyro   = quat_rotate_xyzw(q_bs, gyro);
}

/// World gravity direction [0, 0, -1] expressed in the BASE frame:
///     g_b = R_wb^T * g_w
/// with q (x, y, z, w) the base orientation in the world, i.e.
/// RobotState::orientation after the IMU mount transform. This is mjlab's
/// `projected_gravity_b` (quat_apply_inverse(root_quat_w, [0, 0, -1])).
/// q and -q give the same result.
///
/// Returns false, leaving `g` untouched, when q is not finite, its norm is
/// not within `norm_tol` of 1, or the result is not a finite unit vector.
/// Not derived from Euler angles and not from the accelerometer.
inline bool projected_gravity_xyzw(const std::array<double, 4>& q, std::array<double, 3>& g,
                                   double norm_tol = 0.05)
{
    for (double v : q) { if (!std::isfinite(v)) return false; }
    const double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (!(std::fabs(n - 1.0) <= norm_tol)) return false;
    // R^T v == rotation by the conjugate.
    const std::array<double, 4> qc{-q[0] / n, -q[1] / n, -q[2] / n, q[3] / n};
    const auto r = quat_rotate_xyzw(qc, {0.0, 0.0, -1.0});
    const double rn = std::sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]);
    if (!std::isfinite(rn) || std::fabs(rn - 1.0) > 1e-6) return false;
    g = r;
    return true;
}

inline bool is_finite(const JointArray& a)
{
    for (double v : a) { if (!std::isfinite(v)) return false; }
    return true;
}

} // namespace deploy
