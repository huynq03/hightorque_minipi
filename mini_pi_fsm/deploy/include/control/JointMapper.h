// JointMapper -- owns every coordinate-system change in the Mini-Pi stack.
//
// ============================ PERMUTATION OWNERSHIP ==========================
// There is exactly ONE physical permutation in this package, and it lives here.
//
// HighTorqueHardware talks to `livelybot_serial::robot::Motors` DIRECTLY
// (see src/hardware/HighTorqueHardware.cpp: `rb_->Motors[i]`), i.e. it reads and
// writes RAW HARDWARE MOTOR ORDER and applies no reordering of its own. It does
// NOT use hightorque::RobotMotorGroup / PiMotorGroup, which would have applied
// `mapIndex[i]` internally.
//
// Therefore JointMapper MUST apply `map_index`. If the hardware layer is ever
// switched to a motor group, the permutation here must be removed, or every
// joint will be mapped twice. See docs/source_mapping.md.
// =============================================================================
//
// Orders in play:
//   motor  : global index into livelybot_serial::robot::Motors
//            (canboard -> canport -> motor id).
//   robot  : URDF joint order == config/mapping.yaml `joint_names`.
//   policy : the joint order a future policy is trained in. NOT YET DEFINED --
//            see docs/architecture.md "Policy boundary".
#pragma once

#include "control/Types.h"

#include <string>
#include <vector>

namespace mini_pi
{

/// Which coordinate transform to apply between motor and robot space.
///
/// Both modes are reproduced from the original Mini-Pi controller
/// (`hightorque::common::transform*`, disassembled from
/// install/lib/sim2real_master/sim2real_master_node -- the source of those
/// functions ships only as a prebuilt .so). See docs/source_mapping.md.
enum class TransformMode
{
    /// direction + joint_offset + permutation. The general path taken by every
    /// HighTorque algorithm except `lr`.
    Full,
    /// Permutation only: no sign, no offset. The path the original controller
    /// takes when `algorithm == "lr"`, in BOTH directions.
    Identity
};

const char* to_string(TransformMode m);

class JointMapper
{
public:
    struct Config
    {
        std::vector<std::string> joint_names;   ///< robot order
        std::vector<int>    map_index;          ///< robot index -> motor index
        std::vector<int>    direction;          ///< +1/-1, per ROBOT joint index
        std::vector<double> joint_offset;       ///< per ROBOT joint index [rad]
        TransformMode mode = TransformMode::Full;
    };

    /// Load and validate from a YAML file. Returns false (and logs why) if the
    /// file is missing, a vector has the wrong length, map_index is not a
    /// permutation of [0, MINI_PI_DOF), or a direction entry is not +/-1.
    bool loadFromYaml(const std::string& path);

    bool valid() const { return valid_; }
    TransformMode mode() const { return cfg_.mode; }

    // --- state direction: motor -> robot ---------------------------------
    //
    // Full:
    //   robot_q[j]   = direction[j] * motor_q[map_index[j]] - joint_offset[j]
    //   robot_dq[j]  = direction[j] * motor_dq[map_index[j]]
    //   robot_tau[j] =                motor_tau[map_index[j]]   (NO sign)
    // Identity:
    //   robot_*[j]   = motor_*[map_index[j]]
    void motorToRobot(const JointArray& motor_q,
                      const JointArray& motor_dq,
                      const JointArray& motor_tau,
                      JointArray& robot_q,
                      JointArray& robot_dq,
                      JointArray& robot_tau) const;

    // --- command direction: robot -> motor -------------------------------
    //
    // Full:
    //   motor_q[map_index[j]]   = direction[j] * (robot_q[j] + joint_offset[j])
    //   motor_dq[map_index[j]]  = robot_dq[j]      (NO sign -- matches original)
    //   motor_tau[map_index[j]] = robot_tau[j]     (NO sign -- matches original)
    // Identity:
    //   motor_*[map_index[j]]   = robot_*[j]
    //
    // kp/kd are magnitudes and are only permuted, never signed or offset.
    void robotToMotor(const RobotCommand& in, MotorCommand& out) const;

    const std::vector<std::string>& jointNames() const { return cfg_.joint_names; }
    const std::vector<int>& mapIndex() const { return cfg_.map_index; }
    const std::vector<int>& direction() const { return cfg_.direction; }
    const JointArray& jointOffset() const { return joint_offset_; }

    /// Human readable dump of the full mapping, for the pre-motion debug log.
    std::string describe() const;

private:
    Config cfg_;
    JointArray joint_offset_{};
    bool valid_ = false;
};

} // namespace mini_pi
