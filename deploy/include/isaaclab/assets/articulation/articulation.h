// Articulation -- the policy's view of the robot.
//
// Counterpart of unitree_rl_mjlab/deploy/include/isaaclab/assets/articulation/
// articulation.h. As there, this header holds generic data and an abstract
// update(); each robot package derives the class that fills `data` from its own
// state (Unitree: BaseArticulation in unitree_articulation.h). Nothing here
// knows a robot, a middleware or a transport.
//
// Everything in `data` is in POLICY joint order and POLICY coordinates:
// data.joint_ids_map[i] is the robot joint index of policy joint i, resolved by
// name from deploy.yaml `joint_names` against `robot_joint_names`, and
//     q_policy = q_robot + data.joint_pos_shift[i]
// The shift is 0 unless the package declares the motor->joint calibration its
// joint coordinate was defined with (deploy.yaml `calibration.joint_offset`).
// Then
//     shift = robot_joint_offsets - calibration.joint_offset
// computed from the robot's LIVE offsets, so recalibrating the robot never
// requires editing a policy package.
#pragma once

#include <array>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace isaaclab
{

/// Operator velocity command, unclamped (Unitree: the joystick). The
/// velocity_commands term applies the policy's own ranges.
struct BaseVelocityCommand
{
    double vx = 0.0;   ///< [m/s]
    double vy = 0.0;   ///< [m/s]
    double wz = 0.0;   ///< [rad/s]
};

struct ArticulationData
{
    std::vector<int> joint_ids_map;         ///< policy joint -> robot joint index
    std::vector<double> joint_pos_shift;    ///< q_policy - q_robot (see above)

    std::vector<float> joint_pos;           ///< measured q
    std::vector<float> joint_vel;           ///< measured dq
    std::vector<float> default_joint_pos;   ///< deploy.yaml default_joint_pos
    std::vector<float> joint_stiffness;     ///< deploy.yaml stiffness
    std::vector<float> joint_damping;       ///< deploy.yaml damping

    std::array<float, 3> root_ang_vel_b{};       ///< body-frame gyro [rad/s]
    std::array<float, 3> root_euler{};           ///< roll, pitch, yaw [rad]
    std::array<float, 3> base_lin_vel{};         ///< base frame [m/s]; NaN when invalid
    std::array<float, 3> projected_gravity_b{};  ///< R_wb^T [0,0,-1]; NaN when invalid

    // Source availability, judged by the robot's articulation at update().
    // Observation-term checks read these; invalid values are also NaN above.
    bool imu_valid = false;
    bool base_lin_vel_valid = false;
    double base_lin_vel_age_s = std::numeric_limits<double>::infinity();

    BaseVelocityCommand command{};
};

class Articulation
{
public:
    /// `robot_joint_names`: the robot's joint order, which joint_ids_map
    /// indexes. `robot_joint_offsets`: the robot's own motor->joint offsets in
    /// that order; only needed by packages that declare `calibration`.
    explicit Articulation(std::vector<std::string> robot_joint_names,
                          std::vector<double> robot_joint_offsets = {})
        : robot_joint_names(std::move(robot_joint_names)),
          robot_joint_offsets(std::move(robot_joint_offsets)) {}
    virtual ~Articulation() = default;

    /// Refresh `data` from the robot: joints through joint_ids_map and
    /// joint_pos_shift, base quantities, availability and the command.
    virtual void update() = 0;

    const std::vector<std::string> robot_joint_names;
    const std::vector<double> robot_joint_offsets;
    ArticulationData data;
};

} // namespace isaaclab
