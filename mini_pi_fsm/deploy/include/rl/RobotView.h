// rl::RobotView -- the RL layer's view of the robot.
//
// Structural counterpart of unitree_rl_mjlab/deploy/include/isaaclab/assets/
// articulation/articulation.h (`Articulation` + `ArticulationData`), with two
// substitutions:
//
//   - Eigen types    -> std::vector<float> / std::array<float,3>, because
//                       Types.h is deliberately free of any SDK or matrix
//                       library, and the RL layer must not reintroduce one.
//   - UnitreeJoystick -> VelocityCommand, because InputManager already owns
//                       operator intent (task §22). The RL layer never
//                       subscribes to anything.
//
// ============================ JOINT ORDER ===================================
// Everything in `data` is in POLICY JOINT ORDER: the order of `joint_names` in
// the policy package's deploy.yaml.
//
// The permutation policy <-> robot is resolved BY NAME here, once, at
// construction, and is the ONLY place it exists.
//
// It is NOT composed with JointMapper's motor <-> robot permutation. A value
// crossing this class has already been through JointMapper (RobotState) or is
// about to be (RobotCommand); it is never both at once.
// ============================================================================
#pragma once

#include "control/Types.h"

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace mini_pi
{
namespace rl
{

struct RobotViewData
{
    // --- policy order -----------------------------------------------------
    std::vector<float> joint_pos;          ///< measured q
    std::vector<float> joint_vel;          ///< measured dq
    std::vector<float> default_joint_pos;  ///< the policy's default pose

    // --- base -------------------------------------------------------------
    std::array<float, 3> root_ang_vel_b{};  ///< body-frame gyro [rad/s]
    std::array<float, 3> root_euler{};      ///< roll, pitch, yaw [rad]

    /// Operator intent, straight from InputManager::velocityCommand().
    /// UNCLAMPED here: the `velocity_commands` observation term applies the
    /// policy's own ranges, so the command limit and the observation scale stay
    /// separate quantities (task §22).
    VelocityCommand command{};

    /// False until update() has been called with a state carrying fresh
    /// motor and IMU data.
    bool valid = false;
};

class RobotView
{
public:
    /// Resolves policy order against robot order by NAME.
    /// Throws std::runtime_error if a policy joint name is unknown, duplicated,
    /// or if the two lists have different lengths.
    RobotView(const std::vector<std::string>& robot_joint_names,
              const std::vector<std::string>& policy_joint_names);

    /// Copies one FSM state snapshot into `data`, permuting to policy order.
    void update(const RobotState& state, const VelocityCommand& cmd);

    /// policy index -> robot index.
    const std::vector<int>& policyToRobot() const { return policy_to_robot_; }
    const std::vector<std::string>& jointNames() const { return names_; }
    std::size_t numJoints() const { return names_.size(); }

    /// Scatters a policy-order vector into a robot-order JointArray.
    /// Robot joints not named by the policy are left untouched -- the caller
    /// decides what they should hold.
    void policyToRobotArray(const std::vector<float>& policy_values,
                            JointArray& robot_values) const;

    std::string describe() const;

    RobotViewData data;

private:
    std::vector<std::string> names_;     ///< policy order
    std::vector<int> policy_to_robot_;
};

} // namespace rl
} // namespace mini_pi
