// RobotArticulation -- the adapter between the shared RobotState and the
// generic isaaclab::ArticulationData.
//
// Counterpart of Unitree's BaseArticulation (unitree_articulation.h), which
// fills ArticulationData from a DDS LowState. This one fills it from the
// normalized RobotState (already through JointMapper, identical for the
// sim and the real backend) plus the operator's VelocityCommand:
//
//   motor coordinate  <-JointMapper->  robot coordinate  <-this->  policy coordinate
//
// It applies the policy joint order (data.joint_ids_map) and the calibration
// shift (data.joint_pos_shift) that the env resolved from deploy.yaml, reports
// which sources are available, and maps processed targets back to robot order.
#pragma once

#include "common/Types.h"
#include "isaaclab/assets/articulation/articulation.h"

#include <array>
#include <functional>
#include <limits>
#include <vector>

namespace deploy
{

class RobotArticulation : public isaaclab::Articulation
{
public:
    /// Supplies one consistent state + command sample (the FSM's RobotState
    /// under its mutex, the InputManager command). Without it, update()
    /// re-reads `state` / `command` as they are.
    using Source = std::function<void(RobotState&, VelocityCommand&)>;

    /// mapping.yaml joint_names and joint_offset (zeros in Identity mode).
    using isaaclab::Articulation::Articulation;

    void update() override
    {
        if (source) source(state, command);
        fill(state, command, data);
    }

    /// RobotState + VelocityCommand -> `data`, through data.joint_ids_map and
    /// data.joint_pos_shift (an empty map fills only the base quantities and
    /// the availability flags).
    static void fill(const RobotState& state, const VelocityCommand& cmd,
                     isaaclab::ArticulationData& data)
    {
        const std::size_t n = data.joint_ids_map.size();
        data.joint_pos.resize(n);
        data.joint_vel.resize(n);
        for (std::size_t i = 0; i < n; ++i)
        {
            const auto r = static_cast<std::size_t>(data.joint_ids_map[i]);
            data.joint_pos[i] = static_cast<float>(state.robot_q[r] + data.joint_pos_shift[i]);
            data.joint_vel[i] = static_cast<float>(state.robot_dq[r]);
        }
        // Invalid sources become NaN, never a plausible 0: the availability
        // checks and the finiteness check keep them away from the model.
        constexpr float nan = std::numeric_limits<float>::quiet_NaN();
        std::array<double, 3> g{};
        const bool g_ok = projected_gravity_xyzw(state.orientation, g);
        for (std::size_t k = 0; k < 3; ++k)
        {
            data.root_ang_vel_b[k] = static_cast<float>(state.angular_velocity[k]);
            data.root_euler[k] = static_cast<float>(state.rpy[k]);
            data.base_lin_vel[k] = state.base_lin_vel.valid
                                       ? static_cast<float>(state.base_lin_vel.value[k]) : nan;
            data.projected_gravity_b[k] = g_ok ? static_cast<float>(g[k]) : nan;
        }
        data.imu_valid = state.imu_valid;
        data.base_lin_vel_valid = state.base_lin_vel.valid;
        data.base_lin_vel_age_s = state.base_lin_vel.valid
                                      ? age_seconds(state.base_lin_vel.stamp)
                                      : std::numeric_limits<double>::infinity();
        data.command = {cmd.vx, cmd.vy, cmd.dyaw};
    }

    /// Policy-order joint targets (policy coordinates) -> robot-order command.
    void to_robot(const std::vector<float>& policy_q, JointArray& robot_q) const
    {
        for (std::size_t i = 0; i < data.joint_ids_map.size(); ++i)
            robot_q[static_cast<std::size_t>(data.joint_ids_map[i])] =
                static_cast<double>(policy_q[i]) - data.joint_pos_shift[i];
    }

    Source source;
    RobotState state;          ///< the last sample update() used
    VelocityCommand command;   ///< the last command update() used
};

} // namespace deploy
