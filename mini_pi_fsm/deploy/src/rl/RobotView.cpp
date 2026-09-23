#include "rl/RobotView.h"

#include <map>
#include <sstream>

namespace mini_pi
{
namespace rl
{

RobotView::RobotView(const std::vector<std::string>& robot_joint_names,
                     const std::vector<std::string>& policy_joint_names)
    : names_(policy_joint_names)
{
    if (policy_joint_names.empty())
        throw std::runtime_error("rl::RobotView: deploy.yaml joint_names is empty");

    if (policy_joint_names.size() != robot_joint_names.size())
    {
        // A policy that drives a strict subset of the robot's joints is a
        // legitimate future case, but it needs a decision about what the
        // remaining joints do, and no such decision has been taken. Refuse
        // rather than guess.
        throw std::runtime_error(
            "rl::RobotView: the policy names " + std::to_string(policy_joint_names.size()) +
            " joints but the robot has " + std::to_string(robot_joint_names.size()) +
            ". Partial-DOF policies are not supported.");
    }

    std::map<std::string, int> robot_index;
    for (std::size_t r = 0; r < robot_joint_names.size(); ++r)
    {
        if (!robot_index.emplace(robot_joint_names[r], static_cast<int>(r)).second)
            throw std::runtime_error("rl::RobotView: duplicate robot joint name '" +
                                     robot_joint_names[r] + "' in mapping.yaml");
    }

    std::map<std::string, int> seen;
    policy_to_robot_.reserve(policy_joint_names.size());
    for (std::size_t p = 0; p < policy_joint_names.size(); ++p)
    {
        const std::string& n = policy_joint_names[p];
        if (!seen.emplace(n, static_cast<int>(p)).second)
            throw std::runtime_error("rl::RobotView: duplicate policy joint name '" + n +
                                     "' in deploy.yaml joint_names");

        auto it = robot_index.find(n);
        if (it == robot_index.end())
        {
            std::ostringstream os;
            os << "rl::RobotView: policy joint '" << n
               << "' is not a robot joint. Known robot joints:";
            for (const auto& rn : robot_joint_names) os << " " << rn;
            throw std::runtime_error(os.str());
        }
        policy_to_robot_.push_back(it->second);
    }

    data.joint_pos.assign(names_.size(), 0.0f);
    data.joint_vel.assign(names_.size(), 0.0f);
    data.default_joint_pos.assign(names_.size(), 0.0f);
}

void RobotView::update(const RobotState& state, const VelocityCommand& cmd)
{
    for (std::size_t p = 0; p < policy_to_robot_.size(); ++p)
    {
        const int r = policy_to_robot_[p];
        data.joint_pos[p] = static_cast<float>(state.robot_q[static_cast<std::size_t>(r)]);
        data.joint_vel[p] = static_cast<float>(state.robot_dq[static_cast<std::size_t>(r)]);
    }
    for (std::size_t i = 0; i < 3; ++i)
    {
        data.root_ang_vel_b[i] = static_cast<float>(state.angular_velocity[i]);
        data.root_euler[i]     = static_cast<float>(state.rpy[i]);
    }
    data.command = cmd;
    data.valid   = state.motor_valid && state.imu_valid;
}

void RobotView::policyToRobotArray(const std::vector<float>& policy_values,
                                   JointArray& robot_values) const
{
    for (std::size_t p = 0; p < policy_to_robot_.size() && p < policy_values.size(); ++p)
    {
        robot_values[static_cast<std::size_t>(policy_to_robot_[p])] =
            static_cast<double>(policy_values[p]);
    }
}

std::string RobotView::describe() const
{
    std::ostringstream os;
    os << "rl::RobotView: " << names_.size() << " joints, policy -> robot index\n";
    for (std::size_t p = 0; p < names_.size(); ++p)
    {
        os << "  [" << p << "] " << names_[p] << " -> robot[" << policy_to_robot_[p] << "]"
           << "  default_q=" << (p < data.default_joint_pos.size() ? data.default_joint_pos[p] : 0.0f)
           << "\n";
    }
    return os.str();
}

} // namespace rl
} // namespace mini_pi
