#include "common/JointMapper.h"

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <sstream>

namespace deploy
{

const char* to_string(TransformMode m)
{
    switch (m)
    {
        case TransformMode::Full:     return "Full";
        case TransformMode::Identity: return "Identity";
    }
    return "Unknown";
}

namespace
{
template <typename T>
bool read_vec(const YAML::Node& node, const char* key, std::vector<T>& out, std::size_t expect)
{
    if (!node[key])
    {
        ROS_ERROR("JointMapper: mapping.yaml is missing required key '%s'", key);
        return false;
    }
    out = node[key].as<std::vector<T>>();
    if (out.size() != expect)
    {
        ROS_ERROR("JointMapper: '%s' has %zu entries, expected %zu", key, out.size(), expect);
        return false;
    }
    return true;
}
} // namespace

bool JointMapper::loadFromYaml(const std::string& path)
{
    valid_ = false;
    YAML::Node root;
    try
    {
        root = YAML::LoadFile(path);
    }
    catch (const YAML::Exception& e)
    {
        ROS_ERROR("JointMapper: cannot load '%s': %s", path.c_str(), e.what());
        return false;
    }

    const std::size_t n = NUM_DOF;
    if (!read_vec(root, "joint_names", cfg_.joint_names, n)) return false;
    if (!read_vec(root, "map_index", cfg_.map_index, n)) return false;
    if (!read_vec(root, "direction", cfg_.direction, n)) return false;
    if (!read_vec(root, "joint_offset", cfg_.joint_offset, n)) return false;

    // Reject the superseded key names outright rather than silently ignoring
    // them: their semantics differed (see doc/architecture.md).
    for (const char* dead : {"zero_offset", "default_pose", "urdf_offset"})
    {
        if (root[dead])
        {
            ROS_ERROR("JointMapper: mapping.yaml still contains the obsolete key '%s'. "
                      "Use 'joint_offset' (robot-order, radians). A policy default pose, "
                      "if one is ever needed, belongs to the policy layer, not here.", dead);
            return false;
        }
    }

    if (root["transform_mode"])
    {
        const std::string m = root["transform_mode"].as<std::string>();
        if (m == "Full" || m == "full")          cfg_.mode = TransformMode::Full;
        else if (m == "Identity" || m == "identity") cfg_.mode = TransformMode::Identity;
        else
        {
            ROS_ERROR("JointMapper: unknown transform_mode '%s' (expected Full or Identity)",
                      m.c_str());
            return false;
        }
    }

    // map_index must be a permutation of [0, n)
    std::vector<int> seen(n, 0);
    for (int idx : cfg_.map_index)
    {
        if (idx < 0 || idx >= static_cast<int>(n))
        {
            ROS_ERROR("JointMapper: map_index entry %d out of range [0,%zu)", idx, n);
            return false;
        }
        if (seen[idx]++)
        {
            ROS_ERROR("JointMapper: map_index entry %d appears more than once", idx);
            return false;
        }
    }

    for (int d : cfg_.direction)
    {
        if (d != 1 && d != -1)
        {
            ROS_ERROR("JointMapper: direction entry %d is neither +1 nor -1", d);
            return false;
        }
    }

    for (std::size_t i = 0; i < n; ++i)
    {
        if (!std::isfinite(cfg_.joint_offset[i]))
        {
            ROS_ERROR("JointMapper: joint_offset[%zu] is not finite", i);
            return false;
        }
        joint_offset_[i] = cfg_.joint_offset[i];
    }

    valid_ = true;
    return true;
}

void JointMapper::motorToRobot(const JointArray& motor_q,
                               const JointArray& motor_dq,
                               const JointArray& motor_tau,
                               JointArray& robot_q,
                               JointArray& robot_dq,
                               JointArray& robot_tau) const
{
    const bool full = (cfg_.mode == TransformMode::Full);
    for (std::size_t j = 0; j < NUM_DOF; ++j)
    {
        const int m = cfg_.map_index[j];
        const double s = full ? static_cast<double>(cfg_.direction[j]) : 1.0;
        const double off = full ? joint_offset_[j] : 0.0;

        robot_q[j]   = s * motor_q[m] - off;
        robot_dq[j]  = s * motor_dq[m];
        // The original controller applies no sign to torque in either
        // direction (verified in the disassembly). Reproduced exactly.
        robot_tau[j] = motor_tau[m];
    }
}

void JointMapper::robotToMotor(const RobotCommand& in, MotorCommand& out) const
{
    const bool full = (cfg_.mode == TransformMode::Full);
    for (std::size_t j = 0; j < NUM_DOF; ++j)
    {
        const int m = cfg_.map_index[j];
        const double s = full ? static_cast<double>(cfg_.direction[j]) : 1.0;
        const double off = full ? joint_offset_[j] : 0.0;

        out.q[m] = s * (in.q[j] + off);
        // No sign on outgoing dq / tau -- matches the original controller.
        out.dq[m]     = in.dq[j];
        out.torque[m] = in.torque[j];
        // Gains are magnitudes: permuted only.
        out.kp[m]     = in.kp[j];
        out.kd[m]     = in.kd[j];
    }
    out.valid = in.valid;
}

std::string JointMapper::describe() const
{
    std::ostringstream os;
    os << "JointMapper (" << NUM_DOF << " dof, mode=" << to_string(cfg_.mode) << ")\n";
    os << "  permutation owner: this class (hardware layer uses raw Motors[])\n";
    os << "  robot_idx  joint_name              motor_idx  dir  joint_offset\n";
    for (std::size_t j = 0; j < NUM_DOF; ++j)
    {
        char line[160];
        std::snprintf(line, sizeof(line), "  %2zu  %-22s %2d   %+d   %+.4f\n",
                      j, cfg_.joint_names[j].c_str(), cfg_.map_index[j],
                      cfg_.direction[j], joint_offset_[j]);
        os << line;
    }
    return os.str();
}

} // namespace deploy
