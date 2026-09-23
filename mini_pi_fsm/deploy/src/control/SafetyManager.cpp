#include "control/SafetyManager.h"

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>

namespace mini_pi
{

const char* to_string(SafetyFault f)
{
    switch (f)
    {
        case SafetyFault::None:               return "NONE";
        case SafetyFault::MotorTimeout:       return "MOTOR_TIMEOUT";
        case SafetyFault::ImuTimeout:         return "IMU_TIMEOUT";
        case SafetyFault::NanState:           return "NAN_STATE";
        case SafetyFault::NanCommand:         return "NAN_COMMAND";
        case SafetyFault::JointPositionLimit: return "JOINT_POSITION_LIMIT";
        case SafetyFault::JointVelocityLimit: return "JOINT_VELOCITY_LIMIT";
        case SafetyFault::JointTorqueLimit:   return "JOINT_TORQUE_LIMIT";
        case SafetyFault::CommandTimeout:     return "COMMAND_TIMEOUT";
        case SafetyFault::CommandInvalid:     return "COMMAND_INVALID";
        case SafetyFault::HardwareFault:      return "HARDWARE_FAULT";
        case SafetyFault::BadOrientation:     return "BAD_ORIENTATION";
    }
    return "UNKNOWN";
}

namespace
{
bool read_limits(const YAML::Node& n, const char* key, std::vector<double>& out)
{
    if (!n[key]) return false;
    out = n[key].as<std::vector<double>>();
    if (out.empty())
    {
        // Deliberately left empty in safety.yaml until a real spec exists.
        ROS_INFO("SafetyManager: '%s' is empty (no spec yet) -- that check stays off", key);
        return false;
    }
    if (out.size() != MINI_PI_DOF)
    {
        ROS_ERROR("SafetyManager: '%s' has %zu entries, expected %zu",
                  key, out.size(), MINI_PI_DOF);
        out.clear();
        return false;
    }
    return true;
}
} // namespace

bool SafetyManager::loadFromYaml(const std::string& path)
{
    YAML::Node n;
    try { n = YAML::LoadFile(path); }
    catch (const YAML::Exception& e)
    {
        ROS_ERROR("SafetyManager: cannot load '%s': %s", path.c_str(), e.what());
        return false;
    }

    auto flag = [&](const char* k, bool def) { return n[k] ? n[k].as<bool>() : def; };
    auto num  = [&](const char* k, double def) { return n[k] ? n[k].as<double>() : def; };

    cfg_.enable_motor_timeout   = flag("enable_motor_timeout", true);
    cfg_.enable_imu_timeout     = flag("enable_imu_timeout", true);
    cfg_.enable_finite_check    = flag("enable_finite_check", true);
    cfg_.enable_position_limit  = flag("enable_position_limit", true);
    cfg_.enable_velocity_limit  = flag("enable_velocity_limit", false);
    cfg_.enable_torque_limit    = flag("enable_torque_limit", false);
    cfg_.enable_command_timeout = flag("enable_command_timeout", true);
    cfg_.enable_orientation     = flag("enable_orientation", true);

    cfg_.motor_timeout_s   = num("motor_timeout_s", 0.1);
    cfg_.imu_timeout_s     = num("imu_timeout_s", 0.2);
    cfg_.command_timeout_s = num("command_timeout_s", 0.05);
    cfg_.max_tilt_rad      = num("max_tilt_rad", 1.0);

    // A missing or malformed limit vector disables that check rather than
    // inventing bounds. Never guess a limit.
    if (!read_limits(n, "q_lower", cfg_.q_lower) || !read_limits(n, "q_upper", cfg_.q_upper))
    {
        if (cfg_.enable_position_limit)
            ROS_WARN("SafetyManager: position limit check requested but q_lower/q_upper are "
                     "missing or malformed -- DISABLING it rather than inventing bounds.");
        cfg_.enable_position_limit = false;
    }
    if (!read_limits(n, "dq_max", cfg_.dq_max))
    {
        if (cfg_.enable_velocity_limit)
            ROS_WARN("SafetyManager: velocity limit check requested but dq_max is missing "
                     "or malformed -- DISABLING it.");
        cfg_.enable_velocity_limit = false;
    }
    if (!read_limits(n, "tau_max", cfg_.tau_max))
    {
        if (cfg_.enable_torque_limit)
            ROS_WARN("SafetyManager: torque limit check requested but tau_max is missing "
                     "or malformed -- DISABLING it.");
        cfg_.enable_torque_limit = false;
    }

    if (cfg_.enable_position_limit)
    {
        for (std::size_t j = 0; j < MINI_PI_DOF; ++j)
        {
            if (!(cfg_.q_lower[j] < cfg_.q_upper[j]))
            {
                ROS_ERROR("SafetyManager: q_lower[%zu] (%f) is not below q_upper[%zu] (%f)",
                          j, cfg_.q_lower[j], j, cfg_.q_upper[j]);
                return false;
            }
        }
    }

    ROS_INFO("SafetyManager: motor_timeout=%.3fs imu_timeout=%.3fs command_timeout=%.3fs | "
             "checks: motor=%d imu=%d finite=%d pos=%d vel=%d tau=%d cmd=%d orient=%d",
             cfg_.motor_timeout_s, cfg_.imu_timeout_s, cfg_.command_timeout_s,
             cfg_.enable_motor_timeout, cfg_.enable_imu_timeout, cfg_.enable_finite_check,
             cfg_.enable_position_limit, cfg_.enable_velocity_limit, cfg_.enable_torque_limit,
             cfg_.enable_command_timeout, cfg_.enable_orientation);
    return true;
}

SafetyReport SafetyManager::checkState(const RobotState& state, HardwareStatus hw) const
{
    SafetyReport r;

    if (hw == HardwareStatus::Uninitialized)
    {
        r.fault = SafetyFault::HardwareFault;
        r.detail = "hardware not initialized";
        return r;
    }

    if (hw == HardwareStatus::Fault)
    {
        r.fault = SafetyFault::HardwareFault;
        r.detail = "HighTorqueHardware reported Fault (motor fault code or SDK limit flag)";
        for (std::size_t j = 0; j < MINI_PI_DOF; ++j)
        {
            if (state.motor_fault[j] != 0)
            {
                r.joint = static_cast<int>(j);
                r.value = state.motor_fault[j];
                r.detail += " (motor slot " + std::to_string(j) + " fault code "
                          + std::to_string(state.motor_fault[j]) + ")";
                break;
            }
        }
        return r;
    }

    // Finite check first: a NaN makes every later comparison meaningless.
    if (cfg_.enable_finite_check &&
        (!is_finite(state.robot_q) || !is_finite(state.robot_dq) ||
         !is_finite(state.robot_tau)))
    {
        r.fault = SafetyFault::NanState;
        r.detail = "non-finite joint state";
        for (std::size_t j = 0; j < MINI_PI_DOF; ++j)
        {
            if (!std::isfinite(state.robot_q[j]) || !std::isfinite(state.robot_dq[j]) ||
                !std::isfinite(state.robot_tau[j]))
            {
                r.joint = static_cast<int>(j);
                break;
            }
        }
        return r;
    }

    // Per-motor freshness. stale_motor_count is computed by
    // HighTorqueHardware::read() from motor_back_t::time, per motor.
    if (cfg_.enable_motor_timeout)
    {
        if (!state.motor_valid)
        {
            r.fault = SafetyFault::MotorTimeout;
            r.detail = "not all 12 motors have produced a valid frame yet";
            for (std::size_t j = 0; j < MINI_PI_DOF; ++j)
            {
                if (!state.motor_fresh[j]) { r.joint = static_cast<int>(j); break; }
            }
            return r;
        }
        if (state.stale_motor_count > 0)
        {
            r.fault = SafetyFault::MotorTimeout;
            r.detail = std::to_string(state.stale_motor_count) +
                       " motor(s) stale (timeout " + std::to_string(cfg_.motor_timeout_s) + "s)";
            for (std::size_t j = 0; j < MINI_PI_DOF; ++j)
            {
                if (!state.motor_fresh[j])
                {
                    r.joint = static_cast<int>(j);
                    r.value = state.motor_age[j];
                    break;
                }
            }
            return r;
        }
    }

    if (cfg_.enable_imu_timeout)
    {
        if (!state.imu_valid || state.imu_age > cfg_.imu_timeout_s)
        {
            r.fault = SafetyFault::ImuTimeout;
            r.value = state.imu_age;
            r.detail = state.imu_valid ? "IMU stale" : "no IMU sample received yet";
            return r;
        }
    }

    if (cfg_.enable_velocity_limit)
    {
        for (std::size_t j = 0; j < MINI_PI_DOF; ++j)
        {
            if (std::abs(state.robot_dq[j]) > cfg_.dq_max[j])
            {
                r.fault = SafetyFault::JointVelocityLimit;
                r.joint = static_cast<int>(j);
                r.value = state.robot_dq[j];
                r.detail = "measured joint velocity over limit";
                return r;
            }
        }
    }

    if (cfg_.enable_orientation && state.imu_valid)
    {
        const double tilt = std::max(std::abs(state.rpy[0]), std::abs(state.rpy[1]));
        if (tilt > cfg_.max_tilt_rad)
        {
            r.fault = SafetyFault::BadOrientation;
            r.value = tilt;
            r.detail = "body tilt beyond max_tilt_rad";
            return r;
        }
    }

    return r;
}

SafetyReport SafetyManager::checkCommand(const RobotCommand& cmd) const
{
    SafetyReport r;

    if (!cmd.valid)
    {
        r.fault = SafetyFault::CommandInvalid;
        r.detail = "command not marked valid";
        return r;
    }

    if (cfg_.enable_finite_check &&
        (!is_finite(cmd.q) || !is_finite(cmd.dq) || !is_finite(cmd.torque) ||
         !is_finite(cmd.kp) || !is_finite(cmd.kd)))
    {
        r.fault = SafetyFault::NanCommand;
        r.detail = "non-finite command";
        return r;
    }

    // Freshness: a state that stops refreshing its command cannot have the
    // previous one reused indefinitely.
    if (cfg_.enable_command_timeout)
    {
        const double age = age_seconds(cmd.stamp);
        if (age > cfg_.command_timeout_s)
        {
            r.fault = SafetyFault::CommandTimeout;
            r.value = age;
            r.detail = "command older than command_timeout_s";
            return r;
        }
    }

    if (cfg_.enable_position_limit)
    {
        for (std::size_t j = 0; j < MINI_PI_DOF; ++j)
        {
            if (cmd.q[j] < cfg_.q_lower[j] || cmd.q[j] > cfg_.q_upper[j])
            {
                r.fault = SafetyFault::JointPositionLimit;
                r.joint = static_cast<int>(j);
                r.value = cmd.q[j];
                r.detail = "commanded position outside limits";
                return r;
            }
        }
    }

    if (cfg_.enable_torque_limit)
    {
        for (std::size_t j = 0; j < MINI_PI_DOF; ++j)
        {
            if (std::abs(cmd.torque[j]) > cfg_.tau_max[j])
            {
                r.fault = SafetyFault::JointTorqueLimit;
                r.joint = static_cast<int>(j);
                r.value = cmd.torque[j];
                r.detail = "commanded torque over limit";
                return r;
            }
        }
    }

    return r;
}

int SafetyManager::clampCommand(RobotCommand& cmd) const
{
    int n = 0;
    if (cfg_.enable_position_limit)
    {
        for (std::size_t j = 0; j < MINI_PI_DOF; ++j)
        {
            const double c = std::clamp(cmd.q[j], cfg_.q_lower[j], cfg_.q_upper[j]);
            if (c != cmd.q[j]) { cmd.q[j] = c; ++n; }
        }
    }
    if (cfg_.enable_velocity_limit)
    {
        for (std::size_t j = 0; j < MINI_PI_DOF; ++j)
        {
            const double c = std::clamp(cmd.dq[j], -cfg_.dq_max[j], cfg_.dq_max[j]);
            if (c != cmd.dq[j]) { cmd.dq[j] = c; ++n; }
        }
    }
    if (cfg_.enable_torque_limit)
    {
        for (std::size_t j = 0; j < MINI_PI_DOF; ++j)
        {
            const double c = std::clamp(cmd.torque[j], -cfg_.tau_max[j], cfg_.tau_max[j]);
            if (c != cmd.torque[j]) { cmd.torque[j] = c; ++n; }
        }
    }
    return n;
}

} // namespace mini_pi
