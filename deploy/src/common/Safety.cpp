#include "common/Safety.h"

#include "FSM/ControlContext.h"

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>

namespace deploy
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
        ROS_INFO("Safety: '%s' is empty (no spec yet) -- that check stays off", key);
        return false;
    }
    if (out.size() != NUM_DOF)
    {
        ROS_ERROR("Safety: '%s' has %zu entries, expected %zu",
                  key, out.size(), NUM_DOF);
        out.clear();
        return false;
    }
    return true;
}
} // namespace

bool Safety::loadFromYaml(const std::string& path)
{
    YAML::Node n;
    try { n = YAML::LoadFile(path); }
    catch (const YAML::Exception& e)
    {
        ROS_ERROR("Safety: cannot load '%s': %s", path.c_str(), e.what());
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
            ROS_WARN("Safety: position limit check requested but q_lower/q_upper are "
                     "missing or malformed -- DISABLING it rather than inventing bounds.");
        cfg_.enable_position_limit = false;
    }
    if (!read_limits(n, "dq_max", cfg_.dq_max))
    {
        if (cfg_.enable_velocity_limit)
            ROS_WARN("Safety: velocity limit check requested but dq_max is missing "
                     "or malformed -- DISABLING it.");
        cfg_.enable_velocity_limit = false;
    }
    if (!read_limits(n, "tau_max", cfg_.tau_max))
    {
        if (cfg_.enable_torque_limit)
            ROS_WARN("Safety: torque limit check requested but tau_max is missing "
                     "or malformed -- DISABLING it.");
        cfg_.enable_torque_limit = false;
    }

    if (cfg_.enable_position_limit)
    {
        for (std::size_t j = 0; j < NUM_DOF; ++j)
        {
            if (!(cfg_.q_lower[j] < cfg_.q_upper[j]))
            {
                ROS_ERROR("Safety: q_lower[%zu] (%f) is not below q_upper[%zu] (%f)",
                          j, cfg_.q_lower[j], j, cfg_.q_upper[j]);
                return false;
            }
        }
    }

    ROS_INFO("Safety: motor_timeout=%.3fs imu_timeout=%.3fs command_timeout=%.3fs | "
             "checks: motor=%d imu=%d finite=%d pos=%d vel=%d tau=%d cmd=%d orient=%d",
             cfg_.motor_timeout_s, cfg_.imu_timeout_s, cfg_.command_timeout_s,
             cfg_.enable_motor_timeout, cfg_.enable_imu_timeout, cfg_.enable_finite_check,
             cfg_.enable_position_limit, cfg_.enable_velocity_limit, cfg_.enable_torque_limit,
             cfg_.enable_command_timeout, cfg_.enable_orientation);

    // ---- fault_response ------------------------------------------------------
    auto& resp = cfg_.fault_response;
    const YAML::Node fr = n["fault_response"];
    if (!fr)
    {
        ROS_INFO("Safety: no `fault_response` block in %s -- using the SDK "
                 "default response (kp = 0, kd = 1, no firmware stop).", path.c_str());
        return true;
    }
    if (fr["kd"])
    {
        const auto v = fr["kd"].as<std::vector<double>>();
        if (v.size() != NUM_DOF)
        {
            ROS_ERROR("Safety: fault_response.kd has %zu entries, expected %zu",
                      v.size(), NUM_DOF);
            return false;
        }
        for (std::size_t i = 0; i < NUM_DOF; ++i) resp.kd[i] = v[i];
    }
    if (fr["hard_stop"]) resp.hard_stop = fr["hard_stop"].as<bool>();

    ROS_INFO("Safety: fault response = damping kd[0]=%.3f, hard_stop=%s",
             resp.kd[0], resp.hard_stop ? "true" : "false");
    return true;
}

SafetyReport Safety::checkState(const RobotState& state, HardwareStatus hw) const
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
        r.detail = "hardware backend reported Fault (motor fault code or SDK limit flag)";
        for (std::size_t j = 0; j < NUM_DOF; ++j)
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
        for (std::size_t j = 0; j < NUM_DOF; ++j)
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
    // the backend's read() from its own per-motor feedback stamps.
    if (cfg_.enable_motor_timeout)
    {
        if (!state.motor_valid)
        {
            r.fault = SafetyFault::MotorTimeout;
            r.detail = "not all 12 motors have produced a valid frame yet";
            for (std::size_t j = 0; j < NUM_DOF; ++j)
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
            for (std::size_t j = 0; j < NUM_DOF; ++j)
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
        for (std::size_t j = 0; j < NUM_DOF; ++j)
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

SafetyReport Safety::checkCommand(const RobotCommand& cmd) const
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
        for (std::size_t j = 0; j < NUM_DOF; ++j)
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
        for (std::size_t j = 0; j < NUM_DOF; ++j)
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

int Safety::clampCommand(RobotCommand& cmd, std::uint64_t* mask) const
{
    int n = 0;
    std::uint64_t m = 0;
    const auto clampAll = [&](JointArray& v, const std::vector<double>& lo,
                              const std::vector<double>& hi, bool symmetric, std::size_t bit0) {
        for (std::size_t j = 0; j < NUM_DOF; ++j)
        {
            const double l = symmetric ? -hi[j] : lo[j];
            const double c = std::clamp(v[j], l, hi[j]);
            if (c != v[j]) { v[j] = c; ++n; m |= std::uint64_t{1} << (bit0 + j); }
        }
    };
    if (cfg_.enable_position_limit) clampAll(cmd.q, cfg_.q_lower, cfg_.q_upper, false, 0);
    if (cfg_.enable_velocity_limit) clampAll(cmd.dq, cfg_.dq_max, cfg_.dq_max, true, NUM_DOF);
    if (cfg_.enable_torque_limit)   clampAll(cmd.torque, cfg_.tau_max, cfg_.tau_max, true, 2 * NUM_DOF);
    if (mask) *mask = m;
    return n;
}

// ============================== latch and exit ===============================

void Safety::arm()
{
    armed_.store(true, std::memory_order_release);
    ROS_INFO("Safety: armed. Runtime safety faults now latch and override "
             "the FSM command.");
}

void Safety::raise(const SafetyReport& rep)
{
    if (rep.ok()) return;
    if (!armed()) return;                 // startup: "not ready" is not a fault
    if (faulted_.load(std::memory_order_acquire)) return;   // first report wins

    {
        std::lock_guard<std::mutex> lk(mutex_);
        report_ = rep;
        since_  = Clock::now();
    }
    reacted_ = false;
    faulted_.store(true, std::memory_order_release);

    ROS_ERROR("Safety: CRITICAL FAULT %s (joint=%d, value=%f): %s. "
              "FSM commands are now overridden with a damping hold; the FSM is "
              "being driven to Passive. Clear with the reset input (RB + Y) "
              "once the cause is addressed.",
              to_string(rep.fault), rep.joint, rep.value, rep.detail.c_str());
}

SafetyReport Safety::fault() const
{
    if (!faulted_.load(std::memory_order_acquire)) return SafetyReport{};
    std::lock_guard<std::mutex> lk(mutex_);
    return report_;
}

double Safety::faultAge() const
{
    if (!faulted_.load(std::memory_order_acquire)) return 0.0;
    std::lock_guard<std::mutex> lk(mutex_);
    return age_seconds(since_);
}

bool Safety::clear()
{
    if (!faulted_.load(std::memory_order_acquire)) return false;

    SafetyFault was = SafetyFault::None;
    double held = 0.0;
    {
        std::lock_guard<std::mutex> lk(mutex_);
        was  = report_.fault;
        held = age_seconds(since_);
        report_ = SafetyReport{};
        since_  = TimePoint{};
    }
    faulted_.store(false, std::memory_order_release);
    reacted_ = false;

    ROS_WARN("Safety: fault %s cleared by operator after %.1f s. The FSM is "
             "in Passive; RL must be requested again deliberately.",
             to_string(was), held);
    return true;
}

void Safety::writeSafeOverride(ControlContext& ctx)
{
    const RobotState& s = *ctx.state;

    MotorCommand mcmd;
    RobotCommand safe;
    for (std::size_t j = 0; j < NUM_DOF; ++j)
    {
        // The motor runs POS_VEL_TQE_KP_KD2, so a position field is always
        // transmitted. With kp == 0 it has no effect; the measured pose is sent
        // anyway so that a later resume starts from a consistent target.
        safe.q[j]      = std::isfinite(s.robot_q[j]) ? s.robot_q[j] : 0.0;
        safe.dq[j]     = 0.0;
        safe.torque[j] = 0.0;
        safe.kp[j]     = 0.0;
        safe.kd[j]     = cfg_.fault_response.kd[j];
    }
    safe.valid = true;
    safe.stamp = Clock::now();

    ctx.mapper->robotToMotor(safe, mcmd);
    ctx.hardware->write(mcmd);
}

bool Safety::publish(RobotCommand& cmd, ControlContext& ctx)
{
    if (!armed())
    {
        // Startup has not completed. Nothing is transmitted at all, which is
        // what Unitree does too: no LowCmd is published before fsm->start().
        return false;
    }

    if (faulted_.load(std::memory_order_acquire))
    {
        if (!reacted_)
        {
            reacted_ = true;
            // Detection already happened; this is the response, in order of
            // increasing severity. Default is damping, NOT the firmware stop:
            // motor::stop() (serial mode 0x03) cuts output outright, which on a
            // standing biped means it falls, whereas the SDK's own reflexes --
            // HtdwMotor::protectMotor() (kp=0, kd=1) and robot::motor_send_2()'s
            // IMU tilt fallback -- both damp.
            ctx.policy->stop();   // a no-op when the thread is not running
            if (cfg_.fault_response.hard_stop)
            {
                ROS_WARN("Safety: hard_stop enabled -- issuing the firmware "
                         "motor stop. The robot will go limp.");
                ctx.hardware->stop();
            }
        }

        ROS_WARN_THROTTLE(5.0,
            "Safety: overriding FSM output with a damping hold for %.1f s. "
            "reason=%s. Clear with the reset input once the cause is addressed.",
            faultAge(), to_string(fault().fault));

        writeSafeOverride(ctx);
        return false;
    }

    if (!cmd.valid)
    {
        // A state that produced nothing this cycle. Not promoted to a latched
        // fault here, but the bus must not go quiet, so damp.
        writeSafeOverride(ctx);
        return false;
    }

    std::uint64_t clamp_mask = 0;
    if (clampCommand(cmd, &clamp_mask) > 0) clamped_commands_.fetch_add(1, std::memory_order_relaxed);
    last_clamp_mask_.store(clamp_mask, std::memory_order_relaxed);

    const SafetyReport rep = checkCommand(cmd);
    if (!rep.ok())
    {
        raise(rep);
        writeSafeOverride(ctx);
        return false;
    }

    MotorCommand mcmd;
    ctx.mapper->robotToMotor(cmd, mcmd);
    ctx.hardware->write(mcmd);
    return true;
}

} // namespace deploy
