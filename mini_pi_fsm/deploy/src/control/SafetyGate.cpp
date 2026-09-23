#include "control/SafetyGate.h"

#include "control/ControlContext.h"

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <vector>

namespace mini_pi
{

bool SafetyGate::loadFromYaml(const std::string& path)
{
    YAML::Node n;
    try { n = YAML::LoadFile(path); }
    catch (const YAML::Exception& e)
    {
        ROS_ERROR("SafetyGate: cannot load '%s': %s", path.c_str(), e.what());
        return false;
    }

    const YAML::Node fr = n["fault_response"];
    if (!fr)
    {
        ROS_INFO("SafetyGate: no `fault_response` block in %s -- using the SDK "
                 "default response (kp = 0, kd = 1, no firmware stop).", path.c_str());
        return true;
    }

    if (fr["kd"])
    {
        const auto v = fr["kd"].as<std::vector<double>>();
        if (v.size() != MINI_PI_DOF)
        {
            ROS_ERROR("SafetyGate: fault_response.kd has %zu entries, expected %zu",
                      v.size(), MINI_PI_DOF);
            return false;
        }
        for (std::size_t i = 0; i < MINI_PI_DOF; ++i) cfg_.kd[i] = v[i];
    }
    if (fr["hard_stop"]) cfg_.hard_stop = fr["hard_stop"].as<bool>();

    ROS_INFO("SafetyGate: fault response = damping kd[0]=%.3f, hard_stop=%s",
             cfg_.kd[0], cfg_.hard_stop ? "true" : "false");
    return true;
}

void SafetyGate::arm()
{
    armed_.store(true, std::memory_order_release);
    ROS_INFO("SafetyGate: armed. Runtime safety faults now latch and override "
             "the FSM command.");
}

void SafetyGate::raise(const SafetyReport& rep)
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

    ROS_ERROR("SafetyGate: CRITICAL FAULT %s (joint=%d, value=%f): %s. "
              "FSM commands are now overridden with a damping hold; the FSM is "
              "being driven to Passive. Clear with the reset input (RB + Y) "
              "once the cause is addressed.",
              to_string(rep.fault), rep.joint, rep.value, rep.detail.c_str());
}

SafetyReport SafetyGate::fault() const
{
    if (!faulted_.load(std::memory_order_acquire)) return SafetyReport{};
    std::lock_guard<std::mutex> lk(mutex_);
    return report_;
}

double SafetyGate::faultAge() const
{
    if (!faulted_.load(std::memory_order_acquire)) return 0.0;
    std::lock_guard<std::mutex> lk(mutex_);
    return age_seconds(since_);
}

bool SafetyGate::clear()
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

    ROS_WARN("SafetyGate: fault %s cleared by operator after %.1f s. The FSM is "
             "in Passive; RL must be requested again deliberately.",
             to_string(was), held);
    return true;
}

void SafetyGate::writeSafeOverride(ControlContext& ctx)
{
    const RobotState& s = *ctx.state;

    MotorCommand mcmd;
    RobotCommand safe;
    for (std::size_t j = 0; j < MINI_PI_DOF; ++j)
    {
        // The motor runs POS_VEL_TQE_KP_KD2, so a position field is always
        // transmitted. With kp == 0 it has no effect; the measured pose is sent
        // anyway so that a later resume starts from a consistent target.
        safe.q[j]      = std::isfinite(s.robot_q[j]) ? s.robot_q[j] : 0.0;
        safe.dq[j]     = 0.0;
        safe.torque[j] = 0.0;
        safe.kp[j]     = 0.0;
        safe.kd[j]     = cfg_.kd[j];
    }
    safe.valid = true;
    safe.stamp = Clock::now();

    ctx.mapper->robotToMotor(safe, mcmd);
    ctx.hardware->write(mcmd);
}

bool SafetyGate::publish(RobotCommand& cmd, ControlContext& ctx)
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
            // IMU tilt fallback -- both damp. Unchanged from State_Fault.
            if (ctx.policy->running()) ctx.policy->stop();
            if (cfg_.hard_stop)
            {
                ROS_WARN("SafetyGate: hard_stop enabled -- issuing the firmware "
                         "motor stop. The robot will go limp.");
                ctx.hardware->stop();
            }
        }

        ROS_WARN_THROTTLE(5.0,
            "SafetyGate: overriding FSM output with a damping hold for %.1f s. "
            "reason=%s. Clear with the reset input once the cause is addressed.",
            faultAge(), to_string(fault().fault));

        writeSafeOverride(ctx);
        return false;
    }

    if (!cmd.valid)
    {
        // A state that produced nothing this cycle. Not promoted to a latched
        // fault -- SafetyManager::checkCommand() decides that below, via the
        // stamp -- but the bus must not go quiet, so damp.
        writeSafeOverride(ctx);
        return false;
    }

    ctx.safety->clampCommand(cmd);

    const SafetyReport rep = ctx.safety->checkCommand(cmd);
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

} // namespace mini_pi
