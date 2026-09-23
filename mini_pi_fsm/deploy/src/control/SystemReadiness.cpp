#include "control/SystemReadiness.h"

#include <ros/ros.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>
#include <thread>

namespace mini_pi
{

ReadinessConfig loadReadinessConfig(const YAML::Node& startup)
{
    ReadinessConfig c;
    if (!startup) return c;
    if (startup["require_fresh_s"]) c.require_fresh_s = startup["require_fresh_s"].as<double>();
    if (startup["require_imu"])     c.require_imu     = startup["require_imu"].as<bool>();
    if (startup["timeout_s"])       c.timeout_s       = startup["timeout_s"].as<double>();
    if (startup["poll_dt_s"])       c.poll_dt_s       = startup["poll_dt_s"].as<double>();
    if (!(c.poll_dt_s > 0.0)) c.poll_dt_s = 0.02;
    return c;
}

void refreshReadinessState(ControlContext& ctx)
{
    RobotState& s = *ctx.state;
    std::lock_guard<std::mutex> lk(*ctx.state_mutex);
    ctx.hardware->read(s);
    ctx.mapper->motorToRobot(s.motor_q, s.motor_dq, s.motor_tau,
                             s.robot_q, s.robot_dq, s.robot_tau);
}

// Moved verbatim from State_Init::notReadyReason(). Every check, every
// threshold and every message is unchanged; only the caller moved.
std::string readinessReason(const ControlContext& ctx, const ReadinessConfig& cfg)
{
    const RobotState& s = *ctx.state;
    const HardwareInterface& hw = *ctx.hardware;
    const JointMapper& jm = *ctx.mapper;

    // --- hardware ------------------------------------------------------
    if (!hw.initialized())
        return std::string(hw.backendName()) + " backend not initialized";
    if (hw.motorCount() != MINI_PI_DOF)
        return "motor count " + std::to_string(hw.motorCount()) + " != " +
               std::to_string(MINI_PI_DOF);

    // --- mapping -------------------------------------------------------
    if (!jm.valid())                  return "JointMapper not loaded";
    if (jm.jointNames().size() != MINI_PI_DOF)
        return "mapping joint_names count != " + std::to_string(MINI_PI_DOF);
    if (jm.mapIndex().size() != MINI_PI_DOF)
        return "mapping map_index count != " + std::to_string(MINI_PI_DOF);
    if (jm.direction().size() != MINI_PI_DOF)
        return "mapping direction count != " + std::to_string(MINI_PI_DOF);
    {
        // map_index must be a valid, unique permutation. JointMapper already
        // enforces this at load; re-check here so startup is self-contained and
        // a future mapping source cannot bypass it.
        std::array<int, MINI_PI_DOF> seen{};
        for (int idx : jm.mapIndex())
        {
            if (idx < 0 || idx >= static_cast<int>(MINI_PI_DOF))
                return "mapping map_index contains out-of-range entry " + std::to_string(idx);
            if (seen[idx]++) return "mapping map_index entry " + std::to_string(idx) +
                                    " is not unique";
        }
        for (int d : jm.direction())
        {
            if (d != 1 && d != -1)
                return "mapping direction entry " + std::to_string(d) + " is not +/-1";
        }
        for (std::size_t j = 0; j < MINI_PI_DOF; ++j)
        {
            if (!std::isfinite(jm.jointOffset()[j]))
                return "mapping joint_offset[" + std::to_string(j) + "] is not finite";
        }
    }

    // --- motor feedback -------------------------------------------------
    if (!s.motor_valid)
        return "not all " + std::to_string(MINI_PI_DOF) +
               " motors have produced a valid frame yet";

    for (std::size_t i = 0; i < MINI_PI_DOF; ++i)
    {
        if (s.motor_age[i] > cfg.require_fresh_s)
            return "motor " + std::to_string(i) + " feedback stale (" +
                   std::to_string(s.motor_age[i]) + " s)";
    }
    if (s.stale_motor_count != 0)
        return std::to_string(s.stale_motor_count) + " motor(s) stale";

    if (!is_finite(s.motor_q) || !is_finite(s.motor_dq) || !is_finite(s.motor_tau))
        return "non-finite raw motor feedback";
    if (!is_finite(s.robot_q) || !is_finite(s.robot_dq) || !is_finite(s.robot_tau))
        return "non-finite robot-space state";

    for (std::size_t i = 0; i < MINI_PI_DOF; ++i)
    {
        // Only fault codes the SDK actually reports. Nothing is invented here:
        // motor_back_t::fault is the sole health field the SDK exposes.
        if (s.motor_fault[i] != 0)
            return "motor " + std::to_string(i) + " reports fault code " +
                   std::to_string(s.motor_fault[i]);
    }

    // --- IMU ------------------------------------------------------------
    if (cfg.require_imu)
    {
        // Readiness must never be LOOSER than the runtime check, or the FSM
        // starts and SafetyManager latches IMU_TIMEOUT on its first cycle for
        // the very same snapshot that just passed. The motor path already has
        // this property for free -- readiness and SafetyManager both read
        // `stale_motor_count`, which the backends compute against
        // safety.yaml's motor_timeout_s -- but the IMU has no equivalent
        // shared field, so the bound is taken explicitly here.
        //
        // No new value is introduced: this is the stricter of the two numbers
        // that already exist (robot.yaml startup.require_fresh_s and
        // safety.yaml imu_timeout_s). When the runtime check is disabled it
        // cannot fire at all, so there is nothing to be consistent with and
        // the startup threshold is left alone.
        const double runtime_bound = ctx.safety->config().enable_imu_timeout
            ? ctx.safety->config().imu_timeout_s
            : std::numeric_limits<double>::infinity();
        const double imu_fresh_s = std::min(cfg.require_fresh_s, runtime_bound);

        if (!s.imu_valid) return "no IMU sample received yet";
        if (s.imu_age > imu_fresh_s)
            return "IMU stale (" + std::to_string(s.imu_age) + " s, limit " +
                   std::to_string(imu_fresh_s) + " s)";
        if (!std::isfinite(s.orientation[0]) || !std::isfinite(s.orientation[1]) ||
            !std::isfinite(s.orientation[2]) || !std::isfinite(s.orientation[3]))
            return "non-finite IMU quaternion";
        const double n2 = s.orientation[0] * s.orientation[0]
                        + s.orientation[1] * s.orientation[1]
                        + s.orientation[2] * s.orientation[2]
                        + s.orientation[3] * s.orientation[3];
        if (n2 < 0.5 || n2 > 1.5) return "IMU quaternion is not unit-norm";
    }

    if (hw.status() == HardwareStatus::Fault)
        return std::string(hw.backendName()) + " backend reports Fault";

    return {};
}

namespace
{
/// The readiness dump State_Init::run() printed once, unchanged.
void logReadinessDump(const ControlContext& ctx)
{
    std::ostringstream os;
    os << "\n===== Mini-Pi readiness =====\n"
       << "backend            : " << ctx.hardware->backendName() << "\n"
       << "dry_run            : "
       << (ctx.hardware->dryRun() ? "TRUE (no motor output)" : "FALSE (LIVE MOTORS)")
       << "\nmotors             : " << ctx.hardware->motorCount()
       << "  (stale: " << ctx.state->stale_motor_count << ")\n";
    const auto& names = ctx.hardware->motorNames();
    const auto& ids   = ctx.hardware->motorIds();
    for (std::size_t i = 0; i < names.size(); ++i)
    {
        os << "  motor[" << i << "] id=" << ids[i] << " name=" << names[i]
           << " age=" << ctx.state->motor_age[i] << "s\n";
    }
    os << "imu age            : " << ctx.state->imu_age << " s\n";
    os << ctx.mapper->describe();
    os << ctx.policy->describe();
    os << ctx.input->describe() << "\n";
    os << "=============================\n";
    ROS_INFO_STREAM(os.str());
}
} // namespace

bool waitUntilReady(ControlContext& ctx, const ReadinessConfig& cfg)
{
    const TimePoint t0 = Clock::now();
    const auto poll = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(cfg.poll_dt_s));

    // Named, not a temporary inside the argument list: .c_str() on a temporary
    // std::string would dangle before ROS_INFO read it.
    const std::string timeout_txt =
        cfg.timeout_s > 0.0 ? std::to_string(cfg.timeout_s) + " s" : std::string("none");
    ROS_INFO("Startup: waiting for system readiness (require_imu=%s, "
             "require_fresh_s=%.3f, timeout=%s). No command is transmitted "
             "until the FSM starts.",
             cfg.require_imu ? "true" : "false", cfg.require_fresh_s,
             timeout_txt.c_str());

    while (ros::ok())
    {
        refreshReadinessState(ctx);

        const std::string reason = readinessReason(ctx, cfg);
        if (reason.empty())
        {
            logReadinessDump(ctx);
            ROS_INFO("Startup: system READY after %.1f s.", age_seconds(t0));
            return true;
        }

        if (cfg.timeout_s > 0.0 && age_seconds(t0) > cfg.timeout_s)
        {
            ROS_FATAL("Startup: still not ready after %.1f s -- %s. "
                      "Refusing to start the FSM.", age_seconds(t0), reason.c_str());
            return false;
        }

        // Waiting is the NORMAL startup path, not a fault. This is a warning
        // only so that the reason is visible on a default console.
        ROS_WARN_THROTTLE(2.0, "Startup: waiting (%.1f s) -- %s",
                          age_seconds(t0), reason.c_str());
        std::this_thread::sleep_for(poll);
    }

    ROS_WARN("Startup: interrupted before the system became ready. "
             "The FSM was never started.");
    return false;
}

} // namespace mini_pi
