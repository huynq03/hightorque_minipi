#include "hardware/HardwareBackend.h"

#include <ros/ros.h>

// The one conditional include in the package. MINI_PI_WITH_REAL_HW is defined
// by deploy/CMakeLists.txt when MINI_PI_ENABLE_REAL_HW is ON, which is the
// default and is what the Mini-Pi itself builds with.
#if defined(MINI_PI_WITH_REAL_HW)
#include "hardware/HighTorqueHardware.h"
#endif

namespace mini_pi
{

std::vector<std::string> availableBackends()
{
    std::vector<std::string> v;
#if defined(MINI_PI_WITH_REAL_HW)
    v.push_back("hightorque");
#endif
    v.push_back("sim");
    return v;
}

std::unique_ptr<HardwareInterface> createHardwareBackend(const std::string& name,
                                                         ros::NodeHandle& nh,
                                                         const BackendOptions& opt,
                                                         std::string& error)
{
    error.clear();

    if (name == "hightorque")
    {
#if defined(MINI_PI_WITH_REAL_HW)
        auto hw = std::make_unique<HighTorqueHardware>();
        HighTorqueHardware::Config cfg;
        cfg.dry_run           = opt.sim.dry_run;
        cfg.imu_topic         = opt.imu_topic;
        cfg.motor_watchdog_ms = opt.motor_watchdog_ms;
        cfg.imu_mount_rpy     = opt.imu_mount_rpy;
        cfg.feedback_query    = opt.feedback_query;
        cfg.feedback_query_period_s = opt.feedback_query_period_s;
        cfg.motor_timeout_s   = opt.sim.link.motor_timeout_s;
        cfg.imu_timeout_s     = opt.sim.link.imu_timeout_s;
        if (!hw->initialize(nh, cfg))
        {
            error = "HighTorqueHardware::initialize() failed";
            return nullptr;
        }
        return hw;
#else
        error = "this binary was built with MINI_PI_ENABLE_REAL_HW=OFF, so the "
                "'hightorque' backend is not present. It needs livelybot_serial, "
                "which exists only as an aarch64 build for the Mini-Pi. Rebuild on "
                "the robot with -DMINI_PI_ENABLE_REAL_HW=ON, or use backend:=sim.";
        return nullptr;
#endif
    }

    if (name == "sim")
    {
        // The warning lives here, not in main(), so that main() really does
        // name no backend: adding a third one should mean editing this file
        // and no other.
        ROS_WARN("backend=sim: this process drives a MuJoCo simulation, not the robot. "
                 "The FSM, JointMapper and SafetyManager are the same code either way.");

        auto hw = std::make_unique<SimHardware>();
        // Unused by this backend and therefore not forwarded: imu_topic (the
        // IMU rides the low-level bus, as it does in Unitree's LowState) and
        // motor_watchdog_ms (a firmware register; the simulated equivalent is
        // command_timeout_s in simulate/config.yaml, on the simulator side).
        (void) nh;
        if (!hw->initialize(opt.sim))
        {
            error = "SimHardware::initialize() failed";
            return nullptr;
        }
        return hw;
    }

    error = "unknown backend '" + name + "'. This binary provides:";
    for (const std::string& b : availableBackends()) error += " " + b;
    return nullptr;
}

} // namespace mini_pi
