#include "HardwareBackend.h"
#include "VendorConflict.h"

#include <sstream>

#include <unistd.h>

// The one conditional include in the package. MINI_PI_WITH_REAL_HW is defined
// by deploy/robots/mini_pi/CMakeLists.txt when MINI_PI_ENABLE_REAL_HW is ON, which is the
// default and is what the Mini-Pi itself builds with.
#if defined(MINI_PI_WITH_REAL_HW)
#include "HighTorqueHardware.h"
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

namespace
{
std::unique_ptr<deploy::HardwareInterface> createHighTorque(const deploy::BackendRequest& req,
                                                            ros::NodeHandle& nh,
                                                            std::string& error)
{
    // One owner of the motor bus. The vendor stack auto-starts at login on the
    // Mini-Pi and respawns; refuse rather than share the serial link with it.
    // Applies to dry_run too: a second reader steals our feedback replies.
    const auto conflicts = findVendorConflicts("/proc", static_cast<int>(::getpid()));
    if (!conflicts.empty())
    {
        std::ostringstream os;
        os << "the HighTorque vendor controller (or another motor-serial owner) is running.";
        for (const auto& c : conflicts) os << "\n    pid " << c.pid << ": " << c.what;
        os << "\n  Stop it first (see scripts/check-vendor-conflict.sh), then relaunch.";
        error = os.str();
        return nullptr;
    }

#if defined(MINI_PI_WITH_REAL_HW)
    const YAML::Node& rc = req.robot_cfg;
    HighTorqueHardware::Config cfg;
    cfg.dry_run         = req.dry_run;
    cfg.motor_timeout_s = req.motor_timeout_s;
    cfg.imu_timeout_s   = req.imu_timeout_s;
    if (rc["imu_topic"]) cfg.imu_topic = rc["imu_topic"].as<std::string>();
    if (rc["motor_watchdog_ms"]) cfg.motor_watchdog_ms = rc["motor_watchdog_ms"].as<int>();
    if (const YAML::Node im = rc["imu"])
    {
        if (im["mount_rpy"])
        {
            const auto v = im["mount_rpy"].as<std::vector<double>>();
            if (v.size() != 3)
            {
                error = "robot.yaml imu.mount_rpy must have 3 values [roll, pitch, yaw]";
                return nullptr;
            }
            cfg.imu_mount_rpy = {v[0], v[1], v[2]};
        }
    }
    if (const YAML::Node hw = rc["hightorque"])
    {
        if (hw["feedback_query"]) cfg.feedback_query = hw["feedback_query"].as<std::string>();
        if (hw["feedback_query_period_s"])
            cfg.feedback_query_period_s = hw["feedback_query_period_s"].as<double>();
    }

    auto hw = std::make_unique<HighTorqueHardware>();
    if (!hw->initialize(nh, cfg))
    {
        error = "HighTorqueHardware::initialize() failed";
        return nullptr;
    }
    return hw;
#else
    (void) req;
    (void) nh;
    error = "this binary was built with MINI_PI_ENABLE_REAL_HW=OFF, so the "
            "'hightorque' backend is not present. It needs livelybot_serial, "
            "which exists only as an aarch64 build for the Mini-Pi. Rebuild on "
            "the robot with -DMINI_PI_ENABLE_REAL_HW=ON, or use backend:=sim.";
    return nullptr;
#endif
}
} // namespace

std::unique_ptr<deploy::HardwareInterface> createHardwareBackend(
    const deploy::BackendRequest& req, ros::NodeHandle& nh, std::string& error)
{
    error.clear();

    if (req.name == "hightorque") return createHighTorque(req, nh, error);

    // `sim` never gets here: the shared node builds it (deploy::RobotApp).
    error = "unknown backend '" + req.name + "'. This binary provides:";
    for (const std::string& b : availableBackends()) error += " " + b;
    return nullptr;
}

} // namespace mini_pi
