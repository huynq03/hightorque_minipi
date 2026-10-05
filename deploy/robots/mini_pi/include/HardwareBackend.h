// createHardwareBackend() -- the single point where the concrete hardware
// backend is chosen, and the ONLY translation unit in this package that names
// both backends.
//
// ============================== WHY THIS EXISTS ============================
// The real backend cannot be compiled everywhere. HighTorqueHardware.cpp needs
// livelybot_serial, which ships only as an aarch64 build for the Mini-Pi's own
// computer; there is no x86_64 version. Before this file existed, main.cpp
// constructed `HighTorqueHardware` directly, which forced every build --
// including a PC simulation build that never touches a motor -- to find and
// link that SDK.
//
// Moving the choice behind a factory means main.cpp names neither concrete
// class, so the PC build simply does not compile HighTorqueHardware.cpp and
// the link succeeds with no SDK present. The FSM, the states, JointMapper,
// Safety and InputManager are compiled from the identical sources in
// both configurations.
//
// The build knob is MINI_PI_ENABLE_REAL_HW (CMake), surfaced to the
// preprocessor as MINI_PI_WITH_REAL_HW. It gates exactly one source file and
// exactly one branch below -- nothing else in the package is conditional, and
// no FSM behaviour changes with it.
// ===========================================================================
#pragma once

#include "HardwareInterface.h"
#include "SimHardware.h"

#include <ros/ros.h>

#include <memory>
#include <array>
#include <string>
#include <vector>

namespace mini_pi
{

/// Everything main() has to hand a backend, in one struct.
///
/// The simulation half is `SimHardware::Config` itself rather than a copy of
/// its fields: SimHardware.cpp is compiled on every platform, so its header is
/// always available here and there is no reason to restate bridge_ip /
/// lowcmd_port / lowstate_port / the freshness thresholds and their defaults a
/// second time. Only HighTorqueHardware::Config has to stay out of this header
/// -- it is the one that cannot exist on a PC.
struct BackendOptions
{
    /// Shared: both backends honour dry_run, and both judge staleness by the
    /// thresholds main() takes from config/safety.yaml.
    SimHardware::Config sim;

    /// hightorque only.
    std::string imu_topic = "/imu/data";
    int motor_watchdog_ms = 0;
    std::array<double, 3> imu_mount_rpy{{0.0, 0.0, 0.0}};  ///< hightorque only
    std::string feedback_query = "state2";                 ///< hightorque only
    double feedback_query_period_s = 0.002;                ///< hightorque only
};

/// Construct and initialize the named backend, or return nullptr with `error`
/// set. Valid names are whatever availableBackends() reports for this build.
///
/// Asking for a backend that exists but was compiled out produces an explicit
/// message saying so, rather than "unknown backend" -- the difference matters
/// when someone runs the PC build on the robot by mistake.
std::unique_ptr<HardwareInterface> createHardwareBackend(const std::string& name,
                                                         ros::NodeHandle& nh,
                                                         const BackendOptions& opt,
                                                         std::string& error);

/// Backends this binary was actually built with, for diagnostics and for the
/// error message above.
std::vector<std::string> availableBackends();

} // namespace mini_pi
