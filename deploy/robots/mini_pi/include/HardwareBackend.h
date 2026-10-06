// createHardwareBackend() -- the Mini-Pi's RobotApp::create_backend, and the
// ONLY translation unit that names the real (HighTorque) backend. `sim` is the
// shared simulated backend, built by deploy::runDeployNode().
//
// ============================== WHY THIS EXISTS ============================
// The real backend cannot be compiled everywhere. HighTorqueHardware.cpp needs
// livelybot_serial, which ships only as an aarch64 build for the Mini-Pi's own
// computer; there is no x86_64 version. Keeping the choice behind this factory
// means the shared node (deploy/src/app/DeployApp.cpp) names no concrete
// class, so the PC build simply does not compile HighTorqueHardware.cpp and
// the link succeeds with no SDK present.
//
// The build knob is MINI_PI_ENABLE_REAL_HW (CMake), surfaced to the
// preprocessor as MINI_PI_WITH_REAL_HW. It gates exactly one source file and
// exactly one branch in HardwareBackend.cpp -- nothing else is conditional,
// and no FSM behaviour changes with it.
// ===========================================================================
#pragma once

#include "app/DeployApp.h"

#include <memory>
#include <string>
#include <vector>

namespace mini_pi
{

/// Construct and initialize the requested real backend ("hightorque"), or
/// return nullptr with `error` set. Reads the Mini-Pi keys of robot.yaml
/// (imu_topic, imu.mount_rpy, motor_watchdog_ms, hightorque:) and refuses
/// "hightorque" while a vendor process owns the motor serial link.
///
/// Asking for a backend that exists but was compiled out produces an explicit
/// message saying so, rather than "unknown backend" -- the difference matters
/// when someone runs the PC build on the robot by mistake.
std::unique_ptr<deploy::HardwareInterface> createHardwareBackend(
    const deploy::BackendRequest& req, ros::NodeHandle& nh, std::string& error);

/// Backends this binary was actually built with.
std::vector<std::string> availableBackends();

} // namespace mini_pi
