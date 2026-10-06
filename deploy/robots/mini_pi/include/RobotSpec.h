// RobotSpec -- the compile-time facts the shared deploy code needs from a robot.
//
// Counterpart of unitree_rl_mjlab/deploy/robots/<robot>/include/Types.h: the
// shared headers in deploy/include include "RobotSpec.h" by bare name, and each
// robot's CMakeLists puts its own include/ on the path, so the same shared
// source compiles against whichever robot is being built.
//
// Plain C++, no ROS, no SDK: the simulator includes it too (through
// hardware/LowLevelBus.h).
#pragma once

#include <cstddef>

namespace robot_spec
{

/// Human-readable name for logs and the diagnostic dump.
constexpr const char* kName = "Mini-Pi";

/// Actuated joints == motors on the low-level bus (12dof lubancat build).
/// Evidence: sim2real/robot_param/12dof_STM32H730_pi_lubancat_params.yaml
/// (1 CAN board x 2 CAN ports x 6 motors) and sim2real/config/pi_pd_config.yaml
/// (`dofs: 12`).
constexpr std::size_t kNumDof = 12;

} // namespace robot_spec
