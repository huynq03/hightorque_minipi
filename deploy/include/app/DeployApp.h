// DeployApp -- the shared body of every robot's deploy node.
//
// Counterpart of unitree_rl_mjlab/deploy/robots/<robot>/main.cpp, with the
// part those files repeat per robot (config loading, policy, input, readiness,
// FSM start, diagnostics, shutdown) written once here. A robot's main.cpp only
// fills a RobotApp -- its ROS package and how to build its hardware backends --
// and calls runDeployNode().
//
// Robot-specific code therefore lives in exactly two places:
//   deploy/robots/<robot>/include/RobotSpec.h   dof count and name
//   deploy/robots/<robot>/{main.cpp, src/}       backend factory + real backend
#pragma once

#include "hardware/HardwareInterface.h"

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <functional>
#include <memory>
#include <string>

namespace deploy
{

/// Everything the shared node knows when it asks the robot for a backend.
struct BackendRequest
{
    /// robot.yaml `backend`, overridden by the `~backend` param.
    std::string name;
    /// The whole robot.yaml, for the robot's own backend keys.
    YAML::Node robot_cfg;
    /// robot.yaml `dry_run`, overridden by `~dry_run`. Every backend honours it.
    bool dry_run = true;
    /// From safety.yaml, so the hardware layer and Safety agree on "stale".
    double motor_timeout_s = 0.1;
    double imu_timeout_s = 0.2;
};

/// Construct and initialize the requested REAL backend, or return nullptr with
/// `error` set. Owned by the robot: it is the only code that names a concrete
/// real backend. `backend: sim` never reaches it -- runDeployNode() builds the
/// shared simulated backend itself.
using BackendFactory = std::function<std::unique_ptr<HardwareInterface>(
    const BackendRequest& req, ros::NodeHandle& nh, std::string& error)>;

struct RobotApp
{
    /// catkin package holding config/ and launch/; default config paths and a
    /// relative policy_dir resolve against its share directory.
    std::string ros_package;
    std::string node_name;
    /// Used when robot.yaml has no `backend` key.
    std::string default_backend = "sim";
    BackendFactory create_backend;
};

/// Runs the node until ROS shuts down. Returns the process exit code.
int runDeployNode(int argc, char** argv, const RobotApp& app);

} // namespace deploy
