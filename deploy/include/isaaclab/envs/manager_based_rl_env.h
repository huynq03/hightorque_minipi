// ManagerBasedRLEnv -- one policy package, fully described by its deploy.yaml.
//
// Counterpart of unitree_rl_mjlab/deploy/include/isaaclab/envs/
// manager_based_rl_env.h, with the same step():
//
//     robot->update();                                   Articulation
//     obs = observation_manager->compute();              registered terms
//     action = alg->act(obs);                            OrtRunner
//     action_manager->process_action(action);            registered action terms
//
// The processed targets (policy order and coordinates) are read with
// action_manager->processed_actions(); the robot package maps them back to its
// own command through its Articulation. The gains are
// robot->data.joint_stiffness / joint_damping. Nothing here knows a policy
// name, a dimension, a robot, its middleware or its hardware. It owns no
// thread: the robot package calls step() every step_dt.
//
// deploy.yaml keys read here (Unitree's, except joint_names):
//   step_dt                        [s]
//   joint_names                    POLICY joint order, resolved by name against
//                                  robot->robot_joint_names (Unitree: joint_ids_map)
//   default_joint_pos, stiffness, damping   per policy joint (scalar broadcast)
//   commands.base_velocity.ranges  lin_vel_x / lin_vel_y / ang_vel_z [min, max]
//   calibration.joint_offset       optional, see Articulation (joint_pos_shift)
//   observations, actions          see the two managers
#pragma once

#include "isaaclab/algorithms/algorithms.h"
#include "isaaclab/assets/articulation/articulation.h"
#include "isaaclab/manager/action_manager.h"
#include "isaaclab/manager/observation_manager.h"

#include <yaml-cpp/yaml.h>

#include <array>
#include <memory>
#include <string>

namespace isaaclab
{

/// deploy.yaml `commands.base_velocity.ranges`.
struct CommandRanges
{
    std::array<float, 2> lin_vel_x{{-1.0f, 1.0f}};
    std::array<float, 2> lin_vel_y{{-1.0f, 1.0f}};
    std::array<float, 2> ang_vel_z{{-1.0f, 1.0f}};
};

class ManagerBasedRLEnv
{
public:
    /// Throws std::runtime_error with a specific message on any configuration
    /// error. The model is attached afterwards with set_algorithm().
    ManagerBasedRLEnv(const YAML::Node& cfg, std::shared_ptr<Articulation> robot);

    /// Checks the model's declared inputs / output against the observation
    /// groups (bound by name) and the action width, then takes it. Throws on a
    /// mismatch.
    void set_algorithm(std::unique_ptr<Algorithms> alg);

    /// Zero step count, gait phase, yaw origin; reset both managers.
    void reset();

    /// One policy step. Throws -- before anything advances -- when an
    /// observation term's source is unavailable, and on any inference or
    /// dimension error.
    void step();

    std::string describe() const;

    double step_dt = 0.02;
    YAML::Node cfg;
    CommandRanges command_ranges;

    std::unique_ptr<ObservationManager> observation_manager;
    std::unique_ptr<ActionManager> action_manager;
    std::shared_ptr<Articulation> robot;
    std::unique_ptr<Algorithms> alg;

    // ---- state the observation terms read and advance ---------------------
    long episode_length = 0;      ///< steps completed since reset()
    float global_phase = 0.0f;    ///< gait_phase clock, [0, 1)
    float yaw_origin = 0.0f;      ///< base_euler yaw: relative_to_entry
    bool yaw_origin_set = false;
};

} // namespace isaaclab
