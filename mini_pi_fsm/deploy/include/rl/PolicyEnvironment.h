// rl::PolicyEnvironment -- the Mini-Pi ManagerBasedRLEnv.
//
// Structural counterpart of unitree_rl_mjlab/deploy/include/isaaclab/envs/
// manager_based_rl_env.h. Owns the policy configuration, the two managers, the
// model and the policy-side robot view, and nothing else.
//
// ============================== WHAT IT IS NOT ==============================
// It does NOT touch hardware. It does NOT own a thread. It does NOT know about
// the FSM, SafetyManager, JointMapper or the UDP bus.
//
// step() takes one state snapshot plus one operator command and returns one
// RobotCommand in ROBOT JOINT ORDER -- the same struct every other FSM state
// produces, so RL output travels the identical SafetyManager -> JointMapper ->
// HardwareInterface path. That is what makes "RL cannot bypass safety" a
// structural property rather than a promise.
// ============================================================================
#pragma once

#include "control/Types.h"
#include "rl/ActionManager.h"
#include "rl/ModelRunner.h"
#include "rl/ObservationManager.h"
#include "rl/RobotView.h"

#include <yaml-cpp/yaml.h>

#include <atomic>
#include <memory>
#include <string>
#include <vector>

namespace mini_pi
{
namespace rl
{

/// deploy.yaml `commands.base_velocity.ranges`. Applied by the
/// `velocity_commands` observation term.
struct CommandRanges
{
    std::array<float, 2> lin_vel_x{{-1.0f, 1.0f}};
    std::array<float, 2> lin_vel_y{{-1.0f, 1.0f}};
    std::array<float, 2> ang_vel_z{{-1.0f, 1.0f}};
};

class PolicyEnvironment
{
public:
    /// `cfg`   the parsed deploy.yaml.
    /// `robot_joint_names` mapping.yaml `joint_names` (robot/URDF order).
    ///
    /// Builds RobotView, ObservationManager and ActionManager. Throws
    /// std::runtime_error with a specific message on any configuration error.
    /// The model is attached afterwards with setModel(), so a configuration
    /// can be validated without an .onnx file present (which is what the
    /// config-only unit tests do).
    PolicyEnvironment(const YAML::Node& cfg,
                      const std::vector<std::string>& robot_joint_names);

    /// Attaches the inference backend and cross-checks its declared shapes
    /// against the configured observation groups and action width.
    /// Throws std::runtime_error on a mismatch.
    void setModel(std::unique_ptr<ModelRunner> model);
    bool hasModel() const { return model_ != nullptr; }

    /// Zero history, zero raw action, phase 0, step counter 0.
    void reset();

    /// One policy step:
    ///     robot->update(state, cmd)
    ///  -> observation_manager->compute()
    ///  -> model->act()
    ///  -> action_manager->process_action()
    ///  -> fill `out` in ROBOT joint order with q / dq=0 / tau=0 / kp / kd
    ///
    /// `out.valid`, `out.stamp` and `out.seq` are NOT set here -- stamping is
    /// the FSM's business (FSMState::stampValid), and a policy thread must not
    /// be able to make a command look fresh on its own.
    ///
    /// Throws std::runtime_error if no model is attached, or on any inference
    /// or dimension failure. The caller decides what a throw means.
    void step(const RobotState& state, const VelocityCommand& cmd, RobotCommand& out);

    /// Joint targets of the last step, robot order. Valid after step().
    const JointArray& lastTargetRobot() const { return last_target_; }

    /// ONNX Runtime latency only (observation/action processing excluded).
    /// These atomics are read by the low-rate status reporter while the policy
    /// thread is running; the accumulation itself has one writer.
    double meanInferenceSeconds() const { return mean_inference_s_.load(); }
    double maxInferenceSeconds() const { return max_inference_s_.load(); }

    float step_dt = 0.02f;
    /// Advanced by the gait_phase term; part of the environment, not of the
    /// term, exactly as in Unitree.
    float global_phase = 0.0f;
    long episode_length = 0;

    /// Heading captured on the first policy step after reset(), used by the
    /// base_euler term when `yaw: relative_to_entry`. Cleared by reset().
    float yaw_origin = 0.0f;
    bool yaw_origin_set = false;

    YAML::Node cfg;
    CommandRanges command_ranges;

    std::shared_ptr<RobotView> robot;
    std::unique_ptr<ObservationManager> observation_manager;
    std::unique_ptr<ActionManager> action_manager;

    /// Policy-order PD gains from deploy.yaml. Written into every RobotCommand
    /// this environment produces, so RL gains live in the policy package and
    /// never in fsm.yaml or in State_RLBase (task §15).
    std::vector<float> stiffness;
    std::vector<float> damping;

    std::string describe() const;

private:
    std::unique_ptr<ModelRunner> model_;
    JointArray last_target_{};
    JointArray kp_robot_{};
    JointArray kd_robot_{};
    std::uint64_t inference_count_ = 0;
    double inference_total_s_ = 0.0;
    std::atomic<double> mean_inference_s_{0.0};
    std::atomic<double> max_inference_s_{0.0};
};

} // namespace rl
} // namespace mini_pi
