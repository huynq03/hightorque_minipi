// rl::mdp -- the registered observation terms.
//
// Structural counterpart of unitree_rl_mjlab/deploy/include/isaaclab/envs/mdp/
// observations/observations.h.
//
// Each term is a PURE FUNCTION of the environment: it returns unscaled,
// unclipped values in the term's natural units. Scaling, clipping, history and
// ordering are all the ObservationManager's job, driven by deploy.yaml. Adding
// a scale constant to a function here would make it unconfigurable and is a
// bug, not a shortcut.
//
// Every joint-space term is in POLICY JOINT ORDER, because RobotView::data
// already is.
#pragma once

#include "rl/ObservationManager.h"
#include "rl/PolicyEnvironment.h"
#include "rl/RobotView.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace mini_pi
{
namespace rl
{
namespace mdp
{

/// Wrap an angle to (-pi, pi], the range humanoid-gym's get_euler_xyz_tensor()
/// produces (angle in [0, 2pi), minus 2pi when > pi).
inline float wrap_to_pi(float a)
{
    const float two_pi = 2.0f * static_cast<float>(M_PI);
    a = std::fmod(a, two_pi);
    if (a <= -static_cast<float>(M_PI)) a += two_pi;
    else if (a > static_cast<float>(M_PI)) a -= two_pi;
    return a;
}

/// Body-frame angular velocity, 3 values [rad/s].
MINI_PI_REGISTER_OBSERVATION(base_ang_vel)
{
    (void)params;
    const auto& v = env->robot->data.root_ang_vel_b;
    return {v[0], v[1], v[2]};
}

/// Body orientation as ZYX euler angles, 3 values [rad]: roll, pitch, yaw.
///
/// NOTE this is NOT Unitree's `projected_gravity`. The Mini-Pi baseline
/// observation (task §10, slots 44:47) is euler, and the two are not
/// interchangeable: projected gravity discards yaw, euler does not.
/// `RobotState::rpy` is produced by the single shared quat_xyzw_to_rpy() in
/// Types.h, so sim and hardware cannot disagree about it.
///
/// Params:
///   yaw   "absolute" (default) or "relative_to_entry".
///         relative_to_entry reports yaw minus the yaw seen on the first policy
///         step after reset(), wrapped to (-pi, pi]. This reproduces the
///         training distribution of Mini_Pi_RL_Baseline: every episode spawns
///         with init_state.rot = identity (legged_robot_config.py), so the
///         observed yaw starts at 0 and then accumulates from there.
///         A real IMU heading is arbitrary (magnetic / power-on), which
///         training never produced at episode start.
MINI_PI_REGISTER_OBSERVATION(base_euler)
{
    const auto& e = env->robot->data.root_euler;
    float yaw = e[2];
    const std::string mode = params["yaw"] ? params["yaw"].as<std::string>()
                                           : std::string("absolute");
    if (mode == "relative_to_entry")
    {
        if (!env->yaw_origin_set)
        {
            env->yaw_origin = e[2];
            env->yaw_origin_set = true;
        }
        yaw = wrap_to_pi(e[2] - env->yaw_origin);
    }
    else if (mode != "absolute")
    {
        throw std::runtime_error("observation term base_euler: params.yaw must be "
                                 "'absolute' or 'relative_to_entry', got '" + mode + "'");
    }
    return {e[0], e[1], yaw};
}

/// Measured joint position, policy order. One value per policy joint.
MINI_PI_REGISTER_OBSERVATION(joint_pos)
{
    (void)params;
    return env->robot->data.joint_pos;
}

/// Measured joint position relative to the policy's default pose.
///     q[i] - default_joint_pos[i]
/// `default_joint_pos` comes from deploy.yaml and is NOT assumed to be zero,
/// even though the current baseline happens to use zero (task §10).
MINI_PI_REGISTER_OBSERVATION(joint_pos_rel)
{
    (void)params;
    const auto& d = env->robot->data;
    std::vector<float> out(d.joint_pos.size());
    for (std::size_t i = 0; i < out.size(); ++i) out[i] = d.joint_pos[i] - d.default_joint_pos[i];
    return out;
}

/// Measured joint velocity, policy order.
MINI_PI_REGISTER_OBSERVATION(joint_vel)
{
    (void)params;
    return env->robot->data.joint_vel;
}

/// Alias of joint_vel. IsaacLab calls it `joint_vel_rel` because it subtracts
/// a default velocity, which is always zero for a locomotion policy. Provided
/// so a deploy.yaml written against Unitree's term names loads unchanged.
MINI_PI_REGISTER_OBSERVATION(joint_vel_rel)
{
    (void)params;
    return env->robot->data.joint_vel;
}

/// The network output of the previous step AFTER the raw action clip and
/// BEFORE scale/offset, policy order. Training fed back exactly this:
/// LeggedRobot.step() stores self.actions = clip(actions, +-clip_actions) and
/// PaiFreeEnv.compute_observations() observes self.actions.
MINI_PI_REGISTER_OBSERVATION(last_action)
{
    (void)params;
    return env->action_manager->action();
}

/// Operator velocity command, 3 values: vx, vy, yaw rate, in SI units.
///
/// The per-axis limits come from the policy package
/// (`commands.base_velocity.ranges`) and are applied HERE, as a clamp on the
/// operator's intent. The observation SCALE is applied afterwards by the
/// ObservationManager from `scale:`. Keeping the two separate is task §22:
/// a range is what the robot is allowed to be asked for, a scale is how the
/// network was trained to read it.
///
/// Params:
///   min_lin_norm  Optional, default 0 (off). When ||(vx, vy)|| is below it,
///                 both linear components are set to 0 (yaw rate untouched).
///                 This is humanoid-gym's _resample_commands() rule
///                 (`commands[:, :2] *= norm(commands[:, :2]) > 0.2`): the
///                 trained policy never saw a linear command in (0, 0.2) m/s.
MINI_PI_REGISTER_OBSERVATION(velocity_commands)
{
    const VelocityCommand& c = env->robot->data.command;
    const auto& r = env->command_ranges;
    float vx = std::min(std::max(static_cast<float>(c.vx), r.lin_vel_x[0]), r.lin_vel_x[1]);
    float vy = std::min(std::max(static_cast<float>(c.vy), r.lin_vel_y[0]), r.lin_vel_y[1]);
    const float wz = std::min(std::max(static_cast<float>(c.dyaw), r.ang_vel_z[0]),
                              r.ang_vel_z[1]);
    const float min_norm = params["min_lin_norm"] ? params["min_lin_norm"].as<float>() : 0.0f;
    if (std::sqrt(vx * vx + vy * vy) < min_norm) { vx = 0.0f; vy = 0.0f; }
    return {vx, vy, wz};
}

/// Gait clock, 2 values: [sin(2*pi*phase), cos(2*pi*phase)].
///
/// The phase is advanced by `step_dt / period` on every call, so it is a
/// function of the STEP COUNT and is bit-for-bit reproducible across runs.
/// That is deliberate (task §16): the HighTorque implementation advances the
/// phase from ros::Time::now(), which makes a replay unreproducible.
///
/// Params:
///   period        [s]   REQUIRED. Gait cycle duration. No default: it is a
///                       property of the trained policy (PaiCfg.rewards.
///                       cycle_time), not of this code.
///   zero_on_stand bool  Optional, default false. When true, both outputs are
///                       forced to 0 while ||cmd|| < stand_threshold. This is
///                       Unitree's behaviour; humanoid-gym does not do it.
///   stand_threshold     Optional, default 0.1.
///
/// The phase advances even while zeroed, so the clock does not drift.
MINI_PI_REGISTER_OBSERVATION(gait_phase)
{
    if (!params["period"])
        throw std::runtime_error("observation term gait_phase: params.period is required "
                                 "(seconds). It is intentionally not defaulted.");
    const float period = params["period"].as<float>();
    if (!(period > 0.0f))
        throw std::runtime_error("observation term gait_phase: params.period must be > 0");

    // Sample THEN advance: the first frame after reset() has phase 0, as in
    // training, where compute_observations() runs right after reset_idx() set
    // episode_length_buf = 0 (phase = episode_length_buf * dt / cycle_time).
    std::vector<float> obs(2);
    obs[0] = std::sin(env->global_phase * 2.0f * static_cast<float>(M_PI));
    obs[1] = std::cos(env->global_phase * 2.0f * static_cast<float>(M_PI));

    env->global_phase += env->step_dt / period;
    env->global_phase = std::fmod(env->global_phase, 1.0f);

    if (params["zero_on_stand"] && params["zero_on_stand"].as<bool>())
    {
        const float thr = params["stand_threshold"] ? params["stand_threshold"].as<float>() : 0.1f;
        const auto& c = env->robot->data.command;
        const float n = std::sqrt(static_cast<float>(c.vx * c.vx + c.vy * c.vy + c.dyaw * c.dyaw));
        if (n < thr) { obs[0] = 0.0f; obs[1] = 0.0f; }
    }
    return obs;
}

} // namespace mdp
} // namespace rl
} // namespace mini_pi
