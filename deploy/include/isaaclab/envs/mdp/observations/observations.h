// Registered observation terms -- THE list a deploy.yaml can name.
//
// Same style as unitree_rl_mjlab/deploy/include/isaaclab/envs/mdp/
// observations/observations.h. Each term is a function of the env and its
// params, returning unscaled, unclipped values in natural units, policy joint
// order. Scale, clip and history belong to the ObservationManager. Terms are
// named by MEANING; none knows which policy uses it. A new policy needs C++
// only for a quantity not listed here.
//
// Terms with a REGISTER_OBSERVATION_CHECK refuse (the env does not step) while
// their source is missing, as reported by the robot's articulation in
// ArticulationData; a policy that does not name the term is unaffected.
#pragma once

#include "isaaclab/envs/manager_based_rl_env.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>

namespace isaaclab
{
namespace mdp
{

/// The operator command clamped to commands.base_velocity.ranges: vx, vy, wz.
inline std::array<float, 3> clamped_command(const ManagerBasedRLEnv* env)
{
    const auto& c = env->robot->data.command;
    const auto& r = env->command_ranges;
    return {std::clamp(static_cast<float>(c.vx), r.lin_vel_x[0], r.lin_vel_x[1]),
            std::clamp(static_cast<float>(c.vy), r.lin_vel_y[0], r.lin_vel_y[1]),
            std::clamp(static_cast<float>(c.wz), r.ang_vel_z[0], r.ang_vel_z[1])};
}

/// Wrap to (-pi, pi], the range humanoid-gym's get_euler_xyz_tensor() produces.
inline float wrap_to_pi(float a)
{
    const float two_pi = 2.0f * static_cast<float>(M_PI);
    a = std::fmod(a, two_pi);
    if (a <= -static_cast<float>(M_PI)) a += two_pi;
    else if (a > static_cast<float>(M_PI)) a -= two_pi;
    return a;
}

inline std::vector<float> per_joint_param(const ManagerBasedRLEnv* env, const YAML::Node& n,
                                          const char* term, const char* key)
{
    const std::size_t dim = env->robot->data.joint_ids_map.size();
    if (!n) throw std::runtime_error(std::string(term) + ": params." + key + " is required");
    auto v = n.as<std::vector<float>>();
    if (v.size() != dim)
        throw std::runtime_error(std::string(term) + ": params." + key + " needs " +
                                 std::to_string(dim) + " values (one per policy joint)");
    return v;
}

// ---- base -------------------------------------------------------------------

/// Body-frame angular velocity [rad/s].
REGISTER_OBSERVATION(base_ang_vel)
{
    (void)params;
    const auto& v = env->robot->data.root_ang_vel_b;
    return {v[0], v[1], v[2]};
}

/// Base linear velocity, base frame [m/s]. Available only where the robot's
/// articulation reports a source (e.g. a simulator's ground truth); without one
/// a policy that names this term cannot run.
/// Params:
///   max_age_s  default 0.05 -- older counts as missing.
///   fixed      [vx, vy, vz]: feed this CONSTANT instead of the measurement and
///              skip the availability check. A deliberate, visible
///              substitution for robots with no velocity source; the policy
///              then never sees its real velocity.
REGISTER_OBSERVATION(base_lin_vel)
{
    if (params["fixed"])
    {
        const auto f = params["fixed"].as<std::vector<float>>();
        if (f.size() != 3) throw std::runtime_error("base_lin_vel: params.fixed needs 3 values");
        return f;
    }
    const auto& v = env->robot->data.base_lin_vel;
    return {v[0], v[1], v[2]};
}
REGISTER_OBSERVATION_CHECK(base_lin_vel)
{
    if (params["fixed"]) return {};
    const double max_age = params["max_age_s"] ? params["max_age_s"].as<double>() : 0.05;
    if (!data.base_lin_vel_valid) return "no valid base linear velocity source on this backend";
    const double age = data.base_lin_vel_age_s;
    if (!(age <= max_age))
        return "base linear velocity is stale (" + std::to_string(age) + " s > " +
               std::to_string(max_age) + " s)";
    return {};
}

/// Gravity direction in the base frame, R_wb^T [0, 0, -1] (IsaacLab
/// projected_gravity), from the IMU orientation after the mount transform.
REGISTER_OBSERVATION(projected_gravity)
{
    (void)params;
    const auto& g = env->robot->data.projected_gravity_b;
    return {g[0], g[1], g[2]};
}
REGISTER_OBSERVATION_CHECK(projected_gravity)
{
    (void)params;
    if (!data.imu_valid) return "IMU invalid";
    for (float g : data.projected_gravity_b)
        if (!std::isfinite(g)) return "orientation quaternion is non-finite or not unit norm";
    return {};
}

/// Roll, pitch, yaw [rad] (ZYX).
/// Params: yaw  "absolute" (default) | "relative_to_entry": yaw minus the yaw
///              of the first step after reset(), wrapped to (-pi, pi] -- for
///              policies trained with every episode starting at yaw 0.
REGISTER_OBSERVATION(base_euler)
{
    const auto& e = env->robot->data.root_euler;
    const std::string mode = params["yaw"] ? params["yaw"].as<std::string>() : "absolute";
    float yaw = e[2];
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

// ---- joints -----------------------------------------------------------------

REGISTER_OBSERVATION(joint_pos)
{
    (void)params;
    return env->robot->data.joint_pos;
}

/// q - default_joint_pos.
REGISTER_OBSERVATION(joint_pos_rel)
{
    (void)params;
    const auto& d = env->robot->data;
    std::vector<float> out(d.joint_pos.size());
    for (std::size_t i = 0; i < out.size(); ++i) out[i] = d.joint_pos[i] - d.default_joint_pos[i];
    return out;
}

REGISTER_OBSERVATION(joint_vel)
{
    (void)params;
    return env->robot->data.joint_vel;
}

/// IsaacLab's name; the default joint velocity is zero.
REGISTER_OBSERVATION(joint_vel_rel)
{
    (void)params;
    return env->robot->data.joint_vel;
}

/// ActionManager::action() of the previous step: the network output after any
/// raw_clip, before scale / offset.
REGISTER_OBSERVATION(last_action)
{
    (void)params;
    return env->action_manager->action();
}

// ---- commands and gait ------------------------------------------------------

/// vx, vy, wz clamped to commands.base_velocity.ranges.
/// Params: min_lin_norm (default 0) -- ||(vx, vy)|| below it zeroes both
///         linear components (humanoid-gym _resample_commands).
///         deadzone (default 0) -- ||(vx, vy, wz)|| at or below it zeroes the
///         whole command (mjlab UniformVelocityCommand command_deadzone).
REGISTER_OBSERVATION(velocity_commands)
{
    auto c = clamped_command(env);
    const float min_norm = params["min_lin_norm"] ? params["min_lin_norm"].as<float>() : 0.0f;
    if (std::sqrt(c[0] * c[0] + c[1] * c[1]) < min_norm) { c[0] = 0.0f; c[1] = 0.0f; }
    const float deadzone = params["deadzone"] ? params["deadzone"].as<float>() : 0.0f;
    if (deadzone > 0.0f && std::sqrt(c[0] * c[0] + c[1] * c[1] + c[2] * c[2]) <= deadzone)
        c = {0.0f, 0.0f, 0.0f};
    return {c[0], c[1], c[2]};
}

/// [sin, cos](2*pi*phase). Sample THEN advance by step_dt / period, so the
/// first frame after reset() has phase 0 (NOTE: Unitree advances first).
/// Params: period [s] (required), zero_on_stand (default false: both outputs
///         0 while ||clamped cmd|| <= stand_threshold, default 0.1 -- mjlab's
///         phase observation). The clock keeps running while zeroed.
REGISTER_OBSERVATION(gait_phase)
{
    if (!params["period"])
        throw std::runtime_error("observation term gait_phase: params.period is required "
                                 "(seconds). It is intentionally not defaulted.");
    const float period = params["period"].as<float>();
    if (!(period > 0.0f))
        throw std::runtime_error("observation term gait_phase: params.period must be > 0");

    std::vector<float> obs(2);
    obs[0] = std::sin(env->global_phase * 2.0f * static_cast<float>(M_PI));
    obs[1] = std::cos(env->global_phase * 2.0f * static_cast<float>(M_PI));
    env->global_phase += static_cast<float>(env->step_dt) / period;
    env->global_phase = std::fmod(env->global_phase, 1.0f);

    if (params["zero_on_stand"] && params["zero_on_stand"].as<bool>())
    {
        const float thr = params["stand_threshold"] ? params["stand_threshold"].as<float>() : 0.1f;
        const auto c = clamped_command(env);
        if (std::sqrt(c[0] * c[0] + c[1] * c[1] + c[2] * c[2]) <= thr) { obs[0] = 0.0f; obs[1] = 0.0f; }
    }
    return obs;
}

/// Periodic reference joint pose (humanoid-gym "ref_dof_pos"), one block of
/// one value per policy joint for each entry of `time_offsets`:
///
///   t   = steps since reset * step_dt + time_offset
///   s   = sin(2*pi*t / period)
///   A   = amplitude * max(|vx|, |wz|)          clamped command
///   ref = 0                                     if |s| <= deadband
///   ref[i] = left_coeffs[i] * min(s,0) * A + right_coeffs[i] * max(s,0) * A
///
/// Params: period [s], amplitude, left_coeffs, right_coeffs (required);
///         deadband (default 0), time_offsets [s] (default [0]).
/// The float/double conversions reproduce the vendor reference
/// computeRefDofPos bit for bit (see doc/policy_format.md).
REGISTER_OBSERVATION(gait_joint_reference)
{
    for (const char* k : {"period", "amplitude"})
        if (!params[k])
            throw std::runtime_error(std::string("gait_joint_reference: params.") + k +
                                     " is required");
    const float period = params["period"].as<float>();
    if (!(period > 0.0f)) throw std::runtime_error("gait_joint_reference: period must be > 0");
    const float amplitude = params["amplitude"].as<float>();
    const double deadband = params["deadband"] ? params["deadband"].as<double>() : 0.0;
    const auto left = per_joint_param(env, params["left_coeffs"], "gait_joint_reference", "left_coeffs");
    const auto right = per_joint_param(env, params["right_coeffs"], "gait_joint_reference", "right_coeffs");
    const auto offsets = params["time_offsets"] ? params["time_offsets"].as<std::vector<double>>()
                                                : std::vector<double>{0.0};

    const auto c = clamped_command(env);
    const float a = amplitude * std::max(std::fabs(c[0]), std::fabs(c[2]));
    const float t0 = static_cast<float>(static_cast<double>(env->episode_length) * env->step_dt);
    const double two_pi = 6.283185307179586;

    std::vector<float> out;
    out.reserve(offsets.size() * left.size());
    for (double off : offsets)
    {
        const float t = static_cast<float>(static_cast<double>(t0) + off);
        const float s = static_cast<float>(
            std::sin(static_cast<double>(t) * two_pi / static_cast<double>(period)));
        const bool active = static_cast<double>(std::fabs(s)) > deadband;
        const float l = (s > 0.0f ? 0.0f : s) * a;
        const float r = (s < 0.0f ? 0.0f : s) * a;
        for (std::size_t i = 0; i < left.size(); ++i)
            out.push_back(active ? left[i] * l + right[i] * r : 0.0f);
    }
    return out;
}

} // namespace mdp
} // namespace isaaclab
