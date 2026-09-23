// Non-hardware tests for the RL deployment layer.
//
//   argv[1]  deploy/config          (mapping.yaml, policy.yaml, fsm.yaml)
//   argv[2]  deploy/test/fixtures   (the generated TEST policy packages)
//
// Needs no ROS master, no hardware and no simulator. Covers, in order:
//   - the policy package / version resolver, including every failure mode
//   - the exact 47-value observation frame, index by index
//   - the exact 15 x 47 = 705 history, including the oldest->newest order
//   - the action pipeline: clip -> scale -> offset, and policy<->robot order
//   - ONNX Runtime: shapes, determinism, finiteness
//   - the whole state -> obs -> model -> action -> RobotCommand path
//
// These are value assertions, not smoke tests: an observation that is merely
// the right SIZE is not the right observation.
#include "control/JointMapper.h"
#include "policy/PolicyRunner.h"
#include "rl/PolicyEnvironment.h"
#include "rl/PolicyPackage.h"
#include "rl/mdp/JointActions.h"
#include "rl/mdp/Observations.h"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

using namespace mini_pi;
namespace fs = std::filesystem;

namespace
{
int g_failures = 0, g_checks = 0;

void check(bool cond, const std::string& what)
{
    ++g_checks;
    if (!cond) { std::printf("  FAIL %s\n", what.c_str()); ++g_failures; }
}

void check_close(double got, double want, double tol, const std::string& what)
{
    ++g_checks;
    if (!(std::fabs(got - want) <= tol))
    {
        std::printf("  FAIL %-58s got %.9f want %.9f (tol %g)\n",
                    what.c_str(), got, want, tol);
        ++g_failures;
    }
}

// Must match scripts/generate-test-policy.py.
constexpr float kSumWeight = 1.0e-3f;
constexpr float kBiasStep  = 1.0e-2f;
constexpr std::size_t kObsDim = 705;
constexpr std::size_t kActDim = 12;
constexpr std::size_t kFrameDim = 47;
constexpr std::size_t kHistory = 15;

/// A state with distinct, recognisable values in every field, so that a
/// mis-indexed observation cannot accidentally look correct.
RobotState synthetic_state()
{
    RobotState s;
    const auto now = Clock::now();
    for (std::size_t j = 0; j < MINI_PI_DOF; ++j)
    {
        // robot order. 0.01, 0.02, ... 0.12 -- every joint distinguishable.
        s.robot_q[j]  = 0.01 * static_cast<double>(j + 1);
        s.robot_dq[j] = 0.10 * static_cast<double>(j + 1);
        s.robot_tau[j] = 0.0;
        s.motor_fresh[j] = true;
        s.motor_age[j] = 0.0;
        s.motor_fault[j] = 0;
    }
    s.angular_velocity = {0.31, 0.32, 0.33};
    s.rpy = {0.41, 0.42, 0.43};
    s.orientation = {0.0, 0.0, 0.0, 1.0};
    s.stale_motor_count = 0;
    s.motor_valid = true;
    s.imu_valid = true;
    s.motor_stamp = now;
    s.imu_stamp = now;
    s.imu_age = 0.0;
    return s;
}

std::vector<std::string> robot_joint_names(const JointMapper& m) { return m.jointNames(); }

// ---------------------------------------------------------------------------
// §23 policy package / version resolver
// ---------------------------------------------------------------------------
void test_resolver(const std::string& fixtures)
{
    std::printf("[policy package resolver]\n");

    const std::string velocity = fixtures + "/policy/velocity";

    // -- direct package ----------------------------------------------------
    {
        auto p = rl::resolvePolicyPackage(velocity + "/v0", "", "");
        check(p.valid, "a directory that IS a package resolves directly");
        check(p.version == "v0", "version is the directory name");
        check(fs::is_regular_file(p.deploy_yaml), "deploy_yaml path exists");
        check(fs::is_regular_file(p.model), "model path exists");
    }

    // -- versioned directory, newest wins ----------------------------------
    {
        auto p = rl::resolvePolicyPackage(velocity, "", "");
        check(p.valid, "a directory OF packages resolves");
        check(p.version == "v0", "the only version is selected");
    }

    // -- relative path against a base --------------------------------------
    {
        auto p = rl::resolvePolicyPackage("policy/velocity", fixtures, "");
        check(p.valid, "a relative policy_dir resolves against base_dir");
    }

    // -- explicit version --------------------------------------------------
    {
        auto p = rl::resolvePolicyPackage(velocity, "", "v0");
        check(p.valid, "an explicit version resolves");
        check(p.version == "v0", "the explicit version is the one selected");

        auto bad = rl::resolvePolicyPackage(velocity, "", "v99");
        check(!bad.valid, "a non-existent explicit version is refused");
        check(bad.error.find("v99") != std::string::npos, "the error names the version");
    }

    // -- synthetic failure modes, in a scratch tree ------------------------
    const fs::path tmp = fs::temp_directory_path() / "mini_pi_rl_resolver_test";
    fs::remove_all(tmp);

    {   // newest-valid selection must SKIP a broken newer version
        fs::create_directories(tmp / "multi" / "v0" / "params");
        fs::create_directories(tmp / "multi" / "v0" / "exported");
        std::ofstream(tmp / "multi" / "v0" / "params" / "deploy.yaml") << "a: 1\n";
        std::ofstream(tmp / "multi" / "v0" / "exported" / "policy.onnx") << "x";
        fs::create_directories(tmp / "multi" / "v1" / "params");
        fs::create_directories(tmp / "multi" / "v1" / "exported");
        std::ofstream(tmp / "multi" / "v1" / "params" / "deploy.yaml") << "a: 1\n";
        std::ofstream(tmp / "multi" / "v1" / "exported" / "policy.onnx") << "x";

        auto p = rl::resolvePolicyPackage((tmp / "multi").string(), "", "");
        check(p.valid && p.version == "v1", "lexicographically last version wins");

        // Break v1: it must fall back to v0 rather than fail or pick the parent.
        fs::remove(tmp / "multi" / "v1" / "exported" / "policy.onnx");
        auto q = rl::resolvePolicyPackage((tmp / "multi").string(), "", "");
        check(q.valid && q.version == "v0", "an invalid newest version falls back to v0");
    }

    {   // missing deploy.yaml
        fs::create_directories(tmp / "no_yaml" / "v0" / "exported");
        std::ofstream(tmp / "no_yaml" / "v0" / "exported" / "policy.onnx") << "x";
        auto p = rl::resolvePolicyPackage((tmp / "no_yaml").string(), "", "");
        check(!p.valid, "a package with no deploy.yaml is refused");
        check(p.error.find("deploy.yaml") != std::string::npos,
              "the error names the missing deploy.yaml");
    }

    {   // missing policy.onnx
        fs::create_directories(tmp / "no_model" / "v0" / "params");
        std::ofstream(tmp / "no_model" / "v0" / "params" / "deploy.yaml") << "a: 1\n";
        auto p = rl::resolvePolicyPackage((tmp / "no_model").string(), "", "");
        check(!p.valid, "a package with no policy.onnx is refused");
        check(p.error.find("policy.onnx") != std::string::npos,
              "the error names the missing policy.onnx");
    }

    {   // empty and non-existent
        fs::create_directories(tmp / "empty");
        check(!rl::resolvePolicyPackage((tmp / "empty").string(), "", "").valid,
              "an empty policy_dir is refused");
        check(!rl::resolvePolicyPackage((tmp / "nope").string(), "", "").valid,
              "a non-existent policy_dir is refused");
        check(!rl::resolvePolicyPackage("", "", "").valid, "an empty policy_dir string is refused");
    }

    fs::remove_all(tmp);
}

// ---------------------------------------------------------------------------
// malformed deploy.yaml -> the environment must refuse, with a reason
// ---------------------------------------------------------------------------
void test_bad_configs(const std::vector<std::string>& robot_names)
{
    std::printf("[deploy.yaml validation]\n");

    auto refuses = [&](const char* yaml, const char* what) {
        ++g_checks;
        try
        {
            rl::PolicyEnvironment env(YAML::Load(yaml), robot_names);
            std::printf("  FAIL %s (accepted)\n", what);
            ++g_failures;
        }
        catch (const std::exception&) { /* expected */ }
    };

    refuses("{}", "an empty deploy.yaml is refused");
    refuses("step_dt: 0.02", "a deploy.yaml with no joint_names is refused");

    const std::string names =
        "joint_names: [l_hip_pitch_joint, l_hip_roll_joint, l_thigh_joint, l_calf_joint,"
        " l_ankle_pitch_joint, l_ankle_roll_joint, r_hip_pitch_joint, r_hip_roll_joint,"
        " r_thigh_joint, r_calf_joint, r_ankle_pitch_joint, r_ankle_roll_joint]\n";
    const std::string base = "step_dt: 0.02\n" + names +
        "default_joint_pos: 0.0\nstiffness: 40\ndamping: 1.0\n";

    refuses((base + "actions: [{type: JointPositionAction}]\nobservations: {}").c_str(),
            "an empty observations block is refused");
    refuses((base + "actions: []\nobservations: {policy: {terms: [{name: base_euler}]}}").c_str(),
            "an empty actions block is refused");
    refuses((base +
             "actions: [{type: JointPositionAction}]\n"
             "observations: {policy: {terms: [{name: no_such_term}]}}").c_str(),
            "an unregistered observation term is refused");
    refuses((base +
             "actions: [{type: NoSuchAction}]\n"
             "observations: {policy: {terms: [{name: base_euler}]}}").c_str(),
            "an unregistered action term is refused");
    refuses((base +
             "actions: [{type: JointPositionAction}]\n"
             "observations: {policy: {terms: [{name: base_euler, scale: [1,1]}]}}").c_str(),
            "a scale of the wrong width is refused");
    refuses((base +
             "actions: [{type: JointPositionAction}]\n"
             "observations: {policy: {history: {length: 2, frame_dim: 99},"
             " terms: [{name: base_euler}]}}").c_str(),
            "a frame_dim that disagrees with the terms is refused");
    refuses((base +
             "actions: [{type: JointPositionAction}]\n"
             "observations: {policy: {terms: [{name: gait_phase}]}}").c_str(),
            "gait_phase without params.period is refused");

    // Duplicate / unknown policy joint name.
    {
        ++g_checks;
        try
        {
            rl::RobotView v(robot_names, std::vector<std::string>(12, "l_hip_pitch_joint"));
            std::printf("  FAIL duplicate policy joint names are refused (accepted)\n");
            ++g_failures;
        }
        catch (const std::exception&) {}
        ++g_checks;
        try
        {
            auto bad = robot_names;
            bad[3] = "not_a_joint";
            rl::RobotView v(robot_names, bad);
            std::printf("  FAIL an unknown policy joint name is refused (accepted)\n");
            ++g_failures;
        }
        catch (const std::exception&) {}
    }

    // A term list ordered as a MAP rather than a SEQUENCE must be refused,
    // because map order is exactly what this design refuses to depend on.
    refuses((base +
             "actions: [{type: JointPositionAction}]\n"
             "observations: {policy: {terms: {base_euler: {}}}}").c_str(),
            "observations.terms as a map (not a sequence) is refused");
}

// ---------------------------------------------------------------------------
// §24 the exact 47-D frame and the exact 15 x 47 history
// ---------------------------------------------------------------------------
void test_observation(const std::string& fixtures, const JointMapper& mapper)
{
    std::printf("[47-D observation frame and 15x47 history]\n");

    const std::string deploy_yaml =
        fixtures + "/policy/velocity/v0/params/deploy.yaml";
    rl::PolicyEnvironment env(YAML::LoadFile(deploy_yaml), robot_joint_names(mapper));

    const auto& groups = env.observation_manager->groups();
    check(groups.size() == 1, "one observation group");
    check(groups[0].name == "policy", "group is named 'policy' (== the ONNX input name)");
    check(groups[0].frame_dim == kFrameDim, "frame is 47 values");
    check(groups[0].history_length == kHistory, "history is 15 frames");
    check(groups[0].totalDim() == kObsDim, "total observation is 705");

    // -- at reset: 15 zero frames -----------------------------------------
    env.reset();
    {
        const auto& flat = env.observation_manager->flat("policy");
        check(flat.size() == kObsDim, "flattened observation is 705 long at reset");
        bool all_zero = true;
        for (float v : flat) if (v != 0.0f) all_zero = false;
        check(all_zero, "at reset every one of the 15 frames is zero");
    }

    // -- one frame, index by index ----------------------------------------
    const RobotState s = synthetic_state();
    VelocityCommand cmd;
    cmd.vx = 0.5; cmd.vy = -0.5; cmd.dyaw = 0.2;   // vx and vy exceed the ranges

    env.robot->update(s, cmd);
    env.observation_manager->compute();
    const auto& f = env.observation_manager->lastFrame("policy");
    check(f.size() == kFrameDim, "one computed frame is 47 values");

    // 0:2 gait phase. The FIRST frame after reset samples phase 0 (training:
    // compute_observations() right after reset_idx() zeroed
    // episode_length_buf); the clock then advances step_dt/period.
    check_close(f[0], 0.0, 1e-6, "obs[0] = sin(0) on the first frame after reset");
    check_close(f[1], 1.0, 1e-6, "obs[1] = cos(0) on the first frame after reset");
    {
        rl::PolicyEnvironment ep(YAML::LoadFile(deploy_yaml), robot_joint_names(mapper));
        ep.reset();
        ep.robot->update(s, cmd);
        ep.observation_manager->compute();
        ep.observation_manager->compute();
        const auto& g = ep.observation_manager->lastFrame("policy");
        const double phase = 0.02 / 0.4;   // fixture period 0.4
        check_close(g[0], std::sin(phase * 2.0 * M_PI), 1e-6, "second frame: sin(step_dt/period)");
        check_close(g[1], std::cos(phase * 2.0 * M_PI), 1e-6, "second frame: cos(step_dt/period)");
    }

    // 2:5 velocity command: RANGE-clamped first, then scaled.
    //   vx  0.5 -> clamp to 0.6? no: 0.5 < 0.6, stays 0.5 -> * 2.0 = 1.0
    //   vy -0.5 -> clamp to -0.3                        -> * 2.0 = -0.6
    //   yaw 0.2 -> within [-0.3, 0.3]                   -> * 1.0 = 0.2
    check_close(f[2],  1.0, 1e-6, "obs[2] = vx  clamped then * 2.0");
    check_close(f[3], -0.6, 1e-6, "obs[3] = vy  clamped to -0.3 then * 2.0");
    check_close(f[4],  0.2, 1e-6, "obs[4] = yaw clamped then * 1.0");

    // 5:17 joint_pos_rel, POLICY order, scale 1.0, default_q = 0.
    // The fixture lists the RIGHT leg first, so policy[0] is robot[6].
    const auto& p2r = env.robot->policyToRobot();
    check(p2r[0] == 6, "policy joint 0 is robot joint 6 (r_hip_pitch)");
    check(p2r[6] == 0, "policy joint 6 is robot joint 0 (l_hip_pitch)");
    for (std::size_t i = 0; i < MINI_PI_DOF; ++i)
    {
        check_close(f[5 + i], s.robot_q[static_cast<std::size_t>(p2r[i])], 1e-6,
                    "obs[" + std::to_string(5 + i) + "] = joint_pos_rel in POLICY order");
    }

    // 17:29 joint_vel * 0.05, policy order.
    for (std::size_t i = 0; i < MINI_PI_DOF; ++i)
    {
        check_close(f[17 + i], s.robot_dq[static_cast<std::size_t>(p2r[i])] * 0.05, 1e-6,
                    "obs[" + std::to_string(17 + i) + "] = joint_vel * 0.05");
    }

    // 29:41 last_action -- the RAW previous output, all zero after a reset.
    for (std::size_t i = 0; i < MINI_PI_DOF; ++i)
        check_close(f[29 + i], 0.0, 1e-9,
                    "obs[" + std::to_string(29 + i) + "] = last_action (zero after reset)");

    // 41:44 base angular velocity.
    check_close(f[41], 0.31, 1e-6, "obs[41] = gyro x");
    check_close(f[42], 0.32, 1e-6, "obs[42] = gyro y");
    check_close(f[43], 0.33, 1e-6, "obs[43] = gyro z");

    // 44:47 euler.
    check_close(f[44], 0.41, 1e-6, "obs[44] = roll");
    check_close(f[45], 0.42, 1e-6, "obs[45] = pitch");
    check_close(f[46], 0.43, 1e-6, "obs[46] = yaw");

    // -- clipping ----------------------------------------------------------
    {
        rl::PolicyEnvironment e2(YAML::LoadFile(deploy_yaml), robot_joint_names(mapper));
        e2.reset();
        RobotState big = synthetic_state();
        // joint_vel * 0.05 must exceed +18 -> needs dq > 360.
        for (std::size_t j = 0; j < MINI_PI_DOF; ++j) big.robot_dq[j] = 1000.0;
        e2.robot->update(big, VelocityCommand{});
        e2.observation_manager->compute();
        const auto& g = e2.observation_manager->lastFrame("policy");
        for (std::size_t i = 0; i < MINI_PI_DOF; ++i)
            check_close(g[17 + i], 18.0, 1e-6,
                        "obs[" + std::to_string(17 + i) + "] clipped to +18");
        // And the clip is applied AFTER the scale: 1000 * 0.05 = 50 -> 18.
        // (had it been applied before, the value would be 18 * 0.05 = 0.9)
    }

    // -- history: oldest -> newest, whole frames contiguous ----------------
    {
        rl::PolicyEnvironment e3(YAML::LoadFile(deploy_yaml), robot_joint_names(mapper));
        e3.reset();

        // Push 15 frames whose euler yaw (index 46) is a unique marker.
        for (std::size_t k = 0; k < kHistory; ++k)
        {
            RobotState st = synthetic_state();
            st.rpy[2] = 1.0 + static_cast<double>(k);   // 1, 2, ... 15
            e3.robot->update(st, VelocityCommand{});
            e3.observation_manager->compute();
        }
        const auto& flat = e3.observation_manager->flat("policy");
        check(flat.size() == kObsDim, "history flattens to 705");

        for (std::size_t k = 0; k < kHistory; ++k)
        {
            // Frame k of the flattened vector must be the (k+1)-th pushed
            // frame: index 0 is the OLDEST.
            check_close(flat[k * kFrameDim + 46], 1.0 + static_cast<double>(k), 1e-6,
                        "flat frame " + std::to_string(k) + " is the " +
                        std::to_string(k + 1) + "-th pushed (oldest first)");
        }

        // One more push must drop the oldest and shift everything down.
        RobotState st = synthetic_state();
        st.rpy[2] = 16.0;
        e3.robot->update(st, VelocityCommand{});
        e3.observation_manager->compute();
        const auto& flat2 = e3.observation_manager->flat("policy");
        for (std::size_t k = 0; k < kHistory; ++k)
        {
            check_close(flat2[k * kFrameDim + 46], 2.0 + static_cast<double>(k), 1e-6,
                        "after one more push, frame " + std::to_string(k) + " shifted down");
        }

        // The newest frame must be contiguous, not interleaved: its 47 values
        // must all belong to the same sample.
        const std::size_t last = (kHistory - 1) * kFrameDim;
        check_close(flat2[last + 41], 0.31, 1e-6, "newest frame gyro x is in the newest frame");
        check_close(flat2[last + 46], 16.0, 1e-6, "newest frame yaw is the newest sample");
    }

    // -- reset() really clears the history ---------------------------------
    {
        env.reset();
        const auto& flat = env.observation_manager->flat("policy");
        bool all_zero = true;
        for (float v : flat) if (v != 0.0f) all_zero = false;
        check(all_zero, "reset() restores 15 zero frames");
        check_close(env.global_phase, 0.0, 1e-9, "reset() zeroes the gait phase");
    }
}

// ---------------------------------------------------------------------------
// §25 the action pipeline
// ---------------------------------------------------------------------------
void test_action(const std::string& fixtures, const JointMapper& mapper)
{
    std::printf("[action pipeline: clip -> scale -> offset, policy -> robot order]\n");

    const std::string deploy_yaml = fixtures + "/policy/velocity/v0/params/deploy.yaml";
    rl::PolicyEnvironment env(YAML::LoadFile(deploy_yaml), robot_joint_names(mapper));
    env.reset();

    check(env.action_manager->total_action_dim() == static_cast<int>(kActDim),
          "action dim is 12");

    // Raw action with one value well outside the clip, so clip-before-scale is
    // distinguishable from scale-before-clip.
    std::vector<float> raw(kActDim, 0.0f);
    raw[0] = 100.0f;    // clipped to 18 -> * 0.25 = 4.5
    raw[1] = -100.0f;   // clipped to -18 -> * 0.25 = -4.5
    raw[2] = 4.0f;      // within clip -> * 0.25 = 1.0
    raw[3] = -2.0f;     // -> -0.5
    env.action_manager->process_action(raw);

    const auto& proc = env.action_manager->processed_actions();
    check(proc.size() == kActDim, "processed action is 12 values");
    // offset is 0 in the fixture, so processed == clip(raw) * scale.
    check_close(proc[0],  4.5, 1e-6, "raw 100 -> clip 18 -> * 0.25 = 4.5");
    check_close(proc[1], -4.5, 1e-6, "raw -100 -> clip -18 -> * 0.25 = -4.5");
    check_close(proc[2],  1.0, 1e-6, "raw 4 -> * 0.25 = 1.0");
    check_close(proc[3], -0.5, 1e-6, "raw -2 -> * 0.25 = -0.5");
    // Had the clip been applied AFTER the scale, proc[0] would be 18, not 4.5.
    check(std::fabs(proc[0] - 18.0) > 1.0, "clip is applied to the RAW action, not the target");

    // last_action must report the CLIPPED, UNSCALED value -- humanoid-gym's
    // self.actions = clip(actions) -- never the processed target.
    const auto& last = env.action_manager->action();
    check_close(last[0], 18.0, 1e-6, "action() is the network output after the raw clip");
    check_close(last[1], -18.0, 1e-6, "action() is clipped on the low side too");
    check_close(last[2], 4.0, 1e-6, "action() is NOT scaled");
    check_close(env.action_manager->networkOutput()[0], 100.0, 1e-6,
                "networkOutput() keeps the unmodified model output");

    // -- offset is the default pose, and a non-zero one is honoured --------
    {
        YAML::Node cfg = YAML::LoadFile(deploy_yaml);
        for (std::size_t i = 0; i < kActDim; ++i)
            cfg["default_joint_pos"][i] = 0.1 * static_cast<double>(i + 1);
        cfg["actions"][0].remove("offset");   // omitted => default_joint_pos
        rl::PolicyEnvironment e2(cfg, robot_joint_names(mapper));
        e2.reset();
        e2.action_manager->process_action(std::vector<float>(kActDim, 0.0f));
        const auto& p = e2.action_manager->processed_actions();
        for (std::size_t i = 0; i < kActDim; ++i)
            check_close(p[i], 0.1 * static_cast<double>(i + 1), 1e-6,
                        "a zero action yields default_joint_pos[" + std::to_string(i) + "]");
    }

    // -- policy order -> robot order ---------------------------------------
    {
        std::vector<float> policy_vals(kActDim);
        for (std::size_t i = 0; i < kActDim; ++i) policy_vals[i] = static_cast<float>(i);
        JointArray robot_vals{};
        env.robot->policyToRobotArray(policy_vals, robot_vals);
        const auto& p2r = env.robot->policyToRobot();
        for (std::size_t i = 0; i < kActDim; ++i)
            check_close(robot_vals[static_cast<std::size_t>(p2r[i])],
                        static_cast<double>(i), 1e-9,
                        "policy[" + std::to_string(i) + "] lands at robot[" +
                        std::to_string(p2r[i]) + "]");
        // And it really is a permutation, not the identity.
        check(p2r[0] != 0, "the policy order is NOT the robot order (a real permutation)");
    }

    // -- a wrong-width action is refused, not truncated ---------------------
    {
        ++g_checks;
        try
        {
            env.action_manager->process_action(std::vector<float>(11, 0.0f));
            std::printf("  FAIL an 11-value action is refused (accepted)\n");
            ++g_failures;
        }
        catch (const std::exception&) {}
    }
}

// ---------------------------------------------------------------------------
// §26 ONNX Runtime
// ---------------------------------------------------------------------------
void test_model(const std::string& fixtures)
{
    std::printf("[ONNX Runtime / OrtRunner]\n");

    if (!rl::haveInferenceBackend())
    {
        std::printf("  SKIP: this build has no inference backend "
                    "(MINI_PI_ENABLE_RL=OFF)\n");
        return;
    }

    const std::string exported = fixtures + "/policy/velocity/v0/exported";

    // -- missing file ------------------------------------------------------
    {
        std::string err;
        auto m = rl::createOrtRunner(exported + "/does_not_exist.onnx", err);
        check(!m, "a missing model file is refused");
        check(err.find("does not exist") != std::string::npos, "the error says the file is missing");
    }

    // -- not a model -------------------------------------------------------
    {
        const fs::path junk = fs::temp_directory_path() / "mini_pi_not_a_model.onnx";
        std::ofstream(junk) << "this is not an onnx model";
        std::string err;
        auto m = rl::createOrtRunner(junk.string(), err);
        check(!m, "a file that is not a model is refused");
        check(!err.empty(), "the error is non-empty");
        fs::remove(junk);
    }

    // -- the zero policy ---------------------------------------------------
    {
        std::string err;
        auto m = rl::createOrtRunner(exported + "/test_zero_policy.onnx", err);
        check(m != nullptr, "test_zero_policy.onnx loads: " + err);
        if (m)
        {
            check(m->inputNames().size() == 1, "the model has exactly one input");
            check(m->inputNames()[0] == "policy", "the input is named 'policy'");
            check(m->inputSize(0) == kObsDim, "input is 705 values");
            check(m->outputSize() == kActDim, "output is 12 values");

            std::map<std::string, std::vector<float>> obs;
            obs["policy"] = std::vector<float>(kObsDim, 0.7f);
            // COPY, not a reference: act() returns a reference to a buffer the
            // next call overwrites, so holding a reference across two calls
            // compares a value with itself.
            const std::vector<float> a = m->act(obs);
            check(a.size() == kActDim, "inference returns 12 values");
            for (std::size_t i = 0; i < a.size(); ++i)
                check_close(a[i], 0.0, 1e-7, "zero policy output[" + std::to_string(i) + "] == 0");
            for (float v : a) check(std::isfinite(v), "output is finite");

            // Determinism: the same input twice gives the same output.
            const std::vector<float> b = m->act(obs);
            for (std::size_t i = 0; i < a.size(); ++i)
                check(b[i] == a[i], "inference is deterministic");
        }
    }

    // -- the sum policy: proves the observation really reaches the model ----
    {
        std::string err;
        auto m = rl::createOrtRunner(exported + "/test_sum_policy.onnx", err);
        check(m != nullptr, "test_sum_policy.onnx loads: " + err);
        if (m)
        {
            std::map<std::string, std::vector<float>> obs;
            obs["policy"] = std::vector<float>(kObsDim, 0.5f);
            const std::vector<float> a = m->act(obs);   // copy; see above
            const double sum = 0.5 * static_cast<double>(kObsDim);
            for (std::size_t j = 0; j < kActDim; ++j)
            {
                const double want = kSumWeight * sum + kBiasStep * static_cast<double>(j + 1);
                check_close(a[j], want, 1e-4,
                            "sum policy output[" + std::to_string(j) + "] matches closed form");
            }

            // A different observation must give a different answer -- i.e. the
            // input is genuinely consumed.
            obs["policy"] = std::vector<float>(kObsDim, 1.5f);
            const std::vector<float> b = m->act(obs);
            check(std::fabs(b[0] - a[0]) > 1e-4, "changing the observation changes the output");
            // And the new answer still matches the closed form.
            const double sum2 = 1.5 * static_cast<double>(kObsDim);
            for (std::size_t j = 0; j < kActDim; ++j)
                check_close(b[j], kSumWeight * sum2 + kBiasStep * static_cast<double>(j + 1),
                            1e-3, "second observation also matches the closed form");

            // Wrong-size input is refused rather than read out of bounds.
            ++g_checks;
            try
            {
                std::map<std::string, std::vector<float>> bad;
                bad["policy"] = std::vector<float>(kObsDim - 1, 0.0f);
                m->act(bad);
                std::printf("  FAIL a 704-value observation is refused (accepted)\n");
                ++g_failures;
            }
            catch (const std::exception&) {}

            // A missing input name is refused.
            ++g_checks;
            try
            {
                std::map<std::string, std::vector<float>> bad;
                bad["not_policy"] = std::vector<float>(kObsDim, 0.0f);
                m->act(bad);
                std::printf("  FAIL a missing input name is refused (accepted)\n");
                ++g_failures;
            }
            catch (const std::exception&) {}
        }
    }
}

// ---------------------------------------------------------------------------
// §27 the whole path, without the FSM
// ---------------------------------------------------------------------------
void test_environment(const std::string& fixtures, const JointMapper& mapper)
{
    std::printf("[PolicyEnvironment end to end: state -> obs -> model -> action -> command]\n");

    if (!rl::haveInferenceBackend()) { std::printf("  SKIP: no inference backend\n"); return; }

    const std::string pkg = fixtures + "/policy/velocity/v0";
    rl::PolicyEnvironment env(YAML::LoadFile(pkg + "/params/deploy.yaml"),
                              robot_joint_names(mapper));

    std::string err;
    auto model = rl::createOrtRunner(pkg + "/exported/test_zero_policy.onnx", err);
    check(model != nullptr, "model loads: " + err);
    if (!model) return;
    env.setModel(std::move(model));
    env.reset();

    const RobotState s = synthetic_state();
    VelocityCommand cmd; cmd.vx = 0.2;

    RobotCommand out;
    env.step(s, cmd, out);

    // The zero policy gives action == 0, so q_target == default_joint_pos == 0.
    for (std::size_t j = 0; j < MINI_PI_DOF; ++j)
        check_close(out.q[j], 0.0, 1e-6,
                    "zero action -> robot_q target[" + std::to_string(j) + "] == default (0)");
    for (std::size_t j = 0; j < MINI_PI_DOF; ++j)
    {
        check_close(out.dq[j], 0.0, 1e-12, "dq_target is 0");
        check_close(out.torque[j], 0.0, 1e-12, "tau_ff is 0");
    }

    // Gains: from the package, in POLICY order, scattered to ROBOT order.
    // stiffness [40,20,20,40,40,20]x2 with the RIGHT leg first means
    // robot[6] (r_hip_pitch) gets 40 and robot[7] (r_hip_roll) gets 20.
    const auto& p2r = env.robot->policyToRobot();
    for (std::size_t i = 0; i < kActDim; ++i)
    {
        const std::size_t r = static_cast<std::size_t>(p2r[i]);
        check_close(out.kp[r], env.stiffness[i], 1e-6,
                    "kp lands at robot joint " + std::to_string(r));
        check_close(out.kd[r], env.damping[i], 1e-6,
                    "kd lands at robot joint " + std::to_string(r));
    }

    // The environment must NOT stamp the command -- only the FSM may.
    check(!out.valid, "PolicyEnvironment does not mark the command valid");
    check(out.seq == 0, "PolicyEnvironment does not bump the sequence number");

    // Stepping repeatedly must stay finite and keep advancing the phase.
    const float phase_after_one = env.global_phase;
    for (int k = 0; k < 200; ++k) env.step(s, cmd, out);
    check(env.episode_length == 201, "episode_length counts every step");
    check(env.global_phase != phase_after_one, "the gait phase advances");
    check(env.global_phase >= 0.0f && env.global_phase < 1.0f, "the phase stays in [0, 1)");
    for (std::size_t j = 0; j < MINI_PI_DOF; ++j)
        check(std::isfinite(out.q[j]), "the target stays finite over 200 steps");

    // -- a model whose shape disagrees with the config must be refused -----
    {
        YAML::Node cfg = YAML::LoadFile(pkg + "/params/deploy.yaml");
        cfg["observations"]["policy"]["history"]["length"] = 14;   // 14*47 = 658, not 705
        cfg["observations"]["policy"]["history"].remove("total_dim");
        rl::PolicyEnvironment bad(cfg, robot_joint_names(mapper));
        std::string e2;
        auto m2 = rl::createOrtRunner(pkg + "/exported/test_zero_policy.onnx", e2);
        ++g_checks;
        try
        {
            bad.setModel(std::move(m2));
            std::printf("  FAIL a 658-value observation against a 705-input model is refused "
                        "(accepted)\n");
            ++g_failures;
        }
        catch (const std::exception&) {}
    }

    // -- a group name that does not match the model input is refused -------
    {
        YAML::Node cfg = YAML::LoadFile(pkg + "/params/deploy.yaml");
        YAML::Node obs = YAML::Node(YAML::NodeType::Map);
        obs["not_policy"] = cfg["observations"]["policy"];
        cfg["observations"] = obs;
        rl::PolicyEnvironment bad(cfg, robot_joint_names(mapper));
        std::string e2;
        auto m2 = rl::createOrtRunner(pkg + "/exported/test_zero_policy.onnx", e2);
        ++g_checks;
        try
        {
            bad.setModel(std::move(m2));
            std::printf("  FAIL a mismatched group name is refused (accepted)\n");
            ++g_failures;
        }
        catch (const std::exception&) {}
    }
}

// ---------------------------------------------------------------------------
// Real production package: resolver + config + actual ONNX inference.
// ---------------------------------------------------------------------------
void test_production_policy(const std::string& config_dir, const JointMapper& mapper)
{
    std::printf("[REAL production policy: package -> observation -> ONNX -> command]\n");

    const std::string root = config_dir + "/policy/velocity";
    const auto pkg = rl::resolvePolicyPackage(root, "", "");
    check(pkg.valid, "the production policy package resolves");
    if (!pkg.valid) return;
    check(pkg.version == "v0", "production resolver selects v0");
    check(fs::is_regular_file(pkg.deploy_yaml), "production deploy.yaml exists");
    check(fs::is_regular_file(pkg.model), "production policy.onnx exists");

    YAML::Node cfg = YAML::LoadFile(pkg.deploy_yaml);
    check(!(cfg["simulation_only"] && cfg["simulation_only"].as<bool>()),
          "the real package is not marked simulation_only");
    check_close(cfg["step_dt"].as<double>(), 0.02, 1e-9, "real policy period is 0.02 s");

    rl::PolicyEnvironment env(cfg, robot_joint_names(mapper));
    const auto& groups = env.observation_manager->groups();
    check(groups.size() == 1, "real policy has one observation group");
    check(groups[0].name == "input", "observation group matches real ONNX input 'input'");
    check(groups[0].frame_dim == kFrameDim, "real policy frame dimension is 47");
    check(groups[0].history_length == kHistory, "real policy history is 15 frames");
    check(groups[0].totalDim() == kObsDim, "real policy input dimension is 705");
    check(env.action_manager->total_action_dim() == static_cast<int>(kActDim),
          "real policy action dimension is 12");

    check(std::isfinite(env.step_dt), "real step_dt is finite");
    for (float v : env.robot->data.default_joint_pos)
        check(std::isfinite(v), "real default joint position is finite");
    for (float v : env.stiffness) check(std::isfinite(v), "real stiffness is finite");
    for (float v : env.damping) check(std::isfinite(v), "real damping is finite");
    for (float v : env.command_ranges.lin_vel_x)
        check(std::isfinite(v), "real vx range is finite");
    for (float v : env.command_ranges.lin_vel_y)
        check(std::isfinite(v), "real vy range is finite");
    for (float v : env.command_ranges.ang_vel_z)
        check(std::isfinite(v), "real yaw range is finite");
    for (const auto& group : groups)
    {
        if (group.clip_enabled)
        {
            check(std::isfinite(group.clip_lo), "real observation clip low is finite");
            check(std::isfinite(group.clip_hi), "real observation clip high is finite");
        }
        for (const auto& term : group.terms)
        {
            for (float v : term.scale)
                check(std::isfinite(v), "real observation scale is finite");
            for (float v : term.clip)
                check(std::isfinite(v), "real term clip is finite");
        }
    }
    for (const auto action_cfg : cfg["actions"])
    {
        if (action_cfg["scale"])
        {
            if (action_cfg["scale"].IsScalar())
                check(std::isfinite(action_cfg["scale"].as<float>()),
                      "real action scale is finite");
            else
                for (float v : action_cfg["scale"].as<std::vector<float>>())
                    check(std::isfinite(v), "real action scale is finite");
        }
        if (action_cfg["offset"] && !action_cfg["offset"].IsNull())
            for (float v : action_cfg["offset"].as<std::vector<float>>())
                check(std::isfinite(v), "real action offset is finite");
        if (action_cfg["clip"] && !action_cfg["clip"].IsNull())
            for (float v : action_cfg["clip"].as<std::vector<float>>())
                check(std::isfinite(v), "real action clip is finite");
    }

    // The author's sim2sim.py presents LEFT first to the network, even though
    // its raw MJCF joint order is right first.
    const auto& p2r = env.robot->policyToRobot();
    check(p2r[0] == 0, "real policy[0] is l_hip_pitch / robot[0]");
    check(p2r[6] == 6, "real policy[6] is r_hip_pitch / robot[6]");

    // ---- the TRAINING contract, pinned (Mini_Pi_RL_Baseline PaiCfg) --------
    {
        // Isaac Gym DOF order of pi_12dof_release_v1_rl.urdf, as printed by
        // gym.get_asset_dof_names(): left leg first.
        const std::vector<std::string> isaac_order = {
            "l_hip_pitch_joint", "l_hip_roll_joint", "l_thigh_joint", "l_calf_joint",
            "l_ankle_pitch_joint", "l_ankle_roll_joint", "r_hip_pitch_joint",
            "r_hip_roll_joint", "r_thigh_joint", "r_calf_joint", "r_ankle_pitch_joint",
            "r_ankle_roll_joint"};
        check(env.robot->jointNames() == isaac_order,
              "real policy order == Isaac Gym DOF order of the training asset");

        const float kp[6] = {40, 20, 20, 40, 40, 20};
        const float kd[6] = {0.6f, 0.4f, 0.4f, 0.6f, 0.6f, 0.4f};
        for (std::size_t i = 0; i < 12; ++i)
        {
            check_close(env.stiffness[i], kp[i % 6], 1e-6, "real kp == PaiCfg.control.stiffness");
            check_close(env.damping[i], kd[i % 6], 1e-6, "real kd == PaiCfg.control.damping");
        }
        check_close(env.step_dt, 0.02, 1e-9, "real step_dt == sim.dt * decimation = 0.02");

        const YAML::Node terms = env.cfg["observations"]["input"]["terms"];
        auto term = [&](const std::string& n) -> YAML::Node {
            for (const auto& t : terms) if (t["name"].as<std::string>() == n) return YAML::Node(t);
            return YAML::Node();
        };
        check_close(term("gait_phase")["params"]["period"].as<double>(), 0.4, 1e-9,
                    "real gait period == PaiCfg.rewards.cycle_time (0.4 s)");
        check(!term("gait_phase")["params"]["zero_on_stand"] ||
              !term("gait_phase")["params"]["zero_on_stand"].as<bool>(),
              "real gait clock is never zeroed on stand (training has no such rule)");
        check_close(term("base_ang_vel")["scale"].as<double>(), 1.0, 1e-9,
                    "real gyro scale == obs_scales.ang_vel (1.0)");
        check_close(term("joint_vel")["scale"].as<double>(), 0.05, 1e-9,
                    "real dq scale == obs_scales.dof_vel (0.05)");
        check(term("base_euler")["params"]["yaw"].as<std::string>() == "relative_to_entry",
              "real yaw is relative to RL entry (training spawns at yaw 0)");
        check_close(term("velocity_commands")["params"]["min_lin_norm"].as<double>(), 0.2, 1e-9,
                    "real linear-command deadband == _resample_commands 0.2");

        const YAML::Node act = env.cfg["actions"][0];
        check_close(act["clip"][1].as<double>(), 18.0, 1e-9,
                    "real raw action clip == normalization.clip_actions (18)");
        check_close(act["scale"].as<double>(), 0.25, 1e-9, "real action scale == 0.25");
        // Isaac Gym get_asset_dof_properties() of the training asset.
        const double lo[12] = {-1.25, -0.12, -0.3, -0.65, -0.5, -0.15,
                               -1.25, -0.5, -0.6, -0.65, -0.5, -0.15};
        const double hi[12] = {1.75, 0.5, 0.6, 1.65, 1.3, 0.15,
                               1.75, 0.12, 0.3, 1.65, 1.3, 0.15};
        for (std::size_t i = 0; i < 12; ++i)
        {
            check_close(act["clip_output"]["lower"][i].as<double>(), lo[i], 1e-9,
                        "real q_target clamp lower == training DOF limit");
            check_close(act["clip_output"]["upper"][i].as<double>(), hi[i], 1e-9,
                        "real q_target clamp upper == training DOF limit");
        }
        // And the clamp is live: a saturated action lands exactly on the limit.
        env.action_manager->process_action(std::vector<float>(12, 100.0f));
        for (std::size_t i = 0; i < 12; ++i)
            check_close(env.action_manager->processed_actions()[i], hi[i], 1e-6,
                        "real q_target is clamped to the training upper limit");
        env.action_manager->process_action(std::vector<float>(12, -100.0f));
        for (std::size_t i = 0; i < 12; ++i)
            check_close(env.action_manager->processed_actions()[i], lo[i], 1e-6,
                        "real q_target is clamped to the training lower limit");
        env.reset();
    }

    if (!rl::haveInferenceBackend())
    {
        check(false, "ONNX Runtime is available for the real-policy test");
        return;
    }

    std::string err;
    auto model = rl::createOrtRunner(pkg.model, err);
    check(model != nullptr, "real ONNX loads: " + err);
    if (!model) return;
    check(model->inputNames().size() == 1, "real ONNX has one input");
    check(model->inputNames()[0] == "input", "real ONNX input name is input");
    check(model->inputSize(0) == kObsDim, "real ONNX input is 705 values");
    check(model->outputSize() == kActDim, "real ONNX output is 12 values");
    env.setModel(std::move(model));
    env.reset();

    RobotState state{};
    const auto now = Clock::now();
    for (std::size_t j = 0; j < MINI_PI_DOF; ++j)
    {
        state.motor_fresh[j] = true;
        state.motor_age[j] = 0.0;
    }
    state.orientation = {0.0, 0.0, 0.0, 1.0};
    state.motor_valid = true;
    state.imu_valid = true;
    state.motor_stamp = now;
    state.imu_stamp = now;
    state.imu_age = 0.0;

    // Verify the real package's exact command slots and scales, independently
    // of the deterministic TEST fixture used by the broader observation test.
    VelocityCommand diagnostic_cmd;
    diagnostic_cmd.vx = 0.25;
    diagnostic_cmd.vy = 0.10;
    diagnostic_cmd.dyaw = 0.10;
    env.robot->update(state, diagnostic_cmd);
    env.observation_manager->compute();
    {
        const auto& command_frame = env.observation_manager->lastFrame("input");
        check_close(command_frame[2], 0.50, 1e-6, "real obs receives vx=0.25 scaled by 2");
        check_close(command_frame[3], 0.20, 1e-6, "real obs receives vy=0.10 scaled by 2");
        check_close(command_frame[4], 0.10, 1e-6, "real obs receives yaw=0.10 scaled by 1");
    }
    // Training never saw 0 < ||(vx, vy)|| < 0.2 (_resample_commands zeroes it).
    diagnostic_cmd.vx = 0.10;
    diagnostic_cmd.vy = 0.05;
    env.robot->update(state, diagnostic_cmd);
    env.observation_manager->compute();
    {
        const auto& command_frame = env.observation_manager->lastFrame("input");
        check_close(command_frame[2], 0.0, 1e-9, "real obs: ||v||=0.11 < 0.2 -> vx zeroed");
        check_close(command_frame[3], 0.0, 1e-9, "real obs: ||v||=0.11 < 0.2 -> vy zeroed");
        check_close(command_frame[4], 0.10, 1e-6, "real obs: yaw rate is NOT deadbanded");
    }
    env.reset();

    // Yaw relative to the heading at the first policy step, wrapped to
    // (-pi, pi]. 3.0 -> -3.0 is a +0.2832 rad turn, not -6.0.
    {
        RobotState ys = state;
        ys.rpy = {0.05, -0.04, 3.0};
        env.robot->update(ys, VelocityCommand{});
        env.observation_manager->compute();
        check_close(env.observation_manager->lastFrame("input")[46], 0.0, 1e-6,
                    "real obs: yaw is 0 on the first frame after reset, whatever the heading");
        check_close(env.observation_manager->lastFrame("input")[44], 0.05, 1e-6,
                    "real obs: roll is absolute");
        check_close(env.observation_manager->lastFrame("input")[45], -0.04, 1e-6,
                    "real obs: pitch is absolute");
        ys.rpy[2] = -3.0;
        env.robot->update(ys, VelocityCommand{});
        env.observation_manager->compute();
        check_close(env.observation_manager->lastFrame("input")[46],
                    2.0 * M_PI - 6.0, 1e-5, "real obs: relative yaw wraps across +-pi");
        env.reset();
        ys.rpy[2] = -1.0;
        env.robot->update(ys, VelocityCommand{});
        env.observation_manager->compute();
        check_close(env.observation_manager->lastFrame("input")[46], 0.0, 1e-6,
                    "real obs: reset() re-captures the yaw origin");
        env.reset();
    }

    RobotCommand out;
    env.step(state, VelocityCommand{}, out);
    const std::vector<float> first_raw = env.action_manager->action();
    check(first_raw.size() == kActDim, "real inference returns 12 raw actions");
    for (float v : first_raw) check(std::isfinite(v), "first real-policy action is finite");

    const auto& first_obs = env.observation_manager->flat("input");
    check(first_obs.size() == kObsDim, "real model receives 705 observations");
    bool initial_prefix_zero = true;
    for (std::size_t i = 0; i < (kHistory - 1) * kFrameDim; ++i)
        if (first_obs[i] != 0.0f) initial_prefix_zero = false;
    check(initial_prefix_zero, "first input contains 14 zero history frames");

    // One frame is appended per PolicyEnvironment::step. The next frame's
    // last_action slots must be the previous RAW ONNX result, not q_target.
    env.step(state, VelocityCommand{}, out);
    const auto& second_frame = env.observation_manager->lastFrame("input");
    for (std::size_t j = 0; j < kActDim; ++j)
        check_close(second_frame[29 + j], first_raw[j], 1e-6,
                    "real obs previous raw action[" + std::to_string(j) + "]");

    // Warm up, then measure the exact model->act section instrumented inside
    // PolicyEnvironment. It excludes observation and action processing.
    for (int i = 0; i < 20; ++i) env.step(state, VelocityCommand{}, out);
    env.reset();

    float obs_min = std::numeric_limits<float>::infinity();
    float obs_max = -std::numeric_limits<float>::infinity();
    float raw_min = std::numeric_limits<float>::infinity();
    float raw_max = -std::numeric_limits<float>::infinity();
    double q_min = std::numeric_limits<double>::infinity();
    double q_max = -std::numeric_limits<double>::infinity();
    constexpr int kSteps = 250;
    for (int i = 0; i < kSteps; ++i)
    {
        env.step(state, VelocityCommand{}, out);
        for (float v : env.observation_manager->flat("input"))
        {
            check(std::isfinite(v), "real-policy observation is finite");
            obs_min = std::min(obs_min, v); obs_max = std::max(obs_max, v);
        }
        for (float v : env.action_manager->action())
        {
            check(std::isfinite(v), "real-policy raw action is finite");
            raw_min = std::min(raw_min, v); raw_max = std::max(raw_max, v);
        }
        for (double v : out.q)
        {
            check(std::isfinite(v), "real-policy q_target is finite");
            q_min = std::min(q_min, v); q_max = std::max(q_max, v);
        }
    }
    const double avg_ms = env.meanInferenceSeconds() * 1e3;
    const double max_ms = env.maxInferenceSeconds() * 1e3;
    check(avg_ms < 20.0, "average real-policy inference latency is below 20 ms");
    check(max_ms < 20.0, "maximum warmed real-policy inference latency is below 20 ms");

    std::printf("  input=705 history=15x47; zero-state observation min/max = %.6f / %.6f\n",
                obs_min, obs_max);
    std::printf("  raw action min/max = %.6f / %.6f; q_target min/max = %.6f / %.6f\n",
                raw_min, raw_max, q_min, q_max);
    std::printf("  inference latency over %d warmed steps: avg %.4f ms, max %.4f ms\n",
                kSteps, avg_ms, max_ms);
    std::printf("  final raw action:");
    for (float v : env.action_manager->action()) std::printf(" %+.5f", v);
    std::printf("\n  final q_target (robot order):");
    for (double v : out.q) std::printf(" %+.5f", v);
    std::printf("\n");
}

// ---------------------------------------------------------------------------
// §18 / §33 production readiness and the real-hardware TEST-fixture lock
// ---------------------------------------------------------------------------
void test_policy_gating(const std::string& config_dir, const std::string& fixtures,
                        const JointMapper& mapper)
{
    std::printf("[production default and the test-policy lock]\n");

    const std::string policy_yaml = config_dir + "/policy.yaml";
    YAML::Node fsm = YAML::LoadFile(config_dir + "/fsm.yaml")["FSM"];

    PolicyRunner::Options opt;
    opt.fsm = fsm;
    opt.base_dir = config_dir + "/..";
    opt.robot_joint_names = robot_joint_names(mapper);

    // -- shipped production config: the real package is READY ---------------
    {
        opt.backend = "sim";
        opt.backend_is_simulation = true;
        PolicyRunner p;
        check(p.initialize(policy_yaml, opt), "production PolicyRunner initializes");
        check(p.ready(), "production config/policy/velocity is POLICY_READY");
        check(p.status() == PolicyStatus::Ready, "production status is POLICY_READY");
        check(std::string(to_string(p.status())) == "POLICY_READY",
              "production status token is POLICY_READY");
        check(p.package().version == "v0", "production runner selects v0");
        check(!p.start(), "start() still refuses without a state provider");
        check(!p.latest().valid, "latest() is invalid before start");
        p.stop();
        check(true, "stop() is safe when never started");
    }

    // -- test fixture + real backend: REFUSED even with the flag on --------
    {
        YAML::Node py = YAML::LoadFile(policy_yaml);
        py["policy"]["allow_test_policy"] = true;
        const fs::path tmp = fs::temp_directory_path() / "mini_pi_policy_allow.yaml";
        std::ofstream(tmp) << py;

        YAML::Node f2 = YAML::Clone(fsm);
        f2["Velocity"]["policy_dir"] = fixtures + "/policy/velocity";

        PolicyRunner::Options o2 = opt;
        o2.fsm = f2;
        o2.backend = "hightorque";
        o2.backend_is_simulation = false;

        PolicyRunner p;
        p.initialize(tmp.string(), o2);
        check(!p.ready(), "a simulation_only package is REFUSED on a real backend "
                          "even with allow_test_policy=true");
        check(p.unavailableReason().find("simulation_only") != std::string::npos,
              "the refusal names simulation_only");

        // -- and with allow_test_policy off, on a SIM backend: also refused --
        o2.backend = "sim";
        o2.backend_is_simulation = true;
        PolicyRunner q;
        q.initialize(policy_yaml, o2);   // allow_test_policy is false here
        check(!q.ready(), "a simulation_only package is refused while allow_test_policy "
                          "is false, even on a sim backend");

        // -- both conditions satisfied: accepted ---------------------------
        if (rl::haveInferenceBackend())
        {
            PolicyRunner r;
            r.initialize(tmp.string(), o2);
            check(r.ready(), "a simulation_only package loads on a sim backend with "
                             "allow_test_policy=true: " + r.unavailableReason());
            if (r.ready())
            {
                check(r.package().version == "v0", "the fixture version is v0");
                check(std::fabs(r.stepDt() - 0.02f) < 1e-9f, "step_dt is 0.02 (50 Hz)");
                check(!r.start(), "start() refuses without a state provider");
            }
        }
        fs::remove(tmp);
    }

    // -- a malformed deploy.yaml must not become Ready ---------------------
    {
        const fs::path tmp = fs::temp_directory_path() / "mini_pi_bad_pkg";
        fs::remove_all(tmp);
        fs::create_directories(tmp / "velocity" / "v0" / "params");
        fs::create_directories(tmp / "velocity" / "v0" / "exported");
        std::ofstream(tmp / "velocity" / "v0" / "params" / "deploy.yaml")
            << "this: [is, not, valid,\n";
        fs::copy_file(fixtures + "/policy/velocity/v0/exported/policy.onnx",
                      tmp / "velocity" / "v0" / "exported" / "policy.onnx");

        YAML::Node f2 = YAML::Clone(fsm);
        f2["Velocity"]["policy_dir"] = (tmp / "velocity").string();
        PolicyRunner::Options o2 = opt;
        o2.fsm = f2;
        o2.backend = "sim";
        o2.backend_is_simulation = true;

        PolicyRunner p;
        check(p.initialize(policy_yaml, o2), "initialize() survives a malformed deploy.yaml");
        check(!p.ready(), "a malformed deploy.yaml does not become Ready");
        fs::remove_all(tmp);
    }
}

} // namespace

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        std::printf("usage: %s <config dir> <fixtures dir>\n", argv[0]);
        return 2;
    }
    const std::string config_dir = argv[1];
    const std::string fixtures   = argv[2];

    std::printf("=== RL deployment layer tests ===\n");
    std::printf("config   : %s\nfixtures : %s\ninference: %s\n\n",
                config_dir.c_str(), fixtures.c_str(),
                rl::haveInferenceBackend() ? "ONNX Runtime" : "NONE (MINI_PI_ENABLE_RL=OFF)");

    JointMapper mapper;
    if (!mapper.loadFromYaml(config_dir + "/mapping.yaml"))
    {
        std::printf("FATAL: cannot load %s/mapping.yaml\n", config_dir.c_str());
        return 2;
    }

    try
    {
        test_resolver(fixtures);
        test_bad_configs(robot_joint_names(mapper));
        test_observation(fixtures, mapper);
        test_action(fixtures, mapper);
        test_model(fixtures);
        test_environment(fixtures, mapper);
        test_production_policy(config_dir, mapper);
        test_policy_gating(config_dir, fixtures, mapper);
    }
    catch (const std::exception& e)
    {
        std::printf("\nFATAL: unexpected exception: %s\n", e.what());
        return 1;
    }

    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
