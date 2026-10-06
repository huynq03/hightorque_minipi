// policy_check -- resolve and fully load one policy package, then print what
// the controller would run. Exit 0 when the package is usable, 1 otherwise.
//
//   policy_check <deploy/robots/<robot>/config dir> [policy_version]
//
// It runs the controller's own RLPolicyRunner::initialize() (package resolver,
// sim-only lock, env build, model load and shape checks, operator
// envelope), so a launcher can fail BEFORE starting roscore and MuJoCo with
// the same reason the node would give -- there is no second resolver in
// bash. An empty version means policy.yaml's default.
//
// Checked as the SIM backend would be: test_policy stays off, so a
// simulation_only package is reported unusable here exactly as in the node.
#include "common/JointMapper.h"
#include "policy/RLPolicyRunner.h"

#include <yaml-cpp/yaml.h>

#include <cstdio>
#include <string>

using namespace deploy;

int main(int argc, char** argv)
{
    if (argc < 2 || argc > 3)
    {
        std::fprintf(stderr, "usage: %s <deploy/robots/<robot>/config dir> [policy_version]\n", argv[0]);
        return 2;
    }
    const std::string cfg = argv[1];
    const std::string version = argc == 3 ? argv[2] : std::string();

    JointMapper mapper;
    if (!mapper.loadFromYaml(cfg + "/mapping.yaml"))
    {
        std::fprintf(stderr, "policy_check: cannot load %s/mapping.yaml\n", cfg.c_str());
        return 1;
    }
    RLPolicyRunner::Options opt;
    try { opt.fsm = YAML::LoadFile(cfg + "/fsm.yaml")["FSM"]; }
    catch (const YAML::Exception& e)
    {
        std::fprintf(stderr, "policy_check: cannot load %s/fsm.yaml: %s\n", cfg.c_str(), e.what());
        return 1;
    }
    opt.base_dir = cfg + "/..";
    opt.robot_joint_names = mapper.jointNames();
    opt.robot_joint_offsets = mapper.appliedJointOffset();
    opt.backend = "sim";
    opt.backend_is_simulation = true;
    opt.version_override = version;

    RLPolicyRunner runner;
    runner.initialize(cfg + "/policy.yaml", opt);
    if (!runner.ready())
    {
        std::fprintf(stderr, "\npolicy_check: policy '%s' is NOT usable:\n  %s\n",
                     version.empty() ? "(policy.yaml default)" : version.c_str(),
                     runner.unavailableReason().c_str());
        return 1;
    }

    const auto& e = runner.operatorEnvelope();
    std::printf("\n=== policy '%s' OK ===%s\n", runner.package().version.c_str(),
                version.empty() ? "  (policy.yaml default)" : "");
    std::printf("  config       : %s\n", runner.package().deploy_yaml.c_str());
    std::printf("  model        : %s\n", runner.package().model.c_str());
    std::printf("  rate         : %.1f Hz\n", 1.0 / runner.stepDt());
    std::printf("  cmd limits   : vx [%g, %g]  vy [%g, %g]  wz [%g, %g]\n", e.limits.vx[0],
                e.limits.vx[1], e.limits.vy[0], e.limits.vy[1], e.limits.wz[0], e.limits.wz[1]);
    std::printf("  key step     : linear %g  yaw %g\n", e.key_linear_step, e.key_yaw_step);
    return 0;
}
