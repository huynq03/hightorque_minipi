// Non-hardware tests for SafetyManager, the shipped YAML configs, the
// PolicyRunner refusal contract, and the FixStand interpolator.
//
// Needs no ROS master and no hardware. Takes the config directory as argv[1].
#include "control/JointMapper.h"
#include "control/LinearInterpolator.h"
#include "control/SafetyGate.h"
#include "control/SafetyManager.h"
#include "control/SystemReadiness.h"
#include "hardware/VendorConflict.h"
#include "policy/PolicyRunner.h"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <cstdio>
#include <string>
#include <thread>

using namespace mini_pi;

namespace
{
int g_failures = 0, g_checks = 0;

void check(bool cond, const std::string& what)
{
    ++g_checks;
    if (!cond) { std::printf("  FAIL %s\n", what.c_str()); ++g_failures; }
}

void check_eq(SafetyFault got, SafetyFault want, const std::string& what)
{
    ++g_checks;
    if (got != want)
    {
        std::printf("  FAIL %-46s got %s want %s\n", what.c_str(),
                    to_string(got), to_string(want));
        ++g_failures;
    }
}

/// A state that passes every enabled check.
RobotState healthy_state()
{
    RobotState s;
    const auto now = Clock::now();
    for (std::size_t i = 0; i < MINI_PI_DOF; ++i)
    {
        s.motor_q[i] = s.motor_dq[i] = s.motor_tau[i] = 0.0;
        s.robot_q[i] = s.robot_dq[i] = s.robot_tau[i] = 0.0;
        s.motor_fault[i] = 0;
        s.motor_age[i] = 0.0;
        s.motor_fresh[i] = true;
    }
    s.stale_motor_count = 0;
    s.motor_valid = true;
    s.motor_stamp = now;
    s.imu_valid = true;
    s.imu_stamp = now;
    s.imu_age = 0.0;
    s.orientation = {0.0, 0.0, 0.0, 1.0};
    s.rpy = {0.0, 0.0, 0.0};
    return s;
}

RobotCommand healthy_command()
{
    RobotCommand c;
    c.valid = true;
    c.stamp = Clock::now();
    c.seq = 1;
    return c;
}

void test_safety(const std::string& safety_yaml)
{
    std::printf("[SafetyManager behaviour]\n");
    SafetyManager sm;
    check(sm.loadFromYaml(safety_yaml), "shipped safety.yaml loads");

    const auto& cfg = sm.config();
    check(cfg.enable_motor_timeout, "motor timeout check enabled");
    check(cfg.enable_imu_timeout, "imu timeout check enabled");
    check(cfg.enable_finite_check, "finite check enabled");
    check(cfg.enable_position_limit, "position limit check enabled");
    check(cfg.enable_command_timeout, "command timeout check enabled");
    // These MUST stay off: no verified limits exist for this robot.
    check(!cfg.enable_velocity_limit, "velocity limit check DISABLED (no verified limits)");
    check(!cfg.enable_torque_limit, "torque limit check DISABLED (no verified limits)");
    check(cfg.q_lower.size() == MINI_PI_DOF, "q_lower has 12 entries");
    check(cfg.q_upper.size() == MINI_PI_DOF, "q_upper has 12 entries");

    // --- checkState -----------------------------------------------------
    check_eq(sm.checkState(healthy_state(), HardwareStatus::Ok).fault,
             SafetyFault::None, "healthy state passes");

    check_eq(sm.checkState(healthy_state(), HardwareStatus::Uninitialized).fault,
             SafetyFault::HardwareFault, "uninitialized hardware -> HARDWARE_FAULT");

    {
        auto s = healthy_state();
        s.motor_fault[7] = 3;
        auto r = sm.checkState(s, HardwareStatus::Fault);
        check_eq(r.fault, SafetyFault::HardwareFault, "motor fault code -> HARDWARE_FAULT");
        check(r.joint == 7, "HARDWARE_FAULT names the offending motor slot");
    }
    {
        // One stale motor out of twelve must be enough.
        auto s = healthy_state();
        s.motor_age[4] = 10.0;
        s.motor_fresh[4] = false;
        s.stale_motor_count = 1;
        auto r = sm.checkState(s, HardwareStatus::Stale);
        check_eq(r.fault, SafetyFault::MotorTimeout, "one stale motor -> MOTOR_TIMEOUT");
        check(r.joint == 4, "MOTOR_TIMEOUT names the stale motor");
    }
    {
        auto s = healthy_state();
        s.motor_valid = false;
        s.motor_fresh[0] = false;
        check_eq(sm.checkState(s, HardwareStatus::Stale).fault,
                 SafetyFault::MotorTimeout, "never-seen motor -> MOTOR_TIMEOUT");
    }
    {
        auto s = healthy_state();
        s.imu_age = 99.0;
        check_eq(sm.checkState(s, HardwareStatus::Ok).fault,
                 SafetyFault::ImuTimeout, "stale IMU -> IMU_TIMEOUT");
    }
    {
        auto s = healthy_state();
        s.imu_valid = false;
        check_eq(sm.checkState(s, HardwareStatus::Ok).fault,
                 SafetyFault::ImuTimeout, "never-received IMU -> IMU_TIMEOUT");
    }
    {
        auto s = healthy_state();
        s.robot_q[9] = std::nan("");
        auto r = sm.checkState(s, HardwareStatus::Ok);
        check_eq(r.fault, SafetyFault::NanState, "NaN joint -> NAN_STATE");
        check(r.joint == 9, "NAN_STATE names the joint");
    }
    {
        auto s = healthy_state();
        s.robot_dq[2] = std::numeric_limits<double>::infinity();
        check_eq(sm.checkState(s, HardwareStatus::Ok).fault,
                 SafetyFault::NanState, "Inf velocity -> NAN_STATE");
    }
    {
        auto s = healthy_state();
        s.rpy[1] = 1.4;   // beyond max_tilt_rad 1.0
        check_eq(sm.checkState(s, HardwareStatus::Ok).fault,
                 SafetyFault::BadOrientation, "excess tilt -> BAD_ORIENTATION");
    }
    {
        // Velocity check is disabled, so a wild velocity must NOT fault.
        auto s = healthy_state();
        s.robot_dq[0] = 1e6;
        check_eq(sm.checkState(s, HardwareStatus::Ok).fault,
                 SafetyFault::None, "disabled velocity check does not fire");
    }

    // --- checkCommand ---------------------------------------------------
    check_eq(sm.checkCommand(healthy_command()).fault,
             SafetyFault::None, "healthy command passes");
    {
        RobotCommand c;   // valid == false
        check_eq(sm.checkCommand(c).fault,
                 SafetyFault::CommandInvalid, "invalid command -> COMMAND_INVALID");
    }
    {
        auto c = healthy_command();
        c.q[3] = std::nan("");
        check_eq(sm.checkCommand(c).fault,
                 SafetyFault::NanCommand, "NaN command -> NAN_COMMAND");
    }
    {
        // An un-refreshed command must expire rather than be reused forever.
        auto c = healthy_command();
        c.stamp = Clock::now() - std::chrono::milliseconds(
                      static_cast<int>(cfg.command_timeout_s * 1000.0) + 50);
        check_eq(sm.checkCommand(c).fault,
                 SafetyFault::CommandTimeout, "stale command -> COMMAND_TIMEOUT");
    }
    {
        auto c = healthy_command();
        c.q[5] = 99.0;
        auto r = sm.checkCommand(c);
        check_eq(r.fault, SafetyFault::JointPositionLimit,
                 "out-of-range command -> JOINT_POSITION_LIMIT");
        check(r.joint == 5, "JOINT_POSITION_LIMIT names the joint");
    }
    {
        // Torque check is disabled: a huge torque must not fault.
        auto c = healthy_command();
        c.torque[1] = 1e6;
        check_eq(sm.checkCommand(c).fault,
                 SafetyFault::None, "disabled torque check does not fire");
    }
    {
        // clampCommand must pull a value back inside the envelope.
        auto c = healthy_command();
        c.q[0] = 99.0;
        const int n = sm.clampCommand(c);
        check(n >= 1, "clampCommand reports it clamped something");
        check(c.q[0] <= cfg.q_upper[0] + 1e-12, "clampCommand respects q_upper");
        check_eq(sm.checkCommand(c).fault, SafetyFault::None,
                 "clamped command then passes");
    }
}

// The shipped production package must now resolve normally. The full value and
// inference contract is covered by test_rl_policy; this test protects the
// top-level PolicyRunner state seen by CtrlFSM.
void test_production_policy(const std::string& config_dir, const JointMapper& mapper)
{
    std::printf("[PolicyRunner production-package contract]\n");

    PolicyRunner::Options opt;
    try { opt.fsm = YAML::LoadFile(config_dir + "/fsm.yaml")["FSM"]; }
    catch (const YAML::Exception&) { check(false, "fsm.yaml loads"); return; }
    opt.base_dir = config_dir + "/..";
    opt.robot_joint_names = mapper.jointNames();
    opt.backend = "sim";
    opt.backend_is_simulation = true;

    PolicyRunner p;
    check(p.initialize(config_dir + "/policy.yaml", opt), "initialize() accepts production policy");
    check(p.package().valid, "config/policy/velocity contains a valid package");
    check(p.package().version == "v0", "production version v0 is selected");
    if (rl::haveInferenceBackend())
    {
        check(p.status() == PolicyStatus::Ready, "status is POLICY_READY");
        check(p.ready(), "ready() is true");
        check(std::string(to_string(p.status())) == "POLICY_READY",
              "status token is POLICY_READY");
        check(p.describe().find("POLICY_READY") != std::string::npos,
              "describe() reports POLICY_READY");
    }
    else
    {
        check(!p.ready(), "without ONNX Runtime the production policy stays unavailable");
    }
    check(!p.start(), "start() refuses without a state provider");
    check(!p.running(), "no thread is running");
    check(!p.latest().valid, "latest() target is invalid before the policy starts");
    for (double v : p.latest().cmd.q) check(v == 0.0, "unused target payload is inert");
    p.stop();   // must be safe even though nothing was started
    check(true, "stop() is safe when never started");
}

void test_interpolator()
{
    std::printf("[LinearInterpolator / FixStand semantics]\n");
    std::vector<double> ts = {0.0, 3.0};
    std::vector<std::vector<double>> qs(2, std::vector<double>(MINI_PI_DOF, 0.0));
    // start pose = measured pose, target = robot zero (the nominal pose)
    for (std::size_t j = 0; j < MINI_PI_DOF; ++j) qs[0][j] = 0.4 + 0.1 * static_cast<double>(j);

    auto at0 = linear_interpolate(0.0, ts, qs);
    check(at0.size() == MINI_PI_DOF, "interpolation width");
    for (std::size_t j = 0; j < MINI_PI_DOF; ++j)
        check(std::fabs(at0[j] - qs[0][j]) < 1e-12,
              "t=0 equals the measured start pose (no jump)");

    auto mid = linear_interpolate(1.5, ts, qs);
    for (std::size_t j = 0; j < MINI_PI_DOF; ++j)
        check(std::fabs(mid[j] - 0.5 * qs[0][j]) < 1e-12, "midpoint is the average");

    auto at_end = linear_interpolate(3.0, ts, qs);
    for (double v : at_end) check(std::fabs(v) < 1e-12, "t=end reaches robot_q == 0");

    auto past = linear_interpolate(99.0, ts, qs);
    for (double v : past) check(std::fabs(v) < 1e-12, "clamps past the last keyframe");
    auto before = linear_interpolate(-5.0, ts, qs);
    for (std::size_t j = 0; j < MINI_PI_DOF; ++j)
        check(std::fabs(before[j] - qs[0][j]) < 1e-12, "clamps before the first keyframe");

    // Monotone, bounded progression -- no overshoot.
    double prev = at0[0];
    for (double t = 0.0; t <= 3.0; t += 0.05)
    {
        const double v = linear_interpolate(t, ts, qs)[0];
        check(v <= prev + 1e-12, "monotone decrease toward the target");
        check(v >= -1e-12 && v <= qs[0][0] + 1e-12, "stays within [target, start]");
        prev = v;
    }
}

void test_fsm_config(const std::string& fsm_yaml)
{
    std::printf("[fsm.yaml structure]\n");
    YAML::Node root;
    try { root = YAML::LoadFile(fsm_yaml); }
    catch (const YAML::Exception&) { check(false, "fsm.yaml loads"); return; }

    auto fsm = root["FSM"];
    check(static_cast<bool>(fsm), "FSM block present");
    auto enabled = fsm["_"];
    check(static_cast<bool>(enabled), "FSM._ block present");

    // Every state the non-policy flow needs must exist.
    for (const char* name : {"Passive", "FixStand", "Velocity"})
        check(static_cast<bool>(enabled[name]), std::string("state ") + name + " enabled");

    // The FSM contains ROBOT OPERATING MODES only. `Init` moved to the startup
    // readiness wait (robot.yaml `startup:`) and `Fault` to the runtime safety
    // gate (safety.yaml `fault_response:`); neither may come back as a state,
    // and no stale transition may name one.
    for (const char* gone : {"Init", "Fault"})
    {
        check(!enabled[gone], std::string("state ") + gone + " is NOT an FSM state");
        check(!fsm[gone], std::string("no FSM.") + gone + " config block");
    }
    for (auto it = enabled.begin(); it != enabled.end(); ++it)
    {
        const std::string name = it->first.as<std::string>();
        const YAML::Node tr = fsm[name]["transitions"];
        if (!tr) continue;
        for (const auto& t : tr)
        {
            const std::string target = t.first.as<std::string>();
            check(static_cast<bool>(enabled[target]),
                  name + " -> " + target + " names an enabled state");
            check(t.second.as<std::string>() != "reset",
                  name + " -> " + target + " does not use the fault-reset trigger");
        }
    }

    // Ids must be unique and non-zero (0 means "no transition" in CtrlFSM).
    std::set<int> ids;
    for (auto it = enabled.begin(); it != enabled.end(); ++it)
    {
        const int id = it->second["id"].as<int>();
        check(id != 0, "state id is non-zero");
        check(ids.insert(id).second, "state id is unique");
    }

    // The required flow PASSIVE <-> FIX_STAND <-> VELOCITY.
    check(fsm["Passive"]["transitions"]["FixStand"].as<std::string>() == "stand",
          "Passive -> FixStand on 'stand'");
    check(fsm["FixStand"]["transitions"]["Passive"].as<std::string>() == "passive",
          "FixStand -> Passive on 'passive'");
    check(fsm["FixStand"]["transitions"]["Velocity"].as<std::string>() == "rl",
          "FixStand -> Velocity on 'rl'");
    check(fsm["Velocity"]["transitions"]["Passive"].as<std::string>() == "passive",
          "Velocity -> Passive on 'passive'");
    // RL is reachable only from FixStand: Passive must not offer it directly.
    check(!fsm["Passive"]["transitions"]["Velocity"],
          "Passive does not transition straight into RL");

    // Gain arrays must all be 12 wide.
    for (const char* st : {"Passive", "FixStand", "Velocity"})
    {
        for (const char* key : {"kp", "kd"})
        {
            if (fsm[st][key])
                check(fsm[st][key].as<std::vector<double>>().size() == MINI_PI_DOF,
                      std::string(st) + "." + key + " has 12 entries");
        }
    }
    check(fsm["Passive"]["kd"].as<std::vector<double>>()[0] == 1.0,
          "Passive kd matches HtdwMotor::protectMotor() (1.0)");
    check(fsm["FixStand"]["kp"].as<std::vector<double>>()[0] == 80.0,
          "FixStand kp matches pi_pd_config.yaml (80)");
    check(fsm["FixStand"]["duration_s"].as<double>() > 0.0, "FixStand duration is positive");
}

/// The runtime fault RESPONSE moved out of fsm.yaml into safety.yaml. Same
/// values, same defaults; assert they survived the move.
void test_fault_response_config(const std::string& safety_yaml)
{
    std::printf("[safety.yaml fault_response]\n");
    YAML::Node n;
    try { n = YAML::LoadFile(safety_yaml); }
    catch (const YAML::Exception&) { check(false, "safety.yaml loads"); return; }

    const YAML::Node fr = n["fault_response"];
    check(static_cast<bool>(fr), "fault_response block present");
    if (!fr) return;
    check(fr["kd"].as<std::vector<double>>().size() == MINI_PI_DOF,
          "fault_response.kd has 12 entries");
    check(fr["kd"].as<std::vector<double>>()[0] == 1.0,
          "fault_response kd matches HtdwMotor::protectMotor() (1.0)");
    check(fr["hard_stop"].as<bool>() == false,
          "fault response defaults to damping, not the firmware stop");

    SafetyGate gate;
    check(gate.loadFromYaml(safety_yaml), "SafetyGate loads the shipped safety.yaml");
    check(gate.config().kd[0] == 1.0, "SafetyGate picked up kd");
    check(gate.config().hard_stop == false, "SafetyGate picked up hard_stop");
    // A freshly constructed gate is disarmed, so a startup-time verdict cannot
    // latch. This is the Init-vs-global-fault conflict, asserted.
    check(!gate.armed(), "a new SafetyGate is DISARMED");
    SafetyReport boot;
    boot.fault = SafetyFault::MotorTimeout;
    boot.detail = "no motor packet yet";
    gate.raise(boot);
    check(!gate.faulted(), "a fault raised before arm() does NOT latch");
}

/// robot.yaml grew the startup readiness block that replaced FSM.Init.
void test_startup_config(const std::string& robot_yaml)
{
    std::printf("[robot.yaml startup]\n");
    YAML::Node r;
    try { r = YAML::LoadFile(robot_yaml); }
    catch (const YAML::Exception&) { check(false, "robot.yaml loads"); return; }

    const YAML::Node st = r["startup"];
    check(static_cast<bool>(st), "startup block present");
    if (!st) return;

    const ReadinessConfig c = loadReadinessConfig(st);
    check(c.require_imu, "startup requires a valid IMU sample");
    check(c.require_fresh_s > 0.0, "startup freshness threshold is positive");
    check(c.timeout_s <= 0.0, "startup waits indefinitely by default");
    check(c.poll_dt_s > 0.0, "startup poll period is positive");

    // Defaults must survive an entirely absent block.
    const ReadinessConfig d = loadReadinessConfig(YAML::Node());
    check(d.require_imu && d.require_fresh_s == 0.5,
          "an absent startup block keeps the built-in defaults");
}

void test_robot_config(const std::string& robot_yaml)
{
    std::printf("[robot.yaml]\n");
    YAML::Node r;
    try { r = YAML::LoadFile(robot_yaml); }
    catch (const YAML::Exception&) { check(false, "robot.yaml loads"); return; }

    check(r["dry_run"] && r["dry_run"].as<bool>() == true,
          "dry_run defaults to TRUE");
    check(r["control_hz"].as<double>() == 1000.0,
          "control_hz is 1000 (pd_ctrl_f from walk/lr.yaml)");
    check(r["debug_hz"].as<double>() <= 10.0,
          "debug_hz is far below control frequency");
    // Freshness thresholds must live in safety.yaml only.
    check(!r["motor_timeout_s"], "motor_timeout_s is NOT duplicated in robot.yaml");
    check(!r["imu_timeout_s"], "imu_timeout_s is NOT duplicated in robot.yaml");

    // IMU mount: identity until measured on the robot. A non-identity value
    // here without a recorded measurement would be an invented fact.
    check(r["imu"] && r["imu"]["mount_rpy"], "robot.yaml declares imu.mount_rpy");
    if (r["imu"] && r["imu"]["mount_rpy"])
    {
        const auto v = r["imu"]["mount_rpy"].as<std::vector<double>>();
        check(v.size() == 3 && v[0] == 0.0 && v[1] == 0.0 && v[2] == 0.0,
              "imu.mount_rpy is the identity placeholder (not yet measured)");
    }
    check(r["hightorque"] && r["hightorque"]["feedback_query"].as<std::string>() == "state2",
          "hightorque feedback_query is 'state2' (request-driven; SDK choice for fun_v >= 4)");
    check(r["hightorque"]["feedback_query_period_s"].as<double>() > 0.0 &&
          r["hightorque"]["feedback_query_period_s"].as<double>() <=
              0.5 * 0.1 /* half of safety.yaml motor_timeout_s */,
          "feedback query period is well inside the motor timeout");
    check(r["keyboard"]["speed"].as<double>() > 0.2,
          "keyboard speed is above the policy's 0.2 m/s linear deadband");
}

bool near3(const std::array<double, 3>& a, const std::array<double, 3>& b, double tol)
{
    return std::fabs(a[0] - b[0]) < tol && std::fabs(a[1] - b[1]) < tol &&
           std::fabs(a[2] - b[2]) < tol;
}

void test_imu_mount()
{
    std::printf("[IMU sensor -> base mount transform]\n");
    const double pi = M_PI;

    // Identity leaves everything bit-identical.
    {
        std::array<double, 4> q = rpy_to_quat_xyzw({0.1, -0.2, 2.5});
        std::array<double, 3> w{0.3, -0.4, 0.5};
        const auto q0 = q; const auto w0 = w;
        apply_imu_mount({0, 0, 0}, q, w);
        check(q == q0 && w == w0, "identity mount changes nothing");
    }
    // rpy <-> quat round trip (the Euler convention both backends use).
    {
        std::array<double, 3> rpy{};
        quat_xyzw_to_rpy(rpy_to_quat_xyzw({0.3, -0.2, 1.1}), rpy);
        check(near3(rpy, {0.3, -0.2, 1.1}, 1e-12), "rpy -> quat -> rpy round-trips");
    }
    // Sensor mounted upside down about x. Robot upright and yawed 0.7 rad:
    // the sensor sees roll = pi; after the mount the base must read [0, 0, 0.7].
    {
        const auto q_wb = rpy_to_quat_xyzw({0.0, 0.0, 0.7});
        const auto q_bs = rpy_to_quat_xyzw({pi, 0.0, 0.0});
        auto q = quat_mul_xyzw(q_wb, q_bs);        // what the sensor reports
        std::array<double, 3> rs{};
        quat_xyzw_to_rpy(q, rs);
        check(std::fabs(std::fabs(rs[0]) - pi) < 1e-9, "upside-down sensor reads roll ~ +-pi");
        // Base yawing at +1 rad/s: the flipped sensor measures -1 about its z.
        std::array<double, 3> w = quat_rotate_xyzw({-q_bs[0], -q_bs[1], -q_bs[2], q_bs[3]},
                                                   {0.0, 0.0, 1.0});
        check(near3(w, {0.0, 0.0, -1.0}, 1e-12), "flipped sensor sees yaw rate negated");
        apply_imu_mount({pi, 0.0, 0.0}, q, w);
        std::array<double, 3> rb{};
        quat_xyzw_to_rpy(q, rb);
        check(near3(rb, {0.0, 0.0, 0.7}, 1e-9), "mount {pi,0,0}: base rpy = [0, 0, 0.7]");
        check(near3(w, {0.0, 0.0, 1.0}, 1e-12), "mount {pi,0,0}: base yaw rate = +1");
    }
    // Sensor rotated +90 deg about z: base roll rate appears on sensor -y.
    {
        const auto q_bs = rpy_to_quat_xyzw({0.0, 0.0, pi / 2});
        std::array<double, 3> w = quat_rotate_xyzw({-q_bs[0], -q_bs[1], -q_bs[2], q_bs[3]},
                                                   {1.0, 0.0, 0.0});
        check(near3(w, {0.0, -1.0, 0.0}, 1e-12), "yaw-90 sensor sees base roll rate on -y");
        auto q = quat_mul_xyzw(rpy_to_quat_xyzw({0.2, 0.0, 0.0}), q_bs);
        apply_imu_mount({0.0, 0.0, pi / 2}, q, w);
        std::array<double, 3> rb{};
        quat_xyzw_to_rpy(q, rb);
        check(near3(rb, {0.2, 0.0, 0.0}, 1e-9), "mount {0,0,pi/2}: base roll recovered");
        check(near3(w, {1.0, 0.0, 0.0}, 1e-12), "mount {0,0,pi/2}: base roll rate recovered");
    }
}

void test_vendor_conflict()
{
    std::printf("[vendor controller / motor-serial conflict scan]\n");
    namespace fs = std::filesystem;
    const fs::path root = fs::temp_directory_path() / "mini_pi_fake_proc";
    fs::remove_all(root);
    auto proc = [&](int pid, const std::vector<std::string>& argv) {
        fs::create_directories(root / std::to_string(pid) / "fd");
        std::ofstream f(root / std::to_string(pid) / "cmdline", std::ios::binary);
        for (const auto& a : argv) { f << a; f.put('\0'); }
    };

    proc(100, {"/usr/bin/bash"});
    proc(101, {"/opt/ros/noetic/lib/yesense_imu/yesense_imu_node", "__name:=yesense_imu_node"});
    proc(102, {"/usr/bin/python3", "/opt/ros/noetic/bin/roslaunch", "mini_pi_fsm",
               "mini_pi_fsm.launch", "backend:=hightorque"});
    proc(999, {"/home/x/mini_pi_fsm/lib/mini_pi_fsm/mini_pi_fsm_node"});
    fs::create_symlink("/dev/ttyACM0", root / "999" / "fd" / "7");   // self
    check(findVendorConflicts(root.string(), 999).empty(),
          "own process, own launch and a plain IMU driver are not conflicts");

    proc(200, {"/home/hightorque/install/lib/sim2real_master/sim2real_master_node",
               "__name:=sim2real_master_node"});
    proc(201, {"/usr/bin/python3", "/opt/ros/noetic/bin/roslaunch", "sim2real_master",
               "joy_control_pi.launch"});
    proc(202, {"/home/hightorque/install/lib/livelybot_bringup/motor_feedback"});
    proc(203, {"/usr/local/bin/some_other_tool"});
    fs::create_symlink("/dev/ttyACM0", root / "203" / "fd" / "4");
    proc(204, {"/home/hightorque/lr_control/lr_control_node"});
    const auto c = findVendorConflicts(root.string(), 999);
    auto has = [&](int pid) {
        for (const auto& x : c) if (x.pid == pid) return true;
        return false;
    };
    check(has(200), "sim2real_master_node is a conflict");
    check(has(201), "the vendor roslaunch (joy_control_pi) is a conflict");
    check(has(202), "a livelybot_bringup tool is a conflict");
    check(has(203), "any other process holding /dev/ttyACM* is a conflict");
    check(has(204), "lr_control_node is a conflict");
    check(!has(100) && !has(101) && !has(102) && !has(999), "no false positives");
    check(findVendorConflicts((root / "missing").string(), 1).empty(),
          "an unreadable proc root yields no conflicts rather than crashing");
    fs::remove_all(root);
}
} // namespace

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::printf("usage: %s <config-dir>\n", argv[0]);
        return 2;
    }
    const std::string dir = argv[1];
    std::printf("=== SafetyManager / config / production-policy tests ===\n");

    JointMapper mapper;
    if (!mapper.loadFromYaml(dir + "/mapping.yaml"))
    {
        std::printf("  FATAL: cannot load mapping.yaml\n");
        return 1;
    }

    test_safety(dir + "/safety.yaml");
    test_production_policy(dir, mapper);
    test_interpolator();
    test_fsm_config(dir + "/fsm.yaml");
    test_fault_response_config(dir + "/safety.yaml");
    test_startup_config(dir + "/robot.yaml");
    test_robot_config(dir + "/robot.yaml");
    test_imu_mount();
    test_vendor_conflict();

    std::printf("=== %d checks, %d failures ===\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
