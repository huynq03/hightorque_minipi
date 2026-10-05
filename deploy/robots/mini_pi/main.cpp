// Mini-Pi deployment node.
//
// Structural counterpart of unitree_rl_mjlab/deploy/robots/g1/main.cpp: setup
// and diagnostics only. No control logic lives here -- the control loop itself
// is owned by CtrlFSM.
#include "CtrlFSM.h"
#include "FSMState.h"
#include "State_FixStand.h"
#include "State_Passive.h"
#include "State_RLBase.h"
#include "Safety.h"
#include "SystemReadiness.h"
#include "HardwareBackend.h"
#include "VendorConflict.h"

#include <ros/package.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>

#include <unistd.h>

using namespace mini_pi;

namespace
{
std::string resolve(const ros::NodeHandle& pnh, const std::string& param,
                    const std::string& fallback_rel)
{
    std::string path;
    if (pnh.getParam(param, path) && !path.empty()) return path;
    return ros::package::getPath("mini_pi_fsm") + "/" + fallback_rel;
}

/// Low-rate diagnostic dump. Deliberately NOT called from the control loop.
std::string diagnostics(const ControlContext& ctx, const CtrlFSM& fsm)
{
    // A consistent copy from the last completed tick -- never the live
    // structures, which the FSM thread rewrites every millisecond.
    RobotState s;
    RobotCommand c;
    if (!fsm.snapshot(s, c)) return "\n--- Mini-Pi status: FSM started, no completed tick yet ---\n";

    std::ostringstream os;
    os << std::fixed << std::setprecision(4);
    os << "\n--- Mini-Pi status --------------------------------------------------\n";
    os << "backend          : " << ctx.hardware->backendName() << "\n";
    os << "dry_run          : " << (ctx.hardware->dryRun() ? "TRUE (commands computed, NOT transmitted; read-only state queries only)"
                                                           : "FALSE (LIVE MOTORS)") << "\n";
    // These three lines are deliberately separate. The FSM state is a ROBOT
    // OPERATING MODE and is only ever Passive / FixStand / Velocity;
    // readiness is a startup property; safety is a runtime system condition.
    os << "FSM state        : " << fsm.currentStateName()
       << "   last cycle " << fsm.lastCycleSeconds() * 1e3 << " ms, overruns "
       << fsm.overruns() << "\n";
    os << "system readiness : " << (ctx.safety->armed() ? "READY (FSM running)"
                                                      : "NOT READY (FSM not started)")
       << "\n";
    {
        const SafetyReport f = ctx.safety->fault();
        os << "safety status    : " << (ctx.safety->faulted() ? "FAULTED" : "healthy");
        if (ctx.safety->faulted())
        {
            os << "  latched " << ctx.safety->faultAge() << " s"
               << "  (FSM output overridden with a damping hold; press reset to clear)";
        }
        os << "\n";
        os << "fault reason     : " << to_string(f.fault);
        if (!f.ok())
        {
            os << "  joint=" << f.joint << "  value=" << f.value
               << "  (" << f.detail << ")";
        }
        os << "\n";
    }
    os << "hardware         : " << to_string(ctx.hardware->status())
       << "   motors=" << ctx.hardware->motorCount()
       << "  stale=" << s.stale_motor_count << "\n";
    os << "policy           : " << to_string(ctx.policy->status());
    if (ctx.policy->ready())
    {
        const double period = ctx.policy->measuredPeriod();
        const PolicyTiming tm = ctx.policy->timing();
        os << "  pkg=" << ctx.policy->package().version
           << "  thread=" << (ctx.policy->running() ? "RUNNING" : "stopped")
           << "  steps=" << ctx.policy->steps()
           << "  rate=" << (period > 0.0 ? 1.0 / period : 0.0) << " Hz"
           << "  step_avg=" << tm.mean_update * 1e3 << " ms"
           << "  step_max=" << tm.max_update * 1e3 << " ms";
        const auto t = ctx.policy->latest();
        os << "  cmd_age=" << (t.valid ? age_seconds(t.stamp) : -1.0) << " s";
        if (ctx.policy->failed()) os << "  FAILED: " << ctx.policy->failureReason();
        os << "\npolicy timing    : period last/min/max " << tm.last_period * 1e3 << "/"
           << tm.min_period * 1e3 << "/" << tm.max_period * 1e3 << " ms  update last/max "
           << tm.last_update * 1e3 << "/" << tm.max_update * 1e3 << " ms  overruns " << tm.overruns
           << "\nsafety clamp     : last mask 0x" << std::hex << ctx.safety->lastClampMask()
           << std::dec << "  clamped commands " << ctx.safety->clampedCommands();
        // Why Velocity would be refused right now (e.g. an observation the
        // selected policy needs has no valid source on this backend).
        if (!ctx.policy->running())
            if (const std::string b = ctx.policy->entryBlocker(s); !b.empty())
                os << "\nRL entry blocked : " << b;
        if (t.valid)
        {
            const auto action_bounds = std::minmax_element(t.action.begin(),
                                                            t.action.end());
            const auto target_bounds = std::minmax_element(t.cmd.q.begin(), t.cmd.q.end());
            os << "\npolicy sample    : obs=[" << t.observation_min << ","
               << t.observation_max << "] finite=" << (t.observation_finite ? "yes" : "NO")
               << " action=[" << *action_bounds.first << ","
               << *action_bounds.second << "] q_target=[" << *target_bounds.first << ","
               << *target_bounds.second << "]";
            // The newest observation frame, sliced into named fields by the
            // policy itself. On the robot in dry run this is how IMU signs and
            // joint signs are seen exactly as the network would see them.
            os << ctx.policy->describeFrame(ctx.policy->latestFrame());
            os << "\n  action (policy order)  :";
            for (double v : t.action) os << " " << v;
            os << "\n  q_target (robot order) :";
            for (double v : t.cmd.q) os << " " << v;
        }
    }
    else
    {
        os << "  (" << ctx.policy->unavailableReason() << ")";
    }
    os << "\n";
    // The operator's velocity intent exactly as State_RLBase hands it to the
    // policy, plus which of the two sources won. Without this the only way to
    // tell a dead stick from a correctly-zero one is to read the ONNX input.
    {
        const VelocityCommand v = ctx.input->velocityCommand();
        const char* src = "none";
        switch (ctx.input->velocitySource())
        {
            case InputManager::VelocitySource::Joy:      src = "joy";      break;
            case InputManager::VelocitySource::Keyboard: src = "keyboard"; break;
            case InputManager::VelocitySource::CmdVel:   src = "cmd_vel";  break;
            case InputManager::VelocitySource::None:     src = "none";     break;
        }
        os << "operator cmd     : vx=" << v.vx << " vy=" << v.vy
           << " dyaw=" << v.dyaw << "  source=" << src
           << "  joy=" << (ctx.input->joyConnected() ? "seen" : "never seen")
           << "  keyboard=" << (ctx.input->keyboardActive() ? "on" : "off") << "\n";
    }
    os << "mapping mode     : " << to_string(ctx.mapper->mode()) << "\n";
    os << "command          : seq=" << c.seq << " valid=" << (c.valid ? "yes" : "no")
       << " age=" << (c.valid ? age_seconds(c.stamp) : -1.0) << " s\n";
    os << "imu raw (sensor) : age=" << s.imu_age << " s"
       << "  quat(x,y,z,w)=[" << s.imu_raw_orientation[0] << " " << s.imu_raw_orientation[1]
       << " " << s.imu_raw_orientation[2] << " " << s.imu_raw_orientation[3] << "]"
       << "  gyro=[" << s.imu_raw_angular_velocity[0] << " " << s.imu_raw_angular_velocity[1]
       << " " << s.imu_raw_angular_velocity[2] << "]\n";
    os << "imu base frame   : rpy=[" << s.rpy[0] << " " << s.rpy[1] << " " << s.rpy[2] << "]"
       << "  gyro=[" << s.angular_velocity[0] << " " << s.angular_velocity[1] << " "
       << s.angular_velocity[2] << "]"
       << "  (after imu.mount_rpy; upright robot => roll~0 pitch~0)\n";

    // Both coordinate systems are printed on both backends, so that a real
    // log and a simulated log can be compared column for column without
    // touching the FSM. `mcmd_q` is what actually goes on the wire / on the
    // CAN bus, i.e. the output of JointMapper::robotToMotor().
    MotorCommand mcmd;
    if (c.valid) ctx.mapper->robotToMotor(c, mcmd);

    // Motor identity as the SDK enumerated it (canboard -> canport -> id).
    // Empty on backends that have no such notion.
    const auto& hw_names = ctx.hardware->motorNames();
    const auto& hw_ids   = ctx.hardware->motorIds();

    const auto& names = ctx.mapper->jointNames();
    const auto& dir   = ctx.mapper->direction();
    const auto& off   = ctx.mapper->jointOffset();
    os << "idx joint                slot id sdk_name       dir  offset  age   flt "
          "   raw_q   raw_dq  raw_tau    rbt_q   rbt_dq    cmd_q   mcmd_q    kp    kd\n";
    for (std::size_t j = 0; j < MINI_PI_DOF; ++j)
    {
        const int m = ctx.mapper->mapIndex()[j];
        const std::size_t mu = static_cast<std::size_t>(m);
        const int id = mu < hw_ids.size() ? hw_ids[mu] : -1;
        const std::string sdk = mu < hw_names.size() ? hw_names[mu] : std::string("-");
        char line[400];
        std::snprintf(line, sizeof(line),
            "%2zu  %-20s %4d %2d %-14.14s %+2d %+7.3f %5.3f %4d %8.4f %8.4f %8.4f "
            "%8.4f %8.4f %8.4f %8.4f %5.1f %5.2f %s\n",
            j, names[j].c_str(), m, id, sdk.c_str(), dir[j], off[j],
            std::isfinite(s.motor_age[mu]) ? s.motor_age[mu] : 9.999, s.motor_fault[mu],
            s.motor_q[mu], s.motor_dq[mu], s.motor_tau[mu],
            s.robot_q[j], s.robot_dq[j],
            c.q[j], mcmd.q[mu], c.kp[j], c.kd[j],
            (s.motor_fresh[mu] ? "fresh" : "STALE"));
        os << line;
    }
    os << "---------------------------------------------------------------------\n";
    return os.str();
}
} // namespace

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mini_pi_fsm_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // Callbacks (IMU, joy, cmd_vel) are serviced here, independently of the
    // control loop. ros::spinOnce() is never called at control frequency.
    ros::AsyncSpinner spinner(2);
    spinner.start();

    std::cout << " --- HighTorque Mini-Pi --- \n"
                 "     12-dof FSM Controller \n";

    // ---- configuration ---------------------------------------------------
    const std::string robot_yaml   = resolve(pnh, "robot_config",   "config/robot.yaml");
    const std::string mapping_yaml = resolve(pnh, "mapping_config", "config/mapping.yaml");
    const std::string fsm_yaml     = resolve(pnh, "fsm_config",     "config/fsm.yaml");
    const std::string safety_yaml  = resolve(pnh, "safety_config",  "config/safety.yaml");
    const std::string policy_yaml  = resolve(pnh, "policy_config",  "config/policy.yaml");

    YAML::Node robot_cfg;
    try { robot_cfg = YAML::LoadFile(robot_yaml); }
    catch (const YAML::Exception& e)
    {
        ROS_FATAL("Cannot load robot config '%s': %s", robot_yaml.c_str(), e.what());
        return 1;
    }

    // dry_run defaults to true and can only be turned off deliberately.
    bool dry_run = robot_cfg["dry_run"] ? robot_cfg["dry_run"].as<bool>() : true;
    pnh.param("dry_run", dry_run, dry_run);
    if (!dry_run)
    {
        ROS_WARN("dry_run is DISABLED: this process will drive the motors. "
                 "Ensure sim2real_master_node is NOT running -- only one process "
                 "may own the motor output path.");
    }

    // Startup readiness thresholds. Read here with the rest of robot.yaml; the
    // wait itself happens after every component exists.
    const ReadinessConfig readiness = loadReadinessConfig(robot_cfg["startup"]);

    const double control_hz = robot_cfg["control_hz"] ? robot_cfg["control_hz"].as<double>()
                                                      : 1000.0;
    const double debug_hz   = robot_cfg["debug_hz"] ? robot_cfg["debug_hz"].as<double>() : 1.0;
    if (!(control_hz > 0.0))
    {
        ROS_FATAL("control_hz must be > 0");
        return 1;
    }

    // ---- components ------------------------------------------------------
    auto mapper = std::make_unique<JointMapper>();
    if (!mapper->loadFromYaml(mapping_yaml)) return 1;
    ROS_INFO_STREAM("\n" << mapper->describe());

    // Every runtime check, the fault latch, and the single exit from the FSM
    // to the bus. It starts DISARMED, so nothing can be transmitted and no
    // fault can latch until startup readiness has succeeded below.
    auto safety = std::make_unique<Safety>();
    if (!safety->loadFromYaml(safety_yaml)) return 1;

    // ---- hardware backend ------------------------------------------------
    // The ONLY place in deploy that chooses which robot is underneath. The
    // concrete classes are named in src/HardwareBackend.cpp and
    // nowhere else, so a build without the aarch64 HighTorque SDK simply omits
    // that backend and still links. Nothing downstream -- not CtrlFSM, not a
    // single FSM state, not Safety, not JointMapper -- can tell the
    // difference. This is the Mini-Pi equivalent of Unitree pointing the same
    // deploy binary at a different DDS peer.
    std::string backend = robot_cfg["backend"] ? robot_cfg["backend"].as<std::string>()
                                               : std::string("hightorque");
    pnh.param("backend", backend, backend);

    BackendOptions bo;
    bo.sim.dry_run = dry_run;
    // Freshness thresholds always come from safety.yaml on BOTH backends, so
    // the hardware layer and Safety never disagree about "stale".
    bo.sim.link.motor_timeout_s = safety->config().motor_timeout_s;
    bo.sim.link.imu_timeout_s   = safety->config().imu_timeout_s;
    if (robot_cfg["imu_topic"]) bo.imu_topic = robot_cfg["imu_topic"].as<std::string>();
    if (robot_cfg["motor_watchdog_ms"])
        bo.motor_watchdog_ms = robot_cfg["motor_watchdog_ms"].as<int>();
    if (const YAML::Node im = robot_cfg["imu"])
    {
        if (im["mount_rpy"])
        {
            const auto v = im["mount_rpy"].as<std::vector<double>>();
            if (v.size() != 3)
            {
                ROS_FATAL("robot.yaml imu.mount_rpy must have 3 values [roll, pitch, yaw]");
                return 1;
            }
            bo.imu_mount_rpy = {v[0], v[1], v[2]};
        }
    }
    if (const YAML::Node hw = robot_cfg["hightorque"])
    {
        if (hw["feedback_query"]) bo.feedback_query = hw["feedback_query"].as<std::string>();
        if (hw["feedback_query_period_s"])
            bo.feedback_query_period_s = hw["feedback_query_period_s"].as<double>();
    }
    if (const YAML::Node sb = robot_cfg["sim_backend"])
    {
        if (sb["bridge_ip"])     bo.sim.link.bridge_ip     = sb["bridge_ip"].as<std::string>();
        if (sb["lowcmd_port"])   bo.sim.link.lowcmd_port   = sb["lowcmd_port"].as<int>();
        if (sb["lowstate_port"]) bo.sim.link.lowstate_port = sb["lowstate_port"].as<int>();
        if (sb["wait_for_bridge_s"])
            bo.sim.wait_for_bridge_s = sb["wait_for_bridge_s"].as<double>();
    }

    // One owner of the motor bus. The vendor stack auto-starts at login on the
    // Mini-Pi and respawns; refuse rather than share the serial link with it.
    // Applies to dry_run too: a second reader steals our feedback replies.
    if (backend == "hightorque")
    {
        const auto conflicts = findVendorConflicts("/proc", static_cast<int>(::getpid()));
        if (!conflicts.empty())
        {
            std::ostringstream os;
            for (const auto& c : conflicts) os << "\n    pid " << c.pid << ": " << c.what;
            ROS_FATAL("Refusing backend 'hightorque': the HighTorque vendor controller (or "
                      "another motor-serial owner) is running.%s\n  Stop it first (see "
                      "scripts/check-vendor-conflict.sh), then relaunch.", os.str().c_str());
            return 1;
        }
    }

    std::string backend_error;
    std::unique_ptr<HardwareInterface> hardware =
        createHardwareBackend(backend, nh, bo, backend_error);
    if (!hardware)
    {
        ROS_FATAL("Cannot start backend '%s': %s", backend.c_str(), backend_error.c_str());
        return 1;
    }

    auto input = std::make_unique<InputManager>();
    InputManager::Config in_cfg;
    if (robot_cfg["joy_topic"]) in_cfg.joy_topic = robot_cfg["joy_topic"].as<std::string>();
    if (robot_cfg["cmd_vel_topic"])
        in_cfg.cmd_vel_topic = robot_cfg["cmd_vel_topic"].as<std::string>();
    // Stick mapping. Every key is optional; a missing one keeps the
    // SOURCE-VERIFIED default in InputManager::Config rather than a guess.
    // The command ENVELOPE (ranges, key steps) is not here: it belongs to the
    // selected policy package and is applied below, once the policy is known.
    if (const YAML::Node j = robot_cfg["joy"])
    {
        auto opt_i = [&](const char* k, int& dst) { if (j[k]) dst = j[k].as<int>(); };
        auto opt_d = [&](const char* k, double& dst) { if (j[k]) dst = j[k].as<double>(); };
        opt_i("axis_vx",   in_cfg.axis_vx);
        opt_i("axis_vy",   in_cfg.axis_vy);
        opt_i("axis_dyaw", in_cfg.axis_dyaw);
        opt_d("deadzone",  in_cfg.joy_deadzone);
        opt_d("timeout_s", in_cfg.joy_timeout_s);
        if (j["ranges"])
            ROS_WARN("robot.yaml joy.ranges is IGNORED: command ranges now come from the "
                     "selected policy package (its command limits).");
    }
    if (const YAML::Node k = robot_cfg["keyboard"])
    {
        if (k["enable"]) in_cfg.enable_keyboard = k["enable"].as<bool>();
        if (k["speed"] || k["turn"])
            ROS_WARN("robot.yaml keyboard.speed/turn are IGNORED: key steps now come from "
                     "the selected policy package (deploy.yaml operator.keyboard).");
        if (const YAML::Node m = k["keys"])
        {
            auto opt_k = [&](const char* name, std::string& dst) {
                if (m[name]) dst = m[name].as<std::string>();
            };
            opt_k("forward",   in_cfg.key_forward);
            opt_k("backward",  in_cfg.key_backward);
            opt_k("left",      in_cfg.key_left);
            opt_k("right",     in_cfg.key_right);
            opt_k("yaw_left",  in_cfg.key_yaw_left);
            opt_k("yaw_right", in_cfg.key_yaw_right);
            opt_k("stop",      in_cfg.key_stop);
            opt_k("passive",   in_cfg.key_passive);
            opt_k("stand",     in_cfg.key_stand);
            opt_k("rl",        in_cfg.key_rl);
            opt_k("reset",     in_cfg.key_reset);
        }
    }
    // ---- FSM configuration -----------------------------------------------
    // Loaded here, before RLPolicyRunner, because `policy_dir` is declared
    // per-state in fsm.yaml (FSM.<state>.policy_dir) and RLPolicyRunner needs it.
    try { fsm_config::node = YAML::LoadFile(fsm_yaml); }
    catch (const YAML::Exception& e)
    {
        ROS_FATAL("Cannot load fsm config '%s': %s", fsm_yaml.c_str(), e.what());
        return 1;
    }

    // ---- policy ----------------------------------------------------------
    // Resolves the policy package, builds the RL environment and loads the
    // model. A missing policy is NOT a startup failure: RLPolicyRunner reports
    // POLICY_NOT_CONFIGURED, CtrlFSM refuses every transition into an RLBase
    // state, and the non-RL flow is unaffected.
    auto policy = std::make_unique<RLPolicyRunner>();
    {
        RLPolicyRunner::Options po;
        po.fsm = fsm_config::node["FSM"];
        // Relative policy_dir resolves against the package share directory,
        // the same base the config paths above came from.
        po.base_dir = ros::package::getPath("mini_pi_fsm");
        po.robot_joint_names = mapper->jointNames();
        // A package's `calibration` is expressed against the live mapping.yaml.
        po.robot_joint_offsets = mapper->appliedJointOffset();
        po.backend = hardware->backendName();
        // The RL layer must never have to know WHICH simulator; only whether
        // this is one. See RLPolicyRunner's real-hardware lock.
        po.backend_is_simulation = hardware->isSimulation();

        // Launch-file overrides. `test_policy` can only ENABLE the test gate,
        // and even then RLPolicyRunner still requires a simulation backend.
        pnh.param("policy_dir", po.policy_dir_override, po.policy_dir_override);
        pnh.param("policy_version", po.version_override, po.version_override);
        pnh.param("test_policy", po.allow_test_policy_override,
                  po.allow_test_policy_override);

        if (!policy->initialize(policy_yaml, po)) return 1;
    }

    // ---- operator envelope from the policy -------------------------------
    // Full stick, the /cmd_vel clamp and one key press all follow the selected
    // policy package. Without a policy nothing can be commanded: the envelope
    // is zero, which is harmless because the RL state is unreachable anyway.
    {
        const OperatorEnvelope& e = policy->operatorEnvelope();
        in_cfg.vx_min = e.limits.vx[0];   in_cfg.vx_max = e.limits.vx[1];
        in_cfg.vy_min = e.limits.vy[0];   in_cfg.vy_max = e.limits.vy[1];
        in_cfg.dyaw_min = e.limits.wz[0]; in_cfg.dyaw_max = e.limits.wz[1];
        in_cfg.key_speed = e.key_linear_step;
        in_cfg.key_turn  = e.key_yaw_step;
        if (policy->ready())
            ROS_INFO("Operator envelope from policy '%s': vx[%.2f,%.2f] "
                     "vy[%.2f,%.2f] wz[%.2f,%.2f]  key step linear %.2f yaw %.2f",
                     policy->package().version.c_str(),
                     in_cfg.vx_min, in_cfg.vx_max, in_cfg.vy_min, in_cfg.vy_max,
                     in_cfg.dyaw_min, in_cfg.dyaw_max, in_cfg.key_speed, in_cfg.key_turn);
        else
            ROS_WARN("No policy: operator velocity envelope is zero.");
    }
    // Starts the raw-stdin reader when this process owns an interactive
    // foreground terminal, and quietly does not when it does not.
    input->initialize(nh, in_cfg);

    // ---- shared context --------------------------------------------------
    RobotState state;
    RobotCommand command;
    std::mutex state_mutex;

    ControlContext ctx;
    ctx.hardware    = hardware.get();
    ctx.mapper      = mapper.get();
    ctx.safety      = safety.get();
    ctx.input       = input.get();
    ctx.policy      = policy.get();
    ctx.state       = &state;
    ctx.command     = &command;
    ctx.state_mutex = &state_mutex;
    ctx.control_dt  = 1.0 / control_hz;

    if (!ctx.valid())
    {
        ROS_FATAL("ControlContext is incomplete");
        return 1;
    }

    // ---- FSM -------------------------------------------------------------
    std::unique_ptr<CtrlFSM> fsm;
    try
    {
        fsm = std::make_unique<CtrlFSM>(fsm_config::node["FSM"], &ctx, ctx.control_dt);
    }
    catch (const std::exception& e)
    {
        ROS_FATAL("FSM construction failed: %s", e.what());
        return 1;
    }

    if (policy->ready())
    {
        ROS_INFO("Backend '%s'. Control loop at %.0f Hz. Policy %s at %.0f Hz, started on "
                 "entry to the RL state.", hardware->backendName(), control_hz,
                 to_string(policy->status()),
                 policy->stepDt() > 0.0f ? 1.0 / policy->stepDt() : 0.0);
    }
    else
    {
        ROS_INFO("Backend '%s'. Control loop at %.0f Hz. RL unavailable: %s",
                 hardware->backendName(), control_hz, policy->unavailableReason().c_str());
    }
    ROS_INFO("%s", input->describe().c_str());

    // ---- startup readiness ----------------------------------------------
    // Everything above is construction; nothing has been transmitted. This is
    // the point Unitree reaches with LowState_t::wait_for_connection() in
    // init_fsm_state() (unitree_rl_mjlab/deploy/robots/g1/main.cpp:24): block
    // until the robot is actually talking, THEN build and start the FSM.
    //
    // Waiting here is not a fault, and it cannot become one: Safety is still
    // disarmed, so its verdicts are discarded.
    if (!waitUntilReady(ctx, readiness))
    {
        // Timed out, or Ctrl+C while waiting. The FSM never ran and nothing was
        // ever commanded, so there is nothing to wind down but the backend.
        hardware->protect();
        spinner.stop();
        return ros::ok() ? 1 : 0;
    }

    // From here on a lost motor packet IS a runtime fault: Safety latches it,
    // overrides the FSM's command with a damping hold and drives the FSM to
    // Passive, and only the operator `reset` input clears it.
    safety->arm();

    // Passive, always and explicitly -- the safe operating mode.
    fsm->start("Passive");

    // Diagnostics only -- throttled well below control frequency.
    ros::Rate debug_rate(debug_hz > 0.0 ? debug_hz : 1.0);
    while (ros::ok())
    {
        if (debug_hz > 0.0) ROS_INFO_STREAM(diagnostics(ctx, *fsm));
        debug_rate.sleep();
    }

    // Shutdown. ros::ok() went false (Ctrl+C, rosnode kill, master loss), which
    // also ends CtrlFSM::run_(), so the FSM thread is already winding down.
    // The last thing the robot is told before the bus goes quiet is the SDK's
    // own damping reflex.
    fsm->stop();           // joins the control thread; no state runs after this
    policy->stop();        // joins the policy thread
    hardware->protect();   // kp = 0, kd = 1 damping hold; suppressed when dry_run
    spinner.stop();
    return 0;
}
