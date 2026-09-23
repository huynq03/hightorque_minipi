// Startup readiness and runtime safety, with neither an `Init` nor a `Fault`
// FSM state.
//
// Everything here runs with NO ROS master, NO hardware and NO policy: the
// backend is a fake HardwareInterface whose feedback the test drives directly,
// and the FSM is stepped by hand through CtrlFSM::step() instead of by its
// control thread.
//
//   rosrun mini_pi_fsm test_startup_and_safety <path/to/config>
#include "FSM/CtrlFSM.h"
#include "FSM/FSMState.h"
#include "FSM/State_FixStand.h"
#include "FSM/State_Passive.h"
#include "FSM/State_RLBase.h"
#include "control/ControlContext.h"
#include "control/SafetyGate.h"
#include "control/SystemReadiness.h"
#include "hardware/HardwareInterface.h"

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

using namespace mini_pi;

namespace
{
int g_checks = 0;
int g_failures = 0;

void check(bool ok, const std::string& what)
{
    ++g_checks;
    if (!ok) { ++g_failures; std::printf("  FAIL  %s\n", what.c_str()); }
    else      std::printf("  ok    %s\n", what.c_str());
}

/// A HardwareInterface whose feedback and health the test sets directly, and
/// which records every MotorCommand it is handed. It deliberately implements
/// nothing beyond the interface, so the FSM cannot tell it from the real or the
/// simulated backend.
class FakeHardware : public HardwareInterface
{
public:
    FakeHardware()
    {
        for (std::size_t i = 0; i < MINI_PI_DOF; ++i)
        {
            names_.push_back("fake_motor_" + std::to_string(i));
            ids_.push_back(static_cast<int>(i));
        }
    }

    // --- test controls -----------------------------------------------------

    /// All 12 motors fresh and finite, IMU upright and fresh.
    void makeHealthy()
    {
        feedback_valid_ = true;
        imu_valid_ = true;
        status_ = HardwareStatus::Ok;
    }
    /// Feedback stops arriving: the exact runtime condition that used to send
    /// the FSM into State_Fault.
    void loseFeedback()
    {
        feedback_valid_ = false;
        status_ = HardwareStatus::Stale;
    }
    /// Exact IMU age, for the readiness-vs-runtime boundary test. The real
    /// backends report `Stale` (not `Fault`) for a stale IMU, and readiness
    /// only rejects `Fault`, so `status_` is deliberately left at Ok here --
    /// that is precisely the path the readiness check has to catch on its own.
    void setImuAge(double age) { imu_age_ = age; }

    // --- HardwareInterface -------------------------------------------------

    bool read(RobotState& s) override
    {
        for (std::size_t i = 0; i < MINI_PI_DOF; ++i)
        {
            s.motor_q[i] = 0.1 * static_cast<double>(i);
            s.motor_dq[i] = 0.0;
            s.motor_tau[i] = 0.0;
            s.motor_fault[i] = 0;
            s.motor_age[i] = feedback_valid_ ? 0.0 : 5.0;
            s.motor_fresh[i] = feedback_valid_;
        }
        s.stale_motor_count = feedback_valid_ ? 0 : MINI_PI_DOF;
        s.motor_valid = feedback_valid_;
        s.motor_stamp = Clock::now();

        s.imu_valid = imu_valid_;
        s.imu_age = imu_valid_ ? imu_age_ : 5.0;
        s.imu_stamp = Clock::now();
        s.orientation = {0.0, 0.0, 0.0, 1.0};   // (x, y, z, w) == upright
        s.angular_velocity = {0.0, 0.0, 0.0};
        quat_xyzw_to_rpy(s.orientation, s.rpy);
        return s.motor_valid;
    }

    bool write(const MotorCommand& cmd) override
    {
        last_ = cmd;
        ++writes_;
        return true;
    }

    HardwareStatus status() const override { return status_; }
    void protect() override { ++protects_; }
    void stop() override { ++stops_; }
    bool initialized() const override { return true; }
    bool dryRun() const override { return true; }
    std::size_t motorCount() const override { return MINI_PI_DOF; }
    const std::vector<std::string>& motorNames() const override { return names_; }
    const std::vector<int>& motorIds() const override { return ids_; }
    const char* backendName() const override { return "fake"; }
    bool isSimulation() const override { return true; }

    const MotorCommand& lastWrite() const { return last_; }
    std::size_t writes() const { return writes_; }
    std::size_t stops() const { return stops_; }

private:
    bool feedback_valid_ = false;   ///< startup default: nothing has arrived yet
    bool imu_valid_ = false;
    double imu_age_ = 0.0;
    HardwareStatus status_ = HardwareStatus::Ok;

    MotorCommand last_;
    std::size_t writes_ = 0;
    std::size_t protects_ = 0;
    std::size_t stops_ = 0;

    std::vector<std::string> names_;
    std::vector<int> ids_;
};

/// Everything main() owns, minus ROS. Assembled once per test case so each one
/// starts from a clean FSM.
struct Rig
{
    FakeHardware hardware;
    JointMapper mapper;
    SafetyManager safety;
    SafetyGate gate;
    InputManager input;
    PolicyRunner policy;

    RobotState state;
    RobotCommand command;
    std::mutex state_mutex;
    ControlContext ctx;

    bool load(const std::string& cfg_dir)
    {
        if (!mapper.loadFromYaml(cfg_dir + "/mapping.yaml")) return false;
        if (!safety.loadFromYaml(cfg_dir + "/safety.yaml")) return false;
        if (!gate.loadFromYaml(cfg_dir + "/safety.yaml")) return false;

        ctx.hardware = &hardware;
        ctx.mapper = &mapper;
        ctx.safety = &safety;
        ctx.input = &input;
        ctx.policy = &policy;
        ctx.gate = &gate;
        ctx.state = &state;
        ctx.command = &command;
        ctx.state_mutex = &state_mutex;
        ctx.control_dt = 0.001;
        return ctx.valid();
    }
};

// ===========================================================================
// Startup
// ===========================================================================
void test_startup_gate(const std::string& cfg_dir)
{
    std::printf("[startup readiness]\n");
    Rig rig;
    if (!rig.load(cfg_dir)) { check(false, "rig loads the shipped config"); return; }

    const ReadinessConfig rc;

    // --- no feedback -> startup waits, the FSM has not started -------------
    refreshReadinessState(rig.ctx);
    const std::string why = readinessReason(rig.ctx, rc);
    check(!why.empty(), "with no motor feedback the system is NOT ready");
    check(why.find("motors have produced a valid frame") != std::string::npos,
          "the reason names the missing motor feedback: " + why);
    check(!rig.gate.armed(), "SafetyGate stays disarmed while not ready");
    check(!rig.gate.faulted(),
          "waiting for the first packet is NOT recorded as a runtime fault");
    check(rig.hardware.writes() == 0, "nothing is transmitted before the FSM starts");

    // A SafetyManager verdict at this moment is MOTOR_TIMEOUT -- precisely the
    // condition that used to yank State_Init into State_Fault. The gate must
    // swallow it because it is not armed.
    const SafetyReport boot =
        rig.safety.checkState(rig.state, rig.hardware.status());
    check(!boot.ok(), "SafetyManager does report MOTOR_TIMEOUT at startup");
    rig.gate.raise(boot);
    check(!rig.gate.faulted(), "a disarmed gate refuses to latch a startup verdict");

    // --- IMU still missing -------------------------------------------------
    // (feedback alone is not readiness; robot.yaml `startup.require_imu` is true)

    // --- feedback becomes valid -> ready -----------------------------------
    rig.hardware.makeHealthy();
    refreshReadinessState(rig.ctx);
    check(readinessReason(rig.ctx, rc).empty(),
          "with valid motor + IMU feedback the system IS ready");

    // --- require_imu: false must not require the IMU -----------------------
    ReadinessConfig no_imu = rc;
    no_imu.require_imu = false;
    check(readinessReason(rig.ctx, no_imu).empty(),
          "require_imu: false still passes on a healthy system");
}

// ===========================================================================
// The FSM starts in Passive
// ===========================================================================
void test_starts_in_passive(const std::string& cfg_dir)
{
    std::printf("[FSM starts in Passive]\n");
    Rig rig;
    if (!rig.load(cfg_dir)) { check(false, "rig loads the shipped config"); return; }
    rig.hardware.makeHealthy();

    CtrlFSM fsm(fsm_config::node["FSM"], &rig.ctx, rig.ctx.control_dt);
    check(fsm.currentStateName() == "<none>", "no state is current before start()");

    rig.gate.arm();
    fsm.start("Passive");
    check(fsm.currentStateName() == "Passive", "CtrlFSM starts in Passive");
    fsm.stop();

    // Passive's low-level command: kp = 0 everywhere, kd from fsm.yaml, and the
    // target is the measured pose. Same shape as Unitree's State_Passive.
    fsm.step();
    const MotorCommand& m = rig.hardware.lastWrite();
    bool kp_zero = true, kd_set = true;
    for (std::size_t i = 0; i < MINI_PI_DOF; ++i)
    {
        if (m.kp[i] != 0.0) kp_zero = false;
        if (!(m.kd[i] > 0.0)) kd_set = false;
    }
    check(rig.hardware.writes() > 0, "Passive transmits a command");
    check(kp_zero, "Passive commands kp = 0 on every joint");
    check(kd_set, "Passive commands a non-zero damping on every joint");

    // An unknown initial state must be refused, not silently substituted.
    bool threw = false;
    try { CtrlFSM(fsm_config::node["FSM"], &rig.ctx, 0.001).start("Fault"); }
    catch (const std::exception&) { threw = true; }
    check(threw, "starting in a state that does not exist is an error");
}

// ===========================================================================
// Runtime feedback loss
// ===========================================================================
void test_runtime_feedback_loss(const std::string& cfg_dir)
{
    std::printf("[runtime feedback loss]\n");
    Rig rig;
    if (!rig.load(cfg_dir)) { check(false, "rig loads the shipped config"); return; }
    rig.hardware.makeHealthy();

    CtrlFSM fsm(fsm_config::node["FSM"], &rig.ctx, rig.ctx.control_dt);
    rig.gate.arm();
    // Start in FixStand so that the "driven back to Passive" half of the
    // reaction is observable. It is a normal operating mode like any other.
    fsm.start("FixStand");
    fsm.stop();
    fsm.step();
    check(fsm.currentStateName() == "FixStand", "healthy system stays in FixStand");
    check(!rig.gate.faulted(), "healthy system is not faulted");

    const MotorCommand healthy = rig.hardware.lastWrite();
    bool stand_drives = false;
    for (std::size_t i = 0; i < MINI_PI_DOF; ++i) if (healthy.kp[i] > 0.0) stand_drives = true;
    check(stand_drives, "FixStand commands a non-zero stiffness while healthy");

    // --- feedback goes stale ----------------------------------------------
    rig.hardware.loseFeedback();
    fsm.step();

    check(rig.gate.faulted(), "stale feedback latches a runtime safety fault");
    check(rig.gate.fault().fault == SafetyFault::MotorTimeout,
          "the latched reason is MOTOR_TIMEOUT");
    check(fsm.currentStateName() == "Passive",
          "the FSM is driven to the safe operating mode (Passive), not to a Fault state");

    const MotorCommand overridden = rig.hardware.lastWrite();
    bool all_kp_zero = true, all_kd_is_fault = true;
    for (std::size_t i = 0; i < MINI_PI_DOF; ++i)
    {
        if (overridden.kp[i] != 0.0) all_kp_zero = false;
        if (overridden.kd[i] != rig.gate.config().kd[i]) all_kd_is_fault = false;
    }
    check(all_kp_zero, "the safe override commands kp = 0");
    check(all_kd_is_fault, "the safe override commands the configured fault damping");
    check(rig.hardware.stops() == 0,
          "hard_stop is off by default, so no firmware stop is issued");

    // --- the condition clears, the latch does not --------------------------
    rig.hardware.makeHealthy();
    for (int i = 0; i < 20; ++i) fsm.step();
    check(rig.gate.faulted(),
          "the latch SURVIVES the condition clearing -- no silent resume");
    const MotorCommand still = rig.hardware.lastWrite();
    bool still_override = true;
    for (std::size_t i = 0; i < MINI_PI_DOF; ++i)
        if (still.kp[i] != 0.0 || still.kd[i] != rig.gate.config().kd[i]) still_override = false;
    check(still_override, "the command is still overridden after the condition clears");

    // --- explicit recovery -------------------------------------------------
    check(rig.gate.clear(), "an explicit reset clears the latch");
    check(!rig.gate.faulted(), "the gate is healthy again after the reset");
    fsm.step();
    check(fsm.currentStateName() == "Passive",
          "what resumes after a reset is Passive, never the previous state");
    check(rig.hardware.lastWrite().kd[0] > 0.0, "commanding resumes after the reset");

    // A second clear() is a no-op: recovery is not something that can be
    // accidentally double-counted.
    check(!rig.gate.clear(), "clearing a healthy gate does nothing");
}

// ===========================================================================
// A rejected command latches too
// ===========================================================================
void test_bad_command_is_blocked(const std::string& cfg_dir)
{
    std::printf("[invalid command]\n");
    Rig rig;
    if (!rig.load(cfg_dir)) { check(false, "rig loads the shipped config"); return; }
    rig.hardware.makeHealthy();
    refreshReadinessState(rig.ctx);
    rig.gate.arm();

    RobotCommand bad;
    for (std::size_t i = 0; i < MINI_PI_DOF; ++i)
    {
        bad.q[i] = std::nan("");
        bad.kp[i] = 10.0;
        bad.kd[i] = 1.0;
    }
    bad.valid = true;
    bad.stamp = Clock::now();

    check(!rig.gate.publish(bad, rig.ctx), "a non-finite command is not transmitted");
    check(rig.gate.faulted(), "a non-finite command latches a fault");
    check(rig.gate.fault().fault == SafetyFault::NanCommand, "the reason is NAN_COMMAND");
    for (std::size_t i = 0; i < MINI_PI_DOF; ++i)
    {
        if (!std::isfinite(rig.hardware.lastWrite().q[i]))
        {
            check(false, "the override never forwards a non-finite target");
            return;
        }
    }
    check(true, "the override transmits a finite damping hold instead");
}

// ===========================================================================
// FixStand -> RL is gated on the stand having finished
// ===========================================================================
void test_fixstand_gates_rl(const std::string& cfg_dir)
{
    std::printf("[FixStand -> RL guard]\n");
    Rig rig;
    if (!rig.load(cfg_dir)) { check(false, "rig loads the shipped config"); return; }
    rig.hardware.makeHealthy();
    refreshReadinessState(rig.ctx);

    State_FixStand stand(99, "FixStand", &rig.ctx);
    check(!stand.readyForRL(), "before enter(), FixStand refuses RL");
    check(!stand.notReadyForRLReason().empty(), "and says why");

    stand.enter();
    check(!stand.finished(), "the interpolation has not finished immediately after enter()");
    check(!stand.readyForRL(), "mid-interpolation, FixStand still refuses RL");

    // The shipped duration is 3 s; rather than sleeping for it, assert the two
    // predicates agree -- readyForRL() IS finished(), by construction.
    check(stand.readyForRL() == stand.finished(),
          "readyForRL() is exactly finished()");

    // With no policy loaded, CtrlFSM refuses RL entry regardless. Asserting the
    // reason string is non-empty keeps the refusal observable to an operator.
    check(!rig.policy.ready(), "an uninitialized PolicyRunner is not ready");
    check(!rig.policy.unavailableReason().empty(), "and explains why RL is refused");
}

// ===========================================================================
// Readiness is never LOOSER than the runtime freshness checks
// ===========================================================================
void test_readiness_not_looser_than_runtime(const std::string& cfg_dir)
{
    std::printf("[readiness vs runtime freshness]\n");
    Rig rig;
    if (!rig.load(cfg_dir)) { check(false, "rig loads the shipped config"); return; }
    rig.hardware.makeHealthy();

    const ReadinessConfig rc;   // robot.yaml startup.require_fresh_s == 0.5
    const double imu_timeout = rig.safety.config().imu_timeout_s;
    check(rc.require_fresh_s == 0.5, "startup.require_fresh_s is 0.5 s");
    check(imu_timeout == 0.2, "safety.yaml imu_timeout_s is 0.2 s");
    check(rc.require_fresh_s > imu_timeout,
          "the startup threshold IS the looser of the two -- the case that matters");

    // ready(age) / runtime_imu_timeout(age) for one snapshot.
    const auto probe = [&](double age, bool& ready, bool& runtime_imu_timeout) {
        rig.hardware.setImuAge(age);
        refreshReadinessState(rig.ctx);
        ready = readinessReason(rig.ctx, rc).empty();
        const SafetyReport rep =
            rig.safety.checkState(rig.state, rig.hardware.status());
        runtime_imu_timeout = (rep.fault == SafetyFault::ImuTimeout);
    };

    struct Case { double age; bool want_ready; const char* what; };
    // Both sides are "stale iff age > threshold", so age == the bound is FRESH
    // on both. That shared boundary semantic is what the 0.20 case pins down.
    const Case cases[] = {
        {0.19, true,  "imu_age 0.19 s (< 0.2) -> READY"},
        {0.20, true,  "imu_age 0.20 s (== bound) -> READY, matching runtime's `>`"},
        {0.21, false, "imu_age 0.21 s (> 0.2) -> NOT READY"},
        {0.40, false, "imu_age 0.40 s -> NOT READY (was READY before the fix)"},
    };
    for (const Case& c : cases)
    {
        bool ready = false, timeout = false;
        probe(c.age, ready, timeout);
        check(ready == c.want_ready, c.what);
        // THE INVARIANT: readiness passing must imply the runtime check on the
        // very same snapshot does not immediately fault.
        check(!(ready && timeout),
              std::string("  and READY implies no immediate IMU_TIMEOUT at ") +
              std::to_string(c.age) + " s");
    }

    // Regression sweep across and well past both thresholds.
    bool invariant_held = true;
    for (double age = 0.0; age <= 0.60; age += 0.01)
    {
        bool ready = false, timeout = false;
        probe(age, ready, timeout);
        if (ready && timeout) invariant_held = false;
    }
    check(invariant_held,
          "READY never coexists with an immediate IMU_TIMEOUT, 0.00 - 0.60 s");

    // The motor path must keep the property it already had, unchanged: both
    // sides read `stale_motor_count`, so they cannot disagree.
    rig.hardware.setImuAge(0.0);
    rig.hardware.loseFeedback();
    refreshReadinessState(rig.ctx);
    const bool motor_ready = readinessReason(rig.ctx, rc).empty();
    const SafetyReport mrep =
        rig.safety.checkState(rig.state, rig.hardware.status());
    check(!motor_ready, "stale motors are still NOT ready");
    check(mrep.fault == SafetyFault::MotorTimeout,
          "and runtime still reports MOTOR_TIMEOUT for the same snapshot");
}

// ===========================================================================
// No Init / Fault FSM state remains reachable
// ===========================================================================
void test_no_init_or_fault_state(const std::string& cfg_dir)
{
    (void)cfg_dir;
    std::printf("[no Init / Fault FSM states]\n");

    const YAML::Node enabled = fsm_config::node["FSM"]["_"];
    check(static_cast<bool>(enabled), "FSM._ present");
    for (auto it = enabled.begin(); it != enabled.end(); ++it)
    {
        const std::string name = it->first.as<std::string>();
        check(name != "Init" && name != "Fault",
              "enabled state '" + name + "' is a robot operating mode");
    }
    check(getFsmMap().count("State_Init") == 0,
          "State_Init is not in the FSM factory registry");
    check(getFsmMap().count("State_Fault") == 0,
          "State_Fault is not in the FSM factory registry");
    check(getFsmMap().count("State_Passive") == 1 &&
          getFsmMap().count("State_FixStand") == 1 &&
          getFsmMap().count("State_RLBase") == 1,
          "the three operating modes are registered");
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

    // The throttled log macros in CtrlFSM and SafetyGate read ros::Time::now().
    // This initializes the wall-clock source; it needs no master and no node.
    ros::Time::init();

    try { fsm_config::node = YAML::LoadFile(dir + "/fsm.yaml"); }
    catch (const YAML::Exception& e)
    {
        std::printf("  FATAL: cannot load fsm.yaml: %s\n", e.what());
        return 1;
    }

    std::printf("=== startup readiness / runtime safety tests ===\n");
    test_startup_gate(dir);
    test_starts_in_passive(dir);
    test_runtime_feedback_loss(dir);
    test_bad_command_is_blocked(dir);
    test_fixstand_gates_rl(dir);
    test_readiness_not_looser_than_runtime(dir);
    test_no_init_or_fault_state(dir);

    std::printf("=== %d checks, %d failures ===\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
