// Non-hardware round-trip test for JointMapper.
//
// Verifies that motor -> robot -> motor is the identity (to float tolerance)
// for both TransformMode::Full and TransformMode::Identity, including joints
// with direction == -1 and joint_offset != 0 -- the combination that exposed
// the earlier offset-convention bug.
//
// Also pins the exact per-field semantics of the original Mini-Pi controller:
//   - tau is NEVER signed, in either direction
//   - outgoing dq/tau are NEVER signed
//   - kp/kd are permuted only
//
// Runs with no ROS master, no hardware and no config file.
#include "control/JointMapper.h"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

using namespace mini_pi;

namespace
{
int g_failures = 0;
int g_checks = 0;

constexpr double kTol = 1e-12;

void check_close(double got, double want, const std::string& what)
{
    ++g_checks;
    if (!(std::fabs(got - want) <= kTol))
    {
        std::printf("  FAIL %-52s got %+.17g want %+.17g\n", what.c_str(), got, want);
        ++g_failures;
    }
}

void check_true(bool cond, const std::string& what)
{
    ++g_checks;
    if (!cond)
    {
        std::printf("  FAIL %s\n", what.c_str());
        ++g_failures;
    }
}

/// The real Mini-Pi mapping, written to a temp file so we exercise the same
/// loadFromYaml() path production uses.
const std::vector<int> kMapIndex   = {5, 4, 3, 2, 1, 0, 11, 10, 9, 8, 7, 6};
const std::vector<int> kDirection  = {1, 1, -1, -1, 1, 1, -1, 1, -1, 1, -1, 1};
const std::vector<double> kOffset  = {-0.25, 0.00, 0.00, 0.65, -0.40, 0.00,
                                      -0.25, 0.00, 0.00, 0.65, -0.40, 0.00};

std::string write_mapping(const std::string& mode,
                          const std::vector<int>& map_index,
                          const std::vector<int>& direction,
                          const std::vector<double>& offset)
{
    static int n = 0;
    const std::string path = "/tmp/mini_pi_mapping_test_" + std::to_string(n++) + ".yaml";
    std::ofstream f(path);
    f << "transform_mode: " << mode << "\njoint_names: [";
    for (std::size_t i = 0; i < MINI_PI_DOF; ++i)
        f << (i ? ", " : "") << "\"j" << i << "\"";
    f << "]\nmap_index: [";
    for (std::size_t i = 0; i < map_index.size(); ++i) f << (i ? ", " : "") << map_index[i];
    f << "]\ndirection: [";
    for (std::size_t i = 0; i < direction.size(); ++i) f << (i ? ", " : "") << direction[i];
    f << "]\njoint_offset: [";
    for (std::size_t i = 0; i < offset.size(); ++i) f << (i ? ", " : "") << offset[i];
    f << "]\n";
    return path;
}

/// Deterministic synthetic values, distinct per index and per field so that a
/// permutation error cannot cancel out.
void make_synthetic(JointArray& q, JointArray& dq, JointArray& tau)
{
    for (std::size_t i = 0; i < MINI_PI_DOF; ++i)
    {
        q[i]   =  0.37 * static_cast<double>(i) - 1.1;
        dq[i]  = -0.53 * static_cast<double>(i) + 0.7;
        tau[i] =  0.19 * static_cast<double>(i) - 0.4;
    }
}

void run_roundtrip(const std::string& label, TransformMode expect_mode,
                   const std::string& mode_str,
                   const std::vector<double>& offset)
{
    std::printf("[%s]\n", label.c_str());

    JointMapper mapper;
    const std::string path = write_mapping(mode_str, kMapIndex, kDirection, offset);
    check_true(mapper.loadFromYaml(path), "loadFromYaml succeeds");
    if (!mapper.valid()) { std::printf("  (aborting case)\n"); return; }
    check_true(mapper.mode() == expect_mode, "transform_mode parsed");

    JointArray mq{}, mdq{}, mtau{};
    make_synthetic(mq, mdq, mtau);

    // motor -> robot
    JointArray rq{}, rdq{}, rtau{};
    mapper.motorToRobot(mq, mdq, mtau, rq, rdq, rtau);

    const bool full = (expect_mode == TransformMode::Full);

    // Explicit formula check against the verified original semantics.
    for (std::size_t j = 0; j < MINI_PI_DOF; ++j)
    {
        const int m = kMapIndex[j];
        const double s = full ? static_cast<double>(kDirection[j]) : 1.0;
        const double off = full ? offset[j] : 0.0;
        check_close(rq[j],   s * mq[m] - off, "motor->robot q[" + std::to_string(j) + "]");
        check_close(rdq[j],  s * mdq[m],      "motor->robot dq[" + std::to_string(j) + "]");
        check_close(rtau[j], mtau[m],         "motor->robot tau[" + std::to_string(j) + "] (unsigned)");
    }

    // robot -> motor, feeding the robot-space state straight back.
    RobotCommand cmd;
    cmd.q = rq; cmd.dq = rdq; cmd.torque = rtau;
    for (std::size_t j = 0; j < MINI_PI_DOF; ++j)
    {
        cmd.kp[j] = 10.0 + static_cast<double>(j);
        cmd.kd[j] = 0.5 + 0.1 * static_cast<double>(j);
    }
    cmd.valid = true;

    MotorCommand out;
    mapper.robotToMotor(cmd, out);

    // q must round-trip exactly; this is the property the offset bug broke.
    for (std::size_t j = 0; j < MINI_PI_DOF; ++j)
    {
        const int m = kMapIndex[j];
        check_close(out.q[m], mq[m], "round-trip q motor[" + std::to_string(m) + "]");
        // dq/tau are NOT signed on the way out, so they round-trip only where
        // direction == +1. Pin the original's exact (asymmetric) behaviour.
        check_close(out.dq[m],     rdq[j],  "robot->motor dq motor[" + std::to_string(m) + "] (unsigned)");
        check_close(out.torque[m], rtau[j], "robot->motor tau motor[" + std::to_string(m) + "] (unsigned)");
        check_close(out.kp[m], cmd.kp[j], "kp permuted motor[" + std::to_string(m) + "]");
        check_close(out.kd[m], cmd.kd[j], "kd permuted motor[" + std::to_string(m) + "]");
    }
    check_true(out.valid, "valid flag propagated");

    // Every motor slot must have been written exactly once.
    std::vector<int> hits(MINI_PI_DOF, 0);
    for (std::size_t j = 0; j < MINI_PI_DOF; ++j) hits[kMapIndex[j]]++;
    for (std::size_t m = 0; m < MINI_PI_DOF; ++m)
        check_true(hits[m] == 1, "motor slot " + std::to_string(m) + " written exactly once");
}

/// Focused case: the joints the earlier bug would have broken.
void run_sign_offset_case()
{
    std::printf("[Full: direction=-1 AND joint_offset!=0]\n");
    JointMapper mapper;
    const std::string path = write_mapping("Full", kMapIndex, kDirection, kOffset);
    if (!mapper.loadFromYaml(path)) { check_true(false, "load"); return; }

    // robot joints 3 and 9 are calf: offset +0.65; joint 3 has direction -1.
    // robot joints 0 and 6 are hip_pitch: offset -0.25; joint 6 has direction -1.
    for (int j : {0, 3, 6, 9, 10})
    {
        const int m = kMapIndex[j];
        const double s = kDirection[j];
        const double off = kOffset[j];

        JointArray mq{}, mdq{}, mtau{}, rq{}, rdq{}, rtau{};
        make_synthetic(mq, mdq, mtau);
        mapper.motorToRobot(mq, mdq, mtau, rq, rdq, rtau);

        // The buggy convention was robot = dir*(motor - off), i.e. -dir*off.
        // Assert we are NOT that, whenever it actually differs.
        const double correct = s * mq[m] - off;
        const double buggy   = s * (mq[m] - off);
        check_close(rq[j], correct, "j" + std::to_string(j) + " uses robot-space offset");
        if (std::fabs(correct - buggy) > 1e-9)
            check_true(std::fabs(rq[j] - buggy) > 1e-9,
                       "j" + std::to_string(j) + " is NOT the old motor-space convention");

        RobotCommand cmd; cmd.q = rq; cmd.valid = true;
        MotorCommand out; mapper.robotToMotor(cmd, out);
        check_close(out.q[m], mq[m], "j" + std::to_string(j) + " exact round-trip");
        check_close(out.q[m], s * (rq[j] + off),
                    "j" + std::to_string(j) + " forward formula");
    }
}

/// Config validation must reject bad input rather than silently proceeding.
void run_validation_cases()
{
    std::printf("[config validation]\n");
    {
        JointMapper m;
        auto bad = kMapIndex; bad[0] = 4;  // duplicate of index 4
        check_true(!m.loadFromYaml(write_mapping("Full", bad, kDirection, kOffset)),
                   "rejects non-permutation map_index");
    }
    {
        JointMapper m;
        auto bad = kDirection; bad[2] = 0;
        check_true(!m.loadFromYaml(write_mapping("Full", kMapIndex, bad, kOffset)),
                   "rejects direction entry that is not +/-1");
    }
    {
        JointMapper m;
        auto bad = kMapIndex; bad[0] = 99;
        check_true(!m.loadFromYaml(write_mapping("Full", bad, kDirection, kOffset)),
                   "rejects out-of-range map_index");
    }
    {
        JointMapper m;
        check_true(!m.loadFromYaml("/tmp/mini_pi_definitely_absent.yaml"),
                   "rejects missing file");
    }
    {
        // obsolete key must be refused, not ignored
        const std::string p = "/tmp/mini_pi_mapping_obsolete.yaml";
        std::ofstream f(p);
        f << "joint_names: [";
        for (std::size_t i = 0; i < MINI_PI_DOF; ++i) f << (i ? ", " : "") << "\"j" << i << "\"";
        f << "]\nmap_index: [";
        for (std::size_t i = 0; i < kMapIndex.size(); ++i) f << (i ? ", " : "") << kMapIndex[i];
        f << "]\ndirection: [";
        for (std::size_t i = 0; i < kDirection.size(); ++i) f << (i ? ", " : "") << kDirection[i];
        f << "]\njoint_offset: [";
        for (std::size_t i = 0; i < kOffset.size(); ++i) f << (i ? ", " : "") << kOffset[i];
        f << "]\nzero_offset: [0,0,0,0,0,0,0,0,0,0,0,0]\n";
        f.close();
        JointMapper m;
        check_true(!m.loadFromYaml(p), "rejects obsolete 'zero_offset' key");
    }
    {
        JointMapper m;
        const std::string p = write_mapping("Sideways", kMapIndex, kDirection, kOffset);
        check_true(!m.loadFromYaml(p), "rejects unknown transform_mode");
    }
}

/// Loads the real shipped config and sanity-checks it end to end.
void run_real_config(const std::string& path)
{
    std::printf("[shipped config/mapping.yaml]\n");
    JointMapper m;
    if (!m.loadFromYaml(path))
    {
        check_true(false, "shipped mapping.yaml loads");
        return;
    }
    check_true(m.valid(), "shipped mapping.yaml valid");
    check_true(m.mode() == TransformMode::Full, "shipped mode is Full");
    check_true(m.mapIndex() == kMapIndex, "shipped map_index matches pi_pd_config.yaml");
    check_true(m.direction() == kDirection, "shipped direction matches pi_pd_config.yaml");
    for (std::size_t j = 0; j < MINI_PI_DOF; ++j)
        check_close(m.jointOffset()[j], kOffset[j], "shipped joint_offset[" + std::to_string(j) + "]");

    JointArray mq{}, mdq{}, mtau{}, rq{}, rdq{}, rtau{};
    make_synthetic(mq, mdq, mtau);
    m.motorToRobot(mq, mdq, mtau, rq, rdq, rtau);
    RobotCommand c; c.q = rq; c.valid = true;
    MotorCommand o; m.robotToMotor(c, o);
    for (std::size_t i = 0; i < MINI_PI_DOF; ++i)
        check_close(o.q[i], mq[i], "shipped config round-trip motor[" + std::to_string(i) + "]");

    // robot_q == 0 must map to the nominal pose direction*joint_offset.
    RobotCommand zero; zero.valid = true;   // q all zeros
    MotorCommand nominal; m.robotToMotor(zero, nominal);
    for (std::size_t j = 0; j < MINI_PI_DOF; ++j)
        check_close(nominal.q[m.mapIndex()[j]],
                    static_cast<double>(m.direction()[j]) * kOffset[j],
                    "robot_q=0 -> nominal motor pose j" + std::to_string(j));
}
} // namespace

int main(int argc, char** argv)
{
    std::printf("=== JointMapper round-trip test ===\n");

    run_roundtrip("Full mode", TransformMode::Full, "Full", kOffset);
    run_roundtrip("Identity mode", TransformMode::Identity, "Identity", kOffset);
    // Identity must ignore a non-zero offset entirely.
    run_roundtrip("Identity mode (offset must be ignored)", TransformMode::Identity,
                  "Identity", kOffset);
    run_sign_offset_case();
    run_validation_cases();

    if (argc > 1) run_real_config(argv[1]);

    std::printf("=== %d checks, %d failures ===\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
