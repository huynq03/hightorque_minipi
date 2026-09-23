// Keyboard input mapping: keys -> FSM transition requests and -> VelocityCommand.
//
// Drives InputManager::feedKey() directly, which is exactly what update() calls
// with the reader's current key, so this covers the mapping and the source
// precedence with no terminal, no ROS master and no hardware. The terminal
// handling itself (termios, the foreground-TTY check) is exercised by the
// scripted pty run in scripts/test-keyboard.sh.
//
//   rosrun mini_pi_fsm test_input_keyboard <path/to/config>
#include "input/InputManager.h"

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <cstdio>
#include <set>
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

bool near(double a, double b) { return std::fabs(a - b) < 1e-9; }

/// A key press: the reader reports the key, then "" once it is released.
/// feedKey() is edge-triggered, so the release matters.
void press(InputManager& in, const std::string& key)
{
    in.feedKey(key);
    in.feedKey("");
}

InputManager::Config shippedConfig(const std::string& robot_yaml)
{
    InputManager::Config c;
    YAML::Node r;
    try { r = YAML::LoadFile(robot_yaml); }
    catch (const YAML::Exception&) { return c; }

    if (const YAML::Node j = r["joy"])
    {
        if (const YAML::Node g = j["ranges"])
        {
            auto rng = [&](const char* k, double& lo, double& hi) {
                if (!g[k]) return;
                const auto v = g[k].as<std::vector<double>>();
                if (v.size() == 2) { lo = v[0]; hi = v[1]; }
            };
            rng("vx", c.vx_min, c.vx_max);
            rng("vy", c.vy_min, c.vy_max);
            rng("dyaw", c.dyaw_min, c.dyaw_max);
        }
    }
    if (const YAML::Node k = r["keyboard"])
    {
        if (k["speed"]) c.key_speed = k["speed"].as<double>();
        if (k["turn"])  c.key_turn  = k["turn"].as<double>();
        if (const YAML::Node m = k["keys"])
        {
            auto ks = [&](const char* n, std::string& dst) {
                if (m[n]) dst = m[n].as<std::string>();
            };
            ks("forward", c.key_forward);     ks("backward", c.key_backward);
            ks("left", c.key_left);           ks("right", c.key_right);
            ks("yaw_left", c.key_yaw_left);   ks("yaw_right", c.key_yaw_right);
            ks("stop", c.key_stop);           ks("passive", c.key_passive);
            ks("stand", c.key_stand);         ks("rl", c.key_rl);
            ks("reset", c.key_reset);
        }
    }
    return c;
}

// ===========================================================================
void test_shipped_keys(const std::string& robot_yaml)
{
    std::printf("[robot.yaml keyboard block]\n");
    YAML::Node r;
    try { r = YAML::LoadFile(robot_yaml); }
    catch (const YAML::Exception&) { check(false, "robot.yaml loads"); return; }

    const YAML::Node k = r["keyboard"];
    check(static_cast<bool>(k), "keyboard block present");
    if (!k) return;
    check(k["enable"].as<bool>(), "keyboard is enabled by default");

    // teleop_twist_keyboard's own letters, preserved so a key means the same
    // thing on the built-in reader and on the /cmd_vel node.
    const YAML::Node m = k["keys"];
    check(m["forward"].as<std::string>() == "i",   "forward key is teleop's 'i'");
    check(m["backward"].as<std::string>() == ",",  "backward key is teleop's ','");
    check(m["left"].as<std::string>() == "J",      "strafe-left key is teleop's 'J'");
    check(m["right"].as<std::string>() == "L",     "strafe-right key is teleop's 'L'");
    check(m["yaw_left"].as<std::string>() == "j",  "yaw-left key is teleop's 'j'");
    check(m["yaw_right"].as<std::string>() == "l", "yaw-right key is teleop's 'l'");
    check(m["stop"].as<std::string>() == "k",      "stop key is teleop's 'k'");
    check(k["speed"].as<double>() == 0.25, "speed matches the `keyboard` task's _speed");
    check(k["turn"].as<double>() == 0.30,  "turn matches the `keyboard` task's _turn");

    // Every key must be distinct, or one press would mean two things.
    std::set<std::string> seen;
    bool unique = true;
    for (auto it = m.begin(); it != m.end(); ++it)
        if (!seen.insert(it->second.as<std::string>()).second) unique = false;
    check(unique, "every configured key is distinct");
}

void test_fsm_triggers(const std::string& robot_yaml)
{
    std::printf("[keyboard -> FSM transition requests]\n");
    InputManager in;
    const InputManager::Config cfg = shippedConfig(robot_yaml);
    in.configure(cfg);

    press(in, cfg.key_stand);
    check(in.requestStand(), "'2' requests FixStand");
    check(!in.requestStand(), "  and the request is consumed exactly once");

    press(in, cfg.key_rl);
    check(in.requestRL(), "'3' requests RL");
    press(in, cfg.key_passive);
    check(in.requestPassive(), "'1' requests Passive");
    press(in, cfg.key_reset);
    check(in.requestReset(), "'0' requests a fault reset");

    // Held / auto-repeating key: one request, not one per cycle. This is the
    // same edge contract the gamepad combos have.
    in.feedKey(cfg.key_stand);
    for (int i = 0; i < 50; ++i) in.feedKey(cfg.key_stand);
    int fired = 0;
    for (int i = 0; i < 50; ++i) if (in.requestStand()) ++fired;
    check(fired == 1, "a held key requests the transition once, not repeatedly");
    in.feedKey("");

    press(in, "Z");
    check(!in.requestStand() && !in.requestRL() && !in.requestPassive() &&
          !in.requestReset(), "an unmapped key requests nothing");
}

void test_velocity(const std::string& robot_yaml)
{
    std::printf("[keyboard -> VelocityCommand]\n");
    InputManager in;
    const InputManager::Config cfg = shippedConfig(robot_yaml);
    in.configure(cfg);

    check(in.velocitySource() == InputManager::VelocitySource::None ||
          near(in.velocityCommand().vx, 0.0), "no key held -> zero command");

    press(in, cfg.key_forward);
    check(near(in.velocityCommand().vx, cfg.key_speed), "'i' -> vx = +speed");
    check(in.velocitySource() == InputManager::VelocitySource::Keyboard,
          "  and the keyboard owns the command");

    press(in, cfg.key_backward);
    check(near(in.velocityCommand().vx, -cfg.key_speed), "',' -> vx = -speed");

    press(in, cfg.key_yaw_left);
    check(near(in.velocityCommand().dyaw, cfg.key_turn), "'j' -> dyaw = +turn");
    check(near(in.velocityCommand().vx, -cfg.key_speed),
          "  and vx stays latched (combined command)");

    press(in, cfg.key_yaw_right);
    check(near(in.velocityCommand().dyaw, -cfg.key_turn), "'l' -> dyaw = -turn");
    press(in, cfg.key_left);
    check(near(in.velocityCommand().vy, cfg.key_speed), "'J' -> vy = +speed (left)");
    press(in, cfg.key_right);
    check(near(in.velocityCommand().vy, -cfg.key_speed), "'L' -> vy = -speed (right)");

    press(in, cfg.key_stop);
    const VelocityCommand z = in.velocityCommand();
    check(near(z.vx, 0.0) && near(z.vy, 0.0) && near(z.dyaw, 0.0),
          "'k' zeroes the whole command");
    check(in.velocitySource() != InputManager::VelocitySource::Keyboard,
          "  and releases the source, so /cmd_vel can take over again");

    // The envelope is shared with the stick: a key can never exceed it.
    InputManager::Config fast = cfg;
    fast.key_speed = 99.0;
    fast.key_turn  = 99.0;
    InputManager in2;
    in2.configure(fast);
    press(in2, fast.key_forward);
    check(near(in2.velocityCommand().vx, fast.vx_max),
          "an oversized key speed is clamped to vx_max, not passed through");
    press(in2, fast.key_yaw_left);
    check(near(in2.velocityCommand().dyaw, fast.dyaw_max),
          "  and to dyaw_max");
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
    ros::Time::init();

    std::printf("=== keyboard input tests ===\n");
    test_shipped_keys(dir + "/robot.yaml");
    test_fsm_triggers(dir + "/robot.yaml");
    test_velocity(dir + "/robot.yaml");

    std::printf("=== %d checks, %d failures ===\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
