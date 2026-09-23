#include "input/InputManager.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace mini_pi
{
namespace
{
constexpr int kPassive = 0, kStand = 1, kRL = 2, kReset = 3;
}

bool InputManager::initialize(ros::NodeHandle& nh, const Config& cfg)
{
    configure(cfg);
    joy_sub_     = nh.subscribe(cfg_.joy_topic, 1, &InputManager::joyCallback, this);
    cmd_vel_sub_ = nh.subscribe(cfg_.cmd_vel_topic, 1, &InputManager::cmdVelCallback, this);
    ROS_INFO("InputManager: joy='%s' cmd_vel='%s'",
             cfg_.joy_topic.c_str(), cfg_.cmd_vel_topic.c_str());

    if (cfg_.enable_keyboard)
    {
        if (keyboard_.start())
        {
            ROS_INFO("InputManager: keyboard ENABLED on this terminal. "
                     "%s/%s/%s/%s move, %s/%s yaw, %s stop | "
                     "%s Passive, %s FixStand, %s RL, %s reset fault.",
                     cfg_.key_forward.c_str(), cfg_.key_backward.c_str(),
                     cfg_.key_left.c_str(), cfg_.key_right.c_str(),
                     cfg_.key_yaw_left.c_str(), cfg_.key_yaw_right.c_str(),
                     cfg_.key_stop.c_str(), cfg_.key_passive.c_str(),
                     cfg_.key_stand.c_str(), cfg_.key_rl.c_str(),
                     cfg_.key_reset.c_str());
        }
        else
        {
            // Normal for a scripted or backgrounded run. /joy and /cmd_vel are
            // unaffected, so this is information, not a failure.
            ROS_INFO("InputManager: keyboard disabled (%s). Use /joy, or "
                     "teleop_twist_keyboard on '%s'.",
                     keyboard_.unavailableReason().c_str(), cfg_.cmd_vel_topic.c_str());
        }
    }
    return true;
}

void InputManager::feedKey(const std::string& key)
{
    // Edge, not level: a key held down (or auto-repeating) must request a
    // transition once, exactly like a held gamepad combo does.
    const bool edge = (key != last_key_) && !key.empty();
    last_key_ = key;
    if (!edge) return;

    // Echo is off while the reader owns the terminal, so a key produces no
    // visible trace of its own. Without this line there is no way for an
    // operator to tell "the key was not read" from "the key was read and the
    // FSM declined it", which are very different problems.
    const auto accepted = [&key](const char* what) {
        ROS_INFO("InputManager: key '%s' -> %s", key.c_str(), what);
    };

    if (key == cfg_.key_passive) { accepted("request Passive");  combo_edge_[kPassive] = true; return; }
    if (key == cfg_.key_stand)   { accepted("request FixStand"); combo_edge_[kStand]   = true; return; }
    if (key == cfg_.key_rl)      { accepted("request RL");       combo_edge_[kRL]      = true; return; }
    if (key == cfg_.key_reset)   { accepted("request fault reset"); combo_edge_[kReset] = true; return; }

    // Locomotion. Each key sets one axis and leaves the others latched, so
    // forward-then-yaw gives an arc, as it does with teleop_twist_keyboard.
    VelocityCommand c = key_cmd_;
    if      (key == cfg_.key_forward)   c.vx   =  cfg_.key_speed;
    else if (key == cfg_.key_backward)  c.vx   = -cfg_.key_speed;
    else if (key == cfg_.key_left)      c.vy   =  cfg_.key_speed;
    else if (key == cfg_.key_right)     c.vy   = -cfg_.key_speed;
    else if (key == cfg_.key_yaw_left)  c.dyaw =  cfg_.key_turn;
    else if (key == cfg_.key_yaw_right) c.dyaw = -cfg_.key_turn;
    else if (key == cfg_.key_stop)      c = VelocityCommand{};
    else return;   // a key we do not map changes nothing

    // Same envelope as the stick and /cmd_vel, so no source can mean something
    // different by "as fast as this robot goes".
    c.vx   = std::clamp(c.vx,   cfg_.vx_min,   cfg_.vx_max);
    c.vy   = std::clamp(c.vy,   cfg_.vy_min,   cfg_.vy_max);
    c.dyaw = std::clamp(c.dyaw, cfg_.dyaw_min, cfg_.dyaw_max);

    ROS_INFO("InputManager: key '%s' -> vx=%.3f vy=%.3f dyaw=%.3f",
             key.c_str(), c.vx, c.vy, c.dyaw);

    std::lock_guard<std::mutex> lk(mutex_);
    key_cmd_ = c;
    key_cmd_active_ = (c.vx != 0.0 || c.vy != 0.0 || c.dyaw != 0.0);
}

double InputManager::axisToRange(const sensor_msgs::Joy& msg, int idx,
                                 double lo, double hi, bool& outside_deadzone) const
{
    if (idx < 0 || idx >= static_cast<int>(msg.axes.size())) return 0.0;

    const double raw = msg.axes[idx];
    // A stick that reports NaN must not be read as "hold this heading".
    if (!std::isfinite(raw)) return 0.0;

    const double a  = std::clamp(raw, -1.0, 1.0);
    const double dz = std::clamp(cfg_.joy_deadzone, 0.0, 0.99);
    if (std::abs(a) <= dz) return 0.0;

    outside_deadzone = true;
    // Rescale the remaining travel back onto [0, 1] so the command leaves zero
    // continuously rather than stepping to dz * range.
    const double unit = (std::abs(a) - dz) / (1.0 - dz);
    // `lo` is the value at full NEGATIVE deflection and is itself negative, so
    // multiplying by it carries the sign. Writing `-lo` here would send a
    // backward stick forward at the reverse speed limit.
    return (a > 0.0) ? unit * hi : unit * lo;
}

void InputManager::joyCallback(const sensor_msgs::JoyConstPtr& msg)
{
    bool active = false;
    // Computed before the lock: this touches only the message and cfg_, and
    // cfg_ is written once in initialize() before any subscriber exists.
    VelocityCommand c;
    c.vx   = axisToRange(*msg, cfg_.axis_vx,   cfg_.vx_min,   cfg_.vx_max,   active);
    c.vy   = axisToRange(*msg, cfg_.axis_vy,   cfg_.vy_min,   cfg_.vy_max,   active);
    c.dyaw = axisToRange(*msg, cfg_.axis_dyaw, cfg_.dyaw_min, cfg_.dyaw_max, active);

    std::lock_guard<std::mutex> lk(mutex_);
    buttons_.assign(msg->buttons.begin(), msg->buttons.end());
    joy_cmd_         = c;
    joy_axes_active_ = active;
    joy_stamp_       = Clock::now();
    joy_seen_        = true;
}

void InputManager::cmdVelCallback(const geometry_msgs::TwistConstPtr& msg)
{
    std::lock_guard<std::mutex> lk(mutex_);
    // Same envelope as the stick, so the two sources cannot mean different
    // things by "as fast as this robot goes".
    cmd_vel_.vx   = std::clamp(msg->linear.x,  cfg_.vx_min,   cfg_.vx_max);
    cmd_vel_.vy   = std::clamp(msg->linear.y,  cfg_.vy_min,   cfg_.vy_max);
    cmd_vel_.dyaw = std::clamp(msg->angular.z, cfg_.dyaw_min, cfg_.dyaw_max);
    cmd_vel_stamp_ = Clock::now();
}

bool InputManager::button(int idx) const
{
    if (idx < 0 || idx >= static_cast<int>(buttons_.size())) return false;
    return buttons_[idx] != 0;
}

void InputManager::latchEdge(int combo, bool now)
{
    if (now && !combo_prev_[combo]) combo_edge_[combo] = true;
    combo_prev_[combo] = now;
}

void InputManager::update()
{
    std::array<bool, 4> now{};
    {
        std::lock_guard<std::mutex> lk(mutex_);
        now[kPassive] = button(cfg_.btn_lb) && button(cfg_.btn_b);
        now[kStand]   = button(cfg_.btn_lb) && button(cfg_.btn_a);
        now[kRL]      = button(cfg_.btn_rb) && button(cfg_.btn_a);
        now[kReset]   = button(cfg_.btn_rb) && button(cfg_.btn_y);
    }
    for (std::size_t i = 0; i < now.size(); ++i) latchEdge(static_cast<int>(i), now[i]);

    // Non-blocking: key() reads a value the reader thread already decoded.
    // Nothing on the 1 kHz control thread ever waits on stdin.
    if (keyboard_.running()) feedKey(keyboard_.key());
}

bool InputManager::requestPassive() { bool e = combo_edge_[kPassive]; combo_edge_[kPassive] = false; return e; }
bool InputManager::requestStand()   { bool e = combo_edge_[kStand];   combo_edge_[kStand]   = false; return e; }
bool InputManager::requestRL()      { bool e = combo_edge_[kRL];      combo_edge_[kRL]      = false; return e; }
bool InputManager::requestReset()   { bool e = combo_edge_[kReset];   combo_edge_[kReset]   = false; return e; }

VelocityCommand InputManager::velocityCommand() const
{
    std::lock_guard<std::mutex> lk(mutex_);

    // See the header for the precedence rule. The two sources are selected
    // between, never summed.
    if (joy_axes_active_ && age_seconds(joy_stamp_) <= cfg_.joy_timeout_s)
    {
        last_source_ = VelocitySource::Joy;
        return joy_cmd_;
    }
    // Keyboard sits between them, gated on a non-zero latched command exactly
    // as the stick is gated on being outside its deadzone. Pressing the stop
    // key therefore does not merely command zero, it hands the robot back to
    // /cmd_vel -- the same way releasing the stick hands it to the keyboard.
    if (key_cmd_active_)
    {
        last_source_ = VelocitySource::Keyboard;
        return key_cmd_;
    }
    if (age_seconds(cmd_vel_stamp_) <= cfg_.cmd_vel_timeout_s)
    {
        last_source_ = VelocitySource::CmdVel;
        return cmd_vel_;
    }
    last_source_ = VelocitySource::None;
    return VelocityCommand{};
}

InputManager::VelocitySource InputManager::velocitySource() const
{
    std::lock_guard<std::mutex> lk(mutex_);
    return last_source_;
}

std::string InputManager::describe() const
{
    char buf[768];
    std::snprintf(buf, sizeof(buf),
                  "InputManager: LB+A=FixStand  RB+A=RL  LB+B=Passive  RB+Y=reset fault\n"
                  "InputManager: keyboard %s -- %s=FixStand %s=RL %s=Passive %s=reset | "
                  "%s fwd %s back %s/%s strafe %s/%s yaw %s stop (%.2f m/s, %.2f rad/s)\n"
                  "InputManager: axes vx=%d vy=%d yaw=%d  deadzone=%.3f  "
                  "ranges vx[%.2f,%.2f] vy[%.2f,%.2f] yaw[%.2f,%.2f]  "
                  "(priority: joy off-centre > keyboard non-zero > cmd_vel > zero)",
                  keyboard_.running() ? "ON" : "off",
                  cfg_.key_stand.c_str(), cfg_.key_rl.c_str(),
                  cfg_.key_passive.c_str(), cfg_.key_reset.c_str(),
                  cfg_.key_forward.c_str(), cfg_.key_backward.c_str(),
                  cfg_.key_left.c_str(), cfg_.key_right.c_str(),
                  cfg_.key_yaw_left.c_str(), cfg_.key_yaw_right.c_str(),
                  cfg_.key_stop.c_str(), cfg_.key_speed, cfg_.key_turn,
                  cfg_.axis_vx, cfg_.axis_vy, cfg_.axis_dyaw, cfg_.joy_deadzone,
                  cfg_.vx_min, cfg_.vx_max, cfg_.vy_min, cfg_.vy_max,
                  cfg_.dyaw_min, cfg_.dyaw_max);
    return buf;
}

} // namespace mini_pi
