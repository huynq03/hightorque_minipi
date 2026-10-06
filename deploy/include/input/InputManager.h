// InputManager -- all ROS input in one place. FSM states never register a
// callback; they only read the normalized snapshot this class publishes.
//
// Topics and button layout come from the existing HighTorque teleop setup:
//   /cmd_vel   geometry_msgs/Twist   (install/share/sim2real_master/joy.yaml,
//              `teleop.walk`, and the string table of sim2real_master_node)
//   /joy       sensor_msgs/Joy       (joy_node; joy.yaml maps buttons 0..10 to
//              a/b/x/y/lb/rb/back/start/center/L/R)
// We subscribe to /joy directly rather than sim2real_msg/Joy so that this
// package does not depend on the sim2real_msg message package.
#pragma once

#include "common/Types.h"
#include "input/KeyboardReader.h"

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

#include <atomic>
#include <mutex>
#include <string>
#include <vector>

namespace deploy
{

class InputManager
{
public:
    struct Config
    {
        std::string joy_topic     = "/joy";
        std::string cmd_vel_topic = "/cmd_vel";

        /// Button indices (Logitech F710 / xbox layout, per joy.yaml).
        int btn_a = 0, btn_b = 1, btn_x = 2, btn_y = 3;
        int btn_lb = 4, btn_rb = 5;

        /// Stick axis indices.
        ///
        /// SOURCE-VERIFIED from install/share/sim2real_master/joy.yaml,
        /// `teleop.walk.axis_mappings`: axis 1 -> linear.x, axis 0 -> linear.y,
        /// axis 3 -> angular.z, all at scale 1.0. The same file's
        /// `mode_change` block names those axes l_vertical, l_horizontal and
        /// r_horizontal, so this is the left stick for translation and the
        /// right stick for yaw. No sign flip: the F710's +1 directions are
        /// stick-up, stick-left and stick-left, which are REP-103's +x
        /// (forward), +y (left) and +yaw (counter-clockwise).
        int axis_vx = 1, axis_vy = 0, axis_dyaw = 3;

        /// Applied per axis, then the remainder is rescaled so that the
        /// command is continuous across the deadzone edge instead of jumping
        /// to `deadzone * range` the moment the stick is pushed.
        ///
        /// This is the single owner of the deadzone. joy_node applies the
        /// same shape to the same normalized range, so the launch file pins
        /// its `deadzone` to 0.0 -- two of these compose instead of agreeing.
        double joy_deadzone = 0.05;

        /// A /joy or /cmd_vel sample older than this is treated as absent.
        /// joy_node publishes only on change unless its `autorepeat_rate` is
        /// set, which is why the launch file sets it.
        double cmd_vel_timeout_s = 0.5;
        double joy_timeout_s     = 0.5;

        /// Operator envelope, per axis, asymmetric.
        ///
        /// Set by main(), together with key_speed / key_turn below, from the
        /// selected policy package's envelope (RLPolicyRunner::operatorEnvelope:
        /// the policy command limits + deploy.yaml operator.keyboard). Zero by
        /// default: without a policy nothing can be commanded. The policy
        /// clamps again itself, so this is not the safety boundary -- it is
        /// what a full stick deflection means.
        double vx_min = 0.0, vx_max = 0.0;
        double vy_min = 0.0, vy_max = 0.0;
        double dyaw_min = 0.0, dyaw_max = 0.0;

        // --- keyboard ------------------------------------------------------
        /// The built-in raw-stdin reader. It starts only when the controller
        /// owns an interactive foreground terminal, so leaving this on is safe
        /// for scripted and backgrounded runs -- see KeyboardReader::start().
        bool enable_keyboard = true;

        /// Magnitude a locomotion key commands, before the range clamp. Set by
        /// main() from the policy package's operator.keyboard; zero without one.
        double key_speed = 0.0;
        double key_turn  = 0.0;

        /// Key table. Locomotion keys are teleop_twist_keyboard's, preserved
        /// exactly so the built-in reader and the /cmd_vel node agree.
        /// The FSM keys are new -- nothing in this package defined any.
        std::string key_forward  = "i";
        std::string key_backward = ",";
        std::string key_left     = "J";   ///< strafe
        std::string key_right    = "L";   ///< strafe
        std::string key_yaw_left = "j";
        std::string key_yaw_right = "l";
        std::string key_stop     = "k";   ///< zero the velocity command

        std::string key_passive = "1";
        std::string key_stand   = "2";
        std::string key_rl      = "3";
        std::string key_reset   = "0";
    };

    /// Applies configuration without touching ROS or the terminal. Split out
    /// of initialize() so the key mapping can be unit-tested with no master.
    void configure(const Config& cfg) { cfg_ = cfg; }

    /// configure() + the /joy and /cmd_vel subscriptions + the keyboard reader.
    bool initialize(ros::NodeHandle& nh, const Config& cfg);

    /// Must be called once per control cycle: samples the keyboard and latches
    /// the rising edges of both input devices, so each press is consumed
    /// exactly once. Never blocks -- the keyboard is read on its own thread.
    void update();

    /// Applies one key exactly as update() would. Called by update() with the
    /// reader's current key; also the seam the keyboard unit test drives, so
    /// the mapping is testable without a terminal. Pass "" for "no key held".
    void feedKey(const std::string& key);

    // --- transition requests (each returns true once per press) -----------
    bool requestPassive();    ///< LB + B
    bool requestStand();      ///< LB + A
    bool requestRL();         ///< RB + A
    bool requestReset();      ///< RB + Y  (clears a latched fault)

    /// The operator's velocity intent, from whichever source owns it.
    ///
    /// Precedence is deterministic and evaluated fresh on every call:
    ///
    ///   1. /joy, while it is fresh (<= joy_timeout_s) AND at least one mapped
    ///      axis is outside the deadzone;
    ///   2. otherwise /cmd_vel, while it is fresh (<= cmd_vel_timeout_s);
    ///   3. otherwise zero.
    ///
    /// The two sources are never blended. A physical stick therefore takes the
    /// robot away from the keyboard the instant it is pushed, and hands it
    /// back when it is released -- the person holding the robot's own
    /// controller wins, which is the direction that is safe to get wrong.
    /// A centred stick costs the keyboard nothing.
    VelocityCommand velocityCommand() const;

    /// Which source the last velocityCommand() call used. Diagnostics only.
    enum class VelocitySource { None, Joy, Keyboard, CmdVel };
    VelocitySource velocitySource() const;

    bool joyConnected() const { return joy_seen_; }
    /// True when the built-in reader actually owns the terminal.
    bool keyboardActive() const { return keyboard_.running(); }
    std::string describe() const;

private:
    void joyCallback(const sensor_msgs::JoyConstPtr& msg);
    /// Sets combo_edge_[i] on a false->true transition.
    void latchEdge(int combo, bool now);
    void cmdVelCallback(const geometry_msgs::TwistConstPtr& msg);
    bool button(int idx) const;

    /// Deadzone, rescale, then map [-1, 1] onto [lo, hi] with the two halves
    /// scaled independently, so an asymmetric range such as vx [-0.3, 0.6]
    /// still gives zero at stick centre.
    double axisToRange(const sensor_msgs::Joy& msg, int idx,
                       double lo, double hi, bool& outside_deadzone) const;

    Config cfg_;
    ros::Subscriber joy_sub_;
    ros::Subscriber cmd_vel_sub_;

    mutable std::mutex mutex_;
    std::vector<int> buttons_;
    VelocityCommand cmd_vel_;
    TimePoint cmd_vel_stamp_{};

    VelocityCommand joy_cmd_;
    TimePoint joy_stamp_{};
    bool joy_axes_active_ = false;   ///< any mapped axis outside the deadzone
    mutable VelocitySource last_source_ = VelocitySource::None;

    std::atomic<bool> joy_seen_{false};

    /// Keyboard velocity is STICKY: a key sets the command and it holds until
    /// another key changes it or the stop key zeroes it. Terminal auto-repeat
    /// has a ~0.5 s delay before it starts, so a held-key model would blink the
    /// command back to zero between the first byte and the repeat; latching
    /// also matches how /cmd_vel and teleop_twist_keyboard already behave.
    VelocityCommand key_cmd_;
    bool key_cmd_active_ = false;    ///< a non-zero keyboard command is latched

    KeyboardReader keyboard_;
    std::string last_key_;           ///< for edge detection, update() only

    // latched-edge state, only touched from update()/request*()
    std::array<bool, 4> combo_prev_{};
    std::array<bool, 4> combo_edge_{};
};

} // namespace deploy
