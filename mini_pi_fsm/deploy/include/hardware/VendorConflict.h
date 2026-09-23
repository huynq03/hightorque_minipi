// Detection of HighTorque vendor processes that would fight mini_pi_fsm for the
// motor bus.
//
// Only ONE process may own the livelybot serial link (/dev/ttyACM*): a second
// writer interleaves frames with ours and a second reader steals replies. On
// the Mini-Pi the vendor stack is started at login by an XFCE autostart entry
// (~/.config/autostart/"pi plus newTraj.desktop" -> ~/sim2real_install_pi.sh
// -> roslaunch sim2real_master joy_control_pi.launch), which launches
// sim2real_master_node (respawn=true), lr_control and its own yesense IMU node.
//
// This scan is used by main() to REFUSE the hightorque backend -- in dry run
// too -- while any of those are alive. It never kills or disables anything.
#pragma once

#include <string>
#include <vector>

namespace mini_pi
{

struct VendorConflict
{
    int pid = 0;
    std::string what;   ///< human-readable reason
};

/// Scan `proc_root` (normally "/proc") for:
///   * vendor controller executables: sim2real_master_node, lr_control_node,
///     rl_pd_controller, hightorque_hardware_sdk_node, or anything under a
///     livelybot_bringup/ path (motor_feedback, motor_set_zero, ...);
///   * a roslaunch whose arguments name sim2real_master, joy_control_pi or
///     sim2real.launch (it respawns the above and runs a second IMU driver);
///   * any other process holding a /dev/ttyACM* device open.
/// `self_pid` is excluded. Processes whose /proc entries are unreadable are
/// skipped (their cmdline is still world-readable on Linux).
std::vector<VendorConflict> findVendorConflicts(const std::string& proc_root, int self_pid);

} // namespace mini_pi
