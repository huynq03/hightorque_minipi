// DataRecorder -- high-rate log of what the controller measured and commanded.
//
// A thread of its own samples CtrlFSM::snapshot() (the same consistent copy the
// status dump uses) at `hz`, so nothing is written from the 1 kHz control loop.
// Each sample goes to
//   * a CSV file (`csv_path`): per joint q, dq, tau (measured, robot order; tau
//     in the joint's sign, unlike RobotState::robot_tau which keeps the motor's),
//     q_target, kp, kd (the command after Safety's clamp), the IMU, the FSM
//     state, and the running policy's step (seq, raw action, relative delta);
//   * optionally two sensor_msgs/JointState topics for live plotting
//     (rqt_plot, PlotJuggler) or rosbag:
//       <ns>/joint_states   position = q, velocity = dq, effort = measured tau
//       <ns>/joint_command  position = q_target, velocity = 0,
//                           effort = kp (q_target - q) - kd dq  (PD torque asked of the motor)
//
// While a safety fault is latched the bus carries Safety's damping override, not
// the snapshot command; the `faulted` column says so.
#pragma once

#include "FSM/ControlContext.h"

#include <ros/ros.h>

#include <atomic>
#include <fstream>
#include <string>
#include <thread>

namespace deploy
{

class CtrlFSM;

class DataRecorder
{
public:
    struct Config
    {
        double hz = 200.0;
        std::string csv_path;        ///< empty = no file
        bool publish = true;         ///< JointState topics
        std::string topic_ns = "mini_pi";
    };

    DataRecorder(const Config& cfg, const ControlContext& ctx, const CtrlFSM& fsm,
                 std::vector<std::string> joint_names);
    ~DataRecorder();

    /// Opens the file, advertises the topics, starts the thread. False + `error`
    /// when the file cannot be created.
    bool start(ros::NodeHandle& nh, std::string& error);
    void stop();

private:
    void loop_();
    void writeHeader_();

    Config cfg_;
    const ControlContext& ctx_;
    const CtrlFSM& fsm_;
    std::vector<std::string> names_;

    std::ofstream csv_;
    ros::Publisher pub_state_, pub_cmd_;
    std::thread thread_;
    std::atomic<bool> running_{false};
};

} // namespace deploy
