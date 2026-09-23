// HighTorqueHardware -- the only class in this package allowed to touch the
// HighTorque/livelybot SDK. Conceptually it replaces the Unitree pair
// (LowState_t + LowCmd_t) from unitree_rl_mjlab/deploy.
//
// It talks straight to livelybot_serial::robot, whose full source ships in
// livelybot_hardware_sdk_4_4_5/src/livelybot_serial. It deliberately does NOT
// use hightorque::RobotMotorGroup: that class only exists as a prebuilt
// .so (sim2real/lib/libsim2real_arm_lib.so) and drags in Eigen/ROS message
// types. See docs/source_mapping.md for the reasoning.
#pragma once

#include "control/Types.h"
#include "hardware/HardwareInterface.h"

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace livelybot_serial { class robot; }

namespace mini_pi
{

/// Real-robot backend. Behaviour is unchanged from the version that ran
/// Passive on the physical Mini-Pi; deriving from HardwareInterface added
/// `override` keywords and backendName() and nothing else.
class HighTorqueHardware : public HardwareInterface
{
public:
    struct Config
    {
        /// When true, read()/IMU/mapping all run but no motor frame is ever
        /// handed to the SDK. Default true -- see config/robot.yaml.
        bool dry_run = true;
        /// sensor_msgs/Imu topic. Evidence: yesense_driver.cpp:93 advertises
        /// "imu/data"; sim2real_master_node's string table contains "/imu/data".
        std::string imu_topic = "/imu/data";
        /// Freshness thresholds. These are NOT defaulted from robot.yaml:
        /// main() loads them from config/safety.yaml so SafetyManager and the
        /// hardware layer test feedback against one single source of truth.
        double motor_timeout_s = 0.1;
        double imu_timeout_s   = 0.2;
        /// Per-motor watchdog pushed into the firmware via robot::set_timeout().
        /// <= 0 disables the call (the SDK leaves its own default in place).
        int motor_watchdog_ms = 0;

        /// Sensor-to-base IMU mount, ZYX euler of the sensor frame in the base
        /// frame. {0,0,0} (identity) until MEASURED on the robot.
        std::array<double, 3> imu_mount_rpy{{0.0, 0.0, 0.0}};

        /// Motor feedback is REQUEST-DRIVEN on this bus: a motor answers a
        /// frame, it does not stream. When no command frame has gone out for
        /// `feedback_query_period_s`, read() transmits a read-only state query
        /// (MODE_MOTOR_STATE, 0x06) so that feedback keeps arriving during the
        /// startup readiness wait and for the whole of a dry run.
        ///   "state2" canport::send_get_motor_state_cmd2() (MODE_MOTOR_STATE2,
        ///            0x0A; reply carries mode + fault) -- what the SDK itself
        ///            uses for fun_v >= fun_v4 board firmware (Mini-Pi: fun_v 5)
        ///   "state"  canport::send_get_motor_state_cmd()  (MODE_MOTOR_STATE, 0x06)
        ///   "none"   never query (feedback only as a reply to commands)
        std::string feedback_query = "state2";
        double feedback_query_period_s = 0.002;
    };

    HighTorqueHardware() = default;
    ~HighTorqueHardware() override;

    HighTorqueHardware(const HighTorqueHardware&) = delete;
    HighTorqueHardware& operator=(const HighTorqueHardware&) = delete;

    /// Construct livelybot_serial::robot (which reads the `robot/...` params
    /// loaded from 12dof_STM32H730_pi_lubancat_params.yaml), subscribe to the
    /// IMU, and verify the discovered motor count.
    bool initialize(ros::NodeHandle& nh, const Config& cfg);

    /// Copy the latest motor feedback and IMU sample into `state`
    /// (motor-order fields and IMU only -- robot-order fields are filled by the
    /// caller through JointMapper). Returns state.motor_valid.
    bool read(RobotState& state) override;

    /// Queue `cmd` into the SDK tx buffers and flush the frame.
    /// A no-op that returns true when dry_run is set.
    bool write(const MotorCommand& cmd) override;

    HardwareStatus status() const override;

    /// SDK-native damping hold: kp = 0, kd = 1, target 0/0/0. This mirrors
    /// hightorque::HtdwMotor::protectMotor() exactly (htdw_motor.cpp).
    void protect() override;

    /// Firmware-level stop for every motor (motor::stop via robot::set_stop),
    /// followed by a flush. Honours dry_run.
    void stop() override;

    std::size_t motorCount() const override { return motor_count_; }
    /// Motors whose last frame is older than motor_timeout_s, as of the last
    /// read(). MINI_PI_DOF until the first successful read.
    std::size_t staleMotorCount() const { return stale_motor_count_; }
    double imuAge() const { return age_seconds(imu_stamp_); }
    bool dryRun() const override { return cfg_.dry_run; }
    const std::vector<std::string>& motorNames() const override { return motor_names_; }
    const std::vector<int>& motorIds() const override { return motor_ids_; }
    const char* backendName() const override { return "hightorque"; }

    /// True once initialize() succeeded.
    bool initialized() const override { return initialized_; }

    /// Raised by the SDK when a configured position/torque limit trips
    /// (robot::motor_position_limit_flag / motor_torque_limit_flag).
    bool sdkLimitTripped() const;

    /// Read-only state queries transmitted so far (dry-run diagnostics).
    std::uint64_t feedbackQueries() const { return feedback_queries_; }

private:
    void imuCallback(const sensor_msgs::ImuConstPtr& msg);

    Config cfg_;
    std::shared_ptr<livelybot_serial::robot> rb_;
    ros::Subscriber imu_sub_;

    std::size_t motor_count_ = 0;
    std::vector<std::string> motor_names_;
    std::vector<int> motor_ids_;

    /// Per-motor last-frame wall time, as reported by motor_back_t::time.
    std::array<double, MINI_PI_DOF> last_feedback_time_{};
    /// Per-motor steady_clock stamp of when that value last advanced.
    std::array<TimePoint, MINI_PI_DOF> motor_stamp_per_{};
    std::array<bool, MINI_PI_DOF> motor_seen_{};
    std::size_t stale_motor_count_ = MINI_PI_DOF;

    void queryFeedback_();

    TimePoint last_tx_{};   ///< last command OR query frame transmitted
    std::uint64_t feedback_queries_ = 0;

    mutable std::mutex imu_mutex_;
    std::array<double, 3> imu_ang_vel_{};
    std::array<double, 4> imu_quat_{{0, 0, 0, 1}};
    TimePoint imu_stamp_{};
    bool imu_valid_ = false;

    TimePoint motor_stamp_{};   ///< oldest across all motors
    bool motor_valid_ = false;
    bool motor_fault_ = false;
    bool initialized_ = false;
};

} // namespace mini_pi
