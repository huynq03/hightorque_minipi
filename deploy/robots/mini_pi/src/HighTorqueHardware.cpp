#include "HighTorqueHardware.h"

#include "hardware/robot.h"   // livelybot_serial::robot
#include "hardware/motor.h"   // motor, motor_back_t
#include "hardware/canport.h" // canport::send_get_motor_state_cmd

#include <algorithm>
#include <cmath>
#include <limits>

namespace mini_pi
{

HighTorqueHardware::~HighTorqueHardware()
{
    // livelybot_serial::robot's destructor already issues set_stop() +
    // motor_send_2() twice and joins its receive threads (robot.cc:149).
    rb_.reset();
}

bool HighTorqueHardware::initialize(ros::NodeHandle& nh, const Config& cfg)
{
    cfg_ = cfg;

    try
    {
        // The constructor reads every `robot/...` ROS param, opens the serial
        // ports, spawns the receive threads and probes the motors.
        rb_ = std::make_shared<livelybot_serial::robot>();
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("HighTorqueHardware: livelybot_serial::robot construction failed: %s", e.what());
        return false;
    }

    motor_count_ = rb_->Motors.size();
    if (motor_count_ != MINI_PI_DOF)
    {
        ROS_ERROR("HighTorqueHardware: SDK discovered %zu motors, this package expects %zu. "
                  "Check the robot_param yaml loaded into the parameter server.",
                  motor_count_, MINI_PI_DOF);
        rb_.reset();
        return false;
    }

    motor_names_.clear();
    motor_ids_.clear();
    for (auto* m : rb_->Motors)
    {
        motor_names_.push_back(m->get_motor_name());
        motor_ids_.push_back(m->get_motor_id());
    }

    if (cfg_.motor_watchdog_ms > 0)
    {
        if (cfg_.dry_run)
        {
            ROS_WARN("HighTorqueHardware: dry_run -- skipping set_timeout(%d ms)",
                     cfg_.motor_watchdog_ms);
        }
        else
        {
            rb_->set_timeout(static_cast<int16_t>(cfg_.motor_watchdog_ms));
        }
    }

    imu_sub_ = nh.subscribe(cfg_.imu_topic, 1, &HighTorqueHardware::imuCallback, this);

    if (cfg_.feedback_query != "state" && cfg_.feedback_query != "state2" &&
        cfg_.feedback_query != "none")
    {
        ROS_ERROR("HighTorqueHardware: feedback_query must be 'state2', 'state' or 'none', "
                  "got '%s'",
                  cfg_.feedback_query.c_str());
        rb_.reset();
        return false;
    }

    initialized_ = true;
    ROS_INFO("HighTorqueHardware: initialized, %zu motors on %zu CAN ports, dry_run=%s, "
             "imu_topic=%s, imu_mount_rpy=[%.4f %.4f %.4f], feedback_query=%s",
             motor_count_, rb_->CANPorts.size(), cfg_.dry_run ? "true" : "false",
             cfg_.imu_topic.c_str(), cfg_.imu_mount_rpy[0], cfg_.imu_mount_rpy[1],
             cfg_.imu_mount_rpy[2], cfg_.feedback_query.c_str());
    return true;
}

void HighTorqueHardware::imuCallback(const sensor_msgs::ImuConstPtr& msg)
{
    std::lock_guard<std::mutex> lk(imu_mutex_);
    imu_ang_vel_ = {msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z};
    imu_quat_    = {msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w};
    imu_stamp_   = Clock::now();
    imu_valid_   = true;
}

void HighTorqueHardware::queryFeedback_()
{
    // canport::send_get_motor_state_cmd{,2}() rewrite the port's frame header to
    // MODE_MOTOR_STATE (0x06) / MODE_MOTOR_STATE2 (0x0A) with a one-byte payload
    // and send it. It carries no
    // position, velocity, torque or gain -- it cannot move a motor.
    //
    // robot::send_get_motor_state_cmd() is deliberately NOT used: on fun_v1
    // firmware with a non-zero control_type it falls back to commanding
    // velocity(0) on every motor, which is actuation.
    // "state2" is what robot::send_get_motor_state_cmd() itself picks for
    // fun_v >= fun_v4 firmware, and its reply also carries mode + fault code.
    if (cfg_.feedback_query == "state2")
        for (canport* cp : rb_->CANPorts) cp->send_get_motor_state_cmd2();
    else
        for (canport* cp : rb_->CANPorts) cp->send_get_motor_state_cmd();
    ++feedback_queries_;
}

bool HighTorqueHardware::read(RobotState& state)
{
    if (!initialized_) { state.motor_valid = state.imu_valid = false; return false; }

    const TimePoint now = Clock::now();

    // Keep feedback flowing when nothing else is being transmitted: during the
    // readiness wait (before any command) and throughout a dry run. A live
    // FSM writes every cycle, so this never fires alongside real commands.
    if (cfg_.feedback_query != "none" &&
        std::chrono::duration<double>(now - last_tx_).count() >= cfg_.feedback_query_period_s)
    {
        queryFeedback_();
        last_tx_ = now;
    }

    // --- motors ---------------------------------------------------------
    // Freshness is tracked PER MOTOR. Dereferencing a motor object always
    // succeeds and proves nothing, so the only evidence a frame arrived is
    // motor_back_t::time advancing. We use it purely as a change detector and
    // keep our own steady_clock stamp, because the SDK stamps with
    // ros::Time::now() which is not monotonic.
    bool all_finite = true;
    bool all_seen = true;
    bool any_fault = false;
    std::size_t stale = 0;
    TimePoint oldest = now;

    for (std::size_t i = 0; i < MINI_PI_DOF; ++i)
    {
        const motor_back_t* d = rb_->Motors[i]->get_current_motor_state();

        state.motor_q[i]     = d->position;
        state.motor_dq[i]    = d->velocity;
        state.motor_tau[i]   = d->torque;
        state.motor_fault[i] = static_cast<int>(d->fault);
        if (d->fault > 0) any_fault = true;

        const bool finite = std::isfinite(d->position) && std::isfinite(d->velocity)
                         && std::isfinite(d->torque);
        all_finite = all_finite && finite;

        if (d->time > last_feedback_time_[i] && finite)
        {
            last_feedback_time_[i] = d->time;
            motor_stamp_per_[i] = now;
            motor_seen_[i] = true;
        }

        if (!motor_seen_[i])
        {
            all_seen = false;
            state.motor_age[i] = std::numeric_limits<double>::infinity();
            state.motor_fresh[i] = false;
            ++stale;
            continue;
        }

        const double age = std::chrono::duration<double>(now - motor_stamp_per_[i]).count();
        state.motor_age[i] = age;
        state.motor_fresh[i] = (age <= cfg_.motor_timeout_s);
        if (!state.motor_fresh[i]) ++stale;
        if (motor_stamp_per_[i] < oldest) oldest = motor_stamp_per_[i];
    }

    stale_motor_count_ = stale;
    state.stale_motor_count = stale;
    motor_fault_ = any_fault;
    motor_valid_ = all_seen && all_finite;

    motor_stamp_ = all_seen ? oldest : TimePoint{};
    state.motor_stamp = motor_stamp_;
    state.motor_valid = motor_valid_;

    // --- IMU -----------------------------------------------------------
    {
        std::lock_guard<std::mutex> lk(imu_mutex_);
        state.imu_raw_angular_velocity = imu_ang_vel_;
        state.imu_raw_orientation      = imu_quat_;
        state.imu_stamp                = imu_stamp_;
        state.imu_valid                = imu_valid_;
    }
    // Sensor frame -> base frame. Identity unless robot.yaml imu.mount_rpy
    // says otherwise; see apply_imu_mount() in Types.h.
    state.angular_velocity = state.imu_raw_angular_velocity;
    state.orientation      = state.imu_raw_orientation;
    apply_imu_mount(cfg_.imu_mount_rpy, state.orientation, state.angular_velocity);
    state.imu_age = state.imu_valid
                  ? std::chrono::duration<double>(now - state.imu_stamp).count()
                  : std::numeric_limits<double>::infinity();

    // quaternion (x, y, z, w) -> ZYX euler. The arithmetic moved verbatim into
    // Types.h so SimHardware derives rpy by the identical formula; see the
    // comment there. Nothing about the result changed.
    if (state.imu_valid)
    {
        quat_xyzw_to_rpy(state.orientation, state.rpy);
    }

    return state.motor_valid;
}

bool HighTorqueHardware::write(const MotorCommand& cmd)
{
    if (!initialized_ || !cmd.valid) return false;
    // dry_run: no command frame ever reaches the serial port. The only frames
    // this class transmits in a dry run are the read-only state queries from
    // read(). (The SDK itself additionally sends motor-version queries while
    // constructing livelybot_serial::robot, and MODE_STOP from its destructor.)
    if (cfg_.dry_run) return true;

    for (std::size_t i = 0; i < MINI_PI_DOF; ++i)
    {
        // POS_VEL_TQE_KP_KD2 -- matches `control_type: 12` in
        // 12dof_STM32H730_pi_lubancat_params.yaml and MotorControlType::
        // POS_VEL_TQE_KP_KD2 in sim2real_sdk/include/motor/motor_base.h.
        rb_->Motors[i]->pos_vel_tqe_kp_kd2(
            static_cast<float>(cmd.q[i]),
            static_cast<float>(cmd.dq[i]),
            static_cast<float>(cmd.torque[i]),
            static_cast<float>(cmd.kp[i]),
            static_cast<float>(cmd.kd[i]));
    }
    rb_->motor_send_2();
    last_tx_ = Clock::now();
    return true;
}

void HighTorqueHardware::protect()
{
    if (!initialized_) return;
    if (cfg_.dry_run)
    {
        ROS_WARN_THROTTLE(5.0, "HighTorqueHardware: dry_run -- protect() suppressed, "
                               "nothing transmitted.");
        return;
    }
    // Identical to hightorque::HtdwMotor::protectMotor(): kp = 0, kd = 1,
    // zero position/velocity/torque targets -> pure joint damping.
    for (std::size_t i = 0; i < MINI_PI_DOF; ++i)
    {
        rb_->Motors[i]->pos_vel_tqe_kp_kd2(0.f, 0.f, 0.f, 0.f, 1.f);
    }
    rb_->motor_send_2();
}

void HighTorqueHardware::stop()
{
    if (!initialized_) return;
    if (cfg_.dry_run)
    {
        ROS_WARN("HighTorqueHardware: dry_run -- stop() suppressed, nothing transmitted.");
        return;
    }
    rb_->set_stop();
    rb_->motor_send_2();
}

bool HighTorqueHardware::sdkLimitTripped() const
{
    if (!initialized_) return false;
    return rb_->motor_position_limit_flag != 0 || rb_->motor_torque_limit_flag != 0;
}

HardwareStatus HighTorqueHardware::status() const
{
    if (!initialized_) return HardwareStatus::Uninitialized;
    if (motor_fault_ || sdkLimitTripped()) return HardwareStatus::Fault;
    // Any single stale motor makes the whole transport unhealthy: a command is
    // only ever sent to all 12 at once.
    if (!motor_valid_ || stale_motor_count_ > 0) return HardwareStatus::Stale;
    if (!imu_valid_ || age_seconds(imu_stamp_) > cfg_.imu_timeout_s)
        return HardwareStatus::Stale;
    return HardwareStatus::Ok;
}

} // namespace mini_pi
