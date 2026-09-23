#include "hardware/SimHardware.h"

#include <ros/ros.h>

#include <chrono>
#include <thread>

namespace mini_pi
{

SimHardware::~SimHardware()
{
    link_.stop();
}

bool SimHardware::initialize(const Config& cfg)
{
    cfg_ = cfg;

    if (!link_.start(cfg_.link))
    {
        ROS_ERROR("SimHardware: cannot open the simulated low-level bus: %s",
                  link_.lastError().c_str());
        return false;
    }

    // Labels only. The real backend reads these out of the SDK; here they are
    // synthesised so the startup readiness dump and main.cpp's status table
    // have the same shape on both backends.
    motor_names_.clear();
    motor_ids_.clear();
    for (std::size_t i = 0; i < MINI_PI_DOF; ++i)
    {
        motor_names_.push_back("sim_motor_" + std::to_string(i));
        motor_ids_.push_back(static_cast<int>(i));
    }

    initialized_ = true;
    ROS_INFO("SimHardware: bus up. lowcmd -> %s:%d, lowstate <- :%d, dry_run=%s",
             cfg_.link.bridge_ip.c_str(), cfg_.link.lowcmd_port,
             cfg_.link.lowstate_port, cfg_.dry_run ? "true" : "false");

    if (cfg_.wait_for_bridge_s > 0.0)
    {
        // Purely cosmetic: the startup readiness wait would hold anyway, and
        // would hold with a
        // much better diagnostic. This just turns the common "simulator not
        // started" mistake into one clear line at startup instead of a stream
        // of readiness warnings.
        const auto t0 = Clock::now();
        while (link_.statePackets() == 0 &&
               age_seconds(t0) < cfg_.wait_for_bridge_s && ros::ok())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        if (link_.statePackets() == 0)
        {
            ROS_WARN("SimHardware: no LowState packet after %.1f s. Is mini_pi_mujoco "
                     "running, and does its config.yaml agree with these ports? "
                     "Continuing -- startup readiness holds the FSM until feedback arrives.",
                     cfg_.wait_for_bridge_s);
        }
        else
        {
            ROS_INFO("SimHardware: bridge is alive (sim_time=%.3f s).", link_.simTime());
        }
    }
    return true;
}

bool SimHardware::read(RobotState& state)
{
    if (!initialized_) { state.motor_valid = state.imu_valid = false; return false; }
    return link_.fillState(state);
}

bool SimHardware::write(const MotorCommand& cmd)
{
    if (!initialized_ || !cmd.valid) return false;
    if (cfg_.dry_run) return true;   // nothing ever reaches the bus

    // A previous kModeStop is implicitly cleared: the bridge latches stop only
    // until the next servo packet, mirroring the firmware stop being sticky
    // until a new frame arrives.
    return link_.send(cmd, lowlevel::kModeServo);
}

void SimHardware::protect()
{
    if (!initialized_) return;
    if (cfg_.dry_run)
    {
        ROS_WARN_THROTTLE(5.0, "SimHardware: dry_run -- protect() suppressed, "
                               "nothing transmitted.");
        return;
    }
    // Same numbers as HighTorqueHardware::protect(): kp = 0, kd = 1, zero
    // position/velocity/torque targets.
    link_.sendMode(lowlevel::kModeProtect);
}

void SimHardware::stop()
{
    if (!initialized_) return;
    if (cfg_.dry_run)
    {
        ROS_WARN("SimHardware: dry_run -- stop() suppressed, nothing transmitted.");
        return;
    }
    link_.sendMode(lowlevel::kModeStop);
}

HardwareStatus SimHardware::status() const
{
    // Deliberately the same decision tree as HighTorqueHardware::status(),
    // minus the two checks that have no simulated counterpart:
    //
    //   - motor_back_t::fault      : the bridge always reports 0. The wire
    //                                format carries the field so a fault can
    //                                be injected later without a change here.
    //   - sdkLimitTripped()        : robot::motor_position_limit_flag and
    //                                motor_torque_limit_flag are firmware
    //                                flags. They are DISABLED on the real
    //                                Mini-Pi anyway (every pos_limit_enable /
    //                                tor_limit_enable in
    //                                12dof_STM32H730_pi_lubancat_params.yaml
    //                                is false -- docs/source_mapping.md §5),
    //                                so this is not a behavioural difference
    //                                in the shipped configuration.
    //
    // Everything else -- staleness, motor validity, IMU freshness -- is judged
    // by the identical rule against the identical thresholds from safety.yaml.
    if (!initialized_) return HardwareStatus::Uninitialized;
    if (!link_.motorValid() || link_.staleMotorCount() > 0) return HardwareStatus::Stale;
    if (!link_.imuValid() || link_.imuAge() > link_.imuTimeout())
        return HardwareStatus::Stale;
    return HardwareStatus::Ok;
}

} // namespace mini_pi
