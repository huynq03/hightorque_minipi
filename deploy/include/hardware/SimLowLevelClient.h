// SimLowLevelClient -- the controller's end of the simulated low-level bus.
//
// This is the piece that takes the place of livelybot_serial::robot when the
// robot underneath is MuJoCo: it owns the socket, runs the receive thread, and
// turns incoming LowState packets into the same RobotState fields that
// HighTorqueHardware::read() produces.
//
// It is ROS-free ON PURPOSE: SimHardware (which does link roscpp) is a thin
// adapter over this class, so the bus, the freshness logic and the RobotState
// assembly -- the majority of the deploy-side simulation code -- can be driven
// against a live MuJoCo bridge with no ROS installation.
//
// =========================== FRESHNESS SEMANTICS ===========================
// Reproduced deliberately from HighTorqueHardware::read(), because
// Safety and the startup readiness check judge both backends by the
// same numbers:
//
//   - The sender's timestamp is NEVER used to decide freshness. The incoming
//     `seq` is used purely as a CHANGE DETECTOR, and the receiver stamps with
//     its own steady_clock. This is exactly how the real layer treats
//     `motor_back_t::time` (which is a non-monotonic ros::Time).
//   - Age is tracked PER MOTOR even though the simulated bus delivers all
//     twelve in one datagram, so `state.motor_age[]` / `motor_fresh[]` /
//     `stale_motor_count` mean the same thing on both backends and the
//     diagnostic dump in main.cpp reads identically.
//   - A motor is only `seen` once a finite frame has arrived for it, so
//     the readiness check's "not all 12 motors have produced a valid frame
//     yet" gate
//     behaves the same way before the simulator is started.
//
// Consequence, and it is intended: if the simulator is stopped, runs slower
// than real time, or the socket dies, the controller sees exactly the same
// MOTOR_TIMEOUT fault it would see if the motor bus went quiet. Nothing about
// safety is bypassed for simulation.
// ===========================================================================
#pragma once

#include "common/Types.h"
#include "hardware/LowLevelUdpLink.h"
#include "hardware/LowLevelBus.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <mutex>
#include <string>
#include <thread>

namespace deploy
{

class SimLowLevelClient
{
public:
    struct Config
    {
        /// Where the simulator's bridge listens for LowCmd.
        std::string bridge_ip = "127.0.0.1";
        int lowcmd_port = 21001;
        /// Where this process listens for LowState.
        int lowstate_port = 21002;

        /// Same thresholds the real backend uses; main() takes both from
        /// config/safety.yaml so there is one definition of "stale".
        double motor_timeout_s = 0.1;
        double imu_timeout_s   = 0.2;
    };

    SimLowLevelClient() = default;
    ~SimLowLevelClient() { stop(); }

    SimLowLevelClient(const SimLowLevelClient&) = delete;
    SimLowLevelClient& operator=(const SimLowLevelClient&) = delete;

    bool start(const Config& cfg)
    {
        stop();
        cfg_ = cfg;

        motor_seen_.fill(false);
        motor_stamp_per_.fill(TimePoint{});
        imu_valid_ = false;
        imu_stamp_ = TimePoint{};
        last_seq_  = 0;
        have_state_ = false;
        tx_seq_ = 0;

        if (!link_.open(cfg_.bridge_ip, cfg_.lowcmd_port, cfg_.lowstate_port))
        {
            error_ = link_.lastError();
            return false;
        }

        running_ = true;
        rx_thread_ = std::thread(&SimLowLevelClient::rxLoop_, this);
        return true;
    }

    void stop()
    {
        running_ = false;
        if (rx_thread_.joinable()) rx_thread_.join();
        link_.close();
    }

    bool running() const { return running_; }
    const std::string& lastError() const { return error_; }

    /// Send one command, already in MOTOR order. `mode` is one of
    /// lowlevel::kModeServo / kModeProtect / kModeStop.
    bool send(const MotorCommand& cmd, std::uint8_t mode)
    {
        lowlevel::LowCmd pkt{};
        lowlevel::fill_header(pkt.h, lowlevel::kTypeLowCmd, ++tx_seq_, wallSeconds_());
        for (std::size_t i = 0; i < NUM_DOF; ++i)
        {
            pkt.q[i]   = static_cast<float>(cmd.q[i]);
            pkt.dq[i]  = static_cast<float>(cmd.dq[i]);
            pkt.tau[i] = static_cast<float>(cmd.torque[i]);
            pkt.kp[i]  = static_cast<float>(cmd.kp[i]);
            pkt.kd[i]  = static_cast<float>(cmd.kd[i]);
        }
        pkt.mode = mode;
        return link_.send(pkt);
    }

    /// Send a bare mode packet with zero payload (protect / stop).
    bool sendMode(std::uint8_t mode)
    {
        MotorCommand zero;
        // protect() on the real robot is kp = 0, kd = 1, zero targets. Encode
        // the same numbers here so a bridge that ignores `mode` and just runs
        // the PD law still produces the identical damping hold.
        if (mode == lowlevel::kModeProtect)
        {
            zero.kp.fill(0.0);
            zero.kd.fill(1.0);
        }
        return send(zero, mode);
    }

    /// Fill the MOTOR-order and IMU fields of `state`. Robot-order fields are
    /// left alone -- CtrlFSM runs JointMapper over them, same as the real path.
    bool fillState(RobotState& state)
    {
        const TimePoint now = Clock::now();

        lowlevel::LowState s;
        bool have;
        std::array<TimePoint, NUM_DOF> stamps;
        std::array<bool, NUM_DOF> seen;
        TimePoint imu_stamp, vel_stamp;
        bool imu_valid;
        {
            std::lock_guard<std::mutex> lk(mutex_);
            have      = have_state_;
            s         = latest_;
            stamps    = motor_stamp_per_;
            seen      = motor_seen_;
            imu_stamp = imu_stamp_;
            imu_valid = imu_valid_;
            vel_stamp = oracle_stamp_;
        }

        bool all_seen = true;
        bool all_finite = true;
        std::size_t stale = 0;
        TimePoint oldest = now;

        for (std::size_t i = 0; i < NUM_DOF; ++i)
        {
            if (have)
            {
                state.motor_q[i]     = s.q[i];
                state.motor_dq[i]    = s.dq[i];
                state.motor_tau[i]   = s.tau[i];
                state.motor_fault[i] = static_cast<int>(s.motor_fault[i]);
            }
            const bool finite = std::isfinite(state.motor_q[i]) &&
                                std::isfinite(state.motor_dq[i]) &&
                                std::isfinite(state.motor_tau[i]);
            all_finite = all_finite && finite;

            if (!seen[i])
            {
                all_seen = false;
                state.motor_age[i]   = std::numeric_limits<double>::infinity();
                state.motor_fresh[i] = false;
                ++stale;
                continue;
            }
            const double age = std::chrono::duration<double>(now - stamps[i]).count();
            state.motor_age[i]   = age;
            state.motor_fresh[i] = (age <= cfg_.motor_timeout_s);
            if (!state.motor_fresh[i]) ++stale;
            if (stamps[i] < oldest) oldest = stamps[i];
        }

        state.stale_motor_count = stale;
        state.motor_valid = all_seen && all_finite;
        state.motor_stamp = all_seen ? oldest : TimePoint{};

        if (have)
        {
            state.orientation    = {s.quat[0], s.quat[1], s.quat[2], s.quat[3]};
            state.angular_velocity = {s.gyro[0], s.gyro[1], s.gyro[2]};
            // The simulated IMU site IS the base frame: raw == base.
            state.imu_raw_orientation      = state.orientation;
            state.imu_raw_angular_velocity = state.angular_velocity;
        }
        state.imu_valid = imu_valid;
        state.imu_stamp = imu_stamp;
        state.imu_age   = imu_valid
                        ? std::chrono::duration<double>(now - imu_stamp).count()
                        : std::numeric_limits<double>::infinity();
        if (state.imu_valid) quat_xyzw_to_rpy(state.orientation, state.rpy);

        // Simulator ground truth (an oracle -- see LowLevelBus.h). Stamped
        // with the packet's arrival, so a stopped simulator makes it stale.
        state.base_lin_vel.valid = vel_stamp != TimePoint{};
        state.base_lin_vel.stamp = vel_stamp;
        if (state.base_lin_vel.valid)
            state.base_lin_vel.value = {s.sim_base_lin_vel[0], s.sim_base_lin_vel[1],
                                        s.sim_base_lin_vel[2]};

        stale_motor_count_ = stale;
        motor_valid_ = state.motor_valid;
        imu_age_cached_ = state.imu_age;
        return state.motor_valid;
    }

    std::size_t staleMotorCount() const { return stale_motor_count_; }
    bool motorValid() const { return motor_valid_; }
    double imuAge() const { return imu_age_cached_; }
    bool imuValid() const { return imu_valid_; }
    double imuTimeout() const { return cfg_.imu_timeout_s; }

    /// MuJoCo `d->time` from the last packet. Diagnostics only.
    double simTime() const
    {
        std::lock_guard<std::mutex> lk(mutex_);
        return have_state_ ? latest_.sim_time : 0.0;
    }

    std::size_t droppedPackets() const { return dropped_; }
    std::size_t malformedPackets() const { return malformed_; }
    std::uint64_t statePackets() const { return rx_count_; }

private:
    static double wallSeconds_()
    {
        return std::chrono::duration<double>(
                   std::chrono::system_clock::now().time_since_epoch()).count();
    }

    void rxLoop_()
    {
        lowlevel::LowState pkt;
        while (running_)
        {
            std::size_t drop = 0, bad = 0;
            if (link_.receiveLatest(pkt, lowlevel::kTypeLowState, &drop, &bad))
            {
                const TimePoint now = Clock::now();
                std::lock_guard<std::mutex> lk(mutex_);
                // `seq` as a pure change detector, exactly like the real
                // backend uses motor_back_t::time. A retransmitted or
                // duplicated packet must not refresh the age.
                if (!have_state_ || pkt.h.seq != last_seq_)
                {
                    last_seq_ = pkt.h.seq;
                    latest_ = pkt;
                    have_state_ = true;
                    ++rx_count_;
                    for (std::size_t i = 0; i < NUM_DOF; ++i)
                    {
                        if (std::isfinite(pkt.q[i]) && std::isfinite(pkt.dq[i]) &&
                            std::isfinite(pkt.tau[i]))
                        {
                            motor_stamp_per_[i] = now;
                            motor_seen_[i] = true;
                        }
                    }
                    const double n2 = double(pkt.quat[0]) * pkt.quat[0] +
                                      double(pkt.quat[1]) * pkt.quat[1] +
                                      double(pkt.quat[2]) * pkt.quat[2] +
                                      double(pkt.quat[3]) * pkt.quat[3];
                    if (std::isfinite(n2) && n2 > 0.5 && n2 < 1.5)
                    {
                        imu_stamp_ = now;
                        imu_valid_ = true;
                    }
                    if (pkt.sim_base_lin_vel_valid && std::isfinite(pkt.sim_base_lin_vel[0]) &&
                        std::isfinite(pkt.sim_base_lin_vel[1]) &&
                        std::isfinite(pkt.sim_base_lin_vel[2]))
                        oracle_stamp_ = now;
                }
            }
            dropped_   += drop;
            malformed_ += bad;

            // The bus runs at the bridge rate (1 kHz by default). Polling at
            // ~10 kHz keeps added latency well under one control cycle without
            // spinning a core. `receiveLatest` drains, so a slow poll costs
            // throughput, never correctness.
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    }

    Config cfg_;
    lowlevel::LowLevelUdpLink link_;
    std::thread rx_thread_;
    std::atomic<bool> running_{false};
    std::string error_;

    mutable std::mutex mutex_;
    lowlevel::LowState latest_{};
    bool have_state_ = false;
    std::uint64_t last_seq_ = 0;
    std::array<TimePoint, NUM_DOF> motor_stamp_per_{};
    std::array<bool, NUM_DOF> motor_seen_{};
    TimePoint imu_stamp_{};
    bool imu_valid_ = false;
    TimePoint oracle_stamp_{};

    std::uint64_t tx_seq_ = 0;
    std::atomic<std::size_t> dropped_{0};
    std::atomic<std::size_t> malformed_{0};
    std::atomic<std::uint64_t> rx_count_{0};

    std::atomic<std::size_t> stale_motor_count_{NUM_DOF};
    std::atomic<bool> motor_valid_{false};
    std::atomic<double> imu_age_cached_{std::numeric_limits<double>::infinity()};
};

} // namespace deploy
