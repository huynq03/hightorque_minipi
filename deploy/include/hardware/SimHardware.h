// SimHardware -- HardwareInterface backed by a robot's MuJoCo bridge
// (simulate/robots/<robot>), over the shared UDP low-level bus.
//
// The simulation counterpart of a robot's real backend (e.g. Mini-Pi's
// HighTorqueHardware). Shared by every robot. It sits in exactly the
// same slot in the stack and is the same size of thing:
//
//   real :  FSM -> JointMapper -> HighTorqueHardware -> livelybot_serial -> motors
//   sim  :  FSM -> JointMapper -> SimHardware        -> UDP low-level bus -> <robot>Bridge -> MuJoCo
//
// ========================== WHAT IS NOT IN HERE ============================
// No MuJoCo. No physics. No PD law. No FSM logic, no safety policy, no policy
// inference, no joystick handling. This class only moves MOTOR-SPACE bytes, in
// the same shape livelybot_serial moves them. Everything else the simulated
// robot needs already exists above it and is shared verbatim with the real
// robot.
//
// In particular it does NOT touch map_index, direction or joint_offset. What
// arrives in write() has already been through JointMapper and is in motor
// space; what read() produces is raw motor space and CtrlFSM maps it. Exactly
// one transform, in exactly one place, for both backends.
// ===========================================================================
#pragma once

#include "common/Types.h"
#include "hardware/HardwareInterface.h"
#include "hardware/SimLowLevelClient.h"

#include <string>
#include <vector>

namespace deploy
{

class SimHardware : public HardwareInterface
{
public:
    struct Config
    {
        /// Same meaning as on the real backend: when true nothing is put on
        /// the bus. Kept rather than special-cased away so that the two
        /// backends cannot diverge in how the flag reads in the logs; note
        /// that a dry_run simulation therefore does nothing, which is correct
        /// and is why the launch file defaults it to false for backend:=sim.
        bool dry_run = true;

        SimLowLevelClient::Config link;

        /// How long to wait at startup for the first LowState packet before
        /// giving up. The simulator may legitimately be started after the
        /// controller; 0 disables the wait entirely and lets the startup readiness
        /// readiness gate do the waiting instead.
        double wait_for_bridge_s = 5.0;
    };

    SimHardware() = default;
    ~SimHardware() override;

    SimHardware(const SimHardware&) = delete;
    SimHardware& operator=(const SimHardware&) = delete;

    /// Open the bus and start the receive thread. Unlike the real backend this
    /// cannot fail because of missing hardware -- it fails only if the socket
    /// cannot be created or bound.
    bool initialize(const Config& cfg);

    bool read(RobotState& state) override;
    bool write(const MotorCommand& cmd) override;
    HardwareStatus status() const override;
    void protect() override;
    void stop() override;

    bool initialized() const override { return initialized_; }
    bool dryRun() const override { return cfg_.dry_run; }
    std::size_t motorCount() const override { return motor_names_.size(); }
    const std::vector<std::string>& motorNames() const override { return motor_names_; }
    const std::vector<int>& motorIds() const override { return motor_ids_; }
    const char* backendName() const override { return "sim"; }
    bool isSimulation() const override { return true; }

    /// MuJoCo time from the last packet, for aligning a sim log against a real
    /// log in the real-vs-sim comparison (see doc/simulation.md).
    double simTime() const { return link_.simTime(); }

private:
    Config cfg_;
    SimLowLevelClient link_;
    /// Synthetic, so that the startup readiness dump and main.cpp's diagnostics
    /// print something meaningful on both backends. These are LABELS ONLY and
    /// no control path reads them -- the same status get_motor_name() has on
    /// the real robot (see doc/architecture.md).
    std::vector<std::string> motor_names_;
    std::vector<int> motor_ids_;
    bool initialized_ = false;
};

} // namespace deploy
