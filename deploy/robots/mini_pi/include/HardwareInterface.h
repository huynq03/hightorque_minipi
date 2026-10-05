// HardwareInterface -- the seam that lets one FSM drive either the real
// Mini-Pi or a MuJoCo simulation.
//
// This is the Mini-Pi counterpart of the boundary Unitree draws at the SDK2
// low-level channels: in unitree_rl_mjlab, deploy always talks to `rt/lowcmd`
// and `rt/lowstate`, and whether a real robot or `unitree_mujoco` sits on the
// other side is invisible to the controller. Mini-Pi has no such channel pair,
// so the seam is expressed as a C++ interface instead.
//
// ============================== SCOPE ======================================
// This interface is deliberately the EXACT set of members the FSM already
// called on HighTorqueHardware -- nothing was added, generalized or renamed to
// make the abstraction look nicer. It was derived by grepping every
// `ctx_->hardware->...` use in deploy/, so lifting it is a no-op for the real
// path.
//
// `initialize()` is NOT part of the interface. Each backend needs different
// arguments (the real one needs a ros::NodeHandle and the livelybot params,
// the simulated one needs socket endpoints), and only main() ever calls it.
// Putting it here would force a lowest-common-denominator signature for no
// benefit. main() constructs the concrete type, then stores the base pointer.
//
// EVERY method here works in MOTOR SPACE or is index-free. The robot<->motor
// coordinate transform stays where it already is: JointMapper, in deploy,
// applied exactly once. A backend must never re-apply map_index, direction or
// joint_offset. See doc/architecture.md and doc/simulation.md.
// ===========================================================================
#pragma once

#include "Types.h"

#include <cstddef>
#include <string>
#include <vector>

namespace mini_pi
{

class HardwareInterface
{
public:
    virtual ~HardwareInterface() = default;

    /// Copy the latest motor feedback (MOTOR ORDER) and IMU sample into
    /// `state`. The robot-order fields are NOT filled here; CtrlFSM runs
    /// JointMapper over the result. Returns state.motor_valid.
    virtual bool read(RobotState& state) = 0;

    /// Hand one command, already in MOTOR ORDER, to the backend.
    /// Must be a no-op returning true when dryRun() is set.
    virtual bool write(const MotorCommand& cmd) = 0;

    /// Transport health, as judged against the freshness thresholds from
    /// config/safety.yaml.
    virtual HardwareStatus status() const = 0;

    /// Low-stiffness damping hold (kp = 0, kd = 1, zero targets). The reflex
    /// the HighTorque SDK itself uses; a backend must reproduce it, not
    /// substitute a torque cut.
    virtual void protect() = 0;

    /// Hard stop: motor output cut. A standing robot falls. Honours dryRun().
    virtual void stop() = 0;

    virtual bool initialized() const = 0;
    virtual bool dryRun() const = 0;
    virtual std::size_t motorCount() const = 0;
    virtual const std::vector<std::string>& motorNames() const = 0;
    virtual const std::vector<int>& motorIds() const = 0;

    /// Short token for logs and the diagnostic dump ("hightorque" / "sim").
    /// The one member that is not on the original HighTorqueHardware: with two
    /// backends, every log line needs to say which one produced it.
    virtual const char* backendName() const = 0;

    /// True only for a backend that drives a simulator rather than motors.
    ///
    /// RLPolicyRunner uses this to refuse a `simulation_only` policy package on
    /// real hardware. It is NOT pure virtual, and it defaults to FALSE, so a
    /// backend added later is treated as real until it deliberately says
    /// otherwise -- the safe direction to be wrong in.
    virtual bool isSimulation() const { return false; }
};

} // namespace mini_pi
