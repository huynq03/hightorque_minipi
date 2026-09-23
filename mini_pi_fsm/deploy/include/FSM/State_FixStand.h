// Adapted from unitree_rl_mjlab/deploy/include/FSM/State_FixStand.h.
//
// Same architecture: a start pose captured at enter(), a target pose, and
// linear interpolation between them over a configurable duration.
//
// Differences from Unitree, both deliberate:
//  - The interpolation runs entirely in ROBOT joint space. This state never
//    sees a motor index, a sign or an offset; JointMapper applies those on the
//    way out.
//  - The stand target defaults to robot_q == 0, because in the verified
//    Mini-Pi convention robot zero IS the nominal pose (the motor<->robot
//    offset lives in mapping.yaml `joint_offset`). Unitree instead lists an
//    explicit qs[] pose because its robot space is raw joint angles.
//
// Entirely independent of RL.
#pragma once

#include "FSM/FSMState.h"

#include <vector>

namespace mini_pi
{

class State_FixStand : public FSMState
{
public:
    State_FixStand(int state, std::string state_string, ControlContext* ctx);

    void enter() override;
    void run() override;

    /// True once the interpolation has reached the final keyframe.
    bool finished() const;

    /// RL may only be entered from a COMPLETED stand. Without this, an `rl`
    /// press one cycle after `stand` handed a half-raised robot straight to
    /// the policy, whose first action assumes the nominal pose.
    bool readyForRL() const override { return finished(); }
    std::string notReadyForRLReason() const override;

private:
    JointArray kp_{};
    JointArray kd_{};

    /// Keyframe times [s]; ts_[0] is always 0.
    std::vector<double> ts_;
    /// Keyframe poses in ROBOT joint space. qs_[0] is overwritten at enter()
    /// with the measured pose, so there is never an instantaneous jump.
    std::vector<std::vector<double>> qs_;

    TimePoint t0_{};
    bool entered_ = false;
};

MINI_PI_REGISTER_FSM(State_FixStand)

} // namespace mini_pi
