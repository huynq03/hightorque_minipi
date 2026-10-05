#include "State_Passive.h"

#include <ros/ros.h>

namespace mini_pi
{

State_Passive::State_Passive(int state, std::string state_string, ControlContext* ctx)
    : FSMState(state, state_string, ctx)
{
    // Default matches the HighTorque SDK's own protect path exactly:
    // hightorque::HtdwMotor::protectMotor() issues
    //   setMotor(0, 0, 0, kp=0, ki=0, kd=1.0, ...) with POS_VEL_TQE_KP_KD2.
    // That is the only damping value verifiable from HighTorque source, so it
    // is the default rather than Unitree's kd=3.
    kd_.fill(1.0);
    if (cfg_ && cfg_["kd"])
    {
        const auto v = cfg_["kd"].as<std::vector<double>>();
        if (v.size() == MINI_PI_DOF)
        {
            for (std::size_t i = 0; i < MINI_PI_DOF; ++i) kd_[i] = v[i];
        }
        else
        {
            ROS_ERROR("State_Passive: FSM.Passive.kd has %zu entries, expected %zu; "
                      "keeping the SDK default of 1.0", v.size(), MINI_PI_DOF);
        }
    }
}

void State_Passive::enter()
{
    ROS_INFO("State_Passive: damping hold, kp = 0, kd[0] = %.3f. "
             "No trajectory, no policy, no reuse of previous commands.", kd_[0]);
}

void State_Passive::run()
{
    // Recomputed from the freshly measured pose every single cycle, so the
    // command is always restamped and can never go stale.
    holdDamping(*ctx_->command, kd_);
    stampValid(*ctx_->command);
}

} // namespace mini_pi
