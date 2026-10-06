#include "FSM/CtrlFSM.h"

#include <ros/ros.h>

#include <algorithm>

namespace deploy
{

CtrlFSM::CtrlFSM(const YAML::Node& cfg, ControlContext* ctx, double control_dt)
    : ctx_(ctx), dt_(control_dt)
{
    const YAML::Node enabled = cfg["_"];
    if (!enabled)
    {
        throw std::runtime_error("CtrlFSM: fsm.yaml has no FSM._ block");
    }

    // First pass: register every id<->name pair, so that transitions declared
    // by a state constructed early can still resolve a state constructed later.
    for (auto it = enabled.begin(); it != enabled.end(); ++it)
    {
        FSMStateRegistry::instance().insert(it->second["id"].as<int>(),
                                            it->first.as<std::string>());
    }

    for (auto it = enabled.begin(); it != enabled.end(); ++it)
    {
        const std::string name = it->first.as<std::string>();
        const int id = it->second["id"].as<int>();
        const std::string type = it->second["type"] ? it->second["type"].as<std::string>() : name;

        auto factory = getFsmMap().find("State_" + type);
        if (factory == getFsmMap().end())
        {
            throw std::runtime_error("CtrlFSM: unknown FSM type State_" + type);
        }
        if (type == "RLBase") rl_state_ids_.insert(id);
        add(factory->second(id, name, ctx_));
    }
}

CtrlFSM::~CtrlFSM()
{
    stop();
    states.clear();
}

void CtrlFSM::add(const std::shared_ptr<BaseState>& state)
{
    for (const auto& s : states)
    {
        if (s->isState(state->getState()))
        {
            throw std::runtime_error("CtrlFSM: duplicate FSM id for State_" +
                                     state->getStateString());
        }
    }
    states.push_back(state);
}

void CtrlFSM::start(const std::string& initial_state)
{
    if (states.empty()) throw std::runtime_error("CtrlFSM: no states registered");
    if (running_) return;

    // Entering Passive means a damping-only hold recomputed from the measured
    // pose (State_Passive::run -> holdDamping, kp = 0, kd = 1), which is the
    // same shape Unitree's State_Passive::enter/run produces (kp = 0, kd from
    // config, q = measured). It is the safe operating mode, and it is the only
    // state the controller ever starts in.
    for (const auto& st : states)
    {
        if (st->getStateString() == initial_state) { currentState_ = st; break; }
    }
    if (!currentState_)
    {
        throw std::runtime_error("CtrlFSM: initial state '" + initial_state +
                                 "' is not among the enabled states");
    }

    refreshState_();
    currentState_->enter();
    ROS_INFO("FSM: Start %s", currentState_->getStateString().c_str());

    running_ = true;
    thread_ = std::thread(&CtrlFSM::run_, this);
}

void CtrlFSM::stop()
{
    running_ = false;
    if (thread_.joinable()) thread_.join();
}

void CtrlFSM::run_()
{
    const auto period = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(dt_));
    auto next = Clock::now() + period;

    while (running_ && ros::ok())
    {
        const auto t0 = Clock::now();
        tick_();
        last_cycle_s_ = std::chrono::duration<double>(Clock::now() - t0).count();

        const auto now = Clock::now();
        if (now > next)
        {
            ++overruns_;
            next = now + period;   // resynchronize instead of spiraling
        }
        else
        {
            std::this_thread::sleep_until(next);
            next += period;
        }
    }
}

void CtrlFSM::refreshState_()
{
    ctx_->readState();

    // Detection, then Safety::raise() latches it. Safety ignores everything
    // until main() arms it, which is what keeps "no motor packet yet" during
    // startup from being recorded as a runtime fault.
    ctx_->safety->raise(ctx_->safety->checkState(*ctx_->state, ctx_->hardware->status()));
}

void CtrlFSM::serviceFaultReset_()
{
    // Deliberately the ONLY automatic path out of a latched fault: none. The
    // operator must press reset (RB + Y), exactly as the removed
    // `Fault -> Passive: reset` transition required. A fault that merely stops
    // being true does not restore commanding.
    // The edge is consumed unconditionally, so a reset pressed while healthy
    // cannot sit latched and silently clear the NEXT fault the moment it trips.
    const bool pressed = ctx_->input->requestReset();
    if (pressed && ctx_->safety->faulted()) ctx_->safety->clear();
}

void CtrlFSM::tick_()
{
    refreshState_();

    ctx_->command->reset();
    currentState_->pre_run();   // refreshes InputManager's edges for this cycle
    serviceFaultReset_();
    currentState_->run();
    currentState_->post_run();

    {
        std::lock_guard<std::mutex> lk(snap_mutex_);
        snap_state_ = *ctx_->state;
        snap_command_ = *ctx_->command;
        snap_valid_ = true;
    }

    int next_id = 0;
    for (const auto& check : currentState_->registered_checks)
    {
        if (check.first()) { next_id = check.second; break; }
    }

    if (next_id != 0 && !currentState_->isState(next_id))
    {
        // Guard: never hand control to an RL state whose policy is not ready.
        // State_RLBase would refuse and bounce straight back, so rejecting the
        // transition here keeps the FSM deterministic and avoids a one-cycle
        // excursion into a state that cannot command anything.
        if (rl_state_ids_.count(next_id))
        {
            std::string refusal;
            if (!ctx_->policy->ready())        refusal = ctx_->policy->unavailableReason();
            else if (ctx_->safety->faulted())  refusal = "a safety fault is latched";
            else if (!currentState_->readyForRL())
                                               refusal = currentState_->notReadyForRLReason();
            else                               refusal = ctx_->policy->entryBlocker(*ctx_->state);

            if (!refusal.empty())
            {
                ROS_ERROR_THROTTLE(2.0,
                    "FSM: refusing transition %s -> %s: %s. Staying in %s.",
                    currentState_->getStateString().c_str(),
                    FSMStateRegistry::instance().name(next_id).c_str(),
                    refusal.c_str(),
                    currentState_->getStateString().c_str());
                return;
            }
        }

        for (const auto& state : states)
        {
            if (state->isState(next_id))
            {
                ROS_INFO("FSM: Change state from %s to %s",
                         currentState_->getStateString().c_str(),
                         state->getStateString().c_str());
                currentState_->exit();
                currentState_ = state;
                currentState_->enter();
                break;
            }
        }
    }
}

std::string CtrlFSM::currentStateName() const
{
    return currentState_ ? currentState_->getStateString() : std::string("<none>");
}

} // namespace deploy
