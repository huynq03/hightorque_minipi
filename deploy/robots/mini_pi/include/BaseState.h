// Adapted from unitree_rl_mjlab/deploy/include/FSM/BaseState.h.
// Same lifecycle (enter / pre_run / run / post_run / exit), same
// id<->name bimap idea and same self-registering factory macro.
// Differences: no boost::bimap dependency, and the registry is typed on the
// Mini-Pi ControlContext instead of Unitree's static lowcmd/lowstate.
#pragma once

#include "ControlContext.h"

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace mini_pi
{

/// Bidirectional id <-> name registry for the enabled states.
class FSMStateRegistry
{
public:
    static FSMStateRegistry& instance()
    {
        static FSMStateRegistry r;
        return r;
    }

    void insert(int id, const std::string& name)
    {
        by_id_[id] = name;
        by_name_[name] = id;
    }
    bool hasName(const std::string& name) const { return by_name_.count(name) > 0; }
    bool hasId(int id) const { return by_id_.count(id) > 0; }
    int  id(const std::string& name) const { return by_name_.at(name); }
    const std::string& name(int id) const { return by_id_.at(id); }
    const std::map<int, std::string>& all() const { return by_id_; }

private:
    std::map<int, std::string> by_id_;
    std::map<std::string, int> by_name_;
};

class BaseState
{
public:
    BaseState(int state, std::string state_string, ControlContext* ctx)
        : ctx_(ctx), state_(state)
    {
        FSMStateRegistry::instance().insert(state, state_string);
    }
    virtual ~BaseState() = default;

    virtual void enter() {}
    virtual void pre_run() {}
    virtual void run() {}
    virtual void post_run() {}
    virtual void exit() {}

    const std::string& getStateString() const { return FSMStateRegistry::instance().name(state_); }
    int  getState() const { return state_; }
    bool isState(int state) const { return state_ == state; }

    /// Transition guards: first predicate that returns true wins.
    std::vector<std::pair<std::function<bool()>, int>> registered_checks;

    /// Per-state veto on entering an RL state, checked by CtrlFSM after a
    /// guard has selected one. The default is "yes"; State_FixStand overrides
    /// it so that an `rl` request arriving mid-interpolation does not hand a
    /// half-raised robot to the policy. The request is edge-consumed by
    /// InputManager either way, so a refused request must be made again.
    virtual bool readyForRL() const { return true; }
    /// Operator-facing explanation used when readyForRL() is false.
    virtual std::string notReadyForRLReason() const { return {}; }

protected:
    ControlContext* ctx_ = nullptr;

private:
    int state_;
};

using FsmFactory = std::function<std::shared_ptr<BaseState>(int, std::string, ControlContext*)>;
using FsmMap     = std::unordered_map<std::string, FsmFactory>;

inline FsmMap& getFsmMap()
{
    static FsmMap m;
    return m;
}

#define MINI_PI_REGISTER_FSM(Derived)                                                   \
    inline std::shared_ptr<::mini_pi::BaseState> __factory_##Derived(                   \
        int s, std::string ss, ::mini_pi::ControlContext* ctx)                          \
    {                                                                                   \
        return std::make_shared<Derived>(s, ss, ctx);                                   \
    }                                                                                   \
    inline struct __registrar_##Derived                                                 \
    {                                                                                   \
        __registrar_##Derived()                                                         \
        {                                                                               \
            ::mini_pi::getFsmMap()[#Derived] = __factory_##Derived;                     \
        }                                                                               \
    } __registrar_instance_##Derived;

} // namespace mini_pi
