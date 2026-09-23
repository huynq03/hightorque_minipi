// rl::ObservationManager -- config-driven observation assembly.
//
// Structural counterpart of unitree_rl_mjlab/deploy/include/isaaclab/manager/
// observation_manager.h + manager_term_cfg.h. Same registry idea, same
// REGISTER_OBSERVATION macro, same frame-major ("gym") history layout.
//
// ========================= WHAT THIS CLASS GUARANTEES ========================
// Nothing here knows 47, 15 or 705. Those come from the policy package. What
// is fixed is the ORDER OF OPERATIONS, because getting it wrong silently
// produces a plausible but wrong observation:
//
//   per term, per tick:     raw = func(env, params)
//                           raw *= scale          (elementwise or scalar)
//   per FRAME, after every term has contributed:
//                           frame = clip(frame, lo, hi)
//                           push frame onto the history
//   per inference:
//                           flatten  [frame t-(N-1)] ... [frame t]
//
// Scale-then-clip, and clip applied to the ASSEMBLED FRAME rather than per
// term, is the HighTorque humanoid-gym convention: `clip_obs: 18.0` in
// hightorque_rl_control_box-sw-pizk-r-pkg-2.0.0/src/sim2real/config/walk/
// humanoidgym.yaml is a single bound on the whole observation vector.
// Unitree's default is the opposite (clip, then scale, per term) and is
// available per term via `clip:` for compatibility.
//
// The history is frame-major and each frame is contiguous:
//     [47 oldest][47][47]...[47 newest]
// NOT term-major. See docs/rl_architecture.md.
// ============================================================================
//
// TERM ORDER IS A YAML SEQUENCE, not a map. yaml-cpp happens to preserve map
// insertion order, but the YAML spec does not require it, and here the order
// IS the wire format. A sequence makes it explicit and checkable.
#pragma once

#include <yaml-cpp/yaml.h>

#include <cstddef>
#include <deque>
#include <functional>
#include <map>
#include <string>
#include <vector>

namespace mini_pi
{
namespace rl
{

class PolicyEnvironment;

/// One observation term: a pure function of the environment.
using ObsFunc = std::function<std::vector<float>(PolicyEnvironment*, const YAML::Node&)>;

using ObsMap = std::map<std::string, ObsFunc>;

inline ObsMap& observations_map()
{
    static ObsMap instance;
    return instance;
}

/// Same shape as Unitree's REGISTER_OBSERVATION.
#define MINI_PI_REGISTER_OBSERVATION(name)                                               \
    inline std::vector<float> name(::mini_pi::rl::PolicyEnvironment* env,                \
                                   const YAML::Node& params);                            \
    inline struct name##_registrar                                                       \
    {                                                                                    \
        name##_registrar() { ::mini_pi::rl::observations_map()[#name] = name; }          \
    } name##_registrar_instance;                                                         \
    inline std::vector<float> name(::mini_pi::rl::PolicyEnvironment* env,                \
                                   const YAML::Node& params)

struct ObservationTermCfg
{
    std::string name;
    YAML::Node params;
    ObsFunc func;

    /// Elementwise. Empty = no scaling. A scalar in YAML is broadcast at load.
    std::vector<float> scale;
    /// Per-term clip, applied BEFORE scale (Unitree order). Empty = none.
    /// The group-level clip below is the normal mechanism; this exists so a
    /// future policy trained the Unitree way can be reproduced exactly.
    std::vector<float> clip;

    /// Width, captured at load by calling func() once. Used to assert
    /// frame_dim and to give a precise error when a scale vector is wrong.
    std::size_t dim = 0;
};

/// One observation group. The group name must match an ONNX input name.
struct ObservationGroupCfg
{
    std::string name;
    std::vector<ObservationTermCfg> terms;

    std::size_t frame_dim = 0;     ///< sum of term dims
    std::size_t history_length = 1;
    bool clip_enabled = false;
    float clip_lo = 0.0f, clip_hi = 0.0f;

    std::size_t totalDim() const { return frame_dim * history_length; }
};

class ObservationManager
{
public:
    /// `cfg` is the deploy.yaml `observations:` node.
    /// Throws std::runtime_error on an unregistered term, a scale/clip of the
    /// wrong width, or a frame_dim assertion failure.
    ObservationManager(const YAML::Node& cfg, PolicyEnvironment* env);

    /// Refills the history with `history_length` copies of... nothing:
    /// every frame is set to ZERO, per task §11 ("At reset: 15 zero frames").
    /// NOTE this differs from Unitree, which resets by repeating the CURRENT
    /// observation `history_length` times.
    void reset();

    /// Advances every term by one tick and returns one flattened vector per
    /// group, keyed by group name.
    std::map<std::string, std::vector<float>> compute();

    const std::vector<ObservationGroupCfg>& groups() const { return groups_; }
    /// Last flattened value of `group`, without advancing anything.
    const std::vector<float>& flat(const std::string& group) const;
    /// The most recently appended single frame of `group`, unflattened.
    const std::vector<float>& lastFrame(const std::string& group) const;

    std::string describe() const;

private:
    struct GroupState
    {
        std::deque<std::vector<float>> frames;  ///< oldest at front
        std::vector<float> flat;
        std::vector<float> last_frame;
    };

    ObservationGroupCfg parseGroup_(const std::string& name, const YAML::Node& node);
    std::vector<float> buildFrame_(const ObservationGroupCfg& g);
    void push_(std::size_t gi, std::vector<float> frame);

    PolicyEnvironment* env_ = nullptr;
    std::vector<ObservationGroupCfg> groups_;
    std::vector<GroupState> state_;
    std::map<std::string, std::size_t> index_;
};

} // namespace rl
} // namespace mini_pi
