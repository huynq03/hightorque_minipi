// SystemReadiness -- the startup gate that runs BEFORE the FSM exists.
//
// This is the Mini-Pi counterpart of what Unitree does in
// unitree_rl_mjlab/deploy/robots/g1/main.cpp:11-26 (`init_fsm_state()`):
// build the low-level channels, then BLOCK in
// LowState_t::wait_for_connection() (unitree_sdk2 Subscription.h:53-67) until
// the first state packet arrives, and only then construct and start CtrlFSM.
// Unitree has no `Init` FSM state for the same reason this header exists:
// waiting for the first packet is not a robot operating mode.
//
// The checks below are exactly the ones State_Init used to run; nothing was
// added, removed or loosened. They simply run here, on the main thread, before
// CtrlFSM::start().
//
// ===================== "NOT READY" IS NOT "FAULTED" ========================
// The distinction this header exists to draw:
//
//   startup       no motor packet yet  ->  WAIT (and say why, once every 2 s)
//   after startup feedback lost        ->  runtime fault, see SafetyGate
//
// Previously both were the same thing: SafetyManager::checkState() reported
// MOTOR_TIMEOUT on the very first cycle, and the global "-> Fault" guard moved
// State_Init into State_Fault before it had ever seen a motor. SafetyGate is
// armed only after waitUntilReady() has returned true, so that cannot recur.
// ===========================================================================
#pragma once

#include "control/ControlContext.h"

#include <yaml-cpp/yaml.h>

#include <string>

namespace mini_pi
{

struct ReadinessConfig
{
    /// A motor or IMU sample older than this does not count as readiness.
    /// Deliberately looser than safety.yaml's runtime thresholds: at startup
    /// the transport is still settling.
    double require_fresh_s = 0.5;
    /// Whether a valid IMU sample is required before the FSM may start.
    bool require_imu = true;
    /// Seconds to keep waiting before giving up. <= 0 means wait forever,
    /// which is Unitree's behaviour (wait_for_connection() never times out).
    double timeout_s = 0.0;
    /// How often the readiness condition is re-evaluated while waiting.
    double poll_dt_s = 0.02;
};

/// Reads the `startup:` block of robot.yaml. An undefined node, or any
/// missing key, keeps the default above.
ReadinessConfig loadReadinessConfig(const YAML::Node& startup);

/// THE readiness condition. Returns an empty string when the system is ready,
/// otherwise the first failing reason, phrased for an operator.
///
/// Pure: it only inspects `ctx`. The caller is responsible for having refreshed
/// `*ctx.state` from the hardware first (refreshReadinessState() does that).
std::string readinessReason(const ControlContext& ctx, const ReadinessConfig& cfg);

/// One hardware read + robot-order mapping, writing `*ctx.state`. Same two
/// steps CtrlFSM::refreshState_() performs once the FSM is running; used here
/// so that the startup poll judges the same snapshot the FSM will.
void refreshReadinessState(ControlContext& ctx);

/// Blocks until readinessReason() is empty.
///
/// Returns true when the system became ready, false on timeout or on ROS
/// shutdown (Ctrl+C while waiting). NOTHING is transmitted to the motors while
/// this runs -- exactly as in Unitree, where no LowCmd is published until the
/// FSM starts. State_Init's default command was kp = 0, kd = 0, tau = 0, i.e.
/// already physically a no-op, so this is not a behaviour change.
///
/// On success it logs the readiness dump State_Init used to print.
bool waitUntilReady(ControlContext& ctx, const ReadinessConfig& cfg);

} // namespace mini_pi
