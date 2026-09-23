# Real Mini-Pi policy validation — 2026-09-23

## Artifact and model

- **SOURCE-VERIFIED** source:
  `/home/huynq/Mini_Pi_RL_Baseline/logs/Pai_ppo/exported/policies/policy_onnx.onnx`
- **IMPLEMENTED** destination:
  `deploy/config/policy/velocity/v0/exported/policy.onnx`
- SHA256 at both paths:
  `f28f096bea5efd4eb82ab3674da69c44fc443f47badd5bceec083eded8cc2e62`
- `cmp` result: byte-identical.
- **MODEL-VERIFIED** ONNX checker: valid; IR 6; producer PyTorch 2.4.1;
  opset `ai.onnx` 11; input `input`, FLOAT `[1,705]`; output `output`, FLOAT
  `[1,12]`.

The source model was read and copied only. It was not moved or modified.

## Runtime contract

`deploy.yaml` is source-derived from the local training repository. It uses a
47-value frame, 15 zero-initialized frames in oldest→newest order, group-level
observation clip ±18, raw action clip ±18, zero default positions and
`q_target = 0.25 × action`. A complete frame is appended once per policy step.

The model-facing joint order is left leg then right leg. This is not inferred
from URDF declaration order: the official `sim2sim.py` explicitly swaps its
right-first MJCF joint blocks before inference and swaps torque blocks back.
The mapping to robot order is name-based; `JointMapper` remains responsible
only for robot order ↔ physical motor order.

Two upstream discrepancies are explicit package data:

| setting | training | official sim2sim | selected |
| --- | --- | --- | --- |
| gait period | 0.4 s | 0.5 s | 0.5 s |
| damping | `[.6,.4,.4,.6,.6,.4]×2` | `[1.8,.8,.8,1.8,1.8,.6]×2` | sim2sim |

Both choices follow the author's deployment reference for the initial MuJoCo
validation and can be changed in `deploy.yaml` without changing C++.

## Automated results

**BUILT / UNIT TESTED / REGRESSION TESTED** with `pixi run test`:

- JointMapper: 397 checks, 0 failures.
- Safety/config: 273 checks, 0 failures.
- RL layer including real ONNX: 182,749 checks, 0 failures.
- Bridge loopback: PASS, 0 failures.

The deterministic TEST fixture remains separate and its two-part
simulation-only gate still passes its refusal tests.

Standalone deterministic zero-state inference over 250 warmed steps:

- observation: 705 values, min/max `-1.162703 / 2.099049`;
- raw action: 12 values, min/max `-1.162703 / 2.099049`;
- q_target: 12 values, min/max `-0.290676 / 0.524762`;
- ONNX-only latency in the final post-A/B verification: average `0.0651 ms`,
  maximum `0.2714 ms`;
- all observations, actions and targets finite;
- the second observation frame's slots 29:41 equal the previous raw ONNX
  output, not the processed target.

## End-to-end MuJoCo results

**END-TO-END SIM TESTED** using `mujoco-band-headless` and production
`fsm-sim` (not `fsm-sim-rl`):

- startup readiness → `PASSIVE → FIX_STAND`: stable, safety healthy; settled
  FixStand RPY
  approximately `[-0.0003, 0.0035, 0.0020] rad` and robot joints near zero.
- `FIX_STAND → Velocity`: selected package `v0`; policy ran continuously at
  approximately `50.01 Hz` for more than 10,000 steps.
- Runtime ONNX-only latency was approximately `0.26 ms` average and `1.77 ms`
  maximum, comfortably below the 20 ms policy period.
- Runtime observations, raw actions and targets remained finite. Representative
  zero-command ranges were observation `[-1.218,1.919]`, raw action
  `[-1.150,2.020]`, q_target `[-0.288,0.505]`.
- Conservative commands were sent through the existing `/cmd_vel` path:
  `vx=+0.10 m/s`, `vy=+0.05 m/s`, `yaw=+0.10 rad/s`. All ran without policy,
  mapping, timeout, NaN or safety errors.
- `RL → FIX_STAND`: passed; policy thread stopped/joined, target reset, and
  FixStand gains/zero targets were restored.
- `RL → PASSIVE`: passed; policy thread stopped/joined and damping hold was
  restored.
- Bridge stopped during RL: `MOTOR_TIMEOUT` latched in `SafetyGate`, 12 stale
  motors, policy thread stopped, FSM driven to Passive with the command
  overridden by the damping hold. (Recorded before the FSM refactor, when this
  appeared as an `RL → Fault` transition; the detection and the response are
  unchanged, only their representation moved out of the FSM.) After bridge
  restart the latch was still held, `joy-reset` cleared it, and a subsequent
  FixStand transition passed with hardware `Ok` and safety `healthy`.

### Behavioral limitation

At zero command the run was numerically stable and safe under the repository's
elastic-band preset, but it did not hold a fixed heading: roll/pitch stayed
small while yaw accumulated (a representative sample reached about `-0.95
rad`). The headless elastic-band run therefore proves interface execution and
safety behavior, not unsupported balance or accurate velocity tracking.

A controlled joint-order A/B run changed only the policy package to the
right-first XML declaration order. It did not resolve the heading drift (yaw
reached about `-1.16 rad`) and produced substantially larger asymmetric joint
rates. That variant was rejected; production remains left-first based on the
explicit block swaps in the author's `sim2sim.py` and the training environment's
policy-index semantics.

The original `sim2sim.py` was source-compared, not executed: its default input
is a TorchScript file and its viewer requires Torch/GLFW dependencies absent
from this Pixi environment. No gains, scaling, offsets, physics parameters or
weights were tuned to hide the qualitative yaw mismatch.

## Reproduction

```bash
cd /home/huynq/hightorque/mini_pi_fsm
./.tools/pixi/bin/pixi run test

# Four terminals:
./.tools/pixi/bin/pixi run roscore
./.tools/pixi/bin/pixi run mujoco-band-headless
./.tools/pixi/bin/pixi run fsm-sim
./.tools/pixi/bin/pixi run joy-stand
./.tools/pixi/bin/pixi run joy-rl
```

Publish conservative commands through the existing input path:

```bash
source ws/devel/setup.bash
rostopic pub -r 20 /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.10, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

This validation is **NOT REAL-HARDWARE TESTED**. Do not infer authorization to
run `backend=hightorque` from these results.
