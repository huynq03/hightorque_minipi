# Mini-Pi deploy — runtime architecture

## Scope

This package is a Mini-Pi (12 dof, HighTorque `clpai_12dof_0905`) equivalent of
`/home/huynq/unitree_ws/unitree_rl_mjlab/deploy`. The FSM framework is adapted
from Unitree; the hardware transport underneath it is HighTorque's
`livelybot_serial` SDK.

It is intended to eventually replace what `sim2real_master_node` does today. It
does **not** reimplement that node's internals — those live in a prebuilt
`libsim2real_master_arm_lib.so` and were not inspected beyond their external
contract. See `source_mapping.md` for what was verified versus inferred.

## Status

The non-policy foundation is complete. The policy layer is a deliberate
placeholder: **POLICY CONTRACT NOT YET DEFINED** (see "Policy boundary").

Working end to end (dry-run): startup readiness ->
`PASSIVE -> FIX_STAND -> PASSIVE`, and a latched runtime fault ->
`ANY STATE -> PASSIVE` with the command overridden. No RL model is required for
any of it.

## Hardware backends

The FSM drives the robot through `HardwareInterface`
(`include/hardware/HardwareInterface.h`), which has exactly two
implementations:

| backend | class | underneath |
|---|---|---|
| `hightorque` (default) | `HighTorqueHardware` | `livelybot_serial` → the physical Mini-Pi |
| `sim` | `SimHardware` | a low-level UDP bus → `MiniPiBridge` → MuJoCo |

`main()` is the **only** place in this package that knows which one is in use;
it is selected by `backend:` in `config/robot.yaml` or `backend:=` on the
launch file. `CtrlFSM`, every FSM state, `SafetyManager`, `JointMapper`,
`InputManager` and every config file are identical in both cases — the
simulator replaces the hardware beneath the controller rather than adding a
second controller. This mirrors `unitree_rl_mjlab`, where the same deploy
binary talks to `rt/lowcmd` / `rt/lowstate` whether a real robot or
`unitree_mujoco` is on the other side.

The interface is exactly the set of members the FSM already called on
`HighTorqueHardware`, plus `backendName()` for logging. Introducing it changed
no real-robot behaviour: the concrete class gained a base and `override`
keywords and nothing else.

### The build boundary

`HighTorqueHardware.cpp` is the only source file in this package that needs
`livelybot_serial`, and the HighTorque SDK ships only as an **aarch64** build
for the Mini-Pi's own computer. An x86_64 PC therefore cannot link it at all.

`MINI_PI_ENABLE_REAL_HW` (CMake, default `ON`) gates exactly two things:

* whether `src/hardware/HighTorqueHardware.cpp` is compiled, and
* whether `livelybot_serial` is a required catkin component.

`main()` names neither concrete backend; it calls `createHardwareBackend()`
(`include/hardware/HardwareBackend.h`), whose single implementation carries one
`#if MINI_PI_WITH_REAL_HW`. So a PC build simply omits that backend and links
with no SDK present, while the robot's build is unchanged. Asking a PC build
for `backend:=hightorque` produces an explicit message saying the backend was
compiled out, not "unknown backend".

Everything else -- the FSM, JointMapper, SafetyManager, InputManager,
PolicyRunner, SimHardware -- is compiled from identical sources either way, in
one shared library. See the repository README for the PC workflow.

See `../simulate/README.md` for the simulation side, including the coordinate
identity it rests on and the two things it explicitly cannot validate.

The control path below is written for the real backend. For the simulated one,
substitute `SimHardware` for `HighTorqueHardware` and the UDP bus for
`livelybot_serial`; everything above `JointMapper` is unchanged.

## Control path (per cycle, 1 kHz)

```
Mini-Pi motors
      │  serial / CAN-FD BRS, 4 Mbaud, /dev/ttyACM*
      ▼
livelybot_serial::robot          (SDK receive threads, own thread each port)
      │  motor_back_t {time, ID, mode, fault, position, velocity, torque}
      ▼
HighTorqueHardware::read()       ← the only SDK-aware class
      │  RobotState.motor_q/dq/tau  (HARDWARE MOTOR ORDER)
      │  RobotState.orientation/angular_velocity  (from /imu/data)
      ▼
JointMapper::motorToRobot()          ← THE ONLY PERMUTATION IN THE STACK
      │  robot_q[j]   = direction[j] * motor_q[map_index[j]] - joint_offset[j]
      │  robot_dq[j]  = direction[j] * motor_dq[map_index[j]]
      │  robot_tau[j] =                motor_tau[map_index[j]]
      │  RobotState.robot_q/dq/tau  (ROBOT / URDF JOINT ORDER)
      ▼
SafetyManager::checkState()      ← detection only, never a response
      │  SafetyReport → SafetyGate::raise()   (ignored until the gate is armed)
      ▼
CtrlFSM::tick_()
      │
      ▼
current BaseState  (Passive / FixStand / RLBase)
      │  enter / pre_run / run / post_run / exit
      ▼
RobotCommand  {q, dq, torque, kp, kd}   (ROBOT JOINT ORDER)
      │
      ▼
FSMState::post_run()  →  SafetyGate::publish()
      ├─ latched fault?  → discard the command, write the damping override
      ├─ SafetyManager::clampCommand()
      ├─ SafetyManager::checkCommand()   → on failure: raise() + damping override
      ▼
JointMapper::robotToMotor()
      │  motor_q[map_index[j]] = direction[j] * (robot_q[j] + joint_offset[j])
      │  motor_dq/tau[map_index[j]] = robot_dq/tau[j]   (no sign -- see below)
      │  MotorCommand (HARDWARE MOTOR ORDER)
      ▼
HighTorqueHardware::write()
      │  dry_run == true  → returns here, nothing is transmitted
      │  motor::pos_vel_tqe_kp_kd2(q, dq, tau, kp, kd)   [MODE 0xB0]
      │  robot::motor_send_2()
      ▼
Mini-Pi motors
```

The command path is a single writer: only the FSM control thread ever calls
`HighTorqueHardware::write()`.

## Joint mapping

### Where the permutation happens

`HighTorqueHardware` addresses `livelybot_serial::robot::Motors[]` **raw** and
reorders nothing, so `JointMapper` applies `map_index`. There is exactly one
permutation in the stack.

This is the fork in the road worth remembering: the HighTorque SDK also offers
`RobotMotorGroup` / `PiMotorGroup`, which apply `mapIndex[i]` **internally**
when constructing their `HtdwMotor`s. Had the hardware layer been built on a
motor group, `JointMapper` would have to apply no permutation at all. Mixing
the two would map every joint twice. See `source_mapping.md` §2 for why the raw
path was chosen.

### Exact formulas

Verified against the original controller by disassembling
`hightorque::common::transform*` out of `sim2real_master_node`; those functions
ship only as a prebuilt `.so`. Both directions were read instruction by
instruction — see `source_mapping.md` §3.

**Full mode** (`transform_mode: Full` in `config/mapping.yaml`):

```
motor -> robot
    robot_q[j]   = direction[j] * motor_q[map_index[j]] - joint_offset[j]
    robot_dq[j]  = direction[j] * motor_dq[map_index[j]]
    robot_tau[j] =                motor_tau[map_index[j]]

robot -> motor
    motor_q[map_index[j]]   = direction[j] * (robot_q[j] + joint_offset[j])
    motor_dq[map_index[j]]  = robot_dq[j]
    motor_tau[map_index[j]] = robot_tau[j]
```

Three details that are easy to get wrong, all reproduced deliberately:

1. **The offset is in robot space**, added *before* the sign on the way out and
   subtracted *after* the sign on the way in. It is **not**
   `direction * (motor - offset)`.
2. **Torque is never signed**, in either direction.
3. **Outgoing `dq` and `tau` are never signed** — the original is asymmetric
   here. Harmless in practice because every Mini-Pi command path sends
   `dq = 0`, `tau = 0` and relies on motor-side PD, but it is reproduced so the
   two controllers agree bit for bit.

The forward and inverse are exact inverses of one another (`dir = ±1`, so
`1/dir = dir`). This is asserted by `test/test_joint_mapper.cpp`.

**Identity mode** (`transform_mode: Identity`): permutation only, no sign and
no offset, in both directions. The original controller selects this only when
`algorithm == "lr"`.

> Which mode a future policy needs is a **policy decision and is not made
> here**. `Full` is the default because it is the general convention.

### `joint_offset` is not a policy default pose

`joint_offset` is the motor-zero ↔ robot-zero calibration, applied inside the
coordinate transform. Its consequence is that **`robot_q == 0` is the nominal
pose**, which is why `State_FixStand` interpolates to zero and carries no
motor-space constants.

A *policy* default pose — if some future policy expresses its action as a
residual on one — is a separate quantity that belongs to the policy layer and
must not be merged into `joint_offset`.

## Policy boundary

The production policy contract is implemented by `PolicyRunner` and the
manager-based RL layer, and is documented in `rl_architecture.md`. The trained
package lives at `config/policy/velocity/v0`; the deterministic TEST fixture
remains separate under `test/fixtures` and retains its simulation-only gate.

Package resolution, ONNX validation, observations, history, raw actions and
policy↔robot name mapping all remain outside the simulator. A missing,
malformed or incompatible package still produces `POLICY_NOT_CONFIGURED`, and
`CtrlFSM` rejects entry into `RLBase`; no fallback zero action is fabricated.

### Adding a policy later

Work should be confined to:

```
deploy/include/policy/ObservationBuilder.h   (+ .cpp)
deploy/include/policy/ActionProcessor.h      (+ .cpp)
deploy/include/policy/PolicyRunner.h / .cpp  (backend + thread)
deploy/config/policy.yaml
deploy/config/fsm.yaml                       (the RL state's gains)
deploy/config/mapping.yaml                   (transform_mode, if the policy needs Identity)
```

It must **not** require changes to `HighTorqueHardware`, `JointMapper`,
`SafetyManager`, `SafetyGate`, `InputManager`, `CtrlFSM`, `State_Passive` or
`State_FixStand`. If it does, the boundary has been broken.

The control-side contract is already pinned and unchanged from Unitree's shape:
the policy thread publishes a `PolicyRunner::Target{q, stamp, seq, valid}` and
**never** writes a command or touches the hardware; the control thread reads
it, checks freshness against `action_timeout_s`, and turns it into a
`RobotCommand` that goes through the same safety and mapping path as every
other state.

```
RobotState snapshot + VelocityCommand   (policy thread, ~50 Hz)
      -> ObservationBuilder -> HistoryBuffer -> PolicyRunner::infer
      -> ActionProcessor -> Target{q, stamp, seq}
                                   |
                                   v
State_RLBase::run()  (control thread, 1 kHz)  -- freshness check -> RobotCommand
```

## Component ownership

```
                     deploy/src/main.cpp
                             │  owns every unique_ptr
                             ▼
                          CtrlFSM
                             │  owns the state objects
           ┌─────────────────┼──────────────────┬──────────────┐
           │                 │                  │
         Passive          FixStand            RLBase
           └─────────────────┴──────────────────┘
                             │  non-owning
                             ▼
                       ControlContext
        ┌────────────────────┼────────────────────┐
        │                    │                    │
 HighTorqueHardware      JointMapper          InputManager
        │                    │                    │
        └─ SafetyManager ─ SafetyGate ─ PolicyRunner ─┘
                             │
                             ▼
                    livelybot_serial::robot
                             ▼
                          motors
```

`ControlContext` replaces Unitree's `static FSMState::lowcmd` /
`static FSMState::lowstate` globals. That is the one Unitree design decision
this package deliberately does not copy.

## State machine

The FSM contains **robot operating modes only**. Startup readiness and runtime
faults are not modes; they live outside it. This mirrors
`unitree_rl_mjlab/deploy/robots/g1/config/config.yaml`, whose `FSM._` likewise
lists nothing but `Passive` / `FixStand` / `Velocity`.

### Startup, before the FSM exists

```
main()
  │ load robot/mapping/fsm/safety/policy config
  │ ros::init + AsyncSpinner
  │ JointMapper, SafetyManager, SafetyGate (DISARMED)
  │ hardware backend (hightorque | sim)
  │ InputManager
  │ PolicyRunner            (a missing policy is NOT a startup failure)
  │ CtrlFSM constructed
  ▼
waitUntilReady()             control/SystemReadiness.h
  │ poll hardware->read() + JointMapper every startup.poll_dt_s
  │ NOT READY  → log why, keep waiting, transmit nothing, latch nothing
  │ Ctrl+C or startup.timeout_s → protect() and exit; the FSM never runs
  ▼ READY
gate->arm()                  from here a lost packet IS a runtime fault
  ▼
fsm->start("Passive")
```

Counterpart: `init_fsm_state()` +
`FSMState::lowstate->wait_for_connection()` in
`unitree_rl_mjlab/deploy/robots/g1/main.cpp:11-26`, called before
`CtrlFSM(...)` / `fsm->start()` on lines 48-49.

### The FSM itself

```
         PASSIVE ◄──────────────────────┐
            │ stand (LB+A)              │ passive (LB+B) / policy target stale
            ▼                           │ / RL bail
        FIX_STAND ───── rl (RB+A) ──► VELOCITY (State_RLBase)
            │             ▲             │
            └─────────────┘ stand ──────┘

   ANY STATE ── SafetyGate::faulted() ──► PASSIVE
```

Transition table:

| From | To | Trigger | Guard |
|---|---|---|---|
| Passive | FixStand | `stand` (LB+A) | — |
| FixStand | Passive | `passive` (LB+B) | — |
| FixStand | Velocity | `rl` (RB+A) | `PolicyRunner::ready()` **and** `State_FixStand::finished()` **and** not faulted |
| Velocity | Passive | `passive` (LB+B), or `bail_` (stale/failed policy) | — |
| Velocity | FixStand | `stand` (LB+A) | — |
| *any but Passive* | Passive | `SafetyGate::faulted()` | registered automatically by `FSMState` |

The last row is the direct counterpart of Unitree's unconditional
`lowstate->isTimeout() -> Passive` guard
(`unitree_rl_mjlab/deploy/include/FSM/FSMState.h:48-53`).

### Safety, orthogonal to the FSM

```
   FSM state          Passive | FixStand | Velocity     (operating mode)
   safety condition   healthy | FAULTED                 (system condition)
```

```
      RobotCommand
            │
            ▼
   SafetyGate::publish()
      ├─ healthy → clampCommand → checkCommand → JointMapper → hardware->write()
      └─ FAULTED → the FSM command is DISCARDED
                   → damping hold (kp = 0, kd from safety.yaml fault_response)
                   → JointMapper → hardware->write()
```

The latch is set by `SafetyGate::raise()`, which `CtrlFSM::refreshState_()`
feeds from `SafetyManager::checkState()` and `SafetyGate::publish()` feeds from
`SafetyManager::checkCommand()`. **The first report wins**, so the diagnostic
names the original cause. `SafetyGate::clear()` is the only way it opens, and
`CtrlFSM::serviceFaultReset_()` is its only caller — on the operator `reset`
input (RB+Y). A fault that merely stops being true does not restore commanding,
and because the FSM was driven to Passive meanwhile, what resumes after a reset
is Passive and never the previous RL action.

## Threads

| Thread | Rate | Owner | Responsibility |
|---|---|---|---|
| ROS AsyncSpinner (2) | — | `main` | IMU, joy, cmd_vel callbacks |
| SDK receive (1 per CAN port) | — | `livelybot_serial::robot` | decode motor frames |
| FSM control | `robot.yaml: control_hz` (1000) | `CtrlFSM` | read → state → write |
| Policy | package `step_dt` (v0: 50 Hz) | `PolicyRunner` | observation → ONNX → action target while RL is active |

`ros::spinOnce()` is never called inside the control loop; the spinner handles
callbacks independently. Pacing uses `steady_clock` + `sleep_until`, with an
overrun counter exposed in the status dump. The reference rates come from
`sim2real/config/walk/lr.yaml`: `pd_ctrl_f: 1000`, `rl_ctrl_f: 50`.
`control_hz` is configurable in `config/robot.yaml`.

### Freshness and command ageing

Motor freshness is tracked **per motor**, from `motor_back_t::time`. Merely
dereferencing a motor object proves nothing, so each of the 12 carries its own
age and `stale_motor_count`; a single stale motor makes the whole transport
`HardwareStatus::Stale`. IMU age is tracked separately. The timeouts come from
`config/safety.yaml` and are handed to the hardware layer by `main()`, so the
hardware and `SafetyManager` never disagree about what "stale" means.

Every command carries `stamp` and a monotonic `seq`, bumped by
`FSMState::stampValid()`. `RobotCommand::reset()` clears the stamp each cycle,
so a state that stops refreshing trips `COMMAND_TIMEOUT` rather than having its
last command reused indefinitely. `Passive`, `FixStand` and `SafetyGate`'s own
safe override all recompute from the freshly measured pose every cycle.

## Dry run

`robot.yaml: dry_run` defaults to **true**. In dry run everything runs —
feedback, IMU, mapping, FSM transitions, safety, policy, command computation —
but `HighTorqueHardware::write()`, `protect()` and `stop()` return before
touching the SDK, and `set_timeout()` is skipped at init.

All four transmit paths are guarded: `write()`, `protect()`, `stop()` and the
`set_timeout()` call at init. This is asserted in the code, not assumed.

**Only one process may own the motor output path.** Keep `dry_run: true`
whenever `sim2real_master_node` is running.
