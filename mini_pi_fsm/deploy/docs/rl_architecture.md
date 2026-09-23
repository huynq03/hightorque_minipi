# Mini-Pi RL deployment architecture

Stage 0 report: what was read in `unitree_rl_mjlab`, what was read in this
project, and what the resulting design is.

Status labels used throughout this document and in the final report:

| label | meaning |
| --- | --- |
| SOURCE-VERIFIED | read directly out of a source file, path given |
| IMPLEMENTED | code exists in this repository |
| BUILT | compiles |
| UNIT TESTED | covered by an automated test that asserts values |
| END-TO-END SIM TESTED | exercised through the real FSM against MuJoCo |
| MODEL-VERIFIED | checked directly against the installed ONNX model |
| UNKNOWN | not determined |

---

## 1. Unitree's RL deploy data flow (SOURCE-VERIFIED)

Read from `/home/huynq/unitree_ws/unitree_rl_mjlab/deploy`.

```
CtrlFSM
  └── State_RLBase                       include/FSM/State_RLBase.h
        ├── ctor: param::parser_policy_dir(cfg["policy_dir"])   include/param.h:86
        │         ManagerBasedRLEnv(YAML::LoadFile(dir/"params"/"deploy.yaml"), robot)
        │         env->alg = OrtRunner(dir/"exported"/"policy.onnx")
        ├── enter(): copy stiffness/damping into lowcmd, start policy thread
        │            thread body: env->reset(); loop { env->step(); sleep_until }
        ├── run():   action = env->action_manager->processed_actions();
        │            lowcmd->motor_cmd()[joint_ids_map[i]].q() = action[i];
        └── exit():  stop + join the thread

ManagerBasedRLEnv::step()                include/isaaclab/envs/manager_based_rl_env.h
  robot->update()                        // pull SDK state into ArticulationData
  obs    = observation_manager->compute()
  action = alg->act(obs)                 // OrtRunner -> ONNX Runtime
  action_manager->process_action(action)
```

### The twelve questions answered

1. **`policy_dir`** — a *directory of policy packages*, e.g.
   `config/policy/velocity`, named in `config.yaml` under `FSM.<state>`.
2. **`parser_policy_dir`** (`include/param.h:86-115`) — makes the path absolute
   against `proj_dir`; if the directory has no `exported/` subdirectory it lists
   the subdirectories, sorts them **lexicographically**, and walks from the
   *end* backwards taking the first one that contains `exported/`. Logs the
   choice. There is **no** explicit-version argument.
3. **A policy package** is `<version>/params/deploy.yaml` +
   `<version>/exported/policy.onnx`. Verified: `robots/r1/config/policy/velocity/v0/params/deploy.yaml`.
4. **`ManagerBasedRLEnv`** owns `step_dt`, the `cfg`, the two managers, the
   `Articulation`, the `Algorithms` (model), `episode_length` and `global_phase`.
5. **`ObservationManager`** (`include/isaaclab/manager/observation_manager.h`)
   builds *groups* of terms. Terms are looked up in a global registry populated
   by the `REGISTER_OBSERVATION(name)` macro. Per term: `params`, `scale`,
   `clip`, `history_length`, `scale_first`.
6. **History** lives *inside each term* (`ObservationTermCfg::buff_`, a
   `std::deque<std::vector<float>>`, newest pushed at the back). Two flatten
   modes:
   - default: term-major — all of term A's history, then all of term B's;
   - `use_gym_history: true`: **frame-major** — for `h = 0 .. history_length-1`,
     append every term's frame `h`. `h = 0` is the *oldest*.
   The second mode is exactly the layout this project needs (§11 of the task).
7. **`ActionManager`** (`include/isaaclab/manager/action_manager.h`) — a
   registry of `ActionTerm` factories via `REGISTER_ACTION(name)`, splits the
   flat action vector across terms by `action_dim()`, and exposes
   `action()` (raw, for the `last_action` observation) and
   `processed_actions()` (for the FSM).
8. **`JointPositionAction`** (`include/isaaclab/envs/mdp/actions/joint_actions.h`)
   — `processed = raw * scale`, then `+= offset`, then clip. Note: Unitree
   clips the **processed** value, and `offset` carries the default pose.
9. **`OrtRunner`** (`include/isaaclab/algorithms/algorithms.h`) — reads input
   names/shapes from the session, requires every input name to be present as a
   key in the observation map, runs, copies output 0 under a mutex.
10. **The policy thread** is created in `State_RLBase::enter()` and paced with
    `sleep_until` at `env->step_dt`. `run()` (called at the FSM rate) only reads
    the latest `processed_actions()`.
11. **Joint mapping** — `deploy.yaml`'s `joint_ids_map` maps *policy index →
    SDK motor index*, applied in `State_RLBase::run()`. It is a raw index list,
    not resolved by name.
12. **Policy version selection** — lexicographic-last directory containing
    `exported/` (see 2).

### What this project deliberately does differently, and why

| Unitree | Mini-Pi | reason |
| --- | --- | --- |
| terms ordered by YAML **map** iteration order | terms are a YAML **sequence** | map order is not guaranteed by the YAML spec; task §9 makes order a configured property, so it must be explicit |
| `history_length` repeated on every term | one `history:` block per group | 15 identical repetitions is a transcription hazard; `frame_dim` is asserted against the computed width |
| `joint_ids_map` is an index list | `joint_names` resolved **by name** | task §12; an index list silently mis-maps when the robot order changes |
| `joint_ids_map` goes policy → **motor** | policy → **robot**, `JointMapper` then does robot → motor | task §12: never combine the two permutations |
| action clip applies to the **processed** value | clip applies to the **raw** action, then scale, then offset | task §13 states that order explicitly. A separate optional `clip_output` covers the processed-value case |
| `Articulation` exposes Eigen + the DDS joystick | `rl::RobotView` exposes `std::vector<float>` + `VelocityCommand` | the FSM's `Types.h` is deliberately SDK-free; `InputManager` already owns operator intent (task §22) |
| policy thread writes `lowcmd` directly | policy thread publishes a `RobotCommand` snapshot; the FSM thread consumes it | task §21; also keeps `SafetyManager` on the single write path |
| `State_RLBase` owns the env and the thread | `PolicyRunner` owns them, `State_RLBase` drives it | `CtrlFSM` already gates RL transitions on `ctx_->policy->ready()`; keeping that gate is what makes task §33 enforceable |

---

## 2. Existing Mini-Pi architecture (SOURCE-VERIFIED)

Unchanged by this work and relied upon:

- `CtrlFSM` — 1 kHz thread; `refreshState_()` → `hardware->read()` →
  `mapper->motorToRobot()` → `safety->checkState()`; then
  `pre_run/run/post_run`; then first-guard-wins transition scan. It already
  refuses a transition into any state whose `type` is `RLBase` while
  `ctx_->policy->ready()` is false (`src/FSM/CtrlFSM.cpp`).
- `FSMState::post_run()` — the **only** write path to hardware:
  `safety->clampCommand()` → `safety->checkCommand()` → `mapper->robotToMotor()`
  → `hardware->write()`. RL output goes through this untouched.
- `JointMapper` — owns **motor ↔ robot** only. The RL layer owns
  **policy ↔ robot** only. The two are never composed in one array.
- `InputManager` — `/joy` + `/cmd_vel`; `velocityCommand()` is the sole source
  of `vx, vy, dyaw`.
- `SafetyManager`, `HardwareInterface` (`SimHardware` / `HighTorqueHardware`),
  `MiniPiBridge` — untouched.

The previous `deploy/include/policy/{ObservationBuilder,ActionProcessor,HistoryBuffer}.h`
were explicit "POLICY CONTRACT NOT YET DEFINED" placeholders. They are replaced
by `deploy/include/rl/`. `PolicyRunner` is kept as the name in
`ControlContext`, because `CtrlFSM`'s RL gate already depends on it.

---

## 3. Resulting design

```
                         CtrlFSM  (1 kHz)
                            │
                            ▼
                      State_RLBase            thin: enter/run/exit only
                            │
                            ▼
                      PolicyRunner            package resolution, thread,
                            │                 mutex-protected RobotCommand snapshot
                            ▼
                    rl::PolicyEnvironment     (= ManagerBasedRLEnv)
                     /        │        \
                    ▼         ▼         ▼
      ObservationManager  ModelRunner  ActionManager
              │           (OrtRunner)        │
              │                │             │
              └────────────────┴─────────────┘
                            │
                     RobotCommand  (ROBOT joint order)
                            │
                            ▼
                    FSMState::post_run()
                      SafetyManager
                      JointMapper
                      HardwareInterface
                       /            \
                 SimHardware   HighTorqueHardware
```

Three joint orders, each owned in exactly one place:

| order | defined by | owner |
| --- | --- | --- |
| motor | `livelybot_serial::robot::Motors` | `JointMapper` (`map_index`) |
| robot / URDF | `config/mapping.yaml` `joint_names` | `JointMapper` |
| policy | `deploy.yaml` `joint_names` | `rl::RobotView` (resolved **by name**) |

`rl::` code never sees motor space. `JointMapper` never sees policy space.

---

## 4. The deploy.yaml schema

One file per policy package, at `<package>/params/deploy.yaml`. Every key that
changes policy behaviour lives here, so **installing a new compatible policy
requires no C++ edit and no recompilation** — only a new package directory and
a controller restart.

| key | required | meaning |
| --- | --- | --- |
| `simulation_only` | no | marks a TEST fixture. Refused on a non-simulation backend. |
| `step_dt` | **yes** | policy period, seconds. 0.02 → 50 Hz. |
| `joint_names` | **yes** | POLICY joint order, resolved by NAME against `mapping.yaml`. |
| `default_joint_pos` | **yes** | per policy joint, radians. Never assumed to be zero. |
| `stiffness` / `damping` | **yes** | PD gains, policy order. Not in `fsm.yaml`. |
| `commands.base_velocity.ranges` | no | `lin_vel_x` / `lin_vel_y` / `ang_vel_z` (`ang_vel_yaw` also accepted). |
| `observations.<group>` | **yes** | group name **is** the ONNX input tensor name. |
| `observations.<group>.terms` | **yes** | a **sequence**; its order is the wire format. |
| `observations.<group>.clip` | no | `[lo, hi]`, applied to the assembled, scaled frame. |
| `observations.<group>.history` | no | `{type: frame_stack, length: N, initial_value: zero, frame_dim: …, total_dim: …}`. The two dims are **asserted**, never used to size anything. |
| `actions` | **yes** | a **sequence** of `{type: …}`. |

Registered observation terms: `gait_phase`, `velocity_commands`,
`joint_pos_rel`, `joint_pos`, `joint_vel`, `joint_vel_rel`, `last_action`,
`base_ang_vel`, `base_euler`.
Registered action terms: `JointPositionAction`.

Adding a term is one class plus one `MINI_PI_REGISTER_*` line. No FSM state and
no manager changes.

### Order of operations — the part that fails silently if wrong

```
observation, per term:   raw = func(env, params)
                         [optional per-term clip]      Unitree order
                         raw *= scale
observation, per frame:  frame = clip(frame, lo, hi)   humanoid-gym `clip_obs`
                         push onto the history
observation, flattened:  [frame t-14] ... [frame t]    frame-major, contiguous

action:                  raw
                         clip(raw, lo, hi)             on the RAW action
                         *= scale
                         += offset
                         [optional clip_output]        per joint, on the target
```

## 5. Installed real policy contract

The trained ONNX policy is installed at
`config/policy/velocity/v0/exported/policy.onnx`. Its package config is
`config/policy/velocity/v0/params/deploy.yaml`.

The policy contract is SOURCE-VERIFIED from the local `Mini_Pi_RL_Baseline`
training source and MODEL-VERIFIED against the ONNX graph:

| item | training | official `sim2sim.py` | selected production v0 |
| --- | --- | --- | --- |
| actor tensor | 15 × 47 = 705 | 15 × 47 = 705 | 15 × 47 = 705 |
| action tensor | 12 | 12 | 12 |
| policy period | 0.001 × 20 = 0.020 s | 0.001 × 20 = 0.020 s | 0.020 s (50 Hz) |
| gait period | 0.4 s | 0.5 s | **0.5 s**, matching deployment reference |
| `kp` | `[40,20,20,40,40,20]×2` | same | same |
| `kd` | `[0.6,0.4,0.4,0.6,0.6,0.4]×2` | `[1.8,0.8,0.8,1.8,1.8,0.6]×2` | **sim2sim values** |
| joint order presented to actor | left leg, then right leg | swaps raw right-first MJCF blocks to left-first | left-first, resolved by name |
| observation/action clip | ±18 | ±18 | ±18 |
| action conversion | `default + 0.25 × raw` | `0.25 × raw` (defaults are zero) | `default + 0.25 × raw` |
| command ranges | vx [-0.3,0.6], vy ±0.3, yaw ±0.3 | keyboard can exceed these | package clamps to training ranges |

The 47-slot frame is gait sin/cos, scaled velocity command, relative joint
position, joint velocity, previous **raw** action, body angular velocity and
roll/pitch/yaw. Fifteen complete frames are initialized to zero, pushed once
per 50 Hz policy step, and flattened oldest to newest.

The simulation IMU contract matches the author's deployment script: MuJoCo's
local gyro is used directly; the frame quaternion is converted WXYZ→XYZW; and
the same roll/pitch/yaw equations are applied. Training obtains body angular
velocity through inverse quaternion rotation, which is equivalent to the local
gyro supplied by the MuJoCo sensor.

Still unresolved and requiring a later physical-hardware phase: the actual
motor↔robot calibration cannot be proven in this simulation, because
`MiniPiBridge` and the controller deliberately derive inverse transforms from
the same `mapping.yaml`. No physical backend was used for this validation.

## 6. The test fixture

`deploy/test/fixtures/policy/velocity/v0/` — generated by
`scripts/generate-test-policy.py` (`pixi run -e fixtures gen-test-policy`).

| file | graph | used by |
| --- | --- | --- |
| `test_zero_policy.onnx` | `y = 0·x + 0` | the MuJoCo run: action 0 → `q_target = default_joint_pos` |
| `test_sum_policy.onnx` | `y[j] = 1e-3·Σx + 1e-2·(j+1)` | the unit tests: every output depends on every input, so it **proves** the observation reached the model |
| `policy.onnx` | copy of the zero policy | the fixed filename the resolver looks for |

Neither is trained. Both packages declare `simulation_only: true`.
