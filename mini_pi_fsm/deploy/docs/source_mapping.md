# Source mapping and evidence

Paths are relative to:

- **U** = `/home/huynq/unitree_ws/unitree_rl_mjlab/deploy`
- **H** = `/home/huynq/hightorque/hightorque_rl_control_box-sw-pizk-r-pkg-2.0.0/src`
- **I** = `/home/huynq/hightorque/install`

Claims are tagged **FACT** (read directly in source), **INFERENCE** (derived
from several pieces of evidence) or **UNKNOWN**.

---

## 1. Component table

| New component | Reference source | Status | Evidence | Reason |
|---|---|---|---|---|
| `include/FSM/BaseState.h` | U `include/FSM/BaseState.h` | adapted | lifecycle, id↔name map, `REGISTER_FSM` macro | Same design; dropped `boost::bimap` for a small hand-rolled registry, and the factory takes a `ControlContext*` |
| `include/FSM/FSMState.h` | U `include/FSM/FSMState.h` | adapted | transitions read from config, global guard, `pre_run`/`post_run` | Joystick DSL replaced by named triggers |
| `include/FSM/CtrlFSM.h` | U `include/FSM/CtrlFSM.h` | adapted | `_` block, duplicate-id check, first-guard-wins scan | `RecurrentThread` → `std::thread` + `sleep_until`; configurable period |
| `include/FSM/State_Passive.h` | U `include/FSM/State_Passive.h` | adapted | kp=0, track measured q | Gains are HighTorque's, not Unitree's (see §4) |
| `include/FSM/State_FixStand.h` | U `include/FSM/State_FixStand.h` | adapted | `ts`/`qs` keyframes, `qs[0]` = measured pose at `enter()` | Interpolation moved into robot joint space |
| `include/control/LinearInterpolator.h` | U `include/LinearInterpolator.h` | adapted | same piecewise-linear routine | float→double, assert→guard |
| `include/FSM/State_RLBase.h` | U `include/FSM/State_RLBase.h` | adapted, **currently a refusal gate** | policy thread started in `enter()`, joined in `exit()`; `run()` only reads a target | Added freshness check + blend-in; env replaced |
| `include/control/SystemReadiness.h` | U `robots/g1/main.cpp:11-26` `init_fsm_state()` + `wait_for_connection()` | adapted | readiness is a blocking pre-FSM wait, not an FSM state | Unitree waits for one DDS packet; the Mini-Pi SDK discovers motors at runtime, so there is more to wait for |
| `include/control/SafetyGate.h` | U `FSMState.h:48-53` (`isTimeout() -> Passive`) | adapted | a runtime fault drives the FSM to the safe operating mode, not to a fault mode | Mini-Pi additionally latches the output override, because the HighTorque bus gives no independent guarantee once feedback is untrustworthy |
| `include/control/Types.h` | U `robots/g1/include/Types.h` | rewritten | — | Unitree aliases DDS types; we need SDK-free structs |
| `include/control/ControlContext.h` | U static `FSMState::lowcmd/lowstate` | rewritten | — | Avoid mutable globals |
| `include/hardware/HighTorqueHardware.h` | H `livelybot_serial/include/hardware/*.h` | new adapter | see §2 | Replaces `LowCmd_t` + `LowState_t` |
| `include/control/JointMapper.h` | H `common::transform*` (disassembled) + `pi_pd_config.yaml` | new, formula-equivalent; implements both `TransformMode`s | see §3 | Unitree's `joint_ids_map` lives inside the articulation; we make it a first-class component |
| `include/control/SafetyManager.h` | partly U `isaaclab::mdp::bad_orientation` | new | see §5 | Unitree spreads these checks across env + lowstate |
| `include/input/InputManager.h` | U `unitree_joystick_dsl.hpp` | rewritten | see §6 | DSL not ported |
| `include/policy/*` | — | **placeholder / extension point** | see §7 | Mini-Pi obs layout differs entirely |

## 2. HighTorque hardware interface — FACT

Source: `H/livelybot_hardware_sdk_4_4_5/src/livelybot_serial`.

- **FACT** `livelybot_serial::robot` (`include/hardware/robot.h`) exposes
  `std::vector<motor*> Motors`, `motor_send_2()`, `set_stop()`, `set_reset()`,
  `set_timeout(int16_t)`, `set_reset_zero()`, `set_motor_runzero()`, and the
  public flags `motor_position_limit_flag` / `motor_torque_limit_flag`.
- **FACT** `robot::robot()` reads every `robot/...` ROS parameter, opens the
  serial ports, spawns one receive thread per port and probes the motors
  (`src/hardware/robot.cc:7-148`).
- **FACT** the global `Motors` order is canboard → canport → motor
  (`robot.cc:112` calls `cp->puch_motor(&Motors)`; `canport.cc:479` appends in
  port order).
- **FACT** feedback is `motor_back_t { double time; uint8_t ID, mode, fault;
  float position, velocity, torque; }` (`include/serial_struct.h:183`).
  `time` is `ros::Time::now().toSec()` at decode (`motor.cc`, `fresh_data`).
- **FACT** command entry points are `motor::position/velocity/torque/...`,
  `pos_vel_tqe_kp_kd`, `pos_vel_tqe_kp_kd2`, `pos_vel_kp_kd`, `pos_vel_acc`,
  `pos_vel_MAXtqe`, `stop()`, `brake()`.
- **FACT** `control_type: 12` in
  `H/sim2real/robot_param/12dof_STM32H730_pi_lubancat_params.yaml`
  corresponds to `MotorControlType::POS_VEL_TQE_KP_KD2`
  (`H/sim2real_sdk/include/motor/motor_base.h`), i.e. serial mode `0xB0`.
  This package therefore uses `pos_vel_tqe_kp_kd2`.
- **FACT** `robot::~robot()` already issues `set_stop()` + two `motor_send_2()`
  and joins the receive threads, so `HighTorqueHardware` only needs to drop the
  shared_ptr.

### Why not `hightorque::RobotMotorGroup`?

`H/sim2real_sdk` has full source for `RobotMotorGroup`, `PiMotorGroup`,
`HtdwMotor`, `Motor_State` and `Motor_Output`, and they are a good fit
conceptually. They are **not** used here because:

1. **FACT** `sim2real_sdk` ships no `CMakeLists.txt` / `package.xml`; it is only
   ever compiled into `H/sim2real/lib/libsim2real_*_lib.so`, and those are
   prebuilt aarch64/x86 blobs.
2. **FACT** its headers pull in `H/sim2real/include/robot_data.h`, which drags
   in Eigen, `sim2real_msg/*` and `hightorque_hardware_sdk/*` message types.

Both would leak SDK/message details past the hardware boundary. The adapter
therefore targets `livelybot_serial` directly, which builds from source.
The behaviours that matter were still copied from the group layer:

- **FACT** `HtdwMotor::protectMotor()` (`sim2real_sdk/src/motor/htdw_motor.cpp`)
  is `setMotor(0, 0, 0, kp=0, ki=0, kd=1.0, …)` — this is the source of the
  Mini-Pi passive gains.
- **FACT** `PiMotorGroup` builds its `"all"` / `"lowerBody"` groups by walking
  `map_index` and `joint_names` together
  (`sim2real_sdk/src/motor/pi_motor_group.cpp`), i.e. group order is *robot*
  order and the permutation is already applied at that layer.

## 3. Joint mapping -- FACT (disassembly-verified)

Sources: `H/sim2real/config/pi_pd_config.yaml`, `H/sim2real/config/walk/lr.yaml`,
`H/sim2real/robot_param/12dof_STM32H730_pi_lubancat_params.yaml`,
`H/clpai_12dof_0905/urdf/clpai_12dof_0905_rl.urdf`, and the machine code of
`I/lib/sim2real_master/sim2real_master_node`.

### 3.1 Where the implementations live

- **FACT** `common::transformMotorStateToRobotState` and
  `common::transformRobotOutputToMotorOutput` are **declared** in
  `H/sim2real/include/common.h` (lines 189-264) and **defined nowhere in the
  source tree**. They exist only as compiled code inside
  `H/sim2real/lib/libsim2real_*_lib.so` and `sim2real_master_node`.
- **FACT** the definitions were read by disassembling the (not stripped)
  aarch64 binary with `llvm-objdump-19`:

| Symbol | Address |
|---|---|
| `transformRobotOutputToMotorOutput(const float&, float&, const double&, int)` | `0x13192d0` |
| `transformRobotOutputToMotorOutput(Robot_Output&, Motor_Output&, ..., int)` | `0x1319328` |
| `transformRobotOutputToMotorOutput(Robot_Output&, Motor_Output&, ..., const vector<int>&, int)` | `0x13195ec` |
| `transformMotorStateToRobotState(Motor_State&, Robot_State&, ..., int)` | `0x1319980` |
| `transformMotorStateToRobotState(Motor_State&, Robot_State&, ..., const vector<int>&, int)` | `0x131a75c` |

- **FACT** no function named `motorToRobot`, `robotToMotor`, `actual_to_motor`,
  `motor_direction` or `zero_offset` exists anywhere in the HighTorque source.
  Those were our names, not theirs.

### 3.2 The scalar form -- the cleanest statement of the convention

**FACT**, from `0x13192d0`: `fadd` (r + offset) then `fmul` by direction.

```
m = direction * (r + offset)
```

The offset is added **before** the sign.

### 3.3 The vector forms

**FACT**, `0x13195ec` (robot -> motor) and `0x131a75c` (motor -> robot):

```
k = index[i]
robot -> motor:   tmp = r.target_q[i] + offset[k]
                  r.target_q[i] = tmp                   // in-place mutation
                  m.target_q[i] = direction[k] * tmp
                  m.target_dq[i]  = r.target_dq[i]      // NO sign
                  m.target_tau[i] = r.target_tau[i]     // NO sign

motor -> robot:   r.q[i]   = m.q[i]  * direction[k]
                  r.dq[i]  = m.dq[i] * direction[k]
                  r.q[i]  -= offset[k]                  // fsub, AFTER the sign
                  r.tau[i] = m.tau[i]                   // NO sign
```

- **FACT -- the key structural finding:** the loop index `i` is the **same on
  both sides**. These functions **do not permute anything**. `index[]` is used
  *only* to select which entry of `offset[]` and `direction[]` to apply.
- **FACT** the permutation is applied by `MotorGroup`, not by the transform:
  `PiMotorGroup` constructs `HtdwMotor(rbPtr, mapIndex[i], names[i], ...)`, and
  `MotorGroup::getMotorState()` / `setMotors()` iterate `motors_[i]`
  (`sim2real_sdk/src/motor/{pi_motor_group,motor_group}.cpp`). So
  `Motor_State`/`Motor_Output` index `i` **is already robot joint `i`** and
  physically addresses `Motors[map_index[i]]`.

### 3.4 Which arrays are passed

**FACT**, from the call sites (`Sim2Real::execRlLoop` @ `0x1551c28`,
`0x1552974`; `Sim2Real::upperBodyRecover` @ `0x154b574`), every caller passes:

- `offset` = `pdInfo_.byMotorId.urdf_offset` (`this+0x1bc8`)
- `direction` = `pdInfo_.byMotorId.direction` (`this+0x1b50`)
- `index` = `getMotorGroupIndexByName(...)` = `map_index`

Member offsets were derived from `rlInfo_.algorithm` @ `this+0x1e20` and
`rlInfo_.dofs` @ `this+0x1e40`, giving `pdInfo_` base `0x1970`; independently
confirmed by `this+0x1be0` = `byMotorId.clip_output_lower`, used for clamping in
the same function.

- **FACT** `hightorque_config.cpp::fillByMotorId` does
  `dataById[mapIndex[i]] = data[i]`, so `byMotorId.X[map_index[i]] == X[i]`.
  The double indirection **cancels**: `direction[]` and `urdf_offset[]` as
  written in `pi_pd_config.yaml` are indexed by **robot joint index**.

### 3.5 Net formulas, and what this package implements

```
motor -> robot :  robot_q[j]   = direction[j] * motor_q[map_index[j]] - joint_offset[j]
                  robot_dq[j]  = direction[j] * motor_dq[map_index[j]]
                  robot_tau[j] =                motor_tau[map_index[j]]

robot -> motor :  motor_q[map_index[j]]   = direction[j] * (robot_q[j] + joint_offset[j])
                  motor_dq[map_index[j]]  = robot_dq[j]
                  motor_tau[map_index[j]] = robot_tau[j]
```

- **FACT -- consistent.** From `motor = dir*(robot + off)` with `dir` in
  `{+1,-1}` so `1/dir = dir`: `robot = dir*motor - off`, which is exactly the
  measured inverse. Asserted by `test/test_joint_mapper.cpp`.
- **FACT -- asymmetric by design.** `dq` is signed inbound but not outbound;
  `tau` is signed in neither. Reproduced deliberately. Practically harmless
  because every Mini-Pi command path sends `dq = 0`, `tau = 0`.

### 3.6 `urdf_offset` -> `joint_offset`: what it actually is

- **FACT** it is passed as the `offset` argument of the motor<->robot transform
  and consumed nowhere else.
- **FACT** `utils.cpp::refreshSingle` builds an all-zero `Robot_Output`,
  transforms it, and stores it as pose `"zero"` under the comment `// 站立`
  (*standing*). So **`robot_q == 0` is the standing pose**.
- **FACT** `RLConfig::urdf_offset` is commented
  `// URDF偏移, 从电机0位偏移到policy初始位置` ("offset from motor zero to policy
  initial position"), and `PDConfig::refreshRLConfig` overwrites
  `pdInfo_.urdf_offset` from the *active policy's* `rlInfo_.urdf_offset`
  through `policy_to_actual_map` -- it is per-policy.
- **Verdict:** it is the policy's default pose expressed in robot-joint
  coordinates **and installed as the motor-zero <-> robot-zero calibration
  inside the coordinate transform**. Those are the same quantity in this
  design, because robot space is defined so that zero *is* the default pose.
  It is **not** a term added to a policy action.
- In this package it is `config/mapping.yaml: joint_offset`. The earlier names
  `zero_offset` / `default_pose` are gone, and `JointMapper::loadFromYaml()`
  **rejects** a config that still contains them rather than ignoring them.

### 3.7 The `map_index` vs `name:` mismatch -- RESOLVED

- **FACT (code authority)** `motor::get_motor_name()` is referenced in exactly
  one place in the whole codebase: `robot.cc:208`, inside
  `publishJointStates()`, feeding the diagnostic `/error_joint_states` topic.
  **No control path reads it.** `PiMotorGroup` names its motors from
  `pi_pd_config.yaml: joint_names`, never from the hardware YAML.
  `map_index` is authoritative.
- **FACT (physical corroboration)** the URDF chain from `base_link` outward is
  `hip_pitch (axis 0 1 0)` -> `hip_roll (1 0 0)` -> `thigh (0 0 1 = YAW)` ->
  `calf` -> `ankle_pitch` -> `ankle_roll`. This robot is **pitch-first,
  yaw-third**, which is unusual. CAN slots 0,1,2,4 are unambiguously
  ankle_roll, ankle_pitch, knee, hip_roll, so the daisy chain runs
  distal->proximal; slot 5 is therefore the most proximal joint (`hip_pitch`,
  the one on `base_link`) and slot 3 sits just above the knee (`thigh`/yaw).
  That is exactly what `map_index` says.
- **Conclusion:** the hardware YAML `name:` labels for slots 3/5 and 9/11 are
  the wrong ones -- written under the conventional yaw-first assumption, which
  does not match this URDF. 4 of 6 labels per leg corroborate `map_index`; the
  URDF chain resolves the other 2.

### 3.8 Final table

| Robot joint | Robot idx | Motor idx | HW label | Direction | joint_offset |
|---|--:|--:|---|--:|--:|
| l_hip_pitch_joint | 0 | 5 | `L_hip_yaw` (mislabelled) | +1 | -0.25 |
| l_hip_roll_joint | 1 | 4 | `L_hip_roll` | +1 | 0.00 |
| l_thigh_joint (yaw) | 2 | 3 | `L_hip_pitch` (mislabelled) | -1 | 0.00 |
| l_calf_joint | 3 | 2 | `L_calf` | -1 | +0.65 |
| l_ankle_pitch_joint | 4 | 1 | `L_up_foot` | +1 | -0.40 |
| l_ankle_roll_joint | 5 | 0 | `L_low_foot` | +1 | 0.00 |
| r_hip_pitch_joint | 6 | 11 | `R_hip_yaw` (mislabelled) | -1 | -0.25 |
| r_hip_roll_joint | 7 | 10 | `R_hip_roll` | +1 | 0.00 |
| r_thigh_joint (yaw) | 8 | 9 | `R_hip_pitch` (mislabelled) | -1 | 0.00 |
| r_calf_joint | 9 | 8 | `R_calf` | +1 | +0.65 |
| r_ankle_pitch_joint | 10 | 7 | `R_up_foot` | -1 | -0.40 |
| r_ankle_roll_joint | 11 | 6 | `R_low_foot` | +1 | 0.00 |

**REQUIRES HARDWARE TEST.** Every row above is source-verified but none of it
has been confirmed against the physical robot.

### 3.9 Transform modes

- **FACT** `Sim2Real::execRlLoop` branches on `rlInfo_.algorithm`:

| Branch | motor -> robot | robot -> motor |
|---|---|---|
| `== "lr"` (`0x1551a18` / `0x1552814`) | `robotState_.q = motorState.q.head(dofs)` -- identity | `motorOutput_.target_q = robotOutput_.target_q.head(n)`; `dq`,`tau` `setConstant(0)` -- identity |
| `== "footstep_jz"` (`0x1551b94`) | general transform, `offset = vector<double>(n, 0.0)` | -- |
| else (`0x1551c28` / `0x1552974`) | general transform with `byMotorId.urdf_offset` | same |

  For `lr`, both directions are identity -- symmetric and self-consistent.
- **FACT** `sim2real/config/pi_rl_config.yaml` enables exactly two policies:
  `lr` (walk) and `host` (up). So the **shipped Mini-Pi walk policy runs in raw
  motor space**. Composing `map_index` with `lr`'s `joint_names` gives policy
  index `p` <-> `Motors[p]` exactly: `lr`'s joint list is simply a naming of the
  raw motor sequence.
- **FACT** `lr` is not an ONNX model at all: `lr_inference_engine.cpp` calls the
  ROS service `/model_output` (`sim2real_msg::ModelOutput`).
- **FACT** the shipped binary's `LrInferenceEngine` constructor takes two extra
  `vector<int>` arguments that the source-tree version does not. **The binary
  was built from newer source than what ships in this tree.** Documented, not
  patched.
- This package exposes both as `TransformMode::Full` / `TransformMode::Identity`
  via `config/mapping.yaml: transform_mode`, defaulting to `Full`.
  **Which mode a future policy needs is a policy decision and is deliberately
  not made here.**

## 4. Passive behaviour — FACT

- **FACT** the SDK's own protect path is `kp = 0`, `kd = 1.0`, zero position /
  velocity / torque targets (`htdw_motor.cpp`, `HtdwMotor::protectMotor`).
- **FACT** `robot::motor_send_2()` itself falls back to
  `pos_vel_tqe_kp_kd(current_position, 0, 0, 10, 1)` when the IMU tilt limit
  trips (`robot.cc:294-300`) — again a low-stiffness hold.
- Mini-Pi `State_Passive` therefore uses `kp = 0`, `kd = 1.0` and tracks the
  measured position, which is safe because it is exactly the damping the
  firmware is already tuned for on this robot. Unitree's `kd: 3` was **not**
  carried over.
- **FACT** `motor::stop()` exists (serial mode `0x03`) and is available through
  `HighTorqueHardware::stop()`, gated behind
  `safety.yaml: fault_response.hard_stop`.

## 5. Existing safety / protection — FACT

- **FACT** per-motor position and torque limits exist in the param file
  (`pos_limit_enable`, `pos_upper/lower`, `tor_limit_enable`, `tor_upper/lower`)
  and are enforced in `motor::fresh_data`; violations raise
  `robot::motor_position_limit_flag` / `motor_torque_limit_flag`, which then
  block all sending in `motor_send_2()`. **For the Mini-Pi they are all
  `false`.** `HighTorqueHardware::sdkLimitTripped()` surfaces the flags anyway.
- **FACT** IMU tilt limiting exists in the SDK (`robot::imu_limt()`,
  `imu_limt_flag`, `imu_limt_num: 1.0`) but is **disabled**
  (`imu_limt_flag: false`) for the Mini-Pi.
- **FACT** `motor_back_t::fault` is a per-motor fault code; `HtdwMotor`
  warns on it. We escalate it to `HardwareStatus::Fault`.
- **FACT** a firmware watchdog exists (`robot::set_timeout`, mode `0x85`) but
  the SDK's own call is commented out (`robot.cc:134`). Exposed as
  `robot.yaml: motor_watchdog_ms`, default 0 = leave alone.
- **FACT** separate fall-detection nodes exist
  (`livelybot_bringup/src/robot_falldown_protect.cpp`,
  `cfg/falldown_condition_pi.yaml`) and an emergency-stop node
  (`robot_emergency_stop.cpp`). They are not replaced by this package.
- **UNKNOWN** per-joint velocity and torque envelopes. No Mini-Pi config
  declares them, so `safety.yaml` ships a loose `dq_max` marked as a sanity
  bound and leaves `enable_torque_limit: false` rather than inventing numbers.

## 6. Input — FACT

- **FACT** `I/share/sim2real_master/joy.yaml` maps `/joy` buttons 0…10 to
  `a, b, x, y, lb, rb, back, start, center, L, R` and publishes
  `geometry_msgs/Twist` on `cmd_vel` plus `sim2real_msg/Joy` on `/joy_msg`.
- **FACT** `joy_control_pi.launch` starts `joy_teleop.launch`, the yesense IMU
  node on `/dev/ttyS7` @ 460800, and loads
  `12dof_STM32H730_pi_lubancat_params.yaml`.
- We subscribe to raw `/joy` rather than `sim2real_msg/Joy` so this package does
  not depend on the `sim2real_msg` package.
- Unitree's `unitree_joystick_dsl.hpp` (`"LT + up.on_pressed"`) is **not**
  ported. `fsm.yaml` names a trigger (`stand`, `rl`, `passive`, `reset`,
  `always`) and `InputManager` owns the button combination. No parser, and the
  mapping is greppable.

## 7. Policy — POLICY CONTRACT NOT YET DEFINED

No policy is integrated in this package and nothing below has been adopted as a
contract. Recorded only as background for whoever does the integration.

- **FACT** the Mini-Pi enables `lr` (walk) and `host` (up) via
  `H/sim2real/config/pi_rl_config.yaml`.
- **FACT** `lr.yaml`: `num_single_obs: 37`, `frame_stack: 1`, `clip_obs: 18.0`,
  `action_scale: 1.0`, `pd_ctrl_f: 1000`, `rl_ctrl_f: 50`, `kp: 80`, `kd: 1.1`.
- **FACT** `lr_policy.cpp::updateObservation` fills indices 0..32:
  joint_pos(12), joint_vel(12), base_ang_vel(3), base_euler(3), then
  `Vector3d(cmdVel.vy, cmdVel.vx, cmdVel.dyaw)` — note the x/y swap. Indices
  33..36 (joystick A/B/X/Y) are documented in the yaml but left zero by
  `updateObservation`.
- **FACT** `lr` reaches its model over the ROS service `/model_output`, not a
  local ONNX session (`lr_inference_engine.cpp`).
- **UNKNOWN** the I/O contract of every model under `H/sim2real/policy/`
  (`.rknn`, `.trt`, `combined_model_*.onnx`). None was loaded or run.
- **DECIDED NOT TO DECIDE:** observation layout, action interpretation, policy
  joint order, action scale, policy default pose, transform mode, inference
  backend. All deferred.

`config/policy.yaml` ships `configured: false`, and
`PolicyRunner::initialize()` refuses to report ready if that flag is flipped
without an implemented contract.

## 8. `sim2real_master_node` — external contract only

Source not available: only `I/lib/sim2real_master/sim2real_master_node`
(aarch64 ELF, not stripped) and `H/sim2real/lib/libsim2real_master_arm_lib.so`.

- **FACT** (from the launch file `I/share/sim2real_master/launch/joy_control_pi.launch`)
  it takes params `model_type`, `devel_level`, `kick_duration`,
  `use_fell_down_policy`, `use_fell_down_action`, `config_path`,
  `pd_config_file`, `rl_config_file`, `custom_rl_config_file`, `policy_path`,
  `dynamic_config_file`, `resources_dir`, `urdf_path`, `mesh_path`; and it is
  launched alongside `lr_control`, `power_node`, `livelybot_oled_node`,
  `yesense_imu_node`, `joy_teleop` and `rosbridge_websocket`.
- **EVIDENCE** (symbol/string table of the binary) it publishes or subscribes
  `/cmd_vel`, `/joy_msg`, `/fsm_state`, `/imu/data`, `/joint_goals`,
  `/motor_goals`, `/error_joint_states`, `/way_point`; it hosts a
  `sim2real_msg/lowlevel_controller` actionlib server and a
  `sim2real_msg/Common` service; it uses `hightorque::Sim2Real`,
  `HighlevelControllerNode`, `LowlevelControllerNode`, `DefaultControllerNode`,
  `DevelopControllerInterface`, `CustomControllerInterface`,
  `teach::TeachControllerInterface`, `utils::TrajectoryGenerator` and the
  `Lr/DreaWaQ/FootStep/Humanoidgym/PBHC/Host/BYDMMC` policies.
- **EVIDENCE** its own state machine is a `boost::msm` machine
  `hightorque::ModeFsmDef` with states `Init`, `Candidate{Default, Custom,
  Develop, Remote, Teaching, Calibration}`, `Exec{Default, Custom, Develop,
  Remote, Teaching, Calibrating, CalibrationSuccess, CalibrationFailed}`,
  `ProtectionShutdown`, `Error`, driven by a `TickEvt` with guards
  `GuardNotAllNormal`, `GuardShutdown`, `GuardTimeout3s`.
- **FACT** `H/sim2real/include/sim2real.h` (header available, implementation in
  the `.so`) declares the user-facing state enum `INIT, STANDING, STANDBY,
  RUNNING, SITTING, TRAJ_SINGLE, TRAJ_MULTI, TRAJ_SERIES` plus `FellDown`.
- **INFERENCE** this package covers the `Default`/`Exec` walking path only.
  Teaching, calibration, waypoint trajectories, fall-recovery actions, the
  actionlib interface and the `/fsm_state` + `/joint_goals` + `/motor_goals`
  topics are **not** implemented.
- **UNKNOWN** everything about the node's internal control math. Nothing was
  reconstructed from it; where behaviour was needed it came from the
  source-available layers (`livelybot_serial`, `sim2real_sdk`) or the configs.

## 9. Not carried over from Unitree

| Unitree | Why not |
|---|---|
| `LowCmd_t` / `LowState_t` (DDS) | HighTorque uses serial/CAN-FD, no DDS |
| `unitree::robot::ChannelFactory`, `go2::shutdown()` | DDS-specific |
| `unitree_articulation.h` | tied to the DDS lowstate layout |
| `unitree::common::RecurrentThread` | replaced by `std::thread` + `sleep_until` |
| `unitree_joystick_dsl.hpp` | replaced by named triggers (§6) |
| `isaaclab/*` (env, managers, terminations) | policy contract is entirely different |
| `boost::program_options`, `spdlog`, `param.h` | ROS 1 node: `NodeHandle` params + `ROS_*` logging |
| `thirdparty/cnpy` | only needed for Unitree's motion-mimic `.npz` |
| ONNX Runtime / RKNN | no inference backend is linked; POLICY CONTRACT NOT YET DEFINED |
| `check_mode_machine` / `mode_machine = 5` | Unitree robot-type handshake |
