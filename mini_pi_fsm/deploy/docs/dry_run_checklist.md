# Mini-Pi real-hardware DRY-RUN checklist

Scope: read and diagnose the real robot with `backend:=hightorque dry_run:=true`.
**No motor command frame is transmitted at any step below.** In dry run the
controller sends only read-only `MODE_MOTOR_STATE2` (0x0A) query frames (`feedback_query: state2`, the one the SDK itself uses for fun_v >= 4), which
carry no target and cannot move a motor (verified in livelybot_serial 4.4.5
source and by opcode inspection of the robot's 4.5.2 `liblivelybot_serial.so`).
The SDK itself also sends motor-version queries while it starts, and `MODE_STOP`
from its destructor on exit.

**STOP at the end of this list. `dry_run:=false` is NOT part of it.**

Robot paths below: user `hightorque`, vendor SDK in `~/install`, code in
`~/mini_pi_fsm`.

---

## STEP 0 — the vendor controller must be stopped

```bash
bash ~/mini_pi_fsm/scripts/check-vendor-conflict.sh
```

It must print `OK`. The vendor stack starts at login from
`~/.config/autostart/pi plus newTraj.desktop` → `~/sim2real_install_pi.sh` →
`roslaunch sim2real_master joy_control_pi.launch`. That launch runs
`sim2real_master_node` (respawns itself), `lr_control`, its own
`yesense_imu_node` on `/dev/ttyS7` (also respawns), and joy teleop.

If the check finds it: close the `sim2real` terminal (or `rosnode kill -a` and
kill that roslaunch). To keep it from starting at the next login, set
`Hidden=true` in that `.desktop` file **by hand**. The script never changes it.
`mini_pi_fsm_node` also refuses the hightorque backend by itself while any
vendor controller, or any other process holding `/dev/ttyACM*`, is alive.

## STEP 1 — build the real backend

```bash
source /opt/ros/noetic/setup.bash
source ~/install/setup.bash
mkdir -p ~/mini_pi_ws/src && ln -sfn ~/mini_pi_fsm/deploy ~/mini_pi_ws/src/mini_pi_fsm
cd ~/mini_pi_ws
catkin_make -DCMAKE_BUILD_TYPE=Release -DMINI_PI_ENABLE_REAL_HW=ON \
            -DMINI_PI_ONNXRUNTIME_DIR=$HOME/onnxruntime-linux-aarch64-1.20.1
source devel/setup.bash
```

Expected in the CMake output: `livelybot_serial include dirs: ...` listing
`.../include/livelybot_serial`, `.../crc` and `.../hardware`, and the ONNX
Runtime library path. The `joy` package must be installed
(`rospack find joy`). Otherwise launch with `start_joy:=false`.

## STEP 2 — launch in dry run, robot HUNG (feet off the ground)

```bash
roslaunch mini_pi_fsm mini_pi_fsm.launch backend:=hightorque   # dry_run defaults to true
```

Confirm in the first lines:
- `dry_run : TRUE (commands computed, NOT transmitted; read-only state queries only)`
- `HighTorqueHardware: initialized, 12 motors on 2 CAN ports, dry_run=true, ... feedback_query=state2`
- the SDK prints `All motor connections are normal` and a `fun_v = N` line.
  **Record N.** If readiness never passes and every motor stays `STALE`, the
  board firmware may not answer the state query (fun_v1). Report that and do
  not work around it.

## STEP 3 — 12 motors present and fresh

In the 1 Hz status table:
- `hardware : Ok motors=12 stale=0`
- every row ends in `fresh`, `flt` column is `0`
- `slot` / `id` / `sdk_name` are as expected (slot 0..5 = CANport_1 id 1..6,
  6..11 = CANport_2 id 1..6). Write down any row where the SDK name looks
  inconsistent with the joint name. The known vendor label mismatch at slots
  3/5 and 9/11 is documented in `config/mapping.yaml`.

## STEP 4 — each joint by hand (robot hung, motors unpowered-limp or passive)

For each of the 12 joints, one at a time, move it slowly by hand and watch its
row:

| check | expected |
|---|---|
| the row that changes | the joint you are moving, and only that row |
| sign | `rbt_q` increases for positive rotation about the URDF axis (hip_pitch / calf / ankle_pitch: +y; hip_roll / ankle_roll: +x; thigh: +z) |
| zero / offset | at the nominal standing pose `rbt_q ≈ 0` for every joint (`raw_q ≈ direction·offset`) |
| `raw_dq` / `rbt_dq` | non-zero only while moving, same sign relationship as q |

Record every discrepancy. Do **not** edit `mapping.yaml` until the whole table is
filled in.

## STEP 5 — IMU

Watch `imu raw (sensor)`, `imu base frame` and `obs euler` / `obs gyro`:

| motion | expected (base frame, and in `obs euler`/`obs gyro`) |
|---|---|
| robot upright, still | roll ≈ 0, pitch ≈ 0, gyro ≈ 0 |
| roll to the robot's LEFT (left side down) | roll decreases (negative) — right-hand rule about +x (forward) |
| pitch nose-DOWN | pitch increases (positive) — +y points left |
| yaw counter-clockwise (seen from above) | yaw increases, gyro z > 0 |
| `imu age` | stays well below 0.2 s |

If upright reads roll ≈ ±3.14 (sensor upside down), or the axes are swapped,
work out the sensor→base rotation and set `robot.yaml imu.mount_rpy`
([roll, pitch, yaw] of the sensor frame in the base frame; e.g. upside down
about x = `[3.14159, 0, 0]`). Relaunch and repeat this step until upright reads
[0, 0, *]. The yaw value itself is arbitrary: the policy sees yaw relative to
RL entry.

## STEP 6 — FSM in dry run, observation only

With the robot still hung, use keyboard `2` (FixStand), wait 3 s, then `3` (RL),
and `i`/`,`/`J`/`L`/`j`/`l`/`k`, `1` (Passive):
- transitions are logged exactly as in simulation, and an early `3` is refused;
- in Velocity: `policy ... rate≈50 Hz`, `finite=yes`, `obs cmd` follows the keys
  (vx×2, vy×2, yaw×1; linear norms < 0.2 m/s read as 0 by design);
- `obs q`/`obs dq` match the joint table, `obs euler` roll/pitch match STEP 5,
  yaw starts at 0 on RL entry;
- `cmd_q` / `mcmd_q` columns show targets **that are not being sent**. The
  motors stay limp, and nothing moves.

## STOP HERE

Do not set `dry_run:=false`. Before any actuation, these hardware-only items
must be resolved and recorded:
1. STEP 4 table complete: every joint's name, sign and zero verified.
2. STEP 5: IMU mount measured and `imu.mount_rpy` set, upright → [0, 0, *].
3. The firmware fun_v answers the state query (STEP 2).
4. Physical joint limits measured → `safety.yaml q_lower/q_upper`.
5. Motor torque / velocity limits from HighTorque → `tau_max` / `dq_max`.
6. Motor PD semantics (firmware kp/kd units vs. the trained kp/kd), and
   whether the firmware kd at kp=0 damps as expected.
7. Control-loop timing on the robot: `last cycle`, `overruns`, policy
   `infer_max` in the status block.
