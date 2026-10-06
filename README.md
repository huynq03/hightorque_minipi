# Mini-Pi controller on the robot (`~/huy`)

```
~/huy/deploy        controller source (copied from the PC repo `mini_pi_fsm/deploy`)
~/huy/catkin_ws     catkin workspace; src/mini_pi_fsm -> ~/huy/deploy/robots/mini_pi
~/huy/scripts       check-vendor-conflict.sh
~/huy/logs          build logs
```

## 0. Environment (every new terminal)

```bash
source /opt/ros/noetic/setup.bash
source ~/install/setup.bash
source ~/huy/catkin_ws/devel/setup.bash
```

## 1. Check that the vendor controller is not running

Must print OK. It only reads; it tells you what to stop if something holds the
motor bus or the IMU.

```bash
bash ~/huy/scripts/check-vendor-conflict.sh
```

## 2. Update code from the PC (run on the PC)

```bash
cd ~/hightorque/mini_pi_fsm
rsync -a --delete deploy/ hightorque@100.97.31.5:huy/deploy/
rsync -a scripts/check-vendor-conflict.sh hightorque@100.97.31.5:huy/scripts/
```

A change to `config/` only (policy package, yaml) needs no rebuild.

## 3. Build (after a C++ change)

```bash
source /opt/ros/noetic/setup.bash && source ~/install/setup.bash
cd ~/huy/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release \
  -DMINI_PI_ONNXRUNTIME_DIR=$HOME/onnxruntime-linux-aarch64-1.20.1 -j6 \
  2>&1 | tee ~/huy/logs/build_$(date +%F_%H%M).log
```

## 4. Check a policy package (no hardware)

```bash
~/huy/catkin_ws/devel/lib/mini_pi_fsm/policy_check ~/huy/deploy/robots/mini_pi/config            # default policy
~/huy/catkin_ws/devel/lib/mini_pi_fsm/policy_check ~/huy/deploy/robots/mini_pi/config mjlab47    # one package
```

Must end with `status=POLICY_READY`.

## 5. Dry run (motors are NOT commanded)

```bash
roslaunch mini_pi_fsm mini_pi_fsm.launch backend:=hightorque
```

`dry_run` is on by default for the real backend. Check the status block:
`dry_run : TRUE`, `hardware : Ok motors=12 stale=0`, `safety status : healthy`,
`imu base frame` roll/pitch ~0 when upright, policy `pkg=` as expected.
LB+A then RB+A runs the policy without moving anything (`rate` ~50 Hz,
`finite=yes`, `obs cmd` follows the sticks).

## 6. Real run (LIVE MOTORS)

```bash
roslaunch mini_pi_fsm mini_pi_fsm.launch backend:=hightorque dry_run:=false
```
```bash
roslaunch mini_pi_fsm mini_pi_fsm.launch backend:=hightorque dry_run:=false policy_version:=mjlab47
```

The log must show `dry_run : FALSE (LIVE MOTORS)`.

Policies in `deploy/robots/mini_pi/config/policy/velocity/` (default pinned in
`config/policy.yaml`, currently `mjlab47`):

| policy | real robot |
|---|---|
| `mjlab47` | yes (default) |
| `mjlab50_v3g_zerovel` | yes (base_lin_vel fixed to 0) |
| `isaaclab_pi`, `origin_pi` | yes |
| `mjlab48`, `mjlab50`, `mjlab50_v3g` | no: need base_lin_vel, RL entry is refused |

First time with a new policy:

1. Robot hung, feet off the ground, e-stop in hand.
2. LB+A (FixStand). A joint moving the wrong way or jerking -> LB+B at once.
3. Still hung: RB+A (RL). Small stick input -> legs step; release -> legs stop.
4. LB+B, lower the robot, slack harness. LB+A, wait until it stands, RB+A.
5. Small stick inputs: forward, then yaw, then sideways.

## Gamepad / keyboard

| gamepad | keyboard | action |
|---|---|---|
| LB + A | `2` | FixStand (3 s to the stand pose) |
| RB + A | `3` | RL (policy), only after FixStand finished |
| **LB + B** | `1` | **Passive: damping, robot goes limp (EMERGENCY STOP)** |
| RB + Y | `0` | reset a latched fault |
| left stick | `i` / `,`  `J` / `L` | vx forward/back, vy left/right |
| right stick (horizontal) | `j` / `l` | yaw |
| — | `k` | zero the command (keyboard steps latch) |

Stop with LB+B or the e-stop, never rely on Ctrl+C: the node can take several
seconds to exit (press Ctrl+C twice).

## Logs

```bash
ls -t ~/.ros/log/latest/              # rosout.log holds every status block
pgrep -af "mini_pi_fsm_node|roslaunch"  # is something still running?
```
