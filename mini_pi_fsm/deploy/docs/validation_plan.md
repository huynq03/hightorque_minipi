# Validation plan

Nothing below moves the robot until Phase D. Phases A–C are read-only or
dry-run. **No step here may be run automatically.**

Status tags used throughout:
`TESTED LOCALLY` · `SOURCE-VERIFIED` · `REQUIRES HARDWARE TEST` · `UNKNOWN`

**Before any step: `sim2real_master_node` must not be running once `dry_run` is
disabled.** Only one process may own the serial port and the motor output path.

---

## Phase 0 — what has already been verified off-robot

| Item | Status |
|---|---|
| All 13 sources compile (`-Wall`, C++17) | TESTED LOCALLY |
| Package links with zero internal undefined symbols | TESTED LOCALLY |
| JointMapper round-trip, both transform modes, 397 checks | TESTED LOCALLY |
| SafetyManager fault codes + shipped configs, 273 checks | TESTED LOCALLY |
| All 5 YAML configs parse | TESTED LOCALLY |
| Joint mapping formulas vs. the original controller | SOURCE-VERIFIED (disassembly) |
| Passive gains == `HtdwMotor::protectMotor()` | SOURCE-VERIFIED |
| FixStand gains == `pi_pd_config.yaml` | SOURCE-VERIFIED |
| Physical joint order / sign / offset on the real robot | REQUIRES HARDWARE TEST |

`HighTorqueHardware.cpp` compiles but **cannot be linked off-robot**: it needs
`livelybot_serial` and `libserialport`, neither of which exists on the dev
machine. Everything it calls is source-verified; none of it has been executed.

---

## Phase A — build and bring-up (no motion, no output)

**1. Compile** — `catkin_make --only-pkg-with-deps mini_pi_fsm` on the robot.
This is the first time `HighTorqueHardware.cpp` is linked for real.

**2. Run the off-robot tests on-robot too**
```
rosrun mini_pi_fsm test_joint_mapper $(rospack find mini_pi_fsm)/config/mapping.yaml
rosrun mini_pi_fsm test_safety_and_config $(rospack find mini_pi_fsm)/config
```
Both must report 0 failures.

**3. Initialize the SDK** — `roslaunch mini_pi_fsm mini_pi_fsm.launch`
(`dry_run:=true`, the default). Expect
`HighTorqueHardware: initialized, 12 motors, dry_run=true`.

**4. Verify motor discovery** — the startup readiness dump (printed once by
`waitUntilReady()` before the FSM starts) lists
`motor[i] id=… name=… age=…` for all 12. Confirm ids 1–6 on each of the two CAN
ports.

**5. Verify motor feedback freshness** — in the status dump every `age` column
is well under `motor_timeout_s` (0.1 s) and `stale=0`. Move a leg by hand and
watch `raw_q` change. **This is also the first real check of the `motor_timeout_s`
value, which is currently UNKNOWN-derived** (taken from the SDK's own 0.1 s
threshold in `publishJointStates`). Adjust `safety.yaml` if the real jitter
demands it.

**6. Verify the IMU** — `imu age` < `imu_timeout_s`; quaternion near unit norm;
tilting the body moves `rpy` in the expected direction; `gyro` responds to
rotation. `imu_timeout_s` (0.2 s) is UNKNOWN-derived — confirm against the
actual yesense publish rate.

---

## Phase B — mapping validation (still dry-run)

**The highest-risk area.** Every value in `mapping.yaml` is source-verified but
none is hardware-verified, and §3.7 of `source_mapping.md` documents a known
disagreement between `map_index` and the hardware YAML `name:` labels for slots
3/5 and 9/11. **Resolve it physically here, before anything else.**

**7. Verify the raw motor order** — move one joint at a time by hand, record
which `raw_q` column moves. Build the real slot → physical-joint table.

**8. Verify the robot joint mapping** — for each physical joint, confirm the
`rbt_q` column whose `joint` name matches is the one that moves. A mismatch
means `mapping.yaml: map_index` is wrong. **Fix it there — never in a state
class.**

**9. Verify joint sign** — move each joint in its positive URDF direction;
`rbt_q` must increase. Flip the corresponding `mapping.yaml: direction` entry
for any joint that decreases. Note the URDF is pitch-first / yaw-third
(§3.7) — do not assume the conventional ordering.

**10. Verify `joint_offset`** — put the robot in the nominal standing pose.
Every `rbt_q` should read approximately **0**, because in this convention robot
zero *is* the nominal pose. A consistent non-zero reading means `joint_offset`
is wrong for that joint. This is the single check that validates the whole
offset convention.

**11. Tighten the position envelope** — with the verified mapping, record the
real reachable range per joint and replace the placeholder ±3.14 in
`safety.yaml: q_lower/q_upper`. Remember these are **robot-order, robot-space**
and are *not* the motor-indexed `lower`/`upper` from `pi_pd_config.yaml`.

**12. Decide the UNKNOWN limits** — `dq_max` and `tau_max` are empty and their
checks are disabled. Populate from logged data and enable, or record a decision
to leave them off.

---

## Phase C — dry-run state machine

**13. Startup → PASSIVE** — confirm `waitUntilReady()` prints the readiness
dump and then `FSM: Start Passive` on its own once the readiness checks pass,
and that nothing at all is transmitted while it is still waiting.

**14. Passive in dry-run** — `cmd_q` tracks `rbt_q`, `kp = 0`, `kd = 1`,
`command seq` increments every cycle.

**15. FixStand target in dry-run** — press LB+A. Confirm the transition logs,
`cmd_q` ramps smoothly from the measured pose to **all zeros** over
`duration_s`, and gains are 80 / 1.1. **Confirm the resulting motor-space
target is physically sensible before Phase D** — it should be
`direction[j] * joint_offset[j]`. LB+B must return to Passive.

**16. Keep RL out of this hardware dry-run** — the production policy is now
installed, so RB+A is no longer expected to be refused merely because the
package is absent. Real-policy validation belongs to the simulation procedure
in `real_policy_validation.md`; physical RL requires a separate authorization
and safety phase.

**17. Fault path** — stop the yesense node. Confirm the status dump reports
`safety status: FAULTED` with `fault reason: IMU_TIMEOUT`, that `FSM state`
reads **Passive** (never `Fault` — it is no longer a state), that the damping
hold is applied, and that RB+Y clears the latch once the IMU is restored.
Confirm the latch does NOT clear on its own when the IMU comes back.
Repeat by unplugging one motor to exercise `MOTOR_TIMEOUT` and confirm
`stale=1` names the right slot.

**17b. Startup path** — start the controller with the yesense node stopped.
Expect `Startup: waiting (… s) -- no IMU sample received yet` repeating, the
FSM never starting, `system readiness: NOT READY`, `safety status: healthy`
(not faulted — waiting is not a fault), and nothing transmitted. Start the IMU
and confirm the readiness dump appears, followed by `FSM: Start Passive`.

**18. Timing** — confirm `overruns` stays at or near 0 at 1000 Hz over several
minutes, and that the diagnostic dump appears at `debug_hz`, not at control
rate.

---

## Phase D — physical motion (operator-initiated only)

⚠️ From here the robot moves. Suspend or support it. Keep the e-stop reachable.
Set `dry_run:=false` only for these steps, and confirm no other controller is
running.

**19. Physical Passive test** — launch with `dry_run:=false`. The robot should
go limp with light damping and stay that way. Verify by hand that every joint
back-drives and none fights you. **Abort immediately if any joint stiffens.**

**20. Physical FixStand test** — robot suspended, feet off the ground. Press
LB+A. Legs should move to the nominal pose over `duration_s`. A joint
travelling the wrong way means Phase B missed a sign or permutation error.
Only once this is clean, repeat with feet on the ground.

**21. Fault response under load** — trigger a fault while standing and confirm
the damping response is survivable. Decide whether
`safety.yaml: fault_response.hard_stop` should remain `false` (damping) for
your setup; `true` makes a standing robot fall.

---

## Phase E — policy integration (future task, out of scope here)

Not startable until a policy contract exists. **POLICY CONTRACT NOT YET
DEFINED.** When one does:

22. Define and document the observation layout, action interpretation, policy
    joint order and `transform_mode`, in `config/policy.yaml`.
23. Implement `ObservationBuilder`, `ActionProcessor` and the `PolicyRunner`
    backend. Confirm no file outside the policy layer needed changing.
24. Dry-run: confirm the policy thread holds its rate and
    `policy target age` stays under `action_timeout_s`.
25. Log and verify observations term by term against the training code.
26. Bounded RL command test: tighten the output clip to a narrow band, robot
    suspended, zero `/cmd_vel`.
27. Full RL test.

---

## Rollback

At any point: LB+B → Passive, or kill the node. On shutdown `main()` calls
`hardware->protect()` (kp = 0, kd = 1, suppressed under dry-run) and the
`livelybot_serial::robot` destructor issues `set_stop()` twice.

## Still UNKNOWN / needing a decision

| Item | Where | Resolution |
|---|---|---|
| Per-joint velocity limits | `safety.yaml: dq_max` (empty, check off) | step 12 |
| Per-joint torque limits | `safety.yaml: tau_max` (empty, check off) | step 12 |
| Real `motor_timeout_s` | `safety.yaml` (0.1 s, SDK-derived) | step 5 |
| Real `imu_timeout_s` | `safety.yaml` (0.2 s, provisional) | step 6 |
| Real position envelope | `safety.yaml: q_lower/q_upper` (±3.14) | step 11 |
| `command_timeout_s` | `safety.yaml` (0.05 s, loose) | step 18 |
| Hardware `name:` labels for slots 3/5/9/11 | `source_mapping.md` §3.7 | steps 7–9 |
| Everything about the future policy | `config/policy.yaml` | Phase E |
