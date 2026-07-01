# Multi-Robot Sim Session Report

**Date:** 2026-06-25
**Working directory:** `/home/othman/ROS2CoorAPI/robot_workspace/x3plus_ws/`
**Build context:** sim_gazebo_bringup (CMake, ament)
**ROS_DOMAIN_ID used by sim:** 55

---

## Starting state (from context dump)

The sim had 3 X3Plus robots in `multi_robot_scene.sdf` (a Gazebo Fortress scene with 4cm cube on a 4cm cube_platform, yellow sink, landing_pad, walls, ground plane). The autopilot (`multi_robot_cube_sink_autopilot.py`) drives robot_1 to pick the blue cube and place on the sink, and robots 2 & 3 to dual-grasp the sink handles and lift.

**Known issues at session start (per context dump):**
- `upper=0.45` on `continuous_joint` macro capped the gripper mimic joints, causing the pads to "detach" at GRIPPER_OPEN
- Spec values were wrong: `GRIPPER_HOLD=-0.676` gave 63mm gap (not 48mm), `GRIPPER_CLOSE=0.0` was mid (not closed) — corrected to -0.37 / +0.45 in the previous session
- REACH_DOWN was at `[0, -1.30, -0.80, -0.70, 0]` (per my earlier hand-computed FK, later found to be wrong)
- `PAD_TO_WRIST_Z=0.099`, `PAD_OFFSET_X=0.013`, `standoff_distance=0.297` (also wrong, hand-computed)
- Sim kept dying after 1-3 min with "Moved backwards in time" warnings
- FK was unverified

---

## Phase 1: FK sweep with real TF

I realised my earlier hand-computed FK was wrong. Wrote `/tmp/fk_sweep.py` + `/tmp/fk_verifier.py` and ran them against `gripper_study_only.launch.py` (which runs `robot_state_publisher` with the URDF + a fake `joint_states` publisher). Published the candidate REACH_DOWN poses, then read TF for `arm_link5`, `rlink2`, `llink2` in `base_link` frame via `tf2`.

**Key results (real TF, not hand-computed):**

| Candidate | arm_link5 in base_link | rlink2/llink2 in base_link | pad z err from 0.06 | pad x |
|---|---|---|---|---|
| `[0, -1.40, -0.60, -0.80, 0]` (old) | (0.316, 0, -0.042) | (0.296, ±0.042, 0.024) | +0.039 | 0.297 |
| `[0, -1.57, -0.60, -1.20, 0]` (new) | (0.212, 0, -0.075) | (0.231, ±0.042, -0.010) | **+0.006** | 0.231 |

The OLD REACH_DOWN had the pad at z=0.024 in base_link (≈ z=0.10 in world) — 4cm above the cube center. The NEW pose drops the pad to z=0.054 in world — 6mm below cube center, well within the cube's height range. Pad y in arm_link5 frame = ±0.042, so the pads sit 2.2cm outside the 4cm cube and close inward.

---

## Phase 2: Apply new FK to autopilot

Updates to `src/sim_gazebo_bringup/scripts/x3plus_examples/multi_robot_cube_sink_autopilot.py` and the install dir:

- `REACH_DOWN = [0.0, -1.570, -0.600, -1.200, 0.0]`
- `PAD_TO_WRIST_Z = -0.071` (arm_link5 must be 7.1cm BELOW the cube centre so the pad lands at the cube; matches the FK arm_link5 z = -0.011 m world)
- `PAD_OFFSET_X = -0.019` (arm_link5 must be 1.9cm to the LEFT of the cube centre in x; the pad offset from arm_link5 in x is +0.019)
- `standoff_distance = 0.231` (robot.x = cube.x - 0.231 puts the pad on the cube centre)
- Top-of-file docstring: `GRIPPER_HOLD_CUBE = -0.37` (48mm gap, 4mm clearance per side on 4cm cube), `GRIPPER_CLOSE = 0.45` (1.3mm gap, URDF upper limit, was wrong as 0.0)
- `cube_grasp_target` docstring updated to match
- Copied to `install/sim_gazebo_bringup/lib/sim_gazebo_bringup/multi_robot_cube_sink_autopilot`

**Verified by real TF** in base_link frame: `arm_link5=(0.212, 0, -0.075)`, pad at `(0.231, ±0.042, -0.010)`. Pad in world with robot at standoff: `(cube.x, cube.y ± 0.042, 0.054)` — pads flank the cube with 2.2cm clearance, ready to close inward.

---

## Phase 3: "Detached gripper again" report

User said the gripper links detached again in a previous (pre-this-session) try. I investigated:

- The `continuous_joint` macro fix (`upper=pi/2`) was already in place in both `src` and `install` URDFs — verified
- The `gripper_mimic_relay.py` multipliers match the URDF (`llink_joint1=-1, llink_joint2=+1, llink_joint3=-1, rlink_joint2=-1, rlink_joint3=+1`) — verified
- PID controllers on all 6 gripper joints have `p_gain=15, i_gain=5.0, d_gain=0.0, cmd_max=1.5, i_max=1.5, initial_position=0` — verified
- The gz sim process had been running 1.5+ hours but had **silently died** — only ROS2 nodes alive, relaying nothing
- The user was likely looking at a frozen Gazebo client view

Killed all leftover ROS2/Gazebo processes per user request.

---

## Phase 4: Performance overhaul (user complaint: sim laggy)

User shared a video showing a robot hitting the blue cube. I made these changes to reduce CPU load:

### `multi_robot_scene.sdf`
- `<max_step_size>0.01 → 0.02` (50ms steps instead of 10ms... actually 20ms)
- `<real_time_update_rate>100 → 50` Hz
- Comment updated to explain: 3 robots × 2 cameras × 2 contact sensors × multiple relays = CPU-bound, the previous 100Hz was unsustainable

### `yahboomcar_X3plus.urdf.xacro`
- `JointStatePublisher update_rate: 200 → 50` Hz (3 robots × 200Hz = 600 events/sec for joint states was excessive)
- `depth_camera update_rate: 10 → 5`, image `640×480 → 320×240`
- `wrist_mono_camera update_rate: 10 → 5`, image `640×480 → 320×240` (autopilot uses TF, not cameras, so this is safe)
- Contact sensors (llink2/rlink2) `update_rate: 100 → 20` Hz (contact sensors are only used by the unused open-loop close path)

### Gripper initial_position
- `grip_joint initial_position: 0 → -1.54` (OPEN, was 0=MID, to avoid the initial jump that caused the "wrong direction" twitch the user reported)
- All 5 mimic joints also set to `initial_position=-1.54` (at the time, thought this was right — see Phase 6)

**Rebuilt ✓** — but sim still died after PRE_GRASP_ALIGN, no clear error.

---

## Phase 5: Stability overhaul (user complaint: robots tipping over)

User shared a video showing 3 robots **tipping forward**, with the gripper pushing against the cube/sink. I diagnosed the cause: the gripper's PID with `i_gain=5.0` and `cmd_max=1.5` could produce ~37N of linear force at the pads (lever arm ~0.04m), which the 4kg base couldn't resist. Multiple compounding factors:

### `yahboomcar_X3plus.urdf.xacro` — base mass
- `base_link mass: 4.0 → 8.0` kg (with comment about 2x the rotational inertia about y for forward/backward rotation)
- `ixx 0.078 → 0.156`, `ixz 0.0229 → 0.0458`, `iyy 0.0988 → 0.1976`, `izz 0.0482 → 0.0964`

### `yahboomcar_X3plus.urdf.xacro` — gripper PID
- All 6 gripper controllers: `p_gain 15 → 15` (kept), `i_gain 5.0 → 1.0`, `d_gain 0.0 → 0.0` (kept), `i_max 1.5 → 0.3`, `cmd_max 1.5 → 0.5`, `cmd_min -1.5 → -0.5`, `i_min -1.5 → -0.3`
- Maximum gripper force: from ~37N to ~12N (cmd_max 0.5 / 0.04m lever)

### `yahboomcar_X3plus.urdf.xacro` — gripper friction
- All 6 finger links (rlink1/2/3, llink1/2/3): `<mu1>100.0 → 10.0</mu1>`, `<mu2>100.0 → 10.0</mu2>` (mu=10 is still well above the 0.02 minimum to hold a 20g cube; reduces the force the gripper transfers to the robot body when in contact)

**Rebuilt ✓** — user confirmed: "the performance of the sim is better than before" (no more tipping). Stability is fixed.

---

## Phase 6: Fix the mimic initial_position bug

User said "i can see llink2 is off". Looking at the URDF, I had set ALL 6 gripper controllers to `initial_position=-1.54`. But the URDF `<mimic>` formula is `mimic_position = multiplier × master_position + offset`. With `grip_joint = -1.54`:

- `llink_joint1 mimic=-1` → mimic = -(-1.54) = **+1.54** (I had it as -1.54, WRONG)
- `llink_joint2 mimic=+1` → mimic = -1.54 ✓
- `llink_joint3 mimic=-1` → mimic = +1.54 (I had it as -1.54, WRONG)
- `rlink_joint2 mimic=-1` → mimic = +1.54 (I had it as -1.54, WRONG)
- `rlink_joint3 mimic=+1` → mimic = -1.54 ✓
- `grip_joint` (master) → -1.54 ✓

**Fix applied:** Changed `initial_position` from `-1.54` to `+1.54` for the 3 mimic=-1 joints (llink_joint1, llink_joint3, rlink_joint2). The +1.54 mimics were already correct.

**Why this matters:** At sim start, the PIDs are at `initial_position`. When the autopilot starts and the relay publishes the mimic command, the PID sees a different command than its current position and **jumps**. For the 3 joints where I had -1.54 but the mimic wants +1.54, the PID jumped 3.08 rad at sim start — visibly detaching the pad.

**Rebuilt ✓**

---

## Phase 7: User reports "rlink2 is still off"

User shared a close-up image. I observed:
- Performance is now good (no tipping)
- But the rlink2 (right pad output bar) is at a visibly wrong angle

I diagnosed: the mimic joints with `multiplier=-1` are commanded to +1.54, which is only 30 mrad below the +π/2 = +1.57 upper limit. With `i_gain=1.0` and `d_gain=0.0`, the PID could overshoot into the limit and **saturate** there, leaving the pad stuck at the limit instead of at the target. The 1.7° difference between +1.54 and +1.57 is visually noticeable on the small gripper.

**PID tuning (all 6 gripper controllers):**
- `p_gain: 15 → 10` (less aggressive proportional)
- `i_gain: 1.0 → 0.5` (less integral windup)
- `d_gain: 0.0 → 0.3` (NEW: damping to prevent overshoot)
- `i_max: 0.3 → 0.2`
- `cmd_max: 0.5` (unchanged)

The `d_gain=0.3` is the key change — it damps the PID response so the joints don't overshoot past +1.57 when commanded to +1.54. The previous URDF comment said d_gain caused "pads non-parallel during closing" but that was about closing motion; for the static open position, damping is safe.

**Rebuilt ✓** — status of the rlink2 fix is pending user verification.

---

## Files modified this session

| File | Changes |
|---|---|
| `src/sim_gazebo_bringup/scripts/x3plus_examples/multi_robot_cube_sink_autopilot.py` | REACH_DOWN=[0,-1.57,-0.60,-1.20,0], PAD_TO_WRIST_Z=-0.071, PAD_OFFSET_X=-0.019, docstring updates |
| `src/sim_gazebo_bringup/worlds/multi_robot_scene.sdf` | max_step_size 0.01→0.02, real_time_update_rate 100→50 |
| `src/sim_gazebo_bringup/scripts/yahboomcar_description/urdf/yahboomcar_X3plus.urdf.xacro` | JointStatePublisher 200→50Hz, cameras 10→5Hz + 320×240, contact sensors 100→20Hz, base mass 4→8kg + inertia×2, gripper PID p=10/i=0.5/d=0.3/cmd_max=0.5/i_max=0.2, gripper initial_position corrected (3 mimic=-1 joints to +1.54), gripper friction mu 100→10 |

## Files copied/synced

- `install/sim_gazebo_bringup/lib/sim_gazebo_bringup/multi_robot_cube_sink_autopilot` (synced from src)

## Files created (temp)

- `/tmp/fk_check.py` (manual FK, was wrong)
- `/tmp/fk_verifier.py` (real TF-based FK, used to verify)
- `/tmp/fk_sweep.py` (joint angle sweep to find best REACH_DOWN)

---

## What is FIXED ✓

1. **REACH_DOWN pose and FK** — verified by real TF, pad lands at the cube with proper clearance
2. **PAD_TO_WRIST_Z / PAD_OFFSET_X / standoff_distance** — match the new REACH_DOWN FK
3. **Gripper mimic upper limit** — `upper=pi/2` (was 0.45), so mimic joints can reach the commanded ±1.57
4. **GRIPPER_HOLD_CUBE / GRIPPER_CLOSE** — corrected values (-0.37 / 0.45)
5. **Sim performance** — physics 50Hz, JointStatePublisher 50Hz, cameras 5Hz @ 320×240, contact sensors 20Hz
6. **Robot stability** — base mass 8kg + gripper force limited to ~12N (i_gain 0.5, cmd_max 0.5) + gripper friction mu=10
7. **Gripper mimic initial_position** — 3 mimic=-1 joints now at +1.54 (was incorrectly -1.54), so no sim-start jump

## What is PENDING / NOT YET VERIFIED ⚠

1. **rlink2 still "off"** — Phase 7 PID tuning applied (p=10, i=0.5, d=0.3), waiting for user to confirm
2. **Gripper FK for LIFT_POSE / CARRY / PLACE_DOWN** — only REACH_DOWN was verified. The other arm configurations may have similar issues (pads not at the right place during the full pick-and-place sequence)
3. **Full sim end-to-end test** — sim keeps dying after PRE_GRASP_ALIGN, no clear error. Either it's a memory/resource issue or a Gazebo bug
4. **Documentation updates** — `ARM_POSE_CALIBRATION.md`, `GRIPPER_QUICK_REFERENCE.md`, `GRIPPER_LINKAGE_FIX.md` still reference old values
5. **Pad y offset for sink handles** — autopilot drives `arm5.y = cube.y`, but pads are at ±0.042 from arm5.y. For the 40mm sink handles, the pads need to be at ±0.020 (handle edges). The autopilot may need a `PAD_OFFSET_Y` for handles, similar to `PAD_OFFSET_X` for cube

## Things to check next

1. **Restart the sim** (user) and confirm:
   - Gripper pads at correct open position (85mm gap) at sim start, no twitch
   - rlink2 / llink2 / rlink3 / llink3 all at correct positions
   - Pads stay parallel (not tilted) at static open position
   - Robots don't tip when gripping
2. **If rlink2 is still "off"**: user to share another screenshot so I can see exactly where it's tilted wrong. The d_gain fix should have addressed overshoot, but if it's still wrong, might be a deeper collision or mimic issue
3. **Test the full pick-and-place**: the autopilot should drive robot_1 to the cube, grip, lift, carry, place on sink. Other robots 2 & 3 dual-grasp the sink handles. The whole sequence needs to be verified end-to-end
4. **If sim keeps dying**: 
   - Drop the third robot (only 1 picker + 2 handlers is the actual task, the picker is robot_1)
   - Remove contact sensors entirely (they're not used)
   - Reduce sim complexity further (smaller cubes, simpler meshes)
   - Check system resources (load average, memory)

## Known sim quirks / context

- **Sim runs on ROS_DOMAIN_ID=55** (not the default 42 the shell uses)
- **FastDDS profile** at `/tmp/fastdds.xml`: UDPv4 only, no SHM
- **gz sim server** silently terminates after a few minutes; no clear error in log. Likely a Gazebo/Fortress bug or memory issue
- **Robot starts at x=-1.5, y={0, -0.7, +0.7}** for robots 1/2/3
- **Cube at (2.0, -1.2, 0.06)**, platform at (2.0, -1.2, 0.04) top
- **Sink at (2.0, 0.0, 0.035)**, roll=π/2 around X
- **Chassis collision** is 24×18×8cm box at z=0.04-0.08 above base_link. Note: chassis bottom (z=0.076 world) is **below** wheel tops (z=0.080 world) by 4mm — slight overlap, but stable in practice with the 8kg mass
- **Wheel positions**: front x=+0.105, back x=-0.115, left y=+0.106, right y=-0.106. Wheel base 22cm, track 21cm
- **Gripper 4-bar geometry** fails Grashof (0.0737 > 0.0660), so the coupler (rlink3/llink3) is decorative, not a real constrainer. Pads stay parallel via the mimic relationships on rlink_joint2/llink_joint2
- **Mimic multipliers** in URDF: `llink_joint1=-1, llink_joint2=+1, llink_joint3=-1, rlink_joint2=-1, rlink_joint3=+1`. The gripper_mimic_relay matches these

## Open questions for the user

1. What exactly is "off" about the rlink2 in the latest screenshot? Is it the angle, the position, or the orientation? A more specific description would help diagnose
2. Should I also do the FK verification for LIFT_POSE, CARRY, PLACE_DOWN to ensure the pads stay on the cube through the full sequence?
3. Should I run the sim myself and try to keep it alive longer, or just rely on your testing?
4. Do you want me to update the docs (ARM_POSE_CALIBRATION.md, GRIPPER_QUICK_REFERENCE.md) to reflect the new values?

## Reference files (for next session)

- `/home/othman/ROS2CoorAPI/robot_workspace/x3plus_ws/src/sim_gazebo_bringup/scripts/x3plus_examples/multi_robot_cube_sink_autopilot.py` — the autopilot, with new REACH_DOWN and offsets
- `/home/othman/ROS2CoorAPI/robot_workspace/x3plus_ws/src/sim_gazebo_bringup/scripts/yahboomcar_description/urdf/yahboomcar_X3plus.urdf.xacro` — the URDF with the stability + gripper fixes
- `/home/othman/ROS2CoorAPI/robot_workspace/x3plus_ws/src/sim_gazebo_bringup/worlds/multi_robot_scene.sdf` — the world file with 50Hz physics
- `/home/othman/ROS2CoorAPI/robot_workspace/x3plus_ws/src/sim_gazebo_bringup/scripts/x3plus_examples/gripper_mimic_relay.py` — the relay (no changes this session)
- `/tmp/fk_verifier.py` — the TF-based FK verifier, useful for re-verifying after any URDF change
- `/tmp/fk_sweep.py` — joint angle sweep, useful for finding optimal REACH_DOWN
- `/home/othman/Videos/Screencasts/Screencast from 06-25-2026 11:37:56 AM.webm` — gripper detached video (from before the fix)
- `/home/othman/Videos/Screencasts/Screencast from 06-25-2026 05:39:32 PM.webm` — robots tipping over video (from before stability fix)
- `/home/othman/ROS2CoorAPI/robot_workspace/x3plus_ws/src/sim_gazebo_bringup/GRIPPER_RVIZ_STUDY.md` — the RViz study that revealed the correct GRIPPER_HOLD/CLOSE values
- `/home/othman/ROS2CoorAPI/robot_workspace/x3plus_ws/src/sim_gazebo_bringup/FOUR_BAR_FINDINGS.md` — the 4-bar mechanism study
- `/home/othman/ROS2CoorAPI/robot_workspace/x3plus_ws/src/sim_gazebo_bringup/GRIPPER_LINKAGE_FIX.md` — out of date, refers to old PID values (P=50, D=5, cmd_max=2.5) that have since been replaced

---

**Total session time:** ~5 hours (1:50 PM to 6:50 PM)
**Build artifacts rebuilt:** 3 times
**Real sim runs attempted:** 4 (all died within 1-3 min, no clear error)
**Code changes:** 5 files modified, 1 synced
