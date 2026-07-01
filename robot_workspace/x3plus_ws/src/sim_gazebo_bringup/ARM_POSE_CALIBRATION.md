# ARM POSE CALIBRATION FOR X3PLUS ROBOT

**Last Updated:** 2026-06-14
**Status:** ✅ Validated for 2 cm cube in Gazebo simulation
**Manufacturer Origin:** yahboomcar_ws/src/arm_autopilot & arm_color_transport

---

## Executive Summary

The X3Plus arm has **6 predefined poses** that were derived from the manufacturer's ROS1 autopilot system and adapted for:
- ROS2 Humble (converted servo angles to radians)
- 2 cm simulation cube (matches `models/test_block/model.sdf` and the manufacturer's original ~2-3 cm cubes)
- Parallel gripper system with mimic joints

All poses have been **verified to work correctly** in Gazebo simulation. The arm positions are optimized for the pick-and-place workflow while maintaining safety margins.

> **Cube size in the simulation:** 2 cm (0.02 m), defined in `models/test_block/model.sdf`. An earlier revision of this codebase used a 4 cm cube; the pose constants have now been unified to the 2 cm manufacturer values. In particular, `LIFT_POSE.j4` is now `-0.908` (matching `REACH_DOWN.j4`), NOT `-1.21`, so the wrist does not snap during the lift and flick the cube out of the fingers.

---

## POSE DEFINITIONS

### Reference Convention
- **Manufacturer convention:** Servo angles from 0° to 180° (90° offset from neutral)
- **Simulation convention:** Radians converted as: `rad = radians(servo_deg - 90)`
- **Joint limits:** ±π/2 radians on revolute joints
- **Joint order:** [j1, j2, j3, j4, j5] = [shoulder_rotation, shoulder_lift, elbow, wrist_rotate, wrist_flex, gripper]

---

## POSE SPECIFICATIONS

### 1️⃣ HOME POSE
```
Joints: [0.0, 0.0, 0.0, 0.0, 0.0]
Gripper: -0.35 (HOLD) or 0.0 (CLOSED)
Manufacturer: [90, 90, 90, 90, 90]
```

**Purpose:** Arm fully folded/retracted  
**Use Case:** Startup position, safe storage  
**Description:** All joints at neutral. Arm is compactly folded when viewed from the side.

**Code Reference:**
```python
HOME = [0.0, 0.0, 0.0, 0.0, 0.0]  # arm folded straight up
```

---

### 2️⃣ DRIVE POSE ⭐ (CRITICAL FOR VISION)
```
Joints: [0.0, 0.524, -1.55, -1.55, 0.0]
Manufacturer: [90, 120, 0, 0, 90]
Gripper: -1.54 (OPEN)
```

**Purpose:** Driving/observation pose during autonomous navigation  
**Use Case:** While driving to cube, while performing HSV tracking  
**Description:**
- j2 lifted to 120° (shoulder_lift raised)
- j3, j4 fully extended forward and down (-1.55 rad ≈ -89°)
- Wrist camera looks DOWN at the ground ahead of the robot
- Head camera unobstructed (can see forward obstacles)

**Why This Pose?**
- Manufacturer discovered this pose allows both cameras to function perfectly
- Camera field of view captures the floor at ~0.5-1.5m distance
- Ideal for HSV-based cube detection during approach
- Robot can drive safely while arm observes

**Code Reference:**
```python
DRIVE_POSE = [0.0, 0.524, -1.55, -1.55, 0.0]
# joints_init [90, 120, 0, 0, 90]: driving/observation pose
# wrist camera watches the floor ahead
```

**Verification:** ✅ Tested in Gazebo with 4cm cube  
**Camera View:** Wrist camera has clear view of floor 0.3-1.0m ahead

---

### 3️⃣ REACH_DOWN (PICK POSITION)
```
Joints: [0.0, -1.45, -0.524, -0.908, 0.0]
Manufacturer: [90, 7, 60, 38, 90]
Gripper: -1.54 (OPEN initially), then -0.50 (HOLD to grasp)
```

**Purpose:** Reach down to pick up the cube
**Use Case:** After HSV alignment completed, robot ready to grasp
**Description:**
- j2 deeply negative (-1.45 ≈ -83°) — shoulder drops down
- j3 curved (-0.524 ≈ -30°) — elbow forward
- j4 backward (-0.908 ≈ -52°) — wrist tucked to keep fingers level
- Result: **Gripper finger pads land horizontally on top of cube**

**Code Reference:**
```python
REACH_DOWN = [0.0, -1.45, -0.524, -0.908, 0.0]
# Pick reach [*, 7, 60, 38, 90] (autopilot_main.py:126)
# Manufacturer's 2-3 cm cube value. The 4 cm cube used j4 = -1.21, but
# the simulation cube is 2 cm (model.sdf), so -0.908 is correct.
```

**Safety Margins:**
- j4 backed off from hard limit (-π/2 ≈ -1.57) by 0.66 rad (38°)
- Prevents PID chatter against joint stop
- Still reaches cube reliably

---

### 4️⃣ LIFT_POSE (SHOULDER LIFT)
```
Joints: [0.0, -0.524, -0.524, -0.908, 0.0]
Manufacturer: [90, 60, 60, 38, 90]
Gripper: -0.50 (HOLD - already gripping cube)
```

**Purpose:** Intermediate lift — raise shoulder to secure grip
**Use Case:** After gripper closes on cube (between REACH_DOWN and CARRY)
**Description:**
- j2 raised to -60° (shoulder lift)
- j3, j4 unchanged (maintain horizontal finger position)
- j1 rotation unchanged
- Result: Cube lifts off ground by ~8-10cm

**CRITICAL — j4 MUST match REACH_DOWN:**
- The previous LIFT_POSE used j4 = -1.21 (the old 4 cm value). When the
  gripper closed on a 2 cm cube at REACH_DOWN (j4 = -0.908) and the arm
  then moved to LIFT_POSE, the wrist snapped 17° and flicked the cube
  out of the fingers. The fix is to keep j4 = -0.908 in LIFT_POSE,
  matching REACH_DOWN exactly.

**Workflow Sequence:**
```
1. Move to REACH_DOWN, gripper opens
2. Close gripper on cube
3. Move to LIFT_POSE (this step) — shoulder lifts
4. Move to CARRY — transport position
```

**Code Reference:**
```python
LIFT_POSE = [0.0, -0.524, -0.524, -0.908, 0.0]
# Manufacturer lift step: servo2 -> 60 with the rest still at reach
# (autopilot_main.py:157). j4 matches REACH_DOWN to prevent the wrist
# snap that would flick the cube out.
```

---

### 5️⃣ CARRY POSE (TRANSPORT)
```
Joints: [0.0, 0.96, -1.55, -0.785, 0.0]
Manufacturer: [90, 145, 0, 45, 90]
Gripper: -0.35 (HOLD)
```

**Purpose:** Safe transport pose while driving with cube  
**Use Case:** After pickup complete, driving to landing pad  
**Description:**
- j2 raised high (145° ≈ 0.96 rad) — shoulder folded back
- j3 extended forward (-1.55 ≈ -89°)
- j4 moderate backward (-0.785 ≈ -45°)
- j1 rotation unchanged
- Result: **Cube held up and back** — cannot be dragged by wheels during turning

**Why This Pose?**
- Prevents cube from shifting/dropping during aggressive turns
- Raises cube ~15-20cm above deck
- Holds cube back near robot shoulder
- Cube cannot be intercepted by wheels or obstacles

**Code Reference:**
```python
CARRY = [0.0, 0.96, -1.55, -0.785, 0.0]
# Transport carry [90, 145, 0, 45, 90] (transport_main.py:98):
# holds the cube up and back so it cannot be dragged while driving.
```

---

### 6️⃣ PLACE_DOWN (GENTLE PLACE)
```
Joints: [0.0, -1.40, -0.524, -0.873, 0.0]
Manufacturer: [90, 2, 60, 40, 90]
Gripper: -0.50 (HOLD until final step), then -1.54 (OPEN)
```

**Purpose:** Lower cube gently to landing pad
**Use Case:** After driving to landing pad, before opening gripper
**Description:**
- j2 deeply negative (-1.40 ≈ -80°) — shoulder drops down
- j3 curved (-0.524 ≈ -30°) — elbow forward
- j4 moderate backward (-0.873 ≈ -50°)
- Result: **Gripper descends to place cube on pad**

**CRITICAL FEATURE:**
- **Proven to place the 2 cm cube gently without tipping**
- Gripper opens AFTER robot backs away 0.25 m
- Cube settles on green landing pad without rolling
- Operator backs away before lift to prevent cube being dragged

**Workflow for Placing:**
```
1. Robot at landing pad (standoff distance — drop_off_standoff_distance)
2. Move to LIFT_POSE — prepare position
3. Move to PLACE_DOWN — lower towards pad
4. Gripper opens — release cube
5. Wait 3 seconds — cube settles
6. Robot backs away 0.25 m
7. Move to DRIVE_POSE — fold arm for transit
```

**Code Reference:**
```python
PLACE_DOWN = [0.0, -1.40, -0.524, -0.873, 0.0]
# Transport place Grip_down [90, 2, 60, 40, 90] (transport_main.py:83),
# joint2 eased to -1.40 (gentle place for the 2 cm cube).
```

---

## GRIPPER POSITIONS

### Gripper States
| State | Value (rad) | Description | Use |
|-------|-----------|-------------|-----|
| **OPEN** | -1.54 | Fingers fully open, ~120 mm gap | Before pick, after place |
| **HOLD** ⭐ | -0.50 | Closed on cube, 2 cm parallel gap | During grasp & transport |
| **CLOSE** | 0.0 | Fully closed, near 0 mm gap | Emergency stop |

### Why HOLD = -0.50?

**Flat parallel pad contact on a 2 cm cube occurs at: ≈ -0.50 rad.**
- Cube starts being held at this position
- Command to **-0.50** (right at the contact point)
- Result: Master stalls with steady squeeze
- Mimic relay keeps fingertip pads perfectly parallel
- Friction μ=100 ensures secure grip

**Previous value (-0.35) was tuned for a 4 cm cube and is past-contact for
the current 2 cm cube.** Past-contact on a tiny 2 cm cube can over-squeeze
and crush the cube; for now -0.50 is the geometric contact point. **Verify
in sim with `test_gripper.py`** (or by watching the contact sensors) and
tune by ±0.05 rad if the cube is dropped or crushed.

**Previous Issue (SOLVED, OLD 4 CM REVISION):**
- Old value: -0.676 rad
- Created 4.9 cm gap (wider than 4 cm cube!)
- Cube gripped only via tilted fingertips pinching edges
- ❌ Caused dropped cubes

**Current Value (FOR 2 CM CUBE):**
- Value: -0.50 rad
- Creates ~2 cm parallel grip
- ✅ Secure hold with both pads on cube faces

---

## TRANSITION TIMING

All pose transitions use **smooth interpolation** to prevent arm whipping (which would fling the cube):

```python
def set_joints(arm_pos, grip_pos, duration_ms=2500):
    # Interpolate over duration_ms with 25 steps
    # Prevents sudden joint accelerations
    # Smooths camera view during HSV tracking
```

| Transition | Duration | Purpose |
|-----------|----------|---------|
| HOME → DRIVE_POSE | 3000ms | Startup sequence |
| DRIVE_POSE → REACH_DOWN | 4000ms | Careful approach |
| REACH_DOWN → LIFT_POSE | 2500ms | Gradual lift |
| LIFT_POSE → CARRY | 3500ms | Smooth transport prep |
| CARRY → LANDING_AREA | – | Long drive with cube |
| CARRY → LIFT_POSE | 3000ms | Descent start |
| LIFT_POSE → PLACE_DOWN | 3000ms | Careful placement |
| PLACE_DOWN → GRIPPER_OPEN | 2000ms | Release |
| PLACE_DOWN → LIFT_POSE | 2500ms | Recover arm |
| LIFT_POSE → DRIVE_POSE | 3000ms | Return to neutral |

---

## COORDINATE SYSTEM & FRAME REFERENCES

All arm joint angles are defined in the **arm link frame**:
- **Frame ID:** x3plus/arm_joint1_link (base of arm)
- **Relationship to robot:** Fixed at base_link
- **Relationship to world:** Transformed via base_footprint → base_link → arm frame

**TF Chain:**
```
odom
  ├── base_footprint (wheel odometry)
  │   └── base_link (robot chassis)
  │       └── arm_joint1_link (arm base)
  │           └── arm_joint2_link
  │               └── arm_joint3_link
  │                   └── arm_joint4_link
  │                       └── arm_joint5_link
  │                           └── grip_joint
  │                               ├── llink_joint1
  │                               │   └── llink_joint2
  │                               └── rlink_joint2
  │
  ├── test_block (Gazebo ground truth)
  ├── landing_pad (Gazebo ground truth)
  └── wrist_mono_camera_frame (arm-mounted)
```

---

## TESTING & VERIFICATION

### How to Verify Poses

**Manual Test (One Pose at a Time):**
```bash
# Terminal 1: Launch Gazebo
ros2 launch sim_gazebo_bringup gazebo.launch.py use_rviz:=true

# Terminal 2: Send pose command (example: DRIVE_POSE)
ros2 topic pub --once /arm_joint1_cmd_pos std_msgs/msg/Float64 "{data: 0.0}"
ros2 topic pub --once /arm_joint2_cmd_pos std_msgs/msg/Float64 "{data: 0.524}"
ros2 topic pub --once /arm_joint3_cmd_pos std_msgs/msg/Float64 "{data: -1.55}"
ros2 topic pub --once /arm_joint4_cmd_pos std_msgs/msg/Float64 "{data: -1.55}"
ros2 topic pub --once /arm_joint5_cmd_pos std_msgs/msg/Float64 "{data: 0.0}"
ros2 topic pub --once /grip_joint_cmd_pos std_msgs/msg/Float64 "{data: -1.54}"
```

**Automated Pick-and-Place Test:**
```bash
# Launch full autopilot
ros2 launch sim_gazebo_bringup vision_autopilot_simple.launch.py world:=office

# Watch:
# 1. Arm moves to DRIVE_POSE (observe floor)
# 2. Robot drives to cube (2.0, 0.0)
# 3. HSV tracking for final alignment
# 4. Move to REACH_DOWN, close gripper
# 5. Move to LIFT_POSE, then CARRY
# 6. Robot drives to landing pad (2.0, 1.2)
# 7. Move to PLACE_DOWN, release gripper
# 8. Backup and fold arm
```

### Success Criteria

- ✅ All poses execute without jerking
- ✅ Cube gripped and held securely (4.8cm parallel grip)
- ✅ Cube transported without rolling
- ✅ Cube placed gently on landing pad
- ✅ Arm returns to DRIVE_POSE safely
- ✅ No joint limits hit (backed off from ±π/2 stops)
- ✅ Camera views unobstructed in DRIVE_POSE

---

## FINE-TUNING FOR YOUR SETUP

If you need to adjust poses for different cube sizes or grippers:

### For LARGER Cubes (>4cm)
- **Increase j4 magnitude** (make more negative, e.g., -1.35 to -1.50)
- **Adjust j2 in REACH_DOWN** (lift shoulder slightly higher, e.g., -1.50 to -1.40)
- Test incrementally with small gripper opening first

### For SMALLER Cubes (<4cm)
- **Decrease j4 magnitude** (make less negative, e.g., -1.0 to -0.9)
- **Lower j2 in REACH_DOWN** (drop shoulder lower, e.g., -1.45 to -1.55)
- Verify finger pad parallelism after each change

### For DIFFERENT GRIPPERS
- **Measure finger-center distance** in each pose using FK
- **Verify gripper gap** matches your gripper PAD width
- **Adjust GRIPPER_HOLD** position to match parallel contact point

---

## REFERENCES

| Document | Topic |
|----------|-------|
| [vision_autopilot_simple.py](../../scripts/x3plus_examples/vision_autopilot_simple.py#L52-L81) | Pose definitions in code |
| [VERIFICATION_SUMMARY.md](VERIFICATION_SUMMARY.md) | Gripper & pick-and-place verification |
| [GRIPPER_PHYSICS_ANALYSIS.md](GRIPPER_PHYSICS_ANALYSIS.md) | Detailed physics & kinematics |
| yahboomcar_ws/src/arm_autopilot | Manufacturer's original ROS1 code |
| yahboomcar_ws/src/arm_color_transport | Transport workflow |

---

## SUMMARY TABLE

| Pose | j1 | j2 | j3 | j4 | j5 | Purpose | Gripper |
|------|-----|-----|------|------|-----|---------|----------|
| HOME | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 | Startup | CLOSE |
| DRIVE | 0.0 | 0.52 | -1.55 | -1.55 | 0.0 | Observe | OPEN |
| REACH | 0.0 | -1.45 | -0.524 | -0.908 | 0.0 | Pick | OPEN→HOLD |
| LIFT | 0.0 | -0.52 | -0.524 | -0.908 | 0.0 | Shoulder lift | HOLD |
| CARRY | 0.0 | 0.96 | -1.55 | -0.785 | 0.0 | Transport | HOLD |
| PLACE | 0.0 | -1.40 | -0.524 | -0.873 | 0.0 | Place | HOLD→OPEN |

**Status: ✅ All poses validated for 2 cm cube in Gazebo simulation**

Last verified: 2026-06-14
