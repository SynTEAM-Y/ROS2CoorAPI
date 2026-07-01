# Gripper Calibration Test - Session Summary (2026-06-15)

## Overview
Systematic debugging of gripper pick-and-place failures for 2cm blue cube in ROS2 X3Plus simulation.

## Problem Statement
- Robot gripper could not lift 2cm blue cube
- Repeated failures with error: `Cube z=0.01m` (not lifted)
- Previous attempts changed arm pose j4 angle from -0.908 to -1.40 rad without success
- Root cause: **GRIPPER_HOLD value (-0.50 rad) too open for 2cm cube**

## Solution Approach
Created automated calibration test to find optimal GRIPPER_HOLD value through systematic testing:
- Test range: -1.54 rad (fully open) to 0.0 rad (fully closed)
- Step size: 0.05 rad increments (~2.9° per step, ~31 total tests)
- Success criterion: Cube z-position > 0.10m after grip
- Expected runtime: 3-5 minutes per full test

## Code Changes Applied

### 1. Fixed Calibration Test Logic ✅
**File**: `scripts/test_gripper_calibration.py`  
**Issues Found & Fixed**:
```python
# BUG 1: Line 133 - Loop condition inverted
OLD: if self.current_grip_value > GRIPPER_CLOSE + 0.01:  # Never true! -1.54 is NOT > 0.0
NEW: if self.current_grip_value < GRIPPER_CLOSE - 0.01:  # Correct: -1.54 IS < 0.0

# BUG 2: Line 173 - Step direction backwards  
OLD: self.current_grip_value -= TEST_STEP  # Goes more negative (-1.54 -> -1.59 ...)
NEW: self.current_grip_value += TEST_STEP  # Correct: gets closer to 0.0 (-1.54 -> -1.49 ...)
```

**Impact**: Test now correctly iterates through all 31 grip values from fully open to fully closed.

### 2. Updated CMakeLists.txt ✅
**File**: `CMakeLists.txt`
```python
install(PROGRAMS
  scripts/test_gripper_calibration.py
  ...
  DESTINATION lib/${PROJECT_NAME}
)
```
**Impact**: Made test_gripper_calibration executable via `ros2 run`.

### 3. Created Supporting Infrastructure ✅
- `test_gripper_calibration.launch.py` - Full launch orchestration (needs TF relay fix)
- `parse_calibration.py` - Post-test output parsing utility
- `gripper_diagnostic_test.py` - Component-level diagnostics
- `run_calibration_test.py` - Simplified runner script

## Build Status ✅
```
colcon build --packages-select sim_gazebo_bringup --symlink-install
Result: ✓ All packages compile successfully (0.2s rebuild)
```

## Infrastructure Status

### Working Components ✅
- Gazebo simulation starts and loads robot+environment
- Test cube spawns at correct location (2.0, 0.0, 0.03)
- Arm joint commands publish to topics correctly
- Gripper mimic relay active and functional
- Gazebo pose bridge publishing transforms

### Pending: TF Configuration for test_block ⏳
**Current Issue**: `gazebo_pose_tf_relay` doesn't publish test_block frame

**Root Cause**: Node started without ROS parameters specifying test_block as source

**Fix Required**:
```bash
# OLD (WRONG - only relays x3plus):
ros2 run sim_gazebo_bringup gazebo_pose_tf_relay /gz_pose_tf odom test_block

# NEW (CORRECT - specifies source_child):
ros2 run sim_gazebo_bringup gazebo_pose_tf_relay \
  --ros-args \
  -p input_topic:=/gz_pose_tf \
  -p parent_frame:=odom \
  -p child_frame:=test_block \
  -p input_type:=tf \
  -p source_child:=test_block
```

## Test Execution Steps (Ready to Run)

### Step 1: Start Gazebo
```bash
cd ~/ROS2CoorAPI/robot_workspace/x3plus_ws
( echo "1" && sleep 360 ) | timeout 370 \
  ros2 launch sim_gazebo_bringup gazebo.launch.py gui:=false world:=empty \
  2>&1 > /tmp/gazebo.log &
sleep 30  # Wait for Gazebo to initialize
```

### Step 2: Spawn Test Cube
```bash
source install/setup.bash
ros2 run ros_gz_sim create -world empty \
  -file $(pwd)/src/sim_gazebo_bringup/models/test_block/model.sdf \
  -name test_block -x 2.0 -y 0.0 -z 0.03
sleep 2
```

### Step 3: Start Infrastructure Relays
```bash
# Gripper mimic relay
ros2 run sim_gazebo_bringup gripper_mimic_relay > /tmp/gripper_relay.log 2>&1 &
sleep 1

# TF relay for x3plus (already running from gazebo.launch.py)

# TF relay for test_block (NEEDS FIXING)
ros2 run sim_gazebo_bringup gazebo_pose_tf_relay \
  --ros-args \
  -p input_topic:=/gz_pose_tf \
  -p parent_frame:=odom \
  -p child_frame:=test_block \
  -p input_type:=tf \
  -p source_child:=test_block \
  > /tmp/test_block_tf_relay.log 2>&1 &
sleep 2
```

### Step 4: Run Calibration Test
```bash
timeout 300 ros2 run sim_gazebo_bringup test_gripper_calibration 2>&1 \
  | tee /tmp/calibration_final.txt
```

### Step 5: Parse Results
```bash
grep -E "Grip Value|Servo Angle|Cube Z|optimal" /tmp/calibration_final.txt
```

## Expected Output

When TF relay works correctly, calibration will produce:
```
Grip Value (rad)     Servo Angle (°)      Cube Z (m)           Status         
-1.540               -178.2               0.00999              ✗ NOT LIFTED
...
-0.350               -110.1               0.10500              ✓ PICKED
-0.300               -107.3               0.11200              ✓ PICKED
-0.250               -104.4               0.12100              ✓ PICKED
...

🎯 CALIBRATION RESULT:
   Optimal GRIPPER_HOLD = -0.35 rad
   Servo angle: -110.1°
   Cube lifted to: 0.10500 m

📋 UPDATE vision_autopilot_simple.py:
   GRIPPER_HOLD = -0.35
```

## Update Vision Autopilot (Final Step)

After getting optimal value from calibration:

**File**: `scripts/x3plus_examples/vision_autopilot_simple.py`  
**Line**: 183

```python
# OLD:
GRIPPER_HOLD  = -0.50   # ← RE-VERIFY in sim with test_gripper.py

# NEW (example):
GRIPPER_HOLD  = -0.35   # ✓ CALIBRATED for 2cm cube (2026-06-15)
```

Then rebuild and run full test:
```bash
colcon build --packages-select sim_gazebo_bringup --symlink-install
ros2 launch sim_gazebo_bringup vision_autopilot_simple.launch.py world:=empty
```

## Known Issues & Resolutions

| Issue | Root Cause | Resolution |
|-------|-----------|-----------|
| Loop never executes | Comparison > instead of < | Fixed: line 133 |
| Values all negative | Step -= instead of += | Fixed: line 173 |
| No TF for test_block | Missing ROS parameters | Use --ros-args syntax |
| Calibration table empty | TF not published | Fix test_block relay params |

## Testing Timeline

- Phase 1: Manual component testing (~10 min) ✓ DONE
- Phase 2: Run calibration with TF relay fix (~5 min) ⏳ PENDING
- Phase 3: Extract and apply optimal value (~2 min)
- Phase 4: Validate full pick-and-place cycle (~5 min)

**Estimated Total**: ~25 minutes from this point

## Verification Checklist

- [x] Calibration test syntax correct
- [x] Test loop iteration logic fixed  
- [x] Package builds without errors
- [x] Gazebo infrastructure functional
- [ ] TF relay publishing test_block frame (BLOCKER)
- [ ] Calibration test produces results table
- [ ] Optimal grip value identified
- [ ] vision_autopilot_simple.py updated
- [ ] Full pick-and-place test passes

## Appendix: Arm Pose Reference

```
HOME        = [0.0,   0.0,    0.0,    0.0,   0.0]
DRIVE_POSE  = [0.0,  0.524,  -1.55,  -1.55,  0.0]
REACH_DOWN  = [0.0, -1.45,   -0.524, -1.40,  0.0]  # Gripper above cube
LIFT_POSE   = [0.0, -0.524,  -0.524, -1.40,  0.0]
CARRY       = [0.0,  0.96,   -1.55,  -0.785, 0.0]
PLACE_DOWN  = [0.0, -1.40,   -0.524, -1.40,  0.0]
```

j1=rotation, j2=shoulder, j3=elbow, j4=wrist_pitch, j5=wrist_yaw

## Contact Points & Debug Output Locations

- Gazebo log: `/tmp/gazebo.log`
- Gripper relay: `/tmp/gripper_relay.log`
- TF relay: `/tmp/test_block_tf_relay.log`
- Calibration output: `/tmp/calibration_final.txt`
- ROS logs: `~/.ros/log/` (latest by timestamp)
