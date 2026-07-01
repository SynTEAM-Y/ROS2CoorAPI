# Gripper Calibration Test for 2cm Cube

## Purpose

The `test_gripper_calibration.py` script systematically tests different gripper joint positions to find the optimal `GRIPPER_HOLD` value for the 2cm × 2cm × 2cm test cube.

## Problem Context

The vision_autopilot_simple has been failing because the gripper isn't making contact with the 2cm cube. The current `GRIPPER_HOLD = -0.50` rad appears to be too open. This script determines the correct value.

## How to Run

### Quick Start

```bash
cd ~/ROS2CoorAPI/robot_workspace/x3plus_ws
source install/setup.bash
colcon build --packages-select sim_gazebo_bringup --symlink-install
ros2 launch sim_gazebo_bringup test_gripper_calibration.launch.py
```

The test will:
1. Start Gazebo with empty world (t=0s)
2. Spawn the 2cm test cube at (2.0, 0.0, 0.03) (t=15s)
3. Setup TF relays (t=16s)
4. Run calibration tests (t=20s onward)

### Manual Launch (if you prefer more control)

```bash
# Terminal 1: Start Gazebo infrastructure
ros2 launch sim_gazebo_bringup vision_autopilot_simple.launch.py world:=empty

# Terminal 2: Run calibration only (after infrastructure is ready)
sleep 20  # Wait for Gazebo to stabilize
ros2 run sim_gazebo_bringup test_gripper_calibration
```

## What the Script Does

### Test Sequence

1. **Initialization (t=0-35s)**: Wait for Gazebo and object spawning
2. **Arm Positioning (t=35s)**: Move arm to `REACH_DOWN` pose (gripper above cube)
3. **Grip Tests (t=40s onward)**: For each grip value from `-1.54` (open) to `0.0` (closed):
   - Close gripper at the test value
   - Wait 1.5 seconds for gripper to settle
   - Check if cube lifted to z > 0.10m
   - Record result
   - Open gripper and move to next value

### Test Range

- **Step size**: 0.05 rad (~2.9°)
- **Open limit**: -1.54 rad (fully open)
- **Close limit**: 0.0 rad (fully closed)
- **Total steps**: ~31 tests

### Expected Output

```
================================================================================
GRIPPER CALIBRATION RESULTS FOR 2CM CUBE
================================================================================

Grip Value (rad)     Servo Angle (°)      Cube Z (m)           Status         
---------------------————————————————————————————————————————————————————————————
-1.540               -178.2               0.00997              ✗ NOT LIFTED
-1.490               -176.3               0.00998              ✗ NOT LIFTED
...
-0.350               -110.0               0.15234              ✓ PICKED
-0.300               -108.2               0.16891              ✓ PICKED
-0.250               -106.3               0.17245              ✓ PICKED
...

—————————————————————————————————————————————————————————————————————————————————

🎯 CALIBRATION RESULT:
   Optimal GRIPPER_HOLD = -0.35 rad
   Servo angle: -110.0°
   Cube lifted to: 0.15234 m

📋 UPDATE vision_autopilot_simple.py:
   GRIPPER_HOLD = -0.35
```

## Interpreting Results

### Success Indicators
- ✓ **Multiple consecutive successes**: The gripper can reliably pick up the cube
- Example: If values -0.35, -0.30, -0.25 all show cube lifted

### Failure Patterns

1. **All results z ≈ 0.01m (not lifted)**
   - Gripper is too open across entire range
   - Problem: REACH_DOWN arm pose (j4 angle) may still be wrong
   - Solution: Review j4 angle in REACH_DOWN pose

2. **Transitions from failure to success**
   - Example: -0.40 fails, -0.35 succeeds
   - Use the **least closed** successful value (most open)
   - This gives margin for gripper wear and variations

3. **All results > 0.10m (too successful)**
   - Gripper closing too aggressively
   - May crush the cube in simulation
   - Unlikely but indicates a different problem

## Updating vision_autopilot_simple.py

Once calibration completes, update the GRIPPER_HOLD value:

```python
# OLD (doesn't work for 2cm cube)
GRIPPER_HOLD  = -0.50

# NEW (calibrated result from test)
GRIPPER_HOLD  = -0.35   # ← Example; use your actual result
```

### Full Update Steps

1. Note the optimal value from test output
2. Edit `scripts/x3plus_examples/vision_autopilot_simple.py`
3. Find line ~183: `GRIPPER_HOLD = -0.50`
4. Replace with calibrated value
5. Rebuild: `colcon build --packages-select sim_gazebo_bringup --symlink-install`
6. Re-test pick-and-place: `ros2 launch sim_gazebo_bringup vision_autopilot_simple.launch.py`

## Troubleshooting

### "No successful grip values found!"

**Cause**: Gripper never lifts cube across entire test range

**Check 1**: Verify REACH_DOWN pose
```python
# In vision_autopilot_simple.py around line 150
REACH_DOWN  = [0.0,  -1.45,  -0.524, -1.40,  0.0]
#              j1    j2      j3      j4      j5
```
- j4 = -1.40 should position gripper above cube
- If too shallow, cube won't fit; if too deep, gripper overshoots

**Check 2**: Verify test block is spawned
```bash
ros2 topic list | grep gz_pose_tf
# Should show: /gz_pose_tf
ros2 topic echo /gz_pose_tf 2>/dev/null | grep "test_block" | head -5
# Should show test_block entries
```

**Check 3**: Verify gripper moves
- Open RViz while test is running
- Add RobotModel to display
- Observe if gripper opens/closes at each step
- Check `/joint_states` for `grip_joint` values changing

### Test runs but produces garbage z values

**Cause**: TF relay for test_block not active

**Solution**: Ensure gazebo_pose_tf_relay is running
```bash
ros2 node list | grep tf_relay
# Should show: /test_block_tf_relay, /x3plus_tf_relay
```

### Test completes but motor/gripper seems stuck

**Cause**: PID controller needs tuning or joint hit limit

**Check**: ROS errors
```bash
ros2 topic echo /joint_states | grep grip_joint
# Values should change smoothly from test start
```

## Script Internals

### Key Classes

- **GripperCalibrator**: Main ROS2 node
  - Manages test sequence state machine
  - Publishes joint commands
  - Reads cube pose from TF
  - Records results

### Key Methods

| Method | Purpose |
|--------|---------|
| `publish_arm_pose()` | Send arm+gripper joint targets |
| `get_cube_pose()` | Query cube z-position from TF |
| `sim_sleep()` | Sleep in simulation time (respects /clock) |
| `timer_callback()` | Main state machine tick (100 Hz) |

### State Machine

```
INIT 
  → REACH_DOWN_SETTLE 
  → TEST_NEXT_GRIP ↔ GRIP_CLOSE → GRIP_CHECK ↶
  → PRINT_RESULTS 
  → DONE
```

## Next Steps After Calibration

1. Update GRIPPER_HOLD value in vision_autopilot_simple.py
2. Rebuild package
3. Run full pick-and-place test: `ros2 launch sim_gazebo_bringup vision_autopilot_simple.launch.py`
4. Verify cube is picked, transported, and placed successfully

## References

- [vision_autopilot_simple.py](../scripts/x3plus_examples/vision_autopilot_simple.py) - Main autopilot code
- [GRIPPER_PHYSICS_ANALYSIS.md](../GRIPPER_PHYSICS_ANALYSIS.md) - Gripper mechanism details
- [GRIPPER_QUICK_REFERENCE.md](../GRIPPER_QUICK_REFERENCE.md) - Gripper pose values

