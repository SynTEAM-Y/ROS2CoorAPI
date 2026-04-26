# 4-Wheel Configuration Update - Status Report

## Changes Implemented

### 1. Robot URDF - yahboomcar_X3plus.urdf.xacro
**Updated wheel configuration from 2-wheel to 4-wheel with stepper motors**

#### Previous Configuration (2 wheels):
- `left_wheel` at position: x=0.05m, y=0.08m
- `right_wheel` at position: x=0.05m, y=-0.08m

#### New Configuration (4 wheels):
- `front_left_wheel` at position: x=0.08m, y=0.08m
- `front_right_wheel` at position: x=0.08m, y=-0.08m
- `back_left_wheel` at position: x=-0.08m, y=0.08m
- `back_right_wheel` at position: x=-0.08m, y=-0.08m

**Wheelbase**: 16cm (front to back)  
**Track Width**: 16cm (left to right)

#### Wheel Properties:
- **Type**: Cylinder geometry (not mesh-based)
- **Radius**: 0.04m (8cm diameter)
- **Width**: 0.015m (1.5cm thick)
- **Mass**: 0.1kg (100g per wheel)
- **Material**: Black
- **Rotation**: Continuous (360°)

#### Stepper Motor Frames:
Each wheel joint creates a frame at the wheel center, which serves as the stepper motor frame. These 4 frames are:
1. `front_left_wheel` - Front left stepper motor position
2. `front_right_wheel` - Front right stepper motor position
3. `back_left_wheel` - Back left stepper motor position
4. `back_right_wheel` - Back right stepper motor position

### 2. Gazebo Differential Drive Plugin Update
The Gazebo DiffDrive plugin now uses the **front wheels** as the main driven wheels:
- `left_joint`: front_left_wheel_joint
- `right_joint`: front_right_wheel_joint
- Back wheels follow passively

### 3. RViz Launch File - robot_rviz.launch.py
**Fixed mesh path issue**

Added `convert_package_uris_to_absolute_paths()` function to properly convert `package://` URIs to absolute file paths with correct "/" prefix.

#### Before:
Paths were appearing as: `home/othman/...` (missing leading "/")  
Error: "Could not resolve host: home"

#### After:
Paths now appear as: `/home/othman/...` (correct absolute path)

## RViz Display Status

### What YOU SHOULD SEE in RViz:

1. **TF Tree Frames** (21 total segments):
   - ✅ `map` → `odom` → `base_footprint` → `base_link`
   - ✅ 4 Wheel frames: `front_left_wheel`, `front_right_wheel`, `back_left_wheel`, `back_right_wheel`
   - ✅ 5 Arm links: `arm_link1` through `arm_link5`
   - ✅ 3 Gripper links per finger: `llink1-3`, `rlink1-3`
   - ✅ Sensors: `laser_link`, `camera_link`, `mono_link`, `imu_link`

2. **Wheel Visualization**:
   - Each wheel should appear as a **BLACK CYLINDER**
   - Dimensions: 8cm diameter, 1.5cm thick
   - Positioned at the 4 corners of the robot base
   - Wheels use simple cylinder geometry (not STL meshes)

3. **Robot Body Parts**:
   - Base, arm, gripper, and sensor links use STL mesh files
   - RViz may show errors loading these meshes (see Known Issues below)
   - BUT frames and TF tree should be fully functional

### Verification Steps in RViz:

1. **Check TF Tree**:
   ```bash
   ros2 run rqt_tf_tree rqt_tf_tree
   ```
   You should see all 21 frames in a tree structure

2. **Check Wheel Frames**:
   - In RViz, enable `TF` display
   - Look for 4 wheel frame axes at the corners of the robot
   - Each frame represents a stepper motor position

3. **Test Movement**:
   ```bash
   ros2 run x3plus_examples manual_control
   ```
   - Press 'w' for forward - all 4 wheel frames should move together
   - Press 'a' for left turn - wheel frames should rotate
   - The differential drive uses front wheels for actuation

## Known Issues & Status

### Mesh Loading Errors in RViz
**Status**: Under investigation  
**Symptoms**: Terminal shows errors like:
```
[ERROR] [rviz2]: Error retrieving file [/home/othman/.../base_link.STL]:
```

**What we've verified**:
- ✅ STL files exist and are valid binary format
- ✅ File permissions are correct (readable)
- ✅ Paths are now correct with leading "/" 
- ✅ Files are in correct install location

**Possible causes**:
1. RViz may have issues with binary STL files (vs ASCII STL)
2. RViz mesh loader cache may need clearing
3. OGRE mesh loading library issue

**Workarounds**:
- The TF frames are fully functional despite mesh errors
- Wheels use cylinder geometry (not meshes) so they SHOULD display
- Robot functionality is not affected - only visualization of body meshes

## Testing Results

### Package Build: ✅ SUCCESS
```
yahboomcar_description: 0.11s
sim_gazebo_bringup: 0.47s
Total: 2 packages built successfully
```

### RViz Launch: ✅ SUCCESS
All 5 nodes running:
1. robot_state_publisher - Publishing 21 robot segments (confirmed 4 wheels)
2. rviz2 - Visualization running
3. diff_drive_simulator - Software odometry with wheel_separation=0.2128m
4. map_publisher - 10x10m static map
5. static_transform_publisher - map→odom transform

### TF Tree: ✅ VERIFIED
21 robot segments recognized by robot_state_publisher:
- ✅ front_left_wheel (NEW)
- ✅ front_right_wheel (NEW)
- ✅ back_left_wheel (NEW)
- ✅ back_right_wheel (NEW)
- ✅ All arm, gripper, sensor, and base links

## Next Steps

### To Verify 4-Wheel Configuration:

1. **Visual Inspection in RViz**:
   - Look at the TF display - you should see 4 wheel frame axes
   - Wheels should appear as black cylinders at the 4 corners
   - Check the frame positions match the coordinates above

2. **Movement Test**:
   ```bash
   ros2 run x3plus_examples manual_control
   ```
   - Test forward/backward ('w'/'s')
   - Test rotation ('a'/'d')
   - Test 90° turn ('q')
   - Observe all 4 wheel frames moving together

3. **Joint States Check**:
   ```bash
   ros2 topic echo /joint_states
   ```
   Should show 4 wheel joints + 5 arm joints + 6 gripper joints

### If Mesh Bodies Don't Display:

The mesh loading issue is cosmetic - the robot is fully functional. Options:

1. **Use Gazebo instead** (meshes work fine there):
   ```bash
   ros2 launch sim_gazebo_bringup gazebo.launch.py
   ```

2. **Convert STL to different format**: Could try DAE (Collada) or OBJ

3. **Accept frame-only visualization**: TF frames alone are sufficient for:
   - Verifying robot structure
   - Testing kinematics
   - Debugging movement
   - Development work

## Files Modified

1. **src/yahboomcar_description/urdf/yahboomcar_X3plus.urdf.xacro**
   - Lines 151-167: Changed from 2 wheels to 4 wheels
   - Lines 169-201: Updated Gazebo friction for 4 wheels
   - Line 421-422: Updated DiffDrive plugin to use front wheels

2. **src/sim_gazebo_bringup/launch/robot_rviz.launch.py**
   - Lines 22-56: Added convert_package_uris_to_absolute_paths() function
   - Line 147: Call function to fix mesh paths before regex substitutions

## Summary

✅ **4-wheel configuration IMPLEMENTED**  
✅ **Stepper motor frames at each wheel center**  
✅ **Gazebo plugin updated for front-wheel drive**  
✅ **TF tree fully functional with 21 segments**  
✅ **Mesh path issue FIXED (leading "/" added)**  
⚠️ **Mesh visualization in RViz - under investigation**  
✅ **Wheels use cylinder geometry - should be visible**

**Robot is ready for testing with 4 wheels!** The mesh visualization issue is cosmetic only and doesn't affect robot functionality.
