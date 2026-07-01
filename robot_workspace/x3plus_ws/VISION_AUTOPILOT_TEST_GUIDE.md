#!/usr/bin/env python3
"""
VISION AUTOPILOT SIMPLE - TEST & VALIDATION GUIDE
==================================================

This document summarizes the deep analysis of the manufacturer's code
and the fixes applied to make vision_autopilot_simple.py work correctly
with ROS2 + Gazebo simulation.

Author: Copilot (Deep Manufacture Code Analysis)
Date: 2026-06-15
Status: ✅ READY FOR TESTING
"""

# ==============================================================================
# PART 1: MANUFACTURER'S HARDWARE CONFIGURATION (Reverse-Engineered)
# ==============================================================================

"""
From Rosmaster/Sample/9.arm_servo.ipynb Jupyter notebook:

SERVO HARDWARE:
- 6 UART servos controlled via RosMaster board
- Servo 1-5: Range 0-180°, Neutral = 90°
- Servo 6 (Gripper): Range 30-180°, Neutral = 90°

CONVERSION TO ROS2 RADIANS:
    ROS_rad = (servo_deg - 90) * π / 180
    
Examples:
    - Servo 90°  → ROS  0.0 rad    (neutral)
    - Servo 120° → ROS +0.524 rad  (lift shoulder)
    - Servo 60°  → ROS -0.524 rad  (reach down)
    - Servo 30°  → ROS -1.047 rad  (gripper open)
    - Servo 180° → ROS +1.571 rad  (extreme)

ARM POSES (Manufacturer's yahboomcar_autopilot):
    HOME:       [90, 90,  90,  90, 90]  → ROS [0.0, 0.0, 0.0, 0.0, 0.0]
    DRIVE:      [90, 120, 0,   0,  90]  → ROS [0.0, 0.524, -1.571, -1.571, 0.0]
    REACH_DOWN: [90, 7,   60,  38, 90]  → ROS [0.0, -1.450, -0.524, -0.908, 0.0]
    LIFT:       [90, 60,  60,  38, 90]  → ROS [0.0, -0.524, -0.524, -0.908, 0.0]
    CARRY:      [90, 145, 0,   45, 90]  → ROS [0.0, 0.960, -1.571, -0.785, 0.0]
    PLACE_DOWN: [90, 2,   60,  40, 90]  → ROS [0.0, -1.536, -0.524, -0.873, 0.0]

GRIPPER VALUES:
    OPEN:  30° servo  → -1.54 rad  (fingers fully open)
    HOLD:  ~90-100°   → -0.50 rad  (2cm cube contact point)
    CLOSE: 180°       →  0.0 rad   (fully closed)
"""

# ==============================================================================
# PART 2: SIMULATION CONFIGURATION (Current Setup)
# ==============================================================================

"""
CUBE (test_block):
  - Size: 2cm × 2cm × 2cm (0.02 m sides)
  - Mass: 5 grams (light, gripper can lift easily)
  - Color: Blue (HSV hue 90-124 in Gazebo)
  - Spawn location: (2.0, 0.0, 0.02) via spawn_test_object
  - Plugins: PosePublisher (ground-truth position published to /gz_pose_tf)

LANDING PAD:
  - Size: 50cm × 50cm (0.5 m sides)
  - Color: Green (HSV hue 35-77)
  - Spawn location: (2.0, 1.2, 0.001) via spawn_test_object
  - Plugins: PosePublisher (static, position published to /gz_pose_tf)

GRIPPER REACH:
  - Standoff distance: 0.292m (from base_footprint to gripper pad center)
  - At REACH_DOWN, gripper can contact 2cm cube
  - Pre-approach distance: 0.65m (robot stops here, arm does final approach)

HSV DETECTION (Wrist Camera):
  - Camera: Mono 640×480 (no depth)
  - Blue blob area: 15-400 px² (at approach distance)
  - Stop criterion: blob Y-row ≤ 410 AND blob centered (±10 px from center)
  - Used ONLY in final 0.3m approach (HSV_APPROACH state)
"""

# ==============================================================================
# PART 3: CRITICAL FIXES APPLIED (2026-06-15)
# ==============================================================================

"""
ISSUE #1: Uninitialized backup_start variable
  - Caused: AttributeError if BACKUP state entered unexpectedly
  - Fixed: Now initialized in __init__ as None, reset in RELEASE_WAIT

ISSUE #2: No TF timeout detection
  - Cause: If gazebo_pose_tf_relay crashes, robot silently waits in IDLE
  - Fixed: Added idle_tf_timeout counter, warns every 5 seconds if TF missing
  - Benefit: User knows immediately if relay nodes died

ISSUE #3: No grasp verification
  - Cause: Gripper closes on empty air, robot places empty gripper on pad
  - Fixed: After pickup, verify cube_pose.z > 0.10m (cube lifted)
  - Retry: If grasp fails, automatically retry pickup sequence
  - Benefit: Never places empty gripper

ISSUE #4: Minimal state logging
  - Cause: Impossible to debug where system got stuck
  - Fixed: Every state transition logged with [STATE_NAME] prefix
  - States: 12+ state transitions now visible in logs
  - Benefit: Clear visibility into state machine flow

ISSUE #5: No error context in exception handling
  - Cause: TF errors silently ignored
  - Fixed: All exceptions logged at DEBUG level with context
  - Benefit: Easier troubleshooting when integrating new nodes
"""

# ==============================================================================
# PART 4: HOW TO TEST (Step-by-Step)
# ==============================================================================

"""
PREREQUISITE: Build the package
    cd /home/othman/ROS2CoorAPI/robot_workspace/x3plus_ws
    colcon build --packages-select sim_gazebo_bringup --symlink-install
    source install/setup.bash

LAUNCH THE TEST:
    # Option 1: Use the test script
    bash /home/othman/ROS2CoorAPI/robot_workspace/x3plus_ws/test_vision_autopilot.sh
    
    # Option 2: Manual launch
    ros2 launch sim_gazebo_bringup vision_autopilot_simple.launch.py world:=empty

EXPECTED SEQUENCE (Timing):
    t=0s:    Gazebo starts, bridge connects
    t=2-3s:  ROS services ready
    t=15s:   Cube spawned at (2.0, 0.0)
    t=16s:   TF relay for cube activated → /tf: odom→test_block available
    t=17s:   Landing pad spawned at (2.0, 1.2)
    t=18s:   TF relay for landing pad → /tf: odom→landing_pad available
    t=25s:   Vision autopilot node starts → checks for cube TF
    t=26-30s: IDLE → ARM_TO_DRIVE (arm moves to camera pose)
    t=30-35s: APPROACH_CUBE (drives 0.65m to pre-approach point)
    t=35-40s: FACE_CUBE (rotates to face cube)
    t=40-60s: HSV_APPROACH (vision-guided final approach, 0.3m)
    t=60-65s: PICKUP (gripper closes on cube)
    t=65-70s: PICKUP_WAIT (verifies cube lifted)
    t=70-75s: FIND_LANDING (reads landing pad TF)
    t=75-85s: DRIVE_TO_LANDING (drives to (2.0, 1.2))
    t=85-90s: FACE_LANDING (rotates to face pad)
    t=90-95s: DROP (gripper releases cube on pad)
    t=95-100s: BACKUP (reverses 0.25m)
    t=100-105s: FOLD_WAIT (arm returns to drive pose)
    t=105s:  DONE - ✅ PICK & PLACE COMPLETED!

MONITOR THE LOGS:
    Watch for:
    ✅ "[IDLE] Cube found at (2.00, 0.00)"
    ✅ "[ARM_TO_DRIVE] Complete, moving to APPROACH_CUBE"
    ✅ "[APPROACH_CUBE] Reached pre-approach point"
    ✅ "[HSV_APPROACH] Stop: blob at (320, ...)"
    ✅ "[PICKUP_WAIT] ✓ Cube lifted successfully"
    ✅ "[FIND_LANDING] Landing pad located"
    ✅ "✓ PICK AND PLACE COMPLETED SUCCESSFULLY!"
    
    ❌ Errors to look for:
    ❌ "[IDLE] Waiting for cube TF... Check if gazebo_pose_tf_relay running!"
       → gazebo_pose_tf_relay node didn't start, check launch file
    ❌ "[PICKUP_WAIT] ✗ Grasp verification failed"
       → Gripper didn't close on cube, check gripper parameters
    ❌ "[FIND_LANDING] Waiting for landing pad TF..."
       → landing_pad relay didn't start
"""

# ==============================================================================
# PART 5: PARAMETER TUNING GUIDE
# ==============================================================================

"""
If pick-and-place fails, tune these parameters in vision_autopilot_simple.launch.py:

1. standoff_distance (default: 0.292m)
   - Distance from robot base to gripper at REACH_DOWN
   - Too small: gripper clips ground before cube
   - Too large: gripper misses cube
   - Test: Run gripper at REACH_DOWN, measure actual reach

2. hsv_stop_y (default: 410)
   - Pixel row where blob should be at pickup distance
   - Too high (>410): robot stops too far from cube
   - Too low (<410): robot drives too close, gripper clips ground
   - Calibrate: Move arm to REACH_DOWN, measure pixel row of cube top

3. hsv_x_tol (default: 10)
   - Horizontal pixel tolerance for blob centering
   - Too small: hard to converge on center
   - Too large: gripper approaches off-center
   - Start with 10, adjust if misses consistently left/right

4. pre_approach_distance (default: 0.65m)
   - Distance for face-aligned approach before HSV takeover
   - Too small: Less coarse navigation, more HSV work
   - Too large: GPS approach takes longer
   - Should be: standoff_distance * 2-3

5. approach_speed (default: 0.30 m/s)
   - Maximum drive speed during navigation
   - Too fast: Overshoot, poor accuracy
   - Too slow: Takes forever to test
   - Reasonable range: 0.1-0.5 m/s
"""

# ==============================================================================
# PART 6: KNOWN LIMITATIONS & FUTURE IMPROVEMENTS
# ==============================================================================

"""
CURRENT LIMITATIONS (Simulation-Specific):
1. Ground-truth TF from Gazebo
   - NOT portable to real robots without AprilTag fiducials
   - Full autopilot (vision_autopilot.launch.py) uses vision-only approach
   
2. HSV detection only in final 0.3m
   - Relies on GPS-like TF for coarse navigation
   - Real robots would need camera-based fiducial detection
   
3. No force feedback on gripper
   - Grasp verification only checks height (cube lifted)
   - Real robot needs force sensor or vacuum pressure sensor

4. No dynamic reconfigure
   - Parameters hardcoded at launch time
   - Can't tune while running (must kill and relaunch)

PLANNED IMPROVEMENTS:
- Add force-based grasp verification (when force sensor available)
- Add dynamic parameter tuning via ROS2 parameter callbacks
- Add AprilTag fiducial detection for real-robot portability
- Add emergency stop (E-stop) safety check
- Add timeout limits for each state (max 30s per state?)
- Add telemetry logging to CSV for performance analysis
"""

# ==============================================================================
# PART 7: TROUBLESHOOTING CHECKLIST
# ==============================================================================

"""
PROBLEM: "Stuck in IDLE, waiting for cube TF"
  □ Check: ros2 topic list | grep gz_pose
     → Should show /gz_pose_tf (from ros_gz_bridge)
  □ Check: ros2 topic list | grep test_block  
     → Should show transform on /tf
  □ Fix: Restart gazebo_pose_tf_relay nodes
     ros2 run sim_gazebo_bringup gazebo_pose_tf_relay --ros-args \
       -p source_child:=test_block -p child_frame:=test_block

PROBLEM: "Gripper fails to pick (grasp verification fails)"
  □ Check: ros2 echo /joint_states | grep grip_joint
     → Should see grip_joint value change from -1.54 → -0.50
  □ Check: ros2 echo /gz_pose_tf | grep test_block
     → After pickup, z-position should be > 0.10m
  □ Fix: Verify gripper is properly connected in URDF
     → Check grip_joint controller is running

PROBLEM: "Robot overshoots cube, misses during HSV approach"
  □ Cause: hsv_stop_y calibrated for wrong distance
  □ Fix: Recalibrate hsv_stop_y
     1. Position robot at standoff distance (0.292m from cube)
     2. Move arm to REACH_DOWN
     3. Take wrist camera image, find cube pixel row
     4. Update hsv_stop_y = measured_pixel_row
  □ Also check: blob area bounds (15-400 px²) appropriate for your cube

PROBLEM: "Lands far from landing pad (drops on ground beside pad)"
  □ Cause: drop_off_standoff_distance incorrect
  □ Fix: Recalibrate at landing pad location
     1. Position robot at landing pad
     2. Measure actual distance from gripper center to pad center
     3. Update drop_off_standoff_distance = measured_distance

PROBLEM: "Gazebo crashes or runs very slow"
  □ Check system performance: top, htop
  □ Try: Disable GUI (use headless mode)
     ros2 launch sim_gazebo_bringup gazebo.launch.py gui:=false
  □ Try: Reduce Gazebo physics update rate in world SDF
     <max_step_size>0.01</max_step_size>  (default is 0.001)
"""

# ==============================================================================
# PART 8: SUCCESS CRITERIA
# ==============================================================================

"""
The vision_autopilot_simple system is working correctly if:

✅ Logs show all state transitions: IDLE → ARM_TO_DRIVE → ... → DONE
✅ Robot successfully picks cube (grasp verification passes)
✅ Robot successfully places cube on landing pad
✅ No error messages about missing TF frames
✅ Total execution time: ~100-120 seconds from node startup
✅ Cube ends up on landing pad (not on floor beside it)
✅ No crashes or deadlocks

Partial success (to debug):
⚠️ Reaches APPROACH_CUBE but overshoots: Tune hsv_stop_y
⚠️ Grasp fails (cube not lifted): Check gripper controller
⚠️ Drops off-pad: Tune drop_off_standoff_distance  
⚠️ Arm vibrates violently: Check joint PID gains in URDF
"""

# ==============================================================================
# NEXT STEPS
# ==============================================================================

"""
1. RUN THE TEST
   bash /home/othman/ROS2CoorAPI/robot_workspace/x3plus_ws/test_vision_autopilot.sh

2. MONITOR THE OUTPUT
   - Watch for state transitions in logs
   - Check for any error messages
   - Note timing (should take ~100s total)

3. VALIDATE RESULTS
   - Did robot pick the cube? (check Gazebo visualization)
   - Did robot place on landing pad?
   - Are there any grasp verification failures?

4. TUNE IF NEEDED
   - Adjust parameters based on results
   - Re-run test to validate improvements

5. DOCUMENT FINDINGS
   - Record actual hsv_stop_y value from camera
   - Note any timing differences from expected
   - Share results with team

Good luck! 🚀
"""
