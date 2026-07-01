# VISION AUTOPILOT OPTIMIZATION SUMMARY

**Date:** 2026-06-12  
**Status:** ✅ **COMPLETE - Ready for Testing**

---

## 🎯 OPTIMIZATION GOALS ACHIEVED

Your three requirements have been fully implemented:

### ✅ Requirement 1: ARM POSES FOLLOW MANUFACTURER SETTINGS
- **Status:** Already implemented + documented
- **Improvement:** Created comprehensive [ARM_POSE_CALIBRATION.md](ARM_POSE_CALIBRATION.md) with:
  - Full explanation of each pose (HOME, DRIVE, REACH_DOWN, LIFT, CARRY, PLACE_DOWN)
  - Manufacturer origin and ROS1→ROS2 conversion
  - **2 cm cube values** (matches the SDF and the manufacturer's original 2-3 cm cubes)
  - Why LIFT_POSE.j4 MUST match REACH_DOWN.j4 (otherwise the wrist snaps mid-lift and flicks the cube out)
  - Timing and transition information
  - Testing procedures and verification checklist

**Key Finding:** The simulation cube is 2 cm (0.02 m, see `models/test_block/model.sdf`).
An earlier revision of this codebase used a 4 cm cube and set j4 = -1.21. The
current REACH_DOWN.j4 is -0.908 (manufacturer's 2-3 cm value), and LIFT_POSE.j4
is now also -0.908 to match it. This is THE critical consistency check for
the pick-and-place transition.

---

### ✅ Requirement 2: SIMPLE AUTOPILOT USES WRIST CAMERA ONLY (NO DEPTH)
- **Status:** Already correctly implemented + reinforced with documentation
- **What This Means:**
  - Uses `/wrist_mono_camera/image_raw` only
  - No dependency on depth camera
  - Depth camera available but unused (safe for future enhancements)
- **Improvements Made:**
  - Added clear comments explaining wrist camera-only design
  - Documented why depth isn't needed for simple autopilot
  - Clarified that HSV detection on wrist camera suffices

**Why This is Better:**
- Simpler, more robust (fewer dependencies)
- Manufacturer's proven workflow (they used mono camera)
- Lower computational overhead
- Easier to debug (one camera stream instead of multiple)

---

### ✅ Requirement 3: GAZEBO LOCATIONS READ AS GPS GROUND TRUTH
- **Status:** Already correctly implemented + improved
- **What This Means:**
  - Blue cube position: Read from TF frame `test_block` (Gazebo ground truth)
  - Green landing pad position: Read from TF frame `landing_pad` (Gazebo ground truth)
  - Robot location: Read from `base_footprint` (odometry via ground truth)
  - Two-stage approach:
    1. **Coarse GPS stage:** Use TF to drive near cube/pad at standoff distance
    2. **Fine HSV stage:** Use camera blob centroid for final alignment

**Workflow:**
```
1. Read test_block position from Gazebo TF → (~2.0, 0.0)
2. Calculate face-aligned pre-approach point
3. Drive to pre-approach point (~0.65m away)
4. Square up robot body to cube
5. HSV camera tracking takes over for final 0.3m
6. Close gripper when blob at calibrated pixel row
7. Pickup complete → read landing_pad from TF → (~2.0, 1.2)
8. Repeat drive + HSV approach to landing pad
9. Place cube and return
```

---

## 📊 CODE IMPROVEMENTS MADE

### 1. HSV Color Thresholds - Reconciliation
**Issue:** Mismatch between yaml file and Python code

**Fix Applied:**
- Updated [config/hsv_colors.yaml](config/hsv_colors.yaml) to match Python values
- Python HSV for blue: [90, 43, 46] → [124, 255, 255]
- Added detailed comments explaining Gazebo lighting adjustments
- Now synchronized: yaml and Python use identical thresholds

```yaml
# Before (inconsistent):
blue:
  lower: [80, 50, 50]      # yaml
  
# After (consistent):
blue:
  lower: [90, 43, 46]      # matches Python code
```

### 2. Enhanced HSV Detection Algorithm
**Improvements to `detect()` method in [vision_autopilot_simple.py](scripts/x3plus_examples/vision_autopilot_simple.py):**

#### a) **Better Blob Filtering**
- Minimum area: 80 px² (small distance or partial occlusion)
- Maximum area: 2500 px² (close range or large view angle)
- Prevents false positives and noise

#### b) **Circularity Check**
- Validates blob shape is roughly circular
- Rejects elongated or distorted shapes
- Formula: circularity = 4π×Area / Perimeter²
- Rejects if circularity < 0.4 (too elongated)

#### c) **Morphological Cleaning**
- Added MORPH_OPEN operation to remove small noise
- Combined CLOSE + OPEN for better mask quality
- More robust to lighting variations

#### d) **Better Documentation**
- Added extensive comments explaining 2 cm cube optimization
- Clarified what HSV ranges mean for Gazebo simulation
- Explained why values differ from manufacturer's original

### 3. Arm Pose Documentation
**File:** [ARM_POSE_CALIBRATION.md](ARM_POSE_CALIBRATION.md) (NEW)

Complete reference including:
- All 6 arm poses with detailed explanations
- Gripper positions and why each value was chosen
- Manufacturer origin for each pose
- **2 cm cube adjustments with technical rationale**
- Transition timing for smooth motion
- Testing procedures
- Coordinate system and frame references
- Fine-tuning guide for different cube sizes

### 4. Enhanced Code Comments
**File:** [vision_autopilot_simple.py](scripts/x3plus_examples/vision_autopilot_simple.py)

Added detailed documentation:
- HSVColorDetector class: Explained manufacturer workflow
- Arm pose definitions: Documented 2 cm cube adjustments
- Wrist camera: Clarified why depth camera isn't used
- Image callback: Explained async processing
- Each pose: Linked to manufacturer origin and optimization rationale

---

## 🔧 PARAMETER TUNING REFERENCE

### HSV Thresholds (for different lighting conditions)

**Current (Gazebo Office World):**
```python
"blue": ((90, 43, 46), (124, 255, 255))  # Gazebo consistent lighting
```

**If Detection Fails:**
1. **Cube not detected** → Hue range too narrow
   - Widen: [85, 43, 46] → [130, 255, 255]
   
2. **False positives** → Too many false blobs
   - Tighten: [95, 50, 50] → [120, 255, 255]
   
3. **Intermittent detection** → Saturation range wrong
   - Adjust S lower: [90, 30, 30] → [124, 255, 255]

**To Recalibrate:**
```bash
ros2 run sim_gazebo_bringup object_detector --ros-args -p calibrate_mode:=true
# Adjust trackbars and save to config/hsv_colors.yaml
```

### Blob Area Thresholds

**Current Settings:**
```python
MIN_AREA = 80      # Small distance or 80% occluded
MAX_AREA = 2500    # Very close or wide angle
```

**For Different Distances:**
- If cube at ~0.3m: MAX_AREA ≈ 1000-1500 px²
- If cube at ~0.5m: MAX_AREA ≈ 800-1200 px²
- If cube at ~1.0m: MIN_AREA ≈ 100-150 px²

### HSV Approach Parameters

**File:** [vision_autopilot_simple.py](scripts/x3plus_examples/vision_autopilot_simple.py) (lines ~350)

```python
# Stopping criteria
self.declare_parameter('hsv_stop_y', 410)    # Pixel row for stop (2 cm cube)
self.declare_parameter('hsv_x_tol', 10)      # Horizontal tolerance (px)

# Speed limits
twist.linear.x = float(np.clip(err_y * 0.002, 0.0, 0.1))   # Forward speed
twist.angular.z = float(np.clip(-err_x * 0.004, -0.25, 0.25))  # Turn speed
```

**Tuning:**
- Lower `hsv_x_tol` for tighter centering (more expensive/slower)
- Increase `hsv_stop_y` to stop earlier, decrease to approach closer
- Speed multipliers (0.002, 0.004) control approach aggressiveness

---

## 🚀 WORKFLOW SUMMARY

### Pick-and-Place Cycle

```
STATE: IDLE
  → Read cube_pose from Gazebo TF (test_block)
  → Transition arm to DRIVE_POSE

STATE: APPROACH_CUBE
  → Calculate face-aligned pre-approach point
  → Drive to pre-approach (0.65m standoff)
  → Move to next state

STATE: FACE_CUBE
  → Square up robot body to face cube
  → Use wrist camera (DRIVE_POSE observation)
  → When aligned, move to next state

STATE: HSV_APPROACH ⭐ SIMPLE AUTOPILOT HERE
  → Blob centroid controls forward/turn speed
  → Drive until blob at pixel row 410 + centered horizontally
  → Stop when blob stable for 3 frames (debounce)
  → Move to PICKUP

STATE: PICKUP
  → Run pickup_sequence (background thread):
    • Move to REACH_DOWN (open gripper)
    • Close gripper on cube
    • Move to LIFT_POSE (shoulder lift)
    • Move to CARRY (transport safe position)

STATE: FIND_LANDING
  → Read landing_pad position from Gazebo TF

STATE: DRIVE_TO_LANDING
  → Same as APPROACH_CUBE but for landing pad
  → Calculate standoff from landing pad

STATE: FACE_LANDING
  → Square up to landing pad

STATE: DROP
  → Run lower_and_release (background thread):
    • Move to LIFT_POSE
    • Move to PLACE_DOWN
    • Open gripper
    • Wait 3 seconds for cube to settle

STATE: BACKUP
  → Reverse 0.25m so cube clears robot deck

STATE: FOLD_WAIT
  → Return arm to DRIVE_POSE

STATE: DONE
  → Task complete!
```

**Total Time:** ~60-90 seconds for full cycle

---

## ✅ VERIFICATION CHECKLIST

Before running full autopilot:

- [ ] Gazebo office world loaded
- [ ] Blue cube at (2.0, 0.0) visible
- [ ] Green landing pad at (2.0, 1.2) visible
- [ ] Wrist camera view shows floor ahead
- [ ] HSV detection blob appears on screen
- [ ] TF frames publishing: test_block, landing_pad, base_footprint
- [ ] Arm moves smoothly (no jerking)
- [ ] Gripper parallel (4.8cm gap when closed)

**Run Full Test:**
```bash
ros2 launch sim_gazebo_bringup vision_autopilot_simple.launch.py world:=office
```

**Expected Behavior:**
1. ✅ 0-15s: Gazebo loading, physics settling
2. ✅ 15-20s: Objects spawning (cube, pad)
3. ✅ 20-30s: TF relays connecting, arm moving to DRIVE_POSE
4. ✅ 30-45s: Coarse GPS approach to cube
5. ✅ 45-55s: HSV final approach and pickup
6. ✅ 55-70s: Drive to landing pad
7. ✅ 70-80s: Place cube on pad
8. ✅ 80-90s: Backup and fold arm

---

## 📁 FILES CREATED/MODIFIED

### Created
- [ARM_POSE_CALIBRATION.md](ARM_POSE_CALIBRATION.md) — Complete arm pose reference
- [VISION_AUTOPILOT_OPTIMIZATION_GUIDE.md](VISION_AUTOPILOT_OPTIMIZATION_GUIDE.md) — This file

### Modified
- [config/hsv_colors.yaml](config/hsv_colors.yaml) — HSV threshold reconciliation
- [scripts/x3plus_examples/vision_autopilot_simple.py](scripts/x3plus_examples/vision_autopilot_simple.py) — Enhanced detection + comments

### Reference
- [VISION_AUTOPILOT_README.md](VISION_AUTOPILOT_README.md) — High-level overview
- [VERIFICATION_SUMMARY.md](VERIFICATION_SUMMARY.md) — Gripper specs & pick-place validation

---

## 🎓 KEY LEARNING OUTCOMES

### What Makes This Autopilot Work

1. **Two-Stage Navigation:**
   - GPS (Gazebo TF) for coarse approach ← Simple but accurate
   - Vision (HSV) for fine alignment ← Robust to calibration errors

2. **Manufacturer-Proven Workflow:**
   - Arm poses from yahboomcar_ws/src/arm_autopilot
   - Pick sequence with parallel gripper ← All documented

3. **2 cm Cube Consistency:**
   - LIFT_POSE.j4 now matches REACH_DOWN.j4 (-0.908). The previous value
     (-1.21) was a holdover from an old 4 cm cube revision and caused the
     wrist to snap 17° during the lift, flicking the cube out of the gripper.
   - GRIPPER_HOLD = -0.50 (was -0.35 for a 4 cm cube) sits at the geometric
     contact point for a 2 cm cube. Past-contact over-squeezes and crushes.
   - Prevents tilt and rotation during transport.

4. **Simplicity by Design:**
   - Single wrist camera (not depth)
   - HSV color detection (not ML/RCNN)
   - Pre-calibrated positions (not real-time IK)
   - Result: Fast, reliable, repeatable

---

## 🔮 FUTURE ENHANCEMENTS

If you want to extend this system:

1. **Depth-Camera Integration**
   - Add 3D cube detection
   - Better distance estimation
   - Grasp point optimization

2. **Dynamic Cube Sizes**
   - Measure cube in real-time
   - Adjust REACH_DOWN j4 dynamically
   - Handle unknown objects

3. **Multiple Cubes**
   - Extend to pick multiple objects
   - Segment and track individual blobs
   - Waypointstack navigation

4. **Real Robot Adaptation**
   - Recalibrate HSV for real lighting
   - Adjust timing (servos vs simulation)
   - Handle sensor noise filtering

---

## 📞 TROUBLESHOOTING QUICK REFERENCE

| Issue | Cause | Solution |
|-------|-------|----------|
| Cube not detected | HSV out of range | Widen HSV range, recalibrate |
| False blob detection | Lighting change | Tighten H/S/V thresholds |
| Gripper misses cube | j4 wrong for cube size | Adjust REACH_DOWN j4 value |
| Cube tips during lift | Gripper angle wrong | Check GRIPPER_HOLD = -0.50 (2 cm cube) |
| Robot overshoots cube | HSV approach too fast | Reduce speed multipliers |
| Cube drops on landing | j2 too high in PLACE_DOWN | Lower j2 to -1.40 |

---

## 📚 DOCUMENTATION FILES

| File | Purpose |
|------|---------|
| [ARM_POSE_CALIBRATION.md](ARM_POSE_CALIBRATION.md) | Complete arm pose reference |
| [VISION_AUTOPILOT_README.md](VISION_AUTOPILOT_README.md) | High-level system overview |
| [VERIFICATION_SUMMARY.md](VERIFICATION_SUMMARY.md) | Gripper verification & specs |
| [GRIPPER_PHYSICS_ANALYSIS.md](GRIPPER_PHYSICS_ANALYSIS.md) | Detailed physics analysis |
| [GRIPPER_QUICK_REFERENCE.md](GRIPPER_QUICK_REFERENCE.md) | Quick commands & checklist |

---

**Status: ✅ READY FOR TESTING**

All three requirements implemented and documented. The vision autopilot is ready to run!

```bash
ros2 launch sim_gazebo_bringup vision_autopilot_simple.launch.py world:=office
```

Good luck! 🚀
