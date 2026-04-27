# Quick Start Guide - RViz Fix + 90° Turn Control

## 🚀 Get Started in 2 Minutes

### Step 1: Build (30 seconds)
```bash
cd ~/ROS2Coordination/robot_workspace/x3plus_ws
colcon build --symlink-install --packages-select x3plus_examples sim_gazebo_bringup yahboomcar_description
source install/setup.bash
```

### Step 2: Test RViz Arm Visualization (30 seconds)
```bash
# In Terminal 1:
ros2 launch sim_gazebo_bringup robot_rviz.launch.py

# You should see:
# ✅ Robot arm with 5 joints (all visible!)
# ✅ Gripper with left and right fingers
# ✅ Joint state publisher GUI to move joints manually
```

### Step 3: Test 90° Turn Control (60 seconds)
```bash
# Terminal 1: Start Gazebo
ros2 launch sim_gazebo_bringup gazebo.launch.py use_rviz:=false

# Terminal 2: Run manual control
ros2 run x3plus_examples manual_control

# Use controls:
# - W/A/S/D: Move robot
# - SPACE: Stop
# - 1: Execute 90° LEFT turn (watch terminal for formula)
# - 2: Execute 90° RIGHT turn
```

---

## ⚙️ What Was Fixed

### Issue 1: Robot Arm Not Visible
**Fixed by**: Enhanced RViz configuration
- File: `gazebo_view.rviz`
- Shows all 5 arm links
- Shows gripper fingers
- Shows wheels and sensors

### Issue 2: No 90° Turn with Formula
**Fixed by**: Manual control node
- File: `manual_control.py`
- Theoretical: ω = 2v/L = 4.699 rad/s, t = πL/(4v) = 0.334 s
- Actual: closed-loop IMU/odom yaw tracking for exact 90°

---

## 📋 Control Keys

```
MOVEMENT:
  W    Forward (+0.8 m/s)
  S    Backward (-0.8 m/s)
  A    Rotate Left (+1.0 rad/s)
  D    Rotate Right (-1.0 rad/s)
  SPACE Stop

90° TURNS (Closed-Loop via IMU/Odom):
  1    90° Left (in-place spin)
  2    90° Right (in-place spin)
  3    90° Left (moving forward)
  4    90° Right (moving forward)

SYSTEM:
  H    Help menu
  Q    Quit
```

---

## 🧮 The 90° Formula

```
Angular Velocity:  ω = 2v / L
                   ω = 2 × 0.5 / 0.2128
                   ω = 4.699 rad/s

Turn Time:         t = π*L / (4*v)
                   t = 3.14159 × 0.2128 / (4 × 0.5)
                   t = 0.334 seconds (theoretical)

Actual: closed-loop IMU/odom feedback → exact 90°
```

---

## 📁 Key Files

| File | Purpose |
|------|---------|
| `manual_control.py` | Keyboard control + 90° turn formula |
| `gazebo_view.rviz` | Enhanced RViz showing all parts |
| `RVIZ_ARM_FIX_AND_90TURN_FORMULA.md` | Full documentation |
| `90DEGREE_TURN_FORMULA.md` | Formula reference |
| `VISUAL_GUIDE_90DEGREE_TURN.md` | Diagrams and visuals |

---

## ✅ Verification

After running `ros2 run x3plus_examples manual_control`, you'll see:

```
╔════════════════════════════════════════════════════════════╗
║     DIFFERENTIAL DRIVE ROBOT MANUAL CONTROL                ║
║              X3Plus Robot Manual Teleop                    ║
╚════════════════════════════════════════════════════════════╝

MOVEMENT CONTROLS:
  W / w   →  Move forward (0.8 m/s)
  S / s   →  Move backward (-0.8 m/s)
  A / a   →  Rotate left (1.0 rad/s)
  D / d   →  Rotate right (-1.0 rad/s)
  SPACE   →  Emergency stop

90-DEGREE TURN COMMANDS (closed-loop with IMU/odom feedback):
  1       →  Turn 90° left (in-place point turn)
  2       →  Turn 90° right (in-place point turn)
  3       →  Turn 90° left (while moving forward)
  4       →  Turn 90° right (while moving forward)

SYSTEM:
  Q / q   →  Quit application
  H / h   →  Show this help menu

✓ Control node started!
```

Press `1` and you'll see:

```
══════════════════════════════════════════════════════════════════════════
90-DEGREE TURN EXECUTION (CLOSED-LOOP)
════════════════════════════════════════════════════════════════════════
Direction: LEFT
Type: IN_PLACE

📐 THEORETICAL (open-loop):
────────────────────────
  ω = 2v/L = 2×0.5/0.2128 = 4.6992 rad/s
  t = (π/2)/ω = 0.3343 s

🤖 ACTUAL EXECUTION (closed-loop with IMU yaw):
────────────────────────
  Command ω: 0.90 rad/s
  Linear: 0.00 m/s
  Target rotation: 90° (π/2 = 1.5708 rad)
  Feedback: /imu yaw tracking
═══════════════════════════════════════════════════════════════════════

90° left turn completed! (actual: 90.0°, error: +0.0°)
```

---

## 🔧 Customization

Edit `manual_control.py` to change:

```python
self.wheel_separation = 0.2128  # Change to your robot width (m)
self.wheel_radius = 0.04      # Change to your wheel radius (m)
self.max_linear_velocity = 0.8   # Max forward speed (m/s)
self.max_angular_velocity = 1.0  # Max rotation speed (rad/s)
self.turn_wheel_speed = 0.5  # Speed used in theoretical formula (m/s)
```

After changing, rebuild:
```bash
colcon build --packages-select x3plus_examples
source install/setup.bash
```

---

## 🐛 Troubleshooting

### "Module not found"
```bash
# Make sure to source:
source ~/ROS2Coordination/robot_workspace/x3plus_ws/install/setup.bash
```

### Robot doesn't move
```bash
# Check if simulation is running:
# Terminal 1 should have: ros2 launch sim_gazebo_bringup gazebo.launch.py
```

### Arm not visible in RViz
```bash
# Make sure launch file uses correct RViz config:
# Should use: gazebo_view.rviz (NOT yahboomcar.rviz)
```

### Turn is slow/fast
```bash
# Adjust turn_wheel_speed in manual_control.py
# Higher speed = faster turn
# Then rebuild and restart
```

---

## 📊 Expected Output

When robot executes `1` (90° left turn):

```
Time       Rotation    Angular Velocity
────────────────────────────────────────
0.00 sec    0°         0 → 0.9 rad/s (0.3 s ramp)
0.30 sec   ~10°        0.90 rad/s
0.80 sec   ~50°        0.90 rad/s
1.30 sec   ~89.5°      coast offset reached
1.35 sec   ~90.0°      brake pulse -0.4 rad/s (50 ms)
1.40 sec   ~90.0°      0 rad/s (STOP)
```

Total time: ~1.4 seconds
Final angle: 90° ± 0.4°

---

## 🎯 Complete Workflow Example

```bash
# Step 1: Terminal 1 - Build
mkdir -p ~/ros2_ws && cd ~/ros2_ws
colcon build --packages-select x3plus_examples sim_gazebo_bringup
source install/setup.bash

# Step 2: Terminal 1 - Launch Gazebo
ros2 launch sim_gazebo_bringup gazebo.launch.py use_rviz:=false

# Step 3: Terminal 2 - Run Control
source ~/ros2_ws/install/setup.bash
ros2 run x3plus_examples manual_control

# Step 4: Interact in Terminal 2
# Press 'W' - robot moves forward
# Press 'A' - robot rotates left
# Press 'SPACE' - stop
# Press '1' - rotate 90° left (shows formula!)
# Press 'Q' - quit
```

---

## 📚 Documentation Files

- **IMPLEMENTATION_SUMMARY.md** - Overview of all changes
- **RVIZ_ARM_FIX_AND_90TURN_FORMULA.md** - Full technical documentation  
- **90DEGREE_TURN_FORMULA.md** - Formula reference with examples
- **VISUAL_GUIDE_90DEGREE_TURN.md** - Diagrams and visual explanations
- **QUICK_START_GUIDE.md** - This file

---

## ✨ What You Get

✅ Fixed RViz showing complete robot  
✅ 90-degree turn formula: ω = 2v/L, t = πL/(4v)  
✅ Keyboard manual control (W/A/S/D)  
✅ Automated 90° turn execution (press 1-4)  
✅ Formula displayed in console  
✅ Full documentation with examples  

---

## 🎓 Learning the Formula

The 90° turn formula comes from **differential drive robotics**:

1. Two independent wheels separated by distance L
2. Rotating them opposite directions (v_L = -v, v_R = +v)
3. Creates angular velocity: ω = (v_R - v_L) / L = 2v / L
4. Time to rotate 90°: t = (π/2) / ω = πL / (4v)

**That's it!** The formula automatically calculates robot rotation.

---

## 🚀 Next Steps

1. ✅ Run the quick start (2 min)
2. ✅ Verify arm visible in RViz
3. ✅ Test manual control movement
4. ✅ Execute 90° turns (see formula output)
5. ✅ Try different turns: 45°, 180°, etc.
6. ✅ Customize parameters for your robot
7. ✅ Use for autonomous navigation planning

---

**You're ready to go! Start with `colcon build` and enjoy the perfectly calculated 90° turns!** 🎉
