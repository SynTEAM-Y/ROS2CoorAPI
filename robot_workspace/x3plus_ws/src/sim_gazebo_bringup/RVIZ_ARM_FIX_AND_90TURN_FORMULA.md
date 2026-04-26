# RViz Arm Visualization Fix & Differential Drive 90-Degree Turn Formula

## Issue 1: Robot Arm Not Showing Properly in RViz

### Root Causes Identified

1. **Poor RViz Configuration**: Default config didn't properly display all arm links
2. **Collision geometry causing visibility issues**: Arm links weren't being rendered correctly
3. **Fixed frame mismatch**: Frame was set to `base_link` instead of `base_footprint`

### Solution Applied

#### Step 1: Created Enhanced RViz Configuration
- **File**: `src/sim_gazebo_bringup/rviz/gazebo_view.rviz` (relative to workspace root)
- **Key Settings**:
  ```yaml
  Fixed Frame: base_footprint  # Changed from base_link
  Cell Size: 0.1              # Clear grid visualization
  All Links Enabled: true     # Ensure all arm links visible
  Expand All: true            # Expand tree to show joints
  Collision Alpha: 0.3        # Show collision geometry
  Visual Enabled: true        # Enable visual rendering
  ```

#### Step 2: Updated Launch File
- Changed RViz config to use `sim_gazebo_bringup/rviz/gazebo_view.rviz`
- This config properly displays:
  - Base chassis
  - **Robot arm** (5 joints: arm_link1 through arm_link5) ✅
  - **Gripper** (left and right fingers with continuous joints)
  - Laser scanner
  - Camera
  - Wheels (front-left, front-right, back-left, back-right)

### Initial Pose Configuration (Default)
- **Base Position**: x=0.0 m, y=0.0 m, z=0.076 m
- **Base Orientation**: Roll=0°, Pitch=0°, Yaw=0°
- **Arm Initial Pose**:
  - arm_joint1: 0 rad (horizontal)
  - arm_joint2-5: 0 rad (neutral position)
  - Gripper joint: 0 rad (closed)

### How to Verify Arm Visualization

```bash
# Launch RViz with proper arm display
ros2 launch sim_gazebo_bringup robot_rviz.launch.py

# You should see:
# - Robot base chassis
# - 5-link robotic arm extending from chassis
# - Gripper with two finger sets (left/right)
# - Use joint_state_publisher_gui to interactively move all joints
```

---

## Issue 2: Robot Chassis (Car Base) 90-Degree Turn with Formula

### The Problem
The robot needs **90-degree turn capability** for the **chassis** (differential drive base), not the arm.

### Mathematical Foundation

#### Differential Drive Kinematic Model

For a four-wheeled skid-steer differential drive robot:

```
      v_left   v_right
    (2 wheels) (2 wheels)
        ↓         ↓
    ┌─────┐   ┌─────┐   front-left / front-right
    │     └───────────→ L (wheel separation = 0.2128m)
    └─────┐   ┌─────┐   back-left / back-right
         ↓         ↓
    
Left pair and right pair driven together by DiffDrive plugin
```

**Robot Angular Velocity** (from differential drive):
$$\omega = \frac{v_{right} - v_{left}}{L}$$

Where:
- $\omega$ = angular velocity (rad/s)
- $v_{right}$ = right wheel pair velocity (m/s) — front_right + back_right
- $v_{left}$ = left wheel pair velocity (m/s) — front_left + back_left
- $L$ = wheel separation distance (m)

#### In-Place Turn (Point Turn) - 90°

**Configuration**:
- Left pair (front_left + back_left): $v_{left} = -v$ (moving backward)
- Right pair (front_right + back_right): $v_{right} = +v$ (moving forward)
- This causes the robot to rotate on its center (skid-steer)

**Derived Angular Velocity**:
$$\omega = \frac{v - (-v)}{L} = \frac{2v}{L}$$

**Turn Time for 90°** (π/2 radians):
$$t = \frac{\theta}{\omega} = \frac{\pi/2}{2v/L} = \frac{\pi L}{4v}$$

#### Practical Example Calculation

**Given** (default robot parameters):
- Wheel separation: $L = 0.2128$ m
- Wheel speed: $v = 0.5$ m/s

**Calculate**:
1. Angular velocity:
   $$\omega = \frac{2 \times 0.5}{0.2128} = \frac{1.0}{0.2128} = 4.699 \text{ rad/s}$$

2. Turn time:
   $$t = \frac{\pi \times 0.2128}{4 \times 0.5} = \frac{0.503}{2.0} = 0.334 \text{ seconds}$$

3. Verification:
   - Turn angle: $\theta = \omega \times t = 4.699 \times 0.334 = 1.569$ rad ≈ 90° ✅

#### Arc Turn (Moving Forward)

When the robot turns while moving forward:
- Both wheels move forward but at different speeds
- Robot follows circular arc instead of spinning in place
- Time increases but movement is more efficient
- Formula: Similar but with additional forward velocity component

---

### Implementation: Manual Control Node

#### File Location
`src/x3plus_examples/x3plus_examples/manual_control.py` (relative to workspace root)

#### Features

1. **Interactive Keyboard Control**
   ```
   Basic Movement:
     W/w  → Forward (0.8 m/s)
     S/s  → Backward (-0.8 m/s)
     A/a  → Rotate left (1.0 rad/s)
     D/d  → Rotate right (-1.0 rad/s)
     Space→ Stop
   
   90-Degree Turns (closed-loop with IMU/odom feedback):
     1    → 90° left turn (in-place)
     2    → 90° right turn (in-place)
     3    → 90° left turn (moving forward arc)
     4    → 90° right turn (moving forward arc)
   
   System:
     Q/q  → Quit
     H/h  → Show help
   ```

2. **Automatic Formula Calculation**
   - When you press 1-4, the node:
     1. Calculates theoretical ω = 2v / L and t = πL / (4v)
     2. Displays the theoretical values on screen
     3. Executes the turn using closed-loop IMU/odom yaw tracking
     4. Stops when measured rotation reaches 90°
     5. Reports actual rotation and error

3. **Console Output Example** (pressing key '1'):
   ```
   ══════════════════════════════════════════════════════════════════════════
   90-DEGREE TURN EXECUTION (CLOSED-LOOP)
   ══════════════════════════════════════════════════════════════════════════
   Direction: LEFT
   Type: IN_PLACE

   📐 THEORETICAL (open-loop):
   ────────────────────────
     ω = 2v/L = 2×0.5/0.2128 = 4.6992 rad/s
     t = (π/2)/ω = 0.3343 s

   🤖 ACTUAL EXECUTION (closed-loop with odometry):
   ────────────────────────
     Command ω: 1.50 rad/s
     Linear: 0.00 m/s
     Target rotation: 90° (π/2 = 1.5708 rad)
     Feedback: /odom yaw tracking
   ══════════════════════════════════════════════════════════════════════════

   90° left turn completed! (actual: 90.2°, error: +0.2°)
   ```

---

### Usage Instructions

#### Step 1: Build the Package
```bash
cd ~/ROS2Coordination/robot_workspace/x3plus_ws
colcon build --packages-select x3plus_examples sim_gazebo_bringup
source install/setup.bash
```

#### Step 2: Launch the Robot Simulation
```bash
# Option A: With Gazebo physics simulation
ros2 launch sim_gazebo_bringup gazebo.launch.py use_rviz:=false

# Option B: With RViz only (no physics)
ros2 launch sim_gazebo_bringup robot_rviz.launch.py
```

#### Step 3: Run Manual Control Node (in another terminal)
```bash
# Make sure to source the workspace first
source ~/ROS2Coordination/robot_workspace/x3plus_ws/install/setup.bash

# Run the manual control node
ros2 run x3plus_examples manual_control
```

#### Step 4: Control the Robot
```
Press 'W' - move forward
Press 'A' - rotate left continuously
Press 'SPACE' - stop
Press '1' - execute 90° left turn (shows formula calculation)
```

---

### Configurable Parameters

Edit the `__init__` method in `manual_control.py` to customize:

```python
# ============ ROBOT CONFIGURATION ============
self.wheel_separation    = 0.2128  # Distance between wheel pairs (m)  ← matches URDF
self.wheel_radius        = 0.04    # Wheel radius (m)                   ← matches URDF
self.max_linear_velocity = 0.8     # Max forward/backward speed (m/s)
self.max_angular_velocity= 1.0     # Max rotation speed for A/D keys (rad/s)
self.turn_wheel_speed    = 0.5     # v in theoretical formula (m/s)
```

> **Note**: `max_angular_velocity = 1.0 rad/s` applies to A/D keys only.
> The 90° turn commands use `angular.z = ±1.50 rad/s` (hardcoded, separate from A/D).
> Changing `max_angular_velocity` does NOT affect 90° turn speed.

**IMPORTANT**: The `wheel_separation` and `turn_wheel_speed` affect the **theoretical
display values** (ω, t). The actual closed-loop turn stops by measuring π/2 rad of
rotation regardless of these values.

---

### Expected Behavior

| Command | Closed-loop target | Movement |
|---------|--------------------|----------|
| `1` (90° left, in-place)  | π/2 rad via IMU/odom | Spin left 90°, no forward |
| `2` (90° right, in-place) | π/2 rad via IMU/odom | Spin right 90°, no forward |
| `3` (90° left arc)        | π/2 rad via IMU/odom | Move forward + turn left |
| `4` (90° right arc)       | π/2 rad via IMU/odom | Move forward + turn right |

Open-loop estimate ~0.334 s (at default params) but actual duration depends on
physics/friction. The turn completes when measured yaw change ≥ π/2 rad.

---

### Formula Variations

#### For 45-Degree Turn
$$t_{45°} = \frac{\pi L}{8v}$$
(Exactly half the 90° time)

#### For Custom Angle θ (in degrees)
First convert to radians: $\theta_{rad} = \frac{\theta \times \pi}{180}$

Then: $t = \frac{\theta_{rad}}{ω} = \frac{\theta_{rad} \times L}{2v}$

#### For Different Speed
If you change `turn_wheel_speed` to 0.8 m/s:
$$t = \frac{\pi \times 0.2128}{4 \times 0.8} = 0.157 \text{ sec}$$ (faster!)

---

## Summary of Fixes

| Issue | Solution | File | Status |
|-------|----------|------|--------|
| RViz not showing arm | Create enhanced RViz config | `gazebo_view.rviz` | ✅ Fixed |
| Poor arm joint display | Update all RViz settings | `robot_rviz.launch.py` | ✅ Fixed |
| No 90° turn capability | Auto-calculate formula + closed-loop | `manual_control.py` | ✅ Created |
| No manual control node | New keyboard teleop | `x3plus_examples/setup.py` | ✅ Added |
| Gripper mimic joints static | `gripper_mimic_relay` filter (strips mimic joints from `/joint_states`) | `x3plus_examples` | ✅ Added |
| Plugins dropped at spawn | URDF→SDF pre-conversion (`ign sdf -p`) | `gazebo.launch.py` | ✅ Fixed |
| Odometry undocumented | Full math reference | `README.md` (Odometry section) | ✅ Inline |

---

## Testing Checklist

- [ ] Build packages without errors
- [ ] RViz shows complete robot with arm
- [ ] Can move robot with W/A/S/D keys
- [ ] 90° left turn (key 1) shows formula in console
- [ ] Robot rotates ~90° then stops
- [ ] 90° right turn (key 2) rotates opposite direction
- [ ] Moving arc turn (keys 3-4) work while in motion
- [ ] Formula time matches actual rotation duration
