# 🎯 Wheel Specifications - Quick Reference Guide

## ✅ Status: Wheels Are Now Included in the URDF!

All wheel links, joints, and differential drive configuration have been **successfully added** to the x3plus robot URDF file.

**Every parameter is clearly commented** in the source code, so you can easily find and modify them!

---

## 📍 Where to Find Each Parameter

### File Location (relative to workspace root):
```
src/yahboomcar_description/urdf/yahboomcar_X3plus.urdf.xacro
```

---

## 🔧 All Configurable Parameters

### 1. **Wheel Color**
**Location:** Top of file with other materials  
**Search for:** `<!-- WHEEL COLOR -->`

```xml
<material name="Black">
    <color rgba="0.1 0.1 0.1 1"/>    <!-- RGBA: Red, Green, Blue, Alpha (0-1 scale) -->
</material>
```

**How to change:** Edit RGBA values (0.1 0.1 0.1 = dark gray)

---

### 2. **Wheel Radius**
**Location:** `wheel_link` macro definition  
**Search for:** `<!-- WHEEL RADIUS:`

```xml
<cylinder radius="0.04" length="0.015"/>
                ^^^^
              Edit this!
```

**Default:** 0.04 m (4 cm = 8 cm diameter)  
**Examples:**
- `0.03` = 6 cm diameter wheel
- `0.04` = 8 cm diameter wheel (current)
- `0.05` = 10 cm diameter wheel

⚠️ **IMPORTANT:** Change in TWO places:
1. Visual geometry (visual section)
2. Collision geometry (collision section)

---

### 3. **Wheel Width (Thickness)**
**Location:** `wheel_link` macro definition  
**Search for:** `<!-- WHEEL WIDTH:`

```xml
<cylinder radius="0.04" length="0.015"/>
                              ^^^^^
                            Edit this!
```

**Default:** 0.015 m (1.5 cm)  
**Effect:** How thick/wide the wheel appears  
**Examples:**
- `0.01` = 1 cm (thin wheel)
- `0.015` = 1.5 cm (current)
- `0.02` = 2 cm (thicker wheel)

---

### 4. **Wheel Mass**
**Location:** Wheel link definitions (after base_imu)  
**Search for:** `<xacro:wheel_link name="front_left_wheel" mass=`

```xml
<xacro:wheel_link name="front_left_wheel" mass="0.1"/>
<xacro:wheel_link name="front_right_wheel" mass="0.1"/>
                           mass="^^^"
                        Edit these!
```

**Default:** 0.1 kg (100 grams per wheel)  
**Effect:**
- Heavier wheels = better traction, slower acceleration
- Lighter wheels = faster response, less grip

**Examples:**
- `0.05` = 50g wheels (light foam)
- `0.1` = 100g wheels (current)
- `0.2` = 200g wheels (heavy rubber)

---

### 5. **Wheel Forward Position**
**Location:** Wheel joint definitions  
**Search for:** `<!-- CONFIGURABLE: Wheel Positions -->`

```xml
<xacro:wheel_joint name="front_left_wheel_joint" parent="base_link" child="front_left_wheel" x_pos="0.1054" y_pos="0.1064"/>
                                                                                       ^^^^
                                                                                    Edit this!
```

**Default:** 0.1054 m for front wheels, -0.1146 m for back wheels  
**Effect:** Moves wheels forward/backward  
**Examples:**
- `0.0` = wheels aligned with chassis center
- `0.1054` = 10.5 cm forward (current for front wheels)
- `-0.1146` = 11.5 cm backward (current for back wheels)

**Tip:** For a robot with long front, increase this value (0.08-0.15) to move wheels forward.

---

### 6. **Wheel Separation (Left-Right Distance)**
**Location:** Wheel joint Y positions and gazebo plugin  
**Search for:** `y_pos="0.1064"` and `<!-- WHEEL SEPARATION:`

```xml
<xacro:wheel_joint ... y_pos="0.1064"/>           <!-- Left wheel -->
<xacro:wheel_joint ... y_pos="-0.1064"/>          <!-- Right wheel -->

<!-- In gazebo plugin (at end of file): -->
<wheel_separation>0.2128</wheel_separation>       <!-- Must equal: 0.1064 + 0.1064 -->
                  ^^^^
```

**Default:** 0.2128 m (21.3 cm) = 10.64 cm left + 10.64 cm right  
**IMPORTANT:** These MUST match!

**Calculation:**
```
wheel_separation = abs(left_y_pos) + abs(right_y_pos)
wheel_separation = 0.1064 + 0.1064 = 0.2128
```

**Examples:**
- If you change y_pos to ±0.1 → wheel_separation must be 0.2
- If you change y_pos to ±0.05 → wheel_separation must be 0.1

---

### 7. **Wheel Friction**
**Location:** Gazebo properties for wheels  
**Search for:** `mu1` and `mu2`

```xml
<gazebo reference="front_left_wheel">
    <mu1>2.0</mu1>      <!-- Rolling friction (direction of travel) -->
    <mu2>2.0</mu2>      <!-- Lateral friction (sideways) -->
    <kp>200000.0</kp>
    <kd>50.0</kd>
</gazebo>
```

**Current values:** μ1 = μ2 = 2.0 (symmetric, applied to ALL 4 wheels)  
**Why symmetric?** Earlier asymmetric attempts (μ2 = 0.05) caused chassis
translation instead of clean rotation. With all 4 wheels driven by the
DiffDrive plugin (not just the front pair), symmetric grip combined with
skid-steer wheel scrub produces accurate in-place rotation. The closed-loop
turn algorithm in `manual_control.py` then trims the final heading to within
±0.4° of the 90° target.

**Friction Values Guide:**
- `0.5`  = Low grip (chassis slides, hard to control)
- `1.0`  = Medium grip
- `2.0`  = Strong grip — **Current value for all 4 wheels**
- `4.0`  = Very strong grip (chassis may lock instead of yawing)

---

### 8. **Contact Stiffness**
**Location:** Gazebo properties for wheels  
**Search for:** `<kp>` / `<kd>`

```xml
<kp>200000.0</kp>    <!-- Contact stiffness -->
<kd>50.0</kd>        <!-- Contact damping -->
```

**Default:** kp = 200000.0, kd = 50.0 (softer + better damped than ODE
defaults). Earlier values (kp=1e6, kd=1) made the chassis shake violently
during turns; softer contact stops the high-frequency oscillation.

**Stiffness Guide:**
- `100000`  = Soft wheels (deformable)
- `200000`  = Current value
- `500000`  = Stiff
- `1000000` = Very stiff (legacy default — caused chassis shake)

---

### 9. **Motor Torque (Power)**
**Location:** wheel_joint macro definition  
**Search for:** `<!-- MOTOR TORQUE LIMIT`

```xml
<limit effort="10" velocity="30"/>
        ^^^^^^
      Edit this!
```

**Default:** 10 N⋅m (Newton-meters)  
**Effect:** Maximum force wheels can apply

**Torque Examples:**
- `5`  = Light motor (slow acceleration)
- `10` = Medium motor (balanced) — **Current**
- `20` = Powerful motor (fast acceleration)
- `50` = Very powerful motor (racing)

---

### 10. **Motor Speed Limit**
**Location:** wheel_joint macro definition  
**Search for:** `<!-- MOTOR SPEED LIMIT`

```xml
<limit effort="10" velocity="30"/>
                   ^^^^^^^^^
                 Edit this!
```

**Default:** 30 rad/s. Wheel radius 0.04 m → max linear speed ≈ 1.2 m/s,
which comfortably exceeds the DiffDrive plugin's `max_linear_velocity` of
1.5 m/s for the chassis. The earlier value `velocity="2"` was a hard cap
that prevented the wheels from reaching commanded speeds.  
**Unit:** radians per second (rad/s)

**Speed Examples:**
- `2`  = Old (broken) value — 0.08 m/s linear cap
- `10` = Slow but usable
- `30` = Current (≈1.2 m/s linear)
- `60` = Very fast

---

### 11. **Plugin Acceleration / Velocity Limits**
**Location:** `ignition-gazebo-diff-drive-system` plugin block  
**Search for:** `max_linear_acceleration`

```xml
<max_linear_acceleration>3.0</max_linear_acceleration>
<max_angular_acceleration>3.0</max_angular_acceleration>
<max_linear_velocity>1.5</max_linear_velocity>
<max_angular_velocity>3.0</max_angular_velocity>
```

**Effect:** Hard limits the plugin enforces on `cmd_vel` before sending wheel
commands. The legacy Gazebo Classic tags `max_wheel_torque` and
`max_wheel_accel` are NOT used by the Ignition Fortress DiffDrive plugin.

**Acceleration Examples:**
- `1.0`  = Smooth, gradual
- `3.0`  = Current value (snappy but not violent)
- `6.0`  = Aggressive

---

### 12. **Odometry Publish Rate**
**Location:** `ignition-gazebo-diff-drive-system` plugin block  
**Search for:** `odom_publish_frequency`

```xml
<odom_publish_frequency>30</odom_publish_frequency>
```

**Default:** 30 Hz. The plugin publishes `/model/x3plus/odometry`, which the
ros_gz_bridge in `gazebo.launch.py` remaps to ROS `/odom` at the same rate.
The IMU sensor on `imu_link` runs at 100 Hz (bridged to `/imu` at \u224852 Hz
in practice on a typical laptop) and is preferred by `manual_control.py`'s
closed-loop turn for ground-truth chassis yaw.

**Typical Values:**
- `10`  = Low frequency (faster sim, less accurate)
- `30`  = Current value (good balance)
- `100` = Very high frequency (matches IMU rate)

---

## 📊 Quick Customization Examples

### Example 1: Make the Robot Faster
```bash
# In wheel_joint macro, change:
velocity="30"     →   velocity="60"

# In gazebo plugin, change:
max_linear_velocity>1.5    →   max_linear_velocity>3.0
max_linear_acceleration>3.0    →   max_linear_acceleration>6.0
```

### Example 2: Make the Robot with Bigger Wheels
```bash
# In wheel_link macro, change:
radius="0.04"     →   radius="0.05"
length="0.015"    →   length="0.025"

# In gazebo plugin, change:
wheel_diameter>0.08    →   wheel_diameter>0.10
```

### Example 3: Make the Robot More Slippery (Low Grip)
```bash
# Current values: mu1=mu2=2.0 on all 4 wheels
# To reduce grip:
mu1>2.0    →   mu1>0.8
mu2>2.0    →   mu2>0.8
```

### Example 4: Make Wheels Wider
```bash
# In wheel_link macro (both visual and collision), change:
length="0.015"    →   length="0.035"
```

---

## 🧪 Testing Your Changes

### Step 1: Edit the Parameters
```bash
code src/yahboomcar_description/urdf/yahboomcar_X3plus.urdf.xacro
```

### Step 2: Rebuild
```bash
cd ~/ROS2Coordination/robot_workspace/x3plus_ws
colcon build --packages-select yahboomcar_description
source install/setup.bash
```

### Step 3: Launch Gazebo
```bash
ros2 launch sim_gazebo_bringup gazebo.launch.py use_rviz:=false
```

### Step 4: Test Movement
```bash
# Move forward
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 1.0}}'

# Rotate
ros2 topic pub /cmd_vel geometry_msgs/Twist '{angular: {z: 1.0}}'

# Monitor wheel states
ros2 topic echo /joint_states | grep wheel
```

---

## ⚠️ Important Reminders

1. **Wheel Radius** - Must be changed in TWO places (visual and collision)
2. **Wheel Separation** - Must match in wheel y_pos AND gazebo plugin
3. **Wheel Diameter** - Update in gazebo plugin when you change radius
4. **Units** - All distances in meters (m), all angles in radians (rad)
5. **Comments** - All parameters are clearly commented with their defaults

---

## 🎯 Next Steps

1. ✅ Understand what each parameter does
2. ✅ Identify which one(s) you want to change
3. ✅ Edit the XACRO file (link to comments makes it easy)
4. ✅ Rebuild with `colcon build`
5. ✅ Test in Gazebo
6. ✅ Adjust until satisfied

Good luck customizing your robot! 🤖
