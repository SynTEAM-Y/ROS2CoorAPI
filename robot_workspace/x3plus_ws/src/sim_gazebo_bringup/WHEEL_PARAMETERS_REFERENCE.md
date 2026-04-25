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
    <mu1>1.0</mu1>      <!-- Rolling friction (direction of travel) -->
    <mu2>0.05</mu2>     <!-- Lateral friction (sideways) - low for skid-steer turning -->
</gazebo>
```

**Current values:** μ1=1.0 (rolling), μ2=0.05 (lateral)  
**Why asymmetric?** Low lateral friction allows 4-wheel skid-steer to turn properly.
With symmetric high friction, the robot cannot rotate in place.

**Friction Values Guide:**
- `0.05` = Very low lateral (allows skid-steer turning) - **Current μ2**
- `0.5` = Medium grip
- `0.8` = Good grip (carpet, asphalt)
- `1.0` = Excellent rolling grip - **Current μ1**

---

### 8. **Contact Stiffness**
**Location:** Gazebo properties for wheels  
**Search for:** `<!-- CONTACT STIFFNESS: kp`

```xml
<kp>1000000.0</kp>    <!-- Edit this! -->
<kd>1.0</kd>
```

**Default:** 1000000.0 (very stiff)  
**Effect:** How rigid wheel contact is

**Stiffness Guide:**
- `100000` = Soft wheels (deformable)
- `500000` = Medium firmness
- `1000000` = Stiff wheels (typical rubber) - **Default**

---

### 9. **Motor Torque (Power)**
**Location:** wheel_joint macro definition  
**Search for:** `<!-- MOTOR TORQUE LIMIT`

```xml
<limit effort="10" velocity="2"/>
        ^^^^^^
      Edit this!
```

**Default:** 10 N⋅m (Newton-meters)  
**Effect:** Maximum force wheels can apply

**Torque Examples:**
- `5` = Light motor (slow acceleration)
- `10` = Medium motor (balanced) - **Default**
- `20` = Powerful motor (fast acceleration)
- `50` = Very powerful motor (racing)

---

### 10. **Motor Speed Limit**
**Location:** wheel_joint macro definition  
**Search for:** `<!-- MOTOR SPEED LIMIT`

```xml
<limit effort="10" velocity="2"/>
                   ^^^^^^^^
                 Edit this!
```

**Default:** 2 rad/s  
**Effect:** Maximum rotation speed  
**Unit:** radians per second (rad/s)

**Speed Examples:**
- `1` = Slow rotation (crawling)
- `2` = Medium speed (walking) - **Default**
- `5` = Fast rotation (running)
- `10` = Very fast (racing)

---

### 11. **Motor Acceleration**
**Location:** Gazebo plugin (bottom of file)  
**Search for:** `<!-- ACCELERATION`

```xml
<max_wheel_accel>1.0</max_wheel_accel>
                 ^^^
              Edit this!
```

**Default:** 1.0 rad/s²  
**Effect:** How quickly motor can change speed

**Acceleration Examples:**
- `0.5` = Smooth, gradual acceleration
- `1.0` = Normal acceleration - **Default**
- `2.0` = Snappy, quick response
- `5.0` = Very aggressive acceleration

---

### 12. **Update Rate**
**Location:** Gazebo plugin (bottom of file)  
**Search for:** `<!-- UPDATE RATE:`

```xml
<update_rate>30</update_rate>
             ^^
          Edit this!
```

**Default:** 30 Hz (updates per second)  
**Effect:** How often Gazebo updates wheel physics

**Typical Values:**
- `10` = Low frequency (faster sim, less accurate)
- `30` = Normal frequency (good balance) - **Default**
- `50` = High frequency (slower sim, more accurate)
- `100` = Very high frequency (very accurate, slow)

---

## 📊 Quick Customization Examples

### Example 1: Make the Robot Faster
```bash
# In wheel_joint macro, change:
velocity="2"      →   velocity="5"

# In gazebo plugin, change:
max_wheel_torque>10    →   max_wheel_torque>20
max_wheel_accel>1.0    →   max_wheel_accel>3.0
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
# Current values: mu1=1.0 (rolling), mu2=0.05 (lateral)
# To reduce grip:
mu1>1.0    →   mu1>0.4
mu2>0.05   →   mu2>0.02
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
