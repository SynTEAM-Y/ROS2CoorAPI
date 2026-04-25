# Practical Example: Complete Differential Drive URDF Modifications

## ✅ STATUS: Code Has Been Added to the URDF!

The wheel modifications have been **successfully integrated** into the x3plus URDF file. You no longer need to manually add the code blocks below.

### What's Already in the File:
✅ Black material definition for wheels
✅ `wheel_link` macro for creating wheel links
✅ `wheel_joint` macro for creating wheel joints
✅ 4 wheel links and joints (front-left, front-right, back-left, back-right)
✅ Gazebo friction properties for all 4 wheels
✅ Gazebo 4-wheel skid-steer differential drive plugin

### All Parameters Are Clearly Commented:
Every parameter has **inline comments** explaining:
- What it controls
- Default value
- How to modify it
- Effects of changes

## 🔧 How to Customize the Wheels

### Step 1: Open the URDF file
```bash
code src/yahboomcar_description/urdf/yahboomcar_X3plus.urdf.xacro
```

### Step 2: Find Parameters Using Search

Use Ctrl+F (or Cmd+F on Mac) to find any parameter you want to change:

| What to Change | Search For | Examples |
|---|---|---|
| Wheel radius | `WHEEL RADIUS:` | Change 0.04 to 0.05 (radius in meters) |
| Wheel thickness | `WHEEL WIDTH:` | Change 0.015 to 0.02 (width in meters) |
| Wheel mass | `mass="0.1"` (in wheel_link) | Change to 0.2 for heavier wheels |
| Wheel forward position | `x_pos="0.1054"` | Change to 0.1 to move wheels more forward |
| Wheel separation | `y_pos="0.1064"` and `wheel_separation>0.2128` | Adjust both to match (left y - right y) |
| Motor power | `max_wheel_torque>10` | Change to 20 for more power |
| Motor speed | `velocity>2` | Change to 5 for faster rotation |
| Friction | `mu1` and `mu2` | Current: μ1=1.0 (rolling), μ2=0.05 (lateral, low for skid-steer) |

### Step 3: Make Your Changes

Example: To make wheels 10cm diameter (radius 0.05):

```bash
# Before:
<cylinder radius="0.04" length="0.015"/>

# After:
<cylinder radius="0.05" length="0.015"/>
```

Must change in TWO places:
1. Visual geometry (visual section)
2. Collision geometry (collision section)

### Step 4: Update Related Parameters

**IMPORTANT:** Some parameters must be changed together:

```
If you change wheel radius from 0.04 to 0.05:
  Also change wheel_diameter in plugin from 0.08 to 0.10 (2 * radius)
```

### Step 5: Verify Your Changes

```bash
# Check the XACRO syntax is valid
xacro src/yahboomcar_description/urdf/yahboomcar_X3plus.urdf.xacro ns:="" > /tmp/test.urdf

# If you see XML output with no errors, syntax is good!
```

### Step 6: Rebuild and Test

```bash
cd ~/ROS2Coordination/robot_workspace/x3plus_ws
colcon build --packages-select yahboomcar_description
source install/setup.bash

# Test in Gazebo
ros2 launch sim_gazebo_bringup gazebo.launch.py use_rviz:=false

# Test movement
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 1.0}}'
```

---

# Reference: Original Code Blocks

Below are the code blocks that have already been added to the URDF. This section is for reference/documentation only.

## File to Edit (relative to workspace root)

```
src/yahboomcar_description/urdf/yahboomcar_X3plus.urdf.xacro
```

## Already Included: Block 1 - Material Definition

Find the existing material definitions:
```xml
<material name="Green">
    <color rgba="0 0.7 0 1"/>
</material>
<material name="White">
    <color rgba="0.7 0.7 0.7 1"/>
</material>
```

Add after them:
```xml
<material name="Black">
    <color rgba="0.1 0.1 0.1 1"/>
</material>
```

### Block 2: Add Wheel Macro Definition

Add this macro definition near other macros (after `fixed_joint` macro):

```xml
<!-- wheel link macro with simple cylinder geometry -->
<xacro:macro name="wheel_link" params="name mass">
    <link name="${ns}/${name}">
        <inertial>
            <mass value="${mass}"/>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <!-- Moment of inertia for cylinder: I = (1/12) * m * (3*r^2 + h^2) -->
            <!-- For r=0.04m, h=0.015m, m=0.1kg: I ≈ 0.000067 -->
            <inertia ixx="0.000067" ixy="0" ixz="0" iyy="0.000067" iyz="0" izz="0.000032"/>
        </inertial>
        <visual>
            <geometry>
                <cylinder radius="0.04" length="0.015"/>
            </geometry>
            <origin xyz="0 0 0" rpy="1.5707 0 0"/>
            <material name="Black"/>
        </visual>
        <collision>
            <geometry>
                <cylinder radius="0.04" length="0.015"/>
            </geometry>
            <origin xyz="0 0 0" rpy="1.5707 0 0"/>
        </collision>
    </link>
</xacro:macro>

<!-- wheel joint macro for continuous rotation -->
<xacro:macro name="wheel_joint" params="name parent child x_pos y_pos">
    <joint name="${ns}/${name}" type="continuous">
        <parent link="${ns}/${parent}"/>
        <child link="${ns}/${child}"/>
        <!-- Position wheel: x_pos forward, y_pos sideways, -0.036 down (radius - clearance) -->
        <origin xyz="${x_pos} ${y_pos} -0.036" rpy="0 0 0"/>
        <!-- Rotate around Y axis for side-to-side wheels -->
        <axis xyz="0 1 0"/>
        <limit effort="10" velocity="2"/>
        <dynamics damping="0.1" friction="0.0"/>
    </joint>
</xacro:macro>
```

### Block 3: Add Wheel Links and Joints

Add this **after** the base_link joint definitions (after the `<fixed_joint name="base_imu".../>` line):

```xml
<!-- ==================== WHEELS ==================== -->
<!-- Front Left Wheel -->
<wheel_link name="front_left_wheel" mass="0.1"/>

<!-- Front Right Wheel -->
<wheel_link name="front_right_wheel" mass="0.1"/>

<!-- Back Left Wheel -->
<wheel_link name="back_left_wheel" mass="0.1"/>

<!-- Back Right Wheel -->
<wheel_link name="back_right_wheel" mass="0.1"/>

<!-- Front Left Wheel Joint -->
<wheel_joint name="front_left_wheel_joint" parent="base_link" child="front_left_wheel" x_pos="0.1054" y_pos="0.1064"/>

<!-- Front Right Wheel Joint -->
<wheel_joint name="front_right_wheel_joint" parent="base_link" child="front_right_wheel" x_pos="0.1054" y_pos="-0.1064"/>

<!-- Back Left Wheel Joint -->
<wheel_joint name="back_left_wheel_joint" parent="base_link" child="back_left_wheel" x_pos="-0.1146" y_pos="0.1064"/>

<!-- Back Right Wheel Joint -->
<wheel_joint name="back_right_wheel_joint" parent="base_link" child="back_right_wheel" x_pos="-0.1146" y_pos="-0.1064"/>

<!-- Gazebo wheel friction properties (all 4 wheels) -->
<gazebo reference="${ns}/front_left_wheel">
    <material>Gazebo/Black</material>
    <mu1>1.0</mu1>
    <mu2>0.05</mu2>
    <kp>1000000.0</kp>
    <kd>1.0</kd>
</gazebo>

<gazebo reference="${ns}/front_right_wheel">
    <material>Gazebo/Black</material>
    <mu1>1.0</mu1>
    <mu2>0.05</mu2>
    <kp>1000000.0</kp>
    <kd>1.0</kd>
</gazebo>

<gazebo reference="${ns}/back_left_wheel">
    <material>Gazebo/Black</material>
    <mu1>1.0</mu1>
    <mu2>0.05</mu2>
    <kp>1000000.0</kp>
    <kd>1.0</kd>
</gazebo>

<gazebo reference="${ns}/back_right_wheel">
    <material>Gazebo/Black</material>
    <mu1>1.0</mu1>
    <mu2>0.05</mu2>
    <kp>1000000.0</kp>
    <kd>1.0</kd>
</gazebo>
```

### Block 4: Add Gazebo Differential Drive Plugin

Add this **at the end** of the URDF file, after all other tags but before the closing `</robot>`:

```xml
<!-- ==================== GAZEBO PLUGINS ==================== -->
<gazebo>
    <plugin name="ignition::gazebo::systems::DiffDrive"
            filename="ignition-gazebo-diff-drive-system">
        <!-- 4-wheel configuration: 2 left + 2 right -->
        <left_joint>${ns}/front_left_wheel_joint</left_joint>
        <left_joint>${ns}/back_left_wheel_joint</left_joint>
        <right_joint>${ns}/front_right_wheel_joint</right_joint>
        <right_joint>${ns}/back_right_wheel_joint</right_joint>
        <wheel_separation>0.2128</wheel_separation>
        <wheel_radius>0.04</wheel_radius>
        
        <!-- Motor control limits -->
        <max_linear_acceleration>1.0</max_linear_acceleration>
        
        <!-- ROS topics -->
        <topic>cmd_vel</topic>
        <odom_topic>odom</odom_topic>
        <frame_id>odom</frame_id>
        <child_frame_id>base_footprint</child_frame_id>
        
        <!-- Publishing -->
        <odom_publish_frequency>30</odom_publish_frequency>
    </plugin>
</gazebo>
```

### Block 5: (Optional) Add Wheel Transmissions

If you want ROS actuation controllers, add this before the closing `</robot>`:

```xml
<!-- ==================== TRANSMISSIONS ==================== -->
<!-- Front Left Wheel Transmission -->
<transmission name="${ns}/front_left_wheel_transmission">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="${ns}/front_left_wheel_joint">
        <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
    </joint>
    <actuator name="${ns}/front_left_wheel_motor">
        <mechanicalReduction>1</mechanicalReduction>
        <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
    </actuator>
</transmission>

<!-- Front Right Wheel Transmission -->
<transmission name="${ns}/front_right_wheel_transmission">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="${ns}/front_right_wheel_joint">
        <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
    </joint>
    <actuator name="${ns}/front_right_wheel_motor">
        <mechanicalReduction>1</mechanicalReduction>
        <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
    </actuator>
</transmission>

<!-- Back Left Wheel Transmission -->
<transmission name="${ns}/back_left_wheel_transmission">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="${ns}/back_left_wheel_joint">
        <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
    </joint>
    <actuator name="${ns}/back_left_wheel_motor">
        <mechanicalReduction>1</mechanicalReduction>
        <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
    </actuator>
</transmission>

<!-- Back Right Wheel Transmission -->
<transmission name="${ns}/back_right_wheel_transmission">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="${ns}/back_right_wheel_joint">
        <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
    </joint>
    <actuator name="${ns}/back_right_wheel_motor">
        <mechanicalReduction>1</mechanicalReduction>
        <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
    </actuator>
</transmission>
```

## Step-by-Step Installation

### Step 1: Backup Original File
```bash
cp src/yahboomcar_description/urdf/yahboomcar_X3plus.urdf.xacro \
   src/yahboomcar_description/urdf/yahboomcar_X3plus.urdf.xacro.backup
```

### Step 2: Edit the URDF File

Open the file in an editor:
```bash
code src/yahboomcar_description/urdf/yahboomcar_X3plus.urdf.xacro
```

Make these changes:

1. **Add Black material** (after line 5 or so, where other materials are defined)
2. **Add wheel macros** (around line 20-40, after other macro definitions)
3. **Add wheel links/joints** (around line 70-80, after base link and imu definitions)
4. **Add gazebo plugin** (at very end, before `</robot>`)
5. **Add transmissions** (before closing `</robot>`, optional)

### Step 3: Verify Your Edits

Check that the file is valid:
```bash
xacro src/yahboomcar_description/urdf/yahboomcar_X3plus.urdf.xacro ns:=""
```

You should see URDF output with wheel links included.

### Step 4: Rebuild

```bash
cd ~/ROS2Coordination/robot_workspace/x3plus_ws
colcon build --packages-select yahboomcar_description
source install/setup.bash
```

### Step 5: Test

```bash
# Terminal 1: Launch Gazebo
ros2 launch sim_gazebo_bringup gazebo.launch.py use_rviz:=false

# Terminal 2: Send movement commands
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

# Terminal 3: Monitor robot state
ros2 topic echo /joint_states | grep -i wheel
```

## Configuration Parameters You Might Need to Adjust

### Wheel Radius and Mass
```xml
<wheel_link name="front_left_wheel" mass="0.1"/>
                               ^^^^
                              Change this for heavier/lighter wheels
```

In the macro definition, change the cylinder radius:
```xml
<cylinder radius="0.04" length="0.015"/>
                ^^^^
                Wheel radius in meters
```

### Wheel Separation (Distance Between Left and Right Wheels)
```xml
<wheel_separation>0.2128</wheel_separation>
                  ^^^^
                  Change to match your robot's wheel spacing
```

### Wheel Position (Forward/Backward from Base Center)
```xml
<wheel_joint name="front_left_wheel_joint" parent="base_link" child="front_left_wheel" x_pos="0.1054" y_pos="0.1064"/>
                                                                             ^^^^^
                                                                        Forward distance (0.1054 = 10.5cm)
```

### Motor Torque and Acceleration Limits
```xml
<max_wheel_torque>10</max_wheel_torque>      <!-- Increase for more power -->
<max_wheel_accel>1.0</max_wheel_accel>       <!-- Increase for faster acceleration -->
```

## Expected Results After Setup

### In Gazebo:
- ✅ Robot has four visible black wheel cylinders
- ✅ Wheels are positioned at the sides of the base
- ✅ Wheels are roughly touching the ground plane

### When Sending Commands:
```bash
# Move forward
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 1.0}}'
# Result: Both wheels spin forward, robot moves forward

# Turn left
ros2 topic pub /cmd_vel geometry_msgs/Twist '{angular: {z: 1.0}}'
# Result: Left wheel slower, right wheel faster, robot rotates

# Move backward
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: -1.0}}'
# Result: Both wheels spin backward, robot moves backward
```

### In ROS Topics:
```bash
ros2 topic list | grep joint_states
# Output: /joint_states (includes all 4 wheel joints)

ros2 topic echo /joint_states
# Should show: positions and velocities for front_left, front_right, back_left, back_right wheel joints
```

## Common Issues and Fixes

| Issue | Solution |
|-------|----------|
| Wheels not visible in Gazebo | Check cylinder geometry and visual origin (rpy should be 1.5707 0 0) |
| Wheel penetrating ground | Reduce Z position value (less negative) or reduce wheel radius |
| Robot not moving with cmd_vel | Check joint names in gazebo plugin match URDF exactly |
| Wheel spinning wrong direction | Check axis definition (0 1 0 for Y-axis rotation) |
| Gazebo crash on startup | Verify plugin filename: `ignition-gazebo-diff-drive-system` |

## Next Steps

1. **Add Caster Wheel** - Small wheel in front/back for balance
2. **Implement ROS 2 Control** - Use ros2_control framework for advanced control
3. **Add IMU Odometry** - Combine IMU + wheel odometry for better localization
4. **Implement Nav2** - Use Nav2 stack for autonomous navigation

Happy robotics! 🤖
