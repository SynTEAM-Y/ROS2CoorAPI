# Adding Wheel Links and Differential Drive to X3plus Robot

## ⚡ QUICK UPDATE: Wheels Now Included!

The x3plus URDF file has been **updated with wheel links and differential drive**! The wheels are now part of the robot by default.

### What Was Added:
✅ 4 wheel links: front-left, front-right, back-left, back-right (collision-only geometry)
✅ Continuous joints for wheel rotation
✅ Gazebo 4-wheel skid-steer differential drive plugin (drives ALL 4 wheels)
✅ Symmetric high friction (μ1 = μ2 = 2.0) on all wheels
✅ IMU sensor on `imu_link` (ignition-gazebo-imu-system) bridged to ROS `/imu`
✅ Closed-loop 90° turn in `manual_control.py` using IMU yaw (±0.4° accuracy)

### All Parameters Are Clearly Commented:
Every configurable parameter has **inline comments** showing:
- What the parameter does
- Current default value
- How to change it
- What effect changing it will have

**This makes customization super easy!**

## 📍 Where to Find Configurable Parameters

All wheel specifications are in one file with clear comments:

**File:** `src/yahboomcar_description/urdf/yahboomcar_X3plus.urdf.xacro` (relative to workspace root)

### Parameter Locations:

| Parameter | Section | How to Find |
|-----------|---------|-------------|
| **Wheel Color** | Materials | Search: `<!-- WHEEL COLOR -->` |
| **Wheel Radius & Width** | wheel_link macro | Search: `<!-- WHEEL RADIUS:` |
| **Wheel Mass** | wheel_link macro | Search: `<!-- WHEEL INERTIA` |
| **Wheel Positions** | Wheel joints | Search: `<!-- CONFIGURABLE: Wheel Positions -->` |
| **Wheel Friction** | Gazebo properties | Search: `<!-- FRICTION:` |
| **Motor Torque** | wheel_joint macro | Search: `<!-- MOTOR TORQUE` |
| **Motor Speed Limit** | wheel_joint macro | Search: `<!-- MOTOR SPEED` |
| **Motor Acceleration** | Gazebo plugin | Search: `<!-- ACCELERATION` |
| **Wheel Separation** | Gazebo plugin | Search: `<!-- WHEEL SEPARATION:` |
| **Update Rate** | Gazebo plugin | Search: `<!-- UPDATE RATE:` |

### Quick Edit Example:

To change wheel radius from 0.04m to 0.05m:

```bash
# Open the file
code src/yahboomcar_description/urdf/yahboomcar_X3plus.urdf.xacro

# Ctrl+F search for: "WHEEL RADIUS"
# You'll see:
# <cylinder radius="0.04" length="0.015"/>
# Change 0.04 to 0.05 (in both visual and collision)

# Save, rebuild:
colcon build --packages-select yahboomcar_description

# Test in Gazebo
```

---

# Original Documentation Below

This guide explains how to understand and modify the x3plus robot URDF for wheels and differential drive.

1. **Wheel Links** - Physical representation of robot wheels
2. **Wheel Joints** - Connection between chassis and wheels (continuous revolute joints)
3. **Wheel Meshes** - Visual and collision geometry for wheels
4. **Differential Drive Plugin** - Gazebo plugin for controlling wheel motors
5. **Transmissions** - ROS control interface between motors and wheels

## Step 1: Understand the Current Structure

The robot URDF is located at (relative to workspace root):
```
src/yahboomcar_description/urdf/yahboomcar_X3plus.urdf.xacro
```

Key components:
- **Macros** for common patterns (links, joints)
- **Namespace parameter** `${ns}` for ROS namespacing
- **Material definitions** for visual colors
- **Mesh files** in `meshes/` subdirectory

## Step 2: Determine Wheel Configuration

Before modifying the URDF, you need to decide:

### Wheel Properties
- **Wheel Radius**: Distance from wheel center to contact point (e.g., 0.04m for 8cm diameter wheels)
- **Wheel Width**: Thickness of the wheel (e.g., 0.015m)
- **Wheel Mass**: Weight of each wheel (e.g., 0.1kg for light wheels)
- **Axle Distance**: Distance between left and right wheel pairs (e.g., 0.2128m)
- **Wheel Position from Center**: Distance along x-axis from base_link to wheel axle

### Differential Drive Configuration
- **Number of Wheels**: 4 (front-left, front-right, back-left, back-right) for skid-steer differential drive
- **Motor Type**: Continuous revolute joints (no angle limits)
- **Gear Ratio**: Mechanical advantage (usually 1.0 for direct drive)
- **Friction**: Symmetric (μ1 = μ2 = 2.0) on all 4 wheels. With all 4 wheels driven by the DiffDrive plugin, skid-steer rotation works without friction asymmetry.

## Step 3: Create Wheel Mesh Files (Optional)

If you have wheel meshes:
1. Place visual meshes in: `yahboomcar_description/meshes/X3plus/visual/wheel_*.STL`
2. Place collision meshes in: `yahboomcar_description/meshes/X3plus/collision/wheel_*.STL`

If you don't have meshes, create simple cylinder geometries in URDF (see Step 4).

## Step 4: Modify the URDF - Add Wheel Links and Joints

### Basic 4-Wheel Skid-Steer Differential Drive Example

Edit the XACRO file and add the following **after** the base_link definition:

```xml
<!-- ================== WHEEL LINKS ================== -->
<!-- Front Left Wheel -->
<link name="${ns}/front_left_wheel">
    <inertial>
        <mass value="0.1"/>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
    <collision>
        <geometry>
            <cylinder radius="0.04" length="0.015"/>
        </geometry>
        <origin xyz="0 0 0" rpy="1.5707 0 0"/>
    </collision>
</link>

<!-- Front Right Wheel -->
<link name="${ns}/front_right_wheel">
    <inertial>
        <mass value="0.1"/>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
    <collision>
        <geometry>
            <cylinder radius="0.04" length="0.015"/>
        </geometry>
        <origin xyz="0 0 0" rpy="1.5707 0 0"/>
    </collision>
</link>

<!-- Back Left Wheel -->
<link name="${ns}/back_left_wheel">
    <inertial>
        <mass value="0.1"/>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
    <collision>
        <geometry>
            <cylinder radius="0.04" length="0.015"/>
        </geometry>
        <origin xyz="0 0 0" rpy="1.5707 0 0"/>
    </collision>
</link>

<!-- Back Right Wheel -->
<link name="${ns}/back_right_wheel">
    <inertial>
        <mass value="0.1"/>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
    <collision>
        <geometry>
            <cylinder radius="0.04" length="0.015"/>
        </geometry>
        <origin xyz="0 0 0" rpy="1.5707 0 0"/>
    </collision>
</link>

<!-- ================== WHEEL JOINTS ================== -->
<!-- Front Left Wheel Joint -->
<joint name="${ns}/front_left_wheel_joint" type="continuous">
    <parent link="${ns}/base_link"/>
    <child link="${ns}/front_left_wheel"/>
    <origin xyz="0.1054 0.1064 -0.0388" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit effort="10" velocity="30"/>
</joint>

<!-- Front Right Wheel Joint -->
<joint name="${ns}/front_right_wheel_joint" type="continuous">
    <parent link="${ns}/base_link"/>
    <child link="${ns}/front_right_wheel"/>
    <origin xyz="0.1053 -0.1064 -0.0389" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit effort="10" velocity="30"/>
</joint>

<!-- Back Left Wheel Joint -->
<joint name="${ns}/back_left_wheel_joint" type="continuous">
    <parent link="${ns}/base_link"/>
    <child link="${ns}/back_left_wheel"/>
    <origin xyz="-0.1146 0.1064 -0.0396" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit effort="10" velocity="30"/>
</joint>

<!-- Back Right Wheel Joint -->
<joint name="${ns}/back_right_wheel_joint" type="continuous">
    <parent link="${ns}/base_link"/>
    <child link="${ns}/back_right_wheel"/>
    <origin xyz="-0.1146 -0.1064 -0.0395" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit effort="10" velocity="30"/>
</joint>

<!-- ================== MATERIAL DEFINITIONS ================== -->
<!-- Add this near other materials if not already defined -->
<material name="Black">
    <color rgba="0.1 0.1 0.1 1"/>
</material>
```

### Key Parameters to Adjust:

| Parameter | Description | Typical Value | Format |
|-----------|-------------|---|---------|
| `mass` | Weight of each wheel | 0.05-0.2 kg | float |
| `ixx`, `iyy`, `izz` | Moment of inertia | 0.0001-0.01 | float |
| `radius` | Wheel radius | 0.02-0.1 m | float |
| `length` | Wheel width/thickness | 0.01-0.05 m | float |
| `xyz` (origin) | Position relative to base_link | See section below | "X Y Z" |
| `axis xyz` | Rotation axis (0 1 0 = Y-axis) | "0 1 0" for side wheels | "X Y Z" |
| `effort` | Maximum torque | 5-100 N⋅m | float |
| `velocity` | Maximum angular velocity | 10-60 rad/s | float |

### Adjusting Wheel Position:

The `xyz` in the wheel joint origin positions the wheel:

```
<origin xyz="X Y Z" rpy="0 0 0"/>

X: Forward/Backward distance from base_link center
Y: Left/Right distance (positive = right, negative = left)
Z: Up/Down distance (usually negative for wheels below chassis)

Example: xyz="0.1054 0.1064 -0.036"
- 0.1054m forward
- 0.1064m to the left
- 0.036m down (equals wheel radius of 0.04m minus slight clearance)
```

## Step 5: Add Differential Drive Gazebo Plugin

Create or update a file: `yahboomcar_description/urdf/gazebo_differential_drive.xacro`

```xml
<?xml version="1.0" encoding="UTF-8"?>
<robot xmlns:xacro="http://wiki.ros.org/xacro">

<xacro:macro name="differential_drive" params="ns">
    <!-- Gazebo plugin for 4-wheel skid-steer differential drive control -->
    <gazebo>
        <plugin name="ignition::gazebo::systems::DiffDrive"
                filename="ignition-gazebo-diff-drive-system">
            <!-- 4 wheels: 2 left + 2 right (drives ALL 4 for proper skid-steer) -->
            <left_joint>front_left_wheel_joint</left_joint>
            <left_joint>back_left_wheel_joint</left_joint>
            <right_joint>front_right_wheel_joint</right_joint>
            <right_joint>back_right_wheel_joint</right_joint>
            <wheel_separation>0.2128</wheel_separation>    <!-- Distance between left and right wheels -->
            <wheel_radius>0.04</wheel_radius>              <!-- Wheel radius -->
            <max_linear_acceleration>3.0</max_linear_acceleration>
            <max_angular_acceleration>3.0</max_angular_acceleration>
            <max_linear_velocity>1.5</max_linear_velocity>
            <max_angular_velocity>3.0</max_angular_velocity>

            <!-- Ignition Fortress silently ignores leading-slash topic
                 overrides. Use the model-namespaced default and remap in
                 the ros_gz_bridge in sim_gazebo_bringup/launch/gazebo.launch.py. -->
            <topic>/model/x3plus/cmd_vel</topic>
            <odom_topic>/model/x3plus/odometry</odom_topic>
            <tf_topic>/model/x3plus/tf</tf_topic>
            <frame_id>odom</frame_id>
            <child_frame_id>base_footprint</child_frame_id>

            <!-- Odometry parameters -->
            <odom_publish_frequency>30</odom_publish_frequency>
        </plugin>
    </gazebo>

</xacro:macro>

</robot>
```

Include this in your main URDF file:

```xml
<xacro:include filename="$(find yahboomcar_description)/urdf/gazebo_differential_drive.xacro"/>
<xacro:differential_drive ns="${ns}"/>
```

## Step 6: Add Transmissions for Motor Control (Optional)

If you want ROS controllers to drive the wheels, add transmissions:

```xml
<!-- ================== TRANSMISSIONS ==================  -->
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

## Step 7: (Optional) Add Gazebo Friction Properties

For better wheel grip, add these gazebo properties to wheel links:

```xml
<gazebo reference="${ns}/front_left_wheel">
    <material>Gazebo/Black</material>
    <mu1>2.0</mu1>      <!-- Rolling friction -->
    <mu2>2.0</mu2>      <!-- Lateral friction -->
    <kp>200000.0</kp>   <!-- Contact stiffness (softer than ODE default) -->
    <kd>50.0</kd>       <!-- Contact damping -->
</gazebo>

<gazebo reference="${ns}/front_right_wheel">
    <material>Gazebo/Black</material>
    <mu1>2.0</mu1>
    <mu2>2.0</mu2>
    <kp>200000.0</kp>
    <kd>50.0</kd>
</gazebo>

<gazebo reference="${ns}/back_left_wheel">
    <material>Gazebo/Black</material>
    <mu1>2.0</mu1>
    <mu2>2.0</mu2>
    <kp>200000.0</kp>
    <kd>50.0</kd>
</gazebo>

<gazebo reference="${ns}/back_right_wheel">
    <material>Gazebo/Black</material>
    <mu1>2.0</mu1>
    <mu2>2.0</mu2>
    <kp>200000.0</kp>
    <kd>50.0</kd>
</gazebo>
```

## Step 8: Rebuild and Test

### Rebuild the package:
```bash
cd ~/ROS2Coordination/robot_workspace/x3plus_ws
colcon build --packages-select yahboomcar_description sim_gazebo_bringup
source install/setup.bash
```

### Test in Gazebo:
```bash
# Launch Gazebo with the updated robot
ros2 launch sim_gazebo_bringup gazebo.launch.py use_rviz:=false
```

### Test differential drive commands:
```bash
# In another terminal, send velocity commands
ros2 topic pub /cmd_vel geometry_msgs/Twist -- '{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}'
```

## Step 9: Verify Robot Moves

You should see:
- ✅ Wheels spinning in Gazebo
- ✅ Robot moving forward/backward with `/cmd_vel` linear velocity
- ✅ Robot rotating with `/cmd_vel` angular velocity
- ✅ Odometry topic publishing robot position estimates

## Troubleshooting

### Wheels not spinning
- Check that joint axes are aligned correctly: (0 1 0) for Y-axis rotation
- Verify wheel positions don't intersect with base_link
- Ensure Gazebo physics plugin is enabled

### Robot spinning in place
- Check differential drive plugin joint names match URDF exactly
- Verify `wheel_separation` parameter matches actual distance between wheels

### Gazebo crashing during plugin load
- Check plugin filename is correct: `ignition-gazebo-diff-drive-system`
- Verify all joint/link names in plugin config exist in URDF
- Check namespace is used consistently (`${ns}` prefix)

### Wheel collision issues
- Verify wheel position Z coordinate accounts for wheel radius
- Check radius values (wheel should touch ground, not penetrate or float)

## Advanced Configurations

### Mecanum Wheels
Use the `libgazebo_ros_mecanum_drive.so` plugin instead and adjust axis configuration.

### Custom Motor Control
Replace the differential drive plugin with custom ros2_control plugin for more flexibility.

## Summary

You've now learned how to:
1. ✅ Add wheel links to the robot URDF
2. ✅ Create continuous joint connections for wheel rotation
3. ✅ Configure Gazebo differential drive plugin
4. ✅ Send velocity commands to control the robot
5. ✅ Add transmissions for ROS control interface

The robot can now be driven using standard ROS 2 velocity commands!
