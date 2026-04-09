# High-Level Documentation of `mapping.launch.py`

This document provides a high-level explanation of how the mapping launch file sets up the **TF tree**, **LiDAR driver**, **odometry**, **SLAM**, and **RViz** visualization.
It also explains how the supporting script `odom_integrator.py` converts `/cmd_vel` commands into `/odom` and corresponding TF transforms.
The goal is to describe the code flow clearly, showing which elements are reusable and which are specific to the Yahboom Rosmaster X3 Plus platform.

**References:**
- [`mapping.launch.py`](../../x3plus_ws/src/x3plus_mapping_bringup/launch/mapping.launch.py)
- [`odom_integrator.py`](../../x3plus_ws/src/x3plus_mapping_bringup/x3plus_mapping_bringup/odom_integrator.py)

---

## 1) Launch File: `mapping.launch.py`

### 1.1 Pre-launch Configuration Block

This portion defines **runtime variables**, **file paths**, and **resource references** that other nodes will use later in the launch.  
It does not start any processes, it only prepares configuration values.

#### a) LaunchConfiguration handles
```python
base_frame  = LaunchConfiguration('base_frame')
odom_frame  = LaunchConfiguration('odom_frame')
max_range   = LaunchConfiguration('max_range')
simulate    = LaunchConfiguration('simulate')
start_lidar = LaunchConfiguration('start_lidar')
display_map_live = LaunchConfiguration('display_map_live')
```
These variables act as **deferred handles** to launch arguments.
Their actual values are determined when the launch command runs (either from CLI or default arguments).
This design enables flexible configuration for both real hardware and simulation setups.

**Reusable:** This structure is a standard pattern for ROS 2 launch files.  
**Specific:** Only the argument names and default values.

---

#### b) Package share and URDF loading
```python
pkg_share  = get_package_share_directory('x3plus_mapping_bringup')
urdf_file  = os.path.join(pkg_share, 'urdf', 'yahboomcar_X3plus.urdf')
with open(urdf_file, 'r') as f:
    robot_description = f.read()
```
This block locates the package's **share directory**, constructs the path to the URDF file, and loads it into the `robot_description` variable.
The URDF data is later passed to `robot_state_publisher` to publish the robot's TF tree.

**Reusable:** The technique is fully generic for any robot description.  
**Specific:** The package name and URDF filename are unique to the X3 Plus.

---

#### c) RViz configuration reference
```python
rviz_cfg = PathJoinSubstitution([
    FindPackageShare('x3plus_mapping_bringup'), 'rviz', 'gmapping_view.rviz'
])
```
Constructs a **deferred path substitution** for the RViz configuration file.
This ensures the correct path resolves both in development and after installation.
Later, this path is passed to the `rviz2` node to open RViz with a predefined layout.

**Reusable:** Yes, standard for projects that include visualization presets.  
**Specific:** Only the `.rviz` file name and location differ.

---

#### d) LiDAR parameter file
```python
ydlidar_params_file = os.path.join(
    get_package_share_directory('x3plus_lidar_bringup'),
    'params',
    'tg30.yaml',
)
```
Resolves the path to the **YDLIDAR driver's parameter file**, which defines hardware-specific configuration (e.g., port, baud rate, scan frequency).

**Reusable:** The approach is reusable for any sensor driver.  
**Specific:** The package name and YAML file are device-specific.

---

### 1.2 Launch Arguments
```python
DeclareLaunchArgument('base_frame', default_value='base_footprint', description='...'),
DeclareLaunchArgument('odom_frame', default_value='odom', description='...'),
DeclareLaunchArgument('max_range', default_value='30.0', description='...'),
DeclareLaunchArgument('simulate', default_value='false', description='...'),
DeclareLaunchArgument('start_lidar', default_value='true', description='...'),
DeclareLaunchArgument('range_min', default_value='0.18', description='...'),
DeclareLaunchArgument('range_max', default_value='10.0', description='...'),
DeclareLaunchArgument('display_map_live', default_value='false', description='...'),
```
Defines the adjustable parameters available from the CLI.
These arguments control frame naming, LiDAR range limits, SLAM range thresholds, and simulation behavior.
They make the launch modular and configurable without editing the source code.

**Reusable:** Standard launch argument pattern.  
**Specific:** Default values tuned for the X3 Plus platform.

---

### 1.3 TF Setup (URDF and robot_state_publisher)
```python
Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'robot_description': robot_description}],
)
```
Loads the URDF data into the `robot_state_publisher` node, which broadcasts the robot's TF tree (`odom` &rarr; `base_link` &rarr; `laser_link`).
The TF tree ensures correct spatial alignment for mapping and visualization.
This must be running before SLAM or RViz so that all coordinate frames align correctly.

**Reusable:** Always required for robots with a URDF model.  
**Specific:** Only the URDF content and frame names are robot-dependent.

---

### 1.4 Joint State Publisher (Simulation Only)
```python
Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    condition=IfCondition(simulate),
)
```
Publishes fake joint states when running in simulation, allowing RViz to animate the robot model even without hardware feedback.

**Reusable:** Common for simulation setups.  
**Specific:** Depends on the joints defined in the URDF.

---

### 1.5 LiDAR Driver Node
```python
Node(
    package='ydlidar_ros2_driver',
    executable='ydlidar_ros2_driver_node',
    parameters=[ydlidar_params_file, {
        'range_min': LaunchConfiguration('range_min'),
        'range_max': LaunchConfiguration('range_max'),
    }],
    condition=IfCondition(start_lidar),
)
```
Starts the YDLIDAR TG30 driver if `start_lidar` is set to `true`.
It provides the `/scan` topic used by SLAM and mapping.
It uses the TG30 YAML and applies `range_min` and `range_max` overrides from launch arguments to define how the LiDAR operates.

**Reusable:** Generic structure for any LiDAR driver.  
**Specific:** File paths, frame IDs, and parameter details depend on the hardware.

---

### 1.6 Odometry Integration Node
```python
Node(
    package='x3plus_mapping_bringup',
    executable='odom_integrator',
    name='odom_integrator',
)
```
Runs the `odom_integrator` node that integrates `/cmd_vel` commands into odometry data and publishes the `odom` &rarr; `base_link` transform.
This allows mapping to function even without physical encoders.

**Reusable:** Yes, conceptually portable to any differential drive robot.  
**Specific:** Drift and frame names vary by setup.

> See [**Section 2 Node: `odom_integrator.py`**](#2-node-odom_integratorpy) for an in-depth explanation of the node's behavior.
---

### 1.7 SLAM (GMapping)
```python
Node(
    package='slam_gmapping',
    executable='slam_gmapping',
    parameters=[{
        'maxUrange': max_range,
        'linearUpdate': 0.2,
        'angularUpdate': 0.1,
        'temporalUpdate': 1.0,
        'occ_thresh': 0.25,
    }],
)
```
Launches the GMapping algorithm, which fuses laser scans and odometry to produce a 2D occupancy map.
Parameter tuning controls map update frequency and accuracy.

**Reusable:** The structure is reusable, parameters depend on robot speed and sensor range.  
**Specific:** Tuning values are tailored to the robot's motion profile and LiDAR.

---

### 1.8 Visualization (RViz)
```python
Node(
    package='rviz2',
    executable='rviz2',
    condition=IfCondition(display_map_live),
    arguments=['-d', rviz_cfg],
)
```
Optionally starts RViz using the predefined configuration file when `display_map_live` is true.
The configuration shows the LiDAR scan, TF tree, and generated map in real time.

**Reusable:** Yes, the pattern applies to any visualization setup.  
**Specific:** Only the RViz configuration path differs.

---

### 1.9 Simulation Condition
```python
IfCondition(simulate) # Node runs only if simulate:='true'
UnlessCondition(simulate) # Skipped if simulate:='true'
```
Determines whether to include simulation-only components such as `joint_state_publisher` or a fake odometry source.  
This condition allows safe testing and visualization without hardware.  

**Reusable:** Standard simulation pattern in ROS 2.  
**Specific:** Which nodes are included in simulation mode.  

---

## 2) Node: `odom_integrator.py` 

### 2.1 Parameters and Frames
```python
self.declare_parameter('odom_frame', 'odom')
self.declare_parameter('base_frame', 'base_link')
```
Defines the reference (`odom`) and child (`base_link`) frames used for publishing odometry and TF.
Parameterization makes this node adaptable across platforms.

---

### 2.2 Interfaces and Timer
```python
self.sub = self.create_subscription(Twist, '/cmd_vel', self.on_twist, 10)
self.pub = self.create_publisher(Odometry, '/odom', 10)
self.br = TransformBroadcaster(self)
self.create_timer(0.02, self.on_timer)
```
Subscribes to `/cmd_vel` for velocity commands, publishes `/odom`, and regularly updates TF transforms at 50 Hz.
These interfaces use standard ROS 2 APIs and are portable.

---

### 2.3 Velocity Handling
```python
def on_twist(self, msg: Twist):
    self.vx = float(msg.linear.x)
    self.vw = float(msg.angular.z)
```
Stores the latest velocity commands to be integrated during the next timer cycle.

---

### 2.4 Odometry Integration and Publishing
```python
def on_timer(self):
    """Periodic callback to integrate velocities into pose and publish odometry."""
```
This function runs at a fixed rate to update the robot's estimated position and orientation based on its last measured linear and angular velocities.  
It follows a simple **2D unicycle motion model**:

1. **Time integration:** Computes the elapsed time (`dt`) since the last update to ensure consistent motion calculation.  
2. **Pose update:** Integrates the robot's forward velocity and angular velocity to estimate its new `x`, `y`, and `yaw` pose.  
3. **Orientation conversion:** Converts the yaw angle into a quaternion format required by ROS 2 messages.  
4. **TF broadcast:** Publishes a transform (`odom` &rarr; `base_link`) so other nodes can interpret the robot's current position in the global frame.  
5. **Odometry message:** Publishes an `Odometry` message with the updated pose and velocity, which can be used by other nodes such as mapping, localization, or navigation.

This periodic integration provides a continuous estimate of motion suitable for simulation or robots without wheel encoders.  
While simple and effective, it does not include sensor fusion, so drift will accumulate over time.


## 3) Quick Commands

```bash
use_robot # Always run in a fresh terminal

# Show available launch arguments
ros2 launch x3plus_mapping_bringup mapping.launch.py --show-args

# Start mapping with live RViz visualization
ros2 launch x3plus_mapping_bringup mapping.launch.py display_map_live:='true'

# Use external LiDAR driver instead of launching it here
ros2 launch x3plus_mapping_bringup mapping.launch.py start_lidar:='false'
```

---
