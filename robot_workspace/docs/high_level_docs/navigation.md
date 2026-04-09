# High-Level Documentation of `navigation.launch.py`

This document provides a high-level overview of how the navigation launch file sets up the TF tree, LiDAR driver, and reactive navigation logic.  
It also explains how `reactive_nav.py` processes LiDAR data and makes real-time motion decisions.  
Each section highlights the structure, purpose, and reusability of the components involved.

**References:**
- [`navigation.launch.py`](../../x3plus_ws/src/x3plus_mapping_bringup/launch/navigation.launch.py)
- [`reactive_nav.py`](../../x3plus_ws/src/x3plus_mapping_bringup/x3plus_mapping_bringup/reactive_nav.py)

---

## 1) Launch File: `navigation.launch.py`

### 1.1 Pre-launch Configuration Block

This portion defines **runtime variables**, **file paths**, and **resource references** that other nodes will use later in the launch.  
It does not start any processes, it only prepares configuration values.

#### a) LaunchConfiguration handles
```python
base_frame  = LaunchConfiguration('base_frame')
odom_frame  = LaunchConfiguration('odom_frame')
move_robot  = LaunchConfiguration('move_robot')
simulate    = LaunchConfiguration('simulate')
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


#### c) LiDAR parameter file
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
DeclareLaunchArgument('move_robot', default_value='true', description='...'),
DeclareLaunchArgument('simulate', default_value='false', description='...'),
```
Defines the adjustable parameters available from the CLI.
These arguments control frame naming and simulation behavior.
They make the launch modular and configurable without editing the source code.

**Reusable:** Standard launch argument pattern.  
**Specific:** Default values tuned for the X3 Plus platform.

---

### 1.3 TF and URDF Setup
```python
Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher_x3',
    output='screen',
    parameters=[{'robot_description': robot_description}],
)
```
Starts the `robot_state_publisher` node, which reads the robot's URDF and continuously publishes TF transforms (e.g., `odom` &rarr; `base_link` &rarr; `laser_link`).  
This ensures that all frames are properly aligned before visualization or navigation begins.  

**Reusable:** Always necessary for robots using a URDF model.  
**Specific:** The URDF file and frame naming convention.

---

### 1.4 Joint State Publisher (simulation only)
```python
Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    name='joint_state_publisher',
    output='screen',
    condition=IfCondition(simulate),
)
```
Publishes synthetic joint states in simulation so the URDF animates correctly in RViz.  This is only enabled when `simulate:='true'`.

**Reusable:** Common pattern for simulated bringups.  
**Specific:** The actual joints displayed come from the robot's URDF.  

---

### 1.5 Simulated Odometry Integrator
```python
Node(
    package='x3plus_mapping_bringup',
    executable='odom_integrator',
    name='odom_integrator',
    output='screen',
    parameters=[{'odom_frame': odom_frame, 'base_frame': base_frame}],
    condition=IfCondition(simulate),
)
```
Integrates `/cmd_vel` to publish `/odom` and the `odom` &rarr; `base_frame` TF in simulation. Useful when no real wheel odometry is available.  

**Reusable:** Can be reused in any simulation or robot that estimates motion through basic velocity integration.  
**Specific:** Frame names and expected drift characteristics.  

---


### 1.6 LiDAR Driver
```python
Node(
    package='ydlidar_ros2_driver',
    executable='ydlidar_ros2_driver_node',
    name='ydlidar',
    output='screen',
    parameters=[ydlidar_params_file],
    condition=IfCondition(move_robot),
)
```
Launches the LiDAR driver, providing the `/scan` topic used by navigation and obstacle avoidance.  
The node runs only when `move_robot` is enabled, avoiding unnecessary startup in test or simulation modes.  

**Reusable:** The pattern applies to any LiDAR driver, only package and parameter file paths differ.  
**Specific:** TG30 sensor settings and YAML configuration file.  

---

### 1.7 Reactive Navigation Node
```python
Node(
    package='x3plus_mapping_bringup',
    executable='reactive_nav',
    name='reactive_nav',
    output='screen',
    parameters=[{
        ... # Parameters
    }],
)
```
Runs the main navigation logic. The node subscribes to `/scan`, analyzes obstacles, and publishes `/cmd_vel` velocity commands.  
It supports configurable thresholds for forward motion, turning, and stopping behavior.  

**Reusable:** Fully reusable on any robot exposing `/scan` and consuming `/cmd_vel`.  
**Specific:** Speed and distance thresholds tuned to the robot's physical limits.  

> See [**Section 2 Node: `reactive_nav.py`**](#2-node-reactive_navpy) for an in-depth explanation of the node's behavior.

---

### 1.8 Base Bridge to Motors (real robot only)
```python
Node(
    package='x3plus_examples',
    executable='rosmaster_base_bridge',
    name='rosmaster_base_bridge',
    output='screen',
    parameters=[{
        ... # Parameters
    }],
    condition=UnlessCondition(simulate),
)
```
Bridges `/cmd_vel` to the Rosmaster base hardware with limits, deadbands, acceleration limiting, and timeouts for safety.  Disabled when `simulate:='true'`.  

**Reusable:** The bridge concept is reusable if adapted to another base.  
**Specific:** Parameters and the hardware driver behind this node are Rosmaster-specific.  

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

## 2) Node: `reactive_nav.py`

### 2.1 Node Initialization and Parameters
```python
self.declare_parameter('move_robot', True)
self.declare_parameter('forward_speed', 0.15)
self.declare_parameter('turn_speed', 0.8)
self.declare_parameter('front_clear', 0.5)
self.declare_parameter('hard_stop', 0.2)
self.declare_parameter('min_valid_ratio', 0.05)
self.declare_parameter('left_fov_deg', 90.0)
self.declare_parameter('right_fov_deg', -90.0)
self.declare_parameter('front_half_deg', 20.0)
```
Initializes the node and declares all key parameters that control motion behavior.  
Each value defines how the robot reacts to detected obstacles:  
- **front_clear**: Minimum safe distance before slowing down or turning.  
- **hard_stop**: Distance threshold for an immediate stop.  
- **min_valid_ratio**: Minimum proportion of valid LiDAR points to ensure reliable data.  
- **forward_speed / turn_speed**: Define the robot's movement dynamics.  

**Reusable:** Fully reusable with parameter adjustments.  
**Specific:** Default tuning values depend on the robot's size and speed.  

---

### 2.2 Interfaces (Subscriptions and Publishers)
```python
self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
```
Creates the core communication channels for navigation:  
- **Publishes** to `/cmd_vel` (geometry_msgs/Twist) for velocity commands.  
- **Subscribes** to `/scan` (sensor_msgs/LaserScan) for obstacle data.  

**Reusable:** Yes, uses standard ROS 2 topics.  
**Specific:** None unless topics are remapped.  

---

### 2.3 Scan Handling & Sector Extraction
```python
def on_scan(self, msg: LaserScan):
    """Callback that stores the most recent LiDAR scan message."""
    self.scan = msg
```
Captures the latest `/scan` message and stores it for use by the control loop. Keeps the subscriber callback lightweight and decouples I/O from the decision logic.

```python
def sector(self, msg: LaserScan, deg_min, deg_max):
    """Extracts a slice of the LiDAR scan within a given angular range (in degrees),
    filters valid distances, and returns the minimum distance and ratio of valid readings.
    """
    a0 = math.degrees(msg.angle_min)
    step = math.degrees(msg.angle_increment)
    n = len(msg.ranges)

    # Convert requested degree bounds into index range
    i0 = int(round((deg_min - a0) / step))
    i1 = int(round((deg_max - a0) / step))
    i0 = max(0, min(n - 1, i0))
    i1 = max(0, min(n - 1, i1))
    if i0 > i1:
        i0, i1 = i1, i0

    # Filter out invalid or infinite readings
    vals = [r for r in msg.ranges[i0:i1+1] if math.isfinite(r) and r > 0.0]
    valid_ratio = len(vals) / max(1, (i1 - i0 + 1))
    mind = min(vals) if vals else float('inf')
    return mind, valid_ratio
```
Processes incoming LiDAR data by converting angular ranges into index bounds, filtering out invalid readings, and extracting key sectors (front, left, right).  
For each sector, computes the minimum distance and ratio of valid points, information used by the control loop for obstacle detection and navigation decisions.  

**Reusable:** Compatible with any standard `sensor_msgs/LaserScan`, adaptable to other robots or simulators.  
**Specific:** Sector angles and distance thresholds depend on the LiDAR’s mounting and field of view.

---

### 2.4 Decision Logic
```python
def on_timer(self):
    if front_valid < self.min_valid:
        # Insufficient data → stop

    elif front_min < self.hard_stop:
        # Immediate obstacle → stop and rotate away from the closest side

    elif front_min < self.front_clear:
        # Obstacle approaching → slow down and turn slightly

    else:
        # Path clear → move straight forward

```
Implements a reactive obstacle avoidance policy.  
The node decides between forward motion, turning, or stopping based on obstacle proximity.  
It runs continuously, allowing the robot to adapt to its environment in real time.  
The control loop runs at a fixed timer interval, ensuring consistent reaction times independent of LiDAR frequency.

**Reusable:** Core logic is platform-agnostic.  
**Specific:** Movement speeds and turning angles are hardware-dependent.  

---

## 3) Quick Commands

```bash
use_robot # Always run in a fresh terminal

# Show available launch arguments
ros2 launch x3plus_mapping_bringup navigation.launch.py --show-args

# Run on the real robot
ros2 launch x3plus_mapping_bringup navigation.launch.py

# Simulation mode (no movement commands sent)
ros2 launch x3plus_mapping_bringup navigation.launch.py simulate:='true'
```
---
