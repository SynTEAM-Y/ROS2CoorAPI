# sim_gazebo_bringup

Gazebo simulation bringup package for the x3plus robot. This package provides launch files and world configurations to start a Gazebo simulation with the x3plus robot.

## Quick Start - Running the Simulation

### ⚡ Fastest Way (RViz-Only - No Gazebo Required)
```bash
ros2 launch sim_gazebo_bringup robot_rviz.launch.py
```

### 🤖 Full Gazebo Simulation (Recommended)
```bash
ros2 launch sim_gazebo_bringup gazebo.launch.py
```

### 🎮 Gazebo Without RViz Overlay
```bash
ros2 launch sim_gazebo_bringup gazebo.launch.py use_rviz:=false
```

## Overview

This package includes:
- **Launch files**: Python launch files to start Gazebo simulation with x3plus
- **Fallback launch**: RViz-only visualization without Gazebo (for testing/dependencies missing)
- **World files**: SDF world definitions (empty world, office world)
- **Configuration**: RViz visualization setup

## Status ✅

- **RViz Visualization**: ✅ Fully working (no dependencies)
- **Gazebo Simulation**: ✅ Fully working with mesh loading
- **Robot Spawning**: ✅ Robot successfully spawns in Gazebo
- **Physics Simulation**: ✅ Gazebo physics engine active
- **Joint State Publishing**: ✅ All joints publishing correctly
- **Mesh Files**: ✅ All robot meshes loading correctly

The simulation is **production-ready** and can be used for:
- Robot visualization and inspection
- Joint control testing
- Sensor simulation (with physics)
- ROS node development and testing

## Prerequisites

- ROS 2 Humble
- yahboomcar_description package (contains x3plus URDF and RViz configs)
- robot_state_publisher
- rviz2

### For Gazebo Simulation (Optional)

To use the full Gazebo simulation features, you need `ros-gz-sim`:

```bash
sudo apt-get install ros-humble-ros-gz-sim
```

If you encounter package conflicts, you may need to check your ROS installation or consult the [Gazebo ROS 2 documentation](https://gazebosim.org/docs/latest/ros2_getting_started/).

## Installation & Build

1. Ensure this package is in your workspace:
   ```bash
   ls ~/ROS2Coordination/robot_workspace/x3plus_ws/src/sim_gazebo_bringup
   ```

2. Build the workspace:
   ```bash
   cd ~/ROS2Coordination/robot_workspace/x3plus_ws
   colcon build --packages-select sim_gazebo_bringup
   source install/setup.bash
   ```

## Usage - Detailed Guide

### Option 1: RViz-Only Visualization (⭐ No External Dependencies)

Perfect for testing the robot model without needing Gazebo installed:

```bash
ros2 launch sim_gazebo_bringup robot_rviz.launch.py
```

**What this does:**
- ✅ Loads the x3plus robot model in RViz
- ✅ Opens Joint State Publisher GUI for interactive joint control
- ✅ No additional package dependencies required
- ✅ Great for model testing and development

**Expected visual output:**
- RViz window opens showing the x3plus robot model
- Joint State Publisher window allows you to adjust each joint angle
- All robot segments are visible with proper colors and textures

### Option 2: Full Gazebo Simulation (🎮 Recommended for Physics)

Complete simulation with physics engine and realistic joint behavior:

```bash
ros2 launch sim_gazebo_bringup gazebo.launch.py
```

**What this does:**
- ✅ Starts the Gazebo simulator (ignition-gazebo)
- ✅ Spawns the x3plus robot with proper mesh loading
- ✅ Launches RViz for visualization
- ✅ Enables simulated time for accurate ROS callbacks
- ✅ Ground plane and lighting setup

**Expected visual output:**
- Gazebo window shows empty world with ground plane
- x3plus robot appears in the center of the simulation
- RViz window shows the same robot with joint visualization
- You can interact with the robot using ROS commands

**Performance tip:** If you want faster startup, disable RViz:
```bash
ros2 launch sim_gazebo_bringup gazebo.launch.py use_rviz:=false
```

### Option 3: Gazebo with Custom Configuration

```bash
# Disable simulated time
ros2 launch sim_gazebo_bringup gazebo.launch.py use_sim_time:=false

# Disable both RViz and use custom world
ros2 launch sim_gazebo_bringup gazebo.launch.py use_rviz:=false use_sim_time:=false
```

## Launch File Arguments

### gazebo.launch.py

| Argument | Default | Description |
|----------|---------|-------------|
| `use_rviz` | `true` | Argument accepted but RViz node is **not launched** from this file (Humble `IfAction` limitation). Launch RViz separately with `robot_rviz.launch.py` |
| `use_sim_time` | `true` | Use simulated time (required for Gazebo) |

### robot_rviz.launch.py

| Argument | Default | Description |
|----------|---------|-------------|
| `use_sim_time` | `false` | Use simulated time |

## Verifying the Simulation Works

After launching, verify the robot is properly simulated:

### Check Topics
```bash
# In a new terminal, check published topics
ros2 topic list

# Gazebo mode: you should see at minimum:
# /joint_states_raw   ← from Ignition JointStatePublisher via ros_gz_bridge (18 joints)
# /joint_states       ← from gripper_mimic_relay (13 joints; 5 mimic joints stripped)
# /odom               ← from Ignition DiffDrive plugin via ros_gz_bridge
# /imu                ← from Ignition IMU plugin via ros_gz_bridge
# /cmd_vel            ← received by Ignition DiffDrive plugin
# /clock              ← Ignition simulation clock
# /robot_description  ← URDF string
# /tf, /tf_static     ← robot TF tree (5 mimic joints computed by RSP via URDF <mimic>)
```

### Monitor Joint States
```bash
# Watch joint state updates in real-time
ros2 topic echo /joint_states
```

### Check ROS Transforms
```bash
# Verify robot transforms are being published
ros2 tf2_tools.py view_frames

# Or list available frames
ros2 frame list
```

### Send Commands to the Robot
```bash
# Publish velocity commands to move the robot
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.1}}'
```

## Available Worlds

- **empty.sdf** - Minimal world with ground plane, good for testing
- **office.sdf** - Office environment with obstacles

## Package Structure

```
sim_gazebo_bringup/
├── CMakeLists.txt              # Build configuration
├── package.xml                 # Package metadata
├── README.md                   # This file
├── launch/
│   ├── gazebo.launch.py        # Full Gazebo simulation (ros-gz-sim required)
│   │                           #   - XACRO→URDF→SDF pipeline
│   │                           #   - ros_gz_bridge for odom/joint_states/imu/cmd_vel
│   │                           #   - gripper_mimic_relay node
│   ├── gazebo.launch.xml       # Alternative XML launch (simpler, no XACRO processing)
│   ├── robot_rviz.launch.py    # RViz + diff_drive_simulator + map (no Gazebo needed)
│   ├── robot_rviz_headless.launch.py  # Headless RViz fallback (no GUI, pre-built URDF)
│   ├── sub_cmd_vel.py          # Test subscriber: prints /cmd_vel for 5 seconds
│   ├── run_rviz_preload.sh     # Wrapper: sets LD_PRELOAD before launching robot_rviz
│   └── run_rviz_wrapper.sh     # Advanced wrapper: X11 socket forwarding via socat
├── worlds/
│   ├── empty.sdf               # Minimal world (ODE physics, ground plane, sun)
│   └── office.sdf              # Office environment with 1×1×1 m box obstacle at (2,2)
└── rviz/
    └── gazebo_view.rviz        # Enhanced RViz config (fixed frame: base_footprint)
```
│   ├── empty.sdf               # Minimal world file
│   └── office.sdf              # Office environment world
└── rviz/                       # RViz configuration files (optional)
```

## Quick Troubleshooting

### "package 'ros_gz_sim' not found"

This means Gazebo Sim is not installed. You have two options:

**Option A: Use RViz-only** (no Gazebo required)
```bash
ros2 launch sim_gazebo_bringup robot_rviz.launch.py
```

**Option B: Install Gazebo**
```bash
sudo apt-get install ros-humble-ros-gz-sim
```

### Robot not displaying in RViz

1. Ensure `yahboomcar_description` package is installed and built
2. Run with verbose output to see errors:
   ```bash
   ROS_LOG_LEVEL=debug ros2 launch sim_gazebo_bringup robot_rviz.launch.py
   ```
3. Verify the URDF file exists:
   ```bash
   ls $(ros2 pkg prefix yahboomcar_description)/share/yahboomcar_description/urdf/
   ```

### RViz shows "No transform" errors with Gazebo

- Make sure `use_sim_time:=true` is set (it's the default in gazebo.launch.py)
- Check that Gazebo is running: `ps aux | grep gazebo`
- Verify `/clock` topic is being published: `ros2 topic list | grep clock`

### RViz window crashes with "symbol lookup error"

This is a snap library compatibility issue with RViz. You can:
- Use Gazebo visualization only by disabling RViz:
  ```bash
  ros2 launch sim_gazebo_bringup gazebo.launch.py use_rviz:=false
  ```
- Or use the RViz-only launch instead:
  ```bash
  ros2 launch sim_gazebo_bringup robot_rviz.launch.py
  ```

### Gazebo window shows warnings about SDF version

These are non-critical warnings about SDF format conversion. The simulation should still work correctly. If the robot doesn't appear, check the console output for "Unable to find file" errors.

### Robot entity not being created in Gazebo

If you see `/robot_description` topic but robot doesn't spawn:
1. Check that `robot_state_publisher` started successfully
2. Verify meshes are being found (look for "OK creation of entity" message)
3. Try running with verbose output:
   ```bash
   ROS_LOG_LEVEL=debug ros2 launch sim_gazebo_bringup gazebo.launch.py use_rviz:=false
   ```

## Creating Custom Worlds

To add a custom Gazebo world:

1. Create a new `.sdf` file in the `worlds/` directory:
   ```bash
   cp worlds/empty.sdf worlds/my_world.sdf
   ```

2. Edit `my_world.sdf` according to [Gazebo SDF documentation](https://gazebosim.org/docs/latest/building_models/)

3. Use in launch file (after rebuilding):
   ```bash
   ros2 launch sim_gazebo_bringup gazebo.launch.py
   # The world will be loaded from worlds/ folder
   ```

## Odometry

### Gazebo Mode (`gazebo.launch.py`)

Odometry is computed by the **Ignition DiffDrive plugin** embedded in the URDF.
It integrates wheel encoder data and publishes on Ignition transport, then
`ros_gz_bridge` relays it to `/odom` (nav_msgs/Odometry):

```
/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry
```

The plugin uses standard differential drive integration:

| Variable | Formula |
|----------|---------|
| Linear velocity | `v = (v_R + v_L) / 2` |
| Angular velocity | `ω = (v_R − v_L) / L` where L = **0.2128 m** |
| Position X | `x += v·cos(θ)·Δt` |
| Position Y | `y += v·sin(θ)·Δt` |
| Heading | `θ += ω·Δt` |

> **Note**: Skid-steer robots accumulate yaw error in odom due to wheel slip.
> The `manual_control` node therefore uses `/imu` (ground-truth orientation from
> Ignition's IMU plugin) for 90° turn feedback when Gazebo is running.

### RViz-Only Mode (`robot_rviz.launch.py`)

The `diff_drive_simulator` node (from `x3plus_examples`) replicates the same
differential drive math in software. It subscribes to `/cmd_vel` and publishes:
- `/odom` (nav_msgs/Odometry)
- TF transform: `odom → base_footprint`

A static transform `map → odom` (identity) is also published, allowing the
map to display at the robot's starting position.

### Topic Summary

| Topic | Direction | Mode |
|-------|-----------|------|
| `/odom` | Published | Both (Ignition or diff_drive_simulator) |
| `/imu` | Published | Gazebo only (Ignition IMU plugin → bridge) |
| `/cmd_vel` | Subscribed | Both (Ignition DiffDrive or diff_drive_simulator) |

---

## Advanced Topics

### Adding Wheels and Differential Drive (✅ NOW INCLUDED!)

The x3plus robot now includes **wheel links and differential drive** configured for Gazebo simulation!

#### Configurable Wheel Parameters

All wheel specifications are **clearly commented** in the source code for easy modification:

**Location:** `src/yahboomcar_description/urdf/yahboomcar_X3plus.urdf.xacro` (relative to workspace root)

**📖 📖 FULL REFERENCE: See [WHEEL_PARAMETERS_REFERENCE.md](WHEEL_PARAMETERS_REFERENCE.md) for detailed explanations!**

Quick parameter table:

| Parameter | Search For | Default | What It Does |
|-----------|-----------|---------|-------------|
| **Wheel Radius** | `WHEEL RADIUS:` | 0.04 m | Size of wheels. Change both visual and collision |
| **Wheel Width** | `WHEEL WIDTH:` | 0.015 m | Thickness of wheels |
| **Wheel Mass** | `mass="0.1"` | 0.1 kg | Weight per wheel. Heavier = better grip |
| **Wheel Forward Pos** | front: 0.1054, back: -0.1146 | from mesh | Distance forward from chassis |
| **Wheel Separation** | 0.2128 m (2×0.1064) | 0.2128 m | Track width between wheel centers |
| **Motor Torque** | `effort="10"` | 10 N⋅m | Maximum power output |
| **Motor Speed** | `velocity="2"` | 2 rad/s | Max rotation speed |
| **Motor Acceleration** | `max_wheel_accel` | 1.0 rad/s² | How quickly motor speeds up |
| **Wheel Friction** | `mu1>` and `mu2>` | μ1=1.0, μ2=0.05 | Rolling and lateral grip (asymmetric for skid-steer) |

#### How to Customize Wheels

Simply edit the XACRO file and change the commented parameter values:

```bash
# 1. Open the URDF file
code src/yahboomcar_description/urdf/yahboomcar_X3plus.urdf.xacro

# 2. Find sections marked with "CONFIGURABLE" comments
# 3. Change parameter values (radius, mass, positions, torque, etc.)
# 4. Save and rebuild:

cd ~/ROS2Coordination/robot_workspace/x3plus_ws
colcon build --packages-select yahboomcar_description
source install/setup.bash

# 5. Test in Gazebo:
ros2 launch sim_gazebo_bringup gazebo.launch.py use_rviz:=false
```

#### Testing Differential Drive

After rebuilding, test the robot movement:

```bash
# Terminal 1: Launch Gazebo with the robot
ros2 launch sim_gazebo_bringup gazebo.launch.py use_rviz:=false

# Terminal 2: Move forward
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.5}}'

# Terminal 3: Rotate
ros2 topic pub /cmd_vel geometry_msgs/Twist '{angular: {z: 1.0}}'

# Terminal 4: Monitor wheel joint states
ros2 topic echo /joint_states | grep wheel
```

**Expected Behavior:**
- ✅ Four wheel positions visible in Gazebo on the sides of the robot
- ✅ Wheels spin when movement commands are sent
- ✅ Forward velocity makes both wheels spin the same direction
- ✅ Angular velocity makes wheels spin in opposite directions
- ✅ Robot moves smoothly across the ground plane

### 6-DOF Robotic Arm and Gripper Configuration (✅ FULLY CONFIGURED!)

The x3plus robot includes a **fully simulated 6-DOF robotic arm** with a parallel gripper mechanism.

#### Architecture Overview

**Arm Joints** (5 revolute joints):
- `arm_joint1` through `arm_joint5` — 5-DOF arm control
- Each joint has an Ignition `JointPositionController` (PID) tuned per-joint:
  - arm_joint1: P=15, D=2, cmd_max=8
  - arm_joint2: P=25, D=3, cmd_max=15  (heaviest joint, takes shoulder load)
  - arm_joint3: P=20, D=2.5, cmd_max=12
  - arm_joint4: P=15, D=2, cmd_max=8
  - arm_joint5: P=10, D=1.5, cmd_max=6
- Each joint independently controllable via `/arm_joint{N}_cmd_pos` topics (std_msgs/Float64)
- Joint damping=1.0, friction=0.5 in URDF so reaction torques don't shake the base

**Gripper Mechanism** (6 links, 1 actuated joint):
- `grip_joint` — main gripper control (range: -1.54 to 0 rad)
  - 0 = closed (servo center 90°), -1.54 = fully open (matches manufacturer SRDF)
  - PID tuned for tiny inertia: P=200, D=5, cmd_max=100 N·m
  - Joint damping=0.005, friction=0.001 (rlink1 has ~1e-7 kg.m² inertia; standard
    arm-joint damping/friction over-damps it and the PID can't move it)
- **Parallel linkage**: 5 mimic continuous joints follow `grip_joint` via the URDF `<mimic>` tag
  - `rlink_joint2`, `rlink_joint3` (right finger; multipliers −1, +1)
  - `llink_joint1`, `llink_joint2`, `llink_joint3` (left finger; −1, +1, −1)
  - **No physics controller** — gravity disabled on these links so they don't flop
  - **`gripper_mimic_relay`** strips them from `/joint_states_raw` so RSP computes them
    via the URDF `<mimic>` relationship (RSP only honours `<mimic>` when the joint is
    absent from the incoming JointState message).

**How the mimic chain works**:
```
Ignition physics → /joint_states_raw  (18 joints, 5 mimic frozen at 0)
                          ↓
         gripper_mimic_relay  (strips 5 mimic joints)
                          ↓
                  /joint_states  (13 joints)
                          ↓
         robot_state_publisher
                          ↓
         TF tree  (mimic joint TFs computed from grip_joint via URDF)
```

#### Testing the Gripper

```bash
# Terminal 1: Launch Gazebo with robot
ros2 launch sim_gazebo_bringup gazebo.launch.py use_rviz:=false

# Terminal 2: Control the gripper
# Close gripper completely
ros2 topic pub --once /grip_joint_cmd_pos std_msgs/msg/Float64 "{data: 0.0}"

# Open gripper fully
ros2 topic pub --once /grip_joint_cmd_pos std_msgs/msg/Float64 "{data: -1.54}"

# Half-open position
ros2 topic pub --once /grip_joint_cmd_pos std_msgs/msg/Float64 "{data: -0.77}"

# Monitor gripper state
ros2 topic echo /joint_states | grep -E "grip_joint|rlink|llink"
```

#### Gripper Performance Notes

1. **Per-joint effort limits** (URDF `<limit effort>`):
   - Arm joints: 100 N·m (revolute_joint macro)
   - `grip_joint`: 100 N·m (inlined to allow lower damping)
   - Mimic linkage joints: continuous, no effort limit (they're passive in physics)

2. **PID gains tuned to physics**:
   - Arm joints: gains scaled to each joint's load (joint2 strongest, joint5 lightest)
   - `grip_joint`: P=200, cmd_max=100 — high because rlink1 mass is only ~1.2 mg and
     even slight ODE friction blocks motion if PID is too gentle

3. **Finger collision**:
   - All finger links (`rlink1–3`, `llink1–3`) have the standard collision meshes
     from the URDF
   - Gravity is **disabled** on these links so they don't drag down `arm_link5`

#### Object Grasping in Simulation

The finger collision meshes are enabled by default (loaded by the `common_link`
macro). Grasping behaviour in Ignition is limited because:

  - The 5 mimic joints have **no physics controller** — they don't apply grip force
    on objects, they only follow `grip_joint` kinematically (via RSP, not physics).
  - `grip_joint` rotates `rlink1` only; the rest of the linkage is decoupled in physics.

For true Gazebo grasping (force closure), you would need to either:
  1. Enforce the mimic constraint via a physics joint loop closure (DART/ODE 4-bar
     linkage) — attempted in an earlier revision but caused ODE instability with the
     tiny finger inertias
  2. Add JointPositionControllers to each mimic joint and command them in lock-step
     with `grip_joint` (deprecated approach — caused the same instability)

The current setup is optimised for **visual** pick-and-place (RViz + Gazebo),
not physical grasp force.

#### MoveIt Integration

The gripper is fully configured for MoveIt motion planning:

**Configuration location**: `src/x3plus_config/config/yahboomcar_X3plus.srdf.xacro`

**Motion Groups**:
- `arm_group` - Contains arm_joint1 through arm_joint5
- `gripper_group` - Contains all 6 gripper links
- End effector attached to `arm_link5`

**Pre-defined Poses**:
- Arm: `up`, `down`, `init`
- Gripper: `open`, `close`

```bash
# Test with MoveIt (if configured)
ros2 launch x3plus_config moveit.launch.py
```

#### Detailed Configuration Guides

For in-depth explanations and advanced setups:

- **[Wheel & Differential Drive Setup Guide](WHEEL_DIFFERENTIAL_SETUP.md)** - Comprehensive guide with theory and troubleshooting
- **[Wheel Setup Example Code](WHEEL_SETUP_EXAMPLE.md)** - Ready-to-use code blocks and examples

## Dependencies

**Required:**
- `robot_state_publisher` - Publishes robot state/transforms
- `rviz2` - Visualization tool
- `yahboomcar_description` - Robot URDF and visualization configs

**Optional (for Gazebo simulation):**
- `ros_gz_sim` - Gazebo Sim integration (use `robot_rviz.launch.py` as fallback)

## License

Apache 2.0

## Maintainer

Created for x3plus robot simulation in ROS2 Coordination workspace

