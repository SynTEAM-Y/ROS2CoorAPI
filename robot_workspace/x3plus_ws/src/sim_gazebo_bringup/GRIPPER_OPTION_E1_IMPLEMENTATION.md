# Option E1: GazeboSimSystem `<mimic>` Physics Enforcement

## Implementation Guide — Step by Step

---

### Table of Contents

1. [What This Solution Does](#1-what-this-solution-does)
2. [Architecture Change Overview](#2-architecture-change-overview)
3. [Step 1 — Update the XACRO URDF](#3-step-1--update-the-xacro-urdf)
4. [Step 2 — Update the Multi-Robot XACRO](#4-step-2--update-the-multi-robot-xacro)
5. [Step 3 — Simplify `gripper_mimic_relay.py`](#5-step-3--simplify-gripper_mimic_relaypy)
6. [Step 4 — Update CMakeLists.txt](#6-step-4--update-cmakelisttxt)
7. [Step 5 — Update `package.xml` Dependencies](#7-step-5--update-packagexml-dependencies)
8. [Step 6 — Create ros2_control Controllers YAML](#8-step-6--create-ros2_control-controllers-yaml)
9. [Step 7 — Update Launch Files](#9-step-7--update-launch-files)
10. [Step 8 — Build and Test](#10-step-8--build-and-test)
11. [How to Verify It Works](#11-how-to-verify-it-works)
12. [Rollback Plan](#12-rollback-plan)

---

### 1. What This Solution Does

The current gripper setup uses **6 independent PID controllers** (one for `grip_joint` and one for each of the 5 mimic joints). They all apply torque simultaneously to mechanically coupled links — creating a control loop bidding war that causes the jerking.

**Option E1** replaces the 5 independent mimic PID controllers with a **kinematic constraint** enforced by `gz_ros2_control/GazeboSimSystem`. The physics solver computes mimic joint positions directly from `grip_joint` — no torque fighting, no PID overshoot, no jerking.

The `<mimic>` tags that already exist in your URDF become **physics-enforced constraints** that replace the missing physical connection (`rlink3↔rlink2`) that URDF cannot represent due to its tree-only topology.

---

### 2. Architecture Change Overview

#### Before (current — broken):

```
Publish /robot_1/grip_joint_cmd_pos
            │
            ▼
    gripper_mimic_relay (50 Hz)
      ├── /robot_1/grip_master_target
      │       └── grip_joint PID (P=40, I=0, D=2, cmd_max=5)
      ├── /robot_1/llink_joint1_cmd_pos  ×(-1)
      │       └── llink_joint1 PID (P=40) ← FIGHTS grip_joint
      ├── /robot_1/llink_joint2_cmd_pos  ×(+1)
      │       └── llink_joint2 PID (P=40) ← FIGHTS grip_joint
      ├── /robot_1/llink_joint3_cmd_pos  ×(-1)
      │       └── llink_joint3 PID (P=40) ← FIGHTS grip_joint
      ├── /robot_1/rlink_joint2_cmd_pos  ×(-1)
      │       └── rlink_joint2 PID (P=40) ← FIGHTS grip_joint
      └── /robot_1/rlink_joint3_cmd_pos  ×(+1)
              └── rlink_joint3 PID (P=40) ← FIGHTS grip_joint
```

**6 controllers fighting on 1 DOF → jerking**

#### After (Option E1 — correct):

```
Publish /robot_1/grip_joint_cmd_pos  (via ros2_control controller)
            │
            ▼
  gz_ros2_control / GazeboSimSystem
      │
      ├── grip_joint → command_interface (torque applied)
      ├── llink_joint1 → state_interface ONLY (computed from <mimic>)
      ├── llink_joint2 → state_interface ONLY (computed from <mimic>)
      ├── llink_joint3 → state_interface ONLY (computed from <mimic>)
      ├── rlink_joint2 → state_interface ONLY (computed from <mimic>)
      └── rlink_joint3 → state_interface ONLY (computed from <mimic>)
                                   │
                                   ▼
                    Physics Solver: θ_mimic = θ_grip × multiplier
                    (kinematic constraint, NOT PID torque)
```

**1 torque source + 5 kinematic constraints = smooth, jerk-free motion**

---

### 3. Step 1 — Update the XACRO URDF

**File:** `scripts/yahboomcar_description/urdf/yahboomcar_X3plus.urdf.xacro`

#### 3a. Remove the 5 mimic `JointPositionController` plugins

Delete these blocks (lines 627-676 in the current file):

```xml
<!-- DELETE: All 5 mimic joint PID controllers -->
<plugin filename="ignition-gazebo-joint-position-controller-system"
        name="ignition::gazebo::systems::JointPositionController">
    <joint_name>llink_joint1</joint_name>      <!-- DELETE -->
    <topic>/llink_joint1_cmd_pos</topic>        <!-- DELETE -->
    <p_gain>40.0</p_gain> <i_gain>0.0</i_gain> <d_gain>2.0</d_gain>
    <i_max>0.2</i_max> <i_min>-0.2</i_min>
    <cmd_max>5.0</cmd_max> <cmd_min>-5.0</cmd_min>
    <initial_position>0</initial_position>
</plugin>
<!-- Same for llink_joint2, llink_joint3, rlink_joint2, rlink_joint3 -->
```

Also remove the comment block that introduces them (lines 612-626):

```xml
<!-- DELETE: entire MIMIC GRIPPER FINGER CONTROLLERS comment block -->
<!-- ==================== MIMIC GRIPPER FINGER CONTROLLERS ==================== ...
     ... until the end of the rlink_joint3 plugin -->
```

#### 3b. Keep ONLY the `grip_joint` `JointPositionController`

Keep this plugin (lines 596-611) — this is the only torque source:

```xml
<plugin filename="ignition-gazebo-joint-position-controller-system"
        name="ignition::gazebo::systems::JointPositionController">
    <joint_name>grip_joint</joint_name>
    <topic>/grip_master_target</topic>
    <p_gain>40.0</p_gain> <i_gain>0.0</i_gain>  <d_gain>2.0</d_gain>
    <i_max>0.2</i_max> <i_min>-0.2</i_min>
    <cmd_max>5.0</cmd_max> <cmd_min>-5.0</cmd_min>
    <initial_position>0</initial_position>
</plugin>
```

#### 3c. Add the `<ros2_control>` block

After the `</gazebo>` block that contains the plugins (around line 736) and before the IMU sensor section, add this block **outside** `<gazebo>` (inside `<robot>`):

```xml
<!-- ==================== ROS2_CONTROL HARDWARE INTERFACE ====================
     gz_ros2_control/GazeboSimSystem enforces URDF <mimic> constraints in
     physics for joints that have state interfaces ONLY (no command interface).
     
     grip_joint:   command + state  (the one torque source)
     llink_joint1: state ONLY      (kinematic slave via <mimic>)
     llink_joint2: state ONLY      (kinematic slave via <mimic>)
     llink_joint3: state ONLY      (kinematic slave via <mimic>)
     rlink_joint2: state ONLY      (kinematic slave via <mimic>)
     rlink_joint3: state ONLY      (kinematic slave via <mimic>)
     
     The 5 arm joints also get command+state so the ArmController can
     continue to control them directly.                                    -->
<ros2_control name="X3plusGripperSystem" type="system">
    <hardware>
        <plugin>gz_ros2_control/GazeboSimSystem</plugin>
    </hardware>

    <!-- Arm joints (command + state — same as before, but via ros2_control) -->
    <joint name="${ns}/arm_joint1">
        <command_interface name="position">
            <param name="min">-1.571</param>
            <param name="max">1.571</param>
        </command_interface>
        <state_interface name="position"/>
    </joint>
    <joint name="${ns}/arm_joint2">
        <command_interface name="position">
            <param name="min">-1.571</param>
            <param name="max">1.571</param>
        </command_interface>
        <state_interface name="position"/>
    </joint>
    <joint name="${ns}/arm_joint3">
        <command_interface name="position">
            <param name="min">-1.571</param>
            <param name="max">1.571</param>
        </command_interface>
        <state_interface name="position"/>
    </joint>
    <joint name="${ns}/arm_joint4">
        <command_interface name="position">
            <param name="min">-1.571</param>
            <param name="max">1.571</param>
        </command_interface>
        <state_interface name="position"/>
    </joint>
    <joint name="${ns}/arm_joint5">
        <command_interface name="position">
            <param name="min">-1.571</param>
            <param name="max">3.1416</param>
        </command_interface>
        <state_interface name="position"/>
    </joint>

    <!-- Gripper master joint (command + state — the one torque source) -->
    <joint name="${ns}/grip_joint">
        <command_interface name="position">
            <param name="min">-1.8</param>
            <param name="max">0.45</param>
        </command_interface>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
    </joint>

    <!-- Mimic joints (state ONLY — no command interface!
         GazeboSimSystem reads the URDF <mimic> tag and enforces
         the kinematic constraint automatically in the physics solver.
         With no command interface, no torque is applied — the solver
         simply constrains the joint position to follow the master. -->
    <joint name="${ns}/llink_joint1">
        <state_interface name="position"/>
    </joint>
    <joint name="${ns}/llink_joint2">
        <state_interface name="position"/>
    </joint>
    <joint name="${ns}/llink_joint3">
        <state_interface name="position"/>
    </joint>
    <joint name="${ns}/rlink_joint2">
        <state_interface name="position"/>
    </joint>
    <joint name="${ns}/rlink_joint3">
        <state_interface name="position"/>
    </joint>
</ros2_control>
```

#### 3d. Keep the `<mimic>` tags — they are now physics-active

The existing `<mimic>` tags in the `continuous_joint` macro remain **unchanged**. They were already in your URDF. What changes is that `GazeboSimSystem` now reads and **enforces** them instead of Ignition ignoring them.

```
<llink_joint1>  <mimic joint="grip_joint" multiplier="-1" offset="0"/>  ← STAYS
<llink_joint2>  <mimic joint="grip_joint" multiplier="+1" offset="0"/>  ← STAYS
<llink_joint3>  <mimic joint="grip_joint" multiplier="-1" offset="0"/>  ← STAYS
<rlink_joint2>  <mimic joint="grip_joint" multiplier="-1" offset="0"/>  ← STAYS
<rlink_joint3>  <mimic joint="grip_joint" multiplier="+1" offset="0"/>  ← STAYS
```

#### 3e. Update gravity settings on finger links (optional but recommended)

Re-enable gravity on the finger links. With `<mimic>` enforced by the physics solver, gravity provides natural passive damping instead of destabilizing:

```xml
<!-- BEFORE (remove gravity=0): -->
<gazebo reference="llink1"><gravity>0</gravity>...
<gazebo reference="llink2"><gravity>0</gravity>...
<gazebo reference="rlink1"><gravity>0</gravity>...
<gazebo reference="rlink2"><gravity>0</gravity>...
<gazebo reference="llink3"><gravity>0</gravity>...
<gazebo reference="rlink3"><gravity>0</gravity>...

<!-- AFTER (remove gravity override entirely, or set to 1): -->
<gazebo reference="llink1"><mu1>10.0</mu1><mu2>10.0</mu2><collision><surface><contact><collide_bitmask>0xFF</collide_bitmask></contact></surface></collision></gazebo>
<!-- gravity defaults to 1 (enabled) when the tag is removed -->
```

---

### 4. Step 2 — Update the Multi-Robot XACRO

**File:** `scripts/yahboomcar_description/urdf/yahboomcar_X3plus_multi.urdf.xacro`

Apply **exactly the same changes** as Step 1:
- Remove the 5 mimic JointPositionController plugins (lines 627-676)
- Keep only the `grip_joint` JointPositionController (lines 596-611)
- Add the `<ros2_control>` block with `${ns}` prefix

The `${ns}` parameter already handles namespacing for multi-robot. When a robot is spawned as `robot_1`, the XACRO arg `ns:=robot_1` makes each entry:
- `<joint name="robot_1/arm_joint1">`
- `<joint name="robot_1/grip_joint">`
- `<joint name="robot_1/llink_joint1">`

And the `<mimic joint="grip_joint"` references get prefixed to `<mimic joint="robot_1/grip_joint"` by the `_make_namespaced_urdf()` function in the launch file (line 188-189 of `multi_robot_cube_sink.launch.py`).

---

### 5. Step 3 — Simplify `gripper_mimic_relay.py`

**File:** `scripts/x3plus_examples/gripper_mimic_relay.py`

The relay node currently does two things:

1. **Fans out grip command to 5 mimic joints** ← **REMOVE this function**
2. **Strips mimic joints from joint_states** ← **KEEP this function**

The fan-out is no longer needed because `GazeboSimSystem` enforces the mimic constraint directly in the physics solver. The relay only needs to filter mimic joints from `/joint_states_raw` so `robot_state_publisher` can compute them from `<mimic>` for TF.

#### Simplified relay:

```python
#!/usr/bin/env python3
"""
Gripper Mimic Joint Filter

Ignition Fortress JointStatePublisher publishes ALL joint positions including
the 5 mimic finger joints. Since GazeboSimSystem now enforces <mimic>
kinematically, these joints appear in joint_states with their correct positions.

However, robot_state_publisher still needs the 5 mimic joints to be ABSENT from
/joint_states so it recomputes their TF frames from the URDF <mimic> tag.

This node strips the 5 mimic joints from /joint_states_raw and publishes to
/joint_states so RSP can do its job.

Fan-out of grip command to mimic joints is NO LONGER NEEDED — GazeboSimSystem
enforces <mimic> in the physics solver directly.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


MIMIC_JOINTS = {
    'llink_joint1', 'llink_joint2', 'llink_joint3',
    'rlink_joint2', 'rlink_joint3',
}


class GripperMimicRelay(Node):
    def __init__(self):
        super().__init__('gripper_mimic_relay')

        self.declare_parameter('namespace', '')
        ns = self.get_parameter('namespace').value
        self.ns = ns.rstrip('/') if ns else ''
        self._prefix = f'/{self.ns}' if self.ns else ''

        # Joint_states filter (strips mimic joints so RSP recomputes from <mimic>)
        self.pub_js = self.create_publisher(
            JointState, f'{self._prefix}/joint_states', qos_profile_sensor_data
        )
        self.sub_js = self.create_subscription(
            JointState, f'{self._prefix}/joint_states_raw', self._js_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            'Gripper mimic relay active [%s]: '
            '%s/joint_states_raw -> %s/joint_states (filter only, no fan-out)'
            % (self.ns or 'global',
               self._prefix or '', self._prefix or '')
        )

    def _js_callback(self, msg: JointState):
        filtered = JointState()
        filtered.header.stamp = self.get_clock().now().to_msg()
        filtered.name = []
        filtered.position = []
        filtered.velocity = []
        filtered.effort = []
        ns_prefix = f'{self.ns}_' if self.ns else ''

        for idx, name in enumerate(msg.name):
            bare_name = name[len(ns_prefix):] if ns_prefix and name.startswith(ns_prefix) else name
            if bare_name in MIMIC_JOINTS:
                continue
            filtered.name.append(name)
            if idx < len(msg.position):
                filtered.position.append(msg.position[idx])
            if idx < len(msg.velocity):
                filtered.velocity.append(msg.velocity[idx])
            if idx < len(msg.effort):
                filtered.effort.append(msg.effort[idx])

        self.pub_js.publish(filtered)


def main(args=None):
    rclpy.init(args=args)
    node = GripperMimicRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
```

**Key changes:**
- Removed `_master_pub`, `_mimic_pubs`, `_grip_cmd_callback`, `_tick`, `_rate`, `_dt`, `_target`, `_current`
- Removed `MIMIC_MULTIPLIERS` dict
- Removed `Float64` import
- Node now ONLY filters joint_states — it does NOT publish any grip commands
- Grip commands go directly from ArmController/autopilot to the ros2_control controller topic

---

### 6. Step 4 — Update CMakeLists.txt

**File:** `CMakeLists.txt`

#### 6a. The `gripper_mimic_relay` install target stays the same

No changes needed — the node still exists (just simplified).

#### 6b. Add `ros2_control` controllers config file to install section

Create a new controllers config file (see Step 6) and add it:

```cmake
# Copy ros2_control controller configuration
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
```

This already exists (lines 30-33). You just need to place the new YAML file in the `config/` directory.

---

### 7. Step 5 — Update `package.xml` Dependencies

**File:** `package.xml`

Add the ros2_control execution dependencies:

```xml
<!-- ROS2 Control dependencies for GazeboSimSystem <mimic> enforcement -->
<exec_depend>gz_ros2_control</exec_depend>
<exec_depend>ros2_controllers</exec_depend>
<exec_depend>joint_trajectory_controller</exec_depend>
<exec_depend>joint_state_broadcaster</exec_depend>
```

Insert these after line 17 (`<exec_depend>rviz2</exec_depend>`).

---

### 8. Step 6 — Create ros2_control Controllers YAML

**File:** `config/ros2_control_controllers.yaml`

```yaml
# ros2_control controller configuration for X3Plus robot.
# Loaded by the launch file onto the controller_manager node.
#
# grip_joint_controller: position command interface for the gripper master.
#   The 5 mimic joints have NO command interface — they follow grip_joint
#   via <mimic> enforced by GazeboSimSystem.
#
# joint_state_broadcaster: publishes /joint_states from ros2_control's state
#   interfaces (includes all 6 gripper joints).

controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    grip_joint_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

grip_joint_controller:
  ros__parameters:
    joints:
      - arm_joint1
      - arm_joint2
      - arm_joint3
      - arm_joint4
      - arm_joint5
      - grip_joint
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
    # Allow single-joint position commands (compatible with existing
    # arm_controller.py and autopilot publish_pose() which send Float64
    # to individual joints rather than trajectory goals).
    allow_nonzero_velocity_cmd: true
    constraints:
      stopped_velocity_tolerance: 0.01
      goal_time: 0.0

joint_state_broadcaster:
  ros__parameters:
    extra_joints:
      - llink_joint1
      - llink_joint2
      - llink_joint3
      - rlink_joint2
      - rlink_joint3
```

**Note:** For multi-robot, you need one controller_manager and one set of controllers per robot namespace. The launch file creates namespaced controller instances (e.g., `/robot_1/grip_joint_controller`).

---

### 9. Step 7 — Update Launch Files

#### 9a. Single-robot launch (`gazebo.launch.py`)

Add after the robot_state_publisher node and before the bridge:

```python
# ros2_control controller manager for this robot
controller_manager_node = Node(
    package='controller_manager',
    executable='ros2_control_node',
    name='controller_manager',
    output='screen',
    parameters=[
        {'use_sim_time': True},
        os.path.join(sim_gazebo_bringup_dir, 'config', 'ros2_control_controllers.yaml'),
    ],
    remappings=[('/joint_states', '/joint_states_raw')],
)

# Spawn the joint state broadcaster (publishes joint_states from ros2_control)
spawn_joint_state_broadcaster = Node(
    package='controller_manager',
    executable='spawner',
    name='spawn_joint_state_broadcaster',
    output='screen',
    arguments=['joint_state_broadcaster',
               '--controller-manager', '/controller_manager'],
    parameters=[{'use_sim_time': True}],
)

# Spawn the gripper controller
spawn_grip_controller = Node(
    package='controller_manager',
    executable='spawner',
    name='spawn_grip_controller',
    output='screen',
    arguments=['grip_joint_controller',
               '--controller-manager', '/controller_manager'],
    parameters=[{'use_sim_time': True}],
)
```

#### 9b. Multi-robot launch (`multi_robot_cube_sink.launch.py`)

The `_make_namespaced_urdf()` function already prefixes joints as `robot_1_arm_joint1`, etc. The launch needs to:

1. Load the namespaced URDF (already done)
2. Start a namespaced controller_manager per robot
3. Spawn controllers per robot

Add inside the robot loop (after the `robot_state_publisher` node, around line 428):

```python
# ros2_control controller manager for this robot namespace.
# The URDF's <ros2_control> block uses the namespaced joint names
# (e.g., robot_1_arm_joint1, robot_1_grip_joint) — the controller
# manager reads them from the robot_description parameter.
controller_manager_node = Node(
    package='controller_manager',
    executable='ros2_control_node',
    name=f'{rname}_controller_manager',
    namespace=rname,
    output='screen',
    parameters=[
        {'use_sim_time': True},
        os.path.join(sim_gazebo_bringup_dir, 'config', 'ros2_control_controllers.yaml'),
    ],
    remappings=[
        (f'/{rname}/joint_states', f'/{rname}/joint_states_raw'),
    ],
)

# Spawn joint state broadcaster for this robot
spawn_jsb = Node(
    package='controller_manager',
    executable='spawner',
    name=f'spawn_{rname}_jsb',
    output='screen',
    arguments=['joint_state_broadcaster',
               '--controller-manager', f'/{rname}/controller_manager'],
    parameters=[{'use_sim_time': True}],
)

# Spawn gripper controller for this robot
spawn_grip = Node(
    package='controller_manager',
    executable='spawner',
    name=f'spawn_{rname}_grip',
    output='screen',
    arguments=['grip_joint_controller',
               '--controller-manager', f'/{rname}/controller_manager'],
    parameters=[{'use_sim_time': True}],
)
```

The bridge topic list in `_bridge_args()` must also bridge the ros2_control command topic for `grip_joint` instead of the old `/grip_master_target`:

```python
# Replace this in _bridge_args():
f'/{r}/grip_master_target@std_msgs/msg/Float64]ignition.msgs.Double',

# With this (ros2_control controller command topic):
f'/{r}/grip_joint_controller/commands@std_msgs/msg/Float64MultiArray]ignition.msds.Double',
```

---

### 10. Step 8 — Build and Test

```bash
cd ~/ROS2CoorAPI/robot_workspace/x3plus_ws

# 1. Install new dependencies
rosdep install --from-paths src --ignore-src -r -y

# 2. Build
colcon build --packages-select sim_gazebo_bringup --symlink-install

# 3. Source
source install/setup.bash

# 4. Test single robot first
ros2 launch sim_gazebo_bringup gazebo.launch.py world:=office

# 5. In another terminal, send gripper commands
ros2 topic pub /grip_joint_cmd_pos std_msgs/Float64 "{data: -0.75}" -1
# Then close:
ros2 topic pub /grip_joint_cmd_pos std_msgs/Float64 "{data: 0.0}" -1
```

**Observation:** The gripper should now move smoothly without jerking. The 5 finger links follow `grip_joint` kinematically — no independent PID oscillation.

---

### 11. How to Verify It Works

#### Check 1: No torque commands on mimic joints (Gazebo topic)

```bash
# Before fix: /llink_joint1_cmd_pos was publishing commands
# After fix: NO publishers on /llink_joint1_cmd_pos
ros2 topic info /llink_joint1_cmd_pos
# Should show "No publishers" or only the relay (if you kept fan-out)
```

#### Check 2: Joint states show correct mimic positions

```bash
ros2 topic echo /joint_states --once | grep -A5 "llink_joint1"
# llink_joint1 should show position = -grip_joint * 1
```

#### Check 3: TF shows parallel pads

```bash
ros2 run tf2_ros tf_echo rlink2 llink2
# The Z axes should be parallel (both pointing the same direction)
```

#### Check 4: No oscillation in Gazebo GUI

Visually observe the gripper in Gazebo. Open and close repeatedly:
```bash
# Rapid open/close test
for i in 1 2 3 4 5; do
  ros2 topic pub /grip_joint_cmd_pos std_msgs/Float64 "{data: -0.75}" -1
  sleep 1
  ros2 topic pub /grip_joint_cmd_pos std_msgs/Float64 "{data: 0.0}" -1
  sleep 1
done
```

The pads should track smoothly without overshoot, ringing, or visible jerking.

---

### 12. Rollback Plan

If Option E1 doesn't work as expected, revert to the current architecture:

1. **Restore URDF xacro files** from git:
   ```bash
   cd ~/ROS2CoorAPI/robot_workspace/x3plus_ws/src/sim_gazebo_bringup
   git checkout -- scripts/yahboomcar_description/urdf/yahboomcar_X3plus.urdf.xacro
   git checkout -- scripts/yahboomcar_description/urdf/yahboomcar_X3plus_multi.urdf.xacro
   ```

2. **Restore `gripper_mimic_relay.py`** from git:
   ```bash
   git checkout -- scripts/x3plus_examples/gripper_mimic_relay.py
   ```

3. **Revert `package.xml`** and **CMakeLists.txt**:
   ```bash
   git checkout -- package.xml CMakeLists.txt
   ```

4. **Clean rebuild**:
   ```bash
   colcon build --packages-select sim_gazebo_bringup --cmake-clean-cache
   ```

---

### Appendix: Why This Fixes the Jerking

| Root Cause | How Option E1 Fixes It |
|---|---|
| 6 PIDs fighting on coupled joints | **Eliminated** — 5 mimic PIDs removed, only `grip_joint` has torque |
| P=40 with cmd_max=5 on 1e-5 inertia | **Irrelevant** — only grip_joint has PID, which was never the jerking source |
| Ramp rate 0.1 rad/step from relay | **Eliminated** — no relay fan-out, commands go directly to controller |
| Gravity disabled on finger links | **Re-enabled** — passive damping from gravity |
| Missing `rlink3↔rlink2` physical connection | **Replaced** by `<mimic>` kinematic constraint in physics solver |
