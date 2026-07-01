# Comprehensive Workshop Report: Autonomous Vision-Based Pick-and-Place with Yahboom X3Plus in ROS 2 Gazebo

## Abstract
This extensive report provides a deep-dive analysis of the entire software stack, architectural design, and kinematic modeling developed to simulate the Yahboom X3Plus mobile manipulator in a ROS 2 Gazebo (Ignition Fortress) environment. It documents the transition from a hardware-specific ROS 1 implementation to a modular, robust ROS 2 system capable of autonomous vision-based pick-and-place operations. Every critical script, launch file, URDF modification, and analytical challenge (such as the physics of parallel-jaw linkages) is detailed herein to serve as a complete technical reference for workshop attendees and developers.

---

## 1. Introduction and Objectives

The Yahboom X3Plus is a differential-drive mobile robot equipped with a 5-DOF robotic arm, a 1-DOF parallel-jaw gripper, a front-facing Astra depth camera, and an arm-mounted monocular wrist camera. The primary objective of this project was to:
1.  **Replicate the Physical Robot in Simulation:** Create a high-fidelity Gazebo simulation that accurately reflects the mass, inertia, joint limits, and sensor feeds of the physical robot.
2.  **Migrate Manufacturer Code from ROS 1 to ROS 2:** Translate the proprietary, hardware-dependent ROS 1 autopilot scripts into a standard, open-source ROS 2 Humble paradigm.
3.  **Achieve Autonomous Operations:** Implement an end-to-end pipeline combining Nav2 (for base movement), Object Detection (OpenCV), and custom state machines to pick up a dynamically spawned blue test cube and place it on a landing pad.

---

## 2. Directory Structure Overview (`sim_gazebo_bringup`)

The workspace is organized into several key directories:
*   **`launch/`**: Contains Python launch scripts for bootstrapping Gazebo, RViz, Nav2, MoveIt, and the customized vision autopilot.
*   **`scripts/x3plus_examples/`**: The core logic center. Contains Python nodes for object detection, trajectory bridging, TF relaying, and the central state machine.
*   **`scripts/yahboomcar_description/urdf/`**: Contains the `yahboomcar_X3plus.urdf.xacro` file which defines the entire physical, visual, and collision model of the robot.
*   **`config/`**: Holds tuning parameters for Nav2 (`nav2_params.yaml`), MoveIt (`joint_limits.yaml`, `kinematics.yaml`), and the vision system (`hsv_colors.yaml`).
*   **`models/` & `meshes/`**: 3D assets and Gazebo SDF models for the robot, the test cube, and the environment.

---

## 3. Kinematic Modeling: The URDF / XACRO (`yahboomcar_X3plus.urdf.xacro`)

The URDF is the foundational blueprint of the simulation. Translating it for Ignition Gazebo required significant modifications from standard ROS 1 implementations.

### 3.1 The Robotic Arm Chain
The arm consists of 5 joints, modeled as `revolute` joints with strict limits:
*   `arm_joint1` (Base Pan): Sweeps the arm left to right.
*   `arm_joint2` (Shoulder Pitch): Lifts the arm up and down.
*   `arm_joint3` (Elbow Pitch): Works alongside the shoulder for vertical reach.
*   `arm_joint4` (Wrist Pitch): Tilts the gripper up or down relative to the ground. 
    *   *Workshop Note:* In simulation, we adjusted the manufacturer's pick pose for this joint from `38°` to `45°` to compensate for a 90° roll offset introduced by the `rpy="pi/2 0 0"` on `arm_joint5`.
*   `arm_joint5` (Wrist Roll): Rotates the gripper assembly.

### 3.2 Gazebo System Plugins
Ignition Gazebo requires explicit plugins to drive the joints. We utilize `ignition::gazebo::systems::JointPositionController` for each joint.
```xml
<plugin filename="ignition-gazebo-joint-position-controller-system" name="ignition::gazebo::systems::JointPositionController">
    <joint_name>arm_joint2</joint_name>
    <topic>/arm_joint2_cmd_pos</topic>
    <p_gain>25</p_gain> <i_gain>0.5</i_gain> <d_gain>3</d_gain>
    <i_max>5</i_max> <i_min>-5</i_min>
</plugin>
```
**PID Tuning:** Gravity severely affects articulated arms in simulation. `arm_joint2` requires a high `p_gain` of 25 and an `i_gain` of 0.5 to eliminate steady-state droop when the arm stretches forward.

### 3.3 Sensor Configurations
Two primary sensors are defined in the URDF:
1.  **Wrist Camera (`mono_link`)**: Attached to `arm_link4`, pointing downward at the gripper. Uses `ignition::gazebo::systems::Sensors`.
2.  **Contact Sensors (`llink2` & `rlink2`)**: Attached to the gripper finger pads. These broadcast Boolean arrays when they physically collide with the blue cube, allowing the code to definitively confirm a successful grasp without relying solely on vision.

---

## 4. The Gripper Mimic Joint Challenge

### 4.1 The Physics Engine Limitation
The X3Plus uses a parallel-jaw gripper. Moving the master `grip_joint` physically pulls 5 other joints (`rlink2`, `rlink3`, `llink1`, `llink2`, `llink3`). In ROS, this is traditionally handled using the `<mimic>` tag. 
However, **Gazebo Ignition ignores `<mimic>` tags in its physics engine**. If you only command the master joint, the fingers will detach, explode, or simply hang limp, unable to grip anything.

### 4.2 The Custom Solution: `gripper_mimic_relay.py`
To circumvent this, we converted all passive mimic joints in the URDF to `continuous` joints and gave them their own individual PID position controllers with incredibly high stiffness (`P=100`). We removed gravity from the finger links to prevent numerical instability.

We then built a software relay (`gripper_mimic_relay.py`):
1.  It subscribes to `/grip_joint_cmd_pos` (the master command).
2.  It multiplies this command by the mimic multipliers defined in the URDF (e.g., `1` for the right side, `-1` for the inverted left side).
3.  It fans out these commands synchronously to the 5 hidden topic channels (`/rlink_joint2_cmd_pos`, etc.).
4.  It intercepts `/joint_states_raw` from Gazebo, strips out the chaotic readings of the mimic joints, and allows `robot_state_publisher` to mathematically compute perfectly smooth visualization states.

*Result:* The gripper now functions with immense stability and can securely clamp onto objects in the physics engine.

---

## 5. Software Nodes and Scripts Breakdown

### 5.1 `trajectory_bridge.py`
**Purpose:** ROS 2 standardizes arm control using the `FollowJointTrajectory` action server (part of MoveIt/ros2_control). However, Gazebo plugins only accept raw `std_msgs/Float64` messages. 
**Implementation:** This script instantiates an Action Server. When the state machine sends a complex trajectory (a list of waypoints over time), this script interpolates the points and streams them to the individual Gazebo `/cmd_pos` topics at 50Hz, mimicking smooth `ros2_control` hardware execution.

### 5.2 `gazebo_pose_tf_relay.py`
**Purpose:** Skid-steer differential drive kinematics generate massive odometry drift in simulation.
**Implementation:** Instead of using the wheel encoders (`/odom`), this script listens to Gazebo's Ground Truth `PosePublisher` plugin and broadcasts perfect `odom -> base_footprint` TF transforms. This guarantees that when the robot navigates towards the cube, it does not miss due to wheel slip.

### 5.3 `object_detector.py`
**Purpose:** Vision processing.
**Implementation:** 
*   Subscribes to the wrist camera (`/wrist_mono_camera/image_raw`).
*   Uses OpenCV `cv2.inRange` with HSV bounds (configurable via `hsv_colors.yaml`) to isolate the blue cube.
*   Calculates the largest contour and extracts the centroid (`pixel_x, pixel_y`).
*   In early iterations, this node performed 3D back-projection. In the final optimized version, we rely purely on the 2D pixel coordinates to drive the state machine, mimicking the manufacturer's logic.

---

## 6. The Core State Machine: `vision_pick_place.py`

This is the magnum opus of the project. It orchestrates the entire lifecycle of the autonomous mission. After extensive review of the proprietary ROS 1 manufacturer code, we discarded complex, error-prone TF-based visual servoing in favor of a highly robust, pixel-based linear mapping approach.

### 6.1 State 1: IDLE
The robot moves its arm to the `MFR_HOME` pose (`[0.0, 0.524, -1.571, -1.571, 0.0]`). The wrist camera looks sideways, and the robot waits for the external system to detect the cube.

### 6.2 State 2: DETECT
The system receives the coordinates of the spawned blue block (via Ground Truth TF or the front Astra camera logic) and prepares the navigation goal.

### 6.3 State 3: NAVIGATE
**The Hybrid Approach:**
Unlike the physical robot which uses noisy pixel-PID to drive from across the room, our simulation uses absolute TF positioning to drive the base.
*   The script calls `_drive_to_face_cube()`.
*   It calculates the Euclidean distance `math.hypot(dx, dy)`.
*   It publishes `/cmd_vel` to smoothly drive the robot to exactly `0.30m` away from the cube while adjusting its yaw to face the target.
*   *Crucially:* The arm remains folded in the `HOME` position to prevent the extended arm from colliding with the environment during transit.

### 6.4 State 4: PICK (The Manufacturer's Sequence)
This phase perfectly replicates the physical robot's behavior. The base *stops moving completely*.

1.  **Approach Pose:** The arm lowers to the `pick_approach` pose. Now, the wrist camera points directly down at the workspace.
2.  **Fine Alignment:** `_wrist_camera_align()` gently rotates the robot base *in place* until the cube is perfectly centered in the camera (`abs(pixel_x - 320) < 15`).
3.  **The Parallax Calculation:** The camera is not located exactly where the gripper is. To compensate for lateral offsets, we apply the manufacturer's linear mapping formula:
    ```python
    pos1_deg = 0.2128 * pixel_x + 21.91
    j1_rad = (pos1_deg - 90.0) * math.pi / 180.0
    ```
    This calculates exactly how many degrees the base of the arm (`arm_joint1`) needs to rotate to perfectly hover over the block.
4.  **Staged Lowering:** If the arm lowers all joints at once, it smashes the cube. Instead, it follows a choreographed sequence:
    *   Lower Joint 2 (Shoulder) to 60°.
    *   Lower Joint 1 to Neutral (0°). *Because we already aligned the robot heading, resetting the arm base to neutral perfectly centers the jaws over the cube.*
5.  **Grasp:** The gripper closes.
6.  **Lift:** The arm pulls up to the `CARRY` pose.

### 6.5 Grasp Verification and Retry
The script interrogates the `test_block` TF. If the Z-axis coordinate is greater than `0.10m`, the object has successfully left the floor. 
If it fails (e.g., the cube slipped out), the robot opens its gripper, lowers the arm again, **re-reads the camera pixels**, calculates a fresh `j1_rad` offset, and attempts a second grip.

### 6.6 State 5 & 6: TRANSPORT and PLACE
The robot safely reverses away from the pick location, navigates to the predefined drop-zone coordinates (`drop_off_x`, `drop_off_y`), lowers the arm to the `PLACE` pose, and releases the gripper, concluding the mission.

---

## 7. Launch Files Configuration

### 7.1 `gazebo.launch.py`
This is the lowest-level bringup file.
*   Launches `ros_gz_sim` loading the `office.sdf` or `empty.sdf` world.
*   Uses `xacro` to compile the URDF into raw XML.
*   Spawns the robot using `ros_gz_sim create`.
*   Starts the `ros_gz_bridge` to link `ignition.msgs` (e.g., `/model/x3plus/cmd_vel`) to ROS `geometry_msgs`.

### 7.2 `vision_autopilot.launch.py`
This orchestrates the high-level autonomy.
1.  Includes `gazebo.launch.py` (headless mode).
2.  Starts `spawn_test_object.py` to drop the blue cube into the world.
3.  Starts `gazebo_pose_tf_relay.py` for perfect odometry.
4.  Starts `gripper_mimic_relay.py` to fix the physics.
5.  Starts `trajectory_bridge.py` for arm actuation.
6.  Starts `object_detector.py` for vision processing.
7.  Finally, runs `vision_pick_place.py` to execute the state machine.

---

## 8. Nav2 and MoveIt Integration

While the custom state machine controls the exact picking sequence, the workspace is heavily integrated with standard ROS 2 navigation and manipulation frameworks.

*   **`nav2_params.yaml`:** Configures the local and global costmaps. We increased the inflation radius to ensure the robotic arm (which protrudes past the base) does not strike virtual obstacles during transit. DWB Local Planner is tuned for skid-steer kinematics.
*   **`yahboomcar_X3plus.srdf`:** Semantic Robot Description Format. Defines the planning groups (`arm_group` and `gripper_group`) and pre-defines the poses (like `up`, `down`, `init`) that MoveIt uses for collision-free trajectory generation.

---

## 9. Conclusion

This project successfully ported a highly proprietary hardware control stack into a generalized ROS 2 Simulation. 
By solving the Gazebo mimic joint limitation with closed-loop PIDs, and translating raw pixel data into exact kinematic offsets using linear mapping, the simulated Yahboom X3Plus can consistently and autonomously locate, approach, grasp, and deliver objects in a 3D environment. 

### Key Takeaways for Workshop Attendees:
1.  **Simulation != Reality:** You cannot blindly port hardware code. Real robots rely on vision because odometry drifts; simulations provide perfect odometry but struggle with complex physics linkages. Hybrid approaches (TF for the base, Vision for the arm) yield the best results.
2.  **Joint Controllers:** Always apply distinct PID controllers to passive joints in Gazebo Ignition to prevent explosive physical artifacts.
3.  **Coordinate Math:** Understand the relationship between URDF radians, Hardware degrees, and Camera Pixels. A single linear equation (`y = mx + b`) often outperforms computationally heavy 3D PointCloud back-projections for tabletop manipulation tasks.
EOF
---

## 10. Core Python Scripts Detail

Beyond the main vision autopilot, two highly critical utility scripts drive alternative and foundational behaviors: `manual_control.py` and `pick_and_place.py`. Both are central to verifying and orchestrating the robot's capabilities in ROS 2.

### 10.1 `manual_control.py`: Closed-Loop Odometry Teleoperation
**Purpose:** Provides robust keyboard teleoperation (`w/s/a/d`) for the differential drive base, but uniquely features an automated, closed-loop 90° turning mechanism (keys `1` to `4`).

**Implementation Details:**
Standard teleoperation nodes rely on user reflexes or open-loop timers (e.g., "spin at 1 rad/s for 1.57 seconds"). In a physics-heavy simulator with variable load and slip, timers are highly inaccurate. 
*   **Closed-Loop Feedback:** This script subscribes directly to `/odom` (the Ground Truth TF in simulation). When a 90° turn is triggered, it continuously calculates the accumulated yaw rotation.
*   **The Math:** By comparing `current_yaw` against `start_yaw` using quaternion-to-euler conversions, the node dynamically publishes `/cmd_vel` until exactly $\frac{\pi}{2}$ radians are traversed, ensuring the robot turns perfectly square in the `office` map regardless of wheel slip.

### 10.2 `pick_and_place.py`: Nav2-Based Blind Manipulation
**Purpose:** The predecessor and blind alternative to the Vision Autopilot. This script demonstrates how to execute autonomous transport using absolute coordinate mapping (Nav2) and fixed joint trajectories, bypassing visual servoing entirely.

**Implementation Details:**
1.  **Nav2 Integration:** Instead of driving via custom proportional math, this script communicates with the ROS 2 Navigation Stack via the `NavigateToPose` Action Server. It sends absolute map coordinates for the target object.
2.  **Hardcoded Kinematic Choreography:** It completely bypasses MoveIt IK. The pick-up and drop-off sequences are heavily tuned, pre-calculated arrays of radians:
    *   `REACH_DOWN = [0.0, -1.45, -0.54, -1.21, 0.0]`
    *   `CARRY = [0.0, -0.80, -0.40, -0.30, 0.0]`
3.  **The "FK Settle Compensation" Challenge:** A crucial discovery documented in this file: when commanded to `REACH_DOWN`, the physical gravity load on the simulation joints causes the arm to settle ~40mm short of the theoretical kinematic position. The script explicitly defines a `DESIRED_STANDOFF = 0.02m` and `GAP_BIAS = 0.02m` to mathematical offset the robot base, pre-compensating for this gravitational droop before opening the gripper.
