# Comprehensive Workshop Report: Autonomous Vision-Based Pick-and-Place with Yahboom X3Plus in ROS 2 Gazebo

## Abstract
This report details the architectural design, kinematic modeling, and software implementation of an autonomous pick-and-place system for the Yahboom X3Plus robot using ROS 2 Humble and Gazebo (Ignition Fortress). It covers the translation of ROS 1 hardware-specific manufacturer code into a robust, high-fidelity ROS 2 simulation. Key innovations include a custom closed-loop physics relay for mimicking gripper kinematics, a hybrid TF-vision navigation approach, and a highly precise pixel-to-joint mapping mechanism for visual servoing.

---

## 1. Introduction
The objective of this project is to simulate the Yahboom X3Plus robot—a mobile manipulator featuring a differential drive base, a 5-DOF robotic arm, and a 1-DOF parallel-jaw gripper—within a ROS 2 Gazebo environment. The primary scenario is a vision-based pick-and-place task where the robot must autonomously identify a target object (a blue cube), navigate to it, align its end-effector using visual feedback, grasp the object, and transport it to a designated landing pad.

Transitioning from the manufacturer's ROS 1 physical hardware stack to a ROS 2 simulation presented several challenges:
1. **Simulation Physics:** Gazebo does not natively enforce URDF `<mimic>` joint constraints at the physics engine level, causing explosive unstable behaviors in complex gripper linkages.
2. **Kinematic Discrepancies:** Hardware-calibrated joint angles and vision mappings do not inherently map 1:1 with pure simulated URDF kinematics.
3. **Sensor Noise vs. Ground Truth:** Physical robots rely heavily on camera feedback for navigation due to odometry drift, whereas simulations offer exact TF transforms that require a hybrid logic approach.

---

## 2. System Architecture & Setup

### 2.1 ROS 2 Node Graph
The system operates using a distributed node architecture:
*   **`ros_gz_bridge`**: Facilitates bi-directional communication between ROS 2 topics and Gazebo Transport topics (e.g., `/cmd_vel`, `/clock`, `/joint_states`).
*   **`robot_state_publisher` (RSP)**: Broadcasts the robot's TF tree based on the XACRO/URDF description and the joint states.
*   **`nav2_bringup`**: Manages the costmaps, AMCL, and BT Navigators for path planning.
*   **`gripper_mimic_relay`**: A custom C++ node bypassing Gazebo's mimic joint limitations by fanning out synchronized PID commands.
*   **`vision_pick_place`**: The central Python state machine orchestrating the autonomous sequence.
*   **`object_detector`**: Processes `/wrist_mono_camera/image_raw` using OpenCV HSV filtering.

*[Insert Image: ROS 2 Rqt_graph showing node interactions]*

---

## 3. Kinematics and URDF Description

### 3.1 Arm Joint Definitions
The robotic arm is a 5-DOF articulated manipulator. The URDF was meticulously translated to align with the manufacturer's conventions, mapped into standard ROS (radians).

*   `arm_joint1` (Base Rotation): Rotates the entire arm assembly.
*   `arm_joint2` (Shoulder Pitch): Modifies the reach and height.
*   `arm_joint3` (Elbow Pitch): Acts in tandem with joint 2.
*   `arm_joint4` (Wrist Pitch): Determines the approach angle of the gripper.
*   `arm_joint5` (Wrist Roll): Rotates the gripper axially.

**Code Snippet: Arm Joint 4 Definition**
```xml
<xacro:revolute_joint name="arm_joint4" parent="arm_link3" child="arm_link4" xyz="0 -0.0829 0" rpy="0 0 0" axisZ="-1" lower="-${pi/2}" upper="${pi/2}"/>
```
*Workshop Note:* Explain how `rpy="0 0 0"` and `axisZ="-1"` dictates the coordinate transformation between the arm links.

### 3.2 Joint Position Controllers in Gazebo
To drive the arm in Gazebo, each joint is equipped with an `ignition::gazebo::systems::JointPositionController`.

```xml
<plugin filename="ignition-gazebo-joint-position-controller-system" name="ignition::gazebo::systems::JointPositionController">
    <joint_name>arm_joint2</joint_name>
    <topic>/arm_joint2_cmd_pos</topic>
    <p_gain>25</p_gain> <i_gain>0.5</i_gain> <d_gain>3</d_gain>
    <i_max>5</i_max> <i_min>-5</i_min>
</plugin>
```
*PID Tuning Detail:* High P-gains (15-25) were necessary to overcome gravity and the inertia of the links, with integral terms to eliminate steady-state droop when the arm is fully extended.

---

## 4. The Gripper Physics & Mimic Joint Solution

### 4.1 The Mimic Joint Problem
The X3Plus uses a parallel-jaw linkage system. In the URDF, driving the `grip_joint` mechanically forces the movement of `rlink2`, `rlink3`, `llink1`, `llink2`, and `llink3` via `<mimic>` tags. While `robot_state_publisher` reads these tags to compute TF frames, Gazebo Fortress physics ignores them. Consequently, simulating the grasp caused the fingers to remain static or detach.

### 4.2 The Closed-Loop PID Solution
To solve this, we explicitly modeled the mimic joints as independent `continuous` or `revolute` joints in Gazebo and attached a highly stiff PID controller (`P=100`) to each. 

**URDF Gripper Configuration:**
```xml
<plugin filename="ignition-gazebo-joint-position-controller-system" name="ignition::gazebo::systems::JointPositionController">
    <joint_name>llink_joint1</joint_name>
    <topic>/llink_joint1_cmd_pos</topic>
    <p_gain>100</p_gain> <i_gain>0.0</i_gain> <d_gain>0.0</d_gain>
</plugin>
```

We then implemented a custom `gripper_mimic_relay` node. This node subscribes to the master `/grip_joint_cmd_pos`, applies the exact mathematical multiplier defined in the URDF (e.g., `-1` for inverted joints), and simultaneously publishes the commands to the 5 hidden mimic topics.

**Mathematical Representation:**
For each mimic joint $i$, the commanded position $\theta_i$ is:
$$\theta_i = \theta_{master} \times M_i + O_i$$
Where $M_i$ is the multiplier and $O_i$ is the offset.

### 4.3 Friction and Contact Physics
To ensure the cube does not slip, the `<mu1>` and `<mu2>` friction parameters for the finger pads (`llink2` and `rlink2`) were increased to `100.0`. Gravity was disabled on the tiny 1-gram finger links to prevent numerical instability within Gazebo's physics solver.

---

## 5. Vision System and Object Detection

### 5.1 Camera Configuration
The robot utilizes a monocular wrist camera mounted on `arm_link4`.
*   **Resolution:** 640x480
*   **FOV:** 1.047 radians
*   **Update Rate:** 10 Hz

### 5.2 HSV Color Tracking
The target is identified using the HSV (Hue, Saturation, Value) color space, which is highly robust to lighting changes compared to RGB.
```python
BLUE_LOWER = np.array([80, 50, 50])
BLUE_UPPER = np.array([120, 255, 255])
```
The algorithm:
1. Applies `cv2.cvtColor` to convert the BGR image to HSV.
2. Creates a binary mask using `cv2.inRange`.
3. Extracts contours via `cv2.findContours`.
4. Filters out noise by mandating a `VISION_MIN_AREA` (200 pixels).
5. Computes the bounding box and centroid `(pixel_x, pixel_y)`.

*[Insert Image: Side-by-side of raw camera feed and HSV Mask]*

---

## 6. Hybrid Navigation & Approach Strategy

### 6.1 Manufacturer Logic vs. Simulation Adaptation
The physical manufacturer code relies on the front-facing Astra camera for PID-based navigation toward the cube. However, in a simulated environment, relying purely on depth back-projection can introduce unnecessary drift (e.g., floor clipping misidentified as the object).

### 6.2 TF-Based Base Navigation
We optimized the navigation by utilizing the Gazebo ground-truth TF (`test_block`). 
```python
dx = target_x - self._odom_x
dy = target_y - self._odom_y
dist = math.hypot(dx, dy)
```
The robot computes the Euclidean distance and the heading error (`yaw_err = atan2(dy, dx) - current_yaw`), publishing `/cmd_vel` proportional commands to smoothly drive the differential base to a precise standoff distance (`0.30m`).

---

## 7. Autonomous Pick-and-Place State Machine

The core intelligence resides in `vision_pick_place.py`. The state machine operates in 5 distinct phases.

### 7.1 Manufacturer Joint Angle Conventions
The ROS 1 manufacturer code defines 90° as the neutral position. We mapped these to URDF radians:
*Formula:* `URDF_rad = (MFR_deg - 90) * (π / 180)`

| State | Manufacturer (deg) | URDF (rad) |
|-------|--------------------|------------|
| **Home** | `[90, 120, 0, 0, 90]` | `[0.0, 0.524, -1.571, -1.571, 0.0]` |
| **Pick Grip** | `[0, 7, 60, 38, 90]` | `[-1.571, -1.449, -0.524, -0.908, 0.0]` |

### 7.2 The Lateral Alignment Mathematical Model
A critical discovery during code analysis was that **the manufacturer does not rotate the base to align the gripper**. Instead, they utilize a linear mapping equation to rotate `arm_joint1` based on the wrist camera's `pixel_x`.

**The Calibration Formula:**
```python
pos1_deg = 0.2128 * pixel_x + 21.91
j1_rad = (pos1_deg - 90.0) * math.pi / 180.0
```
*   If `pixel_x = 320` (perfectly centered in a 640px wide image), `pos1_deg = 90.006°` (Neutral).
*   If the cube is to the left/right, the base of the arm rotates exactly enough to compensate for the parallax.

### 7.3 Staged Arm Lowering Sequence
To prevent colliding with the object, the arm is lowered in a strict, staged sequence:
1. **Approach Pose:** `[j1_rad, -1.449, -0.524, -0.908, 0.0]`. The arm points down, wrist camera confirms the cube.
2. **Lower Joint 2:** `j2` lowered to 60° (URDF `-0.524`). Drops the shoulder.
3. **Lower Joint 1:** `j1` is snapped from the *calculated camera offset* to **Neutral (0° / -1.571 rad)**. Because the robot drove until the cube was centered, snapping the base joint to 0 perfectly aligns the gripper center over the cube.
4. **Grip:** Gripper closes to 4.5cm.
5. **Lift:** Arm returns to carry pose.

**Code Snippet: The Pick Sequence**
```python
# Step 3: Further lower — j1 to neutral 0° (manufacturer: id=1, angle=0)
self.get_logger().info('  [MFR] Further lower (j1→0° neutral) — gripper vertical')
lower_pose[0] = -1.571  # j1 = 0° URDF
self._move_arm(lower_pose, 'mfr_lower_j1', duration_sec=1.5)
```

---

## 8. Verification and Retry Mechanics
To ensure reliability, the system features a ground-truth grasp verification system. After the lift command, the code measures the Z-axis coordinate of the `test_block` TF.
*   If `Z > 0.10m`, the grasp is successful.
*   If `Z < 0.10m`, a retry loop is triggered. 

During a retry, the arm opens, returns to the approach pose, **re-reads the camera pixels to calculate a new `j1`**, and attempts the staged lowering again. This accommodates microscopic shifts in the cube's position during a failed grasp.

---

## 9. Conclusion
By dissecting the manufacturer's ROS 1 codebase and bridging the gap to a ROS 2 Gazebo simulation, we successfully created a highly stable, completely autonomous pick-and-place pipeline. The integration of the custom `gripper_mimic_relay` solved deep-rooted physics engine limitations, while the hybrid TF-Vision navigation provided millimeter accuracy.

### 9.1 Future Work
1. **Dynamic Target Tracking:** Updating the PID loop to catch moving targets in real-time.
2. **YOLO Integration:** Porting the manufacturer's YOLOv11 logic to classify multiple objects (e.g., sorting red, green, and blue cubes into respective bins).
3. **Hardware Deployment:** Testing the exact ROS 2 state machine on the physical X3Plus using `micro-ROS` or standard serial interfaces.

---

## Appendix A: Key Source Code Links
1. **`vision_pick_place.py`**: State machine, IK mappings, and visual servoing.
2. **`yahboomcar_X3plus.urdf.xacro`**: Robot geometry and Gazebo controller plugins.
3. **`gripper_mimic_relay.cpp`**: (Reference to the custom package handling linkage math).

*(Note for Presenter: Insert comprehensive code blocks and GitHub links here for the workshop attendees to reference during the lab portion.)*
