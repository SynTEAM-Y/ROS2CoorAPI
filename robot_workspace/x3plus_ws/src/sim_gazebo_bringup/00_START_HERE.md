# X3Plus Robot — Quick Reference

## What's Implemented

### RViz Arm Visualization
- Enhanced RViz configuration (`gazebo_view.rviz`) with proper display settings
- All 5 arm joints + gripper fully visible

### 90° Chassis Turn (Closed-Loop)
- Manual control node with differential drive formula
- **Closed-loop** turns using IMU (Gazebo) or odometry (RViz) feedback
- Stops when actual measured rotation reaches 90°

### Arm & Gripper Control
- 5-DOF arm + gripper via `arm_controller` node
- Individual Gazebo JointPositionControllers for each arm joint
- Active mimic joint **filtering** for gripper linkage (Ignition doesn't enforce URDF mimic)
- Pick-and-place sequence

---

## 📐 The Formula (Theoretical Reference)

```
For differential drive robot:
┌─────────────────────────────────────────────────────────────┐
│ Angular Velocity: ω = 2v / L                                │
│ Turn Time (90°):  t = π·L / (4·v)                           │
│                                                             │
│ Where:                                                      │
│   v = wheel speed = 0.5 m/s  (turn_wheel_speed parameter)  │
│   L = wheel separation = 0.2128 m                           │
│                                                             │
│ Result:                                                     │
│   ω = 2 × 0.5 / 0.2128 = 4.699 rad/s  (theoretical)        │
│   t = 3.14159 × 0.2128 / (4 × 0.5)                         │
│   t = 0.334 seconds           (open-loop estimate only)     │
│                                                             │
│ Actual cmd_vel angular.z = ±1.50 rad/s  ← NOT 4.699        │
│   (lower value for stable closed-loop tracking)             │
│                                                             │
│ NOTE: Actual execution uses closed-loop                     │
│ IMU (Gazebo) or odom (RViz) feedback → exact 90°            │
│ regardless of timing.                                       │
└─────────────────────────────────────────────────────────────┘
```

## 📡 Odometry Quick Reference

| Mode | Provider | Topic |
|------|----------|-------|
| Gazebo | Ignition DiffDrive plugin → ros_gz_bridge | `/odom` |
| RViz-only | `diff_drive_simulator` node | `/odom` |
| Yaw (turns) | Ignition IMU plugin → ros_gz_bridge | `/imu` (Gazebo only) |

Integration: `x += v·cos(θ)·Δt`, `y += v·sin(θ)·Δt`, `θ += ω·Δt`  
See the **Odometry** section of `README.md` for full details.

---

## 🚀 How to Use

### Quick Start
```bash
# Step 1: Build
cd ~/ROS2Coordination/robot_workspace/x3plus_ws
colcon build --symlink-install --packages-select x3plus_examples sim_gazebo_bringup yahboomcar_description
source install/setup.bash

# Step 2a: Gazebo simulation (physics, arm, gripper)
ros2 launch sim_gazebo_bringup gazebo.launch.py

# Step 2b: RViz-only mode (no Gazebo required)
ros2 launch sim_gazebo_bringup robot_rviz.launch.py

# Step 3: Manual control (in another terminal)
source install/setup.bash
ros2 run x3plus_examples manual_control

# Step 4: Arm control (in another terminal)
source install/setup.bash
ros2 run x3plus_examples arm_controller
```

### Keyboard Controls — manual_control
```
Movement:      | 90° Turns:      | System:
───────────────|─────────────────|──────────
W - Forward    | 1 - 90° Left    | H - Help
S - Backward   | 2 - 90° Right   | Q - Quit
A - Turn Left  | 3 - 90° L+Move  |
D - Turn Right | 4 - 90° R+Move  |
Space - Stop   |                 |

Speeds: forward/backward = 0.8 m/s, rotation = 1.0 rad/s
90° turns: closed-loop via IMU (Gazebo) or odom (RViz)
```

### Keyboard Controls — arm_controller
```
Joint Selection: 1-5 = arm joints, 6 = gripper
W/S = increase/decrease selected joint
O/C = open/close gripper
A = home pose, Z = init pose, B = down pose
P = pick-and-place sequence
```

---

## 📋 Console Output Example

When you press `1` for 90° left turn:

```
══════════════════════════════════════════════════════════════════════════
90-DEGREE TURN EXECUTION (CLOSED-LOOP)
══════════════════════════════════════════════════════════════════════════
Direction: LEFT
Type: IN_PLACE

📐 THEORETICAL (open-loop):
────────────────────────
  ω = 2v/L = 2×0.5/0.2128 = 4.6992 rad/s
  t = (π/2)/ω = 0.3343 s

🤖 ACTUAL EXECUTION (closed-loop with odometry):
────────────────────────
  Command ω: 1.50 rad/s
  Linear: 0.00 m/s
  Target rotation: 90° (π/2 = 1.5708 rad)
  Feedback: /odom yaw tracking
══════════════════════════════════════════════════════════════════════════

90° left turn completed! (actual: 90.2°, error: +0.2°)
```

---

## Key Features

1. **Gazebo Simulation** — Full physics with DiffDrive (4-wheel, asymmetric friction μ1=1.0/μ2=0.05), arm JointPositionControllers, IMU sensor
2. **RViz Visualization** — RViz-only mode with `diff_drive_simulator`, map display, static `map→odom` TF
3. **90° Turns** — Closed-loop via IMU (Gazebo) or odom (RViz) yaw tracking; theoretical ω = 4.699 rad/s, commanded ω = 1.50 rad/s
4. **Arm Control** — 5-DOF arm + gripper. Each arm joint and `grip_joint` has an Ignition `JointPositionController` (PID). The 5 finger linkage joints are passive (no controller) — they follow `grip_joint` via the URDF `<mimic>` tag computed by `robot_state_publisher`.
5. **Pick & Place** — Automated 12-step sequence in `arm_controller` (P key)
6. **Odometry** — Ignition DiffDrive plugin → `/odom` (Gazebo) or `diff_drive_simulator` node (RViz-only); IMU crosscheck in turns
7. **Gripper Mimic** — `gripper_mimic_relay` node **filters** the 5 mimic joint entries out of `/joint_states_raw` (raw Ignition output) and republishes the trimmed message on `/joint_states`. Because the mimic joints are absent from the message, `robot_state_publisher` honours the URDF `<mimic>` tag and computes finger positions from `grip_joint`.
8. **URDF→SDF Pipeline** — `ign sdf -p` pre-conversion at launch time preserves all model plugins

---

## Documentation Files

| File | Content |
|------|---------|
| `00_START_HERE.md` | This quick reference |
| `README.md` | Package overview, topics, launch args, troubleshooting |
| `README.md` (Odometry section) | **Odometry math, frames, IMU crosscheck, monitoring** |
| `90DEGREE_TURN_FORMULA.md` | Detailed turn formula math |
| `VISUAL_GUIDE_90DEGREE_TURN.md` | Turn diagrams |
| `WHEEL_PARAMETERS_REFERENCE.md` | Wheel config reference |
| `WHEEL_DIFFERENTIAL_SETUP.md` | How wheels were added |
| `WHEEL_SETUP_EXAMPLE.md` | Wheel customization examples |
| `GAZEBO_MOVEMENT_FIX.md` | Movement fix notes and undocumented features |
| `IMPLEMENTATION_SUMMARY.md` | Full implementation details |
| `RVIZ_ARM_FIX_AND_90TURN_FORMULA.md` | Arm + turn tech docs |
| `SETUP_STATUS.md` | Setup status and known issues |

