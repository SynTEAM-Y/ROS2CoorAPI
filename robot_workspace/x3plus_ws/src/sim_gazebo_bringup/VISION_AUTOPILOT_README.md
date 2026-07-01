# Vision-Based Autonomous Pick-and-Place

This directory contains code ported from the ROS1 `arm_autopilot` package, adapted for ROS2 Humble and integrated with your X3Plus robot simulation.

## What Was Ported

### 1. **SimplePID Controller** (`autopilot_common.py`)
- Discrete PID controller for smooth motion control
- Supports vector-valued targets for multi-axis control
- Used for vision-guided approach and arm positioning

### 2. **Vision Pick-and-Place Node** (`vision_pick_place.py`)
- Full autonomous state machine: DETECT → NAVIGATE → APPROACH → PICK → TRANSPORT → PLACE
- Integrates with your existing `object_detector.py` for vision-based object detection
- Uses PID control for smooth vision-guided approach to objects
- Compatible with Nav2 navigation and FollowJointTrajectory arm control

### 3. **HSV Configuration** (`config/hsv_colors.yaml`)
- Color calibration data for object detection
- Pre-configured for red, green, blue, yellow objects
- Tuned for Gazebo test_block (blue/cyan cube)

### 4. **Launch File** (`launch/vision_autopilot.launch.py`)
- Complete system launch including:
  - Gazebo simulation
  - Nav2 navigation
  - Object detector
  - Vision pick-and-place autopilot
  - RViz visualization

## How to Use

### Basic Usage

```bash
# Build the package
cd /home/othman/ROS2CoorAPI/robot_workspace/x3plus_ws
colcon build --symlink-install --packages-select sim_gazebo_bringup

# Source the workspace
source install/setup.bash

# Launch the vision-based autopilot (Gazebo only, no Nav2)
# Opens office world with landing pad and blue cube by default
bash -c "cd /home/othman/ROS2CoorAPI/robot_workspace/x3plus_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch sim_gazebo_bringup vision_autopilot.launch.py"

# Or just press Enter at the prompts to use defaults (office world, circular_map)
```

**Note:** This launch configuration uses **Gazebo only** without Nav2 navigation. The map selection prompt appears but is not used. You can add Nav2 navigation later when needed.

### HSV Calibration Mode

If you need to calibrate HSV ranges for different objects:

```bash
# Launch object detector in calibration mode
ros2 run sim_gazebo_bringup object_detector --ros-args -p calibrate_mode:=true

# Adjust trackbars in the HSV Calibrator window
# Save the values to config/hsv_colors.yaml
```

### Testing Individual Components

**Test PID Controller:**
```bash
cd src/sim_gazebo_bringup/scripts/x3plus_examples
python3 -c "from autopilot_common import SimplePID; \
    pid = SimplePID([0.0], [1.0], [0.0], [0.1]); \
    print('PID output:', pid.update([0.5]))"
```

**Test Object Detection:**
```bash
ros2 run sim_gazebo_bringup object_detector
```

**Test Vision Pick-and-Place:**
```bash
ros2 run sim_gazebo_bringup vision_pick_place
```

## Key Differences from ROS1 Version

### Architecture Changes
- **ROS1 → ROS2**: All code uses `rclpy` instead of `rospy`
- **Dynamic Reconfigure → Parameters**: Uses ROS2 parameter system
- **Topics**: Updated to match your ROS2 topic naming

### Integration with Existing Code
- Uses your existing `object_detector.py` for vision
- Compatible with your `gripper_mimic_relay.py` for parallel linkage
- Works with your Nav2 navigation stack
- Uses your FollowJointTrajectory action servers

### Parameters You Can Tune

**Vision Pick-and-Place Node:**
```yaml
approach_distance_m: 0.5      # Distance to stop from object (m)
approach_speed: 0.1           # Maximum approach velocity (m/s)
approach_timeout_sec: 10.0    # Timeout for approach phase (s)
arm_pid_p: 2.0                # PID proportional gain
arm_pid_i: 0.0                # PID integral gain
arm_pid_d: 0.5                # PID derivative gain
gripper_open: -1.54           # Open position (rad)
gripper_close: 0.0            # Close position (rad)
```

**Object Detector:**
```yaml
hsv_lower_h: 80               # Hue lower bound (0-180)
hsv_lower_s: 50               # Saturation lower bound (0-255)
hsv_lower_v: 50               # Value lower bound (0-255)
hsv_upper_h: 120              # Hue upper bound (0-180)
hsv_upper_s: 255              # Saturation upper bound (0-255)
hsv_upper_v: 255              # Value upper bound (0-255)
min_area: 200                 # Minimum contour area (pixels)
max_objects: 5                # Maximum objects to detect
```

## State Machine Flow

```
IDLE
  ↓ (object detected)
DETECT
  ↓
NAVIGATE (Nav2 to approach position)
  ↓
APPROACH (Vision-guided PID control)
  ↓
PICK (Open gripper → Move arm → Close gripper → Lift)
  ↓
TRANSPORT (Nav2 to place location)
  ↓
PLACE (Move arm → Open gripper → Retract)
  ↓
IDLE
```

## Comparison: Fixed vs Vision-Based Pick-and-Place

### Your Original `pick_and_place.py`
- ✅ Uses fixed object positions
- ✅ Direct cmd_vel drive to known location
- ✅ Fixed arm trajectories
- ✅ Hardcoded gripper positions
- ❌ Cannot adapt to moving/unknown objects

### New `vision_pick_place.py`
- ✅ Detects objects via camera
- ✅ Adapts to object position in real-time
- ✅ PID-controlled smooth approach
- ✅ Vision-guided arm positioning
- ✅ Can handle multiple objects by color
- ⚠️ Requires camera calibration
- ⚠️ More complex tuning

## Recommended Next Steps

1. **Test with your existing setup:**
   ```bash
   ros2 launch sim_gazebo_bringup vision_autopilot.launch.py
   ```

2. **Calibrate for your specific objects:**
   - Run object_detector in calibration mode
   - Adjust HSV ranges for your target colors
   - Save to `config/hsv_colors.yaml`

3. **Tune PID gains:**
   - Start with P=2.0, I=0.0, D=0.5
   - Increase P for faster response
   - Add D if oscillating
   - Only add I if there's steady-state error

4. **Integrate with your gripper physics:**
   - Update `gripper_open` and `gripper_close` values
   - Adjust arm joint positions for your workspace
   - Test with your parallel linkage constraints

## Troubleshooting

**"Module autopilot_common not found"**
- Rebuild: `colcon build --symlink-install`
- Source: `source install/setup.bash`

**"Navigation goal rejected"**
- Ensure Nav2 is running
- Check map frame exists
- Verify costmap configuration

**"No objects detected"**
- Check camera topics: `ros2 topic list | grep camera`
- Run calibration mode to adjust HSV ranges
- Verify object is in camera field of view

**"PID oscillation during approach"**
- Reduce P gain
- Increase D gain for damping
- Check approach_speed limit

## Files Created

```
scripts/x3plus_examples/
├── autopilot_common.py         # SimplePID + utility functions
└── vision_pick_place.py        # Vision-based pick-and-place node

config/
└── hsv_colors.yaml             # HSV color calibration

launch/
└── vision_autopilot.launch.py # Full system launch file
```

## Credits

Ported from ROS1 `arm_autopilot` package (Yahboomcar) to ROS2 Humble.
Adapted for X3Plus robot simulation in Gazebo Fortress.
