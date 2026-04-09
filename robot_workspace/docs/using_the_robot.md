# Using the Rosmaster X3 Plus

This guide explains how to **access sensors, control the robot, and use helpers** once your workspace is set up.  
> **Note:** If you haven't yet built the workspace, see [Getting Started](getting_started.md).


## Table of Contents

- [1. Shell Helpers (`~/.bashrc`)](#1-shell-helpers-bashrc)
- [2. Rosmaster Library](#2-rosmaster-library)
- [3. Driving the Robot](#3-driving-the-robot)
  - [A) ROS 2](#a-ros-2)
  - [B) Direct SDK using `Rosmaster_Lib`](#b-direct-sdk-using-rosmaster_lib)
- [4. RGB Light bar](#4-rgb-light-bar)
  - [A) Built-in Effects (animations)](#a-built-in-effects-animations)
  - [B) Direct Color Control (static)](#b-direct-color-control-static)
- [5. Buzzer](#5-buzzer)
- [6. Controlling the Robotic Arm (Serial Bus Servos)](#6-controlling-the-robotic-arm-serial-bus-servos)
  - [A) Move a single servo](#a-move-a-single-servo)
  - [B) Move all servos together](#b-move-all-servos-together)
  - [C) Read angles & Torque control](#c-read-angles--torque-control)
- [7. Accessing the LiDAR](#7-accessing-the-lidar)
- [8. Accessing the Astra Plus Orbbec Camera (back camera)](#8-accessing-the-astra-plus-orbbec-camera-back-camera)
  - [Example viewer](#example-viewer)
- [9. Accessing the Front Camera](#9-accessing-the-front-camera)
- [10. Mapping & Navigation](#10-mapping--navigation)
  - [Mapping only (GMapping + RViz)](#a-mapping-only-gmapping--rviz)
  - [Reactive Navigation only (no SLAM)](#b-reactive-navigation-only-no-slam)
  - [Mapping and Navigation together](#c-mapping-and-navigation-together)
  - [ Save a map]()
- [11. Visualization](#11-visualization)
- [12. Example Code Catalog (x3plus_examples)](#12-example-code-catalog-x3plus_examples)
  - [Drive & SDK](#drive--sdk)
    - [rosmaster_base_bridge.py  +  navigation.py](#rosmaster_base_bridgepy--navigationpy)
    - [rosmaster_sequence.py](#rosmaster_sequencepy)
  - [LiDAR & Navigation](#lidar--navigation)
    - [closest.py](#closestpy)
    - [avoid_reflex.py](#avoid_reflexpy)
    - [lidar_viz_sectors.py](#lidar_viz_sectorspy)
  - [Cameras](#cameras)
    - [rgbd_ir_view.py](#rgbd_ir_viewpy)
  - [RGB Light Bar](#rgb-light-bar)
    - [rosmaster_rgb_gui.py](#rosmaster_rgb_guipy)
    - [display_battery.py](#display_batterypy)
- [13. Safety & Best Practices](#13-safety--best-practices)
- [14. Version & Quick Diagnostics](#14-version--quick-diagnostics)
- [15. Known Issues & Tips (Troubleshooting)](#15-known-issues--tips-troubleshooting)



---
&nbsp;
## 1. Shell Helpers (`~/.bashrc`)

The provided `.bashrc` adds a few commands, which can be used in the terminal:

- `ros_ws_info` – Displays information regarding the ROS workspace.
- `use_robot` – Sources the X3Plus workspace and driver overlays. 
- `launch_lidar` – Starts the LiDAR bringup.
- `launch_astra_camera` – Starts the Astra Plus depth camera bringup.
- `launch_front_camera` – Starts the front camera bringup.
- `save_map [name]` – Saves a Nav2 map to `~/maps`. If no `name` is included in the args, then it saves under the current date and time.
- `display_battery` – Displays the battery percentage on the RGB strip at the back of the robot.
- `bashrc_help` – Displays the commands that can be used.
- `clean_build [--pc]` – Performs a clean build by removing the log, build and install directories, then rebuilding everything. Use --pc to skip robot related packages like cameras and lidar drivers.
- `ros2_restart [--force]` – Safely stops all running ROS 2 nodes and restarts the ROS 2 daemon.
  - Use the `--force` flag to terminate stubborn processes with a hard kill (SIGKILL).


> These instructions can be accessed anytime in the terminal by running `bashrc_help`

---
&nbsp;

## 2. Rosmaster Library
`Rosmaster_Lib` is a python library that contains functions which can be used to control the robot, in order to be able to use those functions the following is required to be at the top of the python script:

> ```python
> from Rosmaster_Lib import Rosmaster
> bot = Rosmaster(debug=False)
> bot.create_receive_threading()
> bot.set_car_type(bot.CARTYPE_X3_PLUS)
> ```

---
&nbsp;
## 3. Driving the Robot

There are two ways to control the robot's movement. You can drive the robot either through ROS 2 (standard `/cmd_vel`) or directly via the SDK (Python API). ROS 2 is preferred for integration, SDK is preferred for quick testing or custom behaviors.

> ⚠️ **Safety First:** Before running any motion code, review [**Safety & Best Practices**](#13-safety--best-practices).

### A) ROS 2
Send velocity commands via `/cmd_vel`:
```bash
# In one terminal: run the bridge
use_robot
ros2 run x3plus_examples rosmaster_base_bridge
```

```bash
# In another terminal: publish velocity
use_robot

# manually
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"

# or using a python code
ros2 run x3plus_examples navigation
```
> See [**Rosmaster Base Bridge and Navigation**](#rosmaster_base_bridgepy--navigationpy) for an example use case.

### B) Direct SDK using `Rosmaster_Lib`
> Requires Rosmaster library, see [**Rosmaster library setup**](#2-rosmaster-library).

Control the state of the car directly using the Rosmaster library:


```python
bot.set_car_run(state, speed, adjust=False)
```

  #### Parameters:
- `state` (Movement state)  
  - `0` = stop
  - `1` = forward
  - `2` = backward
  - `3` = left
  - `4` = right
  - `5` = spin left
  - `6` = spin right

- `speed` (`-100 … 100`)  
  - `0` = stop  
  - Positive values = forward  
  - Negative values = reverse  
  - Values are interpreted as **percentage of maximum motor power**, not as exact m/s.
  - `100` = full power forward  
  - `-100` = full power backward  

- `adjust` (Not used)
  - Rreserved for future gyro assist, current firmware ignores it.
  - Default: `False`

> See [**Rosmaster Sequence**](#rosmaster_sequencepy) for an example use case.

---
&nbsp;
## 4. RGB Light bar
> Requires Rosmaster library, see [**Rosmaster library setup**](#2-rosmaster-library). 

Control the addressable RGB bar either with built-in effects or direct static colors (per LED or all LEDs).

### A) Built-in Effects (animations)
```python
bot.set_colorful_effect(effect, speed, parm=255)
``` 
**Parameters:**
- `effect`
  - `0` = Stop (turns off any running effect)
  - `1` = Running-water
  - `2` = Marquee
  - `3` = Breathing
  - `4` = Gradient
  - `5` = Starlight
  - `6` = Battery display (LEDs show battery level)
- `speed`
  - Range `1...10`(smaller number = faster animation)
- `parm` (optional)
  - Extra parameter for certain effect (e.g., breathing color preset `0...6`).
  - Safe default: leave as `255` when not needed.

> **Note:** Effects keep running until you call `set_colorful_effect(0)` to stop them. If you want to set static colors, stop the effect first.

### B) Direct Color Control (static)
```python
bot.set_colorful_lamps(led_id, r, g, b)
```

**Parameters:**
- `led_id`
  - `0xFF` = all LEDs
  - `0...13` = single LED index
- `r, g, b`
  - Each `0...255` (0 = off, 255 = full intensity) 

&nbsp;
> See [**RGB GUI**](#rosmaster_rgb_guipy) for an example use case.

---
&nbsp;
## 5. Buzzer
> Requires Rosmaster library, see [**Rosmaster library setup**](#2-rosmaster-library). 

Control the onboard buzzer with:

```python 
bot.set_beep(on_time)
```
**Parameters:**
- `on_time`
  - `0` = Turn off immediately
  - `1` = Continuous beep (stays on until you turn it off)
  - `≥10` = Beep for a fixed duration in milliseconds
    - Must be a multiple of 10 (e.g. 100, 250, 500) 

---
&nbsp;
## 6. Controlling the Robotic Arm (Serial Bus Servos)
> Requires Rosmaster library, see [**Rosmaster library setup**](#2-rosmaster-library). 

These servos are connected via the dedicated **UART bus** on the controller (not the PWM ports). Each has an ID (`1–6`) and can be moved individually or all at once.

### A) Move a single servo
```python
bot.set_uart_servo_angle(s_id, s_angle, run_time)
```

**Parameters:**
- `s_id` — Servo ID (`1...6`) *See label on the corresponding servo*
- `s_angle` — Target angle
  - IDs 1-4: `0...180°`
  - ID 5: `0...270°`
  - ID 6: `0...180°`
- `run_time` — Motion time in ms (`0...2000`), shorter=faster. Time it should take to reach the target angle `s_angle`.

### B) Move all servos together
```python
bot.set_uart_servo_angle_array([s_angle_1, s_angle_2, s_angle_3, s_angle_4, s_angle_5, s_angle_6], run_time)

# Example
bot.set_uart_servo_angle_array([90, 90, 90, 90, 90, 180], run_time=1000)
```
- Provide a list of six angles (one per ID). Any invalid angle or missing servo ID may be ignored.

### C) Read angles & Torque control
```python
# Returns angle of the servo with id "s_id", or -1 if error
print(bot.get_uart_servo_angle(s_id))

# Returns a list of 6 angles, one angle for each servo
print(bot.get_uart_servo_angle_array())

# Sets the torque of all servos, 1 = torque on (servo holds position), 0 = torque off (servo freewheels, not locked)
bot.set_uart_servo_torque(torque)
```

---
&nbsp;
## 7. Accessing the LiDAR

Start LiDAR bringup:
```bash
use_robot
launch_lidar
```

View raw data:
```bash
ros2 topic echo /scan
```

Run demos:
```bash
ros2 run x3plus_examples closest        # Show nearest obstacle
ros2 run x3plus_examples avoid_reflex   # Simple obstacle avoidance
```

---
&nbsp;
## 8. Accessing the Astra Plus Orbbec Camera (back camera)

This camera is located on the back of the robot on a rod. It is able to display RGB, Depth and IR images.

Start camera bringup:
```bash
launch_astra_camera
```

Typical topics:
- RGB: `/orbbec/color/image_raw`  
- Depth: `/orbbec/depth/image_raw`  
- IR: `/orbbec/ir/image_raw`  

### Example viewer:
```bash
ros2 run x3plus_examples rgbd_ir_view [args...]
```
The arguments are the following, which disables that specific frame from showing. If not provided, it defaults on showing. (Useful to increase the responsiveness of the display)
- `--no-show-rgb`
- `--no-show-depth-image`
- `--no-show-ir`
- `--no-show-pointcloud`

Example, running the following will only display the pointcloud:
```bash 
ros2 run x3plus_examples rgbd_ir_view --no-show-rgb --no-show-depth-image --no-show-ir
```

---
&nbsp;
## 9. Accessing the Front Camera
The front camera is mounted on the robot's arm (above the 5th servo). Use the bringup alias to start the camera driver, then view or consume the image topic.

```bash
# In one terminal: launch the camera which starts the camera bringup
launch_arm_camera

# In a second terminal: run the viewer (Optional)
use_robot
ros2 run rqt_image_view rqt_image_view
```
**The image can be accessed using the topic `/arm_cam/image_raw`**

---
&nbsp;
## 10. Mapping & Navigation

> Tip: see available launch arguments (with defaults):
```bash
ros2 launch x3plus_mapping_bringup mapping.launch.py --show-args
```

---

### A) Mapping only (GMapping + RViz)

**LiDAR already running in another terminal**
```bash
use_robot
ros2 launch x3plus_mapping_bringup mapping.launch.py \
  simulate:='false' start_lidar:='false'
```

**Let this launch also start the LiDAR**
```bash
use_robot
ros2 launch x3plus_mapping_bringup mapping.launch.py \
  simulate:='false' start_lidar:='true'
```
> **Note:** When passing booleans, use `True`/`False` (capitalized) or wrap in quotes: `simulate:="false"`.  

> **Reference:**  
> Detailed explanation of this launch file's structure and components can be found in  
> [**High-Level Documentation of `mapping.launch.py`**](high_level_docs/mapping.md).


---

### B) Reactive Navigation only (no SLAM)

```bash
use_robot
ros2 launch x3plus_mapping_bringup navigation.launch.py \
  simulate:='false' move_robot:='true'
```
- This runs `robot_state_publisher`, your base bridge, and `reactive_nav` (publishes `/cmd_vel` from `/scan`).

> **Reference:**  
> Detailed explanation of this launch file's structure and components can be found in  
> [**High-Level Documentation of `navigation.launch.py`**](high_level_docs/navigation.md).


---

### C) Mapping and Navigation together

```bash
use_robot
ros2 launch x3plus_mapping_bringup slam_and_nav.launch.py \
  simulate:='false' move_robot:='true' start_lidar:='true'
```
- Starts URDF → TF, odometry integrator, **GMapping**, **reactive_nav**, **RViz**, and (optionally) the LiDAR driver.

---

### Save a map
```bash
# Using the .bashrc helper method. (Name is optional)
save_map              # auto timestamped
save_map my_office    # custom name
```

---

### Notes / sanity checks
- In RViz, set **Fixed Frame = map** and add **TF**, **LaserScan(/scan)**, **Map(/map)**.
- Your LiDAR should publish with `frame_id: laser_link` (matches the URDF).


---
&nbsp;
## 11. Visualization

- **RViz2:**  
  ```bash
  # To open a plain Rviz2 view
  rviz2

  # To open a presaved Rviz2 view 
  rviz2 -d .path_to_rviz_view
  # (Example)
  rviz2 -d /x3plus_ws/src/x3plus_vision_bringup/rviz/orbbec_view.rviz 
  ```
  Useful for viewing LiDAR scans, TF, and maps.

- **Matplotlib (no RViz):**  
  ```bash
  ros2 run x3plus_examples lidar_sector_viz --topic /scan
  ```
> See [**Lidar Sector Viz**](#lidar_viz_sectorspy) for more information.

---
&nbsp;
## 12. Example Code Catalog (x3plus_examples)

Prerequisites for all examples, run this in every new terminal:
```bash
use_robot
```
> Paths below assume this repo layout: `../x3plus_ws/src/x3plus_examples/x3plus_examples/`

---

### Drive & SDK

#### rosmaster_base_bridge.py  +  navigation.py
**What it does:** Bridges `/cmd_vel` (ROS 2) to the SDK motor layer, then publishes velocities from Python.  
**Files:**  
- Bridge: [`rosmaster_base_bridge.py`](../x3plus_ws/src/x3plus_examples/x3plus_examples/rosmaster_base_bridge.py)  
- Publisher: [`navigation.py`](../x3plus_ws/src/x3plus_examples/x3plus_examples/navigation.py)

**Run (two terminals):**

**Terminal A – start the bridge**
```bash
use_robot
ros2 run x3plus_examples rosmaster_base_bridge
```

**Terminal B — publish velocity**
```bash
use_robot
# Option 1: run the publisher script
ros2 run x3plus_examples navigation

# Option 2: publish manually
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
```

**Related docs:** [Driving the Robot (ROS 2)](#a-ros-2).

---

#### rosmaster_sequence.py

**What it does:** Minimal SDK demo, initializes `Rosmaster_Lib`, sets car type, runs basic motions, buzzer, and RGB tests.  
**File:** [`rosmaster_sequence.py`](../x3plus_ws/src/x3plus_examples/x3plus_examples/rosmaster_sequence.py)  
**Run (one terminal):**
```bash
ros2 run x3plus_examples rosmaster_sequence
```
**Related docs:** [Driving the Robot (Direct SDK)](#b-direct-sdk-using-rosmaster_lib), [Buzzer](#5-buzzer), [RGB Light Bar](#4-rgb-light-bar).

---


### LiDAR & Navigation

#### closest.py
**What it does:** Reads `/scan` and prints/displays the nearest obstacle (range + angle).  
**File:** [`closest.py`](../x3plus_ws/src/x3plus_examples/x3plus_examples/closest.py)

**Run (two terminals):**

**Terminal A — bring up LiDAR**
```bash
use_robot
launch_lidar
```

**Terminal B — run the example**
```bash
use_robot
ros2 run x3plus_examples closest
```

**Related docs:** [Accessing the LiDAR](#7-accessing-the-lidar).

---

#### avoid_reflex.py
**What it does:** Simple reflex obstacle avoidance from `/scan`: drive forward until an obstacle is within a threshold, rotate to avoid.  
**File:** [`avoid_reflex.py`](../x3plus_ws/src/x3plus_examples/x3plus_examples/avoid_reflex.py)

**Run (two terminals):**

**Terminal A — bring up LiDAR**
```bash
use_robot
launch_lidar
```

**Terminal B — run the example**
```bash
use_robot
ros2 run x3plus_examples avoid_reflex
```

**Related docs:** [Accessing the LiDAR](#7-accessing-the-lidar), [Mapping & Navigation](#10-mapping--navigation).

---

#### lidar_viz_sectors.py
**What it does:** Headless Matplotlib visualizer, splits the scan into sectors and plots ranges (no RViz required).  
**File:** [`lidar_viz_sectors.py`](../x3plus_ws/src/x3plus_examples/x3plus_examples/lidar_viz_sectors.py)

**Run (two terminals recommended):**

**Terminal A — bring up LiDAR**
```bash
use_robot
launch_lidar
```

**Terminal B — run the visualizer**
```bash
use_robot
ros2 run x3plus_examples lidar_viz_sectors
```

**Related docs:** [Visualization](#11-visualization), [Accessing the LiDAR](#7-accessing-the-lidar).

---

### Cameras

#### rgbd_ir_view.py
**What it does:** RGB/Depth/IR/Pointcloud viewer with toggles.  
**File:** [`rgbd_ir_view.py`](../x3plus_ws/src/x3plus_examples/x3plus_examples/rgbd_ir_view.py)

**Run (two terminals):**

**Terminal A — start camera bringup**
```bash
use_robot
launch_astra_camera
```

**Terminal B — run the viewer (choose toggles as needed)**
```bash
use_robot
# show everything (default)
ros2 run x3plus_examples rgbd_ir_view

# only pointcloud
ros2 run x3plus_examples rgbd_ir_view --no-show-rgb --no-show-depth-image --no-show-ir
```

**Related docs:** [Astra Plus Orbbec Camera](#8-accessing-the-astra-plus-orbbec-camera-back-camera), [Example viewer](#example-viewer).

---

### RGB Light Bar

#### rosmaster_rgb_gui.py
**What it does:** GUI to control the RGB light bar (set static colors or trigger effects).  
**File:** [`rosmaster_rgb_gui.py`](../x3plus_ws/src/x3plus_examples/x3plus_examples/rosmaster_rgb_gui.py)

**Run (one terminal):**
```bash
ros2 run x3plus_examples rosmaster_rgb_gui
```
 
**Related docs:** [RGB Light Bar](#4-rgb-light-bar).

---

#### display_battery.py
**What it does:** Reads battery voltage/percent and displays it on the rear RGB strip.  
**File:** [`display_battery.py`](../x3plus_ws/src/x3plus_examples/x3plus_examples/display_battery.py)

**Run (one terminal):**
```bash
ros2 run x3plus_examples display_battery
```
**Related docs:** [Shell Helpers](#1-shell-helpers-bashrc), [RGB Light Bar](#4-rgb-light-bar).

---
&nbsp;
## 13. Safety & Best Practices

- **E-stop:** Keep a hand on `Ctrl+C` in the terminal running motion code.
- **Immediate stop:**
  - ROS 2:  
    ```bash
    ros2 topic pub -1 /cmd_vel geometry_msgs/Twist "{}"
    ```
  - SDK:  
    ```python
    bot.set_car_run(0, 0, False)
    ```
- **First runs:** Test with wheels off the ground or on a stand.
- **Indoor limits:** Keep speeds modest (e.g., `|speed| ≤ 30`) while iterating.

---
&nbsp;
## 14. Version & Quick Diagnostics

```python
from Rosmaster_Lib import Rosmaster
bot = Rosmaster()
print("SDK:", bot.get_version())
print("Car type:", bot.get_car_type_from_machine())
print("Battery (V):", bot.get_battery_voltage())
bot.create_receive_threading()
print("IMU (yaw, roll, pitch):", bot.get_imu_attitude_data(ToAngle=True))
```

**ROS 2 sanity checks**
```bash
# Topics present?
ros2 topic list

# Is LiDAR publishing?
ros2 topic echo -n 1 /scan

# Bandwidth/Hz for a topic:
ros2 topic hz /scan
```

**MCU recovery**
```python
# If saved settings broke telemetry/effects, restore factory defaults:
bot.reset_flash_value()
```

---
&nbsp;
## 15. Known Issues & Tips (Troubleshooting)

- **Static RGB won't “stick”:** Stop animations first:
  ```python
  bot.set_colorful_effect(0)
  ```
&nbsp;

- **No `/scan` data:** Ensure LiDAR bringup ran:
  ```bash
  use_robot
  launch_lidar
  ros2 topic list | grep scan
  ```
&nbsp;

- **LiDAR fails to start**, such as:
  - `[error] Error, cannot retrieve Lidar health code -1`
  - `[error] Fail to get baseplate device information!`

  This usually means the driver is talking to the wrong USB port. The LiDAR might appear as `/dev/ttyUSB0` on one boot and `/dev/ttyUSB1` on another, depending on what else is plugged in.

  **Fix:** Open the ydlidar parameter file [`tg30.yaml`](../x3plus_ws/src/x3plus_lidar_bringup/params/tg30.yaml) and update the `port` field:
  ```yaml
  port: /dev/ttyUSB0   # or /dev/ttyUSB1, depending on where it appears
  ```
&nbsp;

- **`/cmd_vel` doesn't move:** Confirm the bridge is up:
  ```bash
  ros2 run x3plus_examples rosmaster_base_bridge
  ```
&nbsp;
- **Camera viewer blank:** Verify topic selection in `rqt_image_view`, confirm bringup:
  ```bash
  use_robot
  launch_astra_camera
  ros2 topic list | grep arm_cam
  ```
&nbsp;
- **Depth camera blank or showing black:**
  ```bash
  use_robot
  launch_astra_camera
  ros2 topic list | grep arm_cam
  ```
  - If the stream is still blank, ensure the depth camera is connected to a USB 3.0 port. Using a USB 2.0 port or sharing the hub with other high-bandwidth devices can cause insufficient data throughput and prevent the depth stream from appearing.
  
  &nbsp;
- **Laggy viewers:** Close extra RViz panels, lower rate/size or use compressed image transport if available.

  &nbsp;
- **Error when running `bash bootstrap_pc.sh`**:
  - Errors such as `Err:7 http://packages.ros.org/ros/ubuntu bionic InRelease
    The following signatures were invalid: ...`  
    To resolve this issue run the following command:
    ```bash
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    ```
  - Errors such as `missing xacro`, `cmake` or any other library can simply be resolved by downloading that library. If it is a ROS-dependant library use the following command:
    ```bash
    # For example xacro
    sudo apt install ros-$ROS_DISTRO-xacro 
    ```

  &nbsp;
- **Error when using `rviz2`**
  - Error such as the one below can be safely **ignored**, it is harmless and just a graphical issue. Everything *should* still work as intended.
  ```log
    [rviz2-35] [ERROR] [rviz2]: Vertex Program:rviz/glsl120/indexed_8bit_image.vert Fragment Program:rviz/glsl120/indexed_8bit_image.frag GLSL link result : 
    [rviz2-35] active samplers with a different type refer to the same texture image unit
  ```