# scripts/ — your isolated, ACTIVE work

These files live under `sim_gazebo_bringup/` so the upstream packages
(`x3plus_examples`, `yahboomcar_description`) stay byte-identical to
`origin/main`, while your modifications are still the ones that actually
run.

## What gets installed (see `../CMakeLists.txt`)

| Source                                                     | Installed as                                                                 | How to run                                              |
|------------------------------------------------------------|-------------------------------------------------------------------------------|---------------------------------------------------------|
| `x3plus_examples/manual_control.py`                        | `lib/sim_gazebo_bringup/manual_control`                                       | `ros2 run sim_gazebo_bringup manual_control`            |
| `x3plus_examples/arm_controller.py`                        | `lib/sim_gazebo_bringup/arm_controller`                                       | `ros2 run sim_gazebo_bringup arm_controller`            |
| `x3plus_examples/gripper_mimic_relay.py`                   | `lib/sim_gazebo_bringup/gripper_mimic_relay`                                  | `ros2 run sim_gazebo_bringup gripper_mimic_relay`       |
| `yahboomcar_description/urdf/yahboomcar_X3plus.urdf.xacro` | `share/sim_gazebo_bringup/urdf/yahboomcar_X3plus.urdf.xacro` (mesh URIs auto-rewritten to `package://sim_gazebo_bringup/meshes/...`) | Picked up by `gazebo.launch.py`                         |
| `../meshes/`                                               | `share/sim_gazebo_bringup/meshes/`                                            | (resolved by the rewritten xacro)                       |

`yahboomcar_description/CMakeLists.txt` is kept only as a reference for
what the upstream build rule looks like — it is not installed.

## Build and use

```bash
cd ~/ROS2CoorAPI/robot_workspace/x3plus_ws
colcon build --packages-select sim_gazebo_bringup
source install/setup.bash
ros2 launch sim_gazebo_bringup gazebo.launch.py
ros2 run    sim_gazebo_bringup manual_control
```

The launch file prefers the in-package URDF and falls back to upstream
`yahboomcar_description` only if this package was not built.
