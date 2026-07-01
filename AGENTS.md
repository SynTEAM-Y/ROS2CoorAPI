# AGENTS.md — ROS2CoorAPI

## Repository structure

This is a ROS 2 Humble monorepo with two independent build contexts:

### `communication_infra/` — Python communication layer
- **projectn** (ament_python): tree-based multi-agent coordination with REPL-driven robot/agent nodes
- **projectn_interfaces** (ament_cmake): custom msg/srv (DataMsg, RMsg, RegisterChild, GetMsgId, SubmitDm)
- Entrypoints: `server`, `agent`, `projectn-launch-random-tree`, `projectn-agent-hub`, `agent_supervisor`, `agent_hub_robot`, `robot_node`
- Python deps via rosdep/apt, NOT pip: `python3-fastapi`, `python3-uvicorn`, `python3-pydantic`
- Launch order: Tree → Agent → Robot (all must share same `ROS_DOMAIN_ID`)

### `robot_workspace/x3plus_ws/` — ROS 2 robot workspace
- Colcon workspace with packages for X3Plus robot bringup, sim, multi-robot, mapping, scenarios
- `sim_gazebo_bringup/` contains **isolated overrides** (scripts/ dir copies modified upstream code; CMakeLists.txt rewrites mesh URIs at configure time from `package://yahboomcar_description/` → `package://sim_gazebo_bringup/`)
- `scenarios/` package is **COLCON_IGNORE**-d (not built by default — remove that file to enable)
- `x3plus_multi_bringup/` handles multi-robot namespaced launches

## Build commands

```bash
# Communication infra (from communication_infra/)
colcon build --packages-select projectn projectn_interfaces
source install/setup.bash

# Robot workspace (from robot_workspace/x3plus_ws/)
colcon build --packages-select sim_gazebo_bringup
source install/setup.bash
```

## Key launch commands

### Communication infra (ROS_DOMAIN_ID must match everywhere)
```bash
ros2 run projectn projectn-launch-random-tree -- --root Root --nodes 6
ros2 run projectn agent_hub_robot --agent A1 --d-delay-s 0
ros2 run projectn robot_node --robot R1
```

### Simulation (RViz-only, no Gazebo needed)
```bash
ros2 launch sim_gazebo_bringup robot_rviz.launch.py map:=plain_map
```

### Full Gazebo simulation
```bash
ros2 launch sim_gazebo_bringup gazebo.launch.py world:=office use_rviz:=false
```

### Multi-robot
```bash
ros2 launch x3plus_multi_bringup one_robot.launch.py robot_id:=123 map:=/path/to/map.yaml
ros2 launch x3plus_multi_bringup multi_robot.launch.py robots_id:=123,456,789 prefix:=robot map:=/path/to/map.yaml
```

### Interactive menus at launch
Both `world:=<name>` and `map:=<name>` skip interactive prompts. Omit on a TTY to pick from a numbered menu. On non-TTY (CI), the prompt is silently skipped and the default is used.

## Simulation quirks

- `gazebo.launch.py` runs Gazebo-only by default (use_rviz:=false). Start RViz separately with `robot_rviz.launch.py`
- Gripper mimic joints are stripped from `/joint_states_raw` by `gripper_mimic_relay`, then recomputed by RSP via URDF `<mimic>` tags — only 13/18 joints in `/joint_states`
- `use_sim_time:=true` is the default in gazebo.launch.py; omitting it causes "No transform" errors
- `ros2 run sim_gazebo_bringup <node>` runs isolated override nodes (manual_control, arm_controller, gripper_mimic_relay, etc.)

## Testing

- `sim_gazebo_bringup` uses `ament_lint_auto` (activated by `BUILD_TESTING`)
- `x3plus_multi_bringup` has `pytest` in extras
- No CI/CD workflows exist in this repo

## Bootstrap

```bash
bash robot_workspace/x3plus_ws/bootstrap_pc.sh    # for PC
bash robot_workspace/x3plus_ws/bootstrap_robot.sh   # for robot hardware
```

Each script installs deps, pulls camera stack, and builds everything.

## Key packages and ownership

| Directory | Type | Purpose |
|-----------|------|---------|
| `communication_infra/` | Python | Tree/agent/robot coordination layer |
| `robot_workspace/x3plus_ws/src/sim_gazebo_bringup/` | CMake | Gazebo sim with isolated overrides |
| `robot_workspace/x3plus_ws/src/x3plus_multi_bringup/` | Python | Multi-robot namespaced launch |
| `robot_workspace/x3plus_ws/src/x3plus_config/` | CMake | Robot config, SRDF, MoveIt |
| `robot_workspace/x3plus_ws/src/yahboomcar_description/` | CMake | URDF/XACRO, meshes (shared) |
| `robot_workspace/x3plus_ws/src/scenarios/` | Python | Research scenarios (**COLCON_IGNORE** by default) |
