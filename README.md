# ROS2Coordination Project

This repository contains the software stack for multi robot coordination using ROS 2.  
It includes communication infrastructure, robot bringup, simulation tools, and research scenarios.  
The structure separates communication layers from robot specific functionality for clarity and scalability.

---

## Repository-Structure

- [**`communication_infra/`**](./communication_infra/) – Communication layer: AgentHub + infra runtime
    - [**`AgentHub/`**](./communication_infra/AgentHub/) – High-level agent / feed / sequence configs
    - [**`Scenarios/`**](./communication_infra/Scenarios/) – Scenario programs (yaml files)
    - [**`logs/`**](./communication_infra/logs/) – Runtime logs
    - [**`src/`**](./communication_infra/src/) – Supporting scripts / utilities
        - [**`projectn/`**](./communication_infra/src/projectn/) – Core backend for communication / coordination
        - [**`projectn_interfaces/`**](./communication_infra/src/projectn_interfaces/) – Interface definitions and shared types

- [**`robot_workspace/`**](./robot_workspace/) – Main ROS 2 workspace for robot, simulation, and scenarios
    - [**`x3plus_ws/`**](./robot_workspace/x3plus_ws/) – Core ROS 2 packages for the X3Plus platform
        - [**`src/`**](./robot_workspace/x3plus_ws/src/) 
            - [**`scenarios/`**](./robot_workspace/x3plus_ws/src/scenarios/) – Research scenarios (ROS 2)
                - [**`patrol_rescue/`**](./robot_workspace/x3plus_ws/src/scenarios/patrol_rescue/) – Scenario 2
                - [**`pick_and_place/`**](./robot_workspace/x3plus_ws/src/scenarios/pick_and_place/) – Scenario 3 (in progress)
            - [**`x3plus_config/`**](./robot_workspace/x3plus_ws/src/x3plus_config/) – Robot configuration and parameters
            - [**`x3plus_examples/`**](./robot_workspace/x3plus_ws/src/x3plus_examples/) – Examples: navigation, LiDAR, RGB, SDK bridge
            - [**`x3plus_lidar_bringup/`**](./robot_workspace/x3plus_ws/src/x3plus_lidar_bringup/) – LiDAR drivers and launch files
            - [**`x3plus_mapping_bringup/`**](./robot_workspace/x3plus_ws/src/x3plus_mapping_bringup/) – Mapping and SLAM bringup
            - [**`x3plus_multi_bringup/`**](./robot_workspace/x3plus_ws/src/x3plus_multi_bringup/) – Multi-robot navigation and namespace handling
            - [**`x3plus_sim_bringup/`**](./robot_workspace/x3plus_ws/src/x3plus_sim_bringup/) – Simulation bringup
            - [**`x3plus_vision_bringup/`**](./robot_workspace/x3plus_ws/src/x3plus_vision_bringup/) – Cameras and vision bringup
            - [**`yahboomcar_description/`**](./robot_workspace/x3plus_ws/src/yahboomcar_description/) – URDF and robot model
        - [**`maps/`**](./robot_workspace/x3plus_ws/maps/) – Navigation maps for real or simulated robots

---

## Getting started

### Clone the repository
```bash
git clone git@github.com:SynTEAM-Y/ROS2CoorAPI.git
cd ROS2CoorAPI
```

### Robot workspace
To get started with the robot workspace see the robot_workspace [**README**](./robot_workspace/README.md).

### Communication infrastructure
To get started with the communication infrastructure see the communication_infra [**README**](./communication_infra/How%20to%20run%201.0.md).

---

## Documentation Overview

All project documentation is intentionally separated from the code and organized by purpose.

### Robot and Simulation Documentation

Located in [**`robot_workspace/docs/`**](./robot_workspace/docs/)

This folder contains all user facing documentation related to robot usage, simulation, and system behavior:

- [**`getting_started.md`**](./robot_workspace/docs/getting_started.md) – Initial setup instructions for the robot workspace and development environment.

- [**`using_the_robot.md`**](./robot_workspace/docs/using_the_robot.md) – How to operate the Rosmaster X3 Plus, including sensors, motion control, arm servos, RGB lighting, cameras, and example ROS2 commands.

- [**`high_level_docs/`**](./robot_workspace/docs/high_level_docs/) – High-level design and reference documentation:
  - [**`mapping.md`**](./robot_workspace/docs/high_level_docs/mapping.md) – Mapping and SLAM architecture.
  - [**`navigation.md`**](./robot_workspace/docs/high_level_docs/navigation.md) – Navigation and planning overview.

- [**`simulation_and_scenarios.md`**](./robot_workspace/docs/simulation_and_scenarios.md) – How to run the multi-robot simulation, navigate in RViz, and launch research scenarios.

### Communication Infrastructure Documentation

Located in [**`How to run 1.0.md`**](./communication_infra/How%20to%20run%201.0.md)

This document explains how to start and operate the communication infrastructure used for multi-robot coordination.

It is only required when working on or deploying the communication layer.

---

## Scenarios Included

This repository includes multiple research-oriented multi-robot scenarios,
designed to run consistently in both simulation and on real hardware.

### Consensus in Anonymous Multi Robot Systems
Robots coordinate their motion using local interactions only, 
without relying on global identifiers or centralized control.

### Patrol & Rescue
Robots autonomously patrol an environment.
When a rescue event occurs, one robot temporarily leaves patrol,
navigates to a rescue location and rescues the robot.

### Pick and Place (Work in Progress)
An upcoming scenario focusing on manipulation and coordination, including arm control, object pickup, and task execution logic.


> **Note:** Each scenario is modular, namespaced, and designed to scale from single-robot testing to multi-robot deployments.
