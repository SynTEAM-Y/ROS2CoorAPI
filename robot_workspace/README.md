# Yahboom Rosmaster X3 Plus - Robot Workspace

This directory contains the **ROS 2 workspace, simulation setup, and documentation**
for the **Yahboom X3 Plus** platform.

It is intended for users and developers who want to:
- Control the robot using ROS 2
- Run single- and multi-robot simulations
- Experiment with navigation, perception, and manipulation
- Execute research scenarios in simulation or on real hardware

The workspace is designed to scale from **single-robot testing** to **multi-robot coordination**.

---

## What’s Included

- ROS 2 packages for:
  - Robot bringup and description
  - LiDAR, cameras, mapping, and navigation
  - Multi-robot namespace handling
  - Simulation bringup
- Example nodes and SDK bridges
- Research scenarios (e.g. consensus, patrol & rescue, pick and place)
- Structured documentation for users and developers

---

## Getting Started

Start here if this is your first time using the workspace:

- [**Getting Started Guide**](docs/getting_started.md)

This covers:
- Environment setup
- Workspace build
- Sourcing and basic verification

---

## Repository Layout

- `docs/` – Documentation for robot usage and simulation
  - `getting_started.md` – Setup and installation
  - `using_the_robot.md` – Sensors, motion, arm, RGB, cameras, examples
  - `simulation_and_scenarios.md` – Multi-robot simulation and scenarios
  - `high_level_docs/` – Design-level documentation
    - `mapping.md`
    - `navigation.md`

- `x3plus_ws/` – Main ROS 2 workspace
  - `src/` – ROS 2 packages
    - `x3plus_config/` – Robot configuration and parameters
    - `x3plus_examples/` – Examples and demos (LiDAR, RGB, SDK bridge)
    - `x3plus_lidar_bringup/` – LiDAR drivers and launch files
    - `x3plus_mapping_bringup/` – Mapping and SLAM
    - `x3plus_multi_bringup/` – Multi-robot bringup and namespacing
    - `x3plus_sim_bringup/` – Simulation bringup
    - `x3plus_vision_bringup/` – Cameras and vision
    - `yahboomcar_description/` – URDF and robot model
    - `scenarios/` – Research scenarios
      - `patrol_rescue/`
      - `pick_and_place/` (work in progress)
  - `maps/` – Navigation maps for real and simulated robots

---

## Features

- Ready-to-run ROS 2 examples for:
  - LiDAR data access and obstacle avoidance
  - Camera visualization (RGB, depth, IR, pointcloud)
  - Navigation and mapping (Nav2)
- Multi-robot simulation support with namespaced robots
- Direct base control via Yahboom SDK (`Rosmaster_Lib`)
- Research-oriented scenarios runnable in simulation or on hardware

---

## Documentation Overview

- **Robot usage and control**
  - [Using the Robot](docs/using_the_robot.md)

- **Simulation and research scenarios**
  - [Simulation & Scenarios Guide](docs/simulation_and_scenarios.md)

- **System-level design**
  - [High-Level Docs](docs/high_level_docs/)

Each document focuses on *what to run and why*, while implementation details live in the code.

---

## Useful Links

- [Official Rosmaster X3 Plus Documentation (ROS 1)](https://www.yahboom.net/study/ROSMASTER-X3-PLUS)
- [ROS 2 Humble Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [RViz2 User Guide](https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/RViz-User-Guide/RViz-User-Guide.html)

> **Note:** The official Rosmaster X3 Plus documentation targets **ROS 1**.  
> This workspace and documentation are fully **ROS 2 (Humble)** based.
