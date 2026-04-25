# Getting Started with Rosmaster X3 Plus

This guide shows how to set up the `x3plus_ws` workspace and access all key robot functions. Everything shown here is required to be done only once.

---

## 1. Prerequisites
- **Hardware**: Yahboom Rosmaster X3 Plus robot
- **OS**: Ubuntu 22.04
- **ROS 2**: Humble
- **Dependencies**:
  - `rosdep`
  - `colcon`
  - Yahboom Rosmaster SDK (`Rosmaster_Lib`)

---
## 2. Clone the workspace
```bash
git clone git@github.com:SynTEAM-Y/ROS2CoorAPI.git
cd ~/ROS2CoorAPI/robot_workspace/x3plus_ws
```

## 3. One-command bootstrap (installs deps, pulls camera stack, builds)

The bootstrap scripts automate the setup of your ROS 2 workspace by installing dependencies, fetching required packages (including camera stacks), and building everything in one go.  
Use the appropriate script depending on whether you’re setting up the robot or your PC.

### Option A) For the robot, run:
```bash
bash bootstrap_robot.sh
```
This installs all dependencies, pulls the camera stack, and builds the workspace.  
*(Takes around 5 minutes to complete.)*
### Option B) For the PC, run:
```bash
bash bootstrap_pc.sh
```
This performs the same setup, but tailored for the PC environment.  

#### After either script finishes, source the workspace:


```bash
source install/setup.bash
```

## 4. Install the provided `~/.bashrc`

This profile auto-loads ROS 2 Humble, adds helpers (`use_robot`, `launch_lidar`, `launch_camera`, etc.), and shows your IP/ROS info on login.

```bash
# Create a backup of your current .bashrc file
cp ~/.bashrc ~/.bashrc.backup.$(date +%Y%m%d_%H%M%S)

# Copy the .bashrc from the repo to the root
cp ~/ROS2CoorAPI/robot_workspace/x3plus_ws/.bashrc ~/.bashrc
# Apply it now in the current terminal
source ~/.bashrc

# Verify overlay
ros_ws_info
```
---

### Now that your setup is done, head over to [Using The Robot](using_the_robot.md) to start using the robot!