#!/bin/bash
pkill -f "ros2 launch sim_gazebo_bringup gazebo.launch.py" 2>/dev/null || true
pkill -f "rviz2" 2>/dev/null || true
sleep 2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch sim_gazebo_bringup gazebo.launch.py world:=empty map:=plain_map use_rviz:=true &
LAUNCH_PID=$!
sleep 14
echo "=== nodes ==="
ros2 node list | sort
echo "=== topics ==="
ros2 topic list | grep -E "/robot_description|/map|/tf|/joint_states|/joint_states_raw|/clock|/gz_pose_tf" || true
echo "=== /robot_description once ==="
ros2 topic echo --once /robot_description | head -20
echo "=== /map once ==="
ros2 topic echo --once /map | head -20
echo "=== /tf once ==="
ros2 topic echo --once /tf | grep -E "frame_id|child_frame_id|odom|base_footprint|base_link" | head -40
echo "=== /joint_states once ==="
ros2 topic echo --once /joint_states | head -20
kill $LAUNCH_PID
