#!/bin/bash
# Start RViz with manual control for X3Plus robot

cd /home/othman/ROS2CoorAPI/robot_workspace/x3plus_ws
source install/setup.bash

export QT_QPA_PLATFORM=xcb
export LD_PRELOAD=/lib/x86_64-linux-gnu/libc.so.6:/lib/x86_64-linux-gnu/libpthread.so.0

echo "Starting RViz..."
ros2 launch sim_gazebo_bringup robot_rviz.launch.py map:=plain_map &
RVIZ_PID=$!

sleep 3

echo ""
echo "Starting manual control..."
echo "Use W/A/S/D to move, Space to stop, Q to quit"
echo ""
ros2 run sim_gazebo_bringup manual_control

# Cleanup
kill $RVIZ_PID 2>/dev/null
