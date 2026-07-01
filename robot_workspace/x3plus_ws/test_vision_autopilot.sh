#!/bin/bash
# Test script for vision_autopilot_simple pick and place

set -e

cd /home/othman/ROS2CoorAPI/robot_workspace/x3plus_ws

echo "=========================================="
echo "VISION AUTOPILOT SIMPLE - PICK & PLACE TEST"
echo "=========================================="
echo ""
echo "1. Sourcing workspace..."
source install/setup.bash

echo ""
echo "2. Launch parameters:"
echo "   - World: empty (faster than office)"
echo "   - Cube: 2cm × 2cm × 2cm at (2.0, 0.0)"
echo "   - Landing pad: 50cm × 50cm at (2.0, 1.2)"
echo ""
echo "3. Startup sequence:"
echo "   - t=0s:   Gazebo starts"
echo "   - t=15s:  Cube spawned"
echo "   - t=17s:  Landing pad spawned"
echo "   - t=16s:  TF relays for cube activated"
echo "   - t=18s:  TF relay for landing pad activated"
echo "   - t=25s:  Vision autopilot node starts"
echo ""
echo "4. Expected behavior:"
echo "   - Robot arm moves to DRIVE_POSE"
echo "   - Robot navigates to cube (0.65m approach distance)"
echo "   - Robot aligns with cube face"
echo "   - Wrist camera HSV detection fine-tunes approach (final 0.3m)"
echo "   - Robot picks cube (gripper closes on cube)"
echo "   - Grasp verified by checking cube Z height > 0.10m"
echo "   - Robot drives to landing pad (2.0, 1.2)"
echo "   - Robot places cube and backs away"
echo "   - Task complete!"
echo ""
echo "=========================================="
echo "LAUNCHING NOW..."
echo "=========================================="
echo ""

# Launch with empty world (faster) and capture output
ros2 launch sim_gazebo_bringup vision_autopilot_simple.launch.py world:=empty 2>&1

