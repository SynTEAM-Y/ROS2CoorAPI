# QUICK START: Gripper Calibration Test (Ready to Run)

## Prerequisites
```bash
cd ~/ROS2CoorAPI/robot_workspace/x3plus_ws
source install/setup.bash
```

## One-Command Full Test (Copy & Paste)

```bash
# Terminal 1: Start Gazebo (run once, let it stay)
( echo "1" && sleep 360 ) | timeout 370 ros2 launch sim_gazebo_bringup gazebo.launch.py gui:=false world:=empty 2>&1 > /tmp/gazebo.log &
sleep 30

# Terminal 2: Spawn cube, start relays, run test (sequential commands)
source install/setup.bash && \
ros2 run ros_gz_sim create -world empty -file $(pwd)/src/sim_gazebo_bringup/models/test_block/model.sdf -name test_block -x 2.0 -y 0.0 -z 0.03 && \
sleep 2 && \
ros2 run sim_gazebo_bringup gripper_mimic_relay > /tmp/gripper_relay.log 2>&1 &
sleep 1 && \
ros2 run sim_gazebo_bringup gazebo_pose_tf_relay --ros-args -p input_topic:=/gz_pose_tf -p parent_frame:=odom -p child_frame:=test_block -p input_type:=tf -p source_child:=test_block > /tmp/test_block_tf_relay.log 2>&1 &
sleep 3 && \
echo "=== STARTING GRIPPER CALIBRATION TEST ===" && \
timeout 300 ros2 run sim_gazebo_bringup test_gripper_calibration 2>&1 | tee /tmp/calibration_final.txt
```

## View Results
```bash
grep -E "Grip Value|Servo Angle|Cube Z|✓|✗|optimal|UPDATE" /tmp/calibration_final.txt | tail -50
```

## Extract Optimal Value
```bash
# Automatically extract the optimal value:
OPTIMAL=$(grep "Optimal GRIPPER_HOLD" /tmp/calibration_final.txt | awk '{print $NF}')
echo "Update vision_autopilot_simple.py line 183 with:"
echo "GRIPPER_HOLD = $OPTIMAL"
```

## Update & Test
```bash
# Edit the file and update line 183:
sed -i "s/GRIPPER_HOLD.*=.*/GRIPPER_HOLD = $(echo $OPTIMAL | cut -d'=' -f2)/g" \
  src/sim_gazebo_bringup/scripts/x3plus_examples/vision_autopilot_simple.py

# Rebuild:
colcon build --packages-select sim_gazebo_bringup --symlink-install

# Test full pick-and-place:
ros2 launch sim_gazebo_bringup vision_autopilot_simple.launch.py world:=empty
```

## Troubleshooting

### No results in calibration table?
- Check TF relay logs: `tail -20 /tmp/test_block_tf_relay.log`
- Verify test_block spawned: Check Gazebo (should see blue cube at (2.0, 0.0))
- Check ROS topic: `ros2 topic list | grep gz_pose`

### Gripper not responding?
- Check gripper relay: `tail -20 /tmp/gripper_relay.log`
- Verify commands sent: `ros2 topic echo /grip_joint_cmd_pos` (should see values changing)

### Gazebo crashes?
- Gazebo needs display config: Leave it running first terminal
- Check GPU memory: `nvidia-smi`

## Expected Timeline
- Gazebo startup: 30s
- Cube spawn + relays: 5s
- Calibration test: 3-5 min
- Result parsing: 10s
- **Total: ~5 minutes**

## Success Indicators
✓ Test outputs 31 grip values tested (from -1.540 to -0.040)
✓ Table shows mix of "✗ NOT LIFTED" and "✓ PICKED"
✓ "Optimal GRIPPER_HOLD = X.XXX" printed
✓ "UPDATE vision_autopilot_simple.py" section shown
