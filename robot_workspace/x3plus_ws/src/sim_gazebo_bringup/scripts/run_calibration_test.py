#!/usr/bin/env python3
"""
Simple calibration test runner - directly runs gripper calibration without full launch complexity.
This bypasses launch file issues and focuses on the core calibration test.
"""
import subprocess
import sys
import time
import os

def main():
    # Setup environment
    os.chdir('/home/othman/ROS2CoorAPI/robot_workspace/x3plus_ws')
    os.system('source install/setup.bash')
    
    print("=" * 80)
    print("GRIPPER CALIBRATION TEST - SIMPLE RUNNER")
    print("=" * 80)
    print()
    
    # Step 1: Start Gazebo and infrastructure (existing launch that works)
    print("[STEP 1] Starting Gazebo simulation (may take 15-20s)...")
    gazebo_cmd = '. install/setup.bash && timeout 350 ros2 launch sim_gazebo_bringup gazebo.launch.py gui:=false world:=empty 2>&1'
    gazebo_proc = subprocess.Popen(['bash', '-c', gazebo_cmd], 
                                   stdout=subprocess.PIPE, 
                                   stderr=subprocess.PIPE,
                                   text=True)
    print(f"  Gazebo PID: {gazebo_proc.pid}")
    
    # Wait for Gazebo to start
    print("  Waiting 20s for Gazebo to fully initialize...")
    time.sleep(20)
    
    # Step 2: Spawn the test cube
    print()
    print("[STEP 2] Spawning blue test cube at (2.0, 0.0, 0.03)...")
    sim_dir = '/home/othman/ROS2CoorAPI/robot_workspace/x3plus_ws/src/sim_gazebo_bringup'
    model_path = os.path.join(sim_dir, 'models', 'test_block', 'model.sdf')
    
    spawn_cmd = ('. install/setup.bash && '
                 'ros2 run ros_gz_sim create '
                 '-world empty '
                 f'-file {model_path} '
                 '-name test_block '
                 '-x 2.0 -y 0.0 -z 0.03')
    
    spawn_result = subprocess.run(['bash', '-c', spawn_cmd], 
                                  capture_output=True, 
                                  text=True,
                                  cwd='/home/othman/ROS2CoorAPI/robot_workspace/x3plus_ws')
    if spawn_result.returncode == 0:
        print("  ✓ Test cube spawned successfully")
    else:
        print(f"  ✗ Spawn failed: {spawn_result.stderr}")
    
    # Step 3: Wait briefly for cube to settle
    print("  Waiting 5s for cube to settle in simulation...")
    time.sleep(5)
    
    # Step 4: Start the calibration test
    print()
    print("[STEP 3] Starting gripper calibration test...")
    print("  This will test ~31 different grip values from -1.54 to 0.0 rad")
    print("  Expected duration: 3-5 minutes")
    print()
    print("-" * 80)
    
    calib_cmd = ('. install/setup.bash && '
                 'ros2 run sim_gazebo_bringup test_gripper_calibration')
    
    calib_proc = subprocess.Popen(['bash', '-c', calib_cmd],
                                  stdout=subprocess.PIPE,
                                  stderr=subprocess.STDOUT,
                                  text=True,
                                  cwd='/home/othman/ROS2CoorAPI/robot_workspace/x3plus_ws')
    
    # Capture output in real time
    output_lines = []
    try:
        while True:
            line = calib_proc.stdout.readline()
            if not line:
                break
            print(line.rstrip())
            output_lines.append(line)
    except KeyboardInterrupt:
        print("\n[INTERRUPTED] Stopping calibration test...")
    finally:
        calib_proc.wait()
    
    # Save output to file for parsing
    with open('/tmp/calibration_results.txt', 'w') as f:
        f.writelines(output_lines)
    
    print()
    print("-" * 80)
    print("[STEP 4] Calibration test completed. Cleaning up...")
    
    # Kill Gazebo
    gazebo_proc.terminate()
    try:
        gazebo_proc.wait(timeout=5)
    except:
        gazebo_proc.kill()
    
    print()
    print("=" * 80)
    print("TEST COMPLETE")
    print("=" * 80)
    print()
    print(f"Full output saved to: /tmp/calibration_results.txt")
    print()
    print("Next: Check results with: python3 parse_calibration.py")

if __name__ == '__main__':
    main()
