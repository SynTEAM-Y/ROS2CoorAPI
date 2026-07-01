#!/usr/bin/env python3
"""Diagnostic tool to check gripper mimic joint behavior."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class GripperDiagnostic(Node):
    def __init__(self):
        super().__init__('gripper_diagnostic')
        
        self.joint_states = {}
        self.mimic_commands = {}
        
        # Subscribe to joint states from all robots
        for robot in ['robot_1', 'robot_2', 'robot_3']:
            self.create_subscription(
                JointState,
                f'/{robot}/joint_states',
                lambda msg, r=robot: self.joint_states_callback(msg, r),
                10
            )
            
            # Subscribe to mimic command topics
            for joint in ['rlink_joint2', 'rlink_joint3', 'llink_joint1', 'llink_joint2', 'llink_joint3']:
                topic = f'/{robot}/{joint}_cmd_pos'
                self.create_subscription(
                    Float64,
                    topic,
                    lambda msg, r=robot, j=joint: self.mimic_cmd_callback(msg, r, j),
                    10
                )
            
            # Subscribe to grip master target
            self.create_subscription(
                Float64,
                f'/{robot}/grip_master_target',
                lambda msg, r=robot: self.grip_master_callback(msg, r),
                10
            )
        
        self.timer = self.create_timer(1.0, self.print_status)
        
    def joint_states_callback(self, msg, robot):
        if robot not in self.joint_states:
            self.joint_states[robot] = {}
        
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                # Strip robot prefix
                bare_name = name.replace(f'{robot}_', '')
                self.joint_states[robot][bare_name] = msg.position[i]
    
    def mimic_cmd_callback(self, msg, robot, joint):
        if robot not in self.mimic_commands:
            self.mimic_commands[robot] = {}
        self.mimic_commands[robot][joint] = msg.data
    
    def grip_master_callback(self, msg, robot):
        if robot not in self.mimic_commands:
            self.mimic_commands[robot] = {}
        self.mimic_commands[robot]['grip_master'] = msg.data
    
    def print_status(self):
        print("\n" + "="*80)
        print("GRIPPER DIAGNOSTIC STATUS")
        print("="*80)
        
        for robot in sorted(self.joint_states.keys()):
            print(f"\n{robot.upper()}:")
            print("-" * 40)
            
            # Print grip master command
            if robot in self.mimic_commands and 'grip_master' in self.mimic_commands[robot]:
                grip_cmd = self.mimic_commands[robot]['grip_master']
                print(f"  grip_master_target: {grip_cmd:.4f}")
            
            # Print grip_joint actual position
            if 'grip_joint' in self.joint_states[robot]:
                grip_pos = self.joint_states[robot]['grip_joint']
                print(f"  grip_joint actual:  {grip_pos:.4f}")
            
            # Print mimic joints
            mimic_joints = ['rlink_joint2', 'rlink_joint3', 'llink_joint1', 'llink_joint2', 'llink_joint3']
            for joint in mimic_joints:
                actual = self.joint_states[robot].get(joint, None)
                cmd = self.mimic_commands.get(robot, {}).get(joint, None)
                
                if actual is not None or cmd is not None:
                    actual_str = f"{actual:.4f}" if actual is not None else "N/A"
                    cmd_str = f"{cmd:.4f}" if cmd is not None else "N/A"
                    
                    # Compute expected from URDF mimic
                    if 'grip_joint' in self.joint_states[robot]:
                        grip = self.joint_states[robot]['grip_joint']
                        multipliers = {
                            'rlink_joint2': -1.0,
                            'rlink_joint3': +1.0,
                            'llink_joint1': -1.0,
                            'llink_joint2': +1.0,
                            'llink_joint3': -1.0,
                        }
                        expected = grip * multipliers[joint]
                        expected_str = f"{expected:.4f}"
                    else:
                        expected_str = "N/A"
                    
                    print(f"  {joint:15s}: actual={actual_str:8s}  cmd={cmd_str:8s}  expected={expected_str:8s}")


def main():
    rclpy.init()
    node = GripperDiagnostic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
