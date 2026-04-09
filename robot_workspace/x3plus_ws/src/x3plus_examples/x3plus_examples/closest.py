import rclpy, math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSPresetProfiles

class Closest(Node):
    def __init__(self):
        super().__init__('closest') # node name
        # Subscribes to /scan with LIDAR-friendly QoS (low latency, best-effort)
        # self.create_subscription(LaserScan, '/scan', self.cb, 10)  <-- Hardcoded value (used for testing)
        self.create_subscription(LaserScan, '/scan', self.cb,
                                QoSPresetProfiles.SENSOR_DATA.value)

    def cb(self, msg: LaserScan):
        best = None
        best_i = -1

        # Keeps the smallest finite range within declared sensor bounds        
        for i, r in enumerate(msg.ranges):
            if math.isfinite(r) and r >= msg.range_min and r <= msg.range_max:
                if best is None or r < best:
                    best, best_i = r, i

        if best is None:
            # Logs when no usable readings are found
            self.get_logger().info("No valid returns")
            return
            
        # Converts the winning index to an angle (radians)
        angle = msg.angle_min + best_i * msg.angle_increment
        # Logs the closest distance in meters and its bearing in degrees
        self.get_logger().info(f"closest: {best:.2f} m @ {math.degrees(angle):.1f}°")

def main():
    rclpy.init()              # Initializes the ROS client library
    rclpy.spin(Closest())     # Runs the node until shutdown
    rclpy.shutdown()          # Performs cleanup on exit
