#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

# -------------------------------------------------------------------
# GetCoords node
# - Queries the TF tree to obtain the current (x, y) position of a robot
# - Looks for map → base_footprint or map → base_link transforms
# - Prints coordinates once and exits
# -------------------------------------------------------------------
class GetCoords(Node):
    def __init__(self):
        super().__init__('get_coords')

        # Required parameter: robot name (used for TF frame lookup)
        self.declare_parameter('robot', '-')
        robot = self.get_parameter('robot').value
        if robot == '-':
            print('Error: Missing required param "robot", e.g. --ros-args -p robot:=robot123', file=sys.stderr)
            raise SystemExit(2)

        # Candidate TF frame roots and bases for flexibility
        # Tries both "map" and "<robot>_map" as roots
        # Tries both "<robot>_base_footprint" and "<robot>_base_link" as bases
        self.roots = ['map', f'{robot}_map']
        self.bases = [f'{robot}_base_footprint', f'{robot}_base_link']

        # TF2 buffer and listener for transform lookups
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)

    def try_print_once(self):
        """
        Attempt to find and print a transform between map and robot base.
        Returns True if successful, False otherwise.
        """
        for root in self.roots:
            for base in self.bases:
                try:
                    # Query latest available transform
                    t = self.buf.lookup_transform(root, base, rclpy.time.Time())
                    x = round(t.transform.translation.x, 3)
                    y = round(t.transform.translation.y, 3)
                    print(f'[{x}, {y}]', flush=True)
                    return True
                except (LookupException, ConnectivityException, ExtrapolationException):
                    continue
        return False


# -------------------------------------------------------------------
# Main entry point
# -------------------------------------------------------------------
def main():
    rclpy.init()
    node = GetCoords()

    # Spin briefly to allow TF data to populate before lookup
    done = False
    for _ in range(1000):  # ~10 seconds max (1000 * 0.1s)
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.try_print_once():
            done = True
            break

    # Clean up and exit
    node.destroy_node()
    rclpy.shutdown()

    if not done:
        print('Error: TF map->frame unavailable', file=sys.stderr)
        sys.exit(1)


if __name__ == '__main__':
    main()
