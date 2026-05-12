#!/usr/bin/env python3

import math
import os
import re
import rclpy
from geometry_msgs.msg import Point, Quaternion
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


def parse_simple_yaml(path: str) -> dict:
    values = {}
    with open(path, 'r', encoding='utf-8') as file:
        for raw_line in file:
            line = raw_line.split('#', 1)[0].strip()
            if not line or ':' not in line:
                continue
            key, value = line.split(':', 1)
            key = key.strip()
            value = value.strip()
            if value.startswith(('"', "'")) and value.endswith(('"', "'")):
                value = value[1:-1]
            if key == 'image':
                values['image'] = value
            elif key == 'resolution':
                values['resolution'] = float(value)
            elif key == 'origin':
                match = re.search(r'\[(.*)\]', raw_line)
                if match:
                    values['origin'] = [float(x.strip()) for x in match.group(1).split(',')]
            elif key == 'negate':
                values['negate'] = int(value)
            elif key == 'occupied_thresh':
                values['occupied_thresh'] = float(value)
            elif key == 'free_thresh':
                values['free_thresh'] = float(value)
    return values


def load_pgm(path: str):
    with open(path, 'rb') as handle:
        magic = handle.readline().strip()
        if magic not in (b'P5', b'P2'):
            raise RuntimeError(f'Unsupported PGM format: {magic.decode()}')

        def read_token():
            while True:
                line = handle.readline()
                if not line:
                    raise RuntimeError('Unexpected end of PGM header')
                line = line.strip()
                if not line or line.startswith(b'#'):
                    continue
                for token in line.split():
                    yield token

        tokens = read_token()
        width = int(next(tokens))
        height = int(next(tokens))
        maxval = int(next(tokens))

        if magic == b'P5':
            data = handle.read(width * height)
            if len(data) != width * height:
                raise RuntimeError('PGM binary data is too short')
            pixels = list(data)
        else:
            pixels = []
            while len(pixels) < width * height:
                token = next(tokens)
                pixels.append(int(token))

    return width, height, maxval, pixels


def yaw_to_quaternion(yaw: float) -> Quaternion:
    half = yaw * 0.5
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(half)
    q.w = math.cos(half)
    return q


class MapPublisher(Node):
    def __init__(self):
        super().__init__('map_publisher')

        self.declare_parameter('map_path', '')
        map_path = self.get_parameter('map_path').value
        if not map_path:
            raise RuntimeError('map_path parameter is required')

        if not os.path.isabs(map_path):
            map_path = os.path.abspath(map_path)

        if not os.path.isfile(map_path):
            raise RuntimeError(f'Map YAML file not found: {map_path}')

        config = parse_simple_yaml(map_path)
        image_path = config.get('image')
        if image_path is None:
            raise RuntimeError('Map YAML missing image entry')

        if not os.path.isabs(image_path):
            image_path = os.path.join(os.path.dirname(map_path), image_path)

        resolution = config.get('resolution', 0.05)
        origin = config.get('origin', [0.0, 0.0, 0.0])
        negate = config.get('negate', 0)
        occupied_thresh = config.get('occupied_thresh', 0.65)
        free_thresh = config.get('free_thresh', 0.196)

        width, height, maxval, pixels = load_pgm(image_path)
        grid = []
        for value in pixels:
            pixel = maxval - value if negate else value
            if pixel >= occupied_thresh * maxval:
                grid.append(100)
            elif pixel <= free_thresh * maxval:
                grid.append(0)
            else:
                grid.append(-1)

        self.message = OccupancyGrid()
        self.message.header.frame_id = 'map'
        self.message.info.resolution = resolution
        self.message.info.width = width
        self.message.info.height = height
        self.message.info.origin.position = Point(x=origin[0], y=origin[1], z=0.0)
        self.message.info.origin.orientation = yaw_to_quaternion(origin[2])
        self.message.data = grid

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.publisher = self.create_publisher(OccupancyGrid, 'map', qos)
        self.publish_map()
        self.create_timer(1.0, self.publish_map)

    def publish_map(self) -> None:
        self.message.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.message)


def main():
    rclpy.init()
    node = MapPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
