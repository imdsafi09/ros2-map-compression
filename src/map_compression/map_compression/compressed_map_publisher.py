#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import zlib
from copy import deepcopy

class CompressedMapPublisher(Node):
    def __init__(self):
        super().__init__('compressed_map_publisher')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)
        self.publisher = self.create_publisher(OccupancyGrid, '/compressed_map', 10)
        self.get_logger().info("CompressedMapPublisher started, listening on /ro/map")

    def map_callback(self, msg):
        # Make a deep copy of the original message to avoid modifying it
        compressed_msg = deepcopy(msg)

        # Compress the original map data
        compressed_bytes = zlib.compress(bytes(msg.data))

        # Convert to signed 8-bit values (int8[] expected by OccupancyGrid)
        signed_compressed = [(b - 256 if b > 127 else b) for b in compressed_bytes]

        # Replace data field with compressed signed bytes
        compressed_msg.data = signed_compressed

        # Publish the compressed map
        self.publisher.publish(compressed_msg)
        self.get_logger().info(f"Published compressed map: {len(signed_compressed)} bytes")

def main(args=None):
    rclpy.init(args=args)
    node = CompressedMapPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

