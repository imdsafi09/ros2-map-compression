#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import zlib

class MapComparator(Node):
    def __init__(self):
        super().__init__('map_comparator')

        self.original_map = None
        self.decompressed_map = None

        self.sub_original = self.create_subscription(
            OccupancyGrid,
            '/robot1/map',
            self.original_callback,
            10)

        self.sub_decompressed = self.create_subscription(
            OccupancyGrid,
            '/map_decompressed',
            self.decompressed_callback,
            10)

        self.get_logger().info("MapComparator started. Comparing /robot1/map and /map_decompressed")

    def original_callback(self, msg):
        self.original_map = msg
        self.compare_maps()

    def decompressed_callback(self, msg):
        self.decompressed_map = msg
        self.compare_maps()

    def compare_maps(self):
        if self.original_map is None or self.decompressed_map is None:
            return

        # Check map metadata consistency
        if self.original_map.info != self.decompressed_map.info:
            self.get_logger().warn("Map metadata (resolution, width, height, origin) does NOT match!")
            return

        # Compare map data
        if self.original_map.data == self.decompressed_map.data:
            self.get_logger().info("Maps match: decompression was successful.")
        else:
            self.get_logger().error("Maps do NOT match: decompression failed or data was altered.")
            return

        # Compression analysis
        original_size = len(self.original_map.data)

        # Simulate compression (to match original compressed size)
        compressed_bytes = zlib.compress(bytes(self.original_map.data))
        compressed_size = len(compressed_bytes)

        compression_ratio = compressed_size / original_size if original_size > 0 else 0
        reduction_percent = 100 * (1 - compression_ratio)

        self.get_logger().info(f" Original map size:    {original_size} bytes")
        self.get_logger().info(f" Compressed map size: {compressed_size} bytes")
        self.get_logger().info(f" Compression ratio:    {compression_ratio:.2f} ({reduction_percent:.1f}% smaller)")

def main(args=None):
    rclpy.init(args=args)
    node = MapComparator()
    rclpy.spin(node)
    rclpy.shutdown()

