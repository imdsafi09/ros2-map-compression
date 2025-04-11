#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import zlib

class DecompressedMapSubscriber(Node):
    def __init__(self):
        super().__init__('decompressed_map_subscriber')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/compressed_map',
            self.map_callback,
            10)
        self.publisher = self.create_publisher(OccupancyGrid, '/map_decompressed', 10)
        self.get_logger().info("DecompressedMapSubscriber started, listening on /compressed_map")

    def map_callback(self, msg):
        try:
            # Convert from signed [-128,127] to unsigned [0,255]
            byte_data = bytes([(b + 256) if b < 0 else b for b in msg.data])

            # Decompress
            decompressed_bytes = zlib.decompress(byte_data)

            # Convert back to signed 8-bit integers
            signed_int8_data = [(b - 256 if b > 127 else b) for b in decompressed_bytes]

            msg.data = signed_int8_data
            self.publisher.publish(msg)
            self.get_logger().info(f"Published decompressed map with {len(msg.data)} cells")

        except zlib.error as e:
            self.get_logger().error(f"Decompression failed: {e}")
        except Exception as e:
            self.get_logger().error(f"Unhandled error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DecompressedMapSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

