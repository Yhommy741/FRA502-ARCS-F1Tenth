#!/usr/bin/python3

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanForSlam(Node):
    # Set up the node: scan/slam publisher and scan subscriber.
    def __init__(self):
        super().__init__('scan_for_slam')

        # Parameters

        # Variables

        # Publishers
        self.pub = self.create_publisher(LaserScan, 'scan/slam', 10)

        # Subscribers
        self.create_subscription(LaserScan, 'scan', self.on_scan, 10)

        # Service Servers

        # Service Clients

        # Action Servers

        # Action Clients

        # Timers

        # On Start

    # Replace +inf beams with range_max and republish the scan for Cartographer.
    def on_scan(self, msg):
        # +inf only: -inf (object closer than range_min) and nan (invalid) stay dropped
        msg.ranges = [msg.range_max if r == math.inf else r for r in msg.ranges]
        self.pub.publish(msg)


# Start the node and spin until Ctrl+C.
def main():
    rclpy.init()
    node = ScanForSlam()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
