#!/usr/bin/python3
"""Lidar scan for Cartographer: scan -> scan/slam, "no return" (inf) beams kept as free space.

A beam that hits nothing within range comes back as inf (REP 117). cartographer_ros drops
every beam outside [range_min, range_max], so an inf beam adds no free space at all. On a
straight longer than the lidar range the beams straight ahead are all inf, and the cells
along the driven path stayed light grey in the map (only marked free when the car was on
top of them).

Here inf becomes range_max (kept by cartographer_ros). cartographer_2d.lua sets
TRAJECTORY_BUILDER_2D.max_range just below range_max, so Cartographer treats these beams
as "no wall" and draws missing_data_ray_length of free space along them.

Only Cartographer uses scan/slam: scan stays untouched (wall_follow reads inf as "nothing
ahead", which is correct). Works the same in simulation and on the real car.
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanForSlam(Node):
    def __init__(self):
        super().__init__('scan_for_slam')
        self.pub = self.create_publisher(LaserScan, 'scan/slam', 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, 10)

    def on_scan(self, msg):
        # +inf only: -inf (object closer than range_min) and nan (invalid) stay dropped
        msg.ranges = [msg.range_max if r == math.inf else r for r in msg.ranges]
        self.pub.publish(msg)


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
