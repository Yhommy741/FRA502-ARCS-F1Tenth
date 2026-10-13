#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


def namespaced_frame(node, name):
    """TF frame in the node's namespace: /arcsf1tenth + imu -> arcsf1tenth/imu (no namespace -> imu)."""
    namespace = node.get_namespace().strip('/')
    return f'{namespace}/{name}' if namespace else name


class GazeboLidar(Node):
    """Simulated lidar driver: gazebo/scan -> scan, like the RPLidar driver on the real car.

    Sets header.frame_id to the URDF lidar frame (Gazebo stamps its own scoped
    sensor name, which does not exist in TF). Scan specs (range, beams, noise)
    are set on the Gazebo sensor from sensors.yaml.
    """

    def __init__(self):
        super().__init__('gazebo_lidar')

        self.declare_parameter('frame_id', namespaced_frame(self, 'lidar'))
        self.frame_id = self.get_parameter('frame_id').value

        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.create_subscription(LaserScan, 'gazebo/scan', self.scan_callback, 10)

    def scan_callback(self, msg):
        msg.header.frame_id = self.frame_id
        self.scan_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboLidar()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
