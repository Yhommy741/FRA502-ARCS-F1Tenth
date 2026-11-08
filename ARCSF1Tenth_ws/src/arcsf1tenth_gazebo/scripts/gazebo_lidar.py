#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


# Return the TF frame in the node's namespace (/arcsf1tenth + imu -> arcsf1tenth/imu, no namespace -> imu).
def namespaced_frame(node, name):
    namespace = node.get_namespace().strip('/')
    return f'{namespace}/{name}' if namespace else name


class GazeboLidar(Node):
    # Set up the node: frame parameter, scan publisher and subscriber.
    def __init__(self):
        super().__init__('gazebo_lidar')

        # Parameters
        self.declare_parameter('frame_id', namespaced_frame(self, 'lidar'))
        self.frame_id = self.get_parameter('frame_id').value

        # Variables

        # Publishers
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)

        # Subscribers
        self.create_subscription(LaserScan, 'gazebo/scan', self.scan_callback, 10)

        # Service Servers

        # Service Clients

        # Action Servers

        # Action Clients

        # Timers

        # On Start

    # Republish the Gazebo scan with the URDF lidar frame.
    def scan_callback(self, msg):
        msg.header.frame_id = self.frame_id
        self.scan_pub.publish(msg)


# Start the node and spin until Ctrl+C.
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
