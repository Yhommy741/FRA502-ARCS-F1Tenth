#!/usr/bin/python3

from arcsf1tenth_applications.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node


class DummyNode(Node):
    # Create the empty dummy node.
    def __init__(self):
        super().__init__('dummy_node')

        # Parameters

        # Variables

        # Publishers

        # Subscribers

        # Service Servers

        # Service Clients

        # Action Servers

        # Action Clients

        # Timers

        # On Start


# Start the node and spin until Ctrl+C.
def main(args=None):
    rclpy.init(args=args)
    node = DummyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
