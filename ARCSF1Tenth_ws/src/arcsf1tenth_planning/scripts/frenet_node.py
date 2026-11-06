#!/usr/bin/python3
"""Frenet frame of the global path: publishes the car's Frenet state and the global trajectory.

    global path CSV (centerline / raceline) ─┐
    map (map_server, for the track widths) ──┼──► frenet_node ──► global_trajectory   (latched, closed, with track widths)
    TF map -> base_footprint ────────────────┤                ──► global_path         (nav_msgs/Path for RViz)
    odometry/estimated (speed) ──────────────┘                ──► frenet/state        (s, d, s_dot, d_dot, yaw error)
                                                              ──► track_bounds        (RViz markers: left/right walls)

Shared by every planner/controller (yhom_*, non_*). Needs localization on the saved map
(slam:=localization): the path and the map are in the map frame.
"""

import math
import sys

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from arcsf1tenth_interfaces.msg import FrenetState, Trajectory
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid, Odometry
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from arcsf1tenth_planning.conversions import global_path_to_msg, namespaced_frame, trajectory_to_path
from arcsf1tenth_planning.frenet import FrenetPath, load_path_csv, wrap_angle

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class FrenetNode(Node):
    def __init__(self):
        super().__init__('frenet')
        self.declare_parameter('global_path', '')                   # CSV (generate_raceline.py format)
        self.declare_parameter('path_frame', namespaced_frame(self, 'map'))
        self.declare_parameter('base_frame', namespaced_frame(self, 'base_footprint'))
        self.declare_parameter('rate', 50.0)                         # [Hz] frenet/state
        self.declare_parameter('search_window', 5.0)                 # [m] projection search around the last s
        self.declare_parameter('max_width', 3.0)                     # [m] cap of the track width search
        self.declare_parameter('default_half_width', 1.0)            # [m] widths until a map arrives
        p = lambda name: self.get_parameter(name).value
        self.frame = p('path_frame')
        self.base_frame = p('base_frame')
        self.window = p('search_window')
        self.max_width = p('max_width')

        csv = p('global_path')
        if not csv:
            raise RuntimeError('set -p global_path:=<centerline or raceline CSV>')
        data = load_path_csv(csv)
        self.path = FrenetPath(data['x'], data['y'], data['v'])
        half = p('default_half_width')
        self.path.d_left[:] = half
        self.path.d_right[:] = half
        self.index = None
        self.speed = 0.0

        self.traj_pub = self.create_publisher(Trajectory, 'global_trajectory', LATCHED)
        self.path_pub = self.create_publisher(Path, 'global_path', LATCHED)
        self.bounds_pub = self.create_publisher(MarkerArray, 'track_bounds', LATCHED)
        self.state_pub = self.create_publisher(FrenetState, 'frenet/state', 10)
        self.create_subscription(OccupancyGrid, 'map', self.on_map, LATCHED)
        self.create_subscription(Odometry, 'odometry/estimated', self.on_odom, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publish_global()
        self.create_timer(1.0 / p('rate'), self.step)
        self.get_logger().info(f'global path {csv}: {len(self.path.s)} points, {self.path.length:.1f} m '
                               f'(track widths {half} m until the map arrives)')

    def on_map(self, msg):
        grid = np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
        o = msg.info.origin.position
        left, right = self.path.set_widths_from_grid(grid, msg.info.resolution, o.x, o.y, self.max_width)
        self.get_logger().info(f'track widths from the map: left {np.min(left):.2f}-{np.max(left):.2f} m, '
                               f'right {np.min(right):.2f}-{np.max(right):.2f} m')
        self.publish_global()

    def on_odom(self, msg):
        self.speed = msg.twist.twist.linear.x

    def publish_global(self):
        msg = global_path_to_msg(self.path, self.frame, self.get_clock().now().to_msg())
        self.traj_pub.publish(msg)
        self.path_pub.publish(trajectory_to_path(msg))
        self.bounds_pub.publish(self.bound_markers())

    def bound_markers(self):
        markers = MarkerArray()
        psi = self.path.psi_point()
        for i, (name, side, values) in enumerate((('left', 1.0, self.path.d_left), ('right', -1.0, self.path.d_right))):
            m = Marker()
            m.header.frame_id = self.frame
            m.ns, m.id, m.type = 'track_bounds', i, Marker.LINE_STRIP
            m.scale.x = 0.05
            m.color.r, m.color.g, m.color.b, m.color.a = 0.2, 0.6, 1.0, 0.8
            m.pose.orientation.w = 1.0
            for k in range(0, len(self.path.s), 3):
                m.points.append(Point(x=float(self.path.x[k] - side * values[k] * math.sin(psi[k])),
                                      y=float(self.path.y[k] + side * values[k] * math.cos(psi[k]))))
            m.points.append(m.points[0])
            markers.markers.append(m)
        return markers

    def step(self):
        try:
            t = self.tf_buffer.lookup_transform(self.frame, self.base_frame, Time())
        except Exception:
            return
        if self.get_clock().now() - Time.from_msg(t.header.stamp) > Duration(seconds=0.5):
            return  # stale pose: localization stopped
        x, y = t.transform.translation.x, t.transform.translation.y
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

        s, d, self.index = self.path.project(x, y, self.index, self.window if self.index is not None else None)
        if abs(d) > self.max_width + 1.0:
            self.index = None  # far off the path (lost / teleported): search the whole path next time
        kappa = float(self.path.interp(self.path.kappa, s))
        error = float(wrap_angle(yaw - self.path.heading_at(s)))
        msg = FrenetState()
        msg.header.stamp = t.header.stamp
        msg.header.frame_id = self.frame
        msg.s, msg.d = float(s), float(d)
        msg.speed = float(self.speed)
        msg.s_dot = float(self.speed * math.cos(error) / max(1.0 - kappa * d, 0.1))
        msg.d_dot = float(self.speed * math.sin(error))
        msg.yaw_error = error
        msg.path_length = self.path.length
        msg.index = int(self.index if self.index is not None else 0)
        self.state_pub.publish(msg)


def main():
    rclpy.init()
    try:
        node = FrenetNode()
    except (RuntimeError, OSError, ValueError) as error:
        print(f'[frenet] {error}', file=sys.stderr)
        rclpy.try_shutdown()
        sys.exit(1)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
