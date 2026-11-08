#!/usr/bin/python3

import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from arcsf1tenth_interfaces.msg import FrenetState, Obstacle, ObstacleArray, Trajectory
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from scipy import ndimage
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from arcsf1tenth_planning.utils.conversions import msg_to_global_path, namespaced_frame

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class ObstacleDetector(Node):
    # Set up the node: detection parameters, map/path state, subscribers and obstacle publishers.
    def __init__(self):
        super().__init__('obstacle_detector')

        # Parameters
        self.declare_parameter('map_frame', namespaced_frame(self, 'map'))
        self.declare_parameter('wall_margin', 0.3)        # [m] points closer to a map wall = wall
        self.declare_parameter('max_range', 10.0)         # [m] ignore points further from the lidar
        self.declare_parameter('cluster_gap', 0.3)        # [m] max distance between neighbouring points of one obstacle
        self.declare_parameter('min_points', 3)           # points per obstacle (fewer = noise)
        self.declare_parameter('max_size', 2.0)           # [m] bigger clusters = mislocalized wall, ignored
        self.declare_parameter('track_margin', 0.1)       # [m] keep obstacles reaching this far into the track
        p = lambda name: self.get_parameter(name).value
        self.map_frame = p('map_frame')

        # Variables
        self.dist = None            # distance to the nearest wall per map cell [m]
        self.map_info = None
        self.path = None
        self.car_index = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers
        self.pub = self.create_publisher(ObstacleArray, 'obstacles', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'obstacles/markers', 10)

        # Subscribers
        self.create_subscription(OccupancyGrid, 'map', self.on_map, LATCHED)
        self.create_subscription(Trajectory, 'global_trajectory', self.on_path, LATCHED)
        self.create_subscription(FrenetState, 'frenet/state', self.on_state, 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)

        # Service Servers

        # Service Clients

        # Action Servers

        # Action Clients

        # Timers

        # On Start

    # Compute the distance to the nearest wall for every map cell (once per map).
    def on_map(self, msg):
        grid = np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
        wall = (grid >= 65) | (grid < 0)       # occupied or unknown
        self.dist = ndimage.distance_transform_edt(~wall) * msg.info.resolution
        self.map_info = msg.info
        self.get_logger().info(f'map {msg.info.width} x {msg.info.height}: wall distance ready')

    # Store the global path (Frenet frame and track widths).
    def on_path(self, msg):
        if msg.points:
            self.path = msg_to_global_path(msg)

    # Store the car's path index as a hint for the projection search.
    def on_state(self, msg):
        self.car_index = msg.index

    # Turn the scan into obstacles: drop map walls, cluster the rest, convert to Frenet, and publish.
    def on_scan(self, scan):
        if self.dist is None or self.path is None:
            return
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, scan.header.frame_id, Time())
        except TransformException as error:
            self.get_logger().warn(f'{error}', throttle_duration_sec=5.0)
            return
        p = lambda name: self.get_parameter(name).value

        ranges = np.asarray(scan.ranges)
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        valid = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= min(scan.range_max, p('max_range')))
        q = tf.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        lx, ly = ranges * np.cos(angles), ranges * np.sin(angles)
        x = tf.transform.translation.x + math.cos(yaw) * lx - math.sin(yaw) * ly
        y = tf.transform.translation.y + math.sin(yaw) * lx + math.cos(yaw) * ly

        # not a wall of the map: inside the map and further than wall_margin from any wall
        info = self.map_info
        col = np.floor((x - info.origin.position.x) / info.resolution).astype(int)
        row = np.floor((y - info.origin.position.y) / info.resolution).astype(int)
        inside = valid & (col >= 0) & (col < info.width) & (row >= 0) & (row < info.height)
        free = np.zeros_like(valid)
        free[inside] = self.dist[row[inside], col[inside]] > p('wall_margin')

        obstacles = []
        for cluster in self.clusters(free, x, y, p('cluster_gap')):
            if len(cluster) < p('min_points'):
                continue
            obstacle = self.to_frenet(x[cluster], y[cluster], p('max_size'), p('track_margin'))
            if obstacle is not None:
                obstacle.id = len(obstacles)
                obstacles.append(obstacle)

        msg = ObstacleArray()
        msg.header.stamp = scan.header.stamp
        msg.header.frame_id = self.map_frame
        msg.obstacles = obstacles
        self.pub.publish(msg)
        self.publish_markers(msg)

    # Group consecutive beams (scan order) whose points are closer than gap, wrapping around 360 deg.
    @staticmethod
    def clusters(mask, x, y, gap):
        idx = np.flatnonzero(mask)
        if len(idx) == 0:
            return []
        groups = [[idx[0]]]
        for a, b in zip(idx[:-1], idx[1:]):
            if math.hypot(x[b] - x[a], y[b] - y[a]) < gap:
                groups[-1].append(b)
            else:
                groups.append([b])
        if len(groups) > 1 and math.hypot(x[idx[0]] - x[idx[-1]], y[idx[0]] - y[idx[-1]]) < gap:
            groups[0] = groups.pop() + groups[0]   # first and last beam are neighbours (360 deg scan)
        return [np.array(g) for g in groups]

    # Convert one point cluster to an Obstacle with its Frenet extent, or None if too big or outside the track.
    def to_frenet(self, px, py, max_size, track_margin):
        cx, cy = float(np.mean(px)), float(np.mean(py))
        radius = float(np.max(np.hypot(px - cx, py - cy)))
        if 2 * radius > max_size:
            return None
        path = self.path
        window = 20.0 if self.car_index is not None else None
        s_c, d_c, i = path.project(cx, cy, self.car_index, window)
        sd = np.array([path.project(float(a), float(b), i, 3.0)[:2] for a, b in zip(px, py)])
        rel = path.ds(sd[:, 0], s_c)
        d_left = float(path.interp(path.d_left, s_c))
        d_right = float(path.interp(path.d_right, s_c))
        d_min, d_max = float(np.min(sd[:, 1])), float(np.max(sd[:, 1]))
        if d_min > d_left - track_margin or d_max < -d_right + track_margin:
            return None      # outside the track (beyond a wall)
        return Obstacle(x=cx, y=cy, radius=radius, s_center=float(s_c),
                        s_start=float(path.wrap_s(s_c + np.min(rel))), s_end=float(path.wrap_s(s_c + np.max(rel))),
                        d_center=float(d_c), d_min=d_min, d_max=d_max)

    # Publish the obstacles as RViz cylinder markers.
    def publish_markers(self, msg):
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        for o in msg.obstacles:
            m = Marker()
            m.header = msg.header
            m.ns, m.id, m.type = 'obstacles', o.id, Marker.CYLINDER
            m.pose.position.x, m.pose.position.y, m.pose.position.z = o.x, o.y, 0.15
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = max(2 * o.radius, 0.1)
            m.scale.z = 0.3
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.1, 0.1, 0.8
            markers.markers.append(m)
        self.marker_pub.publish(markers)


# Start the node and spin until Ctrl+C.
def main():
    rclpy.init()
    node = ObstacleDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
