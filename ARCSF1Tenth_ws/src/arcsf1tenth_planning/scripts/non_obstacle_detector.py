#!/usr/bin/python3

import math
import sys

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener

from arcsf1tenth_planning.rrt.global_path import GlobalPath
from arcsf1tenth_planning.rrt.obstacles import ObstacleDetector, scan_to_map
from arcsf1tenth_planning.utils.conversions import namespaced_frame

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


# Return the yaw angle of a quaternion.
def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class NonObstacleDetector(Node):
    # Set up the node: detection parameters, the global path (for the track area), subscribers and publisher.
    def __init__(self):
        super().__init__('non_obstacle_detector')

        # Parameters
        declare = self.declare_parameter
        declare('global_path', '')                 # CSV (launch file): the track area is the free space around it
        declare('map_frame', namespaced_frame(self, 'map'))
        declare('max_range', 6.0)                  # [m]
        declare('detect_wall_margin', 0.3)         # [m] points closer than this to a map wall = wall ...
        declare('detect_range_gain', 0.035)        # [m/m] ... plus this * range (0 = fixed margin, like obstacle_detector)
        declare('cluster_gap', 0.3)                # [m] max distance between neighbouring points of one obstacle
        declare('min_points', 3)                   # points per obstacle (fewer = noise)
        declare('max_size', 2.0)                   # [m] wider clusters = mislocalized wall, ignored
        declare('track_filter', True)              # drop clusters outside the track (seen through a gap)
        declare('track_margin', 0.1)               # [m] keep clusters reaching this far into the track
        p = lambda name: self.get_parameter(name).value
        self.p = p
        csv = p('global_path')
        if not csv:
            raise RuntimeError('no global_path: start it with non_controller.launch.py track:=<map_name>')
        self.map_frame = p('map_frame')

        # Variables
        self.path = GlobalPath.from_csv(csv)
        self.detector = ObstacleDetector(p('detect_wall_margin'), p('detect_range_gain'), p('cluster_gap'),
                                         p('min_points'), p('max_size'), p('track_filter'), p('track_margin'))
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers
        self.points_pub = self.create_publisher(point_cloud2.PointCloud2, 'rrt/obstacle_points', 10)

        # Subscribers
        self.create_subscription(OccupancyGrid, 'map', self.on_map, LATCHED)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)

        # Service Servers

        # Service Clients

        # Action Servers

        # Action Clients

        # Timers

        # On Start
        self.get_logger().info('waiting for the map')

    # Compute the wall distance field and the track widths once per map.
    def on_map(self, msg):
        grid = np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
        self.detector.set_map(grid, msg.info.resolution, msg.info.origin.position.x, msg.info.origin.position.y,
                              self.path.x, self.path.y, self.path.psi)
        self.get_logger().info(f'map {msg.info.width} x {msg.info.height}: wall distance and track widths ready')

    # Turn one scan into obstacle points and publish them (stamp = scan time).
    def on_scan(self, scan):
        if not self.detector.ready:
            return
        # pose AT THE SCAN TIME: in a corner the car turns ~2 rad/s, so the latest pose (even
        # 50 ms newer) rotates the scan by several degrees and far walls look like obstacles
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, scan.header.frame_id,
                                                 Time.from_msg(scan.header.stamp))
        except TransformException:
            try:     # TF not there yet for that stamp (localization lags): latest, warn
                tf = self.tf_buffer.lookup_transform(self.map_frame, scan.header.frame_id, Time())
                self.get_logger().warn('no TF at the scan time: using the latest pose for obstacle detection',
                                       throttle_duration_sec=5.0)
            except TransformException as error:
                self.get_logger().warn(f'{error}', throttle_duration_sec=5.0)
                return
        t = tf.transform
        pose = (t.translation.x, t.translation.y, yaw_of(t.rotation))
        x, y, r, valid = scan_to_map(scan.ranges, scan.angle_min, scan.angle_increment,
                                     scan.range_min, scan.range_max, self.p('max_range'), pose)
        pts = self.detector.detect(x, y, r, valid)
        header = Header(stamp=scan.header.stamp, frame_id=self.map_frame)
        xyz = np.column_stack([pts, np.full(len(pts), 0.1)]).astype(np.float32)
        self.points_pub.publish(point_cloud2.create_cloud_xyz32(header, xyz))


# Start the node and spin until Ctrl+C.
def main():
    rclpy.init()
    try:
        node = NonObstacleDetector()
    except (RuntimeError, OSError) as error:
        print(f'[non_obstacle_detector] {error}', file=sys.stderr)
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
