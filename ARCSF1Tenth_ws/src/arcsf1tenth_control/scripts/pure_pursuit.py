#!/usr/bin/python3

import math
import sys

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker


VEHICLE_PARAMS = ('wheelbase', 'max_steering_angle', 'max_speed')


def namespaced_frame(node, name):
    """TF frame in the node's namespace: /arcsf1tenth + map -> arcsf1tenth/map (no namespace -> map)."""
    namespace = node.get_namespace().strip('/')
    return f'{namespace}/{name}' if namespace else name


class PurePursuit(Node):
    """Pure pursuit raceline follower: TF pose + raceline CSV -> drive.

    Each cycle:
      1. car pose (rear axle = base_footprint) in the raceline frame from TF
      2. nearest raceline point (searched near the previous one)
      3. lookahead distance  Ld = clamp(lookahead_min + lookahead_gain * v, lookahead_min, lookahead_max)
      4. goal point: Ld further along the raceline (by arc length)
      5. steering  delta = atan(2 * wheelbase * y_goal / d^2)
         (y_goal = goal sideways in the car frame, d = distance to the goal)
      6. speed = raceline vx at the nearest point * speed_scale

    Raceline CSV: TUM format 's_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2'
    (generate_raceline.py output, or the original F1TENTH raceline.csv).
    """

    def __init__(self):
        super().__init__('pure_pursuit')

        # Vehicle limits: no defaults, must come from arcsf1tenth_description/config/vehicle.yaml
        for name in VEHICLE_PARAMS:
            self.declare_parameter(name, Parameter.Type.DOUBLE)
        missing = []
        for name in VEHICLE_PARAMS:
            try:
                self.get_parameter(name)
            except ParameterUninitializedException:
                missing.append(name)
        if missing:
            raise RuntimeError(
                f'Missing vehicle parameters {missing}. Start it with: '
                'ros2 launch arcsf1tenth_bringup pure_pursuit.launch.py track:=<map_name>'
            )

        self.declare_parameter('raceline', '')               # raceline CSV path
        self.declare_parameter('raceline_frame', namespaced_frame(self, 'map'))  # frame of the CSV x/y
        self.declare_parameter('base_frame', namespaced_frame(self, 'base_footprint'))
        self.declare_parameter('rate', 50.0)                 # [Hz]
        self.declare_parameter('lookahead_min', 0.6)         # [m]
        self.declare_parameter('lookahead_max', 2.0)         # [m]
        self.declare_parameter('lookahead_gain', 0.35)       # [s] extra lookahead per m/s of speed
        self.declare_parameter('speed_scale', 0.6)           # fraction of the raceline speed
        self.declare_parameter('speed_max', 3.0)             # [m/s] (capped at vehicle max_speed)
        self.declare_parameter('speed_min', 0.5)             # [m/s]
        self.declare_parameter('max_tracking_error', 1.0)    # [m] stop when further than this from the raceline
        self.declare_parameter('search_window', 5.0)         # [m] nearest-point search around the previous one

        p = lambda name: self.get_parameter(name).value
        self.raceline_frame = p('raceline_frame')
        self.base_frame = p('base_frame')
        self.wheelbase = p('wheelbase')
        self.max_steer = p('max_steering_angle')
        self.ld_min = p('lookahead_min')
        self.ld_max = p('lookahead_max')
        self.ld_gain = p('lookahead_gain')
        self.speed_scale = p('speed_scale')
        self.speed_max = min(p('speed_max'), p('max_speed'))
        self.speed_min = p('speed_min')
        self.max_error = p('max_tracking_error')

        path = p('raceline')
        if not path:
            raise RuntimeError('No raceline: set -p raceline:=<file.csv> or use pure_pursuit.launch.py track:=<map_name>')
        data = np.loadtxt(path, delimiter=';', comments='#', ndmin=2)
        if data.shape[1] < 6:
            raise RuntimeError(f'{path}: expected columns s; x; y; psi; kappa; vx; ...')
        self.xy = data[:, 1:3]
        self.ref_speed = data[:, 5]
        # Arc length from the points themselves (closed loop)
        seg = np.linalg.norm(np.diff(np.vstack([self.xy, self.xy[:1]]), axis=0), axis=1)
        self.s = np.concatenate([[0.0], np.cumsum(seg)[:-1]])
        self.length = seg.sum()
        spacing = self.length / len(self.xy)
        self.window = max(int(p('search_window') / spacing), 5)

        self.nearest = None         # index of the nearest point last cycle
        self.speed_cmd = 0.0
        self.state = 'start'

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, 'drive', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, '~/raceline', latched)
        self.goal_pub = self.create_publisher(Marker, '~/lookahead', 10)
        self.publish_path()

        self.create_timer(1.0 / p('rate'), self.step)
        self.get_logger().info(
            f'Following {path} ({len(self.xy)} points, {self.length:.1f} m) in frame {self.raceline_frame}, '
            f'speed x{self.speed_scale:.2f} (max {self.speed_max:.1f} m/s)'
        )

    # ------------------------------------------------------------------ control

    def step(self):
        pose = self.car_pose()
        if pose is None:
            self.set_state('no_pose', f'Waiting for TF {self.raceline_frame} -> {self.base_frame}')
            self.publish(0.0, 0.0)
            return
        x, y, yaw = pose

        nearest = self.find_nearest(x, y)
        error = float(np.hypot(*(self.xy[nearest] - (x, y))))
        if error > self.max_error:
            # Lost (bad localization or crashed): search the whole loop again next time
            self.nearest = None
            self.set_state('off_track', f'{error:.2f} m from the raceline (> {self.max_error:.2f} m): stopping')
            self.publish(0.0, 0.0)
            return
        self.nearest = nearest
        self.set_state('tracking', 'Tracking the raceline')

        lookahead = clamp(self.ld_min + self.ld_gain * abs(self.speed_cmd), self.ld_min, self.ld_max)
        gx, gy = self.point_ahead(nearest, lookahead)

        # Goal in the car frame (x forward, y left)
        dx, dy = gx - x, gy - y
        c, s = math.cos(yaw), math.sin(yaw)
        local_x = c * dx + s * dy
        local_y = -s * dx + c * dy
        d2 = local_x ** 2 + local_y ** 2
        curvature = 2.0 * local_y / d2 if d2 > 1e-6 else 0.0
        steering = clamp(math.atan(self.wheelbase * curvature), -self.max_steer, self.max_steer)

        speed = clamp(self.ref_speed[nearest] * self.speed_scale, self.speed_min, self.speed_max)
        self.speed_cmd = speed
        self.publish(speed, steering)
        self.publish_goal(gx, gy)

    def car_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(self.raceline_frame, self.base_frame, Time())
        except Exception:
            return None
        # Stale pose = localization stopped: don't drive on it
        age = self.get_clock().now() - Time.from_msg(t.header.stamp)
        if age > Duration(seconds=0.5):
            return None
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

    def find_nearest(self, x, y):
        """Nearest raceline index; only near the previous one so crossings/close sections don't jump."""
        if self.nearest is None:
            candidates = np.arange(len(self.xy))
        else:
            candidates = np.arange(self.nearest - self.window, self.nearest + self.window + 1) % len(self.xy)
        d2 = np.sum((self.xy[candidates] - (x, y)) ** 2, axis=1)
        return int(candidates[np.argmin(d2)])

    def point_ahead(self, index, distance):
        """Point 'distance' metres further along the raceline from point index (interpolated)."""
        target = (self.s[index] + distance) % self.length
        i = int(np.searchsorted(self.s, target, side='right')) - 1
        j = (i + 1) % len(self.xy)
        seg = (self.s[j] if j else self.length) - self.s[i]
        t = (target - self.s[i]) / seg if seg > 1e-9 else 0.0
        return self.xy[i] + t * (self.xy[j] - self.xy[i])

    def set_state(self, state, message):
        if state != self.state:
            # Separate calls: rclpy fixes the severity of each logging call site
            if state == 'tracking':
                self.get_logger().info(message)
            else:
                self.get_logger().warn(message)
            self.state = state

    # ------------------------------------------------------------------ output

    def publish(self, speed, steering):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steering)
        self.drive_pub.publish(msg)

    def publish_path(self):
        path = Path()
        path.header.frame_id = self.raceline_frame
        path.header.stamp = self.get_clock().now().to_msg()
        for x, y in self.xy:
            pose = PoseStamped()
            pose.header.frame_id = self.raceline_frame
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.path_pub.publish(path)

    def publish_goal(self, x, y):
        marker = Marker()
        marker.header.frame_id = self.raceline_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'lookahead'
        marker.type = Marker.SPHERE
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.2
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.5, 0.0, 1.0
        self.goal_pub.publish(marker)

    def stop(self):
        self.publish(0.0, 0.0)


def clamp(value, low, high):
    return max(low, min(high, value))


def main(args=None):
    rclpy.init(args=args)
    try:
        node = PurePursuit()
    except (RuntimeError, OSError) as error:
        print(f'[pure_pursuit] {error}', file=sys.stderr)
        rclpy.try_shutdown()
        sys.exit(1)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Leave the car stopped (the context may already be shut down by Ctrl-C)
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
