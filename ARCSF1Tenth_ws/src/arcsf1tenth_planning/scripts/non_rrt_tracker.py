#!/usr/bin/python3
"""non's path tracker: rrt/trajectory -> drive.

    rrt/trajectory (non_rrt_planner) ─┐
    TF map -> base_footprint     ─┴──► non_rrt_tracker ──► drive (AckermannDriveStamped)
                                                   ──► non_rrt_tracker/lookahead (RViz)

Each cycle:
  1. car pose (rear axle) from TF, nearest trajectory point: lateral error e (+ = car left of the
     path), heading error e_psi = yaw - psi_ref, path curvature kappa_ref
  2. steering, controller:
       rear_wheel_feedback (default; Paden et al. 2016, "rear wheel position based feedback"):
           kappa_cmd = kappa_ref cos(e_psi) / (1 - kappa_ref e) - k_e e sin(e_psi)/e_psi - k_psi e_psi
           delta = atan(wheelbase * kappa_cmd)
         Uses the path curvature as feedforward: the kinodynamic RRT path is drivable by
         construction, so the car follows it closely instead of cutting its tight corners.
         kappa_ref is read preview_time * v ahead (steering servo lag).
         Error dynamics per metre: e'' + k_psi e' + k_e e = 0, critically damped with settle
         distance l = max(settle_distance_min, v * settle_time): k_psi = 2 / l, k_e = 1 / l^2.
         (Fixed per-metre gains would be too aggressive at speed for the rate-limited steering:
         the same idea as pure pursuit's speed-dependent lookahead.)
       pure_pursuit: goal point Ld = clamp(lookahead_min + lookahead_gain * v) ahead,
           delta = atan(wheelbase * 2 y_goal / dist^2)   (cuts corners; for comparison)
  3. speed     trajectory v at the nearest point * speed_scale, changed at most max_acceleration;
               v = 0 at that point = stop (the planner's braking profile)
Stops (speed 0) on: no trajectory / empty trajectory / older than trajectory_timeout / no pose /
further than max_tracking_error from the trajectory.
"""

import math
import sys

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from arcsf1tenth_interfaces.msg import Trajectory
from rclpy.duration import Duration
from rclpy.exceptions import ParameterUninitializedException
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker

VEHICLE = ('wheelbase', 'max_steering_angle', 'max_speed', 'max_acceleration')


def namespaced_frame(node, name):
    namespace = node.get_namespace().strip('/')
    return f'{namespace}/{name}' if namespace else name


def clamp(value, low, high):
    return max(low, min(high, value))


class RRTTracker(Node):
    def __init__(self):
        super().__init__('non_rrt_tracker')
        for name in VEHICLE:
            self.declare_parameter(name, Parameter.Type.DOUBLE)
        try:
            self.vehicle = {name: self.get_parameter(name).value for name in VEHICLE}
        except ParameterUninitializedException:
            raise RuntimeError('vehicle parameters missing: start it with non_controller.launch.py')
        declare = self.declare_parameter
        declare('trajectory_topic', 'rrt/trajectory')
        declare('base_frame', namespaced_frame(self, 'base_footprint'))
        declare('rate', 50.0)                  # [Hz]
        declare('trajectory_timeout', 0.5)     # [s]
        declare('controller', 'rear_wheel_feedback')   # rear_wheel_feedback | pure_pursuit
        declare('settle_time', 0.5)            # [s] rear_wheel_feedback: errors decay over v * this ...
        declare('settle_distance_min', 0.6)    # [m] ... but at least this distance
        declare('preview_time', 0.1)           # [s] curvature feedforward read this far ahead
        declare('lookahead_min', 0.6)          # [m]
        declare('lookahead_max', 1.8)          # [m]
        declare('lookahead_gain', 0.35)        # [s]
        declare('speed_scale', 1.0)            # fraction of the trajectory speed
        declare('speed_min', 0.3)              # [m/s] unless the trajectory says stop
        declare('max_tracking_error', 1.0)     # [m]
        p = lambda name: self.get_parameter(name).value
        self.p = p
        self.base_frame = p('base_frame')
        self.dt = 1.0 / p('rate')
        self.controller = p('controller')
        if self.controller not in ('rear_wheel_feedback', 'pure_pursuit'):
            raise RuntimeError(f"controller must be rear_wheel_feedback or pure_pursuit, got '{self.controller}'")

        self.traj = None              # dict: xy (N, 2), psi, kappa, v, s (N,), frame
        self.traj_time = None
        self.speed_cmd = 0.0
        self.status = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, 'drive', 10)
        self.goal_pub = self.create_publisher(Marker, '~/lookahead', 10)
        self.create_subscription(Trajectory, p('trajectory_topic'), self.on_trajectory, 10)
        self.create_timer(self.dt, self.step)

    def on_trajectory(self, msg):
        self.traj_time = self.get_clock().now()
        if len(msg.points) < 2:
            self.traj = None
            return
        xy = np.array([(pt.x, pt.y) for pt in msg.points])
        s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))])
        self.traj = dict(xy=xy, s=s, frame=msg.header.frame_id,
                         psi=np.array([pt.psi for pt in msg.points]),
                         kappa=np.array([pt.kappa for pt in msg.points]),
                         v=np.array([pt.v for pt in msg.points]))

    def set_status(self, status):
        if status != self.status:
            if status == 'tracking':
                self.get_logger().info(status)
            else:
                self.get_logger().warn(f'{status}: stopping')
            self.status = status

    def stop(self, status):
        self.set_status(status)
        self.speed_cmd = 0.0
        self.publish(0.0, 0.0)

    def step(self):
        now = self.get_clock().now()
        if self.traj is None or self.traj_time is None or \
                now - self.traj_time > Duration(seconds=self.p('trajectory_timeout')):
            return self.stop('no valid trajectory')
        tr = self.traj
        xy, v_ref, s, frame = tr['xy'], tr['v'], tr['s'], tr['frame']
        try:
            t = self.tf_buffer.lookup_transform(frame, self.base_frame, Time())
        except TransformException:
            return self.stop('no pose')
        if now - Time.from_msg(t.header.stamp) > Duration(seconds=0.5):
            return self.stop('stale pose')
        x, y = t.transform.translation.x, t.transform.translation.y
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

        dist = np.hypot(xy[:, 0] - x, xy[:, 1] - y)
        i = int(np.argmin(dist))
        if dist[i] > self.p('max_tracking_error'):
            return self.stop(f'{dist[i]:.2f} m off the trajectory')
        self.set_status('tracking')

        if self.controller == 'rear_wheel_feedback':
            curvature, (gx, gy) = self.rear_wheel_feedback(tr, i, x, y, yaw)
        else:
            curvature, (gx, gy) = self.pure_pursuit(tr, i, x, y, yaw)
        max_steer = self.vehicle['max_steering_angle']
        steering = clamp(math.atan(self.vehicle['wheelbase'] * curvature), -max_steer, max_steer)

        if v_ref[i] <= 0.05:
            target_speed = 0.0
        else:
            target_speed = clamp(v_ref[i] * self.p('speed_scale'), self.p('speed_min'), self.vehicle['max_speed'])
        step = self.vehicle['max_acceleration'] * self.dt
        self.speed_cmd = clamp(target_speed, self.speed_cmd - step, self.speed_cmd + step)
        self.publish(self.speed_cmd, steering)
        self.publish_goal(frame, gx, gy)

    def rear_wheel_feedback(self, tr, i, x, y, yaw):
        psi = tr['psi'][i]
        e = -math.sin(psi) * (x - tr['xy'][i, 0]) + math.cos(psi) * (y - tr['xy'][i, 1])
        e_psi = math.atan2(math.sin(yaw - psi), math.cos(yaw - psi))
        preview = tr['s'][i] + self.p('preview_time') * self.speed_cmd
        j = min(int(np.searchsorted(tr['s'], preview)), len(tr['s']) - 1)
        kappa = tr['kappa'][j]
        sinc = math.sin(e_psi) / e_psi if abs(e_psi) > 1e-6 else 1.0
        settle = max(self.p('settle_distance_min'), self.p('settle_time') * self.speed_cmd)
        k_psi, k_e = 2.0 / settle, 1.0 / settle ** 2
        curvature = kappa * math.cos(e_psi) / max(1.0 - kappa * e, 0.1) - k_e * e * sinc - k_psi * e_psi
        return curvature, tr['xy'][j]

    def pure_pursuit(self, tr, i, x, y, yaw):
        xy, s = tr['xy'], tr['s']
        ld = clamp(self.p('lookahead_min') + self.p('lookahead_gain') * self.speed_cmd,
                   self.p('lookahead_min'), self.p('lookahead_max'))
        target = min(s[i] + ld, s[-1])
        j = min(int(np.searchsorted(s, target)), len(s) - 1)
        if j > 0 and s[j] > s[j - 1]:
            f = (target - s[j - 1]) / (s[j] - s[j - 1])
            gx, gy = xy[j - 1] + f * (xy[j] - xy[j - 1])
        else:
            gx, gy = xy[j]
        dx, dy = gx - x, gy - y
        local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
        d2 = dx * dx + dy * dy
        return (2.0 * local_y / d2 if d2 > 1e-6 else 0.0), (gx, gy)

    def publish(self, speed, steering):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steering)
        self.drive_pub.publish(msg)

    def publish_goal(self, frame, x, y):
        m = Marker()
        m.header.frame_id = frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.type = 'rrt_lookahead', Marker.SPHERE
        m.pose.position.x, m.pose.position.y = float(x), float(y)
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.15
        m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 0.6, 1.0, 1.0
        self.goal_pub.publish(m)


def main():
    rclpy.init()
    try:
        node = RRTTracker()
    except RuntimeError as error:
        print(f'[non_rrt_tracker] {error}', file=sys.stderr)
        rclpy.try_shutdown()
        sys.exit(1)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.publish(0.0, 0.0)     # leave the car stopped
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
