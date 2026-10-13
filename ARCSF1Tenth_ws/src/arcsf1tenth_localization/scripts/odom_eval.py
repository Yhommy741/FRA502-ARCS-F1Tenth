#!/usr/bin/python3

import math

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64


class Track:
    """Pose error of one odometry source against ground truth, both relative to their first pose."""

    def __init__(self):
        self.origin = None          # (x, y, yaw) of the source when evaluation started
        self.truth_origin = None    # ground truth (world frame) at that moment
        self.pos_sq = []
        self.yaw_sq = []
        self.last = (0.0, 0.0)      # latest (position error, yaw error)

    def add(self, pose, truth):
        if self.origin is None:
            self.origin, self.truth_origin = pose, truth
        x, y, yaw = relative(self.origin, pose)
        tx, ty, tyaw = relative(self.truth_origin, truth)
        pos_err = math.hypot(x - tx, y - ty)
        yaw_err = wrap(yaw - tyaw)
        self.pos_sq.append(pos_err ** 2)
        self.yaw_sq.append(yaw_err ** 2)
        self.last = (pos_err, yaw_err)


class Rate:
    """Error of a yaw rate (or speed) source against ground truth."""

    def __init__(self):
        self.errors = []

    def add(self, value, truth):
        self.errors.append(value - truth)

    def summary(self):
        if not self.errors:
            return '      -'
        e = np.array(self.errors)
        return f'mean {e.mean():+.4f}  rms {math.sqrt((e ** 2).mean()):.4f}'


class OdomEval(Node):
    """Score odometry/measured and odometry/estimated against Gazebo ground truth.

    Both are compared as motion relative to where evaluation started, so it
    can be started at any time. Prints the running error every 'print_period'
    seconds and a summary on Ctrl-C:
      - position / yaw error and drift per metre driven (the end result)
      - yaw rate error of each source (gyro, bicycle model, EKF) and speed error:
        these show WHICH input is wrong, which is what tuning needs
      - EKF gyro bias estimate vs the actual gyro bias (gyro - true yaw rate)
    """

    def __init__(self):
        super().__init__('odom_eval')
        self.declare_parameter('print_period', 2.0)  # [s]

        self.truth = None           # latest (x, y, yaw) in the world frame
        self.truth_twist = None     # latest (speed, yaw rate)
        self.distance = 0.0
        self.prev_truth_xy = None

        self.tracks = {'measured': Track(), 'estimated': Track()}
        self.rates = {'gyro': Rate(), 'bicycle': Rate(), 'ekf': Rate()}
        self.speeds = {'wheel': Rate(), 'ekf': Rate()}
        self.gyro_bias_samples = []  # gyro - true yaw rate
        self.bias_estimate = None

        self.create_subscription(Odometry, 'ground_truth/odometry', self.truth_callback, 50)
        self.create_subscription(Odometry, 'odometry/measured', lambda m: self.odom_callback('measured', m), 50)
        self.create_subscription(Odometry, 'odometry/estimated', lambda m: self.odom_callback('estimated', m), 50)
        self.create_subscription(Imu, 'imu', self.imu_callback, 100)
        self.create_subscription(Float64, 'ekf/gyro_bias', self.bias_callback, 10)
        self.create_timer(self.get_parameter('print_period').value, self.print_status)
        self.get_logger().info('Comparing odometry/measured and odometry/estimated with ground_truth/odometry')

    def truth_callback(self, msg):
        self.truth = pose_of(msg)
        # Gazebo's odometry twist is in the body frame
        self.truth_twist = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)
        if self.prev_truth_xy is not None:
            self.distance += math.hypot(self.truth[0] - self.prev_truth_xy[0], self.truth[1] - self.prev_truth_xy[1])
        self.prev_truth_xy = self.truth[:2]

    def odom_callback(self, name, msg):
        if self.truth is None:
            return
        self.tracks[name].add(pose_of(msg), self.truth)
        speed, yaw_rate = msg.twist.twist.linear.x, msg.twist.twist.angular.z
        true_speed, true_rate = self.truth_twist
        if name == 'measured':
            self.rates['bicycle'].add(yaw_rate, true_rate)
            self.speeds['wheel'].add(speed, true_speed)
        else:
            self.rates['ekf'].add(yaw_rate, true_rate)
            self.speeds['ekf'].add(speed, true_speed)

    def imu_callback(self, msg):
        # IMU is mounted level and facing forward, so gyro z = yaw rate
        if self.truth_twist is None:
            return
        self.rates['gyro'].add(msg.angular_velocity.z, self.truth_twist[1])
        self.gyro_bias_samples.append(msg.angular_velocity.z - self.truth_twist[1])
        self.gyro_bias_samples = self.gyro_bias_samples[-500:]  # last ~5 s

    def bias_callback(self, msg):
        self.bias_estimate = msg.data

    def print_status(self):
        if self.truth is None:
            self.get_logger().info('Waiting for ground_truth/odometry ...', throttle_duration_sec=5.0)
            return
        parts = [f'dist {self.distance:6.1f} m']
        for name, track in self.tracks.items():
            pos, yaw = track.last
            parts.append(f'{name}: {pos:5.2f} m {math.degrees(yaw):+6.1f} deg')
        if self.bias_estimate is not None and self.gyro_bias_samples:
            parts.append(f'bias est {self.bias_estimate:+.4f} / true {np.mean(self.gyro_bias_samples):+.4f} rad/s')
        self.get_logger().info(' | '.join(parts))

    def summary(self):
        lines = ['', f'==== Odometry evaluation over {self.distance:.1f} m ====']
        lines.append('Pose error           final pos   final yaw   rms pos   rms yaw   drift')
        for name, t in self.tracks.items():
            if not t.pos_sq:
                lines.append(f'  {name:18} no data')
                continue
            drift = 100.0 * t.last[0] / self.distance if self.distance > 1.0 else float('nan')
            lines.append(
                f'  {name:18} {t.last[0]:7.2f} m  {math.degrees(t.last[1]):+7.1f} deg'
                f'  {math.sqrt(np.mean(t.pos_sq)):6.2f} m  {math.degrees(math.sqrt(np.mean(t.yaw_sq))):6.1f} deg'
                f'  {drift:5.1f} %'
            )
        lines.append('Yaw rate error [rad/s] (value - truth)')
        for name, r in self.rates.items():
            lines.append(f'  {name:18} {r.summary()}')
        lines.append('Speed error [m/s] (value - truth)')
        for name, r in self.speeds.items():
            lines.append(f'  {name:18} {r.summary()}')
        if self.bias_estimate is not None and self.gyro_bias_samples:
            lines.append(
                f'Gyro bias [rad/s]: EKF {self.bias_estimate:+.4f}, actual (last 5 s) {np.mean(self.gyro_bias_samples):+.4f}')
        print('\n'.join(lines))


def pose_of(msg):
    q = msg.pose.pose.orientation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)


def relative(origin, pose):
    """Pose expressed in the frame of origin (both (x, y, yaw))."""
    dx, dy = pose[0] - origin[0], pose[1] - origin[1]
    c, s = math.cos(origin[2]), math.sin(origin[2])
    return (c * dx + s * dy, -s * dx + c * dy, wrap(pose[2] - origin[2]))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def main(args=None):
    rclpy.init(args=args)
    node = OdomEval()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.summary()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
