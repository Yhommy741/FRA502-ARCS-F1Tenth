#!/usr/bin/python3

import math
import sys

import rclpy
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node
from rclpy.parameter import Parameter
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan


VEHICLE_PARAMS = ('max_steering_angle', 'max_speed')


class WallFollow(Node):
    """PID wall follower: scan -> drive.

    Keeps a fixed distance to the left or right wall using two lidar beams
    (F1TENTH wall-following method):

        b = range at 90 deg (perpendicular to the car)
        a = range at 90 deg - theta (toward the front)
        alpha = atan((a cos(theta) - b) / (a sin(theta)))   car heading vs wall
        D     = b cos(alpha)                                distance to the wall now
        D_ahead = D + lookahead * sin(alpha)                distance after driving 'lookahead' m

    error = D_ahead - desired_distance (> 0: too far from the wall) goes
    through a PID to get the steering angle. Speed drops with the steering
    angle and with the free distance in front of the car.

    Lidar angle 0 = forward, +90 deg = left (lidar frame aligned with base_link).
    """

    def __init__(self):
        super().__init__('wall_follow')

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
                'ros2 launch arcsf1tenth_bringup wall_follow.launch.py '
                '(or add --ros-args --params-file $(ros2 pkg prefix arcsf1tenth_description)'
                '/share/arcsf1tenth_description/config/vehicle.yaml)'
            )

        self.declare_parameter('side', 'left')              # wall to follow: left | right
        self.declare_parameter('desired_distance', 0.8)     # [m] distance to keep from the wall
        self.declare_parameter('lookahead', 0.6)            # [m] predict the distance this far ahead
        self.declare_parameter('beam_angle', 0.7854)        # [rad] theta between beams a and b (45 deg)
        self.declare_parameter('beam_window', 0.035)        # [rad] median over +-window around each beam
        self.declare_parameter('kp', 1.0)                   # [rad/m]
        self.declare_parameter('ki', 0.0)                   # [rad/(m s)]
        self.declare_parameter('kd', 0.15)                  # [rad s/m]
        self.declare_parameter('integral_limit', 0.5)       # [m s] anti-windup clamp on the error integral
        self.declare_parameter('speed_max', 1.5)            # [m/s] on a straight with free space ahead
        self.declare_parameter('speed_min', 0.5)            # [m/s] at full steering / close obstacle
        self.declare_parameter('front_slow_distance', 3.0)  # [m] start slowing below this free distance
        self.declare_parameter('front_stop_distance', 0.4)  # [m] stop if something is this close ahead
        self.declare_parameter('front_half_angle', 0.17)    # [rad] cone in front used for the free distance

        side = self.get_parameter('side').value.lower()
        if side not in ('left', 'right'):
            raise RuntimeError(f"side must be left or right, got '{side}'")
        self.side = 1.0 if side == 'left' else -1.0
        self.desired = self.get_parameter('desired_distance').value
        self.lookahead = self.get_parameter('lookahead').value
        self.theta = self.get_parameter('beam_angle').value
        self.window = self.get_parameter('beam_window').value
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.integral_limit = self.get_parameter('integral_limit').value
        self.speed_max = min(self.get_parameter('speed_max').value, self.get_parameter('max_speed').value)
        self.speed_min = self.get_parameter('speed_min').value
        self.front_slow = self.get_parameter('front_slow_distance').value
        self.front_stop = self.get_parameter('front_stop_distance').value
        self.front_half_angle = self.get_parameter('front_half_angle').value
        self.max_steer = self.get_parameter('max_steering_angle').value

        self.drive_pub = self.create_publisher(AckermannDriveStamped, 'drive', 10)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)

        self.prev_error = None
        self.prev_time = None
        self.integral = 0.0

        self.get_logger().info(
            f'Following the {side} wall at {self.desired:.2f} m (speed {self.speed_min:.1f}-{self.speed_max:.1f} m/s)'
        )

    def scan_callback(self, scan):
        beam_b = self.side * math.pi / 2.0                  # perpendicular to the car
        beam_a = self.side * (math.pi / 2.0 - self.theta)   # toward the front
        a = beam_range(scan, beam_a, self.window)
        b = beam_range(scan, beam_b, self.window)
        if a is None or b is None:
            # No wall in range on that side: go straight slowly and wait for it
            self.reset_pid()
            self.publish(self.speed_min, 0.0)
            return

        alpha = math.atan2(a * math.cos(self.theta) - b, a * math.sin(self.theta))
        distance = b * math.cos(alpha)
        distance_ahead = distance + self.lookahead * math.sin(alpha)
        error = distance_ahead - self.desired

        now = rclpy.time.Time.from_msg(scan.header.stamp).nanoseconds * 1e-9
        dt = now - self.prev_time if self.prev_time is not None else 0.0
        derivative = 0.0
        if dt > 1e-6:
            self.integral = clamp(self.integral + error * dt, -self.integral_limit, self.integral_limit)
            derivative = (error - self.prev_error) / dt
        self.prev_error = error
        self.prev_time = now

        # Too far from a left wall -> steer left (+); mirrored for the right wall
        u = self.kp * error + self.ki * self.integral + self.kd * derivative
        steering = clamp(self.side * u, -self.max_steer, self.max_steer)

        self.publish(self.speed_for(steering, front_distance(scan, self.front_half_angle)), steering)

    def speed_for(self, steering, front):
        """Slow down in turns and when the free distance ahead gets short."""
        if front is not None and front < self.front_stop:
            return 0.0
        turn = 1.0 - abs(steering) / self.max_steer   # 1 straight, 0 full lock
        speed = self.speed_min + (self.speed_max - self.speed_min) * turn
        if front is not None and front < self.front_slow:
            ratio = (front - self.front_stop) / (self.front_slow - self.front_stop)
            speed = min(speed, self.speed_min + (self.speed_max - self.speed_min) * ratio)
        return speed

    def reset_pid(self):
        self.prev_error = None
        self.prev_time = None
        self.integral = 0.0

    def publish(self, speed, steering):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.speed = speed
        msg.drive.steering_angle = steering
        self.drive_pub.publish(msg)

    def stop(self):
        self.publish(0.0, 0.0)


def valid_ranges(scan, angle_low, angle_high):
    """Finite ranges inside [range_min, range_max] for beams between two angles."""
    first = max(0, math.ceil((angle_low - scan.angle_min) / scan.angle_increment))
    last = min(len(scan.ranges) - 1, math.floor((angle_high - scan.angle_min) / scan.angle_increment))
    return [
        r for r in scan.ranges[first:last + 1]
        if math.isfinite(r) and scan.range_min <= r <= scan.range_max
    ]


def beam_range(scan, angle, window):
    """Median range around one beam angle (robust to noise and single dropouts), None if no return."""
    ranges = sorted(valid_ranges(scan, angle - window, angle + window))
    return ranges[len(ranges) // 2] if ranges else None


def front_distance(scan, half_angle):
    """Closest return in the cone ahead of the car, None if nothing in range."""
    ranges = valid_ranges(scan, -half_angle, half_angle)
    return min(ranges) if ranges else None


def clamp(value, low, high):
    return max(low, min(high, value))


def main(args=None):
    rclpy.init(args=args)
    try:
        node = WallFollow()
    except RuntimeError as error:
        print(f'[wall_follow] {error}', file=sys.stderr)
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
