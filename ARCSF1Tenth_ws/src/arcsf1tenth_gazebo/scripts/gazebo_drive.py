#!/usr/bin/python3

import math
import sys

import rclpy
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node
from rclpy.parameter import Parameter
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64


VEHICLE_PARAMS = (
    'wheelbase', 'front_track', 'rear_track',
    'wheel_radius', 'max_steering_angle', 'max_speed', 'max_acceleration',
)


class GazeboDrive(Node):
    """Convert /drive (steering angle + speed) into Gazebo joint commands.

    Simulation counterpart of the real car's drive hardware (servo + VESC):
    steering is sent as a joint position (like a servo), speed as rear-wheel
    velocity (like a motor with speed PID), with an acceleration limit. Uses
    Ackermann geometry so the inner wheel steers more and the rear wheels spin
    at different speeds in a turn.
    """

    def __init__(self):
        super().__init__('gazebo_drive')

        # Vehicle geometry: no defaults, must come from arcsf1tenth_description/config/vehicle.yaml
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
                f'Missing vehicle parameters {missing}. Load them with: '
                '--ros-args --params-file $(ros2 pkg prefix arcsf1tenth_description)'
                '/share/arcsf1tenth_description/config/vehicle.yaml'
            )

        self.declare_parameter('cmd_timeout', 0.5)  # [s] stop the wheels if /drive goes silent
        # +1 or -1: rear wheel joint axes point so that forward motion is negative rotation
        self.declare_parameter('wheel_direction', -1.0)

        self.wheelbase = self.get_parameter('wheelbase').value
        self.front_track = self.get_parameter('front_track').value
        self.rear_track = self.get_parameter('rear_track').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.max_steer = self.get_parameter('max_steering_angle').value
        self.max_speed = self.get_parameter('max_speed').value
        self.max_accel = self.get_parameter('max_acceleration').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value
        self.wheel_direction = self.get_parameter('wheel_direction').value

        self.steer_left_pub = self.create_publisher(Float64, 'steer_left_cmd', 10)
        self.steer_right_pub = self.create_publisher(Float64, 'steer_right_cmd', 10)
        self.wheel_rl_pub = self.create_publisher(Float64, 'wheel_rl_cmd', 10)
        self.wheel_rr_pub = self.create_publisher(Float64, 'wheel_rr_cmd', 10)

        self.create_subscription(AckermannDriveStamped, 'drive', self.drive_callback, 10)

        self.steering_angle = 0.0
        self.speed = 0.0
        self.wheel_speed_cmd = 0.0  # speed actually sent to the wheels (acceleration limited)
        self.last_cmd_time = None
        self.timed_out = True

        self.create_timer(0.02, self.timer_callback)  # 50 Hz
        self.get_logger().info('Listening on /drive')

    def drive_callback(self, msg):
        self.steering_angle = clamp(msg.drive.steering_angle, -self.max_steer, self.max_steer)
        self.speed = clamp(msg.drive.speed, -self.max_speed, self.max_speed)
        self.last_cmd_time = self.get_clock().now()

    def timer_callback(self):
        # Watchdog: stop the wheels (keep steering) if commands stop arriving
        now = self.get_clock().now()
        stale = (
            self.last_cmd_time is None
            or (now - self.last_cmd_time).nanoseconds * 1e-9 > self.cmd_timeout
        )
        if stale and not self.timed_out:
            self.get_logger().warn('No /drive command received, stopping')
        self.timed_out = stale
        target = 0.0 if stale else self.speed

        # Motor/brake limit: ramp toward the target like a real ESC instead of
        # locking the wheels (an instant stop at speed makes the car skid)
        step = self.max_accel * 0.02
        self.wheel_speed_cmd = clamp(target, self.wheel_speed_cmd - step, self.wheel_speed_cmd + step)

        steer_left, steer_right, v_left, v_right = self.ackermann(self.steering_angle, self.wheel_speed_cmd)

        self.steer_left_pub.publish(Float64(data=steer_left))
        self.steer_right_pub.publish(Float64(data=steer_right))
        self.wheel_rl_pub.publish(Float64(data=self.wheel_direction * v_left / self.wheel_radius))
        self.wheel_rr_pub.publish(Float64(data=self.wheel_direction * v_right / self.wheel_radius))

    def ackermann(self, steering_angle, speed):
        """Return (left steer, right steer, left rear speed, right rear speed)."""
        if abs(steering_angle) < 1e-4:
            return steering_angle, steering_angle, speed, speed

        # Signed turning radius of the rear axle center (positive = turning left)
        radius = self.wheelbase / math.tan(steering_angle)

        steer_left = math.atan(self.wheelbase / (radius - self.front_track / 2.0))
        steer_right = math.atan(self.wheelbase / (radius + self.front_track / 2.0))

        v_left = speed * (radius - self.rear_track / 2.0) / radius
        v_right = speed * (radius + self.rear_track / 2.0) / radius

        return steer_left, steer_right, v_left, v_right


def clamp(value, low, high):
    return max(low, min(high, value))


def main(args=None):
    rclpy.init(args=args)
    try:
        node = GazeboDrive()
    except RuntimeError as error:
        print(f'[gazebo_drive] {error}', file=sys.stderr)
        rclpy.try_shutdown()
        sys.exit(1)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
