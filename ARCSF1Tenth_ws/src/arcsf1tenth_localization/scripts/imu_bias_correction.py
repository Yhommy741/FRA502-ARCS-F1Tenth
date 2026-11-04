#!/usr/bin/python3

import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64


class ImuBiasCorrection(Node):
    """Gyro bias removal by zero-velocity update (ZUPT): imu -> imu/corrected.

    robot_localization's EKF has no gyro-bias state, and the BNO055 zero-rate
    offset (±1 deg/s) would turn into ~60 deg/min of heading drift. Whenever the
    wheels report the car standing still for `still_time`, every gyro reading is
    pure bias, so the bias is averaged then and subtracted from all readings.

    Inputs:  imu (sensor_msgs/Imu), odometry/measured (wheel speed)
    Outputs: imu/corrected (sensor_msgs/Imu), ~/gyro_bias (std_msgs/Float64, z axis)
    """

    def __init__(self):
        super().__init__('imu_bias_correction')

        self.declare_parameter('still_speed', 0.01)     # [m/s] below this the car is standing still
        self.declare_parameter('still_time', 0.5)       # [s] standing still this long before measuring
        self.declare_parameter('filter_time', 2.0)      # [s] time constant of the bias average

        self.still_speed = self.get_parameter('still_speed').value
        self.still_time = Duration(seconds=self.get_parameter('still_time').value)
        self.filter_time = self.get_parameter('filter_time').value

        self.bias = [0.0, 0.0, 0.0]
        self.calibrated = False
        self.still_since = None
        self.last_imu_time = None

        self.imu_pub = self.create_publisher(Imu, 'imu/corrected', 50)
        self.bias_pub = self.create_publisher(Float64, '~/gyro_bias', 10)
        self.create_subscription(Imu, 'imu', self.imu_callback, 50)
        self.create_subscription(Odometry, 'odometry/measured', self.odom_callback, 10)

    def odom_callback(self, msg):
        moving = abs(msg.twist.twist.linear.x) > self.still_speed
        now = self.get_clock().now()
        if moving:
            self.still_since = None
        elif self.still_since is None:
            self.still_since = now

    def standing_still(self, now):
        return self.still_since is not None and now - self.still_since >= self.still_time

    def imu_callback(self, msg):
        now = self.get_clock().now()
        gyro = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]

        if self.standing_still(now):
            # Exponential average: weight = dt / filter_time
            dt = 0.01 if self.last_imu_time is None else (now - self.last_imu_time).nanoseconds * 1e-9
            alpha = 1.0 if not self.calibrated else min(1.0, max(dt, 0.0) / self.filter_time)
            self.bias = [b + alpha * (g - b) for b, g in zip(self.bias, gyro)]
            if not self.calibrated:
                self.calibrated = True
                self.get_logger().info(f'Gyro bias calibrated: z = {self.bias[2]:+.4f} rad/s '
                                       f'({math.degrees(self.bias[2]):+.2f} deg/s)')
        self.last_imu_time = now

        msg.angular_velocity.x -= self.bias[0]
        msg.angular_velocity.y -= self.bias[1]
        msg.angular_velocity.z -= self.bias[2]
        self.imu_pub.publish(msg)
        self.bias_pub.publish(Float64(data=self.bias[2]))


def main(args=None):
    rclpy.init(args=args)
    node = ImuBiasCorrection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
