#!/usr/bin/python3

import math
import sys

import rclpy
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


VEHICLE_PARAMS = ('wheelbase', 'wheel_radius', 'max_steering_angle', 'max_steering_speed')


def namespaced_frame(node, name):
    """TF frame in the node's namespace: /arcsf1tenth + imu -> arcsf1tenth/imu (no namespace -> imu)."""
    namespace = node.get_namespace().strip('/')
    return f'{namespace}/{name}' if namespace else name


class GazeboMeasureOdometry(Node):
    """Wheel odometry from the simulated drive, like vesc_to_odom on the real car.

    - speed: rear wheel joint velocities (the VESC measures motor speed)
    - yaw rate: bicycle model v * tan(steering) / wheelbase, using the steering
      *command* through the servo slew-rate limit (a servo has no angle feedback)
    - pose: dead reckoning of the above

    Publishes nav_msgs/Odometry on odometry/measured (odom -> base_footprint).
    Publishes the odom -> base_footprint TF only when publish_tf is true (no EKF
    running); otherwise the EKF fuses this with the IMU and owns the TF.
    """

    def __init__(self):
        super().__init__('gazebo_measure_odometry')

        # Vehicle geometry: must come from arcsf1tenth_description/config/vehicle.yaml
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

        self.declare_parameter('odom_frame', namespaced_frame(self, 'odom'))
        self.declare_parameter('base_frame', namespaced_frame(self, 'base_footprint'))
        self.declare_parameter('publish_tf', True)   # false when the EKF owns odom -> base_footprint
        # +1 or -1: rear wheel joint axes point so that forward motion is negative rotation
        self.declare_parameter('wheel_direction', -1.0)
        self.declare_parameter('speed_std', 0.05)    # [m/s] reported in the twist covariance
        self.declare_parameter('yaw_rate_std', 0.05)  # [rad/s] reported in the twist covariance

        p = lambda name: self.get_parameter(name).value
        self.wheelbase = p('wheelbase')
        self.wheel_radius = p('wheel_radius')
        self.max_steer = p('max_steering_angle')
        self.max_steer_speed = p('max_steering_speed')
        self.odom_frame = p('odom_frame')
        self.base_frame = p('base_frame')
        self.publish_tf = p('publish_tf')
        self.wheel_direction = p('wheel_direction')
        self.speed_var = p('speed_std') ** 2
        self.yaw_rate_var = p('yaw_rate_std') ** 2

        self.cmd_steer = 0.0
        self.steer = 0.0
        self.x = self.y = self.yaw = 0.0
        self.last_stamp = None

        self.odom_pub = self.create_publisher(Odometry, 'odometry/measured', 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.create_subscription(AckermannDriveStamped, 'drive', self.drive_callback, 10)
        self.create_subscription(JointState, 'joint_states', self.joint_states_callback, 10)

    def drive_callback(self, msg):
        self.cmd_steer = clamp(msg.drive.steering_angle, -self.max_steer, self.max_steer)

    def joint_states_callback(self, msg):
        try:
            rl = msg.velocity[msg.name.index('RL_wheel_joint')]
            rr = msg.velocity[msg.name.index('RR_wheel_joint')]
        except (ValueError, IndexError):
            return

        stamp = Time.from_msg(msg.header.stamp)
        if self.last_stamp is None or stamp <= self.last_stamp:
            self.last_stamp = stamp
            return
        dt = (stamp - self.last_stamp).nanoseconds * 1e-9
        self.last_stamp = stamp

        # Servo slew-rate limit on the steering command
        step = self.max_steer_speed * dt
        self.steer = clamp(self.cmd_steer, self.steer - step, self.steer + step)

        # Rear axle center speed (average of both rear wheels) and bicycle yaw rate
        speed = self.wheel_direction * (rl + rr) / 2.0 * self.wheel_radius
        yaw_rate = speed * math.tan(self.steer) / self.wheelbase

        # Dead reckoning (midpoint heading)
        mid_yaw = self.yaw + yaw_rate * dt / 2.0
        self.x += speed * math.cos(mid_yaw) * dt
        self.y += speed * math.sin(mid_yaw) * dt
        self.yaw = math.atan2(math.sin(self.yaw + yaw_rate * dt), math.cos(self.yaw + yaw_rate * dt))

        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        odom.twist.twist.linear.x = speed
        odom.twist.twist.angular.z = yaw_rate
        twist_cov = [0.0] * 36
        twist_cov[0], twist_cov[7], twist_cov[35] = self.speed_var, 1e-6, self.yaw_rate_var
        for i in (14, 21, 28):
            twist_cov[i] = 1e6
        odom.twist.covariance = twist_cov
        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            t = TransformStamped()
            t.header = odom.header
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(t)


def clamp(value, low, high):
    return max(low, min(high, value))


def main(args=None):
    rclpy.init(args=args)
    try:
        node = GazeboMeasureOdometry()
    except RuntimeError as error:
        print(f'[gazebo_measure_odometry] {error}', file=sys.stderr)
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
