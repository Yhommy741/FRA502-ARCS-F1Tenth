#!/usr/bin/python3

import sys

import rclpy
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Imu


# From arcsf1tenth_description/config/sensors.yaml
SENSOR_PARAMS = ('imu.gyro_noise_std', 'imu.accel_noise_std')


def namespaced_frame(node, name):
    """TF frame in the node's namespace: /arcsf1tenth + imu -> arcsf1tenth/imu (no namespace -> imu)."""
    namespace = node.get_namespace().strip('/')
    return f'{namespace}/{name}' if namespace else name


class GazeboImu(Node):
    """Simulated IMU driver: gazebo/imu -> imu, like the MCU's IMU output on the real car.

    - sets header.frame_id to the URDF IMU frame (Gazebo stamps its own scoped
      sensor name, which does not exist in TF)
    - fills the covariance fields from the IMU noise (Gazebo leaves them zero)

    Noise values come from sensors.yaml, the same file the Gazebo IMU noise uses.
    """

    def __init__(self):
        super().__init__('gazebo_imu')

        # IMU noise: no defaults, must come from arcsf1tenth_description/config/sensors.yaml
        for name in SENSOR_PARAMS:
            self.declare_parameter(name, Parameter.Type.DOUBLE)
        missing = []
        for name in SENSOR_PARAMS:
            try:
                self.get_parameter(name)
            except ParameterUninitializedException:
                missing.append(name)
        if missing:
            raise RuntimeError(
                f'Missing sensor parameters {missing}. Load them with: '
                '--ros-args --params-file $(ros2 pkg prefix arcsf1tenth_description)'
                '/share/arcsf1tenth_description/config/sensors.yaml'
            )

        self.declare_parameter('frame_id', namespaced_frame(self, 'imu'))

        self.frame_id = self.get_parameter('frame_id').value
        gyro_var = self.get_parameter('imu.gyro_noise_std').value ** 2
        accel_var = self.get_parameter('imu.accel_noise_std').value ** 2
        self.gyro_cov = [gyro_var, 0.0, 0.0, 0.0, gyro_var, 0.0, 0.0, 0.0, gyro_var]
        self.accel_cov = [accel_var, 0.0, 0.0, 0.0, accel_var, 0.0, 0.0, 0.0, accel_var]

        self.imu_pub = self.create_publisher(Imu, 'imu', 50)
        self.create_subscription(Imu, 'gazebo/imu', self.imu_callback, 50)

    def imu_callback(self, msg):
        msg.header.frame_id = self.frame_id
        msg.angular_velocity_covariance = self.gyro_cov
        msg.linear_acceleration_covariance = self.accel_cov
        self.imu_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = GazeboImu()
    except RuntimeError as error:
        print(f'[gazebo_imu] {error}', file=sys.stderr)
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
