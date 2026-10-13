#!/usr/bin/python3

import math
import sys

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from tf2_ros import Buffer, TransformBroadcaster, TransformListener


# State vector order (rows/columns of the covariance matrices in ekf.yaml)
STATE_NAMES = ('x', 'y', 'yaw', 'v', 'yaw_rate', 'gyro_bias')
# Sensor config mask slots (2D subset of robot_localization's 15-slot mask)
MASK_SLOTS = ('x', 'y', 'yaw', 'vx', 'vy', 'vyaw')


def namespaced_frame(node, name):
    """TF frame in the node's namespace: /arcsf1tenth + imu -> arcsf1tenth/imu (no namespace -> imu)."""
    namespace = node.get_namespace().strip('/')
    return f'{namespace}/{name}' if namespace else name

# State vector indices
X, Y, YAW, V, W, BIAS = range(6)


class EKF(Node):
    """Vehicle odometry: EKF fusing wheel odometry and IMU yaw rate.

    All noise values and sensor masks come from config/ekf.yaml.

    Inputs (the same in simulation and on the real car):
      odometry/measured  nav_msgs/Odometry: speed + bicycle-model yaw rate
                         (sim: gazebo_measure_odometry, real: VESC driver / vesc_to_odom)
      imu                sensor_msgs/Imu: gyro yaw rate (sim: Gazebo IMU, real: MCU)
    Output:
      odometry/estimated nav_msgs/Odometry, and TF odom -> base_footprint

    State: [x, y, yaw, v, yaw_rate, gyro_bias] of base_footprint (rear axle
    center) in the odom frame.

    Predict: constant speed and yaw rate.
    Updates:
      - gyro yaw rate    z = yaw_rate + gyro_bias   (if use_imu)
      - wheel speed      z = v
      - bicycle yaw rate z = yaw_rate               skipped when it disagrees with the gyro (slip)

    The bicycle yaw rate is a measurement, not the prediction, so when the
    tyres slip and it disagrees with the gyro, the filter trusts the gyro.
    """

    def __init__(self):
        super().__init__('ekf')

        self.declare_parameter('frequency', 50.0)            # [Hz] predict + publish
        self.declare_parameter('sensor_timeout', 0.2)        # [s] ignore older wheel odometry / IMU data
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('odom_frame', namespaced_frame(self, 'odom'))
        self.declare_parameter('base_link_frame', namespaced_frame(self, 'base_footprint'))

        # Sensors. Config masks are over MASK_SLOTS; each sensor supports only some slots.
        self.declare_parameter('use_imu', True)              # launch switch, overrides imu0_config
        self.declare_parameter('imu0', 'imu')
        self.declare_parameter('imu0_config', [False] * 6)
        self.declare_parameter('imu0_vyaw_variance', 5.76e-6)
        self.declare_parameter('twist0', 'odometry/measured')
        self.declare_parameter('twist0_config', [False] * 6)
        self.declare_parameter('twist0_vx_variance', 0.0025)
        self.declare_parameter('twist0_vyaw_variance', 0.0225)
        self.declare_parameter('twist0_vyaw_slip_factor', 0.5)
        self.declare_parameter('twist0_slip_gate_sigma', 2.0)

        # 6x6 matrices over STATE_NAMES, row-major
        self.declare_parameter('process_noise_covariance', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('initial_estimate_covariance', Parameter.Type.DOUBLE_ARRAY)

        p = lambda name: self.get_parameter(name).value
        self.odom_frame = p('odom_frame')
        self.base_frame = p('base_link_frame')
        self.publish_tf = p('publish_tf')
        self.sensor_timeout = Duration(seconds=p('sensor_timeout'))

        imu_mask = self.read_mask('imu0_config', supported=('vyaw',))
        twist_mask = self.read_mask('twist0_config', supported=('vx', 'vyaw'))
        self.use_gyro = p('use_imu') and imu_mask['vyaw']
        self.use_wheel_speed = twist_mask['vx']
        self.use_bicycle_rate = twist_mask['vyaw']
        self.r_gyro = p('imu0_vyaw_variance')
        self.r_speed = p('twist0_vx_variance')
        self.bicycle_std = math.sqrt(p('twist0_vyaw_variance'))
        self.slip_factor = p('twist0_vyaw_slip_factor')
        self.slip_gate = p('twist0_slip_gate_sigma')

        self.Q = self.read_matrix('process_noise_covariance')     # per second
        self.x = np.zeros(6)
        self.P = self.read_matrix('initial_estimate_covariance')
        self.last_time = None
        self.slipping = False

        # Latest inputs
        self.wheel = None                   # (speed, bicycle yaw rate)
        self.wheel_time = None
        self.gyro_sum = 0.0
        self.gyro_count = 0
        self.gyro_time = None
        self.imu_rotation = {}              # frame_id -> 3x3 rotation into base_frame

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, 'odometry/estimated', 10)
        self.bias_pub = self.create_publisher(Float64, '~/gyro_bias', 10)

        if self.use_wheel_speed or self.use_bicycle_rate:
            self.create_subscription(Odometry, p('twist0'), self.wheel_odom_callback, 10)
        if self.use_gyro:
            self.create_subscription(Imu, p('imu0'), self.imu_callback, 50)

        self.create_timer(1.0 / p('frequency'), self.step)
        used = [name for name, on in (
            (f"{p('twist0')} vx", self.use_wheel_speed),
            (f"{p('twist0')} vyaw", self.use_bicycle_rate),
            (f"{p('imu0')} vyaw", self.use_gyro),
        ) if on]
        self.get_logger().info(f"Odometry {self.odom_frame} -> {self.base_frame} from {', '.join(used) or 'nothing'}")

    def read_mask(self, name, supported):
        """Config mask [x, y, yaw, vx, vy, vyaw] -> {slot: bool}; rejects slots this sensor can't give."""
        values = list(self.get_parameter(name).value)
        if len(values) != len(MASK_SLOTS):
            raise RuntimeError(f'{name} needs {len(MASK_SLOTS)} entries {list(MASK_SLOTS)}, got {len(values)}')
        mask = dict(zip(MASK_SLOTS, values))
        unsupported = [slot for slot, on in mask.items() if on and slot not in supported]
        if unsupported:
            raise RuntimeError(f'{name}: {unsupported} not supported (this sensor gives {list(supported)})')
        return mask

    def read_matrix(self, name):
        """6x6 covariance from a 36-entry row-major list."""
        try:
            values = self.get_parameter(name).value
        except ParameterUninitializedException:
            raise RuntimeError(f'Missing {name}: load arcsf1tenth_localization/config/ekf.yaml')
        if len(values) != 36:
            raise RuntimeError(f'{name} needs 36 entries (6x6 over {list(STATE_NAMES)}), got {len(values)}')
        matrix = np.array(values, dtype=float).reshape(6, 6)
        if not np.allclose(matrix, matrix.T):
            raise RuntimeError(f'{name} must be symmetric')
        return matrix

    # ---------------------------------------------------------------- inputs

    def wheel_odom_callback(self, msg):
        self.wheel = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)
        self.wheel_time = self.get_clock().now()

    def imu_callback(self, msg):
        rotation = self.get_imu_rotation(msg.header.frame_id)
        if rotation is None:
            return
        w = rotation @ np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        self.gyro_sum += w[2]
        self.gyro_count += 1
        self.gyro_time = self.get_clock().now()

    def get_imu_rotation(self, frame_id):
        """Rotation from the IMU frame into base_frame (IMU may be mounted in any orientation)."""
        if frame_id in self.imu_rotation:
            return self.imu_rotation[frame_id]
        try:
            t = self.tf_buffer.lookup_transform(self.base_frame, frame_id, Time())
        except Exception as error:
            self.get_logger().warn(
                f'Waiting for TF {self.base_frame} <- {frame_id}: {error}', throttle_duration_sec=2.0)
            return None
        q = t.transform.rotation
        self.imu_rotation[frame_id] = quaternion_to_matrix(q.x, q.y, q.z, q.w)
        return self.imu_rotation[frame_id]

    # ---------------------------------------------------------------- filter

    def step(self):
        now = self.get_clock().now()
        if self.last_time is None or now < self.last_time:  # first step, or sim time reset
            self.last_time = now
            return
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0.0:
            return

        self.predict(dt)

        # Gyro first: it measures the actual rotation, so the slip check below compares against it
        has_gyro = self.use_gyro and self.gyro_count > 0 and self.fresh(self.gyro_time, now)
        if has_gyro:
            self.update(self.gyro_sum / self.gyro_count, np.array([0, 0, 0, 0, 1, 1.0]), self.r_gyro)
        self.gyro_sum, self.gyro_count = 0.0, 0

        if self.wheel is not None and self.fresh(self.wheel_time, now):
            speed, bicycle_rate = self.wheel
            if self.use_wheel_speed:
                self.update(speed, np.array([0, 0, 0, 1, 0, 0.0]), self.r_speed)

            # Bicycle yaw rate: less trusted in hard turns, skipped when slipping
            H = np.array([0, 0, 0, 0, 1, 0.0])
            R = (self.bicycle_std + self.slip_factor * abs(bicycle_rate)) ** 2
            slipping = has_gyro and abs(bicycle_rate - self.x[W]) > self.slip_gate * math.sqrt(H @ self.P @ H + R)
            if slipping != self.slipping:
                self.get_logger().info('Slip detected: ignoring bicycle yaw rate' if slipping else 'Grip again')
                self.slipping = slipping
            if self.use_bicycle_rate and not slipping:
                self.update(bicycle_rate, H, R)

        self.publish(now)

    def predict(self, dt):
        x, y, yaw, v, w, b = self.x
        c, s = math.cos(yaw), math.sin(yaw)
        self.x = np.array([x + v * c * dt, y + v * s * dt, wrap(yaw + w * dt), v, w, b])

        F = np.eye(6)
        F[X, YAW] = -v * s * dt
        F[X, V] = c * dt
        F[Y, YAW] = v * c * dt
        F[Y, V] = s * dt
        F[YAW, W] = dt
        self.P = F @ self.P @ F.T + self.Q * dt

    def update(self, z, H, R):
        """Scalar measurement update: z = H @ x + noise(R)."""
        innovation = z - H @ self.x
        S = H @ self.P @ H + R
        K = self.P @ H / S
        self.x = self.x + K * innovation
        self.x[YAW] = wrap(self.x[YAW])
        I_KH = np.eye(6) - np.outer(K, H)
        self.P = I_KH @ self.P @ I_KH.T + np.outer(K, K) * R  # Joseph form: stays symmetric/positive

    def fresh(self, stamp, now):
        return stamp is not None and (now - stamp) < self.sensor_timeout

    # --------------------------------------------------------------- outputs

    def publish(self, now):
        x, y, yaw, v, w, b = self.x
        qz, qw = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        stamp = now.to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        # 6x6 covariances (x, y, z, roll, pitch, yaw); unused axes get a large value
        pose_cov = np.diag([0.0, 0.0, 1e6, 1e6, 1e6, 0.0])
        pose_cov[np.ix_([0, 1, 5], [0, 1, 5])] = self.P[np.ix_([X, Y, YAW], [X, Y, YAW])]
        odom.pose.covariance = pose_cov.flatten().tolist()
        twist_cov = np.diag([self.P[V, V], 1e-6, 1e6, 1e6, 1e6, self.P[W, W]])
        odom.twist.covariance = twist_cov.flatten().tolist()
        self.odom_pub.publish(odom)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)

        self.bias_pub.publish(Float64(data=float(b)))


def quaternion_to_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def main(args=None):
    rclpy.init(args=args)
    try:
        node = EKF()
    except RuntimeError as error:
        print(f'[ekf] {error}', file=sys.stderr)
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
