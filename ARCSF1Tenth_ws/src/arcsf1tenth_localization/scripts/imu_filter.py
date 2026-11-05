#!/usr/bin/python3
"""IMU preprocessing for the EKF: imu -> imu/filtered.

    imu (raw BNO055 / gazebo_imu) ──► imu_filter ──► imu/filtered ──► robot_localization EKF
    odometry/wheel (wheel speed) ──┘                ──► ~/gyro_bias (std_msgs/Float64, z axis)

robot_localization has no gyro-bias state, so the bias is removed here
(design after the carver_f1tenth imu_filter):

1. Startup calibration: while the car stands still for `calibration_seconds`
   (wheel speed ~0 and gyro quiet), measure the gyro bias and the gyro/accel
   noise variance per axis. Restarts if the car moves. Nothing is published
   before calibration, unless it times out (then bias = 0 until the first ZUPT).
2. Bias subtraction on every message.
3. ZUPT (zero-velocity update): whenever the car is verified still again for
   `zupt_settle_time` (wheel speed below `still_speed` AND bias-corrected gyro
   below `still_gyro`), refresh the bias slowly (exponential average,
   `zupt_alpha`), so it follows warm-up / temperature drift.
4. Optional horizontal projection (`project_to_horizontal`): remove the
   chassis roll/pitch (suspension) using the IMU orientation, so
   angular_velocity.z is the yaw rate about gravity.
5. Optional spike/noise filter on the gyro: median of `gyro_median_window`
   samples, then moving average of `gyro_ma_window` samples (1 = off).
6. Covariance: the measured variances go into the message covariance fields,
   so the EKF weights the gyro by how noisy it really is.
"""

import math
from collections import deque
from statistics import median, pvariance

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64


class ImuFilter(Node):
    def __init__(self):
        super().__init__('imu_filter')

        # 1. startup calibration
        self.declare_parameter('calibration_seconds', 3.0)    # [s] still time to measure bias + noise
        self.declare_parameter('calibration_timeout', 15.0)   # [s] give up and publish with bias 0
        # standstill detection (calibration + ZUPT)
        self.declare_parameter('still_speed', 0.05)           # [m/s] wheel speed below this = not driving
        self.declare_parameter('still_gyro', 0.05)            # [rad/s] bias-corrected gyro norm below this = not turning
        # 3. ZUPT
        self.declare_parameter('zupt_enabled', True)
        self.declare_parameter('zupt_settle_time', 0.5)       # [s] still this long before updating the bias
        self.declare_parameter('zupt_alpha', 0.01)            # per sample; 0.01 at 100 Hz ~ 1 s time constant
        # 4. horizontal projection (real car with suspension: true)
        self.declare_parameter('project_to_horizontal', False)
        # 5. gyro filter (1 = off). Real car: median 3, moving average 5 (~20 ms delay at 100 Hz)
        self.declare_parameter('gyro_median_window', 1)
        self.declare_parameter('gyro_ma_window', 1)
        # 6. covariance
        self.declare_parameter('min_variance', 1.0e-8)        # floor for the measured variances

        p = lambda name: self.get_parameter(name).value
        self.calibration_time = Duration(seconds=p('calibration_seconds'))
        self.calibration_timeout = Duration(seconds=p('calibration_timeout'))
        self.still_speed = p('still_speed')
        self.still_gyro = p('still_gyro')
        self.zupt_enabled = p('zupt_enabled')
        self.zupt_settle = Duration(seconds=p('zupt_settle_time'))
        self.zupt_alpha = p('zupt_alpha')
        self.project = p('project_to_horizontal')
        self.min_var = p('min_variance')

        median_window = max(1, int(p('gyro_median_window')))
        ma_window = max(1, int(p('gyro_ma_window')))
        self.median_buffers = [deque(maxlen=median_window) for _ in range(3)]
        self.ma_buffers = [deque(maxlen=ma_window) for _ in range(3)]

        # state
        self.bias = [0.0, 0.0, 0.0]
        self.gyro_var = None              # measured at calibration
        self.accel_var = None
        self.calibrated = False
        self.start_time = None
        self.calib_samples = []           # (gyro, accel) during the current still window
        self.calib_window_start = None
        self.wheel_speed = None           # None until the first wheel odometry arrives
        self.still_since = None

        self.imu_pub = self.create_publisher(Imu, 'imu/filtered', 50)
        self.bias_pub = self.create_publisher(Float64, '~/gyro_bias', 10)
        self.create_subscription(Imu, 'imu', self.imu_callback, 50)
        self.create_subscription(Odometry, 'odometry/wheel', self.wheel_callback, 10)
        self.get_logger().info(
            f'Calibrating gyro: keep the car still for {p("calibration_seconds"):.1f} s '
            f'(projection {"on" if self.project else "off"}, '
            f'median {median_window}, moving average {ma_window})')

    # ---------------------------------------------------------------- inputs

    def wheel_callback(self, msg):
        self.wheel_speed = msg.twist.twist.linear.x

    def imu_callback(self, msg):
        now = self.get_clock().now()
        if self.start_time is None:
            self.start_time = now
        gyro = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        accel = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        corrected = [g - b for g, b in zip(gyro, self.bias)]
        still = self.is_still(corrected)

        if not self.calibrated:
            self.calibrate(now, gyro, accel, still)
            if not self.calibrated:
                return
            corrected = [g - b for g, b in zip(gyro, self.bias)]
        elif self.zupt_enabled:
            self.zupt(now, gyro, still)

        self.publish(msg, corrected)

    # ----------------------------------------------------------- calibration

    def is_still(self, corrected_gyro):
        """Car verified still: wheels stopped and (bias-corrected) gyro quiet."""
        wheels_stopped = self.wheel_speed is not None and abs(self.wheel_speed) < self.still_speed
        return wheels_stopped and math.sqrt(sum(g * g for g in corrected_gyro)) < self.still_gyro

    def calibrate(self, now, gyro, accel, still):
        """1. Startup calibration: bias + variance over one uninterrupted still window."""
        if not still:
            self.calib_samples, self.calib_window_start = [], None   # moved: start over
        else:
            if self.calib_window_start is None:
                self.calib_window_start = now
            self.calib_samples.append((gyro, accel))
            if now - self.calib_window_start >= self.calibration_time and len(self.calib_samples) >= 10:
                gyros = list(zip(*[s[0] for s in self.calib_samples]))
                accels = list(zip(*[s[1] for s in self.calib_samples]))
                self.bias = [sum(axis) / len(axis) for axis in gyros]
                self.gyro_var = [max(pvariance(axis), self.min_var) for axis in gyros]
                self.accel_var = [max(pvariance(axis), self.min_var) for axis in accels]
                self.calibrated = True
                self.get_logger().info(
                    f'Gyro calibrated from {len(self.calib_samples)} samples: '
                    f'bias z {self.bias[2]:+.4f} rad/s ({math.degrees(self.bias[2]):+.2f} deg/s), '
                    f'noise z std {math.sqrt(self.gyro_var[2]):.4f} rad/s')
                return

        if now - self.start_time > self.calibration_timeout:
            self.calibrated = True   # publish anyway; bias 0 until the first ZUPT
            self.get_logger().warn(
                'Gyro calibration timed out (car never still, or no odometry/wheel): '
                'publishing with bias 0 until the car stops (ZUPT)')

    def zupt(self, now, gyro, still):
        """3. Zero-velocity update: refresh the bias while the car stays still."""
        if not still:
            self.still_since = None
            return
        if self.still_since is None:
            self.still_since = now
        if now - self.still_since < self.zupt_settle:
            return
        a = self.zupt_alpha
        self.bias = [(1.0 - a) * b + a * g for b, g in zip(self.bias, gyro)]

    # ---------------------------------------------------------------- output

    def publish(self, msg, gyro):
        if self.project:
            gyro = self.horizontal(msg.orientation, gyro)
        gyro = self.smooth(gyro)

        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = gyro
        if self.gyro_var is not None:
            gx, gy, gz = self.gyro_var
            msg.angular_velocity_covariance = [gx, 0.0, 0.0, 0.0, gy, 0.0, 0.0, 0.0, gz]
        if self.accel_var is not None:
            ax, ay, az = self.accel_var
            msg.linear_acceleration_covariance = [ax, 0.0, 0.0, 0.0, ay, 0.0, 0.0, 0.0, az]
        self.imu_pub.publish(msg)
        self.bias_pub.publish(Float64(data=self.bias[2]))

    @staticmethod
    def horizontal(q, gyro):
        """4. Rotate the gyro vector by the IMU's roll and pitch (not yaw): z = yaw rate about gravity."""
        if q.x == 0.0 and q.y == 0.0 and q.z == 0.0 and q.w == 0.0:
            return gyro   # no orientation from this IMU
        roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y))
        pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
        cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
        x, y, z = gyro
        # R = Ry(pitch) @ Rx(roll), applied to the body-frame vector
        y1, z1 = cr * y - sr * z, sr * y + cr * z
        return [cp * x + sp * z1, y1, -sp * x + cp * z1]

    def smooth(self, gyro):
        """5. Median (spike rejection) then moving average (noise) per axis."""
        out = []
        for i, value in enumerate(gyro):
            self.median_buffers[i].append(value)
            self.ma_buffers[i].append(median(self.median_buffers[i]))
            out.append(sum(self.ma_buffers[i]) / len(self.ma_buffers[i]))
        return out


def main(args=None):
    rclpy.init(args=args)
    node = ImuFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
