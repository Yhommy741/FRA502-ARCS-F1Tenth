#!/usr/bin/python3

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


# Return the yaw angle of a quaternion.
def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


# Wrap an angle to [-pi, pi].
def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class LocalizationError(Node):
    # Set up the node: frames, error statistics, optional CSV, ground-truth subscriber and sample timer.
    def __init__(self):
        super().__init__('localization_error', namespace='arcsf1tenth')

        # Parameters
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('base_frame', 'base_footprint')  # namespaced automatically
        self.declare_parameter('rate', 10.0)                     # [Hz] samples
        self.declare_parameter('csv', '')                        # optional output file
        self.set_parameters([Parameter('use_sim_time', value=True)])  # simulation only

        ns = self.get_namespace().strip('/')
        base = self.get_parameter('base_frame').value
        self.base_frame = f'{ns}/{base}' if ns else base
        self.world_frame = self.get_parameter('world_frame').value

        # Variables
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.truth = None
        self.samples = 0
        self.sum_sq = {'lat': 0.0, 'lon': 0.0, 'yaw': 0.0}
        self.max_abs = {'lat': 0.0, 'lon': 0.0, 'yaw': 0.0}
        self.distance = 0.0
        self.last_xy = None
        self.last_print = None
        self.csv = None
        path = self.get_parameter('csv').value
        if path:
            self.csv = open(path, 'w')
            self.csv.write('t,distance_m,lateral_m,longitudinal_m,yaw_rad,speed_mps\n')
            self.get_logger().info(f'writing {path}')

        # Publishers

        # Subscribers
        self.create_subscription(Odometry, 'ground_truth/odometry', self.on_truth, 10)

        # Service Servers

        # Service Clients

        # Action Servers

        # Action Clients

        # Timers
        self.create_timer(1.0 / self.get_parameter('rate').value, self.sample)

        # On Start
        self.get_logger().info(f'estimate {self.world_frame} -> {self.base_frame} vs '
                               f'{self.get_namespace().rstrip("/")}/ground_truth/odometry')

    # Store the latest ground-truth odometry.
    def on_truth(self, msg):
        self.truth = msg

    # Compare the estimated pose with the ground truth, update the statistics, and log/print the error.
    def sample(self):
        truth = self.truth
        if truth is None:
            return
        stamp = Time.from_msg(truth.header.stamp)
        try:
            tf = self.tf_buffer.lookup_transform(self.world_frame, self.base_frame, stamp)
        except TransformException:
            try:  # TF slightly behind the ground truth: compare with the latest estimate
                tf = self.tf_buffer.lookup_transform(self.world_frame, self.base_frame, Time())
            except TransformException as error:
                self.get_logger().warn(f'no estimate yet: {error}', throttle_duration_sec=5.0)
                return

        tx, ty = truth.pose.pose.position.x, truth.pose.pose.position.y
        tyaw = yaw_of(truth.pose.pose.orientation)
        ex = tf.transform.translation.x - tx
        ey = tf.transform.translation.y - ty
        # error in the true car frame: x forward (longitudinal), y left (lateral)
        lon = math.cos(tyaw) * ex + math.sin(tyaw) * ey
        lat = -math.sin(tyaw) * ex + math.cos(tyaw) * ey
        eyaw = wrap(yaw_of(tf.transform.rotation) - tyaw)
        speed = truth.twist.twist.linear.x

        if self.last_xy is not None:
            self.distance += math.hypot(tx - self.last_xy[0], ty - self.last_xy[1])
        self.last_xy = (tx, ty)

        self.samples += 1
        for key, value in (('lat', lat), ('lon', lon), ('yaw', eyaw)):
            self.sum_sq[key] += value * value
            self.max_abs[key] = max(self.max_abs[key], abs(value))

        t = stamp.nanoseconds * 1e-9
        if self.csv:
            self.csv.write(f'{t:.3f},{self.distance:.3f},{lat:.4f},{lon:.4f},{eyaw:.5f},{speed:.3f}\n')
        if self.last_print is None or t - self.last_print >= 1.0:
            self.last_print = t
            print(f't {t:7.1f} s  driven {self.distance:6.1f} m  v {speed:4.1f}  |  '
                  f'lat {lat:+6.3f}  lon {lon:+6.3f} m  yaw {math.degrees(eyaw):+5.1f} deg  |  '
                  f'RMS lat {self.rms("lat"):.3f}  max lat {self.max_abs["lat"]:.3f} m', flush=True)

    # Return the RMS of one error component over all samples.
    def rms(self, key):
        return math.sqrt(self.sum_sq[key] / self.samples) if self.samples else 0.0

    # Print the RMS / max error summary of the run and close the CSV.
    def summary(self):
        if not self.samples:
            print('no samples (is the simulation running with ground_truth/odometry?)')
            return
        print(f'\n{self.samples} samples, {self.distance:.1f} m driven\n'
              f'  lateral       RMS {self.rms("lat"):.3f} m   max {self.max_abs["lat"]:.3f} m\n'
              f'  longitudinal  RMS {self.rms("lon"):.3f} m   max {self.max_abs["lon"]:.3f} m\n'
              f'  yaw           RMS {math.degrees(self.rms("yaw")):.2f} deg max '
              f'{math.degrees(self.max_abs["yaw"]):.2f} deg')
        if self.csv:
            self.csv.close()


# Start the node, spin until Ctrl+C, then print the error summary.
def main():
    rclpy.init()
    node = LocalizationError()
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
