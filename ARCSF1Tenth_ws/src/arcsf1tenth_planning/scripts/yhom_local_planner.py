#!/usr/bin/python3

import math
import sys

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from arcsf1tenth_interfaces.msg import FrenetState, ObstacleArray, Trajectory
from nav_msgs.msg import Path
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time

from arcsf1tenth_planning.utils.conversions import arrays_to_msg, msg_to_global_path, trajectory_to_path
from arcsf1tenth_planning.frenet.frenet_optimal import PlannerParams, plan

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
VEHICLE = ('wheelbase', 'max_steering_angle', 'max_speed', 'footprint_front', 'footprint_rear', 'footprint_width')


class YhomLocalPlanner(Node):
    # Set up the node: vehicle and planner parameters, plan state, subscribers, publishers and replanning timer.
    def __init__(self):
        super().__init__('yhom_local_planner')

        # Parameters
        for name in VEHICLE:   # from arcsf1tenth_description/config/vehicle.yaml
            self.declare_parameter(name, Parameter.Type.DOUBLE)
        try:
            v = {name: self.get_parameter(name).value for name in VEHICLE}
        except ParameterUninitializedException:
            raise RuntimeError('vehicle parameters missing: start it with yhom_controller.launch.py')

        defaults = PlannerParams()
        self.declare_parameter('rate', 10.0)                   # [Hz]
        self.declare_parameter('replan_distance', 0.3)         # [m] car further than this from the plan: replan from the car
        self.declare_parameter('obstacle_timeout', 0.5)        # [s] ignore older obstacle messages
        for name in ('horizon', 'step', 'min_maneuver_time', 'd_step', 'wall_margin', 'obstacle_margin',
                     'clearance_preferred', 'max_lateral_accel', 'avoid_speed_max',
                     'w_jerk', 'w_offset', 'w_length', 'w_consistency', 'w_clearance'):
            self.declare_parameter(name, float(getattr(defaults, name)))
        self.declare_parameter('maneuver_lengths', list(defaults.maneuver_lengths))
        p = lambda name: self.get_parameter(name).value

        self.params = PlannerParams(**{name: p(name) for name in (
            'horizon', 'step', 'min_maneuver_time', 'd_step', 'wall_margin', 'obstacle_margin',
            'clearance_preferred', 'max_lateral_accel', 'avoid_speed_max',
            'w_jerk', 'w_offset', 'w_length', 'w_consistency', 'w_clearance')})
        self.params.maneuver_lengths = tuple(p('maneuver_lengths'))
        self.params.half_width = v['footprint_width'] / 2
        self.params.car_front = v['footprint_front']
        self.params.car_rear = v['footprint_rear']
        self.params.max_curvature = 0.9 * math.tan(v['max_steering_angle']) / v['wheelbase']
        self.params.speed_max = v['max_speed']
        self.replan_distance = p('replan_distance')
        self.obstacle_timeout = p('obstacle_timeout')

        # Variables
        self.path = None
        self.frame = None
        self.state = None
        self.obstacles = None
        self.previous = None          # last PlanResult
        self.last_ok = None

        # Publishers
        self.traj_pub = self.create_publisher(Trajectory, 'yhom/local_trajectory', 10)
        self.path_pub = self.create_publisher(Path, 'yhom/local_path', 10)

        # Subscribers
        self.create_subscription(Trajectory, 'global_trajectory', self.on_path, LATCHED)
        self.create_subscription(FrenetState, 'frenet/state', self.on_state, 10)
        self.create_subscription(ObstacleArray, 'obstacles', self.on_obstacles, 10)

        # Service Servers

        # Service Clients

        # Action Servers

        # Action Clients

        # Timers
        self.create_timer(1.0 / p('rate'), self.step)

        # On Start

    # Store the global path and reset the previous plan.
    def on_path(self, msg):
        if msg.points:
            self.path = msg_to_global_path(msg)
            self.frame = msg.header.frame_id
            self.previous = None

    # Store the latest Frenet state of the car.
    def on_state(self, msg):
        self.state = msg

    # Store the latest obstacles.
    def on_obstacles(self, msg):
        self.obstacles = msg

    # Return (d, d', d'') to start the plan from: the previous plan at the car's s, or the measured state.
    def initial_state(self, s, d, yaw_error):
        prev = self.previous
        if prev is not None and prev.ok:
            rel = self.path.ds(prev.s, s)
            k = int(np.argmin(np.abs(rel)))
            if abs(rel[k]) < 0.5 and abs(prev.d[k] - d) < self.replan_distance:
                return prev.d[k], prev.dd[k], prev.ddd[k]
        kappa = float(self.path.interp(self.path.kappa, s))
        return d, (1.0 - kappa * d) * math.tan(yaw_error), 0.0

    # Plan the best Frenet path around the current obstacles and publish it (empty if none was found).
    def step(self):
        if self.path is None or self.state is None:
            return
        st = self.state
        now = self.get_clock().now()
        obstacles = []
        if self.obstacles is not None and \
                (now - Time.from_msg(self.obstacles.header.stamp)).nanoseconds * 1e-9 < self.obstacle_timeout:
            obstacles = [(o.s_start, o.s_end, o.d_min, o.d_max) for o in self.obstacles.obstacles]

        d0, dd0, ddd0 = self.initial_state(st.s, st.d, st.yaw_error)
        prev_target = self.previous.d_target if self.previous is not None and self.previous.ok else None
        result = plan(self.path, st.s, d0, dd0, ddd0, max(st.speed, 0.0), obstacles, self.params, prev_target)
        self.previous = result

        stamp = now.to_msg()
        if result.ok:
            msg = arrays_to_msg(self.frame, stamp, result.s, result.d, result.x, result.y,
                                result.psi, result.kappa, result.v)
        else:
            msg = Trajectory()
            msg.header.frame_id, msg.header.stamp = self.frame, stamp
        self.traj_pub.publish(msg)
        self.path_pub.publish(trajectory_to_path(msg))
        if result.ok != self.last_ok:
            if result.ok:
                self.get_logger().info(f'path found ({result.feasible}/{result.candidates} candidates feasible)')
            else:
                self.get_logger().warn(f'no path: {result.reason}')
            self.last_ok = result.ok


# Start the node and spin until Ctrl+C.
def main():
    rclpy.init()
    try:
        node = YhomLocalPlanner()
    except RuntimeError as error:
        print(f'[yhom_local_planner] {error}', file=sys.stderr)
        rclpy.try_shutdown()
        sys.exit(1)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
