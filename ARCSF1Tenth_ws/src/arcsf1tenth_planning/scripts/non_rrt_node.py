#!/usr/bin/python3

import math
import sys

import numpy as np
import rclpy
from arcsf1tenth_interfaces.msg import Waypoint
from arcsf1tenth_interfaces.srv import PlanAvoidance
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from rclpy.exceptions import ParameterUninitializedException
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker

from arcsf1tenth_planning.rrt.avoidance import AvoidanceParams, AvoidancePlanner
from arcsf1tenth_planning.rrt.collision import CollisionChecker
from arcsf1tenth_planning.rrt.global_path import GlobalPath
from arcsf1tenth_planning.rrt.kinodynamic_rrt import RRTParams
from arcsf1tenth_planning.utils.conversions import namespaced_frame

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
VEHICLE = ('wheelbase', 'max_steering_angle', 'max_steering_speed', 'max_speed', 'max_acceleration',
           'footprint_front', 'footprint_rear', 'footprint_width')
RRT_FIELDS = ('edge_length', 'integration_step', 'n_controls', 'goal_bias', 'goal_tolerance', 'goal_depth',
              'goal_yaw_tolerance', 'w_heading', 'w_steer', 'max_fails', 'max_nodes', 'max_iterations',
              'refine_iterations', 'time_budget')
AVOID_FIELDS = ('goal_after_obstacle', 'min_goal_distance', 'max_goal_distance', 'sample_width', 'merge_length',
                'avoid_speed_max', 'min_plan_speed', 'max_lateral_accel', 'obstacle_margin', 'escape_margin')
REQUEST_LATENCY = 0.005     # [s] service round trip (measured ~3 ms)


class NonRRTNode(Node):
    # Set up the node: vehicle, collision and RRT parameters, the planner, the service and the markers.
    def __init__(self):
        super().__init__('non_rrt_node')

        # Parameters
        for name in VEHICLE:   # arcsf1tenth_description/config/vehicle.yaml
            self.declare_parameter(name, Parameter.Type.DOUBLE)
        try:
            self.vehicle = {name: self.get_parameter(name).value for name in VEHICLE}
        except ParameterUninitializedException:
            raise RuntimeError('vehicle parameters missing: start it with non_controller.launch.py')
        declare = self.declare_parameter
        declare('global_path', '')                 # CSV (launch file)
        declare('map_frame', namespaced_frame(self, 'map'))
        declare('rate', 10.0)                      # [Hz] non_state_machine rate (data age for the lead-in)
        declare('wall_margin', 0.05)               # [m] car edge to map wall
        defaults = AvoidanceParams()
        for name in AVOID_FIELDS:
            declare(name, getattr(defaults, name))
        declare('steer_margin', 0.9)               # usable fraction of max_steering_angle
        declare('steer_rate_margin', 0.7)          # usable fraction of max_steering_speed (tracker headroom)
        rrt_defaults = RRTParams()
        for name in RRT_FIELDS:
            declare(name, getattr(rrt_defaults, name))
        declare('publish_tree', True)
        p = lambda name: self.get_parameter(name).value
        self.p = p
        csv = p('global_path')
        if not csv:
            raise RuntimeError('no global_path: start it with non_controller.launch.py track:=<map_name>')
        self.map_frame = p('map_frame')

        # Variables
        v = self.vehicle
        path = GlobalPath.from_csv(csv)
        self.checker = CollisionChecker(v['footprint_width'] / 2, v['footprint_front'], v['footprint_rear'],
                                        p('wall_margin'), p('obstacle_margin'))
        rrt = RRTParams(**{name: p(name) for name in RRT_FIELDS})
        rrt.wheelbase = v['wheelbase']
        rrt.max_steer = p('steer_margin') * v['max_steering_angle']
        rrt.steer_rate = p('steer_rate_margin') * v['max_steering_speed']
        params = AvoidanceParams(**{name: p(name) for name in AVOID_FIELDS},
                                 max_speed=v['max_speed'], max_acceleration=v['max_acceleration'],
                                 lead_time=0.5 / p('rate') + REQUEST_LATENCY)
        self.planner = AvoidancePlanner(path, self.checker, rrt, params)

        # Publishers
        self.tree_pub = self.create_publisher(Marker, 'rrt/tree', 10)
        self.goal_pub = self.create_publisher(Marker, 'rrt/goal', 10)

        # Subscribers
        self.create_subscription(OccupancyGrid, 'map', self.on_map, LATCHED)

        # Service Servers
        self.create_service(PlanAvoidance, 'rrt/plan', self.on_plan)

        # Service Clients

        # Action Servers

        # Action Clients

        # Timers

        # On Start
        self.get_logger().info(f'global path {csv} ({path.length:.1f} m), waiting for the map')

    # Load the map walls into the collision checker.
    def on_map(self, msg):
        grid = np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
        self.checker.set_map(grid, msg.info.resolution, msg.info.origin.position.x, msg.info.origin.position.y)
        self.get_logger().info(f'map {msg.info.width} x {msg.info.height}: wall distance ready')

    # Plan a detour for the requested car state around the requested obstacle points.
    def on_plan(self, req, res):
        if not self.checker.ready:
            res.ok, res.reason = False, 'no map yet'
            return res
        self.checker.set_obstacles(np.column_stack([np.asarray(req.obstacle_x), np.asarray(req.obstacle_y)]))
        av = self.planner.plan(req.x, req.y, req.yaw, req.speed, req.yaw_rate, req.path_index, req.blocked)
        r = av.rrt
        res.ok, res.reason, res.margin = av.ok, av.reason, float(av.margin)
        res.plan_time, res.nodes, res.iterations = float(r.plan_time), int(r.nodes), int(r.iterations)
        res.trajectory.header.frame_id = self.map_frame
        res.trajectory.header.stamp = req.header.stamp
        res.trajectory.closed = False
        if av.ok:
            pl = av.path
            res.end_index = int(pl['end'])
            res.trajectory.points = [
                Waypoint(s=float(pl['s'][k]), d=float(pl['d'][k]), x=float(pl['x'][k]), y=float(pl['y'][k]),
                         psi=float(pl['yaw'][k]), kappa=float(pl['kappa'][k]), v=float(pl['v'][k]))
                for k in range(len(pl['x']))]
            if av.margin < self.p('obstacle_margin'):
                self.get_logger().warn(f'RRT: escape path with obstacle margin {av.margin:.2f} m')
            self.get_logger().info(f'RRT: path found, {r.nodes} nodes, {r.iterations} iterations, '
                                   f'{1000 * r.plan_time:.0f} ms, cost {r.cost:.2f}')
        else:
            self.get_logger().warn(f'RRT: no path ({av.reason}, {1000 * r.plan_time:.0f} ms)',
                                   throttle_duration_sec=1.0)
        self.publish_markers(r, av.goal)
        return res

    # Create an empty RViz marker in the map frame with the given type, colour and scale.
    def marker(self, ns, kind, color, scale):
        m = Marker()
        m.header.frame_id, m.header.stamp = self.map_frame, self.get_clock().now().to_msg()
        m.ns, m.id, m.type = ns, 0, kind
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = scale
        m.color.r, m.color.g, m.color.b = color
        m.color.a = 1.0
        return m

    # Publish the RRT tree and the goal as RViz markers.
    def publish_markers(self, result, goal):
        if not self.p('publish_tree') or result is None:
            return
        m = self.marker('rrt_tree', Marker.LINE_LIST, (0.6, 0.6, 0.6), 0.01)
        points = []     # build the list once: m.points += ... re-validates the whole list every time
        for xs, ys, x0, y0 in result.edges:
            lx = np.r_[x0, xs[2::3], xs[-1]]
            ly = np.r_[y0, ys[2::3], ys[-1]]
            seg = np.repeat(np.arange(len(lx)), 2)[1:-1]     # 0,1, 1,2, 2,3, ... = consecutive segments
            points.extend(Point(x=a, y=b, z=0.02) for a, b in zip(lx[seg].tolist(), ly[seg].tolist()))
        m.points = points
        self.tree_pub.publish(m)
        if goal is not None:
            g = self.marker('rrt_goal', Marker.ARROW, (0.0, 0.6, 1.0), 0.1)
            g.scale.x, g.scale.y, g.scale.z = 0.6, 0.08, 0.08
            gx, gy, gyaw = goal
            g.pose.position.x, g.pose.position.y = float(gx), float(gy)
            g.pose.orientation.z, g.pose.orientation.w = math.sin(gyaw / 2), math.cos(gyaw / 2)
            self.goal_pub.publish(g)


# Start the node and spin until Ctrl+C.
def main():
    rclpy.init()
    try:
        node = NonRRTNode()
    except (RuntimeError, OSError) as error:
        print(f'[non_rrt_node] {error}', file=sys.stderr)
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
