#!/usr/bin/python3
import math
import sys

import numpy as np
import rclpy
from arcsf1tenth_interfaces.msg import Trajectory, Waypoint
from arcsf1tenth_interfaces.srv import PlanAvoidance
from nav_msgs.msg import Odometry, Path
from rclpy.duration import Duration
from rclpy.exceptions import ParameterUninitializedException
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker

from arcsf1tenth_planning.rrt.collision import CollisionChecker
from arcsf1tenth_planning.rrt.global_path import GlobalPath
from arcsf1tenth_planning.utils.conversions import namespaced_frame, trajectory_to_path

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
VEHICLE = ('max_speed', 'footprint_front', 'footprint_rear', 'footprint_width')
COLORS = {'GLOBAL': (0.2, 1.0, 0.2), 'AVOID': (1.0, 0.7, 0.0), 'BLOCKED': (1.0, 0.1, 0.1)}


# Return the yaw angle of a quaternion.
def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class NonStateMachine(Node):
    # Set up the node: vehicle and mode parameters, state, publishers, subscribers, the RRT client and timer.
    def __init__(self):
        super().__init__('non_state_machine')

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
        declare('base_frame', namespaced_frame(self, 'base_footprint'))
        declare('odom_topic', 'odometry/estimated')
        declare('rate', 10.0)                      # [Hz]
        declare('obstacle_timeout', 0.5)           # [s] older obstacle points = no obstacle information
        declare('obstacle_margin', 0.3)            # [m] car edge to obstacle point
        declare('check_distance', 3.0)             # [m] react to obstacles this far ahead ...
        declare('check_time', 1.5)                 # [s] ... plus speed * this
        declare('replan_distance', 0.4)            # [m] car further than this from its RRT path: drop it
        declare('stop_distance', 0.5)              # [m] BLOCKED: rear axle stops this far before the blocked pose
        declare('brake_decel', 2.0)                # [m/s^2] BLOCKED braking profile
        declare('global_ahead', 12.0)              # [m] global path window sent to the controller
        declare('global_behind', 1.0)              # [m]
        declare('request_timeout', 1.0)            # [s] forget an RRT request with no reply after this
        self.p = lambda name: self.get_parameter(name).value
        p = self.p
        csv = p('global_path')
        if not csv:
            raise RuntimeError('no global_path: start it with non_controller.launch.py track:=<map_name>')
        self.map_frame, self.base_frame = p('map_frame'), p('base_frame')

        # Variables
        self.path = GlobalPath.from_csv(csv)
        v = self.vehicle
        self.checker = CollisionChecker(v['footprint_width'] / 2, v['footprint_front'], v['footprint_rear'],
                                        0.0, p('obstacle_margin'))    # obstacles only: no map needed here
        self.points = None            # (stamp, (N, 2) array) from non_obstacle_detector
        self.speed = 0.0
        self.yaw_rate = 0.0
        self.index = None             # nearest global path point last cycle
        self.plan = None              # committed RRT path (dict of arrays)
        self.mode = None
        self.pending = None           # outstanding RRT request: dict(id, sent, plan_blocked)
        self.request_id = 0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers
        self.traj_pub = self.create_publisher(Trajectory, 'rrt/trajectory', 10)
        self.path_pub = self.create_publisher(Path, 'rrt/path', 10)
        self.state_pub = self.create_publisher(String, 'rrt/state', LATCHED)
        self.state_marker_pub = self.create_publisher(Marker, 'rrt/state_marker', 10)

        # Subscribers
        self.create_subscription(PointCloud2, 'rrt/obstacle_points', self.on_points, 10)
        self.create_subscription(Odometry, p('odom_topic'), self.on_odom, 10)

        # Service Servers

        # Service Clients
        self.rrt_client = self.create_client(PlanAvoidance, 'rrt/plan')

        # Action Servers

        # Action Clients

        # Timers
        self.create_timer(1.0 / p('rate'), self.step)

        # On Start
        self.get_logger().info(f'global path {csv} ({self.path.length:.1f} m)')

    # Store the latest obstacle points.
    def on_points(self, msg):
        xy = point_cloud2.read_points_numpy(msg, field_names=('x', 'y')) if msg.width else np.zeros((0, 2))
        self.points = (Time.from_msg(msg.header.stamp), np.asarray(xy, dtype=float).reshape(-1, 2))

    # Store the latest speed and yaw rate.
    def on_odom(self, msg):
        self.speed = msg.twist.twist.linear.x
        self.yaw_rate = msg.twist.twist.angular.z

    # Return the car pose (x, y, yaw) in the map frame, or None if missing or stale.
    def car_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
        except TransformException as error:
            self.get_logger().warn(f'{error}', throttle_duration_sec=5.0)
            return None
        if self.get_clock().now() - Time.from_msg(t.header.stamp) > Duration(seconds=0.5):
            return None    # localization stopped: do not drive on a stale pose
        return t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation)

    # Give the collision checker the current obstacle points (none if they are missing or stale).
    def update_obstacles(self, now):
        if self.points is None or (now - self.points[0]).nanoseconds * 1e-9 > self.p('obstacle_timeout'):
            self.checker.set_obstacles([])
        else:
            self.checker.set_obstacles(self.points[1])

    # One cycle: choose GLOBAL / AVOID / BLOCKED, request a detour if needed, and publish.
    def step(self):
        now = self.get_clock().now()
        pose = self.car_pose()
        if pose is None:
            return       # publish nothing: the controller stops on a stale trajectory
        x, y, yaw = pose
        v = max(self.speed, 0.0)
        path = self.path
        hint = self.index
        if hint is not None and math.hypot(path.x[hint] - x, path.y[hint] - y) > 2.0:
            hint = None  # lost (relocalized): search the whole path
        i = self.index = path.nearest(x, y, hint)
        self.update_obstacles(now)

        blocked = self.first_blocked(i, v)
        plan_blocked = None           # index of the first blocked pose of the committed plan
        if self.plan is not None:
            status, plan_blocked = self.check_plan(x, y)
            if status in ('off', 'done'):
                self.plan = None

        stamp = now.to_msg()
        if self.plan is not None and plan_blocked is None:
            mode = 'AVOID'
            msg = self.plan_msg(stamp)
        elif self.plan is None and blocked is None:
            mode = 'GLOBAL'
            msg = self.global_msg(i, stamp, None)
        else:
            # global path blocked, or the committed plan is: ask for a new RRT plan, and brake meanwhile
            # (far from the obstacle the braking profile does not slow the car down yet)
            self.request(x, y, yaw, v, i, blocked if blocked is not None else 0, plan_blocked, now)
            mode = self.mode if self.pending is not None and self.mode is not None else 'BLOCKED'
            if self.plan is not None:
                # mid-maneuver: stay on the old RRT path, stop before its blocked pose (the global
                # path would steer back toward the obstacle)
                msg = self.plan_msg(stamp, stop_at=plan_blocked)
            else:
                msg = self.global_msg(i, stamp, blocked)
        self.publish(msg)
        self.set_mode(mode)
        self.publish_state_marker(stamp, x, y)

    # Send an RRT request unless one is still waiting for its reply.
    def request(self, x, y, yaw, v, i, blocked, plan_blocked, now):
        if self.pending is not None:
            if (now - self.pending['sent']).nanoseconds * 1e-9 < self.p('request_timeout'):
                return
            self.get_logger().warn('RRT: no reply from non_rrt_node, asking again')
            self.pending = None
        if not self.rrt_client.service_is_ready():
            self.get_logger().warn('RRT: waiting for the rrt/plan service (non_rrt_node)', throttle_duration_sec=5.0)
            return
        req = PlanAvoidance.Request()
        req.header.frame_id, req.header.stamp = self.map_frame, now.to_msg()
        req.x, req.y, req.yaw, req.speed, req.yaw_rate = float(x), float(y), float(yaw), float(v), float(self.yaw_rate)
        req.path_index, req.blocked = int(i), int(blocked)
        pts = self.checker.points
        req.obstacle_x, req.obstacle_y = pts[:, 0].tolist(), pts[:, 1].tolist()
        self.request_id += 1
        self.pending = dict(id=self.request_id, sent=now, plan_blocked=plan_blocked)
        future = self.rrt_client.call_async(req)
        future.add_done_callback(lambda f, rid=self.request_id: self.on_reply(f, rid))

    # Commit to the RRT reply (AVOID, published right away) or stay BLOCKED.
    def on_reply(self, future, rid):
        if self.pending is None or self.pending['id'] != rid:
            return       # an old request that timed out
        self.pending = None
        res = future.result()
        if res is None or not res.ok:
            self.set_mode('BLOCKED')
            return
        pts = res.trajectory.points
        self.plan = dict(x=np.array([q.x for q in pts]), y=np.array([q.y for q in pts]),
                         yaw=np.array([q.psi for q in pts]), kappa=np.array([q.kappa for q in pts]),
                         v=np.array([q.v for q in pts]), s=np.array([q.s for q in pts]),
                         d=np.array([q.d for q in pts]), end=int(res.end_index), j=0, margin=res.margin)
        stamp = self.get_clock().now().to_msg()
        self.publish(self.plan_msg(stamp))
        self.set_mode('AVOID')

    # Return the number of path points from the car to the first global path pose hitting an obstacle, or None.
    def first_blocked(self, i, v):
        path = self.path
        reach = self.p('check_distance') + v * self.p('check_time')
        idx = path.window(i, 0.0, reach)
        free = self.checker.obstacle_free(path.x[idx], path.y[idx], path.psi[idx])
        return None if free.all() else int(np.argmin(free))

    # Check the committed RRT path: return ('ok' | 'off' | 'done' | 'blocked', index of its first blocked pose or None).
    def check_plan(self, x, y):
        pl = self.plan
        dist = np.hypot(pl['x'] - x, pl['y'] - y)
        j = int(np.argmin(dist))
        if dist[j] > self.p('replan_distance'):
            self.get_logger().warn(f'{dist[j]:.2f} m off the RRT path: dropping it')
            return 'off', None
        if j >= pl['end']:
            self.get_logger().info('RRT path done: back on the global path')
            return 'done', None
        pl['j'] = j
        free = self.checker.obstacle_free(pl['x'][j:], pl['y'][j:], pl['yaw'][j:], pl['margin'])
        if not free.all():
            if not pl.get('blocked'):
                self.get_logger().warn('RRT path blocked by a new obstacle: replanning')
            pl['blocked'] = True
            return 'blocked', j + int(np.argmin(free))
        return 'ok', None

    # Build the committed RRT path from the car on; with stop_at (a blocked pose index), brake to a stop before it.
    def plan_msg(self, stamp, stop_at=None):
        pl = self.plan
        v = pl['v']
        if stop_at is not None:
            arc = np.r_[0.0, np.cumsum(np.hypot(np.diff(pl['x']), np.diff(pl['y'])))]
            to_stop = arc[stop_at] - arc - self.p('stop_distance')
            v = np.minimum(v, np.sqrt(2.0 * self.p('brake_decel') * np.maximum(to_stop, 0.0)))
        msg = Trajectory()
        msg.header.frame_id, msg.header.stamp = self.map_frame, stamp
        msg.closed = False
        for k in range(max(pl['j'] - 5, 0), len(pl['x'])):
            msg.points.append(Waypoint(s=float(pl['s'][k]), d=float(pl['d'][k]), x=float(pl['x'][k]),
                                       y=float(pl['y'][k]), psi=float(pl['yaw'][k]), kappa=float(pl['kappa'][k]),
                                       v=float(v[k])))
        return msg

    # Build the global path window around the car; with blocked (points to the blocked pose), brake to a stop before it.
    def global_msg(self, i, stamp, blocked):
        path = self.path
        behind = path.count(self.p('global_behind'))
        idx = path.window(i, self.p('global_behind'), self.p('global_ahead'))
        v = np.minimum(path.v[idx], self.vehicle['max_speed'])
        if blocked is not None:
            to_stop = (blocked - (np.arange(len(idx)) - behind)) * path.step - self.p('stop_distance')
            v = np.minimum(v, np.sqrt(2.0 * self.p('brake_decel') * np.maximum(to_stop, 0.0)))
        msg = Trajectory()
        msg.header.frame_id, msg.header.stamp = self.map_frame, stamp
        msg.closed = False
        for k, j in enumerate(idx):
            msg.points.append(Waypoint(s=float(path.s[j]), d=0.0, x=float(path.x[j]), y=float(path.y[j]),
                                       psi=float(path.psi[j]), kappa=float(path.kappa[j]), v=float(v[k])))
        return msg

    # Publish a trajectory for the controller and as an RViz path.
    def publish(self, msg):
        self.traj_pub.publish(msg)
        self.path_pub.publish(trajectory_to_path(msg))

    # Change the mode, log the transition, and publish the new state.
    def set_mode(self, mode):
        if mode != self.mode:
            self.get_logger().info(f'{self.mode} -> {mode}')
            self.mode = mode
            self.state_pub.publish(String(data=mode))

    # Publish the mode as an RViz text marker above the car.
    def publish_state_marker(self, stamp, x, y):
        m = Marker()
        m.header.frame_id, m.header.stamp = self.map_frame, stamp
        m.ns, m.id, m.type = 'rrt_state', 0, Marker.TEXT_VIEW_FACING
        m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), 1.0
        m.pose.orientation.w = 1.0
        m.scale.z = 0.5
        m.color.r, m.color.g, m.color.b = COLORS[self.mode]
        m.color.a = 1.0
        m.text = self.mode
        self.state_marker_pub.publish(m)


# Start the node and spin until Ctrl+C.
def main():
    rclpy.init()
    try:
        node = NonStateMachine()
    except (RuntimeError, OSError) as error:
        print(f'[non_state_machine] {error}', file=sys.stderr)
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
