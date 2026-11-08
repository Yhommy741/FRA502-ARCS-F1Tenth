#!/usr/bin/python3

import math
import sys

import numpy as np
import rclpy
from arcsf1tenth_interfaces.msg import Trajectory, Waypoint
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.duration import Duration
from rclpy.exceptions import ParameterUninitializedException
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from scipy.spatial import cKDTree
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker

from arcsf1tenth_planning.rrt.collision import CollisionChecker
from arcsf1tenth_planning.rrt.global_path import GlobalPath
from arcsf1tenth_planning.rrt.kinodynamic_rrt import RRTParams, plan, propagate, speed_profile

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
VEHICLE = ('wheelbase', 'max_steering_angle', 'max_steering_speed', 'max_speed', 'max_acceleration',
           'footprint_front', 'footprint_rear', 'footprint_width')
COLORS = {'GLOBAL': (0.2, 1.0, 0.2), 'AVOID': (1.0, 0.7, 0.0), 'BLOCKED': (1.0, 0.1, 0.1)}
RRT_FIELDS = ('edge_length', 'integration_step', 'n_controls', 'goal_bias', 'goal_tolerance', 'goal_depth',
              'goal_yaw_tolerance', 'w_heading', 'w_steer', 'max_fails', 'max_nodes', 'max_iterations',
              'refine_iterations', 'time_budget')


# Return the yaw angle of a quaternion.
def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


# Return the TF frame in the node's namespace (/arcsf1tenth + map -> arcsf1tenth/map, no namespace -> map).
def namespaced_frame(node, name):
    namespace = node.get_namespace().strip('/')
    return f'{namespace}/{name}' if namespace else name


class RRTPlanner(Node):
    # Set up the node: vehicle, detection and RRT parameters, planner state, subscribers, publishers and timer.
    def __init__(self):
        super().__init__('non_rrt_planner')

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
        # obstacle detection
        declare('detect_wall_margin', 0.3)         # [m] lidar points closer than this to a map wall = wall ...
        declare('detect_range_gain', 0.035)        # [m/m] ... plus this * range (0.035 = 2 deg heading error)
        declare('max_range', 6.0)                  # [m]
        declare('neighbor_radius', 0.2)            # [m] a point needs min_neighbors others this close
        declare('min_neighbors', 2)
        declare('obstacle_timeout', 0.5)           # [s] older scan = no obstacle information
        # collision check
        declare('wall_margin', 0.05)               # [m] car edge to map wall
        declare('obstacle_margin', 0.2)            # [m] car edge to obstacle point
        declare('escape_margin', 0.05)             # [m] obstacle margin when the car is already inside obstacle_margin
        # trigger / goal / sampling
        declare('check_distance', 3.0)             # [m] react to obstacles this far ahead ...
        declare('check_time', 1.5)                 # [s] ... plus speed * this
        declare('goal_after_obstacle', 2.0)        # [m] goal on the global path this far past the obstacle
        declare('min_goal_distance', 4.0)          # [m] and at least this far ahead of the car
        declare('max_goal_distance', 10.0)         # [m] goal pushed forward while blocked, up to this
        declare('sample_width', 1.2)               # [m] samples within +- this of the global path
        declare('steer_margin', 0.9)               # usable fraction of max_steering_angle
        declare('steer_rate_margin', 0.7)          # usable fraction of max_steering_speed (tracker headroom)
        # RRT (arcsf1tenth_planning.rrt.kinodynamic_rrt.RRTParams)
        defaults = RRTParams()
        for name in RRT_FIELDS:
            declare(name, getattr(defaults, name))
        # following the plan
        declare('replan_distance', 0.4)            # [m] car further than this from its RRT path: drop it
        declare('merge_length', 3.0)               # [m] global path appended after the RRT goal
        declare('avoid_speed_max', 2.0)            # [m/s] on the RRT path
        declare('min_plan_speed', 1.0)             # [m/s] steering-rate limit is computed for at least this speed
        declare('max_lateral_accel', 4.0)          # [m/s^2]
        declare('stop_distance', 0.5)              # [m] BLOCKED: rear axle stops this far before the blocked pose
        declare('brake_decel', 2.0)                # [m/s^2] BLOCKED braking profile
        declare('global_ahead', 12.0)              # [m] global path window sent to the tracker
        declare('global_behind', 1.0)              # [m]
        declare('publish_tree', True)
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
                                        p('wall_margin'), p('obstacle_margin'))
        self.rrt = RRTParams(**{name: p(name) for name in RRT_FIELDS})
        self.rrt.wheelbase = v['wheelbase']
        self.rrt.max_steer = p('steer_margin') * v['max_steering_angle']
        self.rrt.steer_rate = p('steer_rate_margin') * v['max_steering_speed']
        self.rng = np.random.default_rng()
        self.scan = None
        self.speed = 0.0
        self.yaw_rate = 0.0
        self.index = None             # nearest global path point last cycle
        self.plan = None              # committed RRT path (dict of arrays)
        self.mode = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers
        self.traj_pub = self.create_publisher(Trajectory, 'rrt/trajectory', 10)
        self.path_pub = self.create_publisher(Path, 'rrt/path', 10)
        self.tree_pub = self.create_publisher(Marker, 'rrt/tree', 10)
        self.goal_pub = self.create_publisher(Marker, 'rrt/goal', 10)
        self.obstacle_pub = self.create_publisher(Marker, 'rrt/obstacle_points', 10)
        self.state_pub = self.create_publisher(String, 'rrt/state', LATCHED)
        self.state_marker_pub = self.create_publisher(Marker, 'rrt/state_marker', 10)

        # Subscribers
        self.create_subscription(OccupancyGrid, 'map', self.on_map, LATCHED)
        self.create_subscription(LaserScan, 'scan', lambda m: setattr(self, 'scan', m), qos_profile_sensor_data)
        self.create_subscription(Odometry, p('odom_topic'), self.on_odom, 10)

        # Service Servers

        # Service Clients

        # Action Servers

        # Action Clients

        # Timers
        self.create_timer(1.0 / p('rate'), self.step)

        # On Start
        self.get_logger().info(f'global path {csv} ({self.path.length:.1f} m), waiting for the map')

    # Load the map walls into the collision checker.
    def on_map(self, msg):
        grid = np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
        self.checker.set_map(grid, msg.info.resolution, msg.info.origin.position.x, msg.info.origin.position.y)
        self.get_logger().info(f'map {msg.info.width} x {msg.info.height}: wall distance ready')

    # Store the latest speed and yaw rate.
    def on_odom(self, msg):
        self.speed = msg.twist.twist.linear.x
        self.yaw_rate = msg.twist.twist.angular.z

    # Return the car pose (x, y, yaw) in the map frame from TF, or None if missing or stale.
    def car_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
        except TransformException as error:
            self.get_logger().warn(f'{error}', throttle_duration_sec=5.0)
            return None
        if self.get_clock().now() - Time.from_msg(t.header.stamp) > Duration(seconds=0.5):
            return None    # localization stopped: do not plan on a stale pose
        return t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation)

    # Give the collision checker the lidar points that the map does not explain (the obstacles).
    def detect_obstacles(self, now):
        scan = self.scan
        if scan is None or (now - Time.from_msg(scan.header.stamp)).nanoseconds * 1e-9 > self.p('obstacle_timeout'):
            self.checker.set_obstacles([])
            return
        # pose AT THE SCAN TIME: in a corner the car turns ~2 rad/s, so the latest pose (even
        # 50 ms newer) rotates the scan by several degrees and far walls look like obstacles
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, scan.header.frame_id,
                                                 Time.from_msg(scan.header.stamp))
        except TransformException:
            try:     # TF not there yet for that stamp (localization lags): latest, warn
                tf = self.tf_buffer.lookup_transform(self.map_frame, scan.header.frame_id, Time())
                self.get_logger().warn('no TF at the scan time: using the latest pose for obstacle detection',
                                       throttle_duration_sec=5.0)
            except TransformException:
                self.checker.set_obstacles([])
                return
        r = np.asarray(scan.ranges)
        a = scan.angle_min + np.arange(len(r)) * scan.angle_increment
        ok = np.isfinite(r) & (r >= scan.range_min) & (r <= min(scan.range_max, self.p('max_range')))
        r, a = r[ok], a[ok]
        yaw = yaw_of(tf.transform.rotation)
        lx, ly = r * np.cos(a), r * np.sin(a)
        x = tf.transform.translation.x + math.cos(yaw) * lx - math.sin(yaw) * ly
        y = tf.transform.translation.y + math.sin(yaw) * lx + math.cos(yaw) * ly
        # a heading error of the localization moves far points more: tolerance grows with range
        margin = self.p('detect_wall_margin') + self.p('detect_range_gain') * r
        keep = self.checker.unexplained_points(x, y, margin)
        pts = np.column_stack([x[keep], y[keep]])
        if len(pts):
            counts = cKDTree(pts).query_ball_point(pts, self.p('neighbor_radius'), return_length=True)
            pts = pts[counts > self.p('min_neighbors')]      # count includes the point itself
        self.checker.set_obstacles(pts)

    # One planning cycle: detect obstacles, choose GLOBAL / AVOID / BLOCKED, replan if needed, and publish.
    def step(self):
        now = self.get_clock().now()
        if not self.checker.ready:
            return
        pose = self.car_pose()
        if pose is None:
            return       # publish nothing: non_rrt_tracker stops on a stale trajectory
        x, y, yaw = pose
        v = max(self.speed, 0.0)
        path = self.path
        hint = self.index
        if hint is not None and math.hypot(path.x[hint] - x, path.y[hint] - y) > 2.0:
            hint = None  # lost (relocalized): search the whole path
        i = self.index = path.nearest(x, y, hint)
        self.detect_obstacles(now)

        blocked = self.first_blocked(i, v)
        plan_blocked = None           # index of the first blocked pose of the committed plan
        if self.plan is not None:
            status, plan_blocked = self.check_plan(x, y)
            if status in ('off', 'done'):
                self.plan = None

        stamp = now.to_msg()
        tree = None
        if self.plan is not None and plan_blocked is None:
            mode = 'AVOID'
        elif self.plan is None and blocked is None:
            mode = 'GLOBAL'
        else:
            # global path blocked, or the committed plan is: new RRT plan
            result = self.replan(x, y, yaw, v, i, blocked if blocked is not None else 0)
            tree = result.edges
            if result.ok:
                mode, plan_blocked = 'AVOID', None
            else:
                mode = 'BLOCKED'

        if mode == 'AVOID':
            msg = self.plan_msg(stamp)
        elif mode == 'BLOCKED' and self.plan is not None:
            # mid-maneuver: stay on the old RRT path, stop before its blocked pose (the global
            # path would steer back toward the obstacle)
            msg = self.plan_msg(stamp, stop_at=plan_blocked)
        else:
            msg = self.global_msg(i, stamp, blocked if mode == 'BLOCKED' else None)
        self.traj_pub.publish(msg)
        self.publish_path(msg)
        self.set_mode(mode)
        self.publish_markers(stamp, x, y, tree)

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

    # Run the kinodynamic RRT from the predicted car pose to a clear goal on the global path past the obstacle.
    def replan(self, x, y, yaw, v, i, blocked):
        p, path, rrt = self.p, self.path, self.rrt
        rrt.speed = min(max(v, p('min_plan_speed')), p('avoid_speed_max'))

        # start where the car will be when the plan is ready (it keeps driving meanwhile)
        delta0 = math.atan(rrt.wheelbase * self.yaw_rate / v) if v > 0.3 else 0.0
        delta0 = max(-rrt.max_steer, min(rrt.max_steer, delta0))
        lead = v * (rrt.time_budget + 0.5 / p('rate'))
        if lead > 2 * rrt.integration_step:
            px, py, pyaw, pd = (a[0] for a in propagate(x, y, yaw, delta0, [delta0], rrt, length=lead))
            px, py, pyaw, pd = np.r_[x, px], np.r_[y, py], np.r_[yaw, pyaw], np.r_[delta0, pd]
        else:
            px, py, pyaw, pd = np.array([x]), np.array([y]), np.array([yaw]), np.array([delta0])
        start = (px[-1], py[-1], pyaw[-1], pd[-1])

        # goal: on the global path past the obstacle, with a clear runway: no obstacle within
        # goal_after_obstacle before it or merge_length after it (obstacles close together, e.g. a
        # slalom, get one plan around all of them instead of a merge into the next one)
        before, after = path.count(p('goal_after_obstacle')), path.count(p('merge_length'))
        k_max = path.count(p('max_goal_distance'))
        idx = path.window(i, 0.0, p('max_goal_distance') + p('merge_length'))
        hit = ~self.checker.obstacle_free(path.x[idx], path.y[idx], path.psi[idx])
        k = max(blocked + before, path.count(p('min_goal_distance')))
        while k < k_max:
            g = idx[k]
            if not hit[max(k - before, 0):k + after + 1].any() and \
                    self.checker.free(path.x[[g]], path.y[[g]], path.psi[[g]])[0]:
                break
            k += path.count(0.25)
        k = min(k, k_max)
        g = idx[k]
        goal = (path.x[g], path.y[g], path.psi[g])
        self.goal = goal

        k_end = k + path.count(1.0)
        width = p('sample_width')

        # Sample a random point near the global path between the car and the goal.
        def sample(rng):
            j = (i + int(rng.integers(0, k_end))) % path.n
            return path.offset_point(j, rng.uniform(-width, width))

        margin = p('obstacle_margin')
        result = plan(start, goal, sample, self.checker.free, rrt, self.rng)
        if not result.ok and result.reason in ('start in collision', 'start boxed in'):
            # the car is inside the safety margin (stopped close to an obstacle, or an obstacle
            # appeared next to it): plan with the smaller escape margin, or it never moves again
            margin = p('escape_margin')
            self.checker.obstacle_margin = margin
            try:
                result = plan(start, goal, sample, self.checker.free, rrt, self.rng)
            finally:
                self.checker.obstacle_margin = p('obstacle_margin')
            if result.ok:
                self.get_logger().warn(f'RRT: escape path with obstacle margin {margin:.2f} m')
        if not result.ok:
            self.get_logger().warn(f'RRT: no path ({result.reason}, {1000 * result.plan_time:.0f} ms)',
                                   throttle_duration_sec=1.0)
            return result
        self.get_logger().info(f'RRT: path found, {result.nodes} nodes, {result.iterations} iterations, '
                               f'{1000 * result.plan_time:.0f} ms, cost {result.cost:.2f}')

        # committed path = lead-in + RRT path + merge back onto the global path: the goal gate lets
        # the RRT end up to goal_tolerance beside it, so the merge blends that offset to 0
        # (smoothstep over merge_length) instead of jumping
        gi = path.nearest(result.x[-1], result.y[-1], g)
        merge = path.window(gi, 0.0, p('merge_length'))[1:]
        nx, ny = -np.sin(path.psi[gi]), np.cos(path.psi[gi])
        d_end = (result.x[-1] - path.x[gi]) * nx + (result.y[-1] - path.y[gi]) * ny
        u = np.arange(1, len(merge) + 1) / len(merge)
        blend = d_end * (1.0 - u * u * (3.0 - 2.0 * u))
        mx, my = path.offset_point(merge, blend)
        xs = np.r_[px[:-1], result.x, mx]
        ys = np.r_[py[:-1], result.y, my]
        yaws = np.r_[pyaw[:-1], result.yaw, path.psi[merge]]
        kappa = np.r_[np.tan(pd[:-1]) / rrt.wheelbase, np.tan(result.delta) / rrt.wheelbase, path.kappa[merge]]
        if len(merge) > 2:     # merge heading / curvature from its geometry
            hx, hy = np.r_[result.x[-1], mx], np.r_[result.y[-1], my]
            heading = np.unwrap(np.arctan2(np.diff(hy), np.diff(hx)))
            yaws[-len(merge):] = heading
            kappa[-len(merge):] = np.gradient(heading) / np.maximum(np.hypot(np.diff(hx), np.diff(hy)), 1e-6)
        s, d, nearest = path.project(xs, ys, i, window=p('max_goal_distance') + p('merge_length') + 2.0)
        ds = np.hypot(np.diff(xs), np.diff(ys))
        v_ref = np.minimum(np.minimum(path.v[nearest], p('avoid_speed_max')), self.vehicle['max_speed'])
        speed = speed_profile(kappa, ds, v_ref, p('max_lateral_accel'), self.vehicle['max_acceleration'], v)
        self.plan = dict(x=xs, y=ys, yaw=yaws, kappa=kappa, v=speed, s=s, d=d,
                         end=len(px) - 1 + len(result.x) - 1, j=0, margin=margin)
        return result

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

    # Change the mode, log the transition, and publish the new state.
    def set_mode(self, mode):
        if mode != self.mode:
            self.get_logger().info(f'{self.mode} -> {mode}')
            self.mode = mode
            self.state_pub.publish(String(data=mode))

    # Publish a trajectory as an RViz path.
    def publish_path(self, msg):
        path = Path()
        path.header = msg.header
        for pt in msg.points:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x, pose.pose.position.y = pt.x, pt.y
            pose.pose.orientation.z, pose.pose.orientation.w = math.sin(pt.psi / 2), math.cos(pt.psi / 2)
            path.poses.append(pose)
        self.path_pub.publish(path)

    # Create an empty RViz marker in the map frame with the given type, colour and scale.
    def marker(self, stamp, ns, kind, color, scale):
        m = Marker()
        m.header.frame_id, m.header.stamp = self.map_frame, stamp
        m.ns, m.id, m.type = ns, 0, kind
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = scale
        m.color.r, m.color.g, m.color.b = color
        m.color.a = 1.0
        return m

    # Publish the RViz markers: obstacle points, RRT tree, goal and state text.
    def publish_markers(self, stamp, x, y, tree):
        pts = self.checker.points
        m = self.marker(stamp, 'rrt_obstacles', Marker.POINTS, (1.0, 0.0, 1.0), 0.08)
        m.points = [Point(x=float(a), y=float(b), z=0.1) for a, b in pts]
        self.obstacle_pub.publish(m)

        if tree is not None and self.p('publish_tree'):
            m = self.marker(stamp, 'rrt_tree', Marker.LINE_LIST, (0.6, 0.6, 0.6), 0.01)
            for xs, ys, x0, y0 in tree:
                line = [(x0, y0)] + list(zip(xs[2::3], ys[2::3])) + [(xs[-1], ys[-1])]
                for (a, b), (c, d) in zip(line[:-1], line[1:]):
                    m.points += [Point(x=float(a), y=float(b), z=0.02), Point(x=float(c), y=float(d), z=0.02)]
            self.tree_pub.publish(m)
            g = self.marker(stamp, 'rrt_goal', Marker.ARROW, (0.0, 0.6, 1.0), 0.1)
            g.scale.x, g.scale.y, g.scale.z = 0.6, 0.08, 0.08
            gx, gy, gyaw = self.goal
            g.pose.position.x, g.pose.position.y = float(gx), float(gy)
            g.pose.orientation.z, g.pose.orientation.w = math.sin(gyaw / 2), math.cos(gyaw / 2)
            self.goal_pub.publish(g)

        m = self.marker(stamp, 'rrt_state', Marker.TEXT_VIEW_FACING, COLORS[self.mode], 0.5)
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, 1.0
        m.text = self.mode
        self.state_marker_pub.publish(m)


# Start the node and spin until Ctrl+C.
def main():
    rclpy.init()
    try:
        node = RRTPlanner()
    except (RuntimeError, OSError) as error:
        print(f'[non_rrt_planner] {error}', file=sys.stderr)
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
