#!/usr/bin/python3
"""yhom's state machine: follow the global path, switch to the local planner for obstacles.

    frenet/state, obstacles, global_trajectory, yhom/local_trajectory
        ──► yhom_state_machine ──► yhom/trajectory (what pure_pursuit follows)
                               ──► yhom/state (std_msgs/String) + yhom/state_marker (RViz text)

States:
  GLOBAL   follow the global path (centerline).
           -> AVOID   an obstacle blocks the corridor ahead and the local planner has a path
           -> BLOCKED it blocks the corridor and there is no path around it
  AVOID    follow yhom/local_trajectory (Frenet optimal path around the obstacle).
           -> GLOBAL  corridor ahead clear AND car back on the global path (|d| small)
                      AND at least min_avoid_time in AVOID (no flicker)
           ("clear" = no blocking obstacle for clear_time: single-frame detection dropouts)
           -> BLOCKED local planner lost its path
  BLOCKED  brake along the global path to a stop stop_distance before the obstacle
           (deceleration brake_decel), until a path exists (for clear_time) or the
           obstacle is gone.

"Blocks the corridor": an obstacle's Frenet extent is ahead of the car, closer than
trigger_distance + speed * trigger_time, and overlaps |d| <= half car width + corridor_margin
around the global path. Once in AVOID/BLOCKED the reach is release_distance longer, so a
car that slowed down (shorter reach) does not count the same obstacle as gone.
"""

import math

import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from arcsf1tenth_interfaces.msg import FrenetState, ObstacleArray, Trajectory
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import String
from visualization_msgs.msg import Marker

from arcsf1tenth_planning.conversions import global_window, msg_to_global_path

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
COLORS = {'GLOBAL': (0.2, 1.0, 0.2), 'AVOID': (1.0, 0.7, 0.0), 'BLOCKED': (1.0, 0.1, 0.1)}


class YhomStateMachine(Node):
    def __init__(self):
        super().__init__('yhom_state_machine')
        for name in ('footprint_width', 'footprint_rear', 'footprint_front'):   # vehicle.yaml
            self.declare_parameter(name, Parameter.Type.DOUBLE)
        try:
            self.half_width = self.get_parameter('footprint_width').value / 2
            self.car_rear = self.get_parameter('footprint_rear').value
            self.car_front = self.get_parameter('footprint_front').value
        except ParameterUninitializedException:
            raise RuntimeError('vehicle parameters missing: start it with yhom_controller.launch.py')
        self.declare_parameter('rate', 20.0)                 # [Hz]
        self.declare_parameter('trigger_distance', 3.0)      # [m] react to obstacles this far ahead ...
        self.declare_parameter('trigger_time', 1.5)          # [s] ... plus speed * this
        self.declare_parameter('corridor_margin', 0.25)      # [m] corridor = global path +- (half width + this)
        self.declare_parameter('return_d', 0.15)             # [m] |d| below this = back on the global path
        self.declare_parameter('min_avoid_time', 1.0)        # [s] stay in AVOID at least this long
        self.declare_parameter('clear_time', 0.5)            # [s] corridor clear / path found this long before trusting it
        self.declare_parameter('release_distance', 2.0)      # [m] longer reach once in AVOID / BLOCKED
        self.declare_parameter('stop_distance', 1.0)         # [m] BLOCKED: front bumper stops this far before the obstacle
        self.declare_parameter('brake_decel', 2.0)           # [m/s^2] BLOCKED: braking profile
        self.declare_parameter('timeout', 0.5)               # [s] inputs older than this are ignored / stale
        self.declare_parameter('global_ahead', 12.0)         # [m] global trajectory window sent to the controller
        self.declare_parameter('global_behind', 1.0)         # [m]
        self.declare_parameter('global_step', 0.1)           # [m]
        p = lambda name: self.get_parameter(name).value
        self.p = p

        self.path = None
        self.frame = None
        self.state_msg = None
        self.obstacles = None
        self.local = None
        self.state = 'GLOBAL'
        self.state_since = self.get_clock().now()
        self.last_blocked = self.state_since    # last time the corridor was blocked
        self.last_no_path = self.state_since    # last time the local planner had no path
        self.stop_s = None                      # BLOCKED: global s where the car must stand
        self.create_subscription(Trajectory, 'global_trajectory', self.on_path, LATCHED)
        self.create_subscription(FrenetState, 'frenet/state', lambda m: setattr(self, 'state_msg', m), 10)
        self.create_subscription(ObstacleArray, 'obstacles', lambda m: setattr(self, 'obstacles', m), 10)
        self.create_subscription(Trajectory, 'yhom/local_trajectory', lambda m: setattr(self, 'local', m), 10)
        self.traj_pub = self.create_publisher(Trajectory, 'yhom/trajectory', 10)
        self.state_pub = self.create_publisher(String, 'yhom/state', LATCHED)
        self.marker_pub = self.create_publisher(Marker, 'yhom/state_marker', 10)
        self.state_pub.publish(String(data=self.state))
        self.create_timer(1.0 / p('rate'), self.step)

    def on_path(self, msg):
        if msg.points:
            self.path = msg_to_global_path(msg)
            self.frame = msg.header.frame_id

    def fresh(self, msg, now):
        return msg is not None and (now - Time.from_msg(msg.header.stamp)).nanoseconds * 1e-9 < self.p('timeout')

    def corridor_blocked(self, st, now, extra):
        """Distance along the path to the nearest obstacle blocking the global path ahead
        (within the trigger reach + extra), or None."""
        if not self.fresh(self.obstacles, now):
            return None
        reach = self.p('trigger_distance') + max(st.speed, 0.0) * self.p('trigger_time') + extra
        half = self.half_width + self.p('corridor_margin')
        nearest = None
        for o in self.obstacles.obstacles:
            a = self.path.ds(o.s_start, st.s)
            b = a + (o.s_end - o.s_start) % self.path.length
            if b > -self.car_rear and a < reach and o.d_min < half and o.d_max > -half:
                nearest = a if nearest is None else min(nearest, a)
        return nearest

    def step(self):
        now = self.get_clock().now()
        st = self.state_msg
        if self.path is None or not self.fresh(st, now):
            return   # no pose: publish nothing, pure_pursuit stops on its own (stale trajectory)
        extra = 0.0 if self.state == 'GLOBAL' else self.p('release_distance')
        distance = self.corridor_blocked(st, now, extra)
        blocked = distance is not None
        if blocked:
            # where to stand if there is no way around: front bumper stop_distance before it
            self.stop_s = st.s + distance - self.car_front - self.p('stop_distance')
        local_ok = self.fresh(self.local, now) and len(self.local.points) > 0
        # Hysteresis: a detection or a planner result can drop out for single frames, so
        # "clear" / "path exists" only count after holding for clear_time
        if blocked:
            self.last_blocked = now
        if not local_ok:
            self.last_no_path = now
        hold = self.p('clear_time')
        clear = (now - self.last_blocked).nanoseconds * 1e-9 > hold
        path_stable = local_ok and (now - self.last_no_path).nanoseconds * 1e-9 > hold
        in_state = (now - self.state_since).nanoseconds * 1e-9

        new = self.state
        if self.state == 'GLOBAL':
            if blocked:
                new = 'AVOID' if local_ok else 'BLOCKED'
        elif self.state == 'AVOID':
            if not local_ok:
                new = 'BLOCKED'
            elif clear and abs(st.d) < self.p('return_d') and in_state > self.p('min_avoid_time'):
                new = 'GLOBAL'
        elif self.state == 'BLOCKED':
            if clear and abs(st.d) < self.p('return_d'):
                new = 'GLOBAL'
            elif path_stable:
                new = 'AVOID'
        if new != self.state:
            self.get_logger().info(f'{self.state} -> {new}')
            self.state, self.state_since = new, now
            self.state_pub.publish(String(data=new))

        stamp = now.to_msg()
        if self.state == 'AVOID':
            msg = self.local
        else:
            msg = global_window(self.path, st.s, self.p('global_behind'), self.p('global_ahead'),
                                self.p('global_step'), self.frame, stamp)
            if self.state == 'BLOCKED' and self.stop_s is not None:
                # braking profile v = sqrt(2 a (distance to the stop point)), 0 at and after it
                decel = self.p('brake_decel')
                for pt in msg.points:
                    to_stop = -self.path.ds(pt.s, self.path.wrap_s(self.stop_s))
                    pt.v = min(pt.v, math.sqrt(2.0 * decel * max(to_stop, 0.0)))
        self.traj_pub.publish(msg)
        self.publish_marker(stamp)

    def publish_marker(self, stamp):
        m = Marker()
        m.header.frame_id = self.frame
        m.header.stamp = stamp
        m.ns, m.id, m.type = 'yhom_state', 0, Marker.TEXT_VIEW_FACING
        x, y = self.path.to_cartesian(self.state_msg.s, self.state_msg.d)
        m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), 1.0
        m.pose.orientation.w = 1.0
        m.scale.z = 0.5
        m.color.r, m.color.g, m.color.b = COLORS[self.state]
        m.color.a = 1.0
        m.text = self.state
        self.marker_pub.publish(m)


def main():
    rclpy.init()
    try:
        node = YhomStateMachine()
    except RuntimeError as error:
        print(f'[yhom_state_machine] {error}', file=sys.stderr)
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
