import numpy as np
from arcsf1tenth_interfaces.msg import Trajectory, Waypoint
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from arcsf1tenth_planning.frenet.frenet_path import FrenetPath


# Return the TF frame in the node's namespace (/arcsf1tenth + map -> arcsf1tenth/map).
def namespaced_frame(node, name):
    namespace = node.get_namespace().strip('/')
    return f'{namespace}/{name}' if namespace else name


# Convert a FrenetPath into a closed Trajectory message with its track widths.
def global_path_to_msg(path: FrenetPath, frame, stamp):
    msg = Trajectory()
    msg.header.frame_id = frame
    msg.header.stamp = stamp
    msg.closed = True
    psi = path.psi_point()
    for i in range(len(path.s)):
        msg.points.append(Waypoint(
            s=float(path.s[i]), d=0.0, x=float(path.x[i]), y=float(path.y[i]), psi=float(psi[i]),
            kappa=float(path.kappa[i]), v=float(path.v[i]),
            d_left=float(path.d_left[i]), d_right=float(path.d_right[i])))
    return msg


# Convert a closed Trajectory message back into a FrenetPath with speeds and track widths.
def msg_to_global_path(msg: Trajectory):
    pts = msg.points
    path = FrenetPath([p.x for p in pts], [p.y for p in pts], [p.v for p in pts])
    path.d_left = np.array([p.d_left for p in pts])
    path.d_right = np.array([p.d_right for p in pts])
    return path


# Build a Trajectory message from per-point arrays.
def arrays_to_msg(frame, stamp, s, d, x, y, psi, kappa, v, closed=False):
    msg = Trajectory()
    msg.header.frame_id = frame
    msg.header.stamp = stamp
    msg.closed = closed
    for i in range(len(x)):
        msg.points.append(Waypoint(
            s=float(s[i]), d=float(d[i]), x=float(x[i]), y=float(y[i]), psi=float(psi[i]),
            kappa=float(kappa[i]), v=float(v[i])))
    return msg


# Build an open Trajectory along the global path from s0 - behind to s0 + ahead.
def global_window(path: FrenetPath, s0, behind, ahead, step, frame, stamp, speed_scale=1.0):
    s = np.arange(s0 - behind, s0 + ahead + 1e-9, step)
    x, y = path.to_cartesian(s, np.zeros_like(s))
    return arrays_to_msg(frame, stamp, path.wrap_s(s), np.zeros_like(s), x, y, path.heading_at(s),
                         path.interp(path.kappa, s), path.interp(path.v, s) * speed_scale)


# Convert a Trajectory message into a nav_msgs/Path for RViz.
def trajectory_to_path(msg: Trajectory):
    path = Path()
    path.header = msg.header
    for p in msg.points:
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position.x = p.x
        pose.pose.position.y = p.y
        pose.pose.orientation.z = float(np.sin(p.psi / 2))
        pose.pose.orientation.w = float(np.cos(p.psi / 2))
        path.poses.append(pose)
    return path
