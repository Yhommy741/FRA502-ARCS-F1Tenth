#!/usr/bin/python3

import argparse
import glob
import math
import os
import shutil
import sys
import time

import numpy as np
import rclpy
from ament_index_python.packages import PackageNotFoundError, get_package_prefix, get_package_share_directory
from cartographer_ros_msgs.msg import TrajectoryStates
from cartographer_ros_msgs.srv import FinishTrajectory, GetTrajectoryStates
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

# Same as nav2 map_saver (trinary): grid -> image uses 0.65 / 0.25 ...
OCCUPIED_THRESH = 0.65
FREE_THRESH = 0.25
# ... but the yaml stores free_thresh 0.196 for map LOADERS: unknown pixels (205) have
# occupancy (255 - 205) / 255 = 0.196, which must NOT count as free (free = below free_thresh)
YAML_FREE_THRESH = 0.196


# Print an error message and exit with status 1.
def fail(message):
    print(f'save_map: {message}', file=sys.stderr)
    sys.exit(1)


# Return (source track folder, installed track folder or None) for a track name.
def track_dirs(track):
    try:
        prefix = get_package_prefix('arcsf1tenth_tracks')
    except PackageNotFoundError:
        fail('arcsf1tenth_tracks not found: source install/setup.bash')
    source_root = os.path.join(os.path.dirname(os.path.dirname(prefix)), 'src', 'arcsf1tenth_tracks')
    matches = glob.glob(os.path.join(source_root, '*', track, 'track.yaml'))
    if not matches:
        available = sorted(os.path.basename(os.path.dirname(p))
                           for p in glob.glob(os.path.join(source_root, '*', '*', 'track.yaml')))
        fail(f"track '{track}' not found in {source_root}. Tracks: {', '.join(available)}")
    source = os.path.dirname(matches[0])
    installed = os.path.join(get_package_share_directory('arcsf1tenth_tracks'),
                             os.path.relpath(source, source_root))
    return source, installed if os.path.isdir(installed) else None


class Saver(Node):
    # Set up the node: map subscriber and the Cartographer trajectory service clients.
    def __init__(self):
        super().__init__('save_map', namespace='arcsf1tenth')

        # Parameters

        # Variables
        ns = self.get_namespace().rstrip('/')
        self.map_msg = None
        self.map_time = None
        self.map_topic = f'{ns}/map'

        # Publishers

        # Subscribers
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, f'{ns}/map', self.on_map, qos)

        # Service Servers

        # Service Clients
        self.get_states = self.create_client(GetTrajectoryStates, f'{ns}/get_trajectory_states')
        self.finish = self.create_client(FinishTrajectory, f'{ns}/finish_trajectory')

        # Action Servers

        # Action Clients

        # Timers

        # On Start

    # Store the latest map and the time it arrived.
    def on_map(self, msg):
        self.map_msg = msg
        self.map_time = time.time()

    # Call a service and wait for its response, exiting if it is unavailable or does not answer.
    def call(self, client, request, timeout=30.0):
        if not client.wait_for_service(timeout_sec=5.0):
            fail(f'service {client.srv_name} not available: is slam:=mapping running (same namespace)?')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            fail(f'no answer from {client.srv_name}')
        return future.result()

    # Wait for a map message received after `after` (the map is re-published ~1 Hz).
    def wait_for_map(self, after, timeout=15.0):
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.map_msg is not None and self.map_time >= after:
                return self.map_msg
        fail(f'no map received on {self.map_topic}')

    # Return True in simulation, detected by a /clock publisher (Gazebo has one, the real car does not).
    def is_simulation(self):
        end = time.time() + 2.0
        while time.time() < end:
            if self.count_publishers('/clock') > 0:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False


# Write an OccupancyGrid to <stem>.pgm + <stem>.yaml, like nav2 map_saver in trinary mode.
def write_map_image(grid, stem):
    width, height = grid.info.width, grid.info.height
    data = np.asarray(grid.data, dtype=np.int16).reshape(height, width)[::-1]  # image row 0 = top
    pixels = np.full(data.shape, 205, dtype=np.uint8)                          # unknown
    pixels[(data >= 0) & (data <= FREE_THRESH * 100)] = 254                    # free
    pixels[data >= OCCUPIED_THRESH * 100] = 0                                  # occupied
    with open(f'{stem}.pgm', 'wb') as f:
        f.write(f'P5\n{width} {height}\n255\n'.encode())
        f.write(pixels.tobytes())

    q = grid.info.origin.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
    o = grid.info.origin.position
    with open(f'{stem}.yaml', 'w') as f:
        f.write(f'image: {os.path.basename(stem)}.pgm\n'
                'mode: trinary\n'
                f'resolution: {grid.info.resolution:.3f}\n'
                f'origin: [{o.x:.3f}, {o.y:.3f}, {yaw:.3f}]\n'
                'negate: 0\n'
                f'occupied_thresh: {OCCUPIED_THRESH}\n'
                f'free_thresh: {YAML_FREE_THRESH}\n')
    return width, height


# Finish the mapping trajectory, save the map image, and copy it into install/.
def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('track', help='track folder name in arcsf1tenth_tracks (e.g. austin)')
    args, ros_args = parser.parse_known_args()

    source_dir, installed_dir = track_dirs(args.track)
    rclpy.init(args=ros_args)
    node = Saver()
    sim = node.is_simulation()
    sub = os.path.join('maps', 'sim' if sim else 'real')
    out_dir = os.path.join(source_dir, sub)
    os.makedirs(out_dir, exist_ok=True)
    stem = os.path.join(out_dir, 'cartographer_map')
    print(f"{'simulation' if sim else 'real car'} -> {out_dir}")

    # 1. finish the active mapping trajectory
    states = node.call(node.get_states, GetTrajectoryStates.Request()).trajectory_states
    by_id = dict(zip(states.trajectory_id, states.trajectory_state))
    active = [tid for tid, state in by_id.items() if state == TrajectoryStates.ACTIVE]
    if not active:
        print('no active trajectory (already finished), saving the current state')
    for tid in active:
        print(f'finishing trajectory {tid} (final optimization) ...')
        response = node.call(node.finish, FinishTrajectory.Request(trajectory_id=tid), timeout=120.0)
        if response.status.code != 0:
            fail(f'finish_trajectory failed: {response.status.message}')
    finished_at = time.time()

    # 2. map image (wait for the map re-published after the final optimization)
    grid = node.wait_for_map(after=finished_at + 1.0)
    width, height = write_map_image(grid, stem)
    res = grid.info.resolution
    print(f'saved {stem}.pgm + .yaml ({width} x {height} cells, {width * res:.1f} x {height * res:.1f} m)')
    node.destroy_node()
    rclpy.try_shutdown()

    # A .pbstream from the old Cartographer localization would no longer match this map
    if os.path.isfile(f'{stem}.pbstream'):
        os.remove(f'{stem}.pbstream')
        print(f'removed old {stem}.pbstream (Cartographer localization is no longer used)')

    # 3. copy into install/ so localization can use it without colcon build
    if installed_dir:
        target = os.path.join(installed_dir, sub)
        os.makedirs(target, exist_ok=True)
        for ext in ('pgm', 'yaml'):
            shutil.copy2(f'{stem}.{ext}', target)
        print(f'copied to {target} (no colcon build needed)')
    else:
        print('install/ copy skipped: run colcon build --packages-select arcsf1tenth_tracks')

    launch = 'simulation' if sim else 'real'
    print(f'\ndone. Next:\n'
          f'  ros2 run arcsf1tenth_planning generate_raceline.py {args.track}\n'
          f'  ros2 launch arcsf1tenth_bringup {launch}.launch.py track:={args.track} slam:=localization')


if __name__ == '__main__':
    main()
