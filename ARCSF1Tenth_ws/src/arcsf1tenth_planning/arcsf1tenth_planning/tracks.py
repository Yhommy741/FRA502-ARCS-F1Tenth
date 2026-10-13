import glob
import os
import sys

import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_prefix, get_package_share_directory


# Map files tried for a track name, in order: our SLAM map first, then the original F1TENTH map
MAP_NAMES = ('slam_map.yaml', 'map.yaml')


def tracks_dir():
    """arcsf1tenth_tracks in the SOURCE tree (outputs must land there, not in install/).

    The workspace is two levels above the package's install prefix
    (<ws>/install/arcsf1tenth_tracks). Falls back to the installed share folder.
    """
    try:
        prefix = get_package_prefix('arcsf1tenth_tracks')
    except PackageNotFoundError:
        sys.exit('arcsf1tenth_tracks not found: build it and source install/setup.bash')
    source = os.path.join(os.path.dirname(os.path.dirname(prefix)), 'src', 'arcsf1tenth_tracks')
    if os.path.isdir(source):
        return source
    share = get_package_share_directory('arcsf1tenth_tracks')
    print(f'warning: source tree not found, using installed tracks in {share}', file=sys.stderr)
    return share


def resolve_map(arg):
    """Map yaml path from a path or a track name.

    Returns (map_yaml, start) where start is the default start pose [x, y, yaw]
    in that map's frame: 0 0 0 for a SLAM map (the map starts where mapping
    started), the track.yaml start_pose for the original map.
    """
    if os.path.isfile(arg):
        return arg, None

    name = os.path.basename(arg)
    name = name[:-len('.yaml')] if name.endswith('.yaml') else name
    root = tracks_dir()
    matches = glob.glob(os.path.join(root, '*', name, 'track.yaml'))
    if not matches:
        available = sorted(os.path.basename(os.path.dirname(p)) for p in glob.glob(os.path.join(root, '*', '*', 'track.yaml')))
        sys.exit(f"'{arg}' is neither a map file nor a track. Tracks: {', '.join(available)}")

    track_dir = os.path.dirname(matches[0])
    for map_name in MAP_NAMES:
        map_yaml = os.path.join(track_dir, 'maps', 'sim', map_name)
        if os.path.isfile(map_yaml):
            break
    else:
        sys.exit(f"Track '{name}' has no {' or '.join(MAP_NAMES)} in {track_dir}/maps/sim")

    if map_name == 'slam_map.yaml':
        start = [0.0, 0.0, 0.0]
    else:
        with open(matches[0]) as f:
            pose = yaml.safe_load(f)['start_pose']
        start = [float(pose['x']), float(pose['y']), float(pose['yaw'])]
    print(f'track {name}: {map_yaml} (start {start[0]:g} {start[1]:g} {start[2]:g})')
    return map_yaml, start
