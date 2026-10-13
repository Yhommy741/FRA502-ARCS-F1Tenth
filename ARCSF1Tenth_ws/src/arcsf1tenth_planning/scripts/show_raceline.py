#!/usr/bin/python3
"""Show a map with its centerline and raceline in a matplotlib window.

By default it looks next to the map for <map>_centerline.csv and <map>_raceline.csv
(written by generate_raceline.py). Use --centerline / --raceline for other files,
e.g. the original F1TENTH centerline.csv / raceline.csv.

Usage:
  show_raceline.py <map.yaml | track name> [--centerline FILE] [--raceline FILE] [--start X Y YAW] [--save FILE]
"""

import argparse
import os

from arcsf1tenth_planning.plotting import load_centerline, load_map_image, load_raceline, plot_track
from arcsf1tenth_planning.tracks import resolve_map


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('map_yaml', help='map yaml (e.g. slam_map.yaml) or a track name (e.g. brands_hatch)')
    parser.add_argument('--centerline', default=None, help='centerline CSV (default: <map>_centerline.csv)')
    parser.add_argument('--raceline', default=None, help='raceline CSV (default: <map>_raceline.csv)')
    parser.add_argument('--start', type=float, nargs=3, default=None, metavar=('X', 'Y', 'YAW'),
                        help='draw the start pose arrow')
    parser.add_argument('--save', default=None, help='save the picture to this file instead of showing it')
    args = parser.parse_args()

    map_yaml, track_start = resolve_map(args.map_yaml)
    start = args.start or track_start
    stem = os.path.splitext(os.path.abspath(map_yaml))[0]
    centerline_path = args.centerline or f'{stem}_centerline.csv'
    raceline_path = args.raceline or f'{stem}_raceline.csv'
    # Original F1TENTH track folders ship centerline.csv / raceline.csv next to map.yaml
    folder = os.path.dirname(stem)
    if not args.centerline and not os.path.exists(centerline_path):
        centerline_path = os.path.join(folder, 'centerline.csv') if os.path.exists(os.path.join(folder, 'centerline.csv')) else centerline_path
    if not args.raceline and not os.path.exists(raceline_path):
        raceline_path = os.path.join(folder, 'raceline.csv') if os.path.exists(os.path.join(folder, 'raceline.csv')) else raceline_path

    centerline = load_centerline(centerline_path) if os.path.exists(centerline_path) else None
    raceline = load_raceline(raceline_path) if os.path.exists(raceline_path) else None
    if centerline is None and raceline is None:
        print(f'No centerline/raceline found ({centerline_path}, {raceline_path}); showing the map only.\n'
              'Generate them with generate_raceline.py, or pass --centerline / --raceline.')

    if args.save:
        import matplotlib
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    image, extent = load_map_image(map_yaml)
    plot_track(image, extent, centerline, raceline, start, title=os.path.basename(stem))
    if args.save:
        plt.savefig(args.save, dpi=120, bbox_inches='tight')
        print(f'saved {args.save}')
    else:
        plt.show()


if __name__ == '__main__':
    main()
