#!/usr/bin/env python3
"""
Aligns a map saved from slam_toolbox with the Gazebo world frame.

slam_toolbox starts its map frame at the robot's first odometry pose, i.e. at the
spawn pose (-0.75, -1.25, yaw 0) in the world. Shifting the map's `origin` by the
spawn position makes map coordinates equal world coordinates, so the AMCL initial
pose (-0.75, -1.25), the VIVA goals and the goal_manager bounds check (|x|,|y| <= 1.45)
can all be written in world coordinates.

Usage:
  python3 scripts/align_map_origin.py src/maze_navigation/maps/maze_map.yaml [--dx -0.75] [--dy -1.25]
Run it exactly ONCE on a freshly saved map.
"""
import argparse
import re
from pathlib import Path


def main():
    """Read the map yaml, shift its origin by the spawn position and write it back."""
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('map_yaml')
    parser.add_argument('--dx', type=float, default=-0.75, help='spawn x in the world frame')
    parser.add_argument('--dy', type=float, default=-1.25, help='spawn y in the world frame')
    args = parser.parse_args()

    path = Path(args.map_yaml)
    text = path.read_text()
    m = re.search(r'^origin:\s*\[\s*([-\d.eE+]+)\s*,\s*([-\d.eE+]+)\s*,\s*([-\d.eE+]+)\s*\]', text, re.M)
    if not m:
        raise SystemExit(f'No "origin: [x, y, yaw]" line found in {path}')
    ox, oy, oyaw = (float(v) for v in m.groups())
    if abs(oyaw) > 1e-6:
        raise SystemExit('Map origin has a non-zero yaw; this script only handles yaw = 0.')
    nx, ny = ox + args.dx, oy + args.dy
    new_line = f'origin: [{nx:.6f}, {ny:.6f}, {oyaw:.6f}]'
    path.write_text(text[:m.start()] + new_line + text[m.end():])
    print(f'{path}: origin [{ox:.3f}, {oy:.3f}] -> [{nx:.3f}, {ny:.3f}] (map frame == Gazebo world frame)')


if __name__ == '__main__':
    main()
