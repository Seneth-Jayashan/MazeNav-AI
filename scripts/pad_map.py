#!/usr/bin/env python3
"""
Pads a saved map with a border of unknown cells so Nav2's NavFn planner can return long paths.

NavFn traces the final path through its potential field in half-cell steps and stops after
4 x (costmap width in cells) steps, i.e. it cannot return a path longer than about twice the
costmap width in metres. The global costmap takes its size from the map, and the SLAM map of
the 3 m maze is only ~3.1 m wide, so every goal more than ~6.2 m of driving away failed (the
longest route in the maze is ~12 m). A 2.5 m unknown border makes the map ~8.1 m wide, which
raises the limit to ~16 m. The mapped cells and their world coordinates are unchanged (the
origin is moved by the border width), and nav2_params.yaml sets NavFn allow_unknown: false so
no path can run through the border.

Usage (after align_map_origin.py, once per saved map):
  python3 scripts/pad_map.py src/maze_navigation/maps/maze_map.yaml [--border 2.5]
"""
import argparse
import re
from pathlib import Path

from PIL import Image

UNKNOWN = 205  # map_server value for unknown cells (trinary mode)


def main():
    """Add the unknown border to the map image and move the origin to match."""
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('map_yaml')
    parser.add_argument('--border', type=float, default=2.5, help='border width in metres on every side')
    args = parser.parse_args()

    yaml_path = Path(args.map_yaml)
    text = yaml_path.read_text()
    res = float(re.search(r'^resolution:\s*([\d.]+)', text, re.M).group(1))
    m = re.search(r'^origin:\s*\[\s*([-\d.eE+]+)\s*,\s*([-\d.eE+]+)\s*,\s*([-\d.eE+]+)\s*\]', text, re.M)
    if not m:
        raise SystemExit(f'No "origin: [x, y, yaw]" line found in {yaml_path}')
    ox, oy, oyaw = (float(v) for v in m.groups())
    image_path = yaml_path.parent / re.search(r'^image:\s*(\S+)', text, re.M).group(1)

    img = Image.open(image_path)
    pad = round(args.border / res)
    out = Image.new('L', (img.width + 2 * pad, img.height + 2 * pad), UNKNOWN)
    out.paste(img, (pad, pad))
    out.save(image_path)

    nx, ny = ox - pad * res, oy - pad * res
    yaml_path.write_text(text[:m.start()] + f'origin: [{nx:.6f}, {ny:.6f}, {oyaw:.6f}]' + text[m.end():])
    print(f'{image_path}: {img.width}x{img.height} -> {out.width}x{out.height} px, '
          f'origin [{ox:.3f}, {oy:.3f}] -> [{nx:.3f}, {ny:.3f}]')


if __name__ == '__main__':
    main()
