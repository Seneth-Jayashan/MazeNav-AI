#!/usr/bin/env python3
"""
IE4060 Autonomous Navigation - Benchmark post-processing

Reads the raw, measured trial rows written by `ros2 run goal_interface viva_demo`
(and the DQN benchmark) and adds, for each trial:
  - Ref_Length_m:  length of the shortest collision-free path from the recorded start to
                   the method's success circle around the goal, for a robot of radius
                   ROBOT_RADIUS (the Nav2 footprint): the exact shortest path to the goal
                   (visibility graph over the walls of maze.world inflated by that radius)
                   minus the goal tolerance (a leg ends as soon as the robot is that close).
                   A robot that keeps the clearance cannot drive a shorter successful path, so
                   Efficiency_pct <= 100 unless it cut closer to a wall than the radius.
  - Cell_Route_m:  for comparison, the route through the centres of the cells on the
                   shortest cell path (the earlier reference, longer than any real path)
  - Efficiency_pct: Ref_Length_m / measured Path_Length_m * 100

Nothing here simulates or estimates navigation results: every time, path length,
success flag and collision count comes from the input CSV.

Usage:
  python3 scripts/collect_metrics.py [raw_trials.csv ...]
  (default input: docs/raw_trials.csv)
Outputs: docs/test_results.csv (per trial) and docs/test_summary.csv (per method and goal)
"""

import csv
import math
import statistics
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / 'src' / 'maze_dqn'))
from maze_dqn.maze_env import MazeEnvironment  # noqa: E402

DOCS = REPO_ROOT / 'docs'
ROBOT_RADIUS = 0.10   # [m] robot_radius in nav2_params.yaml
# [m] success radius: Nav2 xy_goal_tolerance (nav2_params.yaml), viva_demo goal_tolerance for the DQN
GOAL_TOLERANCE = {'nav2': 0.10, 'dqn': 0.15}


def goal_tolerance(method):
    """Success radius of the method that produced a row ('DQN' in the name -> 0.15 m)."""
    return GOAL_TOLERANCE['dqn' if 'dqn' in method.lower() else 'nav2']


def _segment_hits_box(p, q, box):
    """True if segment p-q passes through the open interior of axis-aligned box."""
    x0, x1, y0, y1 = box[0] + 1e-7, box[1] - 1e-7, box[2] + 1e-7, box[3] - 1e-7
    t0, t1 = 0.0, 1.0
    for a, d, lo, hi in ((p[0], q[0] - p[0], x0, x1), (p[1], q[1] - p[1], y0, y1)):
        if abs(d) < 1e-12:
            if not lo < a < hi:
                return False
            continue
        ta, tb = sorted(((lo - a) / d, (hi - a) / d))
        t0, t1 = max(t0, ta), min(t1, tb)
        if t0 >= t1:
            return False
    return True


def optimal_path_length(walls, start, goal, r=ROBOT_RADIUS):
    """Shortest path for a disc of radius r (walls inflated by r), via a visibility graph."""
    boxes = [(x1 - r, x2 + r, y1 - r, y2 + r) for x1, x2, y1, y2 in walls]
    inside = lambda pt: any(b[0] < pt[0] < b[1] and b[2] < pt[1] < b[3] for b in boxes)
    eps = 1e-4
    corners = [(bx, by) for b in boxes for bx in (b[0] - eps, b[1] + eps) for by in (b[2] - eps, b[3] + eps)]
    nodes = [tuple(start), tuple(goal)] + [c for c in corners if not inside(c)]
    n = len(nodes)
    dist = [math.inf] * n
    dist[0] = 0.0
    done = [False] * n
    for _ in range(n):                      # Dijkstra on the (dense) visibility graph
        u = min((i for i in range(n) if not done[i]), key=lambda i: dist[i], default=None)
        if u is None or dist[u] == math.inf:
            break
        if u == 1:
            return dist[1]
        done[u] = True
        for v in range(n):
            if done[v]:
                continue
            d = dist[u] + math.dist(nodes[u], nodes[v])
            if d < dist[v] and not any(_segment_hits_box(nodes[u], nodes[v], b) for b in boxes):
                dist[v] = d
    return dist[1]


def grid_reference_path(env, start, goal):
    """Polyline start -> intermediate cell centres -> goal along the shortest cell path."""
    cell, gcell = env.cell_of(*start), env.cell_of(*goal)
    pts = [start]
    while cell != gcell:
        cell = env.next_cell(cell, gcell)
        if cell != gcell:
            pts.append(env.cell_centre(cell))
    pts.append(goal)
    return pts


def path_length(pts):
    """Length of a polyline [(x, y), ...]."""
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))


def fmt_mean_std(vals, digits=2):
    """'mean ± sample std' as text ('n/a' when there are no values)."""
    if not vals:
        return 'n/a'
    if len(vals) == 1:
        return f'{vals[0]:.{digits}f}'
    return f'{statistics.mean(vals):.{digits}f} ± {statistics.stdev(vals):.{digits}f}'


def main():
    """Read the raw trial CSV(s), add reference length and efficiency, write per-trial
    results (docs/test_results.csv) and a per method/goal summary (docs/test_summary.csv)."""
    inputs = [Path(p) for p in sys.argv[1:]] or [DOCS / 'raw_trials.csv']
    rows = []
    for p in inputs:
        if not p.exists():
            print(f'Input not found: {p}')
            sys.exit(1)
        with open(p, newline='') as f:
            rows.extend(csv.DictReader(f))
    if not rows:
        print('No trial rows found.')
        sys.exit(1)

    env = MazeEnvironment()
    out_rows = []
    for r in rows:
        start = (float(r['Start_x']), float(r['Start_y']))
        goal = (float(r['Goal_x']), float(r['Goal_y']))
        ok_start = all(map(math.isfinite, start))
        ref = (max(0.0, optimal_path_length(env._w, start, goal) - goal_tolerance(r['Method']))
               if ok_start else float('nan'))
        cell_route = path_length(grid_reference_path(env, start, goal)) if ok_start else float('nan')
        measured = float(r['Path_Length_m'])
        success = int(r['Success'])
        eff = ref / measured * 100.0 if success and measured > 0 and math.isfinite(ref) else float('nan')
        out = dict(r)
        out['Ref_Length_m'] = f'{ref:.3f}'
        out['Cell_Route_m'] = f'{cell_route:.3f}'
        out['Efficiency_pct'] = f'{eff:.1f}' if math.isfinite(eff) else ''
        out_rows.append(out)

    fields = list(rows[0].keys()) + ['Ref_Length_m', 'Cell_Route_m', 'Efficiency_pct']
    with open(DOCS / 'test_results.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(out_rows)

    # Per (method, goal) summary over trials; time/path/efficiency over successful trials only
    groups = {}
    for r in out_rows:
        groups.setdefault((r['Method'], r['Goal']), []).append(r)
    summary = []
    for (method, goal), rs in groups.items():
        ok = [r for r in rs if int(r['Success'])]
        summary.append({
            'Method': method,
            'Goal': goal,
            'Trials': len(rs),
            'Success_Rate': f'{len(ok)}/{len(rs)}',
            'Time_s': fmt_mean_std([float(r['Time_s']) for r in ok]),
            'Path_Length_m': fmt_mean_std([float(r['Path_Length_m']) for r in ok]),
            'Ref_Length_m': fmt_mean_std([float(r['Ref_Length_m']) for r in rs]),
            'Efficiency_pct': fmt_mean_std([float(r['Efficiency_pct']) for r in ok], 1),
            'Collisions': sum(int(r['Collisions']) for r in rs),
            'Min_Clearance_m': f"{min(float(r['Min_Clearance_m']) for r in rs):.3f}",
        })
    with open(DOCS / 'test_summary.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(summary[0].keys()))
        w.writeheader()
        w.writerows(summary)

    print('=' * 118)
    print(f"{'Method':<18} | {'Goal':<30} | {'Succ':<5} | {'Time (s)':<14} | {'Path (m)':<13} | {'Ref (m)':<13} | {'Eff (%)':<12} | Coll")
    print('-' * 118)
    for s in summary:
        print(f"{s['Method']:<18} | {s['Goal']:<30} | {s['Success_Rate']:<5} | {s['Time_s']:<14} | {s['Path_Length_m']:<13} | "
              f"{s['Ref_Length_m']:<13} | {s['Efficiency_pct']:<12} | {s['Collisions']}")
    print('=' * 118)
    print(f"Per-trial results: {DOCS / 'test_results.csv'}")
    print(f"Summary:           {DOCS / 'test_summary.csv'}")


if __name__ == '__main__':
    main()
