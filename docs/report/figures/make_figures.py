"""Builds the report figures from the measured data in the repository.

  python docs/report/figures/make_figures.py

architecture.png - packages, nodes and topics
results.png      - travel time and collision events per method and goal (docs/test_summary.csv)
dqn_training.png - DQN evaluation during training (src/maze_dqn/models/*_eval_log.csv)
"""
import csv
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent

# Reference categorical palette, slots 1-4 in fixed order (validated: scripts/validate_palette.js)
SERIES = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100']
VIOLET = '#6250d6'   # slot 7, for the retrained DQN (validated in chart order with the hues above)
INK, INK2, GRID, SURFACE = '#0b0b0b', '#52514e', '#e4e3df', '#ffffff'

plt.rcParams.update({
    'font.family': 'DejaVu Sans', 'font.size': 8.5, 'axes.edgecolor': INK2, 'axes.labelcolor': INK,
    'xtick.color': INK2, 'ytick.color': INK2, 'axes.spines.top': False, 'axes.spines.right': False,
    'axes.grid': True, 'grid.color': GRID, 'grid.linewidth': 0.6, 'axes.axisbelow': True,
    'figure.facecolor': SURFACE, 'axes.facecolor': SURFACE, 'legend.frameon': False,
})


def results():
    """Figure: mean travel time and collision events per method and task (test_summary.csv)."""
    rows = list(csv.DictReader(open(ROOT / 'docs' / 'test_summary.csv', encoding='utf-8')))
    # (method in the CSV, legend label, colour, hatch): the shielded DQN reuses the subgoal hue with
    # a texture, so four validated hues + texture encode the five methods
    methods = [('Nav2 (Dijkstra)', 'Nav2 Dijkstra', SERIES[0], None),
               ('Nav2 (A*)', 'Nav2 A*', SERIES[1], None),
               ('DQN (subgoal)', 'DQN subgoal', SERIES[2], None),
               ('DQN (subgoal) + shield', 'DQN subgoal + shield', SERIES[2], '////'),
               ('DQN (subgoal, retrained)', 'DQN subgoal, retrained', VIOLET, None),
               ('DQN (final goal)', 'DQN final goal', SERIES[3], None)]
    goals = ['Goal 1 (Corridor Junction)', 'Goal 2 (Central Passage)', 'Goal 3 (Far Maze Destination)']
    get = {(r['Method'], r['Goal']): r for r in rows}
    methods = [m for m in methods if (m[0], goals[0]) in get]

    fig, (a1, a2) = plt.subplots(1, 2, figsize=(7.2, 2.7), gridspec_kw={'wspace': 0.25})
    w = 0.8 / len(methods)
    for i, (m, lab, col, hatch) in enumerate(methods):
        for j, g in enumerate(goals):
            r = get[(m, g)]
            x = j + (i - (len(methods) - 1) / 2) * w
            ok = r['Time_s'] != 'n/a'
            t = float(r['Time_s'].split(' ± ')[0]) if ok else 0.0
            kw = dict(color=col if not hatch else SURFACE, edgecolor=col if hatch else SURFACE,
                      hatch=hatch, linewidth=1.0)
            a1.bar(x, t, w * 0.92, label=lab if j == 0 else None, **kw)
            a1.text(x, t + 2, r['Success_Rate'], ha='center', va='bottom', fontsize=6, color=INK2, rotation=90)
            c = int(r['Collisions'])
            a2.bar(x, c, w * 0.92, **kw)
            a2.text(x, c + 1.5, str(c), ha='center', va='bottom', fontsize=6, color=INK2)
    for a, title, ylab in ((a1, 'Mean travel time (successful legs)', 'seconds'),
                           (a2, 'Collision events (3 trials)', 'events')):
        a.set_xticks(range(3), ['S → G1', 'G1 → G2', 'G2 → G3'])
        a.set_title(title, fontsize=9, color=INK, loc='left')
        a.set_ylabel(ylab)
        a.grid(axis='x', visible=False)
    a1.set_ylim(0, 110)
    a2.set_ylim(0, 100)
    fig.legend(loc='lower center', ncol=3, bbox_to_anchor=(0.5, -0.15), fontsize=7.2)
    fig.text(0.01, -0.18, 'Numbers above time bars: goals reached / 3 (0/3: no successful leg, no time)',
             fontsize=6.3, color=INK2)
    fig.savefig(OUT / 'results.png', dpi=220, bbox_inches='tight')
    plt.close(fig)


def dqn_training():
    """Figure: random full-maze success during DQN training, from the evaluation logs."""
    models = ROOT / 'src' / 'maze_dqn' / 'models'
    fig, a = plt.subplots(figsize=(7.2, 2.5))
    runs = (('dqn_subgoal_eval_log.csv', 'Subgoal DQN (original)', SERIES[2]),
            ('dqn_eval_log.csv', 'Final-goal DQN', SERIES[3]),
            ('dqn_subgoal_safe_eval_log.csv', 'Subgoal DQN, retrained (0.15 m margin, 0.5 rad/s turns)', VIOLET))
    for k, (f, lab, col) in enumerate(runs):
        if not (models / f).exists():
            continue
        r = list(csv.DictReader(open(models / f, encoding='utf-8')))
        ep = [int(x['Episode']) for x in r]
        sr = [100 * float(x['RandomFullMazeSuccess']) for x in r]
        a.plot(ep, sr, color=col, linewidth=2, marker='o', markersize=3.5, label=lab, zorder=4 - k)
    a.annotate('original: kept checkpoint\n(episode 1300: 38 %)', xy=(1300, 38), xytext=(300, 55),
               fontsize=7, color=INK2, arrowprops={'arrowstyle': '->', 'color': INK2, 'lw': 0.8})
    if (models / 'dqn_subgoal_safe_eval_log.csv').exists():
        a.annotate('retrained: kept checkpoint\n(episode 2200: 82 %)', xy=(2200, 82), xytext=(2550, 66),
                   fontsize=7, color=INK2, arrowprops={'arrowstyle': '->', 'color': INK2, 'lw': 0.8})
    a.set_xlabel('training episode')
    a.set_ylabel('random full-maze goals\nreached (%)')
    a.set_title('Greedy evaluation every 100 episodes (50 random start/goal pairs)', fontsize=9, color=INK, loc='left')
    a.text(2150, 4.0, 'final-goal DQN: 0 % at every evaluation', fontsize=7, color=INK2)
    a.set_xlim(0, 3400)
    a.set_ylim(-3, 95)
    a.legend(loc='upper left', fontsize=7.5)
    fig.savefig(OUT / 'dqn_training.png', dpi=220, bbox_inches='tight')
    plt.close(fig)


def architecture():
    """Figure: packages, main nodes and the topics between them."""
    fig, a = plt.subplots(figsize=(7.2, 3.1))
    a.set_xlim(0, 100); a.set_ylim(-8, 42); a.axis('off')

    def box(x, y, w, h, title, lines, col):
        """Draw one rounded box with a title and a few lines of text."""
        a.add_patch(FancyBboxPatch((x, y), w, h, boxstyle='round,pad=0.4,rounding_size=1.2',
                                   facecolor=col + '1f', edgecolor=col, linewidth=1.2))
        a.text(x + w / 2, y + h - 2.3, title, ha='center', va='center', fontsize=7.4, color=INK, weight='bold')
        a.text(x + w / 2, y + h - 4.6, '\n'.join(lines), ha='center', va='top', fontsize=6.8, color=INK2,
               linespacing=1.35)

    def arrow(p1, p2, text='', at=None, ha='center'):
        """Draw an arrow from p1 to p2 with an optional label at position at."""
        a.annotate('', xy=p2, xytext=p1, arrowprops={'arrowstyle': '->', 'color': INK2, 'lw': 1.0})
        if text:
            a.text(at[0], at[1], text, ha=ha, va='center', fontsize=6.4, color=INK2)

    box(1, 2, 18, 38, 'maze_world', ['Gazebo Harmonic', 'maze.world', '(18 wall boxes)', 'TurtleBot3 Burger', 'LiDAR, IMU,', 'wheel odometry', 'ros_gz_bridge'], SERIES[0])
    box(35, 27, 29, 13, 'maze_navigation: mapping', ['imu_odom.py (wheel + gyro)', 'SLAM Toolbox -> map file', 'explore_maze.py or teleop'], SERIES[1])
    box(35, 2, 29, 18, 'maze_navigation: Nav2', ['map_server + AMCL', 'NavFn planner (Dijkstra / A*)', 'Regulated Pure Pursuit', 'costmaps, recoveries'], SERIES[0])
    box(75, 27, 24.5, 13, 'goal_interface + RViz', ['Nav2 Goal / 2D Goal Pose', 'goal_manager, viva_demo'], SERIES[3])
    box(75, 2, 24.5, 18, 'maze_dqn', ['localization.launch.py', '(map_server + AMCL)', 'dqn_navigation_node', '(Double DQN policy)'], SERIES[2])

    arrow((19.6, 34), (34.4, 34), '/scan /odom /imu', (27, 36.2))
    arrow((19.6, 14), (34.4, 14), '/scan /odom /tf', (27, 16.2))
    arrow((34.4, 8), (19.6, 8), '/cmd_vel', (27, 5.8))
    arrow((49.5, 26.6), (49.5, 20.4), 'saved map', (51, 23.5), ha='left')
    arrow((74.6, 30), (64.4, 17), 'Nav2 Goal', (66, 26.5), ha='left')
    arrow((87, 26.6), (87, 20.4), '/goal_pose', (88.5, 23.5), ha='left')
    # DQN velocity commands go straight to the robot, routed below the Nav2 box
    a.plot([87, 87, 10, 10], [1.6, -3.5, -3.5, 1.6], color=INK2, lw=1.0)
    a.annotate('', xy=(10, 1.6), xytext=(10, -2.5), arrowprops={'arrowstyle': '->', 'color': INK2, 'lw': 1.0})
    a.text(48.5, -6.6, '/cmd_vel from the DQN (Nav2 not running)', ha='center', fontsize=6.4, color=INK2)
    fig.savefig(OUT / 'architecture.png', dpi=220, bbox_inches='tight')
    plt.close(fig)


if __name__ == '__main__':
    architecture()
    results()
    dqn_training()
    print('figures written to', OUT)
