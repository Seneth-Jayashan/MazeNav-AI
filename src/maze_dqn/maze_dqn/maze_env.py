"""
maze_env.py - 2D training environment for the DQN, plus helpers shared with the ROS node.

The real robot runs in Gazebo, which is far too slow to train a DQN for thousands of episodes.
This file is a fast 2D model of the same maze (the same 18 walls as maze.world): a unicycle
robot moving at 10 Hz and a simulated 360-beam LiDAR found by ray casting against the walls.

Contents
  constants        - LiDAR sectors, clipping, cell size and the 5 actions; shared with the ROS
                     node so the state on the robot is built exactly as in training
  make_actions     - the 5 actions with a chosen in-place turn rate
  downsample_scan  - 360 LiDAR beams -> 36 sectors (minimum range per 10 degree sector)
  make_state       - builds the 39-value state the network sees
  MazeEnvironment  - reset()/step() environment with the reward function, the 6x6 cell graph
                     of the maze and the 'subgoal' route used by the subgoal DQN

Used by train_dqn.py (training), dqn_navigation_node.py (on the robot), and the repository
scripts explore_maze.py and collect_metrics.py (for the cell graph and the wall list).
"""
import numpy as np
import math
from collections import deque

# Shared constants (used by both the training environment and the ROS 2 node,
# so the state seen at deployment is normalised exactly as during training)
NUM_BEAMS = 36
MAX_LIDAR_RANGE = 3.5
STATE_LIDAR_CLIP = 1.5   # beams are clipped to 1.5 m: more resolution in 0.5 m corridors
MAX_GOAL_DIST = 4.24     # arena diagonal
CELL = 0.5
CELL_CENTRES = [-1.25, -0.75, -0.25, 0.25, 0.75, 1.25]
WAYPOINT_REACHED = 0.2   # [m] subgoal mode: advance to the next cell centre within this radius

# Hard turns rotate in place: with v=0.08 the turning radius (0.107 m) is too large to turn
# around in a 0.45 m corridor without crossing the 0.12 m collision distance.
ACTIONS = [
    (0.18,  0.0),   # 0: Forward
    (0.0,   1.0),   # 1: Rotate Left
    (0.0,  -1.0),   # 2: Rotate Right
    (0.14,  0.35),  # 3: Soft Left
    (0.14, -0.35),  # 4: Soft Right
]


def make_actions(turn_rate=1.0):
    """The 5 actions with a given in-place turn rate (the trained 'safe' model uses 0.5 rad/s,
    below the speed at which the Burger's wheels slip in Gazebo)."""
    return [(v, turn_rate * (1 if w > 0 else -1) if v == 0.0 and w != 0.0 else w) for v, w in ACTIONS]


def downsample_scan(ranges, angle_min, angle_increment):
    """Min-pools a full scan into NUM_BEAMS sectors of 10 deg centred on 0, 10, ..., 350 deg.

    Taking the minimum of each sector (instead of one ray every 10 deg) means thin
    obstacles such as wall ends cannot fall between the beams.
    """
    ranges = np.asarray(ranges, dtype=np.float32)
    angles = angle_min + angle_increment * np.arange(len(ranges))
    sector_width = 2.0 * np.pi / NUM_BEAMS
    sectors = np.round(angles / sector_width).astype(int) % NUM_BEAMS
    out = np.full(NUM_BEAMS, MAX_LIDAR_RANGE, dtype=np.float32)
    np.minimum.at(out, sectors, ranges)
    return np.clip(out, 0.05, MAX_LIDAR_RANGE)


def make_state(lidar, dist, angle_diff, last_action):
    """Builds the 39-D DQN state: 36 LiDAR beams + goal distance + goal bearing + last action."""
    state = np.zeros(NUM_BEAMS + 3, dtype=np.float32)
    state[:NUM_BEAMS] = np.clip(lidar, 0.0, STATE_LIDAR_CLIP) / STATE_LIDAR_CLIP
    state[NUM_BEAMS] = min(dist / MAX_GOAL_DIST, 1.0)
    state[NUM_BEAMS + 1] = angle_diff / np.pi
    state[NUM_BEAMS + 2] = last_action / float(len(ACTIONS) - 1)
    return state


class MazeEnvironment:
    """
    Kinematic 2D simulation environment matching the exact 18 walls of Figure 1.
    Allows high-speed, verified DQN reinforcement learning training.

    subgoal_mode=False: the goal in the state is the final goal (pure reactive DQN).
    subgoal_mode=True:  the goal in the state is the centre of the next maze cell on the
                        shortest grid path (DQN as a local controller under a grid planner).

    collision_dist, turn_rate and wall_penalty_dist are 0.12 m, 1.0 rad/s and 0.18 m for the
    original models; the retrained 'safe' model uses 0.15 m, 0.5 rad/s and 0.20 m.

    Main methods: reset(start, goal, yaw) -> state, and step(action) -> (state, reward, done,
    info). The reward is +30 at the goal, -20 for a collision, -5 at timeout, and otherwise
    10 x (progress along the maze corridors) - 0.05, with -0.2 per step close to a wall.
    """
    def __init__(self, subgoal_mode=False, collision_penalty=-20.0, goal_reward=30.0,
                 collision_dist=0.12, turn_rate=1.0, wall_penalty_dist=0.18):
        # 18 walls as (x_min, x_max, y_min, y_max)
        h_merged = [
          ((0.00, 1.50), (3.00, 0.05)),
          ((0.25, 0.50), (1.50, 0.05)),
          ((1.00, 0.00), (1.00, 0.05)),
          ((-0.25, -0.50), (1.50, 0.05)),
          ((-1.25, -1.00), (0.50, 0.05)),
          ((0.25, -1.00), (1.50, 0.05)),
          ((0.00, -1.50), (3.00, 0.05)),
        ]
        v_merged = [
          ((-1.50, 0.00), (0.05, 3.00)),
          ((-1.00, 0.25), (0.05, 1.50)),
          ((-0.50, 1.25), (0.05, 0.50)),
          ((-0.50, 0.25), (0.05, 0.50)),
          ((0.00, 0.75), (0.05, 0.50)),
          ((0.00, -0.25), (0.05, 0.50)),
          ((0.50, 1.25), (0.05, 0.50)),
          ((0.50, 0.25), (0.05, 0.50)),
          ((1.00, 0.75), (0.05, 0.50)),
          ((1.00, -0.50), (0.05, 1.00)),
          ((1.50, 0.00), (0.05, 3.00)),
        ]
        self.walls = []
        for (x, y), (sx, sy) in h_merged:
            self.walls.append((x - sx/2, x + sx/2, y - sy/2, y + sy/2))
        for (x, y), (sx, sy) in v_merged:
            self.walls.append((x - sx/2, x + sx/2, y - sy/2, y + sy/2))
        self._w = np.array(self.walls, dtype=np.float64)  # (W, 4)

        # Action definitions (linear_vel m/s, angular_vel rad/s)
        self.actions = make_actions(turn_rate)
        self.action_dim = len(self.actions)
        self.num_beams = NUM_BEAMS
        self.max_lidar_range = MAX_LIDAR_RANGE
        self.robot_radius = 0.09 # TurtleBot3 Burger radius
        self.collision_dist = collision_dist        # [m] robot centre to wall = collision
        self.wall_penalty_dist = wall_penalty_dist  # [m] closer than this: small penalty each step
        self.goal_reach_dist = 0.15
        self.dt = 0.1 # 10 Hz control loop
        self.subgoal_mode = subgoal_mode
        self.collision_penalty = collision_penalty
        self.goal_reward = goal_reward

        self._build_grid()
        self.reset()

    # ------------------------------------------------------------------ grid
    def _segment_blocked(self, p, q):
        """True if the straight segment p->q crosses any wall rectangle."""
        for x1, x2, y1, y2 in self.walls:
            tmin, tmax = 0.0, 1.0
            ok = True
            for a, b, lo, hi in ((p[0], q[0], x1, x2), (p[1], q[1], y1, y2)):
                d = b - a
                if abs(d) < 1e-9:
                    if a < lo or a > hi:
                        ok = False
                        break
                else:
                    t1, t2 = (lo - a) / d, (hi - a) / d
                    tmin, tmax = max(tmin, min(t1, t2)), min(tmax, max(t1, t2))
            if ok and tmin <= tmax:
                return True
        return False

    def _build_grid(self):
        """6x6 cell graph derived from the wall geometry, plus all-pairs BFS distances."""
        self.cells = [(c, r) for r in range(6) for c in range(6)]
        self.adj = {cell: [] for cell in self.cells}
        for (c, r) in self.cells:
            for dc, dr in ((1, 0), (0, 1)):
                nc, nr = c + dc, r + dr
                if nc > 5 or nr > 5:
                    continue
                if not self._segment_blocked(self.cell_centre((c, r)), self.cell_centre((nc, nr))):
                    self.adj[(c, r)].append((nc, nr))
                    self.adj[(nc, nr)].append((c, r))
        # bfs[goal][cell] = number of cell steps from cell to goal
        self.bfs = {}
        for g in self.cells:
            d = {g: 0}
            q = deque([g])
            while q:
                u = q.popleft()
                for v in self.adj[u]:
                    if v not in d:
                        d[v] = d[u] + 1
                        q.append(v)
            self.bfs[g] = d

    @staticmethod
    def cell_centre(cell):
        """World (x, y) of the centre of cell (column, row); (0, 0) is the bottom-left cell."""
        return (CELL_CENTRES[cell[0]], CELL_CENTRES[cell[1]])

    @staticmethod
    def cell_of(x, y):
        """Cell (column, row) containing world point (x, y), clamped to the 6x6 grid."""
        c = int(min(5, max(0, math.floor((x + 1.5) / CELL))))
        r = int(min(5, max(0, math.floor((y + 1.5) / CELL))))
        return (c, r)

    def next_cell(self, cell, goal_cell):
        """Next cell on a shortest grid path from cell to goal_cell."""
        if cell == goal_cell:
            return cell
        dist = self.bfs[goal_cell]
        best = min(self.adj[cell], key=lambda n: dist.get(n, 1e9))
        return best

    def geodesic(self, x, y, gx, gy):
        """Shortest path distance through the maze (cell graph), used only for reward shaping."""
        cell, gcell = self.cell_of(x, y), self.cell_of(gx, gy)
        if cell == gcell:
            return math.hypot(gx - x, gy - y)
        nx, ny = self.cell_centre(self.next_cell(cell, gcell))
        gcx, gcy = self.cell_centre(gcell)
        return (math.hypot(nx - x, ny - y) + self.bfs[gcell][self.next_cell(cell, gcell)] * CELL
                + math.hypot(gx - gcx, gy - gcy))

    def set_goal(self, gx, gy):
        """Sets the goal and resets the subgoal waypoint."""
        self.goal_x, self.goal_y = gx, gy
        self.waypoint = None

    def target(self):
        """Point the agent is steered towards (final goal or next-cell subgoal).

        In subgoal mode the waypoint is the centre of the next cell on the grid path and is
        only advanced once the robot is within WAYPOINT_REACHED of it. Switching as soon as
        the robot crossed a cell boundary made it cut corners and clip wall ends.
        """
        if not self.subgoal_mode:
            return self.goal_x, self.goal_y
        gcell = self.cell_of(self.goal_x, self.goal_y)
        cell = self.cell_of(self.x, self.y)
        wp = getattr(self, 'waypoint', None)
        # (Re)initialise if unset, or if the robot left the path (not in the waypoint cell
        # nor in a cell adjacent to it)
        if wp is None or (cell != wp and cell not in self.adj[wp]):
            wp = self.next_cell(cell, gcell)
        while wp != gcell:
            wx, wy = self.cell_centre(wp)
            if math.hypot(wx - self.x, wy - self.y) >= WAYPOINT_REACHED:
                break
            wp = self.next_cell(wp, gcell)
        self.waypoint = wp
        if wp == gcell:
            return self.goal_x, self.goal_y
        return self.cell_centre(wp)

    # ------------------------------------------------------------- episodes
    def reset(self, start_pos=(-0.75, -1.25), goal_pos=(0.75, 0.75), start_yaw=0.0, max_steps=None):
        """Start a new episode: place the robot, set the goal and return the first state."""
        self.x, self.y = start_pos
        self.yaw = start_yaw
        self.set_goal(*goal_pos)
        self.step_count = 0
        self.prev_geo = self.geodesic(self.x, self.y, self.goal_x, self.goal_y)
        if max_steps is None:
            # 2.5x the time needed at full speed along the maze path, at least 150 steps
            max_steps = max(150, int(2.5 * self.prev_geo / (ACTIONS[0][0] * self.dt)))
        self.max_steps = max_steps
        self.last_action = 0
        return self._get_state()

    def _get_lidar(self):
        """360-ray scan (1 deg resolution, like the LDS-01) pooled into 36 sectors."""
        rel = np.linspace(0, 2*np.pi, 360, endpoint=False)
        angles = self.yaw + rel
        ca = np.cos(angles)[:, None]
        sa = np.sin(angles)[:, None]
        x1, x2, y1, y2 = self._w[:, 0], self._w[:, 1], self._w[:, 2], self._w[:, 3]

        # Vectorised slab method for 2D AABBs: rays (B, 1) x walls (W,)
        with np.errstate(divide='ignore', invalid='ignore'):
            inv_x = np.where(np.abs(ca) > 1e-9, 1.0 / ca, np.inf)
            inv_y = np.where(np.abs(sa) > 1e-9, 1.0 / sa, np.inf)
            tx1, tx2 = (x1 - self.x) * inv_x, (x2 - self.x) * inv_x
            ty1, ty2 = (y1 - self.y) * inv_y, (y2 - self.y) * inv_y
            # parallel rays: inside the slab -> (-inf, inf), outside -> no hit
            in_x = (self.x >= x1) & (self.x <= x2)
            in_y = (self.y >= y1) & (self.y <= y2)
            tx_lo = np.where(np.isfinite(inv_x), np.minimum(tx1, tx2), np.where(in_x, -np.inf, np.inf))
            tx_hi = np.where(np.isfinite(inv_x), np.maximum(tx1, tx2), np.where(in_x, np.inf, -np.inf))
            ty_lo = np.where(np.isfinite(inv_y), np.minimum(ty1, ty2), np.where(in_y, -np.inf, np.inf))
            ty_hi = np.where(np.isfinite(inv_y), np.maximum(ty1, ty2), np.where(in_y, np.inf, -np.inf))
        tmin = np.maximum(tx_lo, ty_lo)
        tmax = np.minimum(tx_hi, ty_hi)
        hit = (tmax >= tmin) & (tmax > 0)
        t = np.where(tmin > 0, tmin, tmax)
        t = np.where(hit & (t > 0), t, self.max_lidar_range)
        ranges = np.clip(np.min(t, axis=1), 0.05, self.max_lidar_range)
        return downsample_scan(ranges, 0.0, 2*np.pi / 360)

    def _wall_clearance(self):
        """Exact distance from the robot centre to the nearest wall."""
        dx = np.maximum(np.maximum(self._w[:, 0] - self.x, 0.0), self.x - self._w[:, 1])
        dy = np.maximum(np.maximum(self._w[:, 2] - self.y, 0.0), self.y - self._w[:, 3])
        return float(np.min(np.hypot(dx, dy)))

    def _get_state(self):
        """Current 39-value state: simulated LiDAR + distance/bearing to the target + last action."""
        self.lidar = self._get_lidar()
        tx, ty = self.target()
        dx = tx - self.x
        dy = ty - self.y
        dist = math.hypot(dx, dy)
        angle_to_goal = math.atan2(dy, dx)
        angle_diff = (angle_to_goal - self.yaw + np.pi) % (2 * np.pi) - np.pi
        return make_state(self.lidar, dist, angle_diff, self.last_action)

    def step(self, action_idx):
        """Apply one action for one 0.1 s step; returns (next state, reward, done, info).
        info holds 'success', 'collision', 'dist' (straight-line distance to the goal) and 'steps'."""
        self.step_count += 1
        self.last_action = action_idx
        v, w = self.actions[action_idx]

        # Integrate differential drive kinematics
        self.x += v * math.cos(self.yaw) * self.dt
        self.y += v * math.sin(self.yaw) * self.dt
        self.yaw = (self.yaw + w * self.dt + np.pi) % (2 * np.pi) - np.pi

        state = self._get_state()
        min_scan = self._wall_clearance()

        dist = math.hypot(self.goal_x - self.x, self.goal_y - self.y)
        geo = self.geodesic(self.x, self.y, self.goal_x, self.goal_y)

        # Rewards
        reward = 0.0
        done = False
        success = False
        collision = False

        # Collision check
        if min_scan < self.collision_dist or abs(self.x) > 1.48 or abs(self.y) > 1.48:
            reward = self.collision_penalty
            done = True
            collision = True
        elif dist < self.goal_reach_dist:
            reward = self.goal_reward
            done = True
            success = True
        elif self.step_count >= self.max_steps:
            reward = -5.0
            done = True
        else:
            # Progress along the maze path (potential-based shaping) + small living penalty
            reward = (self.prev_geo - geo) * 10.0 - 0.05
            # Mild penalty for hugging walls
            if min_scan < self.wall_penalty_dist:
                reward -= 0.2

        self.prev_geo = geo
        info = {'success': success, 'collision': collision, 'dist': dist, 'steps': self.step_count}
        return state, reward, done, info
