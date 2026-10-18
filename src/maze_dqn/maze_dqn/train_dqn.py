#!/usr/bin/env python3
"""
train_dqn.py - trains a DQN agent in the 2D maze model (maze_env.py).

How training works
  - Each episode picks a random start cell and a goal a few cells away (curriculum): it starts
    with goals 1 cell away and moves to longer routes once 70 % of the last 100 episodes at the
    current level succeed.
  - Every --eval-every episodes the greedy policy is tested on the three fixed test goals and on
    50 random full-maze start/goal pairs; the best evaluation so far is saved as the weights.
  - Logs: <tag>_training_log.csv (one row per episode) and <tag>_eval_log.csv (one row per
    evaluation); <tag>_weights.json records the settings, which the ROS node reads.

Run from src/maze_dqn:
  python3 -m maze_dqn.train_dqn --subgoal --episodes 3000                       # original subgoal DQN
  python3 -m maze_dqn.train_dqn --subgoal --episodes 4000 --seed 1 \\
      --collision-dist 0.15 --turn-rate 0.5 --wall-penalty-dist 0.20 --tag safe  # retrained 'safe' DQN
"""
import argparse
import csv
import json
import os
import random
import time
from pathlib import Path

import numpy as np
from maze_dqn.maze_env import MazeEnvironment
from maze_dqn.dqn_model import DQNAgent

# Default output: the package's models/ directory in the source tree
DEFAULT_MODELS_DIR = Path(__file__).resolve().parents[1] / 'models'

START_POS = (-0.75, -1.25)   # robot spawn in Gazebo
TEST_GOALS = [                # the three benchmark goals G1-G3 (called A-C here)
    ('Goal A', (-0.25, -0.75)),
    ('Goal B', (0.75, 0.75)),
    ('Goal C', (1.25, 1.25)),
]


def run_episode(env, agent, start, goal, yaw=0.0, evaluate=False, batch_size=64):
    """Run one episode. While training (evaluate=False) every step is stored in the replay
    buffer and followed by one gradient update. Returns (total reward, last info, mean loss)."""
    state = env.reset(start_pos=start, goal_pos=goal, start_yaw=yaw)
    total_reward = 0.0
    losses = []
    done = False
    info = {}
    while not done:
        action = agent.select_action(state, evaluate=evaluate)
        next_state, reward, done, info = env.step(action)
        total_reward += reward
        if not evaluate:
            agent.memory.push(state, action, reward, next_state, done)
            loss = agent.train_step(batch_size=batch_size)
            if loss > 0:
                losses.append(loss)
        state = next_state
    return total_reward, info, (float(np.mean(losses)) if losses else 0.0)


def sample_task(env, level, rng):
    """Curriculum: random start cell and a goal cell between level-2 and level cells away."""
    while True:
        start_cell = rng.choice(env.cells)
        cands = [c for c, d in env.bfs[start_cell].items() if max(1, level - 2) <= d <= level]
        if cands:
            break
    goal_cell = rng.choice(cands)
    sx, sy = env.cell_centre(start_cell)
    start = (sx + rng.uniform(-0.05, 0.05), sy + rng.uniform(-0.05, 0.05))
    return start, env.cell_centre(goal_cell), rng.uniform(-np.pi, np.pi), env.bfs[start_cell][goal_cell]


def evaluate(env, agent, rng, n_random=50):
    """Greedy test: success (0/1) on each fixed test goal from the spawn, and the success rate
    on n_random random start/goal pairs anywhere in the maze."""
    test = []
    for name, goal in TEST_GOALS:
        _, info, _ = run_episode(env, agent, START_POS, goal, evaluate=True)
        test.append(int(info['success']))
    max_level = max(max(d.values()) for d in env.bfs.values())
    succ = 0
    for _ in range(n_random):
        start, goal, yaw, _ = sample_task(env, max_level, rng)
        _, info, _ = run_episode(env, agent, start, goal, yaw=yaw, evaluate=True)
        succ += int(info['success'])
    return test, succ / n_random


def main():
    """Parse the options, train, log every episode and keep the best-evaluated weights."""
    parser = argparse.ArgumentParser(description='Train the maze DQN agent in the 2D kinematic maze environment')
    parser.add_argument('--episodes', type=int, default=3000)
    parser.add_argument('--subgoal', action='store_true',
                        help='Steer towards the next maze cell on the grid path instead of the final goal')
    parser.add_argument('--out-dir', type=str, default=str(DEFAULT_MODELS_DIR))
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--eval-every', type=int, default=100)
    parser.add_argument('--collision-dist', type=float, default=0.12,
                        help='robot centre to wall distance counted as a collision [m]')
    parser.add_argument('--turn-rate', type=float, default=1.0, help='in-place turn rate of the rotate actions [rad/s]')
    parser.add_argument('--wall-penalty-dist', type=float, default=0.18)
    parser.add_argument('--tag', type=str, default='',
                        help='name suffix for the output files, e.g. "safe" -> dqn_subgoal_safe_weights.npz')
    args = parser.parse_args()

    print("="*65)
    print("       STARTING DEEP Q-LEARNING (DQN) TRAINING PIPELINE")
    print("       IE4060 Autonomous Navigation - TurtleBot3 Burger")
    print(f"       mode={'subgoal' if args.subgoal else 'final-goal'}  episodes={args.episodes}  seed={args.seed}")
    print("="*65)

    np.random.seed(args.seed)
    random.seed(args.seed)
    rng = random.Random(args.seed + 1)

    env = MazeEnvironment(subgoal_mode=args.subgoal, collision_dist=args.collision_dist,
                          turn_rate=args.turn_rate, wall_penalty_dist=args.wall_penalty_dist)
    agent = DQNAgent(state_dim=39, action_dim=5, lr=0.0005, gamma=0.98, epsilon=1.0,
                     epsilon_decay=0.998, epsilon_min=0.05)

    tag = ('dqn_subgoal' if args.subgoal else 'dqn') + (f'_{args.tag}' if args.tag else '')
    save_dir = Path(args.out_dir)
    save_dir.mkdir(parents=True, exist_ok=True)
    log_csv = save_dir / f'{tag}_training_log.csv'
    eval_csv = save_dir / f'{tag}_eval_log.csv'
    weights_path = save_dir / f'{tag}_weights.npz'
    # The node reads this to use the same actions as in training
    with open(save_dir / f'{tag}_weights.json', 'w') as f:
        json.dump({'subgoal': args.subgoal, 'collision_dist': args.collision_dist, 'turn_rate': args.turn_rate,
                   'wall_penalty_dist': args.wall_penalty_dist, 'episodes': args.episodes, 'seed': args.seed}, f, indent=2)

    max_level = max(max(d.values()) for d in env.bfs.values())
    level = 1
    level_hist = []
    best_score = (-1, -1.0)

    with open(log_csv, 'w', newline='') as f, open(eval_csv, 'w', newline='') as fe:
        writer = csv.writer(f)
        writer.writerow(['Episode', 'Level', 'Reward', 'Steps', 'Success', 'Collision', 'Epsilon', 'Loss'])
        ewriter = csv.writer(fe)
        ewriter.writerow(['Episode', 'Level', 'TestGoalA', 'TestGoalB', 'TestGoalC', 'RandomFullMazeSuccess'])

        start_time = time.time()
        for ep in range(1, args.episodes + 1):
            start, goal, yaw, _ = sample_task(env, level, rng)
            total_reward, info, mean_loss = run_episode(env, agent, start, goal, yaw=yaw)
            agent.decay_epsilon()

            level_hist.append(int(info['success']))
            writer.writerow([ep, level, f'{total_reward:.2f}', info['steps'], int(info['success']),
                             int(info['collision']), f'{agent.epsilon:.3f}', f'{mean_loss:.4f}'])

            # Promote the curriculum once 70% of the last 100 episodes at this level succeed
            if len(level_hist) >= 100 and np.mean(level_hist[-100:]) >= 0.7 and level < max_level:
                level += 1
                level_hist = []
                print(f"  -> curriculum level {level} at episode {ep}")

            if ep % args.eval_every == 0 or ep == args.episodes:
                test, rand_succ = evaluate(env, agent, rng)
                ewriter.writerow([ep, level] + test + [f'{rand_succ:.2f}'])
                fe.flush()
                f.flush()
                score = (sum(test), rand_succ)
                if score >= best_score:
                    best_score = score
                    agent.save(str(weights_path))
                print(f"Episode {ep:4d}/{args.episodes} | Level {level:2d} | Train succ(last100): "
                      f"{np.mean(level_hist[-100:]) * 100 if level_hist else 0:5.1f}% | Eps {agent.epsilon:.3f} | "
                      f"Test A/B/C {test} | Random full-maze {rand_succ * 100:5.1f}% | {time.time() - start_time:6.0f}s",
                      flush=True)

    elapsed = time.time() - start_time
    print("-"*65)
    print(f"Training complete in {elapsed:.2f} seconds!")
    print(f"Best weights (test goals {best_score[0]}/3, random {best_score[1] * 100:.0f}%) saved to: {weights_path}")
    print(f"Training logs saved to: {log_csv} and {eval_csv}")
    print("="*65 + "\n")


if __name__ == '__main__':
    main()
