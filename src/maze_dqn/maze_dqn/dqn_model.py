"""
dqn_model.py - the neural network and learning algorithm of the DQN agent.

Contents
  NumpyMLP     : the Q-network, a small fully connected network written in plain NumPy
                 (39 inputs -> 128 -> 128 -> 64 -> 5 outputs, ReLU), so no deep-learning
                 framework is needed on the robot.
  ReplayBuffer : stores past transitions (state, action, reward, next state, done) and returns
                 random mini-batches, which breaks the correlation between consecutive steps.
  DQNAgent     : Double DQN. Chooses actions (epsilon-greedy), learns from mini-batches with a
                 Huber loss and the Adam optimiser, and keeps a slowly updated target network.

Used by
  train_dqn.py            - training in the 2D maze model (maze_env.py)
  dqn_navigation_node.py  - running a trained network on the robot in Gazebo
Weights are saved as .npz files in src/maze_dqn/models/.
"""
import numpy as np
import random
from collections import deque
import json
import os

# Optional: PyTorch is not used by any code path (everything below is NumPy); the import is
# kept only so that a machine with PyTorch installed reports HAS_TORCH = True.
try:
    import torch
    import torch.nn as nn
    import torch.optim as optim
    HAS_TORCH = True
except ImportError:
    HAS_TORCH = False


class NumpyMLP:
    """Q-network: maps a 39-value state to one Q-value per action (5 actions).

    Layers: 39 -> 128 -> 128 -> 64 -> 5, ReLU between layers, linear output. The weights are
    plain NumPy arrays (W1..W4, b1..b4); DQNAgent.train_step computes their gradients by hand.
    """
    def __init__(self, state_dim=39, action_dim=5):
        self.state_dim = state_dim
        self.action_dim = action_dim
        # He initialisation (scaled for ReLU layers)
        self.W1 = np.random.randn(state_dim, 128) * np.sqrt(2.0 / state_dim)
        self.b1 = np.zeros(128)
        self.W2 = np.random.randn(128, 128) * np.sqrt(2.0 / 128)
        self.b2 = np.zeros(128)
        self.W3 = np.random.randn(128, 64) * np.sqrt(2.0 / 128)
        self.b3 = np.zeros(64)
        self.W4 = np.random.randn(64, action_dim) * np.sqrt(2.0 / 64)
        self.b4 = np.zeros(action_dim)

    def forward(self, x):
        """Q-values for a state (shape (39,)) or a batch of states (shape (N, 39)) -> (N, 5)."""
        x = np.atleast_2d(x)
        h1 = np.maximum(0, x @ self.W1 + self.b1)    # ReLU
        h2 = np.maximum(0, h1 @ self.W2 + self.b2)
        h3 = np.maximum(0, h2 @ self.W3 + self.b3)
        out = h3 @ self.W4 + self.b4                 # linear output: one Q-value per action
        return out

    def save_weights(self, path):
        """Write all weights to a compressed .npz file."""
        np.savez_compressed(path, W1=self.W1, b1=self.b1, W2=self.W2, b2=self.b2, W3=self.W3, b3=self.b3, W4=self.W4, b4=self.b4)

    def load_weights(self, path):
        """Read weights written by save_weights."""
        data = np.load(path)
        self.W1 = data['W1']
        self.b1 = data['b1']
        self.W2 = data['W2']
        self.b2 = data['b2']
        self.W3 = data['W3']
        self.b3 = data['b3']
        self.W4 = data['W4']
        self.b4 = data['b4']


class ReplayBuffer:
    """Fixed-size memory of past transitions; the oldest are dropped when it is full."""
    def __init__(self, capacity=50000):
        self.buffer = deque(maxlen=capacity)

    def push(self, state, action, reward, next_state, done):
        """Store one transition."""
        self.buffer.append((state, action, reward, next_state, done))

    def sample(self, batch_size):
        """Random mini-batch as five arrays: states, actions, rewards, next states, done flags."""
        batch = random.sample(self.buffer, batch_size)
        states, actions, rewards, next_states, dones = zip(*batch)
        return (np.array(states, dtype=np.float32),
                np.array(actions, dtype=np.int64),
                np.array(rewards, dtype=np.float32),
                np.array(next_states, dtype=np.float32),
                np.array(dones, dtype=np.float32))

    def __len__(self):
        return len(self.buffer)


class DQNAgent:
    """Double DQN with Huber loss and Adam, implemented in pure NumPy.

    q_net      - the network being trained, used to choose actions
    target_net - a copy of q_net refreshed every target_sync_steps; it provides the learning
                 targets so they do not shift after every update
    epsilon    - probability of a random action while training (exploration); decays once
                 per episode from 1.0 towards epsilon_min
    """
    def __init__(self, state_dim=39, action_dim=5, lr=0.001, gamma=0.99, epsilon=1.0, epsilon_decay=0.995, epsilon_min=0.05,
                 target_sync_steps=1000, double_dqn=True):
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.lr = lr                          # Adam learning rate
        self.gamma = gamma                    # discount factor for future rewards
        self.epsilon = epsilon
        self.epsilon_decay = epsilon_decay
        self.epsilon_min = epsilon_min
        self.target_sync_steps = target_sync_steps
        self.double_dqn = double_dqn
        self.train_steps = 0

        self.memory = ReplayBuffer(100000)
        self.q_net = NumpyMLP(state_dim, action_dim)
        self.target_net = NumpyMLP(state_dim, action_dim)
        self.sync_target()

        # Adam optimiser state: first (m) and second (v) moment estimates per weight array
        self._names = ['W1', 'b1', 'W2', 'b2', 'W3', 'b3', 'W4', 'b4']
        self._m = {n: np.zeros_like(getattr(self.q_net, n)) for n in self._names}
        self._v = {n: np.zeros_like(getattr(self.q_net, n)) for n in self._names}
        self._t = 0

    def sync_target(self):
        """Copy the online network's weights into the target network."""
        for n in ['W1', 'b1', 'W2', 'b2', 'W3', 'b3', 'W4', 'b4']:
            setattr(self.target_net, n, np.copy(getattr(self.q_net, n)))

    def select_action(self, state, evaluate=False):
        """Index of the action to take. evaluate=True always takes the best action (no
        exploration); this is what the robot uses."""
        if not evaluate and np.random.rand() < self.epsilon:
            return np.random.randint(self.action_dim)
        q_vals = self.q_net.forward(state)[0]
        return int(np.argmax(q_vals))

    def decay_epsilon(self):
        """Called once per EPISODE (previously this ran every gradient step, which ended
        exploration after ~300 steps)."""
        self.epsilon = max(self.epsilon_min, self.epsilon * self.epsilon_decay)

    def train_step(self, batch_size=64):
        """One gradient update from a random mini-batch; returns the loss (0 while the buffer
        holds fewer than 1000 transitions, so early learning is not dominated by a few
        episodes)."""
        if len(self.memory) < max(batch_size, 1000):
            return 0.0

        states, actions, rewards, next_states, dones = self.memory.sample(batch_size)
        net = self.q_net

        # Forward pass Q(s, :), keeping the hidden layers for back-propagation
        h1 = np.maximum(0, states @ net.W1 + net.b1)
        h2 = np.maximum(0, h1 @ net.W2 + net.b2)
        h3 = np.maximum(0, h2 @ net.W3 + net.b3)
        q_vals = h3 @ net.W4 + net.b4

        # Learning target y = r + gamma * Q_target(s', a'), with no future value at episode end
        next_q_target = self.target_net.forward(next_states)
        if self.double_dqn:
            # Double DQN: online net selects the action, target net evaluates it
            next_a = np.argmax(net.forward(next_states), axis=1)
            next_q = next_q_target[np.arange(batch_size), next_a]
        else:
            next_q = np.max(next_q_target, axis=1)
        y = rewards + self.gamma * next_q * (1.0 - dones)

        # Temporal-difference error of the actions actually taken, and the Huber loss
        idx = np.arange(batch_size)
        td = q_vals[idx, actions] - y
        loss = float(np.mean(np.where(np.abs(td) < 1.0, 0.5 * td**2, np.abs(td) - 0.5)))

        # Huber loss gradient, only on the taken action
        error = np.zeros_like(q_vals)
        error[idx, actions] = np.clip(td, -1.0, 1.0) / batch_size

        # Back-propagation through the four layers (ReLU gradient is 0 where the unit was off)
        grads = {}
        grads['W4'] = h3.T @ error
        grads['b4'] = np.sum(error, axis=0)
        dh3 = error @ net.W4.T
        dh3[h3 <= 0] = 0
        grads['W3'] = h2.T @ dh3
        grads['b3'] = np.sum(dh3, axis=0)
        dh2 = dh3 @ net.W3.T
        dh2[h2 <= 0] = 0
        grads['W2'] = h1.T @ dh2
        grads['b2'] = np.sum(dh2, axis=0)
        dh1 = dh2 @ net.W2.T
        dh1[h1 <= 0] = 0
        grads['W1'] = states.T @ dh1
        grads['b1'] = np.sum(dh1, axis=0)

        # Adam update with bias correction
        self._t += 1
        b1, b2, eps = 0.9, 0.999, 1e-8
        for n in self._names:
            g = grads[n]
            self._m[n] = b1 * self._m[n] + (1 - b1) * g
            self._v[n] = b2 * self._v[n] + (1 - b2) * g * g
            m_hat = self._m[n] / (1 - b1 ** self._t)
            v_hat = self._v[n] / (1 - b2 ** self._t)
            setattr(net, n, getattr(net, n) - self.lr * m_hat / (np.sqrt(v_hat) + eps))

        # Refresh the target network every target_sync_steps updates
        self.train_steps += 1
        if self.train_steps % self.target_sync_steps == 0:
            self.sync_target()

        return loss

    def save(self, filepath):
        """Save the online network's weights (.npz)."""
        self.q_net.save_weights(filepath)

    def load(self, filepath):
        """Load weights into both networks (used before evaluation or on the robot)."""
        self.q_net.load_weights(filepath)
        self.sync_target()
