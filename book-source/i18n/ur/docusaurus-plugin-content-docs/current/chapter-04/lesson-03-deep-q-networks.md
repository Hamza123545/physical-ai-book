---
title: "Lesson 4.3: Deep Q-Networks (DQN)"
description: "Scale Q-learning to high-dimensional states with neural networks"
chapter: 4
lesson: 3
estimated_time: 70
cefr_level: "B2"
blooms_level: "Apply"
digcomp_level: 6
generated_by: "claude-sonnet-4-5-20250929"
source_spec: "specs/002-physical-ai-textbook/spec.md"
created: "2025-11-29"
last_modified: "2025-11-29"
git_author: "hswat"
workflow: "/sp.implement"
version: "1.0"
prerequisites:
  - "chapter-04-lesson-02"
has_interactive_python: true
interactive_python_count: 4
has_try_with_ai: true
try_with_ai_count: 2
tags: ["dqn", "deep-learning", "experience-replay", "target-network"]
---

# Lesson 4.3: Deep Q-Networks (DQN)

## 🎯 Learning Objectives

<LearningObjectives
  cefr_level="B2"
  objectives={[
    {text: "Implement neural network Q-function approximator", blooms_level: "Apply", assessment_method: "Network exercise"},
    {text: "Apply experience replay for stable learning", blooms_level: "Apply", assessment_method: "Replay buffer exercise"},
    {text: "Use target network to reduce overestimation", blooms_level: "Apply", assessment_method: "DQN exercise"}
  ]}
/>

## Introduction

**Problem**: Q-tables don't scale to large/continuous state spaces.

**Solution**: Approximate $Q(s,a)$ with neural network $Q(s,a; \theta)$.

**DQN innovations**:
1. Experience replay
2. Target network

---

## 1. Neural Q-Network

Replace Q-table with neural network:
- Input: state $s$
- Output: Q-values for all actions

---

## 2. Experience Replay

Store transitions $(s, a, r, s')$ in replay buffer. Sample random minibatch for training.

**Why?** Breaks correlation, stabilizes learning.

---

## 3. Target Network

Use separate target network $Q(s,a; \theta^-)$ for TD target:
$$y = r + \gamma \max_{a'} Q(s', a'; \theta^-)$$

Update $\theta^-$ every N steps.

---

## Exercises

### Exercise 4.3.1: Q-Network Architecture

<InteractivePython
  id="ex-4-3-1"
  title="Build Q-Network (Simplified)"
  starterCode={`import numpy as np

class QNetwork:
    def __init__(self, state_dim, n_actions, hidden=64):
        """Simple Q-network."""
        # TODO: Initialize weights for 2-layer network
        self.W1 = None
        self.W2 = None

    def forward(self, state):
        """Forward pass: state -> Q-values."""
        # TODO: h = relu(state @ W1)
        #       q = h @ W2
        pass

# Test
net = QNetwork(state_dim=4, n_actions=2, hidden=64)
q_values = net.forward(np.array([0.1, 0.2, 0.3, 0.4]))
print(f"Q-values shape: {q_values.shape if q_values is not None else 'None'}")
`}
  hints={[
    "self.W1 = np.random.randn(state_dim, hidden) * 0.01",
    "self.W2 = np.random.randn(hidden, n_actions) * 0.01",
    "h = np.maximum(0, state @ self.W1)  # ReLU",
    "return h @ self.W2"
  ]}
/>

---

### Exercise 4.3.2: Experience Replay Buffer

<InteractivePython
  id="ex-4-3-2"
  title="Replay Buffer"
  starterCode={`import numpy as np
from collections import deque

class ReplayBuffer:
    def __init__(self, capacity=10000):
        self.buffer = deque(maxlen=capacity)

    def add(self, state, action, reward, next_state, done):
        """Store transition."""
        # TODO: Append (s, a, r, s', done) to buffer
        pass

    def sample(self, batch_size):
        """Sample random minibatch."""
        # TODO: Random sample batch_size transitions
        # Return as separate arrays
        pass

# Test
buffer = ReplayBuffer(capacity=100)
buffer.add([0.1], 0, 1.0, [0.2], False)
print(f"Buffer size: {len(buffer.buffer)}")
`}
  hints={[
    "self.buffer.append((state, action, reward, next_state, done))",
    "indices = np.random.choice(len(self.buffer), batch_size)",
    "batch = [self.buffer[i] for i in indices]",
    "states, actions, rewards, next_states, dones = zip(*batch)",
    "return np.array(states), np.array(actions), ..."
  ]}
/>

---

### Exercise 4.3.3: DQN Update

<InteractivePython
  id="ex-4-3-3"
  title="DQN Training Step"
  starterCode={`import numpy as np

def dqn_update(q_net, target_net, batch, gamma=0.99):
    """Compute DQN loss and gradient."""
    states, actions, rewards, next_states, dones = batch

    # TODO:
    # 1. Compute target: y = r + gamma * max Q_target(s', a')
    # 2. Compute current Q: Q(s, a)
    # 3. Loss = (Q - y)^2

    pass

# Placeholder test
print("DQN update function defined")
`}
  hints={[
    "next_q_values = target_net.forward(next_states)",
    "max_next_q = np.max(next_q_values, axis=1)",
    "targets = rewards + gamma * max_next_q * (1 - dones)",
    "current_q = q_net.forward(states)[range(len(actions)), actions]",
    "loss = np.mean((current_q - targets) ** 2)"
  ]}
/>

---

### Exercise 4.3.4: Train DQN Agent

<InteractivePython
  id="ex-4-3-4"
  title="Full DQN Training Loop"
  starterCode={`import numpy as np

def train_dqn(env, episodes=100, batch_size=32, target_update=10):
    """Train DQN agent."""
    q_net = QNetwork(env.state_dim, env.n_actions)
    target_net = QNetwork(env.state_dim, env.n_actions)
    buffer = ReplayBuffer(capacity=10000)

    # TODO: Training loop
    # 1. Collect experience
    # 2. Sample from buffer
    # 3. Update Q-network
    # 4. Update target network every N episodes

    return q_net

# Placeholder
print("DQN training loop defined")
`}
  hints={[
    "for episode in range(episodes):",
    "  state = env.reset()",
    "  while not done:",
    "    action = epsilon_greedy(q_net.forward(state), epsilon)",
    "    next_state, reward, done = env.step(action)",
    "    buffer.add(state, action, reward, next_state, done)",
    "    if len(buffer) > batch_size:",
    "      batch = buffer.sample(batch_size)",
    "      dqn_update(q_net, target_net, batch)",
    "  if episode % target_update == 0:",
    "    target_net = copy(q_net)"
  ]}
/>

---

## Try With AI

### TryWithAI 4.3.1: DQN Architecture Design

<TryWithAI
  id="tryai-4-3-1"
  title="Design Network for Complex States"
  role="Copilot"
  scenario="You need to scale DQN to image inputs (e.g., Atari games)."
  yourTask="Implement Exercise 4.3.1. Consider how to handle 84x84 pixel images."
  aiPromptTemplate="My DQN uses fully-connected layers for state=[x, y, vx, vy]. How do I modify it for image inputs (84x84x4)? Should I use convolutions? Here's my current architecture: [paste]. Can you suggest: (1) CNN architecture for images? (2) When to use conv vs FC? (3) How many parameters is reasonable?"
  successCriteria={["You understand CNN for spatial inputs", "You know FC for low-dim states", "You can estimate parameter counts"]}
  reflectionQuestions={["Why use grayscale images in Atari DQN?", "What's frame stacking?", "How does architecture affect training time?"]}
/>

---

### TryWithAI 4.3.2: Debugging DQN

<TryWithAI
  id="tryai-4-3-2"
  title="Diagnose Training Issues"
  role="Evaluator"
  scenario="Your DQN doesn't learn or diverges (Q-values explode)."
  yourTask="Complete Exercise 4.3.4. Monitor loss and Q-values during training."
  aiPromptTemplate="My DQN training is unstable. Loss starts at X, then explodes to Y after Z episodes. Q-values: [paste statistics]. Here's my code: [paste]. Can you review: (1) Is my learning rate too high? (2) Am I updating target network correctly? (3) Common DQN bugs?"
  successCriteria={["You can diagnose DQN training issues", "You know stabilization techniques", "You understand hyperparameter sensitivity"]}
  reflectionQuestions={["What causes Q-value overestimation?", "Why does replay buffer help?", "How often to update target network?"]}
/>

---

## Summary

1. **DQN**: Neural network Q-function for large state spaces
2. **Experience Replay**: Random sampling breaks correlation
3. **Target Network**: Stabilizes TD targets

**Next**: [Lesson 4.4: Policy Gradients](./lesson-04-policy-gradients.md)
