---
title: "Lesson 4.5: Multi-Agent Reinforcement Learning"
description: "Coordinate multiple robots using multi-agent RL"
chapter: 4
lesson: 5
estimated_time: 60
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
interactive_python_count: 3
has_try_with_ai: true
try_with_ai_count: 2
tags: ["multi-agent", "cooperation", "communication", "coordination"]
---

# Lesson 4.5: Multi-Agent Reinforcement Learning

## 🎯 Learning Objectives

<LearningObjectives
  cefr_level="B2"
  objectives={[
    {text: "Implement independent Q-learning for multiple agents", blooms_level: "Apply", assessment_method: "Multi-agent exercise"},
    {text: "Design reward functions for cooperation", blooms_level: "Apply", assessment_method: "Reward shaping exercise"},
    {text: "Handle non-stationarity in multi-agent settings", blooms_level: "Understand", assessment_method: "TryWithAI"}
  ]}
/>

## Introduction

**Multi-Agent RL**: Multiple robots learning simultaneously.

**Challenges**:
- Non-stationary environment (other agents learning too)
- Credit assignment (who caused success/failure?)
- Communication and coordination

---

## 1. Independent Q-Learning

Each agent learns independently. Simple but ignores other agents.

---

## 2. Centralized Training, Decentralized Execution (CTDE)

Train with global info, execute with local observations.

---

## 3. Communication

Agents share observations or intentions.

---

## Exercises

### Exercise 4.5.1: Multi-Agent Environment

<InteractivePython
  id="ex-4-5-1"
  title="Multi-Robot GridWorld"
  starterCode={`import numpy as np

class MultiAgentGrid:
    def __init__(self, n_agents=2, grid_size=5):
        self.n_agents = n_agents
        self.size = grid_size
        self.positions = [(0, 0), (grid_size-1, grid_size-1)]
        self.goal = (2, 2)  # Shared goal

    def step(self, actions):
        """Take actions for all agents."""
        # TODO: Move each agent, compute rewards
        # Reward: +10 if all at goal, -0.1 per step
        pass

# Test
env = MultiAgentGrid(n_agents=2, grid_size=5)
print(f"Agents: {env.n_agents}, Goal: {env.goal}")
`}
  hints={[
    "rewards = []",
    "for i, action in enumerate(actions):",
    "  self.positions[i] = self.move(self.positions[i], action)",
    "  if self.positions[i] == self.goal:",
    "    rewards.append(10)",
    "  else:",
    "    rewards.append(-0.1)",
    "done = all(pos == self.goal for pos in self.positions)",
    "return self.positions, rewards, done"
  ]}
/>

---

### Exercise 4.5.2: Independent Q-Learning

<InteractivePython
  id="ex-4-5-2"
  title="Train Multiple Agents"
  starterCode={`import numpy as np

def train_multi_agent(env, n_agents, episodes=500):
    """Train multiple agents with independent Q-learning."""
    Q_tables = [np.zeros((env.n_states, env.n_actions)) for _ in range(n_agents)]

    # TODO: For each episode:
    #   Each agent selects action independently
    #   Update each Q-table independently

    return Q_tables

# Placeholder
print("Multi-agent training defined")
`}
  hints={[
    "for episode in range(episodes):",
    "  states = env.reset()",
    "  while not done:",
    "    actions = [epsilon_greedy(Q_tables[i], states[i]) for i in range(n_agents)]",
    "    next_states, rewards, done = env.step(actions)",
    "    for i in range(n_agents):",
    "      q_update(Q_tables[i], states[i], actions[i], rewards[i], next_states[i])",
    "    states = next_states"
  ]}
/>

---

### Exercise 4.5.3: Cooperative Reward Shaping

<InteractivePython
  id="ex-4-5-3"
  title="Design Cooperative Rewards"
  starterCode={`import numpy as np

def compute_cooperative_reward(positions, goal, actions):
    """Compute reward encouraging cooperation."""
    # TODO: Reward closer agents, penalize collisions
    # Bonus if agents coordinate to reach goal together
    pass

# Test
positions = [(1, 1), (3, 3)]
goal = (2, 2)
reward = compute_cooperative_reward(positions, goal, [0, 1])
print(f"Cooperative reward: {reward}")
`}
  hints={[
    "distances = [np.linalg.norm(np.array(pos) - np.array(goal)) for pos in positions]",
    "reward = -np.mean(distances)  # Closer is better",
    "if positions[0] == positions[1]:  # Collision",
    "  reward -= 10",
    "if all(pos == goal for pos in positions):  # All at goal",
    "  reward += 50",
    "return reward"
  ]}
/>

---

## Try With AI

### TryWithAI 4.5.1: Multi-Agent Coordination

<TryWithAI
  id="tryai-4-5-1"
  title="Improve Coordination"
  role="Copilot"
  scenario="Your agents don't cooperate—they interfere with each other."
  yourTask="Implement Exercise 4.5.2. Observe agents blocking each other or racing to goal."
  aiPromptTemplate="My multi-agent system has agents interfering. They block paths or compete instead of cooperate. Here's my code: [paste]. Can you suggest: (1) Better reward shaping? (2) Communication mechanisms? (3) Should I use centralized training?"
  successCriteria={["You understand coordination challenges", "You know reward shaping for cooperation", "You've heard of CTDE approaches"]}
  reflectionQuestions={["How do you measure cooperation?", "What's emergent behavior?", "When is communication essential?"]}
/>

---

### TryWithAI 4.5.2: Non-Stationarity Problem

<TryWithAI
  id="tryai-4-5-2"
  title="Handle Non-Stationary Environments"
  role="Evaluator"
  scenario="As agents learn, environment becomes non-stationary (other agents' policies change)."
  yourTask="Train agents with Exercise 4.5.2. Notice performance oscillates."
  aiPromptTemplate="My multi-agent training is unstable. Agent 1's performance improves then degrades as Agent 2 learns. Here's my training curve: [describe]. Can you review: (1) Why does this happen? (2) What's the non-stationarity problem? (3) Solutions like opponent modeling or parameter sharing?"
  successCriteria={["You understand non-stationarity in MARL", "You know stabilization techniques", "You've explored opponent modeling"]}
  reflectionQuestions={["Can single-agent RL theory apply to MARL?", "What's Nash equilibrium in MARL?", "How to ensure convergence?"]}
/>

---

## Summary

1. **Multi-Agent RL**: Multiple learning agents interacting
2. **Independent Q-Learning**: Simple but treats others as environment
3. **Cooperation**: Reward shaping, communication, CTDE

**What's Next**: [Chapter 4 Quiz](./quiz.md)

---

**Estimated time**: 60 minutes | **Difficulty**: B2
