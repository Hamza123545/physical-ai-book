---
title: "Lesson 4.1: RL Basics and Markov Decision Processes"
description: "Understand states, actions, rewards, and the Bellman equation"
chapter: 4
lesson: 1
estimated_time: 60
cefr_level: "B1+"
blooms_level: "Understand"
digcomp_level: 5
generated_by: "claude-sonnet-4-5-20250929"
source_spec: "specs/002-physical-ai-textbook/spec.md"
created: "2025-11-29"
last_modified: "2025-11-29"
git_author: "hswat"
workflow: "/sp.implement"
version: "1.0"
prerequisites:
  - "chapter-01-lesson-04"
has_interactive_python: true
interactive_python_count: 3
has_try_with_ai: true
try_with_ai_count: 1
tags: ["reinforcement-learning", "mdp", "bellman", "value-function"]
---

# Lesson 4.1: RL Basics and Markov Decision Processes

## 🎯 Learning Objectives

<LearningObjectives
  cefr_level="B1+"
  objectives={[
    {
      text: "Define MDP components: states, actions, rewards, transitions",
      blooms_level: "Understand",
      assessment_method: "Quiz and exercises"
    },
    {
      text: "Compute discounted returns for episode sequences",
      blooms_level: "Apply",
      assessment_method: "Return calculation exercise"
    },
    {
      text: "Apply Bellman equation for value functions",
      blooms_level: "Apply",
      assessment_method: "Value iteration exercise"
    }
  ]}
/>

## 📚 Prerequisites

<Prerequisites
  prerequisites={[
    {
      lessonId: "chapter-01-lesson-04",
      title: "Lesson 1.4: Python for Robotics",
      link: "/docs/chapter-01/lesson-04-python-robotics-intro"
    }
  ]}
/>

## Introduction

**Reinforcement Learning (RL)**: Agent learns by interacting with environment, receiving rewards/penalties.

**Key Concepts**:
- **State** ($s$): Current situation
- **Action** ($a$): What agent can do
- **Reward** ($r$): Immediate feedback
- **Policy** ($\pi$): Strategy mapping states to actions

**Time**: 60 minutes

---

## 1. Markov Decision Process (MDP)

**MDP** = $(S, A, P, R, \gamma)$:
- $S$: State space
- $A$: Action space
- $P(s'|s,a)$: Transition probability
- $R(s,a)$: Reward function
- $\gamma$: Discount factor (0 ≤ γ < 1)

**Example**: GridWorld robot navigation

---

## 2. Returns and Value Functions

**Return** (cumulative discounted reward):
$$
G_t = r_t + \gamma r_{t+1} + \gamma^2 r_{t+2} + \ldots = \sum_{k=0}^{\infty} \gamma^k r_{t+k}
$$

**Value function** (expected return from state $s$):
$$
V(s) = \mathbb{E}[G_t | s_t = s]
$$

---

## 3. Bellman Equation

**Bellman equation** (recursive relationship):
$$
V(s) = \max_a \left[ R(s,a) + \gamma \sum_{s'} P(s'|s,a) V(s') \right]
$$

Used for **value iteration** to find optimal policy.

---

## Exercises

### Exercise 4.1.1: Compute Discounted Return

<InteractivePython
  id="ex-4-1-1"
  title="Discounted Return Calculation"
  starterCode={`import numpy as np

def compute_return(rewards, gamma=0.9):
    """Compute discounted return from reward sequence."""
    # TODO: G = r[0] + gamma*r[1] + gamma^2*r[2] + ...
    pass

# Test
rewards = [1, 2, 3, 4, 5]
G = compute_return(rewards, gamma=0.9)
print(f"Return: {G:.2f}")
print(f"Expected: ~11.4")
`}
  hints={[
    "G = 0",
    "for i, r in enumerate(rewards):",
    "  G += (gamma ** i) * r",
    "return G"
  ]}
/>

---

### Exercise 4.1.2: GridWorld Environment

<InteractivePython
  id="ex-4-1-2"
  title="GridWorld MDP"
  starterCode={`import numpy as np

class GridWorld:
    def __init__(self, size=5):
        self.size = size
        self.state = (0, 0)  # Start top-left
        self.goal = (size-1, size-1)  # Goal bottom-right

    def step(self, action):
        """Take action, return (next_state, reward, done)."""
        # TODO: Implement transitions
        # Actions: 0=up, 1=right, 2=down, 3=left
        # Reward: +10 at goal, -0.1 per step, -1 if hit wall
        pass

    def reset(self):
        self.state = (0, 0)
        return self.state

# Test
env = GridWorld(size=5)
state = env.reset()
next_state, reward, done = env.step(1)  # Move right
print(f"State: {state} -> {next_state}, Reward: {reward}")
`}
  hints={[
    "moves = [(-1,0), (0,1), (1,0), (0,-1)]  # up, right, down, left",
    "new_state = (state[0] + moves[action][0], state[1] + moves[action][1])",
    "Check bounds: if 0 <= new_state[0] < size and 0 <= new_state[1] < size",
    "Reward: +10 if goal, -1 if wall, -0.1 otherwise"
  ]}
/>

---

### Exercise 4.1.3: Value Iteration

<InteractivePython
  id="ex-4-1-3"
  title="Bellman Value Iteration"
  starterCode={`import numpy as np

def value_iteration(grid_size=5, gamma=0.9, theta=0.01):
    """Compute optimal value function using Bellman equation."""
    V = np.zeros((grid_size, grid_size))

    # TODO: Iterate until convergence
    # For each state, compute max over actions:
    #   V_new(s) = max_a [R(s,a) + gamma * V(s')]
    # Stop when max change < theta

    return V

# Test
V_optimal = value_iteration(grid_size=5, gamma=0.9)
print(f"Value at start: {V_optimal[0, 0]:.2f}")
print(f"Value at goal: {V_optimal[4, 4]:.2f}")
`}
  hints={[
    "while True:",
    "  delta = 0",
    "  for each state (i,j):",
    "    v = V[i,j]",
    "    V[i,j] = max over 4 actions of [R + gamma * V[next_state]]",
    "    delta = max(delta, abs(v - V[i,j]))",
    "  if delta < theta: break"
  ]}
/>

---

## Try With AI

### TryWithAI 4.1.1: RL Concepts

<TryWithAI
  id="tryai-4-1-1"
  title="Understand Discount Factor"
  role="Teacher"
  scenario="You're confused about why we discount future rewards with gamma < 1."
  yourTask="Complete Exercise 4.1.1. Experiment with gamma=0.5, 0.9, 0.99. Observe how return changes."
  aiPromptTemplate="I don't understand the discount factor gamma in RL. Why not just sum all rewards equally? Here's my code: [paste]. With gamma=0.5, I get return=X, with gamma=0.99, I get Y. Can you explain: (1) Why discount future rewards? (2) What does gamma=0 vs gamma=1 mean? (3) How to choose gamma for real robots?"
  successCriteria={[
    "You understand gamma represents how much the agent values future vs. immediate rewards",
    "You know gamma=0 (myopic, only immediate reward) vs gamma→1 (far-sighted)",
    "You can choose appropriate gamma for robotics tasks"
  ]}
  reflectionQuestions={[
    "What happens if gamma=1 in infinite-horizon tasks?",
    "How does gamma affect learning speed?",
    "Should walking robots use high or low gamma?"
  ]}
/>

---

## Summary

**Key Takeaways**:
1. **MDP**: $(S, A, P, R, \gamma)$ framework for sequential decision-making
2. **Return**: $G_t = \sum \gamma^k r_{t+k}$ (discounted cumulative reward)
3. **Bellman Equation**: $V(s) = \max_a [R(s,a) + \gamma \sum P(s'|s,a) V(s')]$

**Next**: [Lesson 4.2: Q-Learning](./lesson-02-q-learning.md)

---

**Estimated time**: 60 minutes | **Difficulty**: B1+
