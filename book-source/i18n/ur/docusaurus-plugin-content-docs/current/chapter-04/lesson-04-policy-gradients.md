---
title: "Lesson 4.4: Policy Gradient Methods"
description: "Learn policies directly with REINFORCE algorithm"
chapter: 4
lesson: 4
estimated_time: 60
cefr_level: "B2"
blooms_level: "Apply"
digcomp_level: 5
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
try_with_ai_count: 1
tags: ["policy-gradient", "reinforce", "actor-critic", "continuous-actions"]
---

# Lesson 4.4: Policy Gradient Methods

## 🎯 Learning Objectives

<LearningObjectives
  cefr_level="B2"
  objectives={[
    {text: "Implement policy network for action selection", blooms_level: "Apply", assessment_method: "Policy network exercise"},
    {text: "Compute policy gradient using REINFORCE", blooms_level: "Apply", assessment_method: "REINFORCE exercise"},
    {text: "Compare value-based vs policy-based methods", blooms_level: "Understand", assessment_method: "TryWithAI"}
  ]}
/>

## Introduction

**Policy Gradient**: Learn policy $\pi(a|s; \theta)$ directly, not via Q-values.

**Advantage**: Handles continuous actions, stochastic policies.

**REINFORCE**: Monte Carlo policy gradient algorithm.

---

## 1. Policy Network

Output action probabilities: $\pi(a|s; \theta)$.

**Continuous actions**: Output mean/std for Gaussian policy.

---

## 2. REINFORCE Algorithm

**Policy gradient theorem**:
$$\nabla_\theta J(\theta) = \mathbb{E}[\nabla_\theta \log \pi(a|s; \theta) \cdot G_t]$$

**Update**: $\theta \leftarrow \theta + \alpha \nabla_\theta \log \pi(a|s) \cdot G_t$

---

## 3. Baseline for Variance Reduction

Subtract baseline (e.g., value function) to reduce gradient variance:
$$\nabla_\theta J \propto \nabla_\theta \log \pi(a|s) \cdot (G_t - b(s))$$

---

## Exercises

### Exercise 4.4.1: Policy Network

<InteractivePython
  id="ex-4-4-1"
  title="Build Policy Network"
  starterCode={`import numpy as np

class PolicyNetwork:
    def __init__(self, state_dim, n_actions, hidden=32):
        """Policy network outputting action probabilities."""
        # TODO: Initialize weights
        self.W1 = np.random.randn(state_dim, hidden) * 0.01
        self.W2 = np.random.randn(hidden, n_actions) * 0.01

    def forward(self, state):
        """Output action probabilities (softmax)."""
        # TODO: h = relu(state @ W1)
        #       logits = h @ W2
        #       probs = softmax(logits)
        pass

    def sample_action(self, state):
        """Sample action from policy."""
        probs = self.forward(state)
        return np.random.choice(len(probs), p=probs)

# Test
policy = PolicyNetwork(state_dim=4, n_actions=2)
# action = policy.sample_action(np.array([0.1, 0.2, 0.3, 0.4]))
print("Policy network defined")
`}
  hints={[
    "h = np.maximum(0, state @ self.W1)",
    "logits = h @ self.W2",
    "exp_logits = np.exp(logits - np.max(logits))",
    "probs = exp_logits / np.sum(exp_logits)",
    "return probs"
  ]}
/>

---

### Exercise 4.4.2: Compute Returns

<InteractivePython
  id="ex-4-4-2"
  title="Discounted Returns for Episode"
  starterCode={`import numpy as np

def compute_returns(rewards, gamma=0.99):
    """Compute discounted returns G_t for each timestep."""
    # TODO: G_t = r_t + gamma*r_{t+1} + gamma^2*r_{t+2} + ...
    # Compute backwards from end of episode
    pass

# Test
rewards = [1, 1, 1, 10]  # Final reward = 10
returns = compute_returns(rewards, gamma=0.9)
print(f"Returns: {returns}")
print(f"Expected: [~12.7, ~12.9, ~11.0, 10.0]")
`}
  hints={[
    "returns = []",
    "G = 0",
    "for r in reversed(rewards):",
    "  G = r + gamma * G",
    "  returns.insert(0, G)",
    "return np.array(returns)"
  ]}
/>

---

### Exercise 4.4.3: REINFORCE Training

<InteractivePython
  id="ex-4-4-3"
  title="REINFORCE Algorithm"
  starterCode={`import numpy as np

def train_reinforce(env, policy, episodes=200, alpha=0.01, gamma=0.99):
    """Train policy using REINFORCE."""
    # TODO: For each episode:
    #   1. Generate episode (s, a, r sequence)
    #   2. Compute returns G_t
    #   3. Update policy: theta += alpha * grad_log_pi * G_t

    pass

# Placeholder
print("REINFORCE training loop defined")
`}
  hints={[
    "for episode in range(episodes):",
    "  states, actions, rewards = [], [], []",
    "  state = env.reset()",
    "  while not done:",
    "    action = policy.sample_action(state)",
    "    next_state, reward, done = env.step(action)",
    "    states.append(state); actions.append(action); rewards.append(reward)",
    "    state = next_state",
    "  returns = compute_returns(rewards, gamma)",
    "  for t in range(len(states)):",
    "    grad_log_pi = compute_policy_gradient(policy, states[t], actions[t])",
    "    policy.update(grad_log_pi, returns[t], alpha)"
  ]}
/>

---

## Try With AI

### TryWithAI 4.4.1: Compare Q-Learning vs REINFORCE

<TryWithAI
  id="tryai-4-4-1"
  title="Value-Based vs Policy-Based Trade-offs"
  role="Teacher"
  scenario="You want to understand when to use Q-learning vs REINFORCE."
  yourTask="Implement both Q-learning (4.2.3) and REINFORCE (4.4.3) on same environment. Compare sample efficiency."
  aiPromptTemplate="I've implemented both Q-learning and REINFORCE. Q-learning converges in X episodes, REINFORCE in Y episodes. Here are my results: [paste]. Can you explain: (1) Why is Q-learning more sample-efficient? (2) When should I use policy gradients? (3) What about continuous action spaces? (4) What's actor-critic?"
  successCriteria={["You understand value-based vs policy-based trade-offs", "You know REINFORCE has high variance", "You know when to use each method"]}
  reflectionQuestions={["Can Q-learning handle continuous actions?", "Why do policy gradients need baselines?", "What's the role of entropy regularization?"]}
/>

---

## Summary

1. **Policy Gradient**: Learn policy directly $\pi(a|s; \theta)$
2. **REINFORCE**: $\theta \leftarrow \theta + \alpha \nabla \log \pi \cdot G_t$
3. **Trade-off**: High variance, but handles continuous/stochastic

**Next**: [Lesson 4.5: Multi-Agent RL](./lesson-05-multi-agent-rl.md)
