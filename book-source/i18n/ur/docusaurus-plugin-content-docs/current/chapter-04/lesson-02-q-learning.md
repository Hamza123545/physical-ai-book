---
title: "Lesson 4.2: Q-Learning Algorithm"
description: "Implement tabular Q-learning for discrete state-action spaces"
chapter: 4
lesson: 2
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
  - "chapter-04-lesson-01"
has_interactive_python: true
interactive_python_count: 3
has_try_with_ai: true
try_with_ai_count: 2
tags: ["q-learning", "temporal-difference", "epsilon-greedy", "tabular-rl"]
---

# Lesson 4.2: Q-Learning Algorithm

## 🎯 Learning Objectives

<LearningObjectives
  cefr_level="B2"
  objectives={[
    {text: "Implement Q-learning update rule", blooms_level: "Apply", assessment_method: "Q-table exercise"},
    {text: "Apply epsilon-greedy exploration strategy", blooms_level: "Apply", assessment_method: "Policy exercise"},
    {text: "Train agent to solve GridWorld with 90%+ success", blooms_level: "Apply", assessment_method: "Training exercise"}
  ]}
/>

## Introduction

**Q-learning**: Learn action-value function $Q(s,a)$ without knowing environment dynamics.

**Q-update rule**:
$$Q(s,a) \leftarrow Q(s,a) + \alpha [r + \gamma \max_{a'} Q(s',a') - Q(s,a)]$$

**Key idea**: Bootstrap from next state's best Q-value.

---

## 1. Q-Table Representation

Store Q-values in table: rows=states, columns=actions.

---

## 2. Epsilon-Greedy Policy

**Exploration vs. Exploitation**:
- With probability $\epsilon$: random action (explore)
- With probability $1-\epsilon$: best action (exploit)

---

## Exercises

### Exercise 4.2.1: Q-Table Update

<InteractivePython
  id="ex-4-2-1"
  title="Q-Learning Update"
  starterCode={`import numpy as np

def q_update(Q, s, a, r, s_next, alpha=0.1, gamma=0.9):
    """Apply Q-learning update rule."""
    # TODO: Q[s,a] += alpha * (r + gamma * max(Q[s_next]) - Q[s,a])
    pass

# Test
Q = np.zeros((5, 4))  # 5 states, 4 actions
Q[0, 1] = 0.5
q_update(Q, s=0, a=1, r=1.0, s_next=1, alpha=0.1, gamma=0.9)
print(f"Q[0,1]: {Q[0,1]:.3f}")
`}
  hints={[
    "max_next = np.max(Q[s_next])",
    "td_target = r + gamma * max_next",
    "td_error = td_target - Q[s, a]",
    "Q[s, a] += alpha * td_error"
  ]}
/>

---

### Exercise 4.2.2: Epsilon-Greedy Policy

<InteractivePython
  id="ex-4-2-2"
  title="Exploration Strategy"
  starterCode={`import numpy as np

def epsilon_greedy(Q, state, epsilon=0.1):
    """Select action using epsilon-greedy."""
    # TODO: Random action with prob epsilon, else best action
    pass

# Test
Q = np.array([[0.1, 0.5, 0.2, 0.3]])
action = epsilon_greedy(Q, state=0, epsilon=0.1)
print(f"Action: {action}")
`}
  hints={[
    "if np.random.rand() < epsilon:",
    "  return np.random.randint(Q.shape[1])",
    "else:",
    "  return np.argmax(Q[state])"
  ]}
/>

---

### Exercise 4.2.3: Train Q-Learning Agent

<InteractivePython
  id="ex-4-2-3"
  title="GridWorld Q-Learning"
  starterCode={`import numpy as np

def train_q_learning(env, episodes=500, alpha=0.1, gamma=0.9, epsilon=0.1):
    """Train Q-learning agent."""
    Q = np.zeros((env.n_states, env.n_actions))

    # TODO: For each episode:
    #   Reset env
    #   While not done:
    #     Select action (epsilon-greedy)
    #     Take action, observe r, s'
    #     Update Q-table

    return Q

# Test (requires GridWorld from 4.1)
# Q = train_q_learning(env, episodes=500)
# print(f"Trained Q-table shape: {Q.shape}")
`}
  hints={[
    "for episode in range(episodes):",
    "  state = env.reset()",
    "  done = False",
    "  while not done:",
    "    action = epsilon_greedy(Q, state, epsilon)",
    "    next_state, reward, done = env.step(action)",
    "    q_update(Q, state, action, reward, next_state, alpha, gamma)",
    "    state = next_state"
  ]}
/>

---

## Try With AI

### TryWithAI 4.2.1: Hyperparameter Tuning

<TryWithAI
  id="tryai-4-2-1"
  title="Tune Learning Rate and Epsilon"
  role="Copilot"
  scenario="Your Q-learning agent learns slowly or fails to converge."
  yourTask="Train agent with different alpha (0.01, 0.1, 0.5) and epsilon (0.05, 0.1, 0.3). Plot learning curves."
  aiPromptTemplate="My Q-learning agent learns slowly. With alpha=0.1, epsilon=0.1, it needs 500 episodes. Here's my code: [paste]. Can you help tune hyperparameters? What's the role of alpha vs epsilon? How do I detect convergence?"
  successCriteria={["You understand alpha controls learning speed", "You know epsilon balances exploration/exploitation", "You can tune hyperparameters systematically"]}
  reflectionQuestions={["What happens if alpha is too high?", "Should epsilon decay over time?", "How many episodes are enough?"]}
/>

---

### TryWithAI 4.2.2: Q-Learning Debugging

<TryWithAI
  id="tryai-4-2-2"
  title="Debug Q-Table"
  role="Evaluator"
  scenario="Your Q-table has unexpected values or agent takes bad actions."
  yourTask="Complete Exercise 4.2.3. Inspect Q-table after training. Verify optimal policy."
  aiPromptTemplate="My Q-learning agent trained but takes suboptimal actions. Here's my Q-table: [paste values]. Can you review: (1) Are Q-values reasonable? (2) Does argmax(Q) give optimal policy? (3) Common bugs in my update rule?"
  successCriteria={["You can interpret Q-values", "You verify optimal policy from Q-table", "You know common Q-learning bugs"]}
  reflectionQuestions={["What Q-values indicate good/bad states?", "How to visualize Q-table?", "When is off-policy learning useful?"]}
/>

---

## Summary

1. **Q-Learning**: Model-free TD learning
2. **Update**: $Q(s,a) \leftarrow Q(s,a) + \alpha [r + \gamma \max_{a'} Q(s',a') - Q(s,a)]$
3. **Exploration**: Epsilon-greedy balances exploration/exploitation

**Next**: [Lesson 4.3: Deep Q-Networks](./lesson-03-deep-q-networks.md)
