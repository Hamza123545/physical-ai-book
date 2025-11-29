---
title: "Chapter 4 Quiz: Reinforcement Learning for Robotics"
description: "Test your understanding of RL algorithms and applications"
chapter: 4
quiz_type: "chapter"
total_questions: 10
passing_score: 70
estimated_time: 20
generated_by: "claude-sonnet-4-5-20250929"
source_spec: "specs/002-physical-ai-textbook/spec.md"
created: "2025-11-29"
last_modified: "2025-11-29"
git_author: "hswat"
workflow: "/sp.implement"
version: "1.0"
---

# Chapter 4 Quiz: Reinforcement Learning for Robotics

<Quiz
  chapterId={4}
  questions={[
    {
      id: "q-ch04-01",
      question: "What does the discount factor γ (gamma) represent in reinforcement learning?",
      type: "multiple-choice",
      options: [
        "The learning rate for updating values",
        "How much the agent values future rewards compared to immediate rewards",
        "The probability of exploring vs. exploiting",
        "The number of training episodes"
      ],
      correctAnswer: 1,
      explanation: "The discount factor γ (0 ≤ γ < 1) determines how much the agent values future rewards. γ=0 means only immediate rewards matter (myopic), while γ→1 means the agent is far-sighted and values future rewards almost as much as immediate ones.",
      wrongAnswerExplanations: [
        "Learning rate is typically denoted α (alpha), not γ. Learning rate controls update step size.",
        "Exploration vs. exploitation is controlled by ε (epsilon) in epsilon-greedy policies.",
        "The number of episodes is a hyperparameter for training duration, not related to gamma."
      ],
      difficulty: "medium",
      blooms_level: "Understand",
      tags: ["mdp", "discount-factor", "rl-basics"]
    },
    {
      id: "q-ch04-02",
      question: "What is the Bellman equation used for in reinforcement learning?",
      type: "multiple-choice",
      options: [
        "To compute the gradient of the policy",
        "To recursively define the value function V(s)",
        "To sample actions from the policy",
        "To initialize the Q-table"
      ],
      correctAnswer: 1,
      explanation: "The Bellman equation provides a recursive definition of the value function: V(s) = max_a [R(s,a) + γ Σ P(s'|s,a) V(s')]. This allows value iteration to compute optimal values.",
      wrongAnswerExplanations: [
        "Policy gradients compute policy gradients, not the Bellman equation.",
        "Action sampling is done by the policy π(a|s), not the Bellman equation.",
        "Q-tables are typically initialized to zeros, not using the Bellman equation."
      ],
      difficulty: "medium",
      blooms_level: "Understand",
      tags: ["bellman", "value-function", "mdp"]
    },
    {
      id: "q-ch04-03",
      question: "In Q-learning, what does the Q-function Q(s,a) represent?",
      type: "multiple-choice",
      options: [
        "The probability of taking action a in state s",
        "The expected cumulative reward starting from state s, taking action a",
        "The immediate reward for taking action a in state s",
        "The number of times action a was taken in state s"
      ],
      correctAnswer: 1,
      explanation: "Q(s,a) is the action-value function: the expected cumulative discounted reward starting from state s, taking action a, then following the optimal policy. Q-learning learns this function to find the optimal policy.",
      wrongAnswerExplanations: [
        "The policy π(a|s) represents action probabilities, not Q-values.",
        "Q-values represent expected cumulative rewards (long-term), not just immediate rewards R(s,a).",
        "Visit counts are stored separately for exploration tracking, not in the Q-function."
      ],
      difficulty: "medium",
      blooms_level: "Understand",
      tags: ["q-learning", "value-function"]
    },
    {
      id: "q-ch04-04",
      question: "What is the Q-learning update rule?",
      type: "multiple-choice",
      options: [
        "Q(s,a) = R(s,a)",
        "Q(s,a) ← Q(s,a) + α [r + γ max Q(s',a') - Q(s,a)]",
        "Q(s,a) ← max_a Q(s,a)",
        "Q(s,a) ← γ Q(s',a')"
      ],
      correctAnswer: 1,
      explanation: "The Q-learning update is: Q(s,a) ← Q(s,a) + α [r + γ max_a' Q(s',a') - Q(s,a)], where α is learning rate, r is reward, and the term in brackets is the TD error.",
      wrongAnswerExplanations: [
        "This ignores future rewards and learning from experience.",
        "Taking max doesn't update the Q-value, it just selects the best action.",
        "This doesn't incorporate immediate reward or learning rate."
      ],
      difficulty: "hard",
      blooms_level: "Apply",
      tags: ["q-learning", "td-learning", "update-rule"]
    },
    {
      id: "q-ch04-05",
      question: "Why do we use epsilon-greedy exploration in Q-learning?",
      type: "multiple-choice",
      options: [
        "To speed up computation",
        "To balance exploration of new actions and exploitation of known good actions",
        "To reduce memory usage",
        "To guarantee optimal policy immediately"
      ],
      correctAnswer: 1,
      explanation: "Epsilon-greedy balances exploration (trying new actions to learn about them) and exploitation (using current knowledge to get rewards). With probability ε, explore; with probability 1-ε, exploit the best known action.",
      wrongAnswerExplanations: [
        "Epsilon-greedy doesn't affect computational speed; it affects learning quality.",
        "Memory usage is determined by Q-table size, not exploration strategy.",
        "Exploration is needed precisely because we don't know the optimal policy initially."
      ],
      difficulty: "medium",
      blooms_level: "Understand",
      tags: ["exploration", "epsilon-greedy", "q-learning"]
    },
    {
      id: "q-ch04-06",
      question: "What problem does experience replay solve in DQN?",
      type: "multiple-choice",
      options: [
        "It increases the learning rate",
        "It breaks temporal correlation in training data and stabilizes learning",
        "It reduces the number of parameters in the network",
        "It eliminates the need for a target network"
      ],
      correctAnswer: 1,
      explanation: "Experience replay stores past transitions and samples random minibatches for training. This breaks the correlation between consecutive samples (which are temporally correlated in an episode), leading to more stable and efficient learning.",
      wrongAnswerExplanations: [
        "Experience replay doesn't change the learning rate; it improves data distribution.",
        "Parameter count is determined by network architecture, not replay buffer.",
        "Experience replay and target networks are complementary techniques, both used in DQN."
      ],
      difficulty: "hard",
      blooms_level: "Understand",
      tags: ["dqn", "experience-replay", "deep-rl"]
    },
    {
      id: "q-ch04-07",
      question: "What is the purpose of the target network in DQN?",
      type: "multiple-choice",
      options: [
        "To store the optimal Q-values",
        "To provide stable TD targets and reduce overestimation",
        "To explore the environment",
        "To compute the policy gradient"
      ],
      correctAnswer: 1,
      explanation: "The target network Q(s,a; θ⁻) is a copy of the main network that updates slowly (every N steps). It provides stable TD targets y = r + γ max Q(s',a'; θ⁻), preventing the instability of chasing a moving target.",
      wrongAnswerExplanations: [
        "Neither network stores 'optimal' values; they learn approximations during training.",
        "Exploration is handled by epsilon-greedy, not the target network.",
        "DQN is value-based and doesn't use policy gradients; policy gradient methods like REINFORCE do."
      ],
      difficulty: "hard",
      blooms_level: "Understand",
      tags: ["dqn", "target-network", "deep-rl"]
    },
    {
      id: "q-ch04-08",
      question: "What is the main advantage of policy gradient methods (like REINFORCE) over Q-learning?",
      type: "multiple-choice",
      options: [
        "Policy gradients are always more sample-efficient",
        "Policy gradients can handle continuous action spaces and stochastic policies naturally",
        "Policy gradients don't require a reward signal",
        "Policy gradients converge faster"
      ],
      correctAnswer: 1,
      explanation: "Policy gradient methods directly parameterize the policy π(a|s; θ), allowing them to naturally handle continuous actions (by outputting distribution parameters) and stochastic policies. Q-learning requires discretization for continuous actions.",
      wrongAnswerExplanations: [
        "Policy gradients are typically less sample-efficient than value-based methods due to high variance.",
        "All RL methods require reward signals to learn.",
        "Policy gradients often converge slower due to higher variance, requiring more samples."
      ],
      difficulty: "hard",
      blooms_level: "Understand",
      tags: ["policy-gradient", "reinforce", "continuous-actions"]
    },
    {
      id: "q-ch04-09",
      question: "In the REINFORCE algorithm, what does the return G_t represent?",
      type: "multiple-choice",
      options: [
        "The immediate reward at time t",
        "The discounted sum of all future rewards from time t onwards",
        "The gradient of the policy",
        "The value function V(s_t)"
      ],
      correctAnswer: 1,
      explanation: "The return G_t = Σ_{k=0}^∞ γ^k r_{t+k} is the discounted sum of all future rewards from time t. REINFORCE uses this to weight the policy gradient: ∇ log π(a|s) · G_t.",
      wrongAnswerExplanations: [
        "Immediate reward is just r_t, not the full return G_t.",
        "The policy gradient is ∇_θ log π(a|s; θ), which is multiplied by G_t.",
        "V(s_t) is the expected return, while G_t is the actual observed return from a specific episode."
      ],
      difficulty: "medium",
      blooms_level: "Understand",
      tags: ["reinforce", "return", "policy-gradient"]
    },
    {
      id: "q-ch04-10",
      question: "What is the main challenge in multi-agent reinforcement learning?",
      type: "multiple-choice",
      options: [
        "Agents require more memory",
        "The environment becomes non-stationary as other agents learn and change their policies",
        "Multi-agent systems can't use Q-learning",
        "Communication between agents is impossible"
      ],
      correctAnswer: 1,
      explanation: "In multi-agent RL, as each agent learns and updates its policy, the environment appears non-stationary to other agents (their transition and reward functions effectively change). This violates the stationarity assumption of single-agent RL theory.",
      wrongAnswerExplanations: [
        "Memory requirements scale with number of agents but this isn't the fundamental theoretical challenge.",
        "Independent Q-learning can be applied, though it has limitations in multi-agent settings.",
        "Communication is possible and often beneficial, though not always necessary or available."
      ],
      difficulty: "hard",
      blooms_level: "Understand",
      tags: ["multi-agent", "non-stationarity", "marl"]
    }
  ]}
  questionsPerBatch={10}
/>

---

## Next Steps

Congratulations on completing Chapter 4! 🎉

**You've learned**:
- MDP framework and Bellman equations
- Q-learning for tabular problems
- Deep Q-Networks for high-dimensional states
- Policy gradient methods (REINFORCE)
- Multi-agent coordination

**What's Next**: [Chapter 5: Motion Planning and Control](../chapter-05/index.md)

---

**Estimated time**: 20 minutes | **Passing score**: 70% (7/10 correct)
