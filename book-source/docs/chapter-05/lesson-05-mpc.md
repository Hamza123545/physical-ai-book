---
title: "Lesson 5.5: Model Predictive Control"
description: "Learn MPC for optimal trajectory tracking with constraints"
chapter: 5
lesson: 5
estimated_time: 60
cefr_level: "B2"
blooms_level: "Apply"
digcomp_level: 6
generated_by: "claude-sonnet-4-5-20250929"
source_spec: "specs/002-physical-ai-textbook/spec.md"
created: "2025-11-29"
last_modified: "2025-11-29"
git_author: "claude"
workflow: "/sp.implement"
version: "1.0"
prerequisites: ["chapter-05/lesson-04-feedback-control"]
has_interactive_python: true
interactive_python_count: 3
has_try_with_ai: true
try_with_ai_count: 1
tags: ["mpc", "optimization", "constraints", "receding-horizon"]
---

# Lesson 5.5: Model Predictive Control

## 🎯 Learning Objectives

<LearningObjectives
  cefr_level="B2"
  objectives={[
    {
      text: "Understand MPC's receding horizon optimization principle",
      blooms_level: "Understand",
      assessment_method: "Quiz questions"
    },
    {
      text: "Implement basic MPC for trajectory tracking with constraints",
      blooms_level: "Apply",
      assessment_method: "Interactive Python exercises"
    },
    {
      text: "Compare MPC vs PID trade-offs for constrained systems",
      blooms_level: "Analyze",
      assessment_method: "TryWithAI exercises"
    }
  ]}
/>

## Introduction

**PID** reacts to current error. **MPC** plans ahead, optimizing over a future horizon while respecting constraints.

**Why MPC?**
- Handle constraints (max velocity, acceleration, force)
- Predictive (anticipate future behavior)
- Optimal (minimize cost function)

**Used in**: Self-driving cars, drones, walking robots, industrial automation.

**Time**: 60 minutes

---

## 1. MPC Concept

### Receding Horizon

**Idea**: At each time step:
1. **Predict** system behavior over horizon N steps
2. **Optimize** control sequence to minimize cost
3. **Apply** only first control action
4. **Repeat** at next time step (horizon "recedes")

**Analogy**: Driving a car - you plan the next few seconds, execute immediately, then replan.

---

## 2. MPC Formulation

### Problem Setup

**System model**:
```
x[k+1] = A*x[k] + B*u[k]  (discrete-time dynamics)
```

**Cost function** (minimize over horizon N):
```
J = Σ (||x[k] - x_ref||² + ||u[k]||²)
    k=0 to N-1
```

**Constraints**:
```
u_min ≤ u[k] ≤ u_max  (control limits)
x_min ≤ x[k] ≤ x_max  (state limits)
```

**Solve**: Find optimal u[0], ..., u[N-1] that minimize J subject to constraints.

**Apply**: Use only u[0], discard rest, replan next step.

---

## 3. MPC vs PID

| Aspect | PID | MPC |
|--------|-----|-----|
| **Planning** | Reactive (current error) | Predictive (future horizon) |
| **Constraints** | Hard to enforce | Natural (optimization constraints) |
| **Optimality** | Heuristic tuning | Optimal over horizon |
| **Computation** | Very fast | Slower (solve optimization) |
| **Model** | Not required | Requires system model |

**When to use MPC**: Constraints critical, model available, computation affordable.

---

## 4. Implementation Steps

### 1. Discretize System

Convert continuous ẋ = Ax + Bu to discrete x[k+1] = A_d*x[k] + B_d*u[k].

### 2. Define Cost Function

Quadratic: J = Σ (x-x_ref)ᵀQ(x-x_ref) + uᵀRu
- Q: State cost (tracking error penalty)
- R: Control cost (effort penalty)

### 3. Solve Optimization

Use quadratic programming (QP) solver or gradient descent.

### 4. Apply & Repeat

Use first control, shift horizon, replan.

---

## 5. Exercises

### Exercise 5.5.1: Predict Future States

Simulate future system trajectory given control sequence.

<InteractivePython
  id="ex-5-5-1"
  title="Predict Future States"
  starterCode={`import numpy as np

def predict_trajectory(x0: np.ndarray, u_sequence: np.ndarray,
                       A: np.ndarray, B: np.ndarray) -> np.ndarray:
    """Predict state trajectory over horizon.

    System: x[k+1] = A*x[k] + B*u[k]

    Args:
        x0: Initial state (n,)
        u_sequence: Control sequence (N, m)
        A: State transition matrix (n, n)
        B: Control matrix (n, m)

    Returns:
        State trajectory (N+1, n) [x0, x1, ..., xN]
    """
    N = len(u_sequence)
    n = len(x0)
    trajectory = np.zeros((N + 1, n))
    trajectory[0] = x0

    # TODO: Simulate forward dynamics
    # For k in 0..N-1:
    #   x[k+1] = A @ x[k] + B @ u[k]
    pass

# Test
A = np.array([[1.0, 0.1], [0.0, 1.0]])  # Simple integrator
B = np.array([[0.0], [0.1]])
x0 = np.array([0.0, 0.0])
u_sequence = np.ones((10, 1))  # Constant control

traj = predict_trajectory(x0, u_sequence, A, B)
print(f"Final state: {traj[-1]}")
`}
  hints={[
    "for k in range(N):",
    "trajectory[k+1] = A @ trajectory[k] + B @ u_sequence[k]",
    "return trajectory"
  ]}
/>

---

### Exercise 5.5.2: Simple MPC (Unconstrained)

Implement basic MPC without constraints.

<InteractivePython
  id="ex-5-5-2"
  title="Unconstrained MPC"
  starterCode={`import numpy as np

def simple_mpc(x_current: np.ndarray, x_ref: np.ndarray,
               A: np.ndarray, B: np.ndarray,
               Q: np.ndarray, R: float, N: int) -> float:
    """Simple MPC without constraints (analytical solution).

    For 1D system, minimize: J = Σ Q*(x[k]-x_ref)² + R*u[k]²

    Args:
        x_current: Current state (scalar)
        x_ref: Reference state (scalar)
        A, B: System matrices (scalars for 1D)
        Q, R: Cost weights
        N: Horizon length

    Returns:
        Optimal first control u[0]
    """
    # Simplified: For 1D system, use analytical MPC solution
    # u[0] ≈ K * (x_ref - x_current) where K depends on Q, R, A, B, N

    # TODO: Implement simple proportional-like MPC
    # Hint: For small horizon, acts like high-gain feedback
    # K = Q / (R + B²*Q)  (simplified)
    pass

# Test
x_current = 0.5
x_ref = 1.0
A, B = 1.0, 0.1
Q, R = 1.0, 0.01
N = 10

u = simple_mpc(x_current, x_ref, A, B, Q, R, N)
print(f"Optimal control: {u:.3f}")
`}
  hints={[
    "error = x_ref - x_current",
    "K = Q / (R + B**2 * Q)  # Simplified gain",
    "return K * error"
  ]}
/>

---

### Exercise 5.5.3: MPC with Constraints

Implement MPC with control saturation.

<InteractivePython
  id="ex-5-5-3"
  title="MPC with Control Constraints"
  starterCode={`import numpy as np

def mpc_with_constraints(x_current: float, x_ref: float,
                         A: float, B: float, Q: float, R: float,
                         u_min: float, u_max: float, N: int) -> float:
    """MPC with control saturation constraints.

    Args:
        x_current: Current state
        x_ref: Reference state
        A, B: System dynamics (scalars)
        Q, R: Cost weights
        u_min, u_max: Control limits
        N: Horizon length

    Returns:
        Optimal first control u[0] (saturated)
    """
    # Simplified: Compute unconstrained control, then saturate
    error = x_ref - x_current
    K = Q / (R + B**2 * Q)
    u_unconstrained = K * error

    # TODO: Apply saturation constraints
    # Clamp u between u_min and u_max
    # Hint: np.clip(u, u_min, u_max)
    pass

# Test
x_current = 0.0
x_ref = 2.0
A, B = 1.0, 0.1
Q, R = 1.0, 0.01
u_min, u_max = -1.0, 1.0
N = 10

u = mpc_with_constraints(x_current, x_ref, A, B, Q, R, u_min, u_max, N)
print(f"Saturated control: {u:.3f} (limits: [{u_min}, {u_max}])")
`}
  hints={[
    "u_saturated = np.clip(u_unconstrained, u_min, u_max)",
    "return u_saturated"
  ]}
/>

---

## 6. Try With AI

### TryWithAI 5.5.1: When to Use MPC

<TryWithAI
  id="tryai-5-5-1"
  title="MPC vs PID Decision Making"
  role="Teacher"
  scenario="You need to choose between MPC and PID for a control problem"
  yourTask="Describe your system and propose MPC or PID, then validate choice"
  aiPromptTemplate="I have a [describe system: e.g., '2-DOF robot arm with joint limits and obstacle avoidance']. I think [MPC/PID] is better because [your reasoning: constraints, computation, model availability]. Can you: (1) Validate my choice? (2) Explain scenarios where my choice would fail? (3) Suggest hybrid approaches (e.g., PID with feedforward)?"
  successCriteria={[
    "You can identify when constraints make MPC necessary",
    "You understand computation trade-offs (real-time feasibility)",
    "You recognize when simple PID is sufficient"
  ]}
  reflectionQuestions={[
    "How fast must MPC solve optimization to run at 100 Hz control rate?",
    "Can MPC work without a perfect model? What happens with model mismatch?",
    "What is explicit MPC and when is it useful?"
  ]}
/>

---

## Summary

**Key Takeaways**:

1. **MPC = Optimize over future horizon**: Plan ahead, apply first action, repeat
2. **Receding horizon**: Re-solve optimization at each time step
3. **Natural constraints**: Encode limits in optimization (u_min, u_max, etc.)
4. **Cost function**: Balance tracking (Q) and control effort (R)
5. **Trade-offs**: Optimality + constraints vs computation cost
6. **vs PID**: MPC handles constraints, requires model; PID is fast, simple

**What's Next**: **[Chapter 5 Quiz](./quiz.md)** - Test your understanding of motion planning and control!

---

**Estimated completion time**: 60 minutes | **Prerequisites**: Lesson 5.4 | **Difficulty**: B2 (Upper Intermediate)
