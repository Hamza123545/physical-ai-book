---
title: "Lesson 5.3: Trajectory Generation"
description: "Generate smooth, time-parameterized trajectories for robot motion"
chapter: 5
lesson: 3
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
prerequisites: ["chapter-05/lesson-01-path-planning", "chapter-02"]
has_interactive_python: true
interactive_python_count: 4
has_try_with_ai: true
try_with_ai_count: 2
tags: ["trajectory", "polynomial", "interpolation", "time-optimal"]
---

# Lesson 5.3: Trajectory Generation

## 🎯 Learning Objectives

<LearningObjectives
  cefr_level="B2"
  objectives={[
    {
      text: "Distinguish between path (geometric) and trajectory (time-parameterized)",
      blooms_level: "Understand",
      assessment_method: "Quiz questions"
    },
    {
      text: "Generate polynomial trajectories with position, velocity, acceleration constraints",
      blooms_level: "Apply",
      assessment_method: "Interactive Python exercises"
    },
    {
      text: "Design minimum-jerk trajectories for smooth robot motion",
      blooms_level: "Apply",
      assessment_method: "Interactive Python exercises"
    }
  ]}
/>

## Introduction

**Path planning** gives waypoints: [(0,0), (1,2), (3,3)].
**Trajectory generation** adds time: "reach (1,2) at t=1s, (3,3) at t=2s" with smooth velocity/acceleration.

**Why?** Motors need smooth commands. Abrupt changes cause vibration, wear, instability.

**Time**: 60 minutes

---

## 1. Path vs Trajectory

| Concept | Definition | Example |
|---------|------------|---------|
| **Path** | Sequence of waypoints (geometry only) | [(0,0), (5,0), (5,5)] |
| **Trajectory** | Path + time parameterization | q(t): position at time t |
| **Velocity profile** | Speed along path | v(t): velocity at time t |

**Real robots** execute trajectories, not paths!

---

## 2. Polynomial Trajectories

### Cubic Polynomial (3rd degree)

For 1-DOF motion from q0 to qf in time T:

```
q(t) = a0 + a1*t + a2*t^2 + a3*t^3
```

**Constraints** (4 equations, 4 unknowns):
- q(0) = q0 (start position)
- q(T) = qf (end position)
- v(0) = 0 (start at rest)
- v(T) = 0 (end at rest)

**Solve**: Find a0, a1, a2, a3 satisfying constraints.

### Quintic Polynomial (5th degree)

Adds acceleration constraints:
- a(0) = 0 (start with zero acceleration)
- a(T) = 0 (end with zero acceleration)

**More DOFs**: Smoother motion, less jerk.

---

## 3. Minimum-Jerk Trajectories

**Jerk** = rate of change of acceleration (d³q/dt³).

High jerk causes:
- Motor stress
- Vibration
- Inaccuracy

**Minimum-jerk trajectory**: Minimize ∫ jerk² dt over [0, T].

**Result**: Natural, human-like motion (humans minimize jerk unconsciously!).

---

## 4. Time-Optimal Trajectories

**Goal**: Reach goal as fast as possible while respecting:
- Max velocity: |v| ≤ v_max
- Max acceleration: |a| ≤ a_max

**Bang-bang control**: Accelerate at max, coast, decelerate at max.

**Trade-off**: Speed vs smoothness (jerk increases).

---

## 5. Exercises

### Exercise 5.3.1: Cubic Polynomial Trajectory

Generate cubic polynomial for 1-DOF joint motion.

<InteractivePython
  id="ex-5-3-1"
  title="Cubic Polynomial Trajectory"
  starterCode={`import numpy as np

def cubic_trajectory(q0: float, qf: float, T: float, t: float) -> float:
    """Compute position at time t using cubic polynomial.

    Constraints:
        q(0) = q0, q(T) = qf
        v(0) = 0, v(T) = 0

    Args:
        q0: Start position
        qf: End position
        T: Total time
        t: Current time (0 <= t <= T)

    Returns:
        Position at time t
    """
    # Coefficients for cubic: q(t) = a0 + a1*t + a2*t^2 + a3*t^3
    # From constraints:
    # a0 = q0
    # a1 = 0
    # a2 = 3*(qf - q0) / T^2
    # a3 = -2*(qf - q0) / T^3

    # TODO: Implement cubic trajectory
    pass

# Test
q0, qf, T = 0.0, 1.0, 2.0
times = np.linspace(0, T, 100)
positions = [cubic_trajectory(q0, qf, T, t) for t in times]
print(f"Start: {positions[0]:.3f}, End: {positions[-1]:.3f}")
`}
  hints={[
    "a0 = q0",
    "a1 = 0 (zero initial velocity)",
    "a2 = 3*(qf - q0) / (T**2)",
    "a3 = -2*(qf - q0) / (T**3)",
    "return a0 + a1*t + a2*t**2 + a3*t**3"
  ]}
/>

---

### Exercise 5.3.2: Velocity Profile

Compute velocity from cubic trajectory.

<InteractivePython
  id="ex-5-3-2"
  title="Velocity Profile"
  starterCode={`import numpy as np

def cubic_velocity(q0: float, qf: float, T: float, t: float) -> float:
    """Compute velocity at time t (derivative of cubic trajectory).

    Args:
        q0: Start position
        qf: End position
        T: Total time
        t: Current time

    Returns:
        Velocity at time t
    """
    # v(t) = dq/dt = a1 + 2*a2*t + 3*a3*t^2
    # TODO: Compute velocity using derivative
    pass

# Test
q0, qf, T = 0.0, 1.0, 2.0
times = np.linspace(0, T, 100)
velocities = [cubic_velocity(q0, qf, T, t) for t in times]
print(f"Start vel: {velocities[0]:.3f}, End vel: {velocities[-1]:.3f}")
print(f"Max vel: {max(velocities):.3f}")
`}
  hints={[
    "Use same a2, a3 from cubic_trajectory",
    "a1 = 0",
    "return a1 + 2*a2*t + 3*a3*t**2"
  ]}
/>

---

### Exercise 5.3.3: Quintic Trajectory

Generate quintic polynomial with acceleration constraints.

<InteractivePython
  id="ex-5-3-3"
  title="Quintic Polynomial Trajectory"
  starterCode={`import numpy as np

def quintic_trajectory(q0: float, qf: float, T: float, t: float) -> float:
    """Quintic polynomial with zero boundary velocities and accelerations.

    Constraints:
        q(0) = q0, q(T) = qf
        v(0) = 0, v(T) = 0
        a(0) = 0, a(T) = 0

    Returns:
        Position at time t
    """
    # Quintic: q(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
    # From 6 constraints:
    a0 = q0
    a1 = 0.0
    a2 = 0.0
    a3 = 10.0 * (qf - q0) / (T ** 3)
    a4 = -15.0 * (qf - q0) / (T ** 4)
    a5 = 6.0 * (qf - q0) / (T ** 5)

    # TODO: Return quintic polynomial value
    pass

# Test
q0, qf, T = 0.0, 1.0, 2.0
times = np.linspace(0, T, 100)
positions = [quintic_trajectory(q0, qf, T, t) for t in times]
print(f"Start: {positions[0]:.3f}, End: {positions[-1]:.3f}")
`}
  hints={[
    "return a0 + a1*t + a2*t**2 + a3*t**3 + a4*t**4 + a5*t**5"
  ]}
/>

---

### Exercise 5.3.4: Multi-Waypoint Trajectory

Connect multiple waypoints with smooth trajectories.

<InteractivePython
  id="ex-5-3-4"
  title="Multi-Waypoint Trajectory"
  starterCode={`import numpy as np

def multi_waypoint_trajectory(waypoints, times, t):
    """Generate trajectory through multiple waypoints.

    Args:
        waypoints: List of positions [q0, q1, q2, ...]
        times: List of times [0, t1, t2, ...] for each waypoint
        t: Current time

    Returns:
        Position at time t
    """
    # TODO: Find which segment t belongs to
    # Apply cubic_trajectory for that segment
    # Hint: Use cubic_trajectory(qi, qi+1, Ti, t_local)

    pass

# Test
waypoints = [0.0, 1.0, 0.5, 1.5]
times = [0.0, 1.0, 2.0, 3.0]
test_times = np.linspace(0, 3, 100)
traj = [multi_waypoint_trajectory(waypoints, times, t) for t in test_times]
print(f"Trajectory length: {len(traj)}")
`}
  hints={[
    "Find segment: for i in range(len(times)-1): if times[i] <= t <= times[i+1]",
    "Local time: t_local = t - times[i]",
    "Segment duration: T_seg = times[i+1] - times[i]",
    "return cubic_trajectory(waypoints[i], waypoints[i+1], T_seg, t_local)"
  ]}
/>

---

## 6. Try With AI

### TryWithAI 5.3.1: Trajectory Smoothness Analysis

<TryWithAI
  id="tryai-5-3-1"
  title="Compare Cubic vs Quintic Smoothness"
  role="Teacher"
  scenario="You want to understand when to use cubic vs quintic polynomials"
  yourTask="Implement both cubic and quintic trajectories for same motion, compare results"
  aiPromptTemplate="I implemented cubic and quintic trajectories for moving from 0 to 1 in 2 seconds. Here's what I observe: [describe position/velocity/acceleration plots or values]. Can you explain: (1) Why quintic is smoother? (2) When would I prefer cubic for simplicity? (3) What happens if I need to pass through intermediate waypoints?"
  successCriteria={[
    "You understand quintic has smoother acceleration (zero jerk at boundaries)",
    "You can identify when extra smoothness is worth the complexity",
    "You understand waypoint constraints affect polynomial choice"
  ]}
  reflectionQuestions={[
    "What is jerk and why does it matter for motors?",
    "How would you handle velocity constraints at waypoints?"
  ]}
/>

---

### TryWithAI 5.3.2: Time-Optimal Trajectory Design

<TryWithAI
  id="tryai-5-3-2"
  title="Design Time-Optimal Motion"
  role="Evaluator"
  scenario="You need fastest possible motion with velocity/acceleration limits"
  yourTask="Design a bang-bang trajectory for 1-DOF motion with v_max=2.0, a_max=1.0"
  aiPromptTemplate="I want to move from 0 to 10 meters as fast as possible with v_max=2.0 m/s and a_max=1.0 m/s². I think the trajectory should: [describe your approach: accelerate/coast/decelerate phases]. Can you verify: (1) Is my time calculation correct? (2) Do I violate constraints? (3) How does this compare to a smooth polynomial trajectory?"
  successCriteria={[
    "You can compute minimum time analytically",
    "You understand bang-bang creates non-smooth jerk",
    "You can identify trade-off between speed and smoothness"
  ]}
  reflectionQuestions={[
    "When would you sacrifice time-optimality for smoothness?",
    "How do you handle cases where coast phase duration is zero?"
  ]}
/>

---

## Summary

**Key Takeaways**:

1. **Path ≠ Trajectory**: Add time parameterization for execution
2. **Cubic polynomial**: 4 constraints (position + velocity boundaries)
3. **Quintic polynomial**: 6 constraints (+ acceleration boundaries, smoother)
4. **Minimum jerk**: Natural motion, reduces vibration
5. **Time-optimal**: Bang-bang control, fast but jerky
6. **Trade-offs**: Speed vs smoothness, complexity vs performance

**What's Next**: [Lesson 5.4: Feedback Control](./lesson-04-feedback-control.md) ensures robots track generated trajectories accurately.

---

**Estimated completion time**: 60 minutes | **Prerequisites**: Lesson 5.1, Chapter 2 | **Difficulty**: B2 (Upper Intermediate)
