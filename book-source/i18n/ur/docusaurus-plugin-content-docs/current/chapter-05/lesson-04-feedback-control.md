---
title: "Lesson 5.4: Feedback Control"
description: "Design PID and state-space controllers for trajectory tracking"
chapter: 5
lesson: 4
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
prerequisites: ["chapter-05/lesson-03-trajectory-generation", "chapter-01/lesson-04-python-robotics-intro"]
has_interactive_python: true
interactive_python_count: 3
has_try_with_ai: true
try_with_ai_count: 2
tags: ["pid", "feedback-control", "state-space", "tracking"]
---

# Lesson 5.4: Feedback Control

## 🎯 Learning Objectives

<LearningObjectives
  cefr_level="B2"
  objectives={[
    {
      text: "Understand feedback control principles for trajectory tracking",
      blooms_level: "Understand",
      assessment_method: "Quiz questions"
    },
    {
      text: "Implement PID controller with tuning for position control",
      blooms_level: "Apply",
      assessment_method: "Interactive Python exercises"
    },
    {
      text: "Design state-space controller for multi-variable systems",
      blooms_level: "Apply",
      assessment_method: "Interactive Python exercises"
    }
  ]}
/>

## Introduction

**Planning** creates trajectories. **Control** makes robots follow them despite:
- Motor dynamics
- External disturbances (friction, gravity)
- Sensor noise
- Model uncertainties

**Feedback control** measures error and corrects it continuously.

**Time**: 60 minutes

---

## 1. Feedback Control Loop

### Open-Loop vs Closed-Loop

**Open-loop**: Send commands, hope for the best (no correction).
```
Command → Robot → (hope it works)
```

**Closed-loop (feedback)**: Measure error, correct continuously.
```
Command → Error → Controller → Robot → Sensor → (loop back)
```

**Why feedback?** Robustness to disturbances and model errors.

---

## 2. PID Control

### Proportional-Integral-Derivative

**PID** computes control from three error terms:

```
u(t) = Kp * e(t) + Ki * ∫e(τ)dτ + Kd * de/dt
```

Where:
- **e(t) = desired - actual** (error)
- **Kp**: Proportional gain (react to current error)
- **Ki**: Integral gain (eliminate steady-state error)
- **Kd**: Derivative gain (dampen oscillations)

### Each Term's Role

**P (Proportional)**:
- React to *current* error
- Too high: oscillations
- Too low: slow response

**I (Integral)**:
- Eliminate *accumulated* error (steady-state offset)
- Too high: overshoot, instability
- Too low: never reaches target

**D (Derivative)**:
- Dampen *rate of change*
- Reduce overshoot
- Too high: amplifies noise

---

## 3. PID Tuning

### Ziegler-Nichols Method

1. Set Ki=0, Kd=0
2. Increase Kp until oscillations start (Kp_crit)
3. Measure oscillation period T_crit
4. Use formulas:
   - Kp = 0.6 * Kp_crit
   - Ki = 2 * Kp / T_crit
   - Kd = Kp * T_crit / 8

### Manual Tuning

1. Start with Kp only (Ki=0, Kd=0)
2. Add Kd to reduce overshoot
3. Add Ki to eliminate steady-state error
4. Iterate

---

## 4. State-Space Control

### State Representation

**State vector**: x = [position, velocity]ᵀ

**System dynamics**:
```
ẋ = A*x + B*u  (state update)
y = C*x        (output)
```

**Example** (mass on spring):
```
x = [position, velocity]ᵀ
u = force
A = [[0, 1], [-k/m, -b/m]]
B = [[0], [1/m]]
```

### State Feedback

**Control law**: u = -K*x (K = gain matrix)

**Goal**: Choose K such that system is stable and responsive.

**LQR (Linear Quadratic Regulator)**: Optimal K that minimizes cost function.

---

## 5. Exercises

### Exercise 5.4.1: PID Controller

Implement PID controller for position tracking.

<InteractivePython
  id="ex-5-4-1"
  title="PID Position Controller"
  starterCode={`import numpy as np

class PIDController:
    def __init__(self, Kp: float, Ki: float, Kd: float, dt: float):
        """Initialize PID controller.

        Args:
            Kp: Proportional gain
            Ki: Integral gain
            Kd: Derivative gain
            dt: Time step
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, setpoint: float, measurement: float) -> float:
        """Compute PID control output.

        Args:
            setpoint: Desired value
            measurement: Actual measured value

        Returns:
            Control signal
        """
        # TODO: Implement PID
        # 1. Compute error
        # 2. Update integral
        # 3. Compute derivative
        # 4. Return Kp*error + Ki*integral + Kd*derivative
        pass

# Test
pid = PIDController(Kp=1.0, Ki=0.1, Kd=0.05, dt=0.01)
setpoint = 1.0
measurement = 0.5
control = pid.compute(setpoint, measurement)
print(f"Control output: {control:.3f}")
`}
  hints={[
    "error = setpoint - measurement",
    "self.integral += error * self.dt",
    "derivative = (error - self.prev_error) / self.dt",
    "self.prev_error = error",
    "return self.Kp*error + self.Ki*self.integral + self.Kd*derivative"
  ]}
/>

---

### Exercise 5.4.2: PID Simulation

Simulate PID controlling a first-order system.

<InteractivePython
  id="ex-5-4-2"
  title="PID Trajectory Tracking Simulation"
  starterCode={`import numpy as np

def simulate_pid_control(setpoint, duration, dt, Kp, Ki, Kd):
    """Simulate PID controlling a simple system.

    System: dv/dt = u (velocity control with force)
            dx/dt = v (position from velocity)

    Args:
        setpoint: Target position
        duration: Simulation time
        dt: Time step
        Kp, Ki, Kd: PID gains

    Returns:
        times, positions arrays
    """
    position = 0.0
    velocity = 0.0
    integral = 0.0
    prev_error = 0.0

    times = []
    positions = []

    # TODO: Simulate for duration
    # At each step:
    #   1. Compute PID control
    #   2. Update velocity: v += u * dt
    #   3. Update position: x += v * dt
    #   4. Record time and position
    pass

# Test
times, positions = simulate_pid_control(
    setpoint=1.0, duration=5.0, dt=0.01, Kp=2.0, Ki=0.5, Kd=1.0
)
print(f"Final position: {positions[-1]:.3f} (target: 1.0)")
print(f"Steady-state error: {abs(1.0 - positions[-1]):.4f}")
`}
  hints={[
    "t = 0.0; while t < duration:",
    "error = setpoint - position",
    "integral += error * dt",
    "derivative = (error - prev_error) / dt",
    "u = Kp*error + Ki*integral + Kd*derivative",
    "velocity += u * dt",
    "position += velocity * dt",
    "times.append(t); positions.append(position); t += dt"
  ]}
/>

---

### Exercise 5.4.3: State-Space Controller

Implement state feedback controller.

<InteractivePython
  id="ex-5-4-3"
  title="State-Space Feedback Controller"
  starterCode={`import numpy as np

def state_feedback_control(x: np.ndarray, K: np.ndarray, x_desired: np.ndarray) -> float:
    """Compute state feedback control: u = -K * (x - x_desired).

    Args:
        x: Current state vector [position, velocity]
        K: Gain vector [Kp, Kd]
        x_desired: Desired state [pos_desired, vel_desired]

    Returns:
        Control signal
    """
    # TODO: Compute u = -K * (x - x_desired)
    # Hint: Use np.dot for matrix multiplication
    pass

# Test
x = np.array([0.5, 0.1])  # Current: pos=0.5, vel=0.1
x_desired = np.array([1.0, 0.0])  # Desired: pos=1.0, vel=0.0
K = np.array([2.0, 0.5])  # Gains: Kp=2.0, Kd=0.5

u = state_feedback_control(x, K, x_desired)
print(f"Control output: {u:.3f}")
`}
  hints={[
    "error = x - x_desired",
    "u = -np.dot(K, error)",
    "return u"
  ]}
/>

---

## 6. Try With AI

### TryWithAI 5.4.1: PID Tuning Strategies

<TryWithAI
  id="tryai-5-4-1"
  title="Learn PID Tuning"
  role="Teacher"
  scenario="You need to tune a PID controller for a new system"
  yourTask="Run Exercise 5.4.2 with different PID gains and observe behavior"
  aiPromptTemplate="I tested PID control with these gains: [Kp, Ki, Kd] = [your values]. I observed: [describe response: overshoot, oscillation, steady-state error]. Can you explain: (1) Why this happened? (2) Which gain should I adjust and in what direction? (3) What are common signs of poor tuning?"
  successCriteria={[
    "You can identify overshoot, oscillation, and steady-state error symptoms",
    "You understand which gain to adjust for each symptom",
    "You can achieve <5% steady-state error and <20% overshoot"
  ]}
  reflectionQuestions={[
    "What happens if Kd is too high with noisy sensors?",
    "Why might integral windup occur and how to prevent it?"
  ]}
/>

---

### TryWithAI 5.4.2: Compare PID vs State-Space

<TryWithAI
  id="tryai-5-4-2"
  title="PID vs State Feedback Trade-offs"
  role="Evaluator"
  scenario="You want to choose between PID and state-space control"
  yourTask="Implement both controllers for the same system, compare performance"
  aiPromptTemplate="I implemented both PID and state-space controllers for [describe system]. PID gave [describe performance], state-space gave [describe performance]. Can you explain: (1) When is PID sufficient? (2) When do you need state-space? (3) What are implementation trade-offs (complexity, tuning, observability)?"
  successCriteria={[
    "You understand PID is simpler but limited to SISO (single input, single output)",
    "You understand state-space handles MIMO (multiple inputs, multiple outputs)",
    "You can identify when model-based control (state-space) is worth complexity"
  ]}
  reflectionQuestions={[
    "How do you get full state feedback if you can't measure velocity?",
    "What is an observer/estimator and when do you need one?"
  ]}
/>

---

## Summary

**Key Takeaways**:

1. **Feedback control**: Measure error, correct continuously (robustness)
2. **PID**: Simple, effective for many systems
   - P: current error
   - I: accumulated error (steady-state)
   - D: rate of change (damping)
3. **Tuning**: Ziegler-Nichols or manual (start P, add D, add I)
4. **State-space**: Model-based, optimal (LQR), handles MIMO
5. **Trade-offs**: Simplicity (PID) vs optimality (state-space)

**What's Next**: [Lesson 5.5: Model Predictive Control](./lesson-05-mpc.md) optimizes control over future horizon.

---

**Estimated completion time**: 60 minutes | **Prerequisites**: Lesson 5.3, Chapter 1 | **Difficulty**: B2 (Upper Intermediate)
