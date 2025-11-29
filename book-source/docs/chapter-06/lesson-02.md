---
title: "Lesson 6.2: Balance Control"
description: "Inverted pendulum models, stabilization strategies"
chapter: 6
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
prerequisites: ["chapter-06-lesson-01", "chapter-05-lesson-04"]
has_interactive_python: false
interactive_python_count: 0
has_try_with_ai: true
try_with_ai_count: 2
tags: ["bipedal", "locomotion", "balance", "zmp", "gait"]
---

# Lesson 6.2: Bipedal Locomotion Basics

**Prerequisites**: [Lesson 6.1: Humanoid Kinematics and Dynamics](lesson-01.md), [Chapter 5: Dynamics and Control](../chapter-05/index.md)

---

## Introduction

Maintaining balance is critical for humanoid robots. The inverted pendulum model simplifies humanoid dynamics to a single mass on a massless rod, enabling tractable controller design. This lesson covers Linear Inverted Pendulum (LIP) dynamics, stabilization strategies, and ankle/hip balance mechanisms.

**Learning Objectives**:
- Derive Linear Inverted Pendulum (LIP) equations
- Implement PD control for balance stabilization
- Compare ankle vs. hip strategies for balance recovery
- Design ZMP-based reference trajectory tracking

---

## 1. Linear Inverted Pendulum Model (LIP)

The **LIP** approximates a humanoid's Center of Mass (CoM) as a point mass at constant height $z_c$, connected to a pivot (ankle or ZMP) by a massless rod.

**Dynamics** (2D sagittal plane):

$$
\ddot{x} = \omega^2 (x - x_{\text{ZMP}})
$$

Where:
- $x$: CoM horizontal position (m)
- $x_{\text{ZMP}}$: ZMP position (m)
- $\omega = \sqrt{g / z_c}$: Natural frequency (rad/s)
- $g = 9.81$ m/s²

**Key Insight**: If $x_{\text{ZMP}} = x$, then $\ddot{x} = 0$ (equilibrium). If $x > x_{\text{ZMP}}$, CoM accelerates away from ZMP (unstable).

---

## 2. Ankle vs. Hip Strategies

**Ankle Strategy**: Torque at ankle joint to shift ZMP under CoM. Effective for small disturbances.

**Hip Strategy**: Bend at hip to reposition CoM over support foot. Used for large disturbances or when ankle torque limits are reached.

**Stepping Strategy**: Take a step to expand support polygon. Last resort for large perturbations.

---

## 3. ZMP-Based Stabilization

To stabilize LIP, control $x_{\text{ZMP}}$ such that $x \to x_d$ (desired position).

**PD Controller**:

$$
x_{\text{ZMP}} = x + \frac{1}{\omega^2} (\ddot{x}_d - K_p (x - x_d) - K_d \dot{x})
$$

Where:
- $K_p$: Proportional gain (tracking)
- $K_d$: Derivative gain (damping)

---

## Interactive Exercises

### Exercise 6.2.1: LIP Dynamics Simulator

```python
"""Simulate Linear Inverted Pendulum dynamics."""
import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple

def lip_dynamics(
    state: np.ndarray,
    zmp_pos: float,
    z_com: float = 0.8,
    g: float = 9.81
) -> np.ndarray:
    """
    Compute LIP state derivative.

    Args:
        state: [x, x_dot] CoM position and velocity (m, m/s)
        zmp_pos: ZMP position (m)
        z_com: CoM height (m)
        g: Gravity (m/s²)

    Returns:
        state_dot: [x_dot, x_ddot] velocity and acceleration
    """
    x, x_dot = state
    omega = np.sqrt(g / z_com)
    x_ddot = omega**2 * (x - zmp_pos)

    return np.array([x_dot, x_ddot])

def simulate_lip(
    initial_state: np.ndarray,
    zmp_trajectory: np.ndarray,
    times: np.ndarray,
    z_com: float = 0.8
) -> np.ndarray:
    """
    Simulate LIP over time with given ZMP trajectory.

    Args:
        initial_state: Initial [x, x_dot] (m, m/s)
        zmp_trajectory: ZMP positions at each time (m), shape (n,)
        times: Time array (s), shape (n,)
        z_com: CoM height (m)

    Returns:
        states: State history [x, x_dot], shape (n, 2)
    """
    dt = times[1] - times[0]
    states = np.zeros((len(times), 2))
    states[0] = initial_state

    for i in range(len(times) - 1):
        # Euler integration
        state_dot = lip_dynamics(states[i], zmp_trajectory[i], z_com)
        states[i + 1] = states[i] + state_dot * dt

    return states

# Test: Uncontrolled fall from initial offset
times = np.linspace(0, 2.0, 200)
dt = times[1] - times[0]

# Scenario 1: ZMP fixed at origin (uncontrolled)
zmp_fixed = np.zeros_like(times)
states_uncontrolled = simulate_lip(
    initial_state=np.array([0.05, 0.0]),  # 5cm offset, zero velocity
    zmp_trajectory=zmp_fixed,
    times=times,
    z_com=0.8
)

# Scenario 2: ZMP tracks CoM (perfect control)
states_controlled = np.zeros((len(times), 2))
states_controlled[0] = np.array([0.05, 0.0])
zmp_controlled = np.zeros_like(times)

for i in range(len(times) - 1):
    # Set ZMP to current CoM position
    zmp_controlled[i] = states_controlled[i, 0]
    state_dot = lip_dynamics(states_controlled[i], zmp_controlled[i], z_com=0.8)
    states_controlled[i + 1] = states_controlled[i] + state_dot * dt

# Visualize
fig, axes = plt.subplots(2, 1, figsize=(10, 8))

# Position plot
axes[0].plot(times, states_uncontrolled[:, 0], 'r-', linewidth=2, label='Uncontrolled (ZMP=0)')
axes[0].plot(times, states_controlled[:, 0], 'g-', linewidth=2, label='Controlled (ZMP=x)')
axes[0].axhline(0, color='black', linestyle='--', alpha=0.3, label='Target')
axes[0].set_ylabel('CoM Position (m)')
axes[0].set_title('LIP Dynamics: Position')
axes[0].legend()
axes[0].grid(True, alpha=0.3)

# Velocity plot
axes[1].plot(times, states_uncontrolled[:, 1], 'r-', linewidth=2, label='Uncontrolled')
axes[1].plot(times, states_controlled[:, 1], 'g-', linewidth=2, label='Controlled')
axes[1].set_xlabel('Time (s)')
axes[1].set_ylabel('CoM Velocity (m/s)')
axes[1].set_title('LIP Dynamics: Velocity')
axes[1].legend()
axes[1].grid(True, alpha=0.3)

plt.tight_layout()
plt.show()

print(f"Uncontrolled final position: {states_uncontrolled[-1, 0]:.3f} m")
print(f"Controlled final position: {states_controlled[-1, 0]:.3f} m")
print(f"Natural frequency omega: {np.sqrt(9.81/0.8):.2f} rad/s")
```

**Expected Output**:
- Uncontrolled: CoM diverges exponentially from origin
- Controlled: CoM remains at initial offset (no restoring force)
- Natural frequency ≈ 3.5 rad/s for 0.8m height

**What You Learned**:
- LIP dynamics are inherently unstable (exponential divergence)
- Simply matching ZMP to CoM maintains current position (no tracking)
- Stabilization requires feedback control to drive CoM to target

---

### Exercise 6.2.2: PD Balance Controller

```python
"""Implement PD controller for LIP stabilization."""
import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple

def pd_controller(
    state: np.ndarray,
    target: float,
    Kp: float,
    Kd: float,
    z_com: float = 0.8,
    g: float = 9.81
) -> float:
    """
    Compute ZMP command using PD control.

    Args:
        state: [x, x_dot] current CoM state (m, m/s)
        target: Desired CoM position (m)
        Kp: Proportional gain
        Kd: Derivative gain
        z_com: CoM height (m)
        g: Gravity (m/s²)

    Returns:
        zmp_cmd: Commanded ZMP position (m)
    """
    x, x_dot = state
    omega = np.sqrt(g / z_com)

    # Error terms
    error = x - target
    error_dot = x_dot  # Target velocity is zero

    # PD control law
    # x_ddot_desired = -Kp * error - Kd * error_dot
    # From LIP: x_ddot = omega^2 * (x - x_zmp)
    # Solve for x_zmp:
    x_ddot_desired = -Kp * error - Kd * error_dot
    zmp_cmd = x - x_ddot_desired / (omega**2)

    return zmp_cmd

def simulate_pd_control(
    initial_state: np.ndarray,
    target: float,
    Kp: float,
    Kd: float,
    times: np.ndarray,
    z_com: float = 0.8,
    zmp_limits: Tuple[float, float] = (-0.1, 0.1)
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Simulate LIP with PD control.

    Args:
        initial_state: Initial [x, x_dot]
        target: Desired CoM position (m)
        Kp: Proportional gain
        Kd: Derivative gain
        times: Time array (s)
        z_com: CoM height (m)
        zmp_limits: (min, max) ZMP bounds (m)

    Returns:
        states: State history [x, x_dot]
        zmp_history: ZMP command history
    """
    dt = times[1] - times[0]
    states = np.zeros((len(times), 2))
    zmp_history = np.zeros(len(times))

    states[0] = initial_state

    for i in range(len(times) - 1):
        # Compute ZMP command
        zmp_cmd = pd_controller(states[i], target, Kp, Kd, z_com)

        # Apply ZMP limits (saturation)
        zmp_cmd = np.clip(zmp_cmd, zmp_limits[0], zmp_limits[1])
        zmp_history[i] = zmp_cmd

        # Simulate dynamics
        state_dot = lip_dynamics(states[i], zmp_cmd, z_com)
        states[i + 1] = states[i] + state_dot * dt

    return states, zmp_history

# Test different PD gains
times = np.linspace(0, 3.0, 300)
initial_state = np.array([0.15, 0.0])  # 15cm offset
target = 0.0

gains = [
    (5.0, 3.0, 'Under-damped'),
    (10.0, 6.0, 'Critically damped'),
    (15.0, 10.0, 'Over-damped')
]

fig, axes = plt.subplots(3, 1, figsize=(10, 10))

for (Kp, Kd, label), color in zip(gains, ['blue', 'green', 'red']):
    states, zmp = simulate_pd_control(initial_state, target, Kp, Kd, times)

    # Position
    axes[0].plot(times, states[:, 0], color=color, linewidth=2, label=f'{label} (Kp={Kp}, Kd={Kd})')

    # Velocity
    axes[1].plot(times, states[:, 1], color=color, linewidth=2, label=label)

    # ZMP
    axes[2].plot(times, zmp, color=color, linewidth=2, label=label)

# Format plots
axes[0].axhline(target, color='black', linestyle='--', alpha=0.5, label='Target')
axes[0].set_ylabel('CoM Position (m)')
axes[0].set_title('PD Control Response: Position')
axes[0].legend()
axes[0].grid(True, alpha=0.3)

axes[1].set_ylabel('CoM Velocity (m/s)')
axes[1].set_title('PD Control Response: Velocity')
axes[1].legend()
axes[1].grid(True, alpha=0.3)

axes[2].axhline(0.1, color='red', linestyle=':', alpha=0.5, label='ZMP Limits')
axes[2].axhline(-0.1, color='red', linestyle=':', alpha=0.5)
axes[2].set_xlabel('Time (s)')
axes[2].set_ylabel('ZMP Command (m)')
axes[2].set_title('PD Control Response: ZMP')
axes[2].legend()
axes[2].grid(True, alpha=0.3)

plt.tight_layout()
plt.show()

# Analyze settling time (within 1% of target)
for Kp, Kd, label in gains:
    states, _ = simulate_pd_control(initial_state, target, Kp, Kd, times)
    settling_idx = np.where(np.abs(states[:, 0] - target) < 0.01 * abs(initial_state[0]))[0]
    settling_time = times[settling_idx[0]] if len(settling_idx) > 0 else np.inf
    print(f"{label}: Settling time = {settling_time:.2f} s")
```

**Expected Output**:
- Under-damped: Oscillations before settling
- Critically damped: Fast settling without overshoot
- Over-damped: Slow, monotonic approach
- ZMP stays within ±10cm limits

**What You Learned**:
- PD control stabilizes LIP by adjusting ZMP
- Higher Kd reduces oscillations (damping)
- Higher Kp increases response speed but can cause overshoot
- ZMP saturation limits achievable accelerations

---

### Exercise 6.2.3: Ankle vs. Hip Strategy Comparison

```python
"""Compare ankle and hip balance strategies."""
import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple

def ankle_strategy(
    com_offset: float,
    max_ankle_torque: float = 100.0,
    robot_mass: float = 60.0,
    com_height: float = 0.8
) -> Tuple[float, bool]:
    """
    Compute ZMP shift using ankle strategy.

    Args:
        com_offset: CoM horizontal offset from ankle (m)
        max_ankle_torque: Maximum ankle torque (Nm)
        robot_mass: Robot mass (kg)
        com_height: CoM height (m)

    Returns:
        zmp_shift: Achievable ZMP displacement (m)
        feasible: Whether ankle strategy can handle disturbance
    """
    # Required torque to balance: tau = m * g * com_offset
    required_torque = robot_mass * 9.81 * com_offset

    if abs(required_torque) <= max_ankle_torque:
        # ZMP shift equals CoM offset (perfect compensation)
        zmp_shift = com_offset
        feasible = True
    else:
        # Saturate at max torque
        zmp_shift = np.sign(com_offset) * max_ankle_torque / (robot_mass * 9.81)
        feasible = False

    return zmp_shift, feasible

def hip_strategy(
    com_offset: float,
    max_hip_angle: float = 0.3,
    com_height: float = 0.8
) -> Tuple[float, bool]:
    """
    Compute hip angle to reposition CoM.

    Args:
        com_offset: CoM horizontal offset from support (m)
        max_hip_angle: Maximum hip flexion angle (rad)
        com_height: CoM height (m)

    Returns:
        hip_angle: Required hip angle (rad)
        feasible: Whether hip strategy can handle disturbance
    """
    # Approximate: hip_angle ≈ com_offset / com_height (small angle)
    required_angle = com_offset / com_height

    if abs(required_angle) <= max_hip_angle:
        hip_angle = required_angle
        feasible = True
    else:
        hip_angle = np.sign(required_angle) * max_hip_angle
        feasible = False

    return hip_angle, feasible

# Test range of disturbances
com_offsets = np.linspace(0, 0.3, 50)  # 0 to 30cm

ankle_shifts = []
ankle_feasibility = []
hip_angles = []
hip_feasibility = []

for offset in com_offsets:
    zmp, ankle_ok = ankle_strategy(offset, max_ankle_torque=100.0)
    ankle_shifts.append(zmp)
    ankle_feasibility.append(ankle_ok)

    angle, hip_ok = hip_strategy(offset, max_hip_angle=0.3)
    hip_angles.append(angle)
    hip_feasibility.append(hip_ok)

ankle_shifts = np.array(ankle_shifts)
hip_angles = np.array(hip_angles)

# Visualize
fig, axes = plt.subplots(1, 2, figsize=(14, 5))

# Ankle strategy
ax1 = axes[0]
ax1.plot(com_offsets * 100, ankle_shifts * 100, 'b-', linewidth=2, label='ZMP Shift')
ax1.axhline(10, color='red', linestyle='--', alpha=0.5, label='Support Limit (±10cm)')
ax1.axhline(-10, color='red', linestyle='--', alpha=0.5)

# Mark feasibility boundary
feasible_ankle = com_offsets[np.array(ankle_feasibility)]
infeasible_ankle = com_offsets[~np.array(ankle_feasibility)]

if len(feasible_ankle) > 0:
    ax1.axvspan(0, feasible_ankle[-1] * 100, alpha=0.2, color='green', label='Feasible')
if len(infeasible_ankle) > 0:
    ax1.axvspan(infeasible_ankle[0] * 100, com_offsets[-1] * 100, alpha=0.2, color='red', label='Infeasible')

ax1.set_xlabel('CoM Offset (cm)')
ax1.set_ylabel('ZMP Shift (cm)')
ax1.set_title('Ankle Strategy: ZMP Compensation')
ax1.legend()
ax1.grid(True, alpha=0.3)

# Hip strategy
ax2 = axes[1]
ax2.plot(com_offsets * 100, np.rad2deg(hip_angles), 'g-', linewidth=2, label='Hip Angle')
ax2.axhline(np.rad2deg(0.3), color='red', linestyle='--', alpha=0.5, label='Hip Limit (±17°)')
ax2.axhline(-np.rad2deg(0.3), color='red', linestyle='--', alpha=0.5)

# Mark feasibility
feasible_hip = com_offsets[np.array(hip_feasibility)]
infeasible_hip = com_offsets[~np.array(hip_feasibility)]

if len(feasible_hip) > 0:
    ax2.axvspan(0, feasible_hip[-1] * 100, alpha=0.2, color='green', label='Feasible')
if len(infeasible_hip) > 0:
    ax2.axvspan(infeasible_hip[0] * 100, com_offsets[-1] * 100, alpha=0.2, color='red', label='Infeasible')

ax2.set_xlabel('CoM Offset (cm)')
ax2.set_ylabel('Hip Angle (degrees)')
ax2.set_title('Hip Strategy: Hip Flexion')
ax2.legend()
ax2.grid(True, alpha=0.3)

plt.tight_layout()
plt.show()

# Find crossover point
ankle_limit = ankle_shifts[np.array(ankle_feasibility)][-1] if np.any(ankle_feasibility) else 0
hip_limit = com_offsets[np.array(hip_feasibility)][-1] if np.any(hip_feasibility) else 0

print(f"Ankle strategy limit: {ankle_limit * 100:.1f} cm")
print(f"Hip strategy limit: {hip_limit * 100:.1f} cm")
print(f"Recommendation: Use ankle for < {ankle_limit * 100:.1f} cm, hip for larger disturbances")
```

**Expected Output**:
- Ankle strategy saturates around 10-15cm offset (torque limit)
- Hip strategy handles larger offsets (up to 24cm for 0.3 rad limit)
- Green/red regions show feasible/infeasible ranges

**What You Learned**:
- Ankle strategy is efficient for small disturbances (direct ZMP control)
- Hip strategy extends balance range but requires larger joint motion
- Hybrid approach: ankle first, then hip if needed, finally step

---

### Exercise 6.2.4: Push Recovery Simulation

```python
"""Simulate balance recovery from external push."""
import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple

def simulate_push_recovery(
    push_magnitude: float,
    push_time: float,
    Kp: float = 10.0,
    Kd: float = 6.0,
    duration: float = 3.0,
    dt: float = 0.01
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Simulate LIP response to impulsive push.

    Args:
        push_magnitude: Push impulse (m/s velocity change)
        push_time: Time of push application (s)
        Kp: PD proportional gain
        Kd: PD derivative gain
        duration: Simulation duration (s)
        dt: Time step (s)

    Returns:
        times: Time array
        states: [x, x_dot] state history
        zmp: ZMP command history
    """
    times = np.arange(0, duration, dt)
    states = np.zeros((len(times), 2))
    zmp = np.zeros(len(times))

    z_com = 0.8
    omega = np.sqrt(9.81 / z_com)
    target = 0.0

    for i in range(len(times) - 1):
        # Apply push at specified time
        if abs(times[i] - push_time) < dt / 2:
            states[i, 1] += push_magnitude  # Velocity impulse

        # PD control
        x, x_dot = states[i]
        error = x - target
        x_ddot_cmd = -Kp * error - Kd * x_dot
        zmp_cmd = x - x_ddot_cmd / (omega**2)

        # Saturate ZMP
        zmp_cmd = np.clip(zmp_cmd, -0.1, 0.1)
        zmp[i] = zmp_cmd

        # Dynamics
        x_ddot = omega**2 * (x - zmp_cmd)
        states[i + 1, 0] = x + x_dot * dt
        states[i + 1, 1] = x_dot + x_ddot * dt

    return times, states, zmp

# Test different push magnitudes
push_magnitudes = [0.1, 0.2, 0.3]  # m/s
colors = ['blue', 'orange', 'red']
push_time = 0.5

fig, axes = plt.subplots(3, 1, figsize=(10, 10))

for push_mag, color in zip(push_magnitudes, colors):
    times, states, zmp = simulate_push_recovery(
        push_magnitude=push_mag,
        push_time=push_time,
        Kp=10.0,
        Kd=6.0
    )

    # Position
    axes[0].plot(times, states[:, 0] * 100, color=color, linewidth=2,
                 label=f'Push: {push_mag} m/s')

    # Velocity
    axes[1].plot(times, states[:, 1], color=color, linewidth=2,
                 label=f'Push: {push_mag} m/s')

    # ZMP
    axes[2].plot(times, zmp * 100, color=color, linewidth=2,
                 label=f'Push: {push_mag} m/s')

# Format
axes[0].axvline(push_time, color='black', linestyle='--', alpha=0.3, label='Push Time')
axes[0].set_ylabel('CoM Position (cm)')
axes[0].set_title('Push Recovery: CoM Position')
axes[0].legend()
axes[0].grid(True, alpha=0.3)

axes[1].axvline(push_time, color='black', linestyle='--', alpha=0.3)
axes[1].set_ylabel('CoM Velocity (m/s)')
axes[1].set_title('Push Recovery: CoM Velocity')
axes[1].legend()
axes[1].grid(True, alpha=0.3)

axes[2].axhline(10, color='red', linestyle=':', alpha=0.5, label='ZMP Limit')
axes[2].axhline(-10, color='red', linestyle=':', alpha=0.5)
axes[2].axvline(push_time, color='black', linestyle='--', alpha=0.3)
axes[2].set_xlabel('Time (s)')
axes[2].set_ylabel('ZMP (cm)')
axes[2].set_title('Push Recovery: ZMP Command')
axes[2].legend()
axes[2].grid(True, alpha=0.3)

plt.tight_layout()
plt.show()

# Analyze recovery time
for push_mag in push_magnitudes:
    times, states, _ = simulate_push_recovery(push_mag, push_time)
    recovery_idx = np.where((times > push_time + 0.5) & (np.abs(states[:, 0]) < 0.01))[0]
    if len(recovery_idx) > 0:
        recovery_time = times[recovery_idx[0]] - push_time
        max_excursion = np.max(np.abs(states[times > push_time, 0])) * 100
        print(f"Push {push_mag} m/s: Recovery in {recovery_time:.2f}s, Max: {max_excursion:.1f}cm")
    else:
        print(f"Push {push_mag} m/s: Did not recover (exceeded limits)")
```

**Expected Output**:
- Larger pushes → larger CoM excursions
- ZMP saturates for strong pushes (limits recovery acceleration)
- All disturbances < 0.3 m/s recover within 2-3 seconds

**What You Learned**:
- Push recovery requires rapid ZMP adjustment
- ZMP saturation limits disturbance rejection capability
- Recovery time increases with disturbance magnitude
- If ZMP limits insufficient, stepping strategy required

---

## TryWithAI Exercises

### TryWithAI 6.2.1: Adaptive Balance Controller

**Prompt**:
```
Design an adaptive balance controller for a humanoid robot that estimates and compensates for unknown external forces (e.g., wind, payload). The controller should:

1. Use a Kalman filter to estimate constant external force from CoM state observations
2. Augment PD control with feedforward compensation based on estimated force
3. Demonstrate improved disturbance rejection compared to standard PD control
4. Visualize force estimation convergence and CoM tracking error

Test with step force (constant push) and ramp force (gradually increasing load). Compare performance with and without adaptation.
```

**Expected Skills**:
- Kalman filtering for state and parameter estimation
- Feedforward control design
- Disturbance observer patterns
- Performance metric analysis (settling time, overshoot)

---

### TryWithAI 6.2.2: Multi-Contact Balance Optimization

**Prompt**:
```
Implement a balance controller for a humanoid using multiple contact points (two feet, optionally hands on wall). The system should:

1. Compute the combined support polygon from all active contacts
2. Optimize ZMP location within the polygon to minimize control effort
3. Decide when to activate/deactivate contacts based on stability margin
4. Visualize the support polygon, CoM projection, and contact forces

Test scenarios: narrow ledge (requires hand support), wide stance (two feet only), single-foot balance. Show how additional contacts expand stability region.
```

**Expected Skills**:
- Convex hull computation for support polygon
- Constrained optimization (minimize effort subject to stability)
- Contact state management
- Force distribution across multiple contacts

---

## Summary

This lesson covered balance control for humanoid robots:

**Key Concepts**:
- Linear Inverted Pendulum (LIP) simplifies humanoid dynamics
- ZMP control stabilizes LIP by adjusting ground reaction point
- Ankle, hip, and stepping strategies handle different disturbance magnitudes
- PD control provides feedback stabilization

**Practical Skills**:
- Simulating LIP dynamics
- Implementing PD controllers for ZMP tracking
- Comparing ankle vs. hip balance strategies
- Analyzing push recovery performance

**Next Steps**: [Lesson 6.3](lesson-03.md) explores whole-body motion planning with inverse kinematics and center-of-mass constraints.

---

**Further Reading**:
- Kajita, S., et al. (2001). "The 3D Linear Inverted Pendulum Mode: A simple modeling for a biped walking pattern generation." *IEEE/RSJ IROS*.
- Pratt, J., et al. (2006). "Capture Point: A Step toward Humanoid Push Recovery." *IEEE-RAS Humanoids*.
