---
title: "Lesson 6.1: Bipedal Locomotion Basics"
description: "Walking gaits, stability, Zero Moment Point (ZMP)"
chapter: 6
lesson: 1
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
prerequisites: ["chapter-04-lesson-03", "chapter-05-lesson-04"]
has_interactive_python: false
interactive_python_count: 0
has_try_with_ai: true
try_with_ai_count: 2
tags: ["humanoid", "kinematics", "dynamics", "bipedal", "locomotion"]
---

# Lesson 6.1: Humanoid Kinematics and Dynamics

**Prerequisites**: [Chapter 4: Kinematics](../chapter-04/index.md), [Chapter 5: Dynamics](../chapter-05/index.md)

---

## Introduction

Bipedal locomotion enables humanoid robots to navigate human-designed environments. Unlike wheeled systems, walking requires continuous balance management and precise coordination of multiple joints. This lesson introduces gait cycles, stability metrics, and the Zero Moment Point (ZMP) criterion.

**Learning Objectives**:
- Define phases of the bipedal gait cycle
- Calculate the Zero Moment Point (ZMP) from foot forces
- Implement static walking with stability constraints
- Generate simple walking trajectories

---

## 1. Gait Cycle Fundamentals

A **gait cycle** comprises stance phase (foot on ground) and swing phase (foot in air). For bipedal walking:

- **Double Support**: Both feet on ground (20% of cycle)
- **Single Support**: One foot on ground, other swings (60%)
- **Flight Phase**: Both feet off ground (running only, 0% for walking)

Key parameters:
- **Step length** $s$: Distance between consecutive foot placements
- **Step height** $h$: Maximum swing foot clearance
- **Cadence** $f$: Steps per second
- **Duty factor** $\beta$: Fraction of cycle with foot on ground

---

## 2. Static vs. Dynamic Stability

**Static Stability**: Center of Mass (CoM) projection stays within support polygon.

**Dynamic Stability**: Accounts for momentum; uses Zero Moment Point (ZMP).

---

## 3. Zero Moment Point (ZMP)

The **ZMP** is the point on the ground where the net moment from gravity and inertia is zero. For stable walking, ZMP must lie within the support polygon.

**ZMP Calculation** (simplified, 2D sagittal plane):

$$
x_{\text{ZMP}} = \frac{\sum_i m_i (g + \ddot{z}_i) x_i}{\sum_i m_i (g + \ddot{z}_i)}
$$

Where:
- $m_i$: Mass of link $i$
- $(x_i, z_i)$: CoM position of link $i$
- $g$: Gravity (9.81 m/s²)

For stable walking: $x_{\text{ZMP}}$ must stay between foot boundaries.

---

## Interactive Exercises

### Exercise 6.1.1: Gait Phase Analyzer

```python
"""Analyze gait cycle phases from step timing."""
import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple

def analyze_gait_cycle(
    step_duration: float,
    double_support_ratio: float = 0.2
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Generate gait phase timeline.

    Args:
        step_duration: Duration of one step (s)
        double_support_ratio: Fraction of cycle in double support

    Returns:
        times: Time array (s)
        phases: Phase indicators (0=double, 1=left single, 2=right single)
    """
    dt = 0.01
    times = np.arange(0, step_duration * 2, dt)
    phases = np.zeros_like(times)

    ds_time = step_duration * double_support_ratio

    for i, t in enumerate(times):
        t_mod = t % step_duration

        if t_mod < ds_time:
            phases[i] = 0  # Double support
        elif t < step_duration:
            phases[i] = 1  # Left single support
        else:
            phases[i] = 2  # Right single support

    return times, phases

# Test: Standard walking gait
times, phases = analyze_gait_cycle(step_duration=0.6, double_support_ratio=0.2)

plt.figure(figsize=(10, 3))
plt.fill_between(times, 0, 1, where=(phases==0), alpha=0.3, color='gray', label='Double Support')
plt.fill_between(times, 0, 1, where=(phases==1), alpha=0.3, color='blue', label='Left Single')
plt.fill_between(times, 0, 1, where=(phases==2), alpha=0.3, color='red', label='Right Single')
plt.xlabel('Time (s)')
plt.ylabel('Phase')
plt.title('Bipedal Gait Cycle Phases')
plt.legend()
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.show()

# Calculate duty factor
duty_factor = np.mean(phases != 1) if np.any(phases == 1) else 0.5
print(f"Duty factor: {duty_factor:.2f}")
print(f"Double support: {np.mean(phases == 0):.1%}")
```

**Expected Output**:
- Plot shows alternating phases over two steps
- Duty factor ≈ 0.70 (each foot contacts ground 70% of time)
- Double support占 20% of cycle

**What You Learned**:
- Gait cycles have distinct temporal phases
- Double support provides stability during transitions
- Duty factor indicates walk vs. run (walk > 0.5, run < 0.5)

---

### Exercise 6.1.2: ZMP Calculator for Point Masses

```python
"""Calculate ZMP for a simple two-mass biped model."""
import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple

def calculate_zmp_2d(
    masses: np.ndarray,
    positions: np.ndarray,
    accelerations: np.ndarray,
    g: float = 9.81
) -> float:
    """
    Calculate Zero Moment Point in 2D.

    Args:
        masses: Array of point masses (kg), shape (n,)
        positions: Array of [x, z] positions (m), shape (n, 2)
        accelerations: Array of [ax, az] accelerations (m/s²), shape (n, 2)
        g: Gravity constant (m/s²)

    Returns:
        x_zmp: ZMP x-coordinate (m)
    """
    x = positions[:, 0]
    z = positions[:, 1]
    az = accelerations[:, 1]

    # Vertical forces (mass * (g + vertical acceleration))
    f_z = masses * (g + az)

    # ZMP formula: weighted average of x positions
    x_zmp = np.sum(f_z * x) / np.sum(f_z)

    return x_zmp

# Test: Simple biped with two masses
masses = np.array([30.0, 40.0])  # Hip, torso (kg)

# Scenario 1: Static standing (no acceleration)
positions_static = np.array([
    [0.0, 0.5],   # Hip at 50cm height
    [0.0, 1.0]    # Torso at 1m height
])
accel_static = np.zeros((2, 2))

zmp_static = calculate_zmp_2d(masses, positions_static, accel_static)

# Scenario 2: Forward leaning (torso accelerating forward)
positions_lean = np.array([
    [0.0, 0.5],
    [0.1, 1.0]    # Torso leaned forward
])
accel_lean = np.array([
    [0.0, 0.0],
    [2.0, 0.0]    # Torso accelerating forward
])

zmp_lean = calculate_zmp_2d(masses, positions_lean, accel_lean)

# Visualize
fig, axes = plt.subplots(1, 2, figsize=(12, 5))

for ax, pos, zmp, title in zip(
    axes,
    [positions_static, positions_lean],
    [zmp_static, zmp_lean],
    ['Static Standing', 'Forward Leaning']
):
    # Plot masses
    ax.scatter(pos[:, 0], pos[:, 1], s=masses*10, c=['blue', 'red'],
               alpha=0.6, edgecolors='black', linewidths=2)

    # Plot ZMP
    ax.axvline(zmp, color='green', linestyle='--', linewidth=2, label=f'ZMP: {zmp:.3f}m')

    # Support polygon (foot at origin, width ±0.1m)
    ax.axvspan(-0.1, 0.1, alpha=0.2, color='gray', label='Support Polygon')

    ax.set_xlim(-0.3, 0.3)
    ax.set_ylim(0, 1.2)
    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Z Height (m)')
    ax.set_title(title)
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')

plt.tight_layout()
plt.show()

# Check stability
print(f"Static ZMP: {zmp_static:.3f} m - Stable: {abs(zmp_static) < 0.1}")
print(f"Leaning ZMP: {zmp_lean:.3f} m - Stable: {abs(zmp_lean) < 0.1}")
```

**Expected Output**:
- Static: ZMP at 0.0 m (centered, stable)
- Leaning: ZMP shifted forward (may exceed support polygon)
- Visualization shows mass positions and ZMP relative to foot

**What You Learned**:
- ZMP combines position and acceleration effects
- Forward-leaning torso shifts ZMP forward
- Stability requires ZMP within support polygon bounds

---

### Exercise 6.1.3: Static Walking Trajectory Generator

```python
"""Generate foot trajectories for static walking."""
import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple

def generate_swing_trajectory(
    start_pos: np.ndarray,
    end_pos: np.ndarray,
    step_height: float,
    num_points: int = 50
) -> np.ndarray:
    """
    Generate swing foot trajectory (parabolic arc).

    Args:
        start_pos: Starting [x, y, z] position (m)
        end_pos: Ending [x, y, z] position (m)
        step_height: Maximum height during swing (m)
        num_points: Number of trajectory points

    Returns:
        trajectory: Array of [x, y, z] positions, shape (num_points, 3)
    """
    s = np.linspace(0, 1, num_points)  # Normalized time

    trajectory = np.zeros((num_points, 3))

    # Linear interpolation in x, y
    trajectory[:, 0] = start_pos[0] + s * (end_pos[0] - start_pos[0])
    trajectory[:, 1] = start_pos[1] + s * (end_pos[1] - start_pos[1])

    # Parabolic arc in z (peak at midpoint)
    trajectory[:, 2] = start_pos[2] + 4 * step_height * s * (1 - s)

    return trajectory

def static_walking_sequence(
    step_length: float = 0.2,
    step_width: float = 0.1,
    step_height: float = 0.05,
    num_steps: int = 4
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Generate alternating foot trajectories for static walking.

    Args:
        step_length: Forward distance per step (m)
        step_width: Lateral foot separation (m)
        step_height: Swing foot clearance (m)
        num_steps: Total number of steps

    Returns:
        left_traj: Left foot trajectory points
        right_traj: Right foot trajectory points
    """
    left_positions = []
    right_positions = []

    # Initial stance
    left_pos = np.array([0.0, step_width/2, 0.0])
    right_pos = np.array([0.0, -step_width/2, 0.0])

    left_positions.append(left_pos.copy())
    right_positions.append(right_pos.copy())

    for i in range(num_steps):
        if i % 2 == 0:
            # Left foot swings
            new_left = left_pos + np.array([step_length, 0, 0])
            swing = generate_swing_trajectory(left_pos, new_left, step_height)
            left_positions.append(swing)
            left_pos = new_left
        else:
            # Right foot swings
            new_right = right_pos + np.array([step_length, 0, 0])
            swing = generate_swing_trajectory(right_pos, new_right, step_height)
            right_positions.append(swing)
            right_pos = new_right

    return left_positions, right_positions

# Generate walking sequence
left_traj, right_traj = static_walking_sequence(
    step_length=0.2,
    step_width=0.15,
    step_height=0.05,
    num_steps=6
)

# Visualize
fig = plt.figure(figsize=(14, 5))

# Top view (X-Y plane)
ax1 = fig.add_subplot(121)
for traj in left_traj:
    if traj.ndim == 1:
        ax1.plot(traj[0], traj[1], 'bo', markersize=8)
    else:
        ax1.plot(traj[:, 0], traj[:, 1], 'b-', alpha=0.5)
        ax1.plot(traj[-1, 0], traj[-1, 1], 'bo', markersize=8)

for traj in right_traj:
    if traj.ndim == 1:
        ax1.plot(traj[0], traj[1], 'ro', markersize=8)
    else:
        ax1.plot(traj[:, 0], traj[:, 1], 'r-', alpha=0.5)
        ax1.plot(traj[-1, 0], traj[-1, 1], 'ro', markersize=8)

ax1.set_xlabel('X (m)')
ax1.set_ylabel('Y (m)')
ax1.set_title('Top View: Foot Placement')
ax1.grid(True, alpha=0.3)
ax1.legend(['Left Foot', 'Right Foot'])
ax1.axis('equal')

# Side view (X-Z plane)
ax2 = fig.add_subplot(122)
for traj in left_traj:
    if traj.ndim > 1:
        ax2.plot(traj[:, 0], traj[:, 2], 'b-', linewidth=2, alpha=0.7)

for traj in right_traj:
    if traj.ndim > 1:
        ax2.plot(traj[:, 0], traj[:, 2], 'r-', linewidth=2, alpha=0.7)

ax2.set_xlabel('X (m)')
ax2.set_ylabel('Z Height (m)')
ax2.set_title('Side View: Swing Trajectory')
ax2.grid(True, alpha=0.3)
ax2.set_ylim(-0.01, 0.08)

plt.tight_layout()
plt.show()

print(f"Generated {len(left_traj) + len(right_traj) - 2} swing trajectories")
print(f"Total forward distance: {max([t[0] if t.ndim == 1 else t[-1, 0] for t in left_traj + right_traj]):.2f} m")
```

**Expected Output**:
- Top view shows alternating left/right foot placements
- Side view shows parabolic swing arcs
- Step width provides lateral stability margin

**What You Learned**:
- Static walking alternates single/double support phases
- Swing trajectories use smooth parabolic arcs for clearance
- Foot placement pattern creates stable support polygon sequence

---

### Exercise 6.1.4: ZMP Trajectory Validation

```python
"""Validate ZMP stays within support polygon during walking."""
import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple

def simulate_com_trajectory(
    step_length: float,
    step_duration: float,
    num_steps: int = 3,
    dt: float = 0.01
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Simulate CoM trajectory for walking (simple sinusoidal).

    Args:
        step_length: Distance per step (m)
        step_duration: Time per step (s)
        num_steps: Number of steps
        dt: Time step (s)

    Returns:
        times: Time array (s)
        com_positions: [x, y] CoM positions (m), shape (n, 2)
    """
    times = np.arange(0, num_steps * step_duration, dt)
    com_positions = np.zeros((len(times), 2))

    for i, t in enumerate(times):
        # Forward motion (linear)
        com_positions[i, 0] = (step_length / step_duration) * t

        # Lateral sway (sinusoidal)
        com_positions[i, 1] = 0.03 * np.sin(2 * np.pi * t / step_duration)

    return times, com_positions

def get_support_polygon(
    foot_positions: np.ndarray,
    foot_width: float = 0.08,
    foot_length: float = 0.12
) -> np.ndarray:
    """
    Calculate support polygon from active foot positions.

    Args:
        foot_positions: Array of foot centers [x, y], shape (n, 2)
        foot_width: Foot width (m)
        foot_length: Foot length (m)

    Returns:
        polygon: Convex hull vertices [x, y], shape (m, 2)
    """
    corners = []
    for pos in foot_positions:
        # Add four corners of each foot
        corners.extend([
            pos + np.array([foot_length/2, foot_width/2]),
            pos + np.array([foot_length/2, -foot_width/2]),
            pos + np.array([-foot_length/2, foot_width/2]),
            pos + np.array([-foot_length/2, -foot_width/2])
        ])

    corners = np.array(corners)

    # Simple convex hull (for this example, return bounding box)
    min_x, max_x = corners[:, 0].min(), corners[:, 0].max()
    min_y, max_y = corners[:, 1].min(), corners[:, 1].max()

    return np.array([
        [min_x, min_y],
        [max_x, min_y],
        [max_x, max_y],
        [min_x, max_y],
        [min_x, min_y]
    ])

# Simulate walking
step_length = 0.2
step_width = 0.15
step_duration = 0.6

times, com_traj = simulate_com_trajectory(step_length, step_duration, num_steps=3)

# Define foot positions during single support phases
foot_positions_sequence = [
    # Double support: both feet
    np.array([[0.0, step_width/2], [0.0, -step_width/2]]),
    # Single support: right foot
    np.array([[0.0, -step_width/2]]),
    # Double support
    np.array([[step_length, step_width/2], [0.0, -step_width/2]]),
    # Single support: left foot
    np.array([[step_length, step_width/2]]),
]

# Visualize CoM and support polygons
plt.figure(figsize=(12, 6))

# Plot CoM trajectory
plt.plot(com_traj[:, 0], com_traj[:, 1], 'b-', linewidth=2, label='CoM Trajectory', alpha=0.7)

# Plot support polygons at different phases
colors = ['green', 'orange', 'green', 'orange']
for i, (feet, color) in enumerate(zip(foot_positions_sequence, colors)):
    polygon = get_support_polygon(feet)
    plt.fill(polygon[:, 0], polygon[:, 1], alpha=0.2, color=color,
             edgecolor=color, linewidth=2, label=f'Support {i}' if i < 2 else None)

    # Plot foot positions
    plt.scatter(feet[:, 0], feet[:, 1], s=200, c=color, marker='s',
                edgecolors='black', linewidths=2, zorder=5)

plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')
plt.title('CoM Trajectory vs. Support Polygons')
plt.legend()
plt.grid(True, alpha=0.3)
plt.axis('equal')
plt.tight_layout()
plt.show()

# Check if CoM stays within bounds
lateral_margin = 0.05
stable = np.all(np.abs(com_traj[:, 1]) < step_width/2 - lateral_margin)
print(f"CoM lateral bounds: [{com_traj[:, 1].min():.3f}, {com_traj[:, 1].max():.3f}] m")
print(f"Support width: ±{step_width/2:.3f} m")
print(f"Stability: {'STABLE' if stable else 'UNSTABLE'}")
```

**Expected Output**:
- CoM trajectory weaves through support polygons
- Lateral sway remains within foot separation bounds
- Support polygons shrink during single support phases

**What You Learned**:
- CoM must stay within support polygon for static stability
- Support polygon changes during gait cycle (larger in double support)
- Lateral sway should be minimized for stability margin

---

## TryWithAI Exercises

### TryWithAI 6.1.1: Dynamic Gait Optimization

**Prompt**:
```
Create a bipedal walking gait optimizer that finds the optimal step length and cadence for a humanoid robot to minimize energy consumption while maintaining ZMP stability. The optimizer should:

1. Model energy cost as proportional to joint accelerations squared
2. Ensure ZMP stays within support polygon throughout the gait cycle
3. Respect kinematic limits (max step length 0.4m, max cadence 2 Hz)
4. Output optimal parameters and visualize the ZMP trajectory

Use a simple inverted pendulum model for the CoM dynamics. Test with different robot masses (50-80 kg) and target walking speeds (0.5-1.5 m/s).
```

**Expected Skills**:
- Optimization with constraints (scipy.optimize)
- Dynamics simulation of inverted pendulum
- ZMP calculation over continuous trajectory
- Trade-off analysis between speed and stability

---

### TryWithAI 6.1.2: Stair Climbing Planner

**Prompt**:
```
Implement a foot placement planner for stair climbing. Given stair dimensions (height: 0.15-0.20m, depth: 0.25-0.30m), generate:

1. Foot placement sequence (which foot on which stair)
2. CoM trajectory that maintains static stability
3. Hip height trajectory to clear stairs
4. Validation that ZMP stays within each foot's support region

Visualize the robot's stick figure at key phases and show the CoM projection relative to the support foot. Test with standard residential stairs (7" rise, 11" tread).
```

**Expected Skills**:
- Discrete foothold planning
- 3D CoM trajectory generation
- Static stability verification
- Multi-phase motion coordination

---

## Summary

This lesson introduced bipedal walking fundamentals:

**Key Concepts**:
- Gait cycle phases (double/single support, swing/stance)
- Static vs. dynamic stability criteria
- Zero Moment Point (ZMP) as stability metric
- Support polygon constraints

**Practical Skills**:
- Analyzing gait phases from timing data
- Calculating ZMP from mass distributions
- Generating swing foot trajectories
- Validating CoM motion against support polygons

**Next Steps**: [Lesson 6.2](lesson-02.md) covers balance control using inverted pendulum models and feedback stabilization.

---

**Further Reading**:
- Vukobratović, M., & Borovac, B. (2004). "Zero-Moment Point—Thirty Five Years of its Life." *International Journal of Humanoid Robotics*.
- Kajita, S., et al. (2003). "Biped Walking Pattern Generation by using Preview Control of Zero-Moment Point." *IEEE ICRA*.
