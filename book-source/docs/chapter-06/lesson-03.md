---
title: "Lesson 6.3: Whole-Body Motion Planning"
description: "Inverse kinematics for humanoids, center-of-mass control"
chapter: 6
lesson: 3
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
prerequisites: ["chapter-04-lesson-03", "chapter-06-lesson-02"]
has_interactive_python: false
interactive_python_count: 0
has_try_with_ai: true
try_with_ai_count: 2
tags: ["balance", "inverted-pendulum", "stabilization", "control"]
---

# Lesson 6.3: Balance Control

**Prerequisites**: [Chapter 4: Inverse Kinematics](../chapter-04/lesson-03.md), [Lesson 6.2: Bipedal Locomotion Basics](lesson-02.md)

---

## Introduction

Humanoid robots have many degrees of freedom (typically 20-40 joints), creating complex motion planning challenges. Whole-body planning coordinates all joints to achieve task goals while maintaining balance, respecting joint limits, and avoiding self-collisions. This lesson covers inverse kinematics for redundant systems, center-of-mass constraints, and prioritized task control.

**Learning Objectives**:
- Formulate whole-body IK with multiple constraints
- Implement center-of-mass (CoM) control during manipulation
- Use task prioritization for redundancy resolution
- Plan collision-free whole-body motions

---

## 1. Whole-Body Inverse Kinematics

For a humanoid with $n$ joints and multiple task constraints (hand position, CoM, orientation), we solve:

$$
\min_{\mathbf{q}} \sum_i w_i \| \mathbf{J}_i(\mathbf{q}) \dot{\mathbf{q}} - \dot{\mathbf{x}}_i^d \|^2 + \lambda \| \dot{\mathbf{q}} \|^2
$$

Where:
- $\mathbf{J}_i$: Jacobian for task $i$ (e.g., hand position, CoM)
- $\dot{\mathbf{x}}_i^d$: Desired task velocity
- $w_i$: Task weight/priority
- $\lambda$: Regularization (prefer small joint velocities)

**Closed-Form Solution** (damped least squares):

$$
\dot{\mathbf{q}} = (\mathbf{J}^T \mathbf{W} \mathbf{J} + \lambda \mathbf{I})^{-1} \mathbf{J}^T \mathbf{W} \dot{\mathbf{x}}^d
$$

Where $\mathbf{W} = \text{diag}(w_1, \ldots, w_k)$ are task weights.

---

## 2. Center of Mass (CoM) Control

The CoM position is critical for balance. For a robot with link masses $m_i$ and CoM positions $\mathbf{p}_i(\mathbf{q})$:

$$
\mathbf{p}_{\text{CoM}} = \frac{1}{M} \sum_i m_i \mathbf{p}_i(\mathbf{q})
$$

**CoM Jacobian**:

$$
\mathbf{J}_{\text{CoM}} = \frac{1}{M} \sum_i m_i \mathbf{J}_i
$$

Where $\mathbf{J}_i$ is the position Jacobian for link $i$.

---

## 3. Task Prioritization

When tasks conflict (e.g., reach target while keeping CoM centered), use **prioritized IK**:

1. **Primary task** (highest priority, e.g., balance/CoM)
2. **Secondary task** (e.g., hand position)
3. **Tertiary task** (e.g., joint limits, posture)

**Null-Space Projection**: Execute secondary task in null space of primary task:

$$
\dot{\mathbf{q}} = \mathbf{J}_1^\dagger \dot{\mathbf{x}}_1 + (\mathbf{I} - \mathbf{J}_1^\dagger \mathbf{J}_1) \mathbf{J}_2^\dagger \dot{\mathbf{x}}_2
$$

Where $\mathbf{J}_1^\dagger$ is the pseudoinverse of the primary task Jacobian.

---

## Interactive Exercises

### Exercise 6.3.1: Center of Mass Calculator

```python
"""Compute center of mass for a multi-link robot."""
import numpy as np
import matplotlib.pyplot as plt
from typing import List, Tuple

def forward_kinematics_chain(
    joint_angles: np.ndarray,
    link_lengths: np.ndarray,
    base_pos: np.ndarray = np.array([0.0, 0.0])
) -> np.ndarray:
    """
    Compute link endpoint positions for planar serial chain.

    Args:
        joint_angles: Joint angles (rad), shape (n,)
        link_lengths: Link lengths (m), shape (n,)
        base_pos: Base position [x, y] (m)

    Returns:
        positions: Link endpoints [x, y], shape (n+1, 2)
    """
    n = len(joint_angles)
    positions = np.zeros((n + 1, 2))
    positions[0] = base_pos

    cumulative_angle = 0.0
    for i in range(n):
        cumulative_angle += joint_angles[i]
        positions[i + 1] = positions[i] + link_lengths[i] * np.array([
            np.cos(cumulative_angle),
            np.sin(cumulative_angle)
        ])

    return positions

def compute_com(
    link_positions: np.ndarray,
    link_masses: np.ndarray
) -> np.ndarray:
    """
    Compute center of mass from link positions.

    Args:
        link_positions: Link CoM positions [x, y], shape (n, 2)
        link_masses: Link masses (kg), shape (n,)

    Returns:
        com: Center of mass [x, y] (m)
    """
    total_mass = np.sum(link_masses)
    com = np.sum(link_positions * link_masses[:, np.newaxis], axis=0) / total_mass
    return com

# Define 4-link planar humanoid (simplified: torso, thigh, shin, foot)
link_lengths = np.array([0.3, 0.4, 0.4, 0.1])  # m
link_masses = np.array([20.0, 10.0, 5.0, 2.0])  # kg

# Test different configurations
configs = [
    (np.array([0.0, 0.0, 0.0, 0.0]), 'Standing Straight'),
    (np.array([0.0, -0.2, 0.4, -0.2]), 'Slight Crouch'),
    (np.array([0.3, -0.5, 0.8, -0.3]), 'Deep Crouch')
]

fig, axes = plt.subplots(1, 3, figsize=(15, 5))

for ax, (angles, title) in zip(axes, configs):
    # Compute forward kinematics
    positions = forward_kinematics_chain(angles, link_lengths)

    # Link CoMs are at midpoints
    link_com_positions = (positions[:-1] + positions[1:]) / 2

    # Compute overall CoM
    com = compute_com(link_com_positions, link_masses)

    # Plot robot
    ax.plot(positions[:, 0], positions[:, 1], 'o-', color='blue',
            markersize=8, linewidth=3, label='Links')

    # Plot link CoMs
    ax.scatter(link_com_positions[:, 0], link_com_positions[:, 1],
               s=link_masses * 10, c='orange', alpha=0.6,
               edgecolors='black', linewidths=1.5, label='Link CoMs')

    # Plot overall CoM
    ax.scatter(com[0], com[1], s=300, c='red', marker='X',
               edgecolors='black', linewidths=2, label='Overall CoM', zorder=10)

    # Support polygon (foot)
    foot_x = positions[-1, 0]
    ax.axvspan(foot_x - 0.05, foot_x + 0.05, alpha=0.2, color='green',
               label='Support')

    # CoM projection
    ax.plot([com[0], com[0]], [0, com[1]], 'r--', linewidth=2, alpha=0.5,
            label='CoM Projection')

    ax.set_xlim(-0.2, 0.6)
    ax.set_ylim(-0.1, 1.3)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Z (m)')
    ax.set_title(f'{title}\nCoM X: {com[0]:.3f}m')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')

plt.tight_layout()
plt.show()

# Analyze stability
for angles, title in configs:
    positions = forward_kinematics_chain(angles, link_lengths)
    link_coms = (positions[:-1] + positions[1:]) / 2
    com = compute_com(link_coms, link_masses)

    foot_x = positions[-1, 0]
    stable = abs(com[0] - foot_x) < 0.05

    print(f"{title}: CoM at {com[0]:.3f}m, Foot at {foot_x:.3f}m - {'STABLE' if stable else 'UNSTABLE'}")
```

**Expected Output**:
- Standing: CoM directly above foot (stable)
- Crouch: CoM shifts forward as joints flex
- Visualization shows CoM relative to support polygon

**What You Learned**:
- CoM is mass-weighted average of link positions
- Joint configuration significantly affects CoM location
- Stability requires CoM projection within support polygon
- Crouching can destabilize if not compensated

---

### Exercise 6.3.2: CoM Jacobian and Control

```python
"""Compute CoM Jacobian and implement velocity control."""
import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple

def com_jacobian_planar(
    joint_angles: np.ndarray,
    link_lengths: np.ndarray,
    link_masses: np.ndarray
) -> np.ndarray:
    """
    Compute CoM Jacobian for planar chain (numerical approximation).

    Args:
        joint_angles: Joint angles (rad), shape (n,)
        link_lengths: Link lengths (m), shape (n,)
        link_masses: Link masses (kg), shape (n,)

    Returns:
        J_com: CoM Jacobian [2 x n] (m)
    """
    n = len(joint_angles)
    J_com = np.zeros((2, n))
    epsilon = 1e-6

    # Compute current CoM
    positions = forward_kinematics_chain(joint_angles, link_lengths)
    link_coms = (positions[:-1] + positions[1:]) / 2
    com_0 = compute_com(link_coms, link_masses)

    # Numerical derivative for each joint
    for i in range(n):
        angles_perturbed = joint_angles.copy()
        angles_perturbed[i] += epsilon

        positions_p = forward_kinematics_chain(angles_perturbed, link_lengths)
        link_coms_p = (positions_p[:-1] + positions_p[1:]) / 2
        com_p = compute_com(link_coms_p, link_masses)

        J_com[:, i] = (com_p - com_0) / epsilon

    return J_com

def com_velocity_control(
    current_angles: np.ndarray,
    com_velocity_desired: np.ndarray,
    link_lengths: np.ndarray,
    link_masses: np.ndarray,
    damping: float = 0.01
) -> np.ndarray:
    """
    Compute joint velocities to achieve desired CoM velocity.

    Args:
        current_angles: Current joint angles (rad), shape (n,)
        com_velocity_desired: Desired CoM velocity [vx, vy] (m/s)
        link_lengths: Link lengths (m)
        link_masses: Link masses (kg)
        damping: Damping factor for pseudoinverse

    Returns:
        joint_velocities: Joint velocities (rad/s), shape (n,)
    """
    J_com = com_jacobian_planar(current_angles, link_lengths, link_masses)

    # Damped least squares pseudoinverse
    J_pinv = np.linalg.solve(
        J_com.T @ J_com + damping * np.eye(len(current_angles)),
        J_com.T
    )

    joint_velocities = J_pinv @ com_velocity_desired
    return joint_velocities

# Test: Control CoM to move horizontally
link_lengths = np.array([0.3, 0.4, 0.4, 0.1])
link_masses = np.array([20.0, 10.0, 5.0, 2.0])

# Initial configuration (standing)
q0 = np.array([0.0, -0.1, 0.2, -0.1])

# Simulate CoM control
dt = 0.05
duration = 2.0
times = np.arange(0, duration, dt)

q_history = np.zeros((len(times), len(q0)))
com_history = np.zeros((len(times), 2))
q_history[0] = q0

# Desired CoM velocity (move forward at 0.1 m/s)
com_vel_desired = np.array([0.1, 0.0])

for i in range(len(times) - 1):
    # Compute joint velocities for desired CoM motion
    q_dot = com_velocity_control(
        q_history[i], com_vel_desired, link_lengths, link_masses
    )

    # Integrate (Euler)
    q_history[i + 1] = q_history[i] + q_dot * dt

    # Compute CoM
    positions = forward_kinematics_chain(q_history[i], link_lengths)
    link_coms = (positions[:-1] + positions[1:]) / 2
    com_history[i] = compute_com(link_coms, link_masses)

# Final CoM
positions_final = forward_kinematics_chain(q_history[-1], link_lengths)
link_coms_final = (positions_final[:-1] + positions_final[1:]) / 2
com_history[-1] = compute_com(link_coms_final, link_masses)

# Visualize
fig, axes = plt.subplots(2, 2, figsize=(12, 10))

# Initial and final configurations
for ax, idx, title in zip([axes[0, 0], axes[0, 1]], [0, -1], ['Initial', 'Final']):
    positions = forward_kinematics_chain(q_history[idx], link_lengths)
    link_coms = (positions[:-1] + positions[1:]) / 2
    com = compute_com(link_coms, link_masses)

    ax.plot(positions[:, 0], positions[:, 1], 'o-', color='blue',
            markersize=8, linewidth=3)
    ax.scatter(com[0], com[1], s=300, c='red', marker='X',
               edgecolors='black', linewidths=2, zorder=10)
    ax.set_xlim(-0.1, 0.5)
    ax.set_ylim(-0.1, 1.3)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Z (m)')
    ax.set_title(f'{title} Config (t={times[idx]:.1f}s)')
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')

# CoM trajectory
axes[1, 0].plot(com_history[:, 0], com_history[:, 1], 'r-', linewidth=2, label='Actual')
axes[1, 0].plot([com_history[0, 0], com_history[0, 0] + com_vel_desired[0] * duration],
                [com_history[0, 1], com_history[0, 1] + com_vel_desired[1] * duration],
                'g--', linewidth=2, label='Desired')
axes[1, 0].set_xlabel('CoM X (m)')
axes[1, 0].set_ylabel('CoM Z (m)')
axes[1, 0].set_title('CoM Trajectory')
axes[1, 0].legend()
axes[1, 0].grid(True, alpha=0.3)
axes[1, 0].axis('equal')

# Joint angles over time
for i in range(len(q0)):
    axes[1, 1].plot(times, np.rad2deg(q_history[:, i]), linewidth=2, label=f'Joint {i+1}')
axes[1, 1].set_xlabel('Time (s)')
axes[1, 1].set_ylabel('Joint Angle (degrees)')
axes[1, 1].set_title('Joint Trajectories')
axes[1, 1].legend()
axes[1, 1].grid(True, alpha=0.3)

plt.tight_layout()
plt.show()

# Analyze tracking
com_vel_actual = np.diff(com_history, axis=0) / dt
com_vel_avg = np.mean(com_vel_actual, axis=0)
print(f"Desired CoM velocity: {com_vel_desired}")
print(f"Actual CoM velocity (avg): {com_vel_avg}")
print(f"Tracking error: {np.linalg.norm(com_vel_avg - com_vel_desired):.4f} m/s")
```

**Expected Output**:
- CoM moves forward approximately at desired velocity
- Joint angles adjust to maintain CoM motion
- Some tracking error due to numerical integration and damping

**What You Learned**:
- CoM Jacobian maps joint velocities to CoM velocity
- Pseudoinverse enables velocity-level control
- Multiple joint solutions exist for same CoM motion (redundancy)
- Damping improves numerical stability

---

### Exercise 6.3.3: Task Prioritization (CoM + End-Effector)

```python
"""Implement prioritized IK: primary CoM control, secondary hand reaching."""
import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple

def end_effector_jacobian_planar(
    joint_angles: np.ndarray,
    link_lengths: np.ndarray
) -> np.ndarray:
    """
    Compute end-effector Jacobian for planar chain (numerical).

    Args:
        joint_angles: Joint angles (rad), shape (n,)
        link_lengths: Link lengths (m), shape (n,)

    Returns:
        J_ee: End-effector Jacobian [2 x n]
    """
    n = len(joint_angles)
    J_ee = np.zeros((2, n))
    epsilon = 1e-6

    positions_0 = forward_kinematics_chain(joint_angles, link_lengths)
    ee_0 = positions_0[-1]

    for i in range(n):
        angles_p = joint_angles.copy()
        angles_p[i] += epsilon
        positions_p = forward_kinematics_chain(angles_p, link_lengths)
        ee_p = positions_p[-1]
        J_ee[:, i] = (ee_p - ee_0) / epsilon

    return J_ee

def prioritized_ik(
    current_angles: np.ndarray,
    com_velocity_desired: np.ndarray,
    ee_velocity_desired: np.ndarray,
    link_lengths: np.ndarray,
    link_masses: np.ndarray,
    damping: float = 0.01
) -> np.ndarray:
    """
    Compute joint velocities with prioritized tasks.

    Priority:
        1. CoM control (primary)
        2. End-effector reaching (secondary, in null space of CoM)

    Args:
        current_angles: Current joint angles (rad)
        com_velocity_desired: Desired CoM velocity [vx, vy] (m/s)
        ee_velocity_desired: Desired end-effector velocity [vx, vy] (m/s)
        link_lengths: Link lengths (m)
        link_masses: Link masses (kg)
        damping: Damping factor

    Returns:
        joint_velocities: Joint velocities (rad/s)
    """
    # Primary task: CoM
    J_com = com_jacobian_planar(current_angles, link_lengths, link_masses)
    J_com_pinv = np.linalg.solve(
        J_com.T @ J_com + damping * np.eye(len(current_angles)),
        J_com.T
    )
    q_dot_primary = J_com_pinv @ com_velocity_desired

    # Null space projector
    N = np.eye(len(current_angles)) - J_com_pinv @ J_com

    # Secondary task: End-effector
    J_ee = end_effector_jacobian_planar(current_angles, link_lengths)
    J_ee_null = J_ee @ N  # Project EE Jacobian into CoM null space
    J_ee_null_pinv = np.linalg.solve(
        J_ee_null.T @ J_ee_null + damping * np.eye(len(current_angles)),
        J_ee_null.T
    )

    # Secondary contribution (only affects null space)
    q_dot_secondary = N @ J_ee_null_pinv @ (ee_velocity_desired - J_ee @ q_dot_primary)

    # Combined solution
    q_dot = q_dot_primary + q_dot_secondary

    return q_dot

# Test: Keep CoM fixed while moving end-effector
link_lengths = np.array([0.3, 0.4, 0.4, 0.1])
link_masses = np.array([20.0, 10.0, 5.0, 2.0])

q0 = np.array([0.0, -0.1, 0.2, -0.1])

dt = 0.05
duration = 3.0
times = np.arange(0, duration, dt)

q_history = np.zeros((len(times), len(q0)))
com_history = np.zeros((len(times), 2))
ee_history = np.zeros((len(times), 2))
q_history[0] = q0

# Desired velocities
com_vel_des = np.array([0.0, 0.0])  # Keep CoM fixed
ee_vel_des = np.array([0.05, 0.03])  # Move EE diagonally

for i in range(len(times) - 1):
    q_dot = prioritized_ik(
        q_history[i], com_vel_des, ee_vel_des,
        link_lengths, link_masses
    )

    q_history[i + 1] = q_history[i] + q_dot * dt

    # Record CoM and EE
    positions = forward_kinematics_chain(q_history[i], link_lengths)
    link_coms = (positions[:-1] + positions[1:]) / 2
    com_history[i] = compute_com(link_coms, link_masses)
    ee_history[i] = positions[-1]

# Final state
positions_final = forward_kinematics_chain(q_history[-1], link_lengths)
link_coms_final = (positions_final[:-1] + positions_final[1:]) / 2
com_history[-1] = compute_com(link_coms_final, link_masses)
ee_history[-1] = positions_final[-1]

# Visualize
fig, axes = plt.subplots(2, 2, figsize=(12, 10))

# Configuration snapshots
for ax, idx, title in zip([axes[0, 0], axes[0, 1]], [0, -1], ['Initial', 'Final']):
    positions = forward_kinematics_chain(q_history[idx], link_lengths)
    link_coms = (positions[:-1] + positions[1:]) / 2
    com = compute_com(link_coms, link_masses)

    ax.plot(positions[:, 0], positions[:, 1], 'o-', color='blue',
            markersize=8, linewidth=3, label='Robot')
    ax.scatter(com[0], com[1], s=300, c='red', marker='X',
               edgecolors='black', linewidths=2, label='CoM', zorder=10)
    ax.scatter(positions[-1, 0], positions[-1, 1], s=200, c='green',
               marker='*', edgecolors='black', linewidths=2, label='EE', zorder=10)

    ax.set_xlim(-0.2, 0.6)
    ax.set_ylim(-0.1, 1.4)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Z (m)')
    ax.set_title(title)
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')

# CoM trajectory (should be nearly stationary)
axes[1, 0].plot(com_history[:, 0], com_history[:, 1], 'r-', linewidth=2, marker='o', markersize=3)
axes[1, 0].scatter(com_history[0, 0], com_history[0, 1], s=200, c='green',
                   marker='o', edgecolors='black', linewidths=2, label='Start', zorder=10)
axes[1, 0].set_xlabel('CoM X (m)')
axes[1, 0].set_ylabel('CoM Z (m)')
axes[1, 0].set_title('CoM Trajectory (Should Stay Fixed)')
axes[1, 0].legend()
axes[1, 0].grid(True, alpha=0.3)
axes[1, 0].axis('equal')

# EE trajectory
axes[1, 1].plot(ee_history[:, 0], ee_history[:, 1], 'g-', linewidth=2, marker='o', markersize=3)
axes[1, 1].scatter(ee_history[0, 0], ee_history[0, 1], s=200, c='blue',
                   marker='o', edgecolors='black', linewidths=2, label='Start', zorder=10)
axes[1, 1].set_xlabel('EE X (m)')
axes[1, 1].set_ylabel('EE Z (m)')
axes[1, 1].set_title('End-Effector Trajectory')
axes[1, 1].legend()
axes[1, 1].grid(True, alpha=0.3)
axes[1, 1].axis('equal')

plt.tight_layout()
plt.show()

# Analyze
com_drift = np.linalg.norm(com_history[-1] - com_history[0])
ee_displacement = np.linalg.norm(ee_history[-1] - ee_history[0])

print(f"CoM drift: {com_drift * 100:.2f} cm (should be ~0)")
print(f"EE displacement: {ee_displacement * 100:.1f} cm")
print(f"EE velocity achieved: {ee_displacement / duration:.3f} m/s (desired: {np.linalg.norm(ee_vel_des):.3f} m/s)")
```

**Expected Output**:
- CoM stays nearly fixed (drift < 2cm)
- End-effector moves diagonally as commanded
- Joint configuration changes to accommodate both tasks

**What You Learned**:
- Prioritized IK satisfies primary task exactly, secondary task optimally
- Null-space projection allows secondary task without violating primary
- CoM control (balance) typically has highest priority
- Redundancy enables simultaneous task achievement

---

### Exercise 6.3.4: Joint Limit Avoidance

```python
"""Add joint limit avoidance as tertiary task."""
import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple

def joint_limit_cost_gradient(
    joint_angles: np.ndarray,
    joint_limits: np.ndarray,
    margin: float = 0.1
) -> np.ndarray:
    """
    Compute gradient of joint limit cost (repulsive potential).

    Args:
        joint_angles: Current joint angles (rad), shape (n,)
        joint_limits: Joint limits [min, max] (rad), shape (n, 2)
        margin: Activation margin from limits (rad)

    Returns:
        gradient: Cost gradient (pushes away from limits), shape (n,)
    """
    gradient = np.zeros_like(joint_angles)

    for i, (q, (q_min, q_max)) in enumerate(zip(joint_angles, joint_limits)):
        # Repulsion from lower limit
        if q - q_min < margin:
            gradient[i] -= 1.0 / (q - q_min + 1e-6)**2

        # Repulsion from upper limit
        if q_max - q < margin:
            gradient[i] += 1.0 / (q_max - q + 1e-6)**2

    return gradient

def prioritized_ik_with_limits(
    current_angles: np.ndarray,
    com_velocity_desired: np.ndarray,
    ee_velocity_desired: np.ndarray,
    link_lengths: np.ndarray,
    link_masses: np.ndarray,
    joint_limits: np.ndarray,
    alpha_limits: float = 0.05,
    damping: float = 0.01
) -> np.ndarray:
    """
    Prioritized IK with joint limit avoidance as tertiary task.

    Args:
        current_angles: Current joint angles (rad)
        com_velocity_desired: Desired CoM velocity
        ee_velocity_desired: Desired EE velocity
        link_lengths: Link lengths
        link_masses: Link masses
        joint_limits: Joint limits [min, max] per joint, shape (n, 2)
        alpha_limits: Gain for limit avoidance
        damping: Damping factor

    Returns:
        joint_velocities: Joint velocities (rad/s)
    """
    # Primary: CoM
    J_com = com_jacobian_planar(current_angles, link_lengths, link_masses)
    J_com_pinv = np.linalg.solve(
        J_com.T @ J_com + damping * np.eye(len(current_angles)),
        J_com.T
    )
    q_dot_primary = J_com_pinv @ com_velocity_desired
    N1 = np.eye(len(current_angles)) - J_com_pinv @ J_com

    # Secondary: EE
    J_ee = end_effector_jacobian_planar(current_angles, link_lengths)
    J_ee_null = J_ee @ N1
    J_ee_null_pinv = np.linalg.solve(
        J_ee_null.T @ J_ee_null + damping * np.eye(len(current_angles)),
        J_ee_null.T
    )
    q_dot_secondary = N1 @ J_ee_null_pinv @ (ee_velocity_desired - J_ee @ q_dot_primary)
    N2 = N1 @ (np.eye(len(current_angles)) - J_ee_null_pinv @ J_ee_null)

    # Tertiary: Joint limit avoidance (gradient descent in null space)
    limit_gradient = joint_limit_cost_gradient(current_angles, joint_limits)
    q_dot_tertiary = -alpha_limits * N2 @ limit_gradient

    return q_dot_primary + q_dot_secondary + q_dot_tertiary

# Test with restrictive joint limits
link_lengths = np.array([0.3, 0.4, 0.4, 0.1])
link_masses = np.array([20.0, 10.0, 5.0, 2.0])

# Joint limits (more restrictive than typical)
joint_limits = np.array([
    [-0.5, 0.5],   # Joint 1
    [-0.8, 0.2],   # Joint 2
    [-0.2, 1.0],   # Joint 3
    [-0.5, 0.1]    # Joint 4
])

q0 = np.array([0.0, -0.1, 0.2, -0.05])

dt = 0.05
duration = 4.0
times = np.arange(0, duration, dt)

q_history = np.zeros((len(times), len(q0)))
q_history[0] = q0

# Push EE in direction that would violate joint limits
com_vel_des = np.array([0.0, 0.0])
ee_vel_des = np.array([0.1, -0.05])

for i in range(len(times) - 1):
    q_dot = prioritized_ik_with_limits(
        q_history[i], com_vel_des, ee_vel_des,
        link_lengths, link_masses, joint_limits,
        alpha_limits=0.1
    )

    q_history[i + 1] = q_history[i] + q_dot * dt

# Visualize joint angles vs. limits
fig, axes = plt.subplots(2, 2, figsize=(12, 8))

for i, ax in enumerate(axes.flat):
    if i < len(q0):
        ax.plot(times, np.rad2deg(q_history[:, i]), 'b-', linewidth=2, label='Angle')
        ax.axhline(np.rad2deg(joint_limits[i, 0]), color='red', linestyle='--',
                   linewidth=2, label='Lower Limit')
        ax.axhline(np.rad2deg(joint_limits[i, 1]), color='red', linestyle='--',
                   linewidth=2, label='Upper Limit')

        # Check violations
        violations = (q_history[:, i] < joint_limits[i, 0]) | (q_history[:, i] > joint_limits[i, 1])
        if np.any(violations):
            ax.fill_between(times, -90, 90, where=violations, alpha=0.3,
                            color='red', label='Violation')

        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Angle (degrees)')
        ax.set_title(f'Joint {i+1}')
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.set_ylim(-90, 90)

plt.tight_layout()
plt.show()

# Check for violations
for i in range(len(q0)):
    min_angle = np.rad2deg(q_history[:, i].min())
    max_angle = np.rad2deg(q_history[:, i].max())
    limit_min = np.rad2deg(joint_limits[i, 0])
    limit_max = np.rad2deg(joint_limits[i, 1])

    violated = min_angle < limit_min or max_angle > limit_max

    print(f"Joint {i+1}: Range [{min_angle:.1f}°, {max_angle:.1f}°], " +
          f"Limits [{limit_min:.1f}°, {limit_max:.1f}°] - {'VIOLATED' if violated else 'OK'}")
```

**Expected Output**:
- Joint angles stay within specified limits
- Repulsive gradient pushes joints away as they approach limits
- EE motion may be slowed/modified to respect constraints

**What You Learned**:
- Tertiary tasks use null space of primary + secondary tasks
- Gradient descent in null space optimizes posture without affecting higher tasks
- Joint limit avoidance prevents hardware damage and singularities
- Multi-level prioritization handles complex constraint hierarchies

---

## TryWithAI Exercises

### TryWithAI 6.3.1: Reaching While Walking

**Prompt**:
```
Implement a whole-body planner for a humanoid that reaches for an object while walking. The system should:

1. Generate a walking gait (from Lesson 6.1) with CoM trajectory
2. Plan arm reaching motion to grasp target at specified time during gait
3. Use prioritized IK: (1) ZMP/CoM, (2) swing foot, (3) hand position
4. Ensure arm motion doesn't destabilize walking (check ZMP bounds)
5. Visualize the robot at key phases and show CoM/ZMP trajectories

Test with target at different heights and lateral positions. Show how reach timing affects feasibility (can't reach during single support if too far).
```

**Expected Skills**:
- Multi-phase motion planning
- Task synchronization (gait + manipulation)
- Dynamic feasibility checking
- 3D visualization of humanoid configuration

---

### TryWithAI 6.3.2: Self-Collision Avoidance

**Prompt**:
```
Add self-collision avoidance to whole-body IK. Given a simplified humanoid model with cylindrical link volumes:

1. Detect potential collisions between non-adjacent links
2. Compute collision cost gradient (distance-based repulsive potential)
3. Integrate collision avoidance as a low-priority task in the IK hierarchy
4. Visualize configuration space (2-joint subspace) showing collision regions

Test with motions that naturally cause collisions (e.g., touching knee to chest, crossing arms). Show how avoidance modifies trajectories to maintain clearance.
```

**Expected Skills**:
- Geometric collision detection
- Gradient-based collision avoidance
- Configuration space visualization
- Balancing task achievement with safety constraints

---

## Summary

This lesson covered whole-body motion planning for humanoid robots:

**Key Concepts**:
- Whole-body IK handles multiple simultaneous constraints
- Center-of-mass Jacobian enables balance control during manipulation
- Task prioritization resolves conflicting objectives
- Null-space projection exploits redundancy

**Practical Skills**:
- Computing CoM from link masses and positions
- Deriving and using CoM Jacobian
- Implementing prioritized IK (primary/secondary/tertiary tasks)
- Adding joint limit avoidance

**Next Steps**: [Lesson 6.4](lesson-04.md) introduces compliant control for safe physical interaction.

---

**Further Reading**:
- Sentis, L., & Khatib, O. (2005). "Synthesis of Whole-Body Behaviors through Hierarchical Control of Behavioral Primitives." *International Journal of Humanoid Robotics*.
- Kanoun, O., et al. (2011). "Kinematic Control of Redundant Manipulators: Generalizing the Task-Priority Framework to Inequality Task." *IEEE Transactions on Robotics*.
