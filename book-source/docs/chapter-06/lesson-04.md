---
title: "Lesson 6.4: Compliant Control"
description: "Impedance control, force regulation, collision safety"
chapter: 6
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
prerequisites: ["chapter-05-lesson-04", "chapter-06-lesson-03"]
has_interactive_python: false
interactive_python_count: 0
has_try_with_ai: true
try_with_ai_count: 2
tags: ["manipulation", "grasping", "compliant-control", "impedance"]
---

# Lesson 6.4: Manipulation and Grasping

**Prerequisites**: [Chapter 5: Dynamics and Control](../chapter-05/index.md), [Lesson 6.3: Balance Control](lesson-03.md)

---

## Introduction

Humanoid robots must physically interact with humans and environments safely. Rigid position control can cause damage during unexpected contact. Compliant control regulates interaction forces while maintaining position accuracy, enabling safe handshakes, collaborative lifting, and collision response. This lesson covers impedance control, admittance control, and force regulation strategies.

**Learning Objectives**:
- Understand impedance vs. admittance control paradigms
- Implement virtual mass-spring-damper systems
- Design force controllers for contact tasks
- Apply collision detection and reaction strategies

---

## 1. Impedance Control

**Impedance control** regulates the dynamic relationship between position error and force:

$$
\mathbf{F} = \mathbf{M}_d (\ddot{\mathbf{x}}_d - \ddot{\mathbf{x}}) + \mathbf{D}_d (\dot{\mathbf{x}}_d - \dot{\mathbf{x}}) + \mathbf{K}_d (\mathbf{x}_d - \mathbf{x})
$$

Where:
- $\mathbf{M}_d$: Desired inertia matrix (virtual mass)
- $\mathbf{D}_d$: Desired damping matrix
- $\mathbf{K}_d$: Desired stiffness matrix
- $\mathbf{x}, \mathbf{x}_d$: Actual and desired end-effector positions

**Physical Interpretation**: Robot behaves like a mass-spring-damper attached to the target position.

**Tuning Guidelines**:
- **High stiffness $K_d$**: Accurate tracking, but rigid (low compliance)
- **Low stiffness**: Compliant (safe), but poor tracking
- **Critical damping**: $D_d = 2\sqrt{M_d K_d}$ (no oscillations)

---

## 2. Admittance Control

**Admittance control** computes desired position from measured force:

$$
\mathbf{M}_d \ddot{\mathbf{x}}_d + \mathbf{D}_d \dot{\mathbf{x}}_d + \mathbf{K}_d \mathbf{x}_d = \mathbf{F}_{\text{ext}}
$$

Where $\mathbf{F}_{\text{ext}}$ is the external force (measured by force/torque sensor).

**Difference from Impedance**:
- **Impedance**: Position controlled, force is outcome
- **Admittance**: Force controlled, position is outcome
- Impedance better for stiff robots; admittance for backdrivable robots

---

## 3. Force Control

For tasks requiring specific contact forces (e.g., polishing, pushing button):

**Hybrid Position/Force Control**: Control position in some directions, force in others.

Example: Wiping surface (vertical)
- $x$, $y$: Position control (trajectory)
- $z$: Force control (constant normal force)

**PI Force Controller**:

$$
\dot{x}_z = K_p (F_d - F_z) + K_i \int (F_d - F_z) dt
$$

---

## Interactive Exercises

### Exercise 6.4.1: 1D Impedance Control Simulation

```python
"""Simulate impedance control for 1D point mass."""
import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple

def impedance_control_1d(
    x: float,
    x_dot: float,
    x_desired: float,
    x_dot_desired: float,
    M_d: float,
    D_d: float,
    K_d: float,
    F_ext: float = 0.0
) -> float:
    """
    Compute control force using impedance control.

    Args:
        x: Current position (m)
        x_dot: Current velocity (m/s)
        x_desired: Desired position (m)
        x_dot_desired: Desired velocity (m/s)
        M_d: Desired mass (kg)
        D_d: Desired damping (Ns/m)
        K_d: Desired stiffness (N/m)
        F_ext: External force (N)

    Returns:
        F_control: Control force (N)
    """
    # Impedance law (assumes x_ddot_desired = 0 for simplicity)
    F_control = (
        -M_d * 0.0 +  # No desired acceleration
        D_d * (x_dot_desired - x_dot) +
        K_d * (x_desired - x) +
        F_ext  # Feedforward external force compensation
    )

    return F_control

def simulate_impedance(
    mass_robot: float,
    stiffness: float,
    damping: float,
    desired_pos: float,
    external_force: float,
    duration: float = 5.0,
    dt: float = 0.01
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Simulate 1D impedance control with optional external force.

    Args:
        mass_robot: Actual robot mass (kg)
        stiffness: Desired stiffness K_d (N/m)
        damping: Desired damping D_d (Ns/m)
        desired_pos: Target position (m)
        external_force: Constant external force (N)
        duration: Simulation time (s)
        dt: Time step (s)

    Returns:
        times: Time array
        positions: Position history
        forces: Control force history
    """
    times = np.arange(0, duration, dt)
    positions = np.zeros(len(times))
    velocities = np.zeros(len(times))
    forces = np.zeros(len(times))

    # Initial state: at origin, zero velocity
    x, x_dot = 0.0, 0.0

    # Desired mass (for impedance)
    M_d = mass_robot

    for i, t in enumerate(times):
        # Apply external force after t=2s
        F_ext = external_force if t > 2.0 else 0.0

        # Impedance control
        F_control = impedance_control_1d(
            x, x_dot, desired_pos, 0.0, M_d, damping, stiffness, F_ext=0.0
        )

        # Robot dynamics: m * x_ddot = F_control + F_ext
        x_ddot = (F_control + F_ext) / mass_robot

        # Integrate (Euler)
        x_dot = x_dot + x_ddot * dt
        x = x + x_dot * dt

        positions[i] = x
        velocities[i] = x_dot
        forces[i] = F_control

    return times, positions, forces

# Test different stiffness values
mass_robot = 5.0  # kg
desired_pos = 1.0  # m
external_force = -10.0  # N (pushing back)

stiffness_values = [50, 200, 1000]  # N/m
colors = ['blue', 'orange', 'red']

fig, axes = plt.subplots(2, 1, figsize=(10, 8))

for K_d, color in zip(stiffness_values, colors):
    D_d = 2 * np.sqrt(mass_robot * K_d)  # Critical damping

    times, positions, forces = simulate_impedance(
        mass_robot, K_d, D_d, desired_pos, external_force
    )

    # Position
    axes[0].plot(times, positions, color=color, linewidth=2, label=f'K={K_d} N/m')

    # Force
    axes[1].plot(times, forces, color=color, linewidth=2, label=f'K={K_d} N/m')

# Format
axes[0].axhline(desired_pos, color='black', linestyle='--', alpha=0.5, label='Desired')
axes[0].axvline(2.0, color='gray', linestyle=':', alpha=0.5, label='Force Applied')
axes[0].set_ylabel('Position (m)')
axes[0].set_title('Impedance Control: Position Response')
axes[0].legend()
axes[0].grid(True, alpha=0.3)

axes[1].axvline(2.0, color='gray', linestyle=':', alpha=0.5)
axes[1].set_xlabel('Time (s)')
axes[1].set_ylabel('Control Force (N)')
axes[1].set_title('Impedance Control: Control Force')
axes[1].legend()
axes[1].grid(True, alpha=0.3)

plt.tight_layout()
plt.show()

# Analyze steady-state displacement
for K_d in stiffness_values:
    displacement = external_force / K_d
    print(f"Stiffness {K_d} N/m: Steady-state displacement = {abs(displacement)*100:.2f} cm")
```

**Expected Output**:
- Low stiffness (50 N/m): Large displacement under force (~20cm)
- High stiffness (1000 N/m): Small displacement (~1cm)
- All critically damped (no overshoot)

**What You Learned**:
- Impedance control creates virtual spring between robot and target
- Lower stiffness → more compliant (safer, but less accurate)
- External forces cause steady-state displacement $\Delta x = F/K$
- Critical damping eliminates oscillations

---

### Exercise 6.4.2: Collision Detection and Response

```python
"""Detect collision from acceleration and trigger compliant response."""
import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple

def detect_collision(
    acceleration: float,
    threshold: float = 5.0
) -> bool:
    """
    Detect collision from abnormal acceleration.

    Args:
        acceleration: Current acceleration (m/s²)
        threshold: Collision threshold (m/s²)

    Returns:
        collision_detected: True if collision detected
    """
    return abs(acceleration) > threshold

def simulate_collision_response(
    collision_time: float = 2.0,
    collision_force: float = 50.0,
    duration: float = 5.0,
    dt: float = 0.01
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Simulate robot with collision detection and compliant response.

    Args:
        collision_time: Time of collision (s)
        collision_force: Impact force (N)
        duration: Simulation duration (s)
        dt: Time step (s)

    Returns:
        times, positions, forces, mode (0=normal, 1=collision response)
    """
    times = np.arange(0, duration, dt)
    positions = np.zeros(len(times))
    velocities = np.zeros(len(times))
    forces = np.zeros(len(times))
    modes = np.zeros(len(times))  # 0=normal, 1=compliant

    # Robot parameters
    mass = 5.0  # kg
    desired_pos = 1.5  # m

    # Normal mode: high stiffness
    K_normal = 1000.0  # N/m
    D_normal = 2 * np.sqrt(mass * K_normal)

    # Collision response: low stiffness
    K_compliant = 50.0  # N/m
    D_compliant = 2 * np.sqrt(mass * K_compliant)

    x, x_dot = 0.0, 0.0
    collision_detected = False

    for i, t in enumerate(times):
        # Apply collision force
        F_ext = collision_force if abs(t - collision_time) < 0.1 else 0.0

        # Select impedance parameters
        if collision_detected:
            K_d, D_d = K_compliant, D_compliant
            modes[i] = 1
        else:
            K_d, D_d = K_normal, D_normal
            modes[i] = 0

        # Impedance control
        F_control = impedance_control_1d(x, x_dot, desired_pos, 0.0, mass, D_d, K_d)

        # Dynamics
        x_ddot = (F_control + F_ext) / mass

        # Collision detection
        if not collision_detected and detect_collision(x_ddot, threshold=5.0):
            collision_detected = True
            print(f"Collision detected at t={t:.2f}s, acceleration={x_ddot:.1f} m/s²")

        # Integrate
        x_dot = x_dot + x_ddot * dt
        x = x + x_dot * dt

        positions[i] = x
        forces[i] = F_control

    return times, positions, forces, modes

# Simulate collision scenario
times, positions, forces, modes = simulate_collision_response(
    collision_time=2.0,
    collision_force=50.0
)

# Visualize
fig, axes = plt.subplots(3, 1, figsize=(10, 10))

# Position
axes[0].plot(times, positions, 'b-', linewidth=2, label='Position')
axes[0].axhline(1.5, color='black', linestyle='--', alpha=0.5, label='Target')
axes[0].fill_between(times, 0, 2, where=(modes == 1), alpha=0.2,
                      color='red', label='Compliant Mode')
axes[0].set_ylabel('Position (m)')
axes[0].set_title('Collision Response: Position')
axes[0].legend()
axes[0].grid(True, alpha=0.3)

# Force
axes[1].plot(times, forces, 'g-', linewidth=2, label='Control Force')
axes[1].fill_between(times, -100, 100, where=(modes == 1), alpha=0.2, color='red')
axes[1].set_ylabel('Force (N)')
axes[1].set_title('Collision Response: Control Force')
axes[1].legend()
axes[1].grid(True, alpha=0.3)

# Mode
axes[2].fill_between(times, 0, 1, where=(modes == 0), alpha=0.5,
                      color='green', label='Normal (Stiff)')
axes[2].fill_between(times, 0, 1, where=(modes == 1), alpha=0.5,
                      color='red', label='Collision Response (Compliant)')
axes[2].set_xlabel('Time (s)')
axes[2].set_ylabel('Control Mode')
axes[2].set_title('Control Mode State')
axes[2].set_ylim(-0.1, 1.1)
axes[2].legend()
axes[2].grid(True, alpha=0.3)

plt.tight_layout()
plt.show()

print(f"Max position displacement: {positions.max():.3f} m")
print(f"Compliant mode duration: {np.sum(modes) * 0.01:.2f} s")
```

**Expected Output**:
- High acceleration spike triggers collision detection
- Robot switches to low stiffness (compliant mode)
- Position deviates significantly during collision (safe absorption)
- Control force drops in compliant mode

**What You Learned**:
- Collision detection via acceleration monitoring
- Mode switching reduces impact forces (safety)
- Compliant mode allows robot to "give" during collision
- Trade-off: safety vs. task performance during collision

---

### Exercise 6.4.3: Force Control (Contact Task)

```python
"""Implement force control for maintaining contact force."""
import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple

def pi_force_controller(
    F_measured: float,
    F_desired: float,
    error_integral: float,
    Kp: float = 0.01,
    Ki: float = 0.005
) -> Tuple[float, float]:
    """
    PI controller for force tracking.

    Args:
        F_measured: Current measured force (N)
        F_desired: Desired force (N)
        error_integral: Accumulated error
        Kp: Proportional gain
        Ki: Integral gain

    Returns:
        velocity_command: Commanded velocity (m/s)
        updated_integral: Updated error integral
    """
    error = F_desired - F_measured
    velocity_command = Kp * error + Ki * error_integral

    return velocity_command, error_integral + error

def simulate_surface_contact(
    desired_force: float = 10.0,
    surface_stiffness: float = 1000.0,
    duration: float = 5.0,
    dt: float = 0.01
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Simulate force-controlled contact with surface.

    Args:
        desired_force: Target contact force (N)
        surface_stiffness: Surface spring constant (N/m)
        duration: Simulation time (s)
        dt: Time step (s)

    Returns:
        times, positions, forces
    """
    times = np.arange(0, duration, dt)
    positions = np.zeros(len(times))
    forces = np.zeros(len(times))

    # Robot state
    x = 0.0  # Position (0 = surface)
    x_dot = 0.0
    error_integral = 0.0

    # Surface contact at x > 0 (compression)
    surface_position = 0.0

    for i, t in enumerate(times):
        # Measure force (spring contact model)
        if x > surface_position:
            F_measured = surface_stiffness * (x - surface_position)
        else:
            F_measured = 0.0

        # PI force control
        x_dot_cmd, error_integral = pi_force_controller(
            F_measured, desired_force, error_integral, Kp=0.02, Ki=0.01
        )

        # Simple velocity control (assume perfect tracking)
        x_dot = x_dot_cmd

        # Integrate
        x = x + x_dot * dt

        # Ensure no penetration below surface
        x = max(x, surface_position)

        positions[i] = x
        forces[i] = F_measured

    return times, positions, forces

# Test different desired forces
desired_forces = [5, 10, 20]  # N
colors = ['blue', 'green', 'red']
surface_stiffness = 1000.0  # N/m

fig, axes = plt.subplots(2, 1, figsize=(10, 8))

for F_d, color in zip(desired_forces, colors):
    times, positions, forces = simulate_surface_contact(
        desired_force=F_d,
        surface_stiffness=surface_stiffness
    )

    # Position
    axes[0].plot(times, positions * 1000, color=color, linewidth=2,
                 label=f'F_desired = {F_d} N')

    # Force
    axes[1].plot(times, forces, color=color, linewidth=2,
                 label=f'F_desired = {F_d} N')

# Format
axes[0].set_ylabel('Indentation (mm)')
axes[0].set_title('Force Control: Surface Indentation')
axes[0].legend()
axes[0].grid(True, alpha=0.3)

for F_d, color in zip(desired_forces, colors):
    axes[1].axhline(F_d, color=color, linestyle='--', alpha=0.5)

axes[1].set_xlabel('Time (s)')
axes[1].set_ylabel('Contact Force (N)')
axes[1].set_title('Force Control: Force Tracking')
axes[1].legend()
axes[1].grid(True, alpha=0.3)

plt.tight_layout()
plt.show()

# Analyze steady-state
for F_d in desired_forces:
    times, positions, forces = simulate_surface_contact(F_d, surface_stiffness)
    steady_force = np.mean(forces[-100:])  # Last 1 second
    steady_position = np.mean(positions[-100:]) * 1000  # mm
    error = abs(steady_force - F_d)

    print(f"Desired: {F_d} N → Achieved: {steady_force:.2f} N " +
          f"(error: {error:.2f} N), Indent: {steady_position:.2f} mm")
```

**Expected Output**:
- Force converges to desired value
- Indentation depth = F_desired / K_surface
- PI controller achieves zero steady-state error

**What You Learned**:
- Force control regulates contact force via position adjustment
- PI control eliminates steady-state force error
- Indentation depth depends on surface stiffness
- Applications: polishing, assembly, tactile exploration

---

### Exercise 6.4.4: Variable Stiffness Control

```python
"""Implement variable stiffness based on task phase."""
import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple

def variable_stiffness_profile(
    time: float,
    phases: list
) -> Tuple[float, float]:
    """
    Get desired stiffness and damping for current time.

    Args:
        time: Current time (s)
        phases: List of (t_end, K, D) tuples

    Returns:
        K_d, D_d: Desired stiffness and damping
    """
    for t_end, K, D in phases:
        if time <= t_end:
            return K, D
    return phases[-1][1], phases[-1][2]  # Return last phase

def simulate_variable_stiffness(
    trajectory: np.ndarray,
    times: np.ndarray,
    stiffness_profile: list
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Simulate impedance control with time-varying stiffness.

    Args:
        trajectory: Desired position trajectory (m)
        times: Time array (s)
        stiffness_profile: List of (t_end, K, D)

    Returns:
        positions, forces, stiffnesses
    """
    dt = times[1] - times[0]
    positions = np.zeros(len(times))
    forces = np.zeros(len(times))
    stiffnesses = np.zeros(len(times))

    mass = 5.0
    x, x_dot = 0.0, 0.0

    for i, t in enumerate(times):
        K_d, D_d = variable_stiffness_profile(t, stiffness_profile)
        stiffnesses[i] = K_d

        x_desired = trajectory[i]

        # Impedance control
        F_control = impedance_control_1d(x, x_dot, x_desired, 0.0, mass, D_d, K_d)

        # Dynamics
        x_ddot = F_control / mass
        x_dot = x_dot + x_ddot * dt
        x = x + x_dot * dt

        positions[i] = x
        forces[i] = F_control

    return positions, forces, stiffnesses

# Define task with three phases
duration = 6.0
dt = 0.01
times = np.arange(0, duration, dt)

# Trajectory: move to 1m, hold, return
trajectory = np.zeros(len(times))
for i, t in enumerate(times):
    if t < 2.0:
        trajectory[i] = 0.5 * t  # Move to 1m
    elif t < 4.0:
        trajectory[i] = 1.0  # Hold
    else:
        trajectory[i] = 1.0 - 0.5 * (t - 4.0)  # Return

# Stiffness profile
# Phase 1 (0-2s): Low stiffness (free motion)
# Phase 2 (2-4s): High stiffness (precise hold, e.g., assembly)
# Phase 3 (4-6s): Low stiffness (safe return)
stiffness_profile = [
    (2.0, 100.0, 2*np.sqrt(5.0*100.0)),
    (4.0, 1000.0, 2*np.sqrt(5.0*1000.0)),
    (6.0, 100.0, 2*np.sqrt(5.0*100.0))
]

positions, forces, stiffnesses = simulate_variable_stiffness(
    trajectory, times, stiffness_profile
)

# Visualize
fig, axes = plt.subplots(3, 1, figsize=(10, 10))

# Trajectory
axes[0].plot(times, trajectory, 'k--', linewidth=2, label='Desired', alpha=0.7)
axes[0].plot(times, positions, 'b-', linewidth=2, label='Actual')
axes[0].set_ylabel('Position (m)')
axes[0].set_title('Variable Stiffness Control: Position')
axes[0].legend()
axes[0].grid(True, alpha=0.3)

# Force
axes[1].plot(times, forces, 'g-', linewidth=2)
axes[1].set_ylabel('Control Force (N)')
axes[1].set_title('Control Force')
axes[1].grid(True, alpha=0.3)

# Stiffness
axes[2].plot(times, stiffnesses, 'r-', linewidth=3)
axes[2].fill_between(times, 0, 1200, where=(stiffnesses == 100), alpha=0.2,
                      color='blue', label='Low Stiffness (Compliant)')
axes[2].fill_between(times, 0, 1200, where=(stiffnesses == 1000), alpha=0.2,
                      color='red', label='High Stiffness (Precise)')
axes[2].set_xlabel('Time (s)')
axes[2].set_ylabel('Stiffness (N/m)')
axes[2].set_title('Impedance Stiffness Profile')
axes[2].legend()
axes[2].grid(True, alpha=0.3)

plt.tight_layout()
plt.show()

# Analyze tracking error by phase
phases_times = [(0, 2), (2, 4), (4, 6)]
phase_names = ['Approach', 'Hold', 'Return']

for (t_start, t_end), name in zip(phases_times, phase_names):
    mask = (times >= t_start) & (times < t_end)
    error_rms = np.sqrt(np.mean((positions[mask] - trajectory[mask])**2))
    print(f"{name} phase: RMS tracking error = {error_rms*1000:.2f} mm")
```

**Expected Output**:
- Low stiffness during approach/return (smooth, safe motion)
- High stiffness during hold (accurate positioning)
- Tracking error lowest during high-stiffness phase

**What You Learned**:
- Stiffness can be varied based on task requirements
- High stiffness for precision tasks (assembly, holding)
- Low stiffness for safety during motion and human proximity
- Variable impedance enables adaptive robot behavior

---

## TryWithAI Exercises

### TryWithAI 6.4.1: Adaptive Impedance from Force Feedback

**Prompt**:
```
Design an adaptive impedance controller that automatically adjusts stiffness based on measured interaction forces. The system should:

1. Monitor contact force magnitude and rate of change
2. Decrease stiffness when high forces or rapid force changes detected (safety)
3. Increase stiffness when in free motion for better tracking
4. Use an exponential filter to smooth stiffness transitions

Test scenarios: sudden obstacle contact, human hand-guiding, and collaborative pushing. Show how stiffness adapts in real-time and compare safety (peak forces) vs. fixed high/low stiffness.
```

**Expected Skills**:
- Real-time parameter adaptation
- Force-based state estimation (contact vs. free motion)
- Exponential smoothing filters
- Safety metric analysis (peak force, acceleration)

---

### TryWithAI 6.4.2: Hybrid Position-Force Control for Assembly

**Prompt**:
```
Implement hybrid position/force control for a peg-in-hole insertion task. The controller should:

1. Control X-Y position (align peg with hole opening)
2. Control Z force (constant insertion force, e.g., 5N)
3. Detect successful insertion (force drops when peg enters hole)
4. Visualize peg position, hole location, contact forces, and success detection

Model the hole as a region where the peg can slide freely (low friction) once aligned. Test with different hole tolerances (tight: ±0.5mm, loose: ±2mm). Show how force control compensates for position uncertainty.
```

**Expected Skills**:
- Task-space decomposition (position vs. force directions)
- Contact state detection
- Geometry-based constraint modeling
- Success/failure analysis

---

## Summary

This lesson introduced compliant control for safe human-robot interaction:

**Key Concepts**:
- Impedance control regulates position-force relationship
- Virtual stiffness/damping enable tunable compliance
- Collision detection triggers safe response modes
- Force control maintains desired contact forces

**Practical Skills**:
- Implementing 1D impedance control
- Collision detection and response
- PI force control for contact tasks
- Variable stiffness control for multi-phase tasks

**Next Steps**: [Lesson 6.5](lesson-05.md) covers human-robot interaction including gesture recognition and social cues.

---

**Further Reading**:
- Hogan, N. (1985). "Impedance Control: An Approach to Manipulation." *Journal of Dynamic Systems, Measurement, and Control*.
- Albu-Schäffer, A., et al. (2007). "Cartesian Impedance Control of Redundant and Flexible-Joint Robots." *Springer Tracts in Advanced Robotics*.
