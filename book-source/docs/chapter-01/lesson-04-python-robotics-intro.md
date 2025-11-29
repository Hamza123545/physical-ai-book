---
title: "Lesson 1.4: Python for Robotics Introduction"
description: "Master NumPy and Matplotlib for robot simulation and control loop implementation"
chapter: 1
lesson: 4
estimated_time: 60
cefr_level: "B1"
blooms_level: "Apply"
digcomp_level: 3
generated_by: "claude-sonnet-4-5-20250929"
source_spec: "specs/002-physical-ai-textbook/spec.md"
created: "2025-11-29"
last_modified: "2025-11-29"
git_author: "hswat"
workflow: "/sp.implement"
version: "1.0"
prerequisites:
  - "chapter-01-lesson-01"
  - "chapter-01-lesson-02"
  - "chapter-01-lesson-03"
has_interactive_python: true
interactive_python_count: 4
has_try_with_ai: true
try_with_ai_count: 2
tags: ["numpy", "matplotlib", "simulation", "control-loops"]
---

# Lesson 1.4: Python for Robotics Introduction

## 🎯 Learning Objectives

<LearningObjectives
  cefr_level="B1"
  objectives={[
    {
      text: "Use NumPy arrays for efficient numerical computation in robotics",
      blooms_level: "Apply",
      assessment_method: "Python exercises"
    },
    {
      text: "Visualize robot data using Matplotlib",
      blooms_level: "Apply",
      assessment_method: "Plotting exercise"
    },
    {
      text: "Implement a basic robot control loop with feedback",
      blooms_level: "Apply",
      assessment_method: "Control loop and simulation exercises"
    }
  ]}
/>

## 📚 Prerequisites

<Prerequisites
  prerequisites={[
    {
      lessonId: "chapter-01-lesson-01",
      title: "Lesson 1.1: What is Physical AI?",
      link: "/docs/chapter-01/lesson-01-what-is-physical-ai"
    },
    {
      lessonId: "chapter-01-lesson-02",
      title: "Lesson 1.2: Robot vs Software AI",
      link: "/docs/chapter-01/lesson-02-robot-vs-software-ai"
    },
    {
      lessonId: "chapter-01-lesson-03",
      title: "Lesson 1.3: Sensors and Actuators Overview",
      link: "/docs/chapter-01/lesson-03-sensors-actuators-overview"
    }
  ]}
/>

## Introduction

Python is the most popular language for robotics and AI. Two libraries are essential:

- **NumPy**: Fast numerical computation with arrays
- **Matplotlib**: Data visualization and plotting

In this lesson, you'll learn to use these tools to simulate and control robots. By the end, you'll build a complete robot simulation!

**Time**: 60 minutes

---

## 1. Why NumPy for Robotics?

### The Problem with Python Lists

```python
# Slow: Python lists
positions = [1.0, 2.0, 3.0]
velocities = [0.5, 1.0, 1.5]
result = [p + v for p, v in zip(positions, velocities)]
```

### The NumPy Solution

```python
import numpy as np
# Fast: NumPy arrays
positions = np.array([1.0, 2.0, 3.0])
velocities = np.array([0.5, 1.0, 1.5])
result = positions + velocities  # Vectorized!
```

**Why NumPy is faster**:
- Written in C (compiled, not interpreted)
- Vectorized operations (no Python loops)
- Optimized for numerical computation

### Key NumPy Operations for Robotics

```python
# Creating arrays
pos = np.array([1.0, 2.0, 3.0])
zeros = np.zeros(10)
ones = np.ones(5)
sequence = np.arange(0, 10, 0.1)  # 0 to 10, step 0.1

# Math operations
angles = np.array([0, 30, 60, 90])
radians = np.deg2rad(angles)
sines = np.sin(radians)

# Statistics
mean = np.mean(data)
std = np.std(data)
minimum = np.min(data)
```

---

## 2. Matplotlib for Visualization

Robots generate lots of data. Visualization helps us understand it.

### Basic Plotting

```python
import matplotlib.pyplot as plt

time = np.linspace(0, 10, 100)
position = np.sin(time)

plt.plot(time, position)
plt.xlabel('Time (s)')
plt.ylabel('Position (m)')
plt.title('Robot Position over Time')
plt.grid(True)
plt.show()
```

### Multiple Plots

```python
plt.figure(figsize=(12, 4))

plt.subplot(1, 2, 1)  # 1 row, 2 cols, plot 1
plt.plot(time, position)
plt.title('Position')

plt.subplot(1, 2, 2)  # Plot 2
plt.plot(time, velocity)
plt.title('Velocity')

plt.tight_layout()
plt.show()
```

---

## 3. Control Loops

Robots use **feedback control loops**:

1. Read sensor
2. Calculate error (desired - actual)
3. Compute control command
4. Send to actuator
5. Repeat

### PID Control (Preview)

The most common controller:
- **P** (Proportional): Response proportional to error
- **I** (Integral): Corrects persistent errors
- **D** (Derivative): Dampens oscillations

---

## 4. Exercises

### Exercise 1.4.1: NumPy Array Operations

Master NumPy basics for robot computation.

<InteractivePython
  id="ex-1-4-1"
  title="NumPy Arrays for Robot State"
  starterCode={`import numpy as np

def robot_state_update(positions, velocities, dt=0.1):
    """
    Update robot joint positions using velocities.

    Args:
        positions: Current joint angles (array)
        velocities: Joint velocities (rad/s)
        dt: Time step (seconds)

    Returns:
        New positions after dt seconds
    """
    # TODO: Implement position update
    # new_position = old_position + velocity * dt
    pass

# Test with 3-joint robot
pos = np.array([0.0, 0.5, 1.0])  # radians
vel = np.array([0.1, -0.2, 0.3])  # rad/s

new_pos = robot_state_update(pos, vel, dt=0.1)
print(f"Old positions: {pos}")
print(f"Velocities: {vel}")
print(f"New positions: {new_pos}")

# Convert to degrees
print(f"New positions (deg): {np.rad2deg(new_pos)}")
`}
  hints={[
    "Use NumPy array addition: new_pos = positions + velocities * dt",
    "np.rad2deg() converts radians to degrees",
    "No loops needed - NumPy handles arrays element-wise"
  ]}
/>

---

### Exercise 1.4.2: Plotting Robot Trajectory

Visualize a robot's path over time.

<InteractivePython
  id="ex-1-4-2"
  title="Plot Robot Trajectory"
  starterCode={`import numpy as np
import matplotlib.pyplot as plt

def simulate_trajectory(initial_pos, velocity, duration=10, dt=0.1):
    """
    Simulate 1D robot motion and plot trajectory.

    Args:
        initial_pos: Starting position (m)
        velocity: Constant velocity (m/s)
        duration: Simulation time (s)
        dt: Time step (s)

    Returns:
        (time_array, position_array)
    """
    # TODO:
    # 1. Create time array: np.arange(0, duration, dt)
    # 2. Calculate positions: pos = initial_pos + velocity * time
    # 3. Return (time, positions)
    pass

# Simulate
time, pos = simulate_trajectory(initial_pos=0, velocity=1.5, duration=10)

# Plot
plt.figure(figsize=(10, 4))
plt.plot(time, pos, 'b-', linewidth=2, label='Robot position')
plt.xlabel('Time (s)')
plt.ylabel('Position (m)')
plt.title('Robot 1D Motion')
plt.grid(True)
plt.legend()
plt.show()

print(f"Final position: {pos[-1]:.2f} m")
print(f"Distance traveled: {pos[-1] - pos[0]:.2f} m")
`}
  hints={[
    "time = np.arange(0, duration, dt)",
    "positions = initial_pos + velocity * time",
    "Return both arrays as tuple"
  ]}
/>

---

### Exercise 1.4.3: Simple Control Loop

Implement proportional control to reach a target.

<InteractivePython
  id="ex-1-4-3"
  title="Proportional Controller"
  starterCode={`import numpy as np
import matplotlib.pyplot as plt

def proportional_control(target, initial_pos=0, kp=0.5, steps=100):
    """
    Simulate proportional controller reaching target.

    Args:
        target: Desired position
        initial_pos: Starting position
        kp: Proportional gain (0-1)
        steps: Number of control iterations

    Returns:
        Array of positions over time
    """
    positions = np.zeros(steps)
    positions[0] = initial_pos

    # TODO: Implement control loop
    # for i in range(1, steps):
    #     error = target - positions[i-1]
    #     control = kp * error
    #     positions[i] = positions[i-1] + control

    pass

    return positions

# Test
target = 10.0
positions = proportional_control(target, initial_pos=0, kp=0.3, steps=50)

# Plot
plt.figure(figsize=(10, 4))
plt.plot(positions, 'b-', label='Actual position')
plt.axhline(y=target, color='r', linestyle='--', label='Target')
plt.xlabel('Control Step')
plt.ylabel('Position')
plt.title('Proportional Controller Response')
plt.legend()
plt.grid(True)
plt.show()

print(f"Final position: {positions[-1]:.2f}")
print(f"Final error: {abs(target - positions[-1]):.3f}")
`}
  hints={[
    "error = target - current_position",
    "control_output = kp * error",
    "next_position = current_position + control_output",
    "Store each position in the array"
  ]}
/>

---

### Exercise 1.4.4: 2D Robot Simulation

Bring it all together: simulate a robot moving in 2D space.

<InteractivePython
  id="ex-1-4-4"
  title="2D Robot Motion Simulation"
  starterCode={`import numpy as np
import matplotlib.pyplot as plt

def simulate_robot_2d(start_pos, velocity, duration=5, dt=0.1):
    """
    Simulate robot moving in 2D plane.

    Args:
        start_pos: [x, y] starting position
        velocity: [vx, vy] velocity vector
        duration: Simulation time
        dt: Time step

    Returns:
        (x_positions, y_positions)
    """
    # TODO:
    # 1. Calculate number of steps
    # 2. Create arrays for x and y positions
    # 3. Loop and update: pos = pos + velocity * dt
    # 4. Return x and y arrays
    pass

# Test: Robot moves diagonally
start = np.array([0.0, 0.0])
vel = np.array([1.0, 0.5])  # Moving right and up

x_pos, y_pos = simulate_robot_2d(start, vel, duration=10)

# Plot trajectory
plt.figure(figsize=(8, 8))
plt.plot(x_pos, y_pos, 'b-', linewidth=2, label='Robot path')
plt.plot(x_pos[0], y_pos[0], 'go', markersize=10, label='Start')
plt.plot(x_pos[-1], y_pos[-1], 'ro', markersize=10, label='End')
plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')
plt.title('2D Robot Trajectory')
plt.grid(True)
plt.legend()
plt.axis('equal')
plt.show()

print(f"Start: ({x_pos[0]:.1f}, {y_pos[0]:.1f})")
print(f"End: ({x_pos[-1]:.1f}, {y_pos[-1]:.1f})")
print(f"Distance: {np.sqrt((x_pos[-1]-x_pos[0])**2 + (y_pos[-1]-y_pos[0])**2):.2f} m")
`}
  hints={[
    "steps = int(duration / dt)",
    "x = np.zeros(steps), y = np.zeros(steps)",
    "In loop: x[i] = x[i-1] + velocity[0] * dt",
    "Similarly for y with velocity[1]"
  ]}
/>

---

## 5. Try With AI

### TryWithAI 1.4.1: NumPy Vectorization

<TryWithAI
  id="tryai-1-4-1"
  title="Optimize Code with NumPy Vectorization"
  role="Copilot"
  scenario="You have slow robot code using Python loops. You want to speed it up with NumPy vectorization."
  yourTask="Complete Exercise 1.4.1, then write a version using a Python for loop instead of NumPy array operations."
  aiPromptTemplate="I have this robot update code using loops: [paste your loop version]. Can you help me convert it to use NumPy vectorization? Explain why the NumPy version is faster."
  successCriteria={[
    "You understand the difference between loops and vectorization",
    "You can convert loop-based code to NumPy operations",
    "You know when vectorization provides benefits"
  ]}
  reflectionQuestions={[
    "How much faster is vectorized code for 1000 joints?",
    "Are there cases where loops are better than vectorization?",
    "What operations can't be vectorized?"
  ]}
/>

---

### TryWithAI 1.4.2: Control Loop Debugging

<TryWithAI
  id="tryai-1-4-2"
  title="Debug Control Loop Oscillations"
  role="Evaluator"
  scenario="Your proportional controller (Exercise 1.4.3) oscillates around the target instead of settling smoothly."
  yourTask="Run Exercise 1.4.3 with kp=0.9 (high gain). Observe the oscillations. Think about why this happens."
  aiPromptTemplate="My proportional controller oscillates with kp={kp_value}. Here's my code: [paste]. Can you review it and explain why high gain causes oscillations? Suggest how to fix it (hint: PID control)."
  successCriteria={[
    "You understand how gain affects controller behavior",
    "You can identify oscillation in plots",
    "You know that D (derivative) term reduces oscillations"
  ]}
  reflectionQuestions={[
    "What happens with very low gain (kp=0.1)?",
    "What happens with very high gain (kp=1.5)?",
    "How would you choose the right gain value?"
  ]}
/>

---

## Summary

**Key Takeaways**:

1. **NumPy for Robotics**:
   - Arrays are faster than lists
   - Vectorized operations (no loops)
   - Essential functions: `np.array`, `np.zeros`, `np.sin`, `np.mean`

2. **Matplotlib for Visualization**:
   - Plot trajectories and sensor data
   - Multiple subplots for comparison
   - Helps debug and understand robot behavior

3. **Control Loops**:
   - Read sensor → Calculate error → Command actuator → Repeat
   - Proportional control: output = kp * error
   - Higher gain = faster response but more oscillation

4. **Practical Skills**:
   - Array operations for robot states
   - Trajectory visualization
   - Implementing basic controllers
   - 2D motion simulation

**🎉 Chapter 1 Complete!** You now have the foundational knowledge of Physical AI:
- What makes Physical AI different
- Real-time and safety requirements
- Common sensors and actuators
- Python tools for robotics

**What's Next**: Take the [Chapter 1 Quiz](./quiz.md) to test your understanding, then move on to **Chapter 2: Robot Kinematics and Dynamics** to learn how to program robot arms!

---

**Estimated completion time**: 60 minutes | **Prerequisites**: Lessons 1.1, 1.2, 1.3 | **Difficulty**: B1 (Intermediate)
