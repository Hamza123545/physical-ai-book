---
title: "Lesson 3.3: Physics Simulation in Gazebo"
description: "Master physics simulation: gravity, collisions, and rigid body dynamics"
chapter: 3
lesson: 3
estimated_time: 60
cefr_level: "B1+"
blooms_level: "Apply"
digcomp_level: 4
generated_by: "claude-sonnet-4-5-20250929"
source_spec: "specs/002-physical-ai-textbook/spec.md"
created: "2025-11-29"
last_modified: "2025-11-29"
git_author: "hswat"
workflow: "/sp.implement"
version: "1.0"
prerequisites: ["chapter-03-lesson-02"]
has_interactive_python: true
interactive_python_count: 1
has_try_with_ai: false
try_with_ai_count: 0
tags: ["gazebo", "physics", "collisions", "dynamics"]
---

# Lesson 3.3: Physics Simulation in Gazebo

## 🎯 Learning Objectives

- Understand physics engine principles
- Configure gravity, friction, and damping
- Handle collisions and contact forces
- Simulate realistic robot dynamics

**Time**: 60 minutes

---

## Introduction

Gazebo uses physics engines (ODE, Bullet, Simbody) to simulate realistic physical behavior. Understanding physics simulation is crucial for accurate robot behavior prediction.

**Key Concepts**:
- Rigid body dynamics
- Collision detection and response
- Friction and damping
- Gravity and forces

---

## 1. Physics Engines

### Available Engines

1. **ODE** (Open Dynamics Engine): Default, fast, good for most cases
2. **Bullet**: More accurate, better for complex collisions
3. **Simbody**: Most accurate, slower, for research

### Physics Parameters

```xml
<physics name="default_physics" type="ode">
  <gravity>0 0 -9.81</gravity>
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
  <ode>
    <solver>
      <type>quick</type>
      <iters>10</iters>
      <sor>1.3</sor>
    </solver>
    <constraints>
      <cfm>0.00001</cfm>
      <erp>0.2</erp>
    </constraints>
  </ode>
</physics>
```

---

## 2. Collision Detection

### Collision Geometries

Collision shapes should be simpler than visual geometries for performance:

```xml
<link name="robot_link">
  <visual>
    <geometry>
      <mesh filename="complex_robot.dae"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <box size="0.2 0.2 0.4"/>  <!-- Simplified -->
    </geometry>
  </collision>
</link>
```

### Contact Properties

```xml
<collision name="collision">
  <surface>
    <friction>
      <ode>
        <mu>0.6</mu>
        <mu2>0.6</mu2>
      </ode>
    </friction>
    <bounce>
      <restitution_coefficient>0.1</restitution_coefficient>
    </bounce>
  </surface>
</collision>
```

---

## 3. Forces and Dynamics

### Applying Forces

In Gazebo plugins, you can apply forces programmatically:

```python
# Apply force to link
link = model.get_link("robot_link")
link.add_force([fx, fy, fz])
link.add_torque([tx, ty, tz])
```

### Inertial Properties

Accurate inertial properties are crucial for realistic dynamics:

```xml
<inertial>
  <mass>10.0</mass>
  <origin xyz="0 0 0.25"/>
  <inertia>
    <ixx>0.5</ixx>
    <ixy>0</ixy>
    <ixz>0</ixz>
    <iyy>0.3</iyy>
    <iyz>0</iyz>
    <izz>0.5</izz>
  </inertia>
</inertial>
```

---

## 4. Exercises

### Exercise 3.3.1: Simulate Falling Object

Calculate and visualize the motion of a falling object with gravity.

<InteractivePython
  id="ex-3-3-1"
  title="Physics Simulation: Falling Object"
  starterCode={`import numpy as np
import matplotlib.pyplot as plt

def simulate_falling_object(initial_height=10.0, dt=0.01, duration=2.0):
    """Simulate object falling under gravity."""
    g = 9.81  # m/s^2
    # TODO: Implement physics simulation
    # Position: y(t) = y0 - 0.5 * g * t^2
    # Velocity: v(t) = -g * t
    pass

# Run simulation and plot
times, positions, velocities = simulate_falling_object()
plt.plot(times, positions)
plt.xlabel('Time (s)')
plt.ylabel('Height (m)')
plt.title('Falling Object Simulation')
plt.show()
`}
  hints={[
    "Use kinematic equations: y = y0 + v0*t + 0.5*a*t^2",
    "Create time array: np.arange(0, duration, dt)",
    "Calculate position and velocity at each timestep"
  ]}
/>

---

## Summary

**Key Takeaways**:
1. Physics engines simulate realistic dynamics
2. Collision geometries should be simplified for performance
3. Accurate inertial properties are essential
4. Forces, friction, and damping affect robot behavior

**What's Next**: [Lesson 3.4: Sensor Simulation in Gazebo](./lesson-04-sensor-simulation.md) teaches you how to simulate realistic sensor data.

---

**Estimated completion time**: 60 minutes | **Prerequisites**: Lesson 3.2 | **Difficulty**: B1+ (Upper Intermediate)

