---
title: "Lesson 3.1: Gazebo Simulation Environment"
description: "Learn the fundamentals of Gazebo physics simulation for robots"
chapter: 3
lesson: 1
estimated_time: 60
cefr_level: "B1"
blooms_level: "Understand"
digcomp_level: 3
generated_by: "claude-sonnet-4-5-20250929"
source_spec: "specs/002-physical-ai-textbook/spec.md"
created: "2025-11-29"
last_modified: "2025-11-29"
git_author: "hswat"
workflow: "/sp.implement"
version: "1.0"
prerequisites: ["chapter-02-lesson-05"]
has_interactive_python: true
interactive_python_count: 1
has_try_with_ai: false
try_with_ai_count: 0
tags: ["gazebo", "simulation", "physics", "robotics"]
---

# Lesson 3.1: Gazebo Simulation Environment

## 🎯 Learning Objectives

- Understand Gazebo architecture and simulation loop
- Set up a basic Gazebo simulation environment
- Create simple world files with physics properties
- Run and interact with simulated robots

**Time**: 60 minutes

---

## Introduction

**Gazebo** is a powerful open-source 3D robotics simulator that provides realistic physics simulation, sensor modeling, and environment building. It's the industry standard for testing robot behaviors before deploying to real hardware.

**Key Features**:
- Physics engine (ODE, Bullet, Simbody)
- Realistic sensor simulation
- Plugin system for custom behaviors
- ROS 2 integration
- Multi-robot support

---

## 1. Gazebo Architecture

### Core Components

1. **Physics Engine**: Simulates forces, collisions, and dynamics
2. **Rendering Engine**: Visualizes the simulation (OGRE)
3. **Sensor System**: Generates realistic sensor data
4. **Plugin System**: Extends functionality with custom code

### Simulation Loop

```
1. Update physics (forces, collisions)
2. Update sensors (generate data)
3. Update plugins (custom behaviors)
4. Render scene (visualization)
5. Repeat at fixed timestep
```

---

## 2. Setting Up Gazebo

### Installation (Ubuntu 22.04)

```bash
sudo apt-get update
sudo apt-get install gazebo11 libgazebo11-dev
```

### Basic Usage

```bash
# Launch empty world
gazebo

# Launch specific world file
gazebo my_world.world

# Launch with ROS 2
ros2 launch gazebo_ros gazebo.launch.py
```

---

## 3. World Files

World files (`.world`) define the simulation environment:

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="default">
    <!-- Physics settings -->
    <physics name="default_physics" default="0" type="ode">
      <gravity>0 0 -9.81</gravity>
      <ode>
        <solver>
          <type>quick</type>
          <iters>10</iters>
        </solver>
      </ode>
    </physics>

    <!-- Lighting -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
  </world>
</sdf>
```

---

## 4. Exercises

### Exercise 3.1.1: Create a Simple World

Create a world file with a ground plane and a box model.

<InteractivePython
  id="ex-3-1-1"
  title="Generate World File XML"
  starterCode={`def create_simple_world():
    """Generate XML for a simple Gazebo world."""
    # TODO: Create XML string with world, physics, and ground plane
    pass

world_xml = create_simple_world()
print(world_xml)
`}
  hints={[
    "Use XML string formatting",
    "Include physics with gravity",
    "Add ground_plane model"
  ]}
/>

---

## Summary

**Key Takeaways**:
1. Gazebo provides realistic physics simulation for robots
2. World files define simulation environments
3. Physics engine handles forces, collisions, and dynamics
4. Gazebo integrates with ROS 2 for robot control

**What's Next**: [Lesson 3.2: URDF and SDF Robot Description Formats](./lesson-02-urdf-sdf.md) teaches you how to model humanoid robots for simulation.

---

**Estimated completion time**: 60 minutes | **Prerequisites**: Chapter 2 (ROS 2) | **Difficulty**: B1 (Intermediate)

