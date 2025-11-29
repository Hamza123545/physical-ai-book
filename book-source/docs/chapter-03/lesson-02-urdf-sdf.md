---
title: "Lesson 3.2: URDF and SDF Robot Description Formats"
description: "Model humanoid robots using URDF and SDF formats for Gazebo simulation"
chapter: 3
lesson: 2
estimated_time: 70
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
prerequisites: ["chapter-03-lesson-01"]
has_interactive_python: true
interactive_python_count: 1
has_try_with_ai: false
try_with_ai_count: 0
tags: ["urdf", "sdf", "robot-modeling", "gazebo"]
---

# Lesson 3.2: URDF and SDF Robot Description Formats

## 🎯 Learning Objectives

- Understand URDF and SDF robot description formats
- Model robot links, joints, and kinematic chains
- Create visual and collision geometries
- Build a humanoid robot model for simulation

**Time**: 70 minutes

---

## Introduction

**URDF** (Unified Robot Description Format) and **SDF** (Simulation Description Format) are XML-based formats for describing robot structure, appearance, and physical properties. They enable you to model complex robots like humanoids in simulation.

**Key Differences**:
- **URDF**: ROS standard, tree structure, good for kinematics
- **SDF**: Gazebo native, supports nested models, better for simulation

---

## 1. URDF Basics

### Robot Structure

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Links (rigid bodies) -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.2 0.4"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.2 0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Joints (connections) -->
  <joint name="head_joint" type="revolute">
    <parent link="base_link"/>
    <child link="head_link"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>
</robot>
```

---

## 2. Humanoid Robot Modeling

### Key Components

1. **Base/Torso**: Central body
2. **Head**: Vision and sensing
3. **Arms**: Shoulder, elbow, wrist joints
4. **Legs**: Hip, knee, ankle joints
5. **Hands/Feet**: End effectors

### Example: Simple Humanoid Torso

```xml
<link name="torso">
  <visual>
    <geometry>
      <box size="0.3 0.2 0.5"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 1 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <box size="0.3 0.2 0.5"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="10.0"/>
    <origin xyz="0 0 0.25"/>
    <inertia ixx="0.5" ixy="0" ixz="0" iyy="0.3" iyz="0" izz="0.5"/>
  </inertial>
</link>
```

---

## 3. SDF Format

SDF is Gazebo's native format and supports more simulation features:

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <model name="humanoid_robot">
    <static>false</static>
    <link name="base_link">
      <pose>0 0 1 0 0 0</pose>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.2 0.2 0.4</size>
          </box>
        </geometry>
      </visual>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.2 0.2 0.4</size>
          </box>
        </geometry>
      </collision>
      <inertial>
        <mass>5.0</mass>
        <inertia>
          <ixx>0.1</ixx>
          <iyy>0.1</iyy>
          <izz>0.1</izz>
        </inertia>
      </inertial>
    </link>
  </model>
</sdf>
```

---

## 4. Exercises

### Exercise 3.2.1: Create a Simple Arm

Model a 2-DOF robot arm with shoulder and elbow joints.

<InteractivePython
  id="ex-3-2-1"
  title="Generate URDF for Robot Arm"
  starterCode={`def create_robot_arm_urdf():
    """Generate URDF XML for a 2-DOF arm."""
    # TODO: Create URDF with base_link, upper_arm, lower_arm
    # Include shoulder and elbow joints
    pass

urdf = create_robot_arm_urdf()
print(urdf)
`}
  hints={[
    "Define links: base_link, upper_arm_link, lower_arm_link",
    "Add joints: shoulder_joint, elbow_joint",
    "Use revolute joints with limits"
  ]}
/>

---

## Summary

**Key Takeaways**:
1. URDF and SDF describe robot structure in XML
2. Links are rigid bodies, joints connect them
3. Visual and collision geometries can differ
4. Inertial properties affect physics simulation

**What's Next**: [Lesson 3.3: Physics Simulation in Gazebo](./lesson-03-physics-simulation.md) explores how physics properties affect robot behavior.

---

**Estimated completion time**: 70 minutes | **Prerequisites**: Lesson 3.1 | **Difficulty**: B1+ (Upper Intermediate)

