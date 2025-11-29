---
title: "Lesson 3.5: Unity for High-Fidelity Rendering"
description: "Use Unity for photorealistic robot visualization and human-robot interaction"
chapter: 3
lesson: 5
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
prerequisites: ["chapter-03-lesson-04"]
has_interactive_python: true
interactive_python_count: 1
has_try_with_ai: false
try_with_ai_count: 0
tags: ["unity", "rendering", "visualization", "human-robot-interaction"]
---

# Lesson 3.5: Unity for High-Fidelity Rendering

## 🎯 Learning Objectives

- Set up Unity scenes for robot visualization
- Integrate Unity with ROS 2 for real-time robot control
- Create photorealistic human-robot interaction scenes
- Animate robot movements in Unity

**Time**: 60 minutes

---

## Introduction

**Unity** provides high-fidelity rendering capabilities that complement Gazebo's physics simulation. While Gazebo focuses on accurate physics, Unity excels at photorealistic visualization and human-robot interaction scenes.

**Key Use Cases**:
- Photorealistic robot visualization
- Human-robot interaction design
- Virtual reality (VR) robot training
- Marketing and demonstration videos

---

## 1. Unity Basics

### Unity Scene Setup

1. **Create Scene**: New 3D scene
2. **Import Robot Model**: FBX or OBJ format
3. **Add Lighting**: Realistic lighting setup
4. **Configure Camera**: Optimal viewing angle

### ROS 2 Integration

Unity can communicate with ROS 2 using:
- **ROS#**: C# ROS client for Unity
- **ROS TCP Connector**: Real-time communication
- **Custom Plugins**: Direct socket communication

---

## 2. Robot Visualization

### Importing Robot Models

```csharp
// Unity C# script for robot control
using UnityEngine;
using RosSharp.RosBridgeClient;

public class RobotController : MonoBehaviour {
    public RosConnector rosConnector;
    private JointStateSubscriber jointSubscriber;
    
    void Start() {
        jointSubscriber = GetComponent<JointStateSubscriber>();
        jointSubscriber.Start();
    }
    
    void Update() {
        // Update robot joint positions from ROS 2
        UpdateJointPositions();
    }
}
```

---

## 3. Human-Robot Interaction

### Character Animation

Unity's animation system enables:
- Human character models
- Gesture recognition visualization
- Social interaction scenarios
- Proxemics (personal space) visualization

### Interaction Scenarios

- **Handshaking**: Robot and human hand interaction
- **Object Handoff**: Transferring objects
- **Conversational Gestures**: Natural language interaction
- **Collaborative Tasks**: Working together

---

## 4. Photorealistic Rendering

### Lighting Setup

- **Directional Light**: Sun/moon simulation
- **Point Lights**: Indoor lighting
- **Spot Lights**: Focused illumination
- **Ambient Occlusion**: Realistic shadows

### Materials and Textures

- **PBR Materials**: Physically-based rendering
- **Normal Maps**: Surface detail
- **Reflection Probes**: Realistic reflections
- **Post-Processing**: Color grading, bloom

---

## 5. Exercises

### Exercise 3.5.1: ROS 2 to Unity Bridge

Create a Python script that bridges ROS 2 joint states to Unity.

<InteractivePython
  id="ex-3-5-1"
  title="ROS 2 Unity Bridge"
  starterCode={`import json
import socket

def create_unity_message(joint_states):
    """
    Convert ROS 2 joint states to Unity message format.
    
    Args:
        joint_states: Dict with joint names and positions
    
    Returns:
        JSON string for Unity
    """
    # TODO: Format joint states as JSON for Unity
    pass

# Test
joints = {
    "shoulder": 0.5,
    "elbow": 0.3,
    "wrist": 0.1
}
message = create_unity_message(joints)
print(message)
`}
  hints={[
    "Use json.dumps() to create JSON string",
    "Format: {'joints': [{'name': 'shoulder', 'position': 0.5}, ...]}",
    "Ensure numeric values are floats"
  ]}
/>

---

## Summary

**Key Takeaways**:
1. Unity provides photorealistic rendering
2. ROS 2 integration enables real-time robot visualization
3. Human-robot interaction scenes help design natural behaviors
4. Unity complements Gazebo's physics with visual fidelity

**What's Next**: You've completed Module 2! Next, [Chapter 4: Reinforcement Learning for Robotics](./../chapter-04/index.md) introduces AI-powered robot control.

---

**Estimated completion time**: 60 minutes | **Prerequisites**: Lesson 3.4 | **Difficulty**: B2 (Advanced)

