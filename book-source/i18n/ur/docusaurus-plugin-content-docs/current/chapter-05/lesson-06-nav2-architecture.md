---
title: "Lesson 5.6: Nav2 - Path Planning Framework"
description: "Master Nav2, the ROS 2 navigation framework for robot path planning and execution"
chapter: 5
lesson: 6
estimated_time: 70
cefr_level: "B2"
blooms_level: "Apply"
digcomp_level: 6
generated_by: "claude-sonnet-4-5-20250929"
source_spec: "specs/002-physical-ai-textbook/spec.md"
created: "2025-11-29"
last_modified: "2025-11-29"
git_author: "hswat"
workflow: "/sp.implement"
version: "1.0"
prerequisites:
  - "chapter-02-index"
  - "chapter-05-lesson-01"
has_interactive_python: true
interactive_python_count: 4
has_try_with_ai: true
try_with_ai_count: 2
tags: ["nav2", "navigation", "path-planning", "ros2", "localization"]
---

# Lesson 5.6: Nav2 - Path Planning Framework

## 🎯 Learning Objectives

<LearningObjectives
  cefr_level="B2"
  objectives={[
    {
      text: "Understand Nav2 architecture and core components for ROS 2 navigation",
      blooms_level: "Understand",
      assessment_method: "Quiz questions"
    },
    {
      text: "Configure Nav2 stack for robot navigation with localization and path planning",
      blooms_level: "Apply",
      assessment_method: "Interactive Python exercises"
    },
    {
      text: "Implement Nav2 action clients to command robot navigation",
      blooms_level: "Apply",
      assessment_method: "Interactive Python exercises"
    },
    {
      text: "Integrate Nav2 with robot sensors and actuators for complete navigation system",
      blooms_level: "Apply",
      assessment_method: "Interactive Python exercises"
    }
  ]}
/>

## 📚 Prerequisites

<Prerequisites
  prerequisites={[
    {
      lessonId: "chapter-02-index",
      title: "Chapter 2: ROS 2 Fundamentals",
      link: "/docs/chapter-02/index"
    },
    {
      lessonId: "chapter-05-lesson-01",
      title: "Lesson 5.1: Path Planning Basics",
      link: "/docs/chapter-05/lesson-01-path-planning"
    }
  ]}
/>

## Introduction

**Nav2** (Navigation2) is the ROS 2 navigation framework that provides a complete solution for robot navigation, including localization, path planning, and obstacle avoidance. It's the successor to ROS 1's `move_base` and is designed for ROS 2's distributed architecture.

**Key Features**:
- **Modular Architecture**: Pluggable planners, controllers, and recovery behaviors
- **Action-Based**: Uses ROS 2 actions for navigation goals
- **Costmap-Based**: Dynamic obstacle avoidance using costmaps
- **Localization Integration**: Works with AMCL, SLAM, or custom localization
- **Recovery Behaviors**: Automatic recovery from navigation failures

**Time**: 70 minutes

---

## 1. What is Nav2?

Nav2 is a behavior tree-based navigation framework that orchestrates multiple components:

### Core Components:
- **Planner**: Computes global path from start to goal
- **Controller**: Executes local path following with obstacle avoidance
- **Recovery**: Handles navigation failures (rotate, clear costmap, etc.)
- **BT Navigator**: Behavior tree orchestrator
- **Costmap 2D**: Dynamic obstacle representation

### Architecture:
```
┌─────────────────────────────────────┐
│     Nav2 Behavior Tree              │
│     (BT Navigator)                  │
├─────────────────────────────────────┤
│  Planner  │  Controller  │ Recovery │
├─────────────────────────────────────┤
│     Costmap 2D (Obstacle Map)       │
├─────────────────────────────────────┤
│     Localization (AMCL/SLAM)        │
├─────────────────────────────────────┤
│     ROS 2 Topics & Actions          │
└─────────────────────────────────────┘
```

---

## 2. Nav2 Components

### Planner Server:
- Computes global path from current pose to goal
- Pluggable planners: NavFn, Smac Planner, Theta*
- Input: Start pose, goal pose, costmap
- Output: Global path (sequence of waypoints)

### Controller Server:
- Executes local path following
- Obstacle avoidance using local costmap
- Pluggable controllers: DWB, TEB, Pure Pursuit
- Input: Global path, current pose, costmap
- Output: Velocity commands

### Recovery Server:
- Handles navigation failures
- Behaviors: Spin, Back Up, Clear Costmap
- Triggered when controller fails

### BT Navigator:
- Behavior tree orchestrator
- Coordinates planner, controller, recovery
- Manages navigation state machine

---

## 3. Costmap 2D

**Costmap** represents obstacles and free space as a 2D grid.

### Costmap Layers:
- **Static Layer**: Static map (from SLAM)
- **Obstacle Layer**: Dynamic obstacles (from sensors)
- **Inflation Layer**: Safety margins around obstacles

### Cost Values:
- `0`: Free space
- `1-252`: Increasing cost (near obstacles)
- `253`: Inscribed obstacle
- `254`: Lethal obstacle (collision)

---

## 4. Setting Up Nav2

### Installation:
```bash
# Install Nav2 (conceptual)
sudo apt install ros-humble-navigation2
```

### Basic Configuration:
1. **Launch Nav2**: Start Nav2 stack with configuration
2. **Provide Map**: Load static map (from SLAM)
3. **Set Initial Pose**: Provide robot's starting location
4. **Send Goal**: Use Nav2 action to command navigation

---

## 5. Exercises

### Exercise 5.6.1: Design Nav2 Configuration

Design a Nav2 configuration for a mobile robot.

<InteractivePython
  id="ex-5-6-1"
  title="Nav2 Configuration Design"
  starterCode={`def design_nav2_config(robot_type="differential_drive"):
    """
    Designs Nav2 configuration for a robot.
    """
    config = {
        "planner": {
            "plugin": "nav2_smac_planner/SmacPlanner",
            "tolerance": 0.25,  # meters
            "max_iterations": 1000000
        },
        "controller": {
            "plugin": "dwb_core::DWBLocalPlanner",
            "max_vel_x": 0.5,  # m/s
            "max_vel_theta": 1.0,  # rad/s
            "acc_lim_x": 2.5,  # m/s²
            "acc_lim_theta": 3.2  # rad/s²
        },
        "costmap": {
            "global_frame": "map",
            "robot_base_frame": "base_link",
            "update_frequency": 1.0,  # Hz
            "publish_frequency": 1.0,  # Hz
            "resolution": 0.05,  # meters per pixel
            "inflation_radius": 0.55  # meters
        },
        "recovery": {
            "behaviors": ["spin", "back_up", "wait"],
            "spin_angle": 1.57,  # radians (90 degrees)
            "back_up_distance": 0.15  # meters
        }
    }
    
    # Adjust for robot type
    if robot_type == "bipedal_humanoid":
        config["controller"]["max_vel_x"] = 0.3  # Slower for humanoids
        config["controller"]["max_vel_theta"] = 0.5
        config["costmap"]["inflation_radius"] = 0.8  # Larger safety margin
    
    return config

# Test configurations
print("Differential Drive Robot Configuration:")
dd_config = design_nav2_config("differential_drive")
print(f"  Max velocity: {dd_config['controller']['max_vel_x']} m/s")
print(f"  Inflation radius: {dd_config['costmap']['inflation_radius']} m")

print("\nBipedal Humanoid Configuration:")
humanoid_config = design_nav2_config("bipedal_humanoid")
print(f"  Max velocity: {humanoid_config['controller']['max_vel_x']} m/s")
print(f"  Inflation radius: {humanoid_config['costmap']['inflation_radius']} m")
`}
  hints={[
    "Consider robot-specific constraints when configuring Nav2.",
    "Humanoid robots need more conservative parameters due to balance constraints."
  ]}
/>

---

### Exercise 5.6.2: Simulate Nav2 Action Client

Simulate sending navigation goals to Nav2 using ROS 2 actions.

<InteractivePython
  id="ex-5-6-2"
  title="Nav2 Action Client Simulation"
  starterCode={`class Nav2ActionClient:
    """
    Simulates Nav2 action client for sending navigation goals.
    """
    def __init__(self):
        self.action_name = "/navigate_to_pose"
        print(f"Nav2 Action Client initialized: {self.action_name}")
    
    def send_goal(self, x, y, theta):
        """
        Sends navigation goal to Nav2.
        In real ROS 2, this would use NavigateToPose action.
        """
        goal = {
            "pose": {
                "position": {"x": x, "y": y, "z": 0.0},
                "orientation": {"z": theta, "w": 1.0}  # Simplified
            }
        }
        
        print(f"Sending navigation goal: x={x:.2f}, y={y:.2f}, theta={theta:.2f}")
        print("Goal sent. Waiting for result...")
        
        # Simulate navigation result
        result = {
            "status": "SUCCEEDED",
            "total_elapsed_time": 15.3  # seconds
        }
        
        return goal, result
    
    def cancel_goal(self):
        """Cancels current navigation goal."""
        print("Navigation goal cancelled.")
        return True

# Test
client = Nav2ActionClient()

# Send goal to position (2.0, 3.0) with orientation 0.0
goal, result = client.send_goal(2.0, 3.0, 0.0)
print(f"\nResult: {result['status']}")
print(f"Time taken: {result['total_elapsed_time']} seconds")
`}
  hints={[
    "Nav2 uses ROS 2 actions for navigation goals, allowing for feedback and cancellation.",
    "In real systems, you would subscribe to feedback topics to monitor progress."
  ]}
/>

---

### Exercise 5.6.3: Configure Costmap for Obstacle Avoidance

Design costmap configuration for effective obstacle avoidance.

<InteractivePython
  id="ex-5-6-3"
  title="Costmap Configuration"
  starterCode={`def configure_costmap(robot_radius=0.3, safety_margin=0.2):
    """
    Configures costmap parameters for obstacle avoidance.
    """
    costmap_config = {
        "global_costmap": {
            "plugin": "nav2_costmap_2d::Nav2Costmap2D",
            "global_frame": "map",
            "robot_base_frame": "base_link",
            "update_frequency": 1.0,
            "publish_frequency": 1.0,
            "resolution": 0.05,  # 5 cm per pixel
            "width": 10.0,  # meters
            "height": 10.0,  # meters
            "inflation_radius": robot_radius + safety_margin
        },
        "local_costmap": {
            "plugin": "nav2_costmap_2d::Nav2Costmap2D",
            "global_frame": "odom",
            "robot_base_frame": "base_link",
            "update_frequency": 5.0,  # Higher frequency for local
            "publish_frequency": 2.0,
            "resolution": 0.05,
            "width": 3.0,  # Smaller for local
            "height": 3.0,
            "inflation_radius": robot_radius + safety_margin
        },
        "obstacle_layer": {
            "observation_sources": "scan",
            "scan": {
                "topic": "/scan",
                "data_type": "LaserScan",
                "clearing": True,
                "marking": True
            }
        }
    }
    
    return costmap_config

# Test
config = configure_costmap(robot_radius=0.3, safety_margin=0.2)
print("Costmap Configuration:")
print(f"\nGlobal Costmap:")
print(f"  Resolution: {config['global_costmap']['resolution']} m/pixel")
print(f"  Size: {config['global_costmap']['width']} x {config['global_costmap']['height']} m")
print(f"  Inflation radius: {config['global_costmap']['inflation_radius']} m")

print(f"\nLocal Costmap:")
print(f"  Resolution: {config['local_costmap']['resolution']} m/pixel")
print(f"  Size: {config['local_costmap']['width']} x {config['local_costmap']['height']} m")
print(f"  Update frequency: {config['local_costmap']['update_frequency']} Hz")
`}
  hints={[
    "Global costmap is for long-term planning, local costmap is for immediate obstacle avoidance.",
    "Inflation radius should include robot radius plus safety margin."
  ]}
/>

---

### Exercise 5.6.4: Design Nav2 Integration Pipeline

Design a complete integration pipeline for Nav2 with robot sensors and actuators.

<InteractivePython
  id="ex-5-6-4"
  title="Nav2 Integration Pipeline"
  starterCode={`def design_nav2_integration():
    """
    Designs integration pipeline for Nav2 with robot system.
    """
    integration = {
        "localization": {
            "source": "amcl",  # or "slam_toolbox"
            "map_topic": "/map",
            "initial_pose_topic": "/initialpose",
            "amcl_pose_topic": "/amcl_pose"
        },
        "sensors": {
            "laser_scan": {
                "topic": "/scan",
                "frame": "laser_frame",
                "type": "LaserScan"
            },
            "odometry": {
                "topic": "/odom",
                "frame": "odom",
                "type": "Odometry"
            }
        },
        "nav2_components": {
            "bt_navigator": "/navigate_to_pose",
            "planner_server": "/plan",
            "controller_server": "/cmd_vel",
            "recovery_server": "/recoveries"
        },
        "actuators": {
            "cmd_vel": {
                "topic": "/cmd_vel",
                "type": "Twist",
                "subscriber": "robot_base_controller"
            }
        },
        "data_flow": [
            "1. Localization provides robot pose",
            "2. Sensors provide obstacle data to costmap",
            "3. User sends navigation goal via Nav2 action",
            "4. Planner computes global path",
            "5. Controller generates velocity commands",
            "6. Commands sent to robot actuators",
            "7. Recovery behaviors handle failures"
        ]
    }
    
    return integration

# Display integration
integration = design_nav2_integration()
print("Nav2 Integration Pipeline:")
print("\n1. Localization:")
for key, value in integration["localization"].items():
    print(f"   - {key}: {value}")

print("\n2. Sensors:")
for sensor, config in integration["sensors"].items():
    print(f"   - {sensor}: {config['topic']} ({config['type']})")

print("\n3. Nav2 Components:")
for component, topic in integration["nav2_components"].items():
    print(f"   - {component}: {topic}")

print("\n4. Actuators:")
for actuator, config in integration["actuators"].items():
    print(f"   - {actuator}: {config['topic']} ({config['type']})")

print("\n5. Data Flow:")
for step in integration["data_flow"]:
    print(f"   {step}")
`}
  hints={[
    "Nav2 integrates multiple components: localization, sensors, planning, control, and actuation.",
    "Consider the data flow from goal to execution."
  ]}
/>

---

## 6. Try With AI

### TryWithAI 5.6.1: Optimize Nav2 for Specific Robot

<TryWithAI
  id="tryai-5-6-1"
  title="Optimize Nav2 Configuration for Your Robot"
  role="Copilot"
  scenario="You need to configure Nav2 for a specific robot with unique constraints (e.g., slow acceleration, large footprint, or specific sensor configuration)."
  yourTask="List your robot's constraints (size, speed, sensors) and suggest Nav2 configuration parameters (planner, controller, costmap settings) that would work well."
  aiPromptTemplate="I'm configuring Nav2 for a robot with these constraints: [paste your constraints]. Here are my suggested Nav2 parameters: [paste your suggestions]. Can you help me refine these parameters, identify potential issues, and suggest optimal values for planner, controller, and costmap settings?"
  successCriteria={[
    "You identified at least 5 Nav2 configuration parameters.",
    "You understand how robot constraints affect Nav2 settings.",
    "You can explain the trade-offs between different parameter choices."
  ]}
  reflectionQuestions={[
    "How would you test if your Nav2 configuration is optimal?",
    "What metrics would you use to evaluate navigation performance?",
    "How would you handle navigation failures in your configuration?"
  ]}
/>

---

### TryWithAI 5.6.2: Debug Nav2 Navigation Failures

<TryWithAI
  id="tryai-5-6-2"
  title="Debug Nav2 Navigation Failures"
  role="Evaluator"
  scenario="Your robot using Nav2 frequently fails to reach goals, gets stuck, or takes inefficient paths. You need to diagnose the issues."
  yourTask="List 3-5 potential causes for Nav2 navigation failures and suggest diagnostic steps and solutions for each."
  aiPromptTemplate="My robot using Nav2 is experiencing navigation failures: [describe failures]. Here are my hypotheses: [paste your list]. Can you help me prioritize which issues are most likely, suggest specific diagnostic steps (e.g., checking costmap, planner output, controller behavior), and recommend fixes for each identified problem?"
  successCriteria={[
    "You identified at least 4 potential causes of navigation failures.",
    "You suggested specific diagnostic steps for each issue.",
    "You can explain how to fix common Nav2 problems."
  ]}
  reflectionQuestions={[
    "How would you systematically debug Nav2 navigation issues?",
    "What ROS 2 topics would you monitor to diagnose problems?",
    "How would you validate that your fixes actually work?"
  ]}
/>

---

## Summary

**Key Takeaways**:

1. **Nav2** is the ROS 2 navigation framework providing complete navigation solution.
2. **Modular Architecture** with pluggable planners, controllers, and recovery behaviors.
3. **Costmap 2D** represents obstacles and free space for path planning and obstacle avoidance.
4. **Action-Based** interface allows for goal management, feedback, and cancellation.
5. **Integration** requires localization, sensors, and actuators working together.

**What's Next**: [Lesson 5.7: Nav2 for Bipedal Humanoids](./lesson-07-nav2-humanoids.md) will explore how to adapt Nav2 specifically for bipedal humanoid robots with balance constraints.

---

**Estimated completion time**: 70 minutes | **Prerequisites**: Chapter 2, Lesson 5.1 | **Difficulty**: B2 (Advanced)

