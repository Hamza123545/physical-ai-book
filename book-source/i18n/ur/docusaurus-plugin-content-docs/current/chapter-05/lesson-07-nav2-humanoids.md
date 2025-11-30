---
title: "Lesson 5.7: Nav2 for Bipedal Humanoid Path Planning"
description: "Adapt Nav2 for bipedal humanoid robots with balance constraints and bipedal locomotion"
chapter: 5
lesson: 7
estimated_time: 60
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
  - "chapter-05-lesson-06"
  - "chapter-06-index"
has_interactive_python: true
interactive_python_count: 3
has_try_with_ai: true
try_with_ai_count: 1
tags: ["nav2", "humanoid", "bipedal", "balance", "locomotion", "path-planning"]
---

# Lesson 5.7: Nav2 for Bipedal Humanoid Path Planning

## 🎯 Learning Objectives

<LearningObjectives
  cefr_level="B2"
  objectives={[
    {
      text: "Understand unique challenges of applying Nav2 to bipedal humanoid robots",
      blooms_level: "Understand",
      assessment_method: "Quiz questions"
    },
    {
      text: "Configure Nav2 with balance constraints for humanoid navigation",
      blooms_level: "Apply",
      assessment_method: "Interactive Python exercises"
    },
    {
      text: "Design custom Nav2 plugins for humanoid-specific path planning",
      blooms_level: "Apply",
      assessment_method: "Interactive Python exercises"
    },
    {
      text: "Integrate Nav2 with humanoid locomotion and balance control",
      blooms_level: "Apply",
      assessment_method: "Interactive Python exercises"
    }
  ]}
/>

## 📚 Prerequisites

<Prerequisites
  prerequisites={[
    {
      lessonId: "chapter-05-lesson-06",
      title: "Lesson 5.6: Nav2 Architecture",
      link: "/docs/chapter-05/lesson-06-nav2-architecture"
    },
    {
      lessonId: "chapter-06-index",
      title: "Chapter 6: Humanoid Robot Development",
      link: "/docs/chapter-06/index"
    }
  ]}
/>

## Introduction

Applying **Nav2 to bipedal humanoid robots** presents unique challenges compared to wheeled robots. Humanoids must maintain balance, have limited turning radius, and require careful footstep planning. This lesson adapts Nav2 for humanoid-specific navigation.

**Key Challenges**:
- **Balance Constraints**: Paths must respect stability requirements
- **Footstep Planning**: Need to plan foot placements, not just center-of-mass path
- **Limited Mobility**: Slower speeds, larger turning radius
- **Dynamic Stability**: Must account for dynamic balance during motion

**Time**: 60 minutes

---

## 1. Humanoid Navigation Challenges

### Differences from Wheeled Robots:

#### 1. Balance Requirements:
- **Support Polygon**: Feet must remain in stable support region
- **ZMP Constraint**: Zero Moment Point must stay within support polygon
- **Dynamic Balance**: Must account for momentum during walking

#### 2. Locomotion Constraints:
- **Step Length**: Limited by leg kinematics
- **Turning Radius**: Larger minimum turning radius
- **Speed Limits**: Slower maximum velocity for stability

#### 3. Footstep Planning:
- **Discrete Steps**: Must plan discrete foot placements
- **Terrain Adaptation**: Adjust steps for uneven terrain
- **Gait Patterns**: Coordinate walking gaits with path

---

## 2. Adapting Nav2 for Humanoids

### Configuration Adjustments:

#### 1. Conservative Parameters:
```yaml
controller:
  max_vel_x: 0.3  # Slower than wheeled robots
  max_vel_theta: 0.5  # Limited turning
  acc_lim_x: 1.0  # Gentle acceleration
  acc_lim_theta: 1.0  # Smooth turning
```

#### 2. Larger Safety Margins:
```yaml
costmap:
  inflation_radius: 0.8  # Larger than robot radius
  obstacle_range: 2.5  # Look further ahead
```

#### 3. Custom Planner:
- Consider balance constraints in path planning
- Plan paths that avoid sharp turns
- Account for support polygon requirements

---

## 3. Footstep Planning Integration

### Approach:
1. **Nav2 Plans Path**: Global path from Nav2 planner
2. **Footstep Planner**: Converts path to footstep sequence
3. **Balance Check**: Validates each step for stability
4. **Gait Execution**: Executes walking gait

### Integration Architecture:
```
Nav2 Global Path
    ↓
Footstep Planner (converts path to steps)
    ↓
Balance Validator (checks ZMP/stability)
    ↓
Gait Controller (executes walking)
    ↓
Robot Motion
```

---

## 4. Exercises

### Exercise 5.7.1: Configure Nav2 for Humanoid

Configure Nav2 parameters specifically for a bipedal humanoid robot.

<InteractivePython
  id="ex-5-7-1"
  title="Nav2 Humanoid Configuration"
  starterCode={`def configure_nav2_for_humanoid(robot_height=1.5, foot_length=0.2):
    """
    Configures Nav2 specifically for bipedal humanoid robots.
    """
    config = {
        "planner": {
            "plugin": "nav2_smac_planner/SmacPlanner",
            "tolerance": 0.3,  # Larger tolerance for humanoids
            "allow_unknown": False,  # Require known space
            "max_planning_time": 5.0  # Allow more time
        },
        "controller": {
            "plugin": "dwb_core::DWBLocalPlanner",
            "max_vel_x": 0.3,  # m/s - conservative for balance
            "max_vel_theta": 0.5,  # rad/s - limited turning
            "min_vel_x": 0.0,
            "min_vel_theta": -0.5,
            "acc_lim_x": 1.0,  # m/s² - gentle acceleration
            "acc_lim_theta": 1.0,  # rad/s² - smooth turning
            "decel_lim_x": 1.5,  # Faster deceleration for safety
            "xy_goal_tolerance": 0.25,  # meters
            "yaw_goal_tolerance": 0.25  # radians
        },
        "costmap": {
            "global_frame": "map",
            "robot_base_frame": "base_link",
            "resolution": 0.05,
            "inflation_radius": 0.8,  # Larger safety margin
            "obstacle_range": 2.5,  # Look further ahead
            "raytrace_range": 3.0
        },
        "humanoid_specific": {
            "balance_constraint": True,
            "min_step_length": 0.1,  # meters
            "max_step_length": 0.3,  # meters
            "support_polygon_margin": 0.1,  # meters
            "zmp_safety_margin": 0.05  # meters
        }
    }
    
    return config

# Test
config = configure_nav2_for_humanoid()
print("Nav2 Configuration for Bipedal Humanoid:")
print(f"\nController:")
print(f"  Max velocity: {config['controller']['max_vel_x']} m/s")
print(f"  Max angular velocity: {config['controller']['max_vel_theta']} rad/s")
print(f"  Acceleration limit: {config['controller']['acc_lim_x']} m/s²")

print(f"\nCostmap:")
print(f"  Inflation radius: {config['costmap']['inflation_radius']} m")
print(f"  Obstacle range: {config['costmap']['obstacle_range']} m")

print(f"\nHumanoid-Specific:")
print(f"  Min step length: {config['humanoid_specific']['min_step_length']} m")
print(f"  Max step length: {config['humanoid_specific']['max_step_length']} m")
print(f"  Support polygon margin: {config['humanoid_specific']['support_polygon_margin']} m")
`}
  hints={[
    "Humanoid robots need more conservative parameters due to balance constraints.",
    "Consider step length limits and support polygon requirements."
  ]}
/>

---

### Exercise 5.7.2: Validate Path for Balance Constraints

Simulate validating a Nav2 path for humanoid balance constraints.

<InteractivePython
  id="ex-5-7-2"
  title="Balance Constraint Validation"
  starterCode={`import numpy as np

def validate_path_for_balance(path, support_polygon_size=0.2, zmp_margin=0.05):
    """
    Validates a Nav2 path for humanoid balance constraints.
    """
    issues = []
    
    for i, waypoint in enumerate(path):
        x, y, theta = waypoint["x"], waypoint["y"], waypoint["theta"]
        
        # Check if waypoint is too close to obstacles (simplified)
        # In real system, this would check costmap
        
        # Check turning radius
        if i > 0:
            prev_theta = path[i-1]["theta"]
            angle_change = abs(theta - prev_theta)
            
            # Humanoids have limited turning capability
            max_angle_per_step = 0.3  # radians (~17 degrees)
            if angle_change > max_angle_per_step:
                issues.append({
                    "waypoint": i,
                    "issue": "excessive_turn",
                    "angle_change": angle_change,
                    "max_allowed": max_angle_per_step
                })
        
        # Check step length (distance from previous waypoint)
        if i > 0:
            prev_x, prev_y = path[i-1]["x"], path[i-1]["y"]
            step_length = np.sqrt((x - prev_x)**2 + (y - prev_y)**2)
            
            max_step_length = 0.3  # meters
            if step_length > max_step_length:
                issues.append({
                    "waypoint": i,
                    "issue": "step_too_long",
                    "step_length": step_length,
                    "max_allowed": max_step_length
                })
    
    return issues

# Test with sample path
sample_path = [
    {"x": 0.0, "y": 0.0, "theta": 0.0},
    {"x": 0.2, "y": 0.0, "theta": 0.0},
    {"x": 0.4, "y": 0.0, "theta": 0.0},
    {"x": 0.6, "y": 0.1, "theta": 0.5},  # Large turn
    {"x": 1.0, "y": 0.2, "theta": 0.5}   # Long step
]

issues = validate_path_for_balance(sample_path)

if issues:
    print("Path Validation Issues Found:")
    for issue in issues:
        print(f"  Waypoint {issue['waypoint']}: {issue['issue']}")
        if 'angle_change' in issue:
            print(f"    Angle change: {issue['angle_change']:.2f} rad (max: {issue['max_allowed']:.2f})")
        if 'step_length' in issue:
            print(f"    Step length: {issue['step_length']:.2f} m (max: {issue['max_allowed']:.2f} m)")
else:
    print("Path validated successfully - all balance constraints satisfied!")
`}
  hints={[
    "Humanoid paths must respect step length and turning angle limits.",
    "Balance validation ensures the robot can execute the path safely."
  ]}
/>

---

### Exercise 5.7.3: Design Humanoid Navigation Pipeline

Design a complete navigation pipeline integrating Nav2 with humanoid locomotion.

<InteractivePython
  id="ex-5-7-3"
  title="Humanoid Navigation Pipeline"
  starterCode={`def design_humanoid_navigation_pipeline():
    """
    Designs navigation pipeline for bipedal humanoid using Nav2.
    """
    pipeline = {
        "localization": {
            "source": "amcl",  # or VSLAM for humanoids
            "base_frame": "base_link",
            "map_frame": "map"
        },
        "path_planning": {
            "global_planner": "nav2_smac_planner",
            "footstep_planner": "custom_humanoid_footstep_planner",
            "balance_validator": "zmp_validator"
        },
        "execution": {
            "gait_controller": "walking_gait_controller",
            "balance_controller": "zmp_balance_controller",
            "step_executor": "footstep_executor"
        },
        "integration": {
            "nav2_to_footsteps": "Convert Nav2 path to footstep sequence",
            "footsteps_to_gait": "Generate walking gait from footsteps",
            "gait_to_balance": "Execute gait with balance control",
            "feedback_to_nav2": "Provide pose feedback to Nav2"
        },
        "data_flow": [
            "1. Nav2 receives navigation goal",
            "2. Nav2 planner computes global path",
            "3. Footstep planner converts path to footsteps",
            "4. Balance validator checks each step",
            "5. Gait controller generates walking motion",
            "6. Balance controller maintains stability",
            "7. Robot executes steps",
            "8. Pose feedback updates Nav2"
        ]
    }
    
    return pipeline

# Display pipeline
pipeline = design_humanoid_navigation_pipeline()
print("Humanoid Navigation Pipeline:")
print("\n1. Localization:")
for key, value in pipeline["localization"].items():
    print(f"   - {key}: {value}")

print("\n2. Path Planning:")
for component, plugin in pipeline["path_planning"].items():
    print(f"   - {component}: {plugin}")

print("\n3. Execution:")
for component, plugin in pipeline["execution"].items():
    print(f"   - {component}: {plugin}")

print("\n4. Integration Steps:")
for step, description in pipeline["integration"].items():
    print(f"   - {step}: {description}")

print("\n5. Data Flow:")
for step in pipeline["data_flow"]:
    print(f"   {step}")
`}
  hints={[
    "Humanoid navigation requires additional layers: footstep planning and balance control.",
    "Consider how Nav2's path integrates with humanoid-specific locomotion."
  ]}
/>

---

## 5. Try With AI

### TryWithAI 5.7.1: Design Humanoid-Specific Nav2 Plugin

<TryWithAI
  id="tryai-5-7-1"
  title="Design Custom Nav2 Plugin for Humanoids"
  role="Copilot"
  scenario="You need to create a custom Nav2 planner plugin that considers humanoid balance constraints when planning paths."
  yourTask="List the key features your custom humanoid planner would need (e.g., balance checking, step length limits, turning constraints) and suggest how to implement them."
  aiPromptTemplate="I'm designing a custom Nav2 planner plugin for humanoid robots. Here are my ideas: [paste your list]. Can you help me refine the design, suggest specific algorithms or techniques for balance-aware path planning, and recommend how to integrate it with Nav2's plugin architecture?"
  successCriteria={[
    "You identified at least 4 key features for humanoid path planning.",
    "You understand how to create Nav2 plugins.",
    "You can explain how balance constraints affect path planning."
  ]}
  reflectionQuestions={[
    "How would you test your custom planner to ensure it respects balance constraints?",
    "What's the trade-off between path optimality and balance safety?",
    "How would you handle dynamic obstacles with a humanoid planner?"
  ]}
/>

---

## Summary

**Key Takeaways**:

1. **Humanoid Navigation** requires adapting Nav2 for balance constraints and bipedal locomotion.
2. **Conservative Parameters** are needed: slower speeds, larger safety margins, gentle accelerations.
3. **Footstep Planning** converts Nav2's continuous path to discrete foot placements.
4. **Balance Validation** ensures each step maintains stability (ZMP within support polygon).
5. **Integration** requires coordinating Nav2, footstep planner, gait controller, and balance controller.

**Best Practices**:
- Use larger inflation radius and safety margins
- Limit maximum velocities and accelerations
- Validate paths for balance constraints
- Integrate footstep planning with Nav2 path
- Monitor balance during execution

**What's Next**: You've completed Chapter 5! Review the [Chapter 5 Quiz](./quiz.md) to test your understanding, then proceed to [Chapter 6: Humanoid Robot Development](../chapter-06/index.md) to learn about humanoid control systems.

---

**Estimated completion time**: 60 minutes | **Prerequisites**: Lesson 5.6, Chapter 6 | **Difficulty**: B2 (Advanced)

