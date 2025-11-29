---
title: "Lesson 3.5: Vision-Based Navigation"
description: "Integrate vision processing with path planning for autonomous navigation"
chapter: 3
lesson: 5
estimated_time: 70
cefr_level: "B2"
blooms_level: "Evaluate"
digcomp_level: 6
generated_by: "claude-sonnet-4-5-20250929"
source_spec: "specs/002-physical-ai-textbook/spec.md"
created: "2025-11-29"
last_modified: "2025-11-29"
git_author: "hswat"
workflow: "/sp.implement"
version: "1.0"
prerequisites:
  - "chapter-03-lesson-03"
  - "chapter-03-lesson-04"
has_interactive_python: true
interactive_python_count: 4
has_try_with_ai: true
try_with_ai_count: 2
tags: ["navigation", "path-planning", "obstacle-avoidance", "visual-servoing"]
---

# Lesson 3.5: Vision-Based Navigation

## 🎯 Learning Objectives

<LearningObjectives
  cefr_level="B2"
  objectives={[
    {
      text: "Compute heading to target from visual features",
      blooms_level: "Apply",
      assessment_method: "Heading computation exercise"
    },
    {
      text: "Detect obstacles in navigation path",
      blooms_level: "Apply",
      assessment_method: "Obstacle detection exercise"
    },
    {
      text: "Implement bug algorithm for path planning",
      blooms_level: "Apply",
      assessment_method: "Path planning exercise"
    },
    {
      text: "Complete navigation challenge with 90%+ success",
      blooms_level: "Evaluate",
      assessment_method: "Integration exercise"
    }
  ]}
/>

## 📚 Prerequisites

<Prerequisites
  prerequisites={[
    {
      lessonId: "chapter-03-lesson-03",
      title: "Lesson 3.3: Object Detection",
      link: "/docs/chapter-03/lesson-03-object-detection"
    },
    {
      lessonId: "chapter-03-lesson-04",
      title: "Lesson 3.4: Depth Perception",
      link: "/docs/chapter-03/lesson-04-depth-perception"
    }
  ]}
/>

## Introduction

This lesson integrates everything: detect goals, avoid obstacles, plan paths.

**Time**: 70 minutes

---

## 1. Visual Servoing

**Visual servoing**: Control robot using visual feedback.

---

## 2. Bug Algorithm

**Bug algorithm**: Simple reactive navigation:
1. Move toward goal
2. If obstacle, follow boundary
3. When clear path to goal, resume direct approach

---

## 3. Exercises

### Exercise 3.5.1: Compute Heading to Target

<InteractivePython
  id="ex-3-5-1"
  title="Visual Heading Estimation"
  starterCode={`import numpy as np

def compute_heading_to_target(image, target_color_range):
    """Compute heading angle to colored target."""
    # TODO: Segment target by color
    # Compute centroid
    # Return heading angle (left/right of center)
    pass

# Test
img = np.zeros((100, 200, 3), dtype=np.uint8)
img[40:60, 150:170] = [255, 0, 0]  # Red target on right

heading = compute_heading_to_target(img, ([200,0,0], [255,50,50]))
print(f"Heading: {heading:.1f} degrees")
print(f"Expected: positive (turn right)")
`}
  hints={[
    "Segment by color",
    "Find centroid x-coordinate",
    "Center of image: img.shape[1] / 2",
    "Heading = (centroid_x - center_x) * scale_factor"
  ]}
/>

---

### Exercise 3.5.2: Detect Obstacles in Path

<InteractivePython
  id="ex-3-5-2"
  title="Path Obstacle Detection"
  starterCode={`import numpy as np

def obstacles_in_path(depth_image, safe_distance=1.5, path_width=0.5):
    """Check if path ahead is clear."""
    # TODO: Extract center region (path)
    # Check if any depth < safe_distance
    # Return boolean (True = blocked)
    pass

# Test
depth = np.ones((100, 100)) * 3.0
depth[40:60, 45:55] = 0.8  # Obstacle in center

blocked = obstacles_in_path(depth, safe_distance=1.5)
print(f"Path blocked: {blocked}")
print(f"Expected: True")
`}
  hints={[
    "height, width = depth_image.shape",
    "path_start = width//2 - path_width//2",
    "path_end = width//2 + path_width//2",
    "path_region = depth_image[:, path_start:path_end]",
    "return np.any(path_region < safe_distance)"
  ]}
/>

---

### Exercise 3.5.3: Bug Algorithm Planner

<InteractivePython
  id="ex-3-5-3"
  title="Bug Algorithm Navigation"
  starterCode={`import numpy as np

def bug_algorithm_step(robot_pos, goal_pos, occupancy_grid):
    """Compute next move using bug algorithm."""
    # TODO: If direct path clear, move toward goal
    # Else, follow obstacle boundary
    pass

# Test
grid = np.zeros((50, 50))
grid[20:30, 20:30] = 1  # Obstacle

robot = (10, 10)
goal = (40, 40)

next_pos = bug_algorithm_step(robot, goal, grid)
print(f"Next position: {next_pos}")
`}
  hints={[
    "Compute direction to goal",
    "Check if path clear using occupancy grid",
    "If clear: move toward goal",
    "Else: move along obstacle edge"
  ]}
/>

---

### Exercise 3.5.4: Complete Navigation Challenge

<InteractivePython
  id="ex-3-5-4"
  title="Navigation Challenge (90%+ Success)"
  starterCode={`import numpy as np
import matplotlib.pyplot as plt

def navigate_to_goal(start, goal, obstacles, max_steps=1000):
    """Navigate from start to goal avoiding obstacles.

    Returns:
        path: List of positions
        success: Boolean (reached goal)
    """
    # TODO: Integrate all vision techniques
    # 1. Detect obstacles
    # 2. Plan path
    # 3. Move step-by-step
    # Return path and success flag
    pass

# Test scenarios
scenarios = [
    {"start": (5, 5), "goal": (45, 45), "obstacles": []},
    {"start": (5, 5), "goal": (45, 45), "obstacles": [(25, 25, 10)]},  # (x, y, radius)
]

success_count = 0
for scenario in scenarios:
    path, success = navigate_to_goal(**scenario)
    if success:
        success_count += 1

print(f"Success rate: {success_count / len(scenarios) * 100:.1f}%")
print(f"Target: 90%+")
`}
  hints={[
    "Use bug algorithm for each step",
    "Track path",
    "Check goal reached: distance < threshold",
    "Return (path, True) if reached, (path, False) if max_steps exceeded"
  ]}
/>

---

## 4. Try With AI

### TryWithAI 3.5.1: Navigation Strategy

<TryWithAI
  id="tryai-3-5-1"
  title="Design Navigation Strategy"
  role="Copilot"
  scenario="Your bug algorithm gets stuck in local minima (U-shaped obstacles)."
  yourTask="Implement Exercise 3.5.3. Test with U-shaped obstacle. Observe failure."
  aiPromptTemplate="My bug algorithm navigation gets stuck in U-shaped obstacles. Here's my code: [paste]. Can you help me: (1) Understand why it fails? (2) Suggest improvements (random walk, potential fields)? (3) When should I use A* instead of bug algorithm?"
  successCriteria={[
    "You understand bug algorithm limitations",
    "You know alternatives (A*, potential fields, RRT)",
    "You can choose appropriate planner for scenario"
  ]}
  reflectionQuestions={[
    "What's the trade-off between simple (bug) and complex (A*) planners?",
    "How do you detect local minima?",
    "When is reactive vs. deliberative planning better?"
  ]}
/>

---

### TryWithAI 3.5.2: Test Edge Cases

<TryWithAI
  id="tryai-3-5-2"
  title="Validate Navigation Robustness"
  role="Evaluator"
  scenario="You need to test your navigator with edge cases before deployment."
  yourTask="Complete Exercise 3.5.4. Design 10 test scenarios (easy to impossible)."
  aiPromptTemplate="I've implemented vision-based navigation. Here's my code: [paste]. I achieve [X]% success on basic tests. Can you review and suggest edge case tests? I want to test: narrow passages, dense obstacles, unreachable goals, sensor noise. How do I systematically validate?"
  successCriteria={[
    "You have comprehensive test suite",
    "You achieve 90%+ on realistic scenarios",
    "You understand failure modes"
  ]}
  reflectionQuestions={[
    "What constitutes a 'fair' test scenario?",
    "How do you balance safety vs. efficiency?",
    "When should the robot admit failure?"
  ]}
/>

---

## Summary

**Key Takeaways**:

1. **Visual Servoing**: Control using visual feedback
2. **Bug Algorithm**: Simple reactive navigation
3. **Integration**: Combine vision, planning, control

**🎉 Chapter 3 Complete!** You can now implement vision-based navigation with obstacle detection.

**What's Next**: Take the [Chapter 3 Quiz](./quiz.md)!

---

**Estimated completion time**: 70 minutes | **Difficulty**: B2
