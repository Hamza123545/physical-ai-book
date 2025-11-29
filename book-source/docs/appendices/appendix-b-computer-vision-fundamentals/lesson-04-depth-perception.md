---
title: "Lesson 3.4: Depth Perception and Obstacles"
description: "Process depth images to detect obstacles and build occupancy grids"
chapter: 3
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
prerequisites:
  - "chapter-03-lesson-01"
has_interactive_python: true
interactive_python_count: 3
has_try_with_ai: true
try_with_ai_count: 1
tags: ["depth", "obstacles", "occupancy-grid", "3d-vision"]
---

# Lesson 3.4: Depth Perception and Obstacles

## 🎯 Learning Objectives

<LearningObjectives
  cefr_level="B2"
  objectives={[
    {
      text: "Process depth images to detect obstacles",
      blooms_level: "Apply",
      assessment_method: "Obstacle detection exercise"
    },
    {
      text: "Build occupancy grids from depth data",
      blooms_level: "Apply",
      assessment_method: "Occupancy grid exercise"
    },
    {
      text: "Identify free space for navigation",
      blooms_level: "Apply",
      assessment_method: "Free space detection exercise"
    }
  ]}
/>

## 📚 Prerequisites

<Prerequisites
  prerequisites={[
    {
      lessonId: "chapter-03-lesson-01",
      title: "Lesson 3.1: Image Representation",
      link: "/docs/chapter-03/lesson-01-image-representation"
    }
  ]}
/>

## Introduction

**Depth cameras** (like Kinect, RealSense) provide distance to each pixel. This enables:
- Obstacle detection
- 3D mapping
- Safe navigation

**Time**: 60 minutes

---

## 1. Depth Images

Depth image: 2D array where each pixel value = distance (in meters or millimeters).

```python
depth_image = np.array([
    [1.0, 1.5, 2.0],  # Row 1: distances
    [0.8, 1.2, 1.8],  # Row 2
])
```

---

## 2. Obstacle Detection

**Obstacles**: Pixels closer than safe distance threshold.

```python
obstacle_mask = depth_image < safe_distance
```

---

## 3. Occupancy Grids

**Occupancy grid**: Top-down map showing occupied/free cells.

---

## 4. Exercises

### Exercise 3.4.1: Detect Obstacles from Depth

<InteractivePython
  id="ex-3-4-1"
  title="Obstacle Detection"
  starterCode={`import numpy as np
import matplotlib.pyplot as plt

def detect_obstacles(depth_image, safe_distance=1.5):
    """Detect obstacles closer than safe_distance."""
    # TODO: Return binary mask of obstacles
    pass

# Test
depth = np.random.uniform(0.5, 3.0, (50, 50))
depth[20:30, 20:30] = 0.8  # Close obstacle

obstacles = detect_obstacles(depth, safe_distance=1.5)
plt.imshow(obstacles, cmap='gray')
plt.title('Obstacles (white = danger)')
plt.show()
`}
  hints={[
    "obstacles = (depth_image < safe_distance)",
    "Return obstacles.astype(np.uint8) * 255"
  ]}
/>

---

### Exercise 3.4.2: Build Occupancy Grid

<InteractivePython
  id="ex-3-4-2"
  title="Occupancy Grid from Depth"
  starterCode={`import numpy as np

def depth_to_occupancy_grid(depth_image, cell_size=0.1, max_range=5.0):
    """Convert depth image to occupancy grid."""
    # TODO: Project depth pixels to grid coordinates
    # Mark occupied cells
    pass

# Test
depth = np.random.uniform(1.0, 4.0, (100, 100))
grid = depth_to_occupancy_grid(depth, cell_size=0.2)
print(f"Grid shape: {grid.shape}")
`}
  hints={[
    "grid_size = int(max_range / cell_size)",
    "grid = np.zeros((grid_size, grid_size))",
    "For each depth pixel: compute grid cell (x, y)",
    "grid[y, x] = 1  # occupied"
  ]}
/>

---

### Exercise 3.4.3: Find Free Space

<InteractivePython
  id="ex-3-4-3"
  title="Free Space Detection"
  starterCode={`import numpy as np

def find_free_space(occupancy_grid):
    """Find largest free space region."""
    # TODO: Identify connected free regions
    # Return largest region
    pass

# Test
grid = np.random.randint(0, 2, (50, 50))
free = find_free_space(grid)
print(f"Largest free region size: {np.sum(free)}")
`}
  hints={[
    "Free cells: grid == 0",
    "Use connected component labeling",
    "Find component with most pixels"
  ]}
/>

---

## 5. Try With AI

### TryWithAI 3.4.1: Obstacle Detection Optimization

<TryWithAI
  id="tryai-3-4-1"
  title="Optimize Depth Processing"
  role="Copilot"
  scenario="Your depth processing is too slow for real-time navigation (need 30 FPS)."
  yourTask="Profile exercises 3.4.1 and 3.4.2. Measure execution time for 640x480 depth images."
  aiPromptTemplate="My depth obstacle detection takes [X] ms for 640x480 images. I need < 33ms for 30 FPS. Here's my code: [paste]. Can you help optimize? Should I downsample? Use GPU? What's the computational bottleneck?"
  successCriteria={[
    "You can profile Python code",
    "You understand downsampling trade-offs",
    "Your code runs in real-time"
  ]}
  reflectionQuestions={[
    "How does resolution affect obstacle detection accuracy?",
    "When should you use GPU acceleration?",
    "What's the minimum viable resolution for navigation?"
  ]}
/>

---

## Summary

**Key Takeaways**:

1. **Depth Images**: Distance to each pixel
2. **Obstacles**: Pixels closer than threshold
3. **Occupancy Grids**: Top-down map of environment

**What's Next**: [Lesson 3.5: Vision-Based Navigation](./lesson-05-vision-navigation.md)

---

**Estimated completion time**: 60 minutes | **Difficulty**: B2
