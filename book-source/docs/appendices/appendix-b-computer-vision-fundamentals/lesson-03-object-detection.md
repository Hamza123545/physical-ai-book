---
title: "Lesson 3.3: Object Detection and Tracking"
description: "Segment, detect, and track objects using color-based methods and centroids"
chapter: 3
lesson: 3
estimated_time: 70
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
interactive_python_count: 4
has_try_with_ai: true
try_with_ai_count: 2
tags: ["object-detection", "color-segmentation", "centroids", "tracking"]
---

# Lesson 3.3: Object Detection and Tracking

## 🎯 Learning Objectives

<LearningObjectives
  cefr_level="B2"
  objectives={[
    {
      text: "Segment objects using color-based thresholds",
      blooms_level: "Apply",
      assessment_method: "Color segmentation exercise"
    },
    {
      text: "Compute object centroids for position estimation",
      blooms_level: "Apply",
      assessment_method: "Centroid calculation exercise"
    },
    {
      text: "Track objects across video frames",
      blooms_level: "Apply",
      assessment_method: "Object tracking exercise"
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

Robots need to **find** and **track** objects to manipulate them or avoid them. This lesson covers:
- Color-based segmentation to isolate objects
- Computing object centroids (center of mass)
- Tracking objects over time

**Time**: 70 minutes

---

## 1. Color-Based Segmentation

**Idea**: Objects often have distinct colors. Segment by filtering color ranges.

```python
# Find red pixels
red_mask = (image[:,:,0] > 200) & (image[:,:,1] < 50) & (image[:,:,2] < 50)
```

---

## 2. Centroid Computation

**Centroid** = center of mass of an object:

$$
x_c = \frac{\sum x_i}{N}, \quad y_c = \frac{\sum y_i}{N}
$$

Where $(x_i, y_i)$ are object pixels.

---

## 3. Object Tracking

**Simple tracking**: Match centroids between frames (nearest neighbor).

---

## 4. Exercises

### Exercise 3.3.1: Color-Based Segmentation

<InteractivePython
  id="ex-3-3-1"
  title="Color Segmentation"
  starterCode={`import numpy as np
import matplotlib.pyplot as plt

def segment_by_color(image, color_min, color_max):
    """Segment objects by color range."""
    # TODO: Create mask where all channels are in range
    pass

# Test
img = np.zeros((100, 100, 3), dtype=np.uint8)
img[20:40, 20:40] = [255, 0, 0]  # Red square

mask = segment_by_color(img, [200, 0, 0], [255, 50, 50])
plt.imshow(mask, cmap='gray')
plt.show()
`}
  hints={[
    "mask = np.all((image >= color_min) & (image <= color_max), axis=2)",
    "Return mask.astype(np.uint8) * 255"
  ]}
/>

---

### Exercise 3.3.2: Compute Centroid

<InteractivePython
  id="ex-3-3-2"
  title="Object Centroid"
  starterCode={`import numpy as np

def compute_centroid(binary_mask):
    """Compute centroid of binary object."""
    # TODO: Find y, x coordinates of True pixels
    # Return mean(x), mean(y)
    pass

# Test
mask = np.zeros((100, 100), dtype=bool)
mask[40:60, 30:50] = True

cx, cy = compute_centroid(mask)
print(f"Centroid: ({cx:.1f}, {cy:.1f})")
print(f"Expected: (40.0, 50.0)")
`}
  hints={[
    "y_coords, x_coords = np.where(binary_mask)",
    "cx = np.mean(x_coords)",
    "cy = np.mean(y_coords)",
    "return cx, cy"
  ]}
/>

---

### Exercise 3.3.3: Object Orientation

<InteractivePython
  id="ex-3-3-3"
  title="Object Orientation Angle"
  starterCode={`import numpy as np

def compute_orientation(binary_mask):
    """Compute object orientation using moments."""
    # TODO: Compute second moments and orientation
    pass

# Test
mask = np.zeros((100, 100), dtype=bool)
for i in range(30, 70):
    mask[i, i] = True  # Diagonal line

angle = compute_orientation(mask)
print(f"Orientation: {angle:.1f} degrees")
print(f"Expected: ~45 degrees")
`}
  hints={[
    "y, x = np.where(binary_mask)",
    "cx, cy = np.mean(x), np.mean(y)",
    "x_centered = x - cx",
    "y_centered = y - cy",
    "angle = np.arctan2(y_centered.sum(), x_centered.sum())",
    "return np.degrees(angle)"
  ]}
/>

---

### Exercise 3.3.4: Simple Object Tracking

<InteractivePython
  id="ex-3-3-4"
  title="Track Object Across Frames"
  starterCode={`import numpy as np

def track_object(centroids_frame1, centroids_frame2, max_distance=50):
    """Match objects between frames using nearest neighbor."""
    # TODO: For each centroid in frame1, find nearest in frame2
    # Return list of matches: [(idx1, idx2), ...]
    pass

# Test
frame1_centroids = [(50, 50), (100, 100)]
frame2_centroids = [(55, 52), (102, 98)]  # Slightly moved

matches = track_object(frame1_centroids, frame2_centroids)
print(f"Matches: {matches}")
print(f"Expected: [(0, 0), (1, 1)]")
`}
  hints={[
    "For each c1 in centroids_frame1:",
    "  distances = [np.linalg.norm(np.array(c1) - np.array(c2)) for c2 in centroids_frame2]",
    "  nearest_idx = np.argmin(distances)",
    "  if distances[nearest_idx] < max_distance:",
    "    matches.append((i, nearest_idx))"
  ]}
/>

---

## 5. Try With AI

### TryWithAI 3.3.1: Object Detection Implementation

<TryWithAI
  id="tryai-3-3-1"
  title="Robust Color Segmentation"
  role="Copilot"
  scenario="Your color-based object detector fails under different lighting conditions."
  yourTask="Implement Exercise 3.3.1. Test with images at different brightnesses. Notice failures."
  aiPromptTemplate="My color segmentation works in good lighting but fails when lighting changes. Here's my code: [paste]. Can you help me make it more robust? Should I use HSV color space instead of RGB? How do I handle shadows and highlights?"
  successCriteria={[
    "You understand RGB vs HSV for color segmentation",
    "You can handle lighting variations",
    "You know when color-based methods fail"
  ]}
  reflectionQuestions={[
    "When should you use HSV instead of RGB?",
    "How do you handle objects with multiple colors?",
    "What are alternatives to color-based detection?"
  ]}
/>

---

### TryWithAI 3.3.2: Tracking Robustness

<TryWithAI
  id="tryai-3-3-2"
  title="Improve Object Tracking"
  role="Evaluator"
  scenario="Your tracker loses objects when they move quickly or overlap."
  yourTask="Complete Exercise 3.3.4. Think about failure cases: fast motion, occlusion, multiple objects."
  aiPromptTemplate="My object tracker uses nearest-neighbor matching. Here's my code: [paste]. It fails when: (1) Objects move > 50 pixels between frames, (2) Objects overlap. Can you review and suggest improvements? Should I use Kalman filtering? How do I handle occlusion?"
  successCriteria={[
    "You understand limitations of nearest-neighbor tracking",
    "You know about prediction-based tracking (Kalman filter)",
    "You can handle edge cases (occlusion, fast motion)"
  ]}
  reflectionQuestions={[
    "How would you track multiple objects of the same color?",
    "What's the role of prediction in tracking?",
    "When should you re-initialize tracking?"
  ]}
/>

---

## Summary

**Key Takeaways**:

1. **Color Segmentation**: Filter by RGB/HSV ranges to isolate objects
2. **Centroids**: Compute center of mass for object position
3. **Tracking**: Match objects between frames using nearest neighbor

**What's Next**: [Lesson 3.4: Depth Perception](./lesson-04-depth-perception.md)

---

**Estimated completion time**: 70 minutes | **Difficulty**: B2
