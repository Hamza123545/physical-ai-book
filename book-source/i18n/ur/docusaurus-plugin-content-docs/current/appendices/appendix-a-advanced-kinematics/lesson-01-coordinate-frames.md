---
title: "Lesson 2.1: Coordinate Frames and Rotations"
description: "Understand how robots represent position and orientation using coordinate frames and rotation matrices"
chapter: 2
lesson: 1
estimated_time: 50
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
prerequisites:
  - "chapter-01-lesson-04"
has_interactive_python: true
interactive_python_count: 3
has_try_with_ai: true
try_with_ai_count: 1
tags: ["coordinate-frames", "rotation-matrices", "transformations", "2d-geometry"]
---

# Lesson 2.1: Coordinate Frames and Rotations

## 🎯 Learning Objectives

<LearningObjectives
  cefr_level="B1"
  objectives={[
    {
      text: "Understand coordinate frames and their role in robot kinematics",
      blooms_level: "Understand",
      assessment_method: "Quiz and exercises"
    },
    {
      text: "Implement 2D rotation matrices in NumPy",
      blooms_level: "Apply",
      assessment_method: "Python exercises"
    },
    {
      text: "Visualize multiple coordinate frames with Matplotlib",
      blooms_level: "Apply",
      assessment_method: "Visualization exercise"
    }
  ]}
/>

## 📚 Prerequisites

<Prerequisites
  prerequisites={[
    {
      lessonId: "chapter-01-lesson-04",
      title: "Lesson 1.4: Python for Robotics Introduction",
      link: "/docs/chapter-01/lesson-04-python-robotics-intro"
    }
  ]}
/>

## Introduction

When programming a robot arm, you need to answer questions like:
- "Where is the robot's hand in 3D space?"
- "How do I rotate the gripper to grasp an object?"
- "What's the position of the object relative to the robot?"

**Coordinate frames** are the fundamental tool for answering these questions. A coordinate frame defines a reference point (origin) and a set of axes (x, y, z) for measuring positions and orientations.

In this lesson, you'll learn how robots use multiple coordinate frames and how to transform between them using rotation matrices.

**Time**: 50 minutes

---

## 1. What is a Coordinate Frame?

### Definition

A **coordinate frame** (or reference frame) is a coordinate system with:
- **Origin**: A reference point (0, 0) in 2D or (0, 0, 0) in 3D
- **Axes**: Directions for measuring position (x, y, z)
- **Orientation**: How the axes are rotated

### Why Multiple Frames?

Robots use many coordinate frames:

| Frame Name | Description | Use Case |
|------------|-------------|----------|
| **World Frame** | Fixed global reference | Maps, absolute positions |
| **Base Frame** | Robot's mounting location | Robot positioning |
| **Tool Frame** | Gripper/end-effector | Grasp planning |
| **Object Frame** | Target object | Pick-and-place tasks |

**Example**: To pick up a cup:
1. Cup position in world frame: (1.0, 0.5, 0.8) meters
2. Convert to base frame: (0.5, 0.3, 0.2) relative to robot
3. Plan gripper approach in tool frame: (-0.1, 0, 0) approach from above

---

## 2. 2D Rotation Matrices

### The Rotation Problem

**Question**: If a point is at $(x, y) = (1, 0)$ in frame A, where is it after rotating frame A by 45° counterclockwise?

**Answer**: We use a **rotation matrix**.

### 2D Rotation Matrix

To rotate a point by angle $\theta$ counterclockwise:

$$
R(\theta) = \begin{bmatrix}
\cos\theta & -\sin\theta \\
\sin\theta & \cos\theta
\end{bmatrix}
$$

**Applying the rotation**:

$$
\begin{bmatrix} x' \\ y' \end{bmatrix} =
R(\theta) \begin{bmatrix} x \\ y \end{bmatrix} =
\begin{bmatrix}
\cos\theta & -\sin\theta \\
\sin\theta & \cos\theta
\end{bmatrix}
\begin{bmatrix} x \\ y \end{bmatrix}
$$

**Example**: Rotate point $(1, 0)$ by $90°$:

$$
R(90°) = \begin{bmatrix} 0 & -1 \\ 1 & 0 \end{bmatrix}
$$

$$
\begin{bmatrix} x' \\ y' \end{bmatrix} =
\begin{bmatrix} 0 & -1 \\ 1 & 0 \end{bmatrix}
\begin{bmatrix} 1 \\ 0 \end{bmatrix} =
\begin{bmatrix} 0 \\ 1 \end{bmatrix}
$$

The point moves from $(1, 0)$ to $(0, 1)$ ✓

### Properties of Rotation Matrices

1. **Orthogonal**: $R^T R = I$ (transpose equals inverse)
2. **Determinant**: $\det(R) = 1$ (preserves orientation)
3. **Preserves distance**: $\|R\vec{v}\| = \|\vec{v}\|$ (no scaling)

---

## 3. Right-Hand Rule

**Convention**: We use the **right-hand rule** for rotations.

**Right-hand rule**:
- Point thumb along rotation axis (z-axis for 2D)
- Fingers curl in **positive rotation direction**
- Positive rotation: counterclockwise when looking down the axis

**Example**:
- Rotate 45° → counterclockwise
- Rotate -45° → clockwise

---

## 4. Rotation in NumPy

### Creating a Rotation Matrix

```python
import numpy as np

def rotation_matrix_2d(theta_degrees):
    """
    Create 2D rotation matrix.

    Args:
        theta_degrees: Rotation angle in degrees

    Returns:
        2x2 rotation matrix
    """
    theta = np.deg2rad(theta_degrees)
    c = np.cos(theta)
    s = np.sin(theta)
    return np.array([
        [c, -s],
        [s,  c]
    ])

# Rotate 90 degrees
R = rotation_matrix_2d(90)
print(R)
# [[0. -1.]
#  [1.  0.]]
```

### Applying Rotation

```python
# Point at (1, 0)
point = np.array([1.0, 0.0])

# Rotate 90 degrees
R = rotation_matrix_2d(90)
rotated = R @ point  # Matrix multiplication

print(f"Original: {point}")
print(f"Rotated:  {rotated}")
# Rotated: [0. 1.]
```

---

## 5. Exercises

### Exercise 2.1.1: Implement 2D Rotation Matrix

Create a function that generates 2D rotation matrices.

<InteractivePython
  id="ex-2-1-1"
  title="2D Rotation Matrix"
  starterCode={`import numpy as np

def rotation_matrix_2d(theta_degrees):
    """
    Create 2D rotation matrix for counterclockwise rotation.

    Args:
        theta_degrees: Rotation angle in degrees

    Returns:
        2x2 NumPy array (rotation matrix)
    """
    # TODO: Implement rotation matrix
    # 1. Convert degrees to radians
    # 2. Compute cos(theta) and sin(theta)
    # 3. Build 2x2 matrix:
    #    [[cos, -sin],
    #     [sin,  cos]]
    pass

# Test cases
R90 = rotation_matrix_2d(90)
print("R(90°):")
print(R90)
print()

R45 = rotation_matrix_2d(45)
print("R(45°):")
print(R45)
print()

# Verify orthogonality: R.T @ R should be identity
print("Orthogonality check (R.T @ R):")
print(R90.T @ R90)
print("Should be close to:")
print(np.eye(2))
`}
  hints={[
    "Use np.deg2rad() to convert degrees to radians",
    "theta_rad = np.deg2rad(theta_degrees)",
    "c = np.cos(theta_rad), s = np.sin(theta_rad)",
    "Return np.array([[c, -s], [s, c]])"
  ]}
/>

---

### Exercise 2.1.2: Rotate a Point

Apply rotation matrices to transform points.

<InteractivePython
  id="ex-2-1-2"
  title="Rotate Points in 2D"
  starterCode={`import numpy as np

def rotation_matrix_2d(theta_degrees):
    """Create 2D rotation matrix."""
    theta = np.deg2rad(theta_degrees)
    c = np.cos(theta)
    s = np.sin(theta)
    return np.array([[c, -s], [s, c]])

def rotate_point(point, theta_degrees):
    """
    Rotate a 2D point by theta_degrees counterclockwise.

    Args:
        point: [x, y] NumPy array
        theta_degrees: Rotation angle in degrees

    Returns:
        Rotated [x', y'] point
    """
    # TODO: Implement point rotation
    # 1. Get rotation matrix R for theta_degrees
    # 2. Multiply: rotated = R @ point
    # 3. Return rotated point
    pass

# Test 1: Rotate (1, 0) by 90° should give (0, 1)
p1 = np.array([1.0, 0.0])
p1_rot = rotate_point(p1, 90)
print(f"Rotate (1, 0) by 90°: {p1_rot}")
print(f"Expected: [0, 1]")
print()

# Test 2: Rotate (1, 1) by 45°
p2 = np.array([1.0, 1.0])
p2_rot = rotate_point(p2, 45)
print(f"Rotate (1, 1) by 45°: {p2_rot}")
print(f"Expected: [0, 1.414...]")
print()

# Test 3: Rotate by 360° should give original point
p3 = np.array([2.0, 3.0])
p3_rot = rotate_point(p3, 360)
print(f"Rotate (2, 3) by 360°: {p3_rot}")
print(f"Expected: [2, 3]")
`}
  hints={[
    "R = rotation_matrix_2d(theta_degrees)",
    "rotated_point = R @ point",
    "Use @ operator for matrix multiplication",
    "Return the rotated point"
  ]}
/>

---

### Exercise 2.1.3: Visualize Coordinate Frames

Visualize multiple coordinate frames with Matplotlib.

<InteractivePython
  id="ex-2-1-3"
  title="Visualize Coordinate Frames"
  starterCode={`import numpy as np
import matplotlib.pyplot as plt

def rotation_matrix_2d(theta_degrees):
    """Create 2D rotation matrix."""
    theta = np.deg2rad(theta_degrees)
    c = np.cos(theta)
    s = np.sin(theta)
    return np.array([[c, -s], [s, c]])

def plot_frame(origin, rotation_degrees, length=1.0, label="Frame"):
    """
    Plot a 2D coordinate frame.

    Args:
        origin: [x, y] position of frame origin
        rotation_degrees: Frame rotation in degrees
        length: Length of axis arrows
        label: Frame label
    """
    # TODO: Implement frame visualization
    # 1. Get rotation matrix R
    # 2. Define x-axis: [length, 0], y-axis: [0, length]
    # 3. Rotate axes: x_axis = R @ [length, 0]
    #                  y_axis = R @ [0, length]
    # 4. Plot x-axis (red) from origin to origin + x_axis
    # 5. Plot y-axis (green) from origin to origin + y_axis
    # 6. Add origin marker and label

    pass

# Create visualization
plt.figure(figsize=(8, 8))

# Plot world frame (origin at (0,0), no rotation)
plot_frame([0, 0], 0, length=1.5, label="World")

# Plot robot base frame (origin at (2,1), rotated 30°)
plot_frame([2, 1], 30, length=1.0, label="Robot")

# Plot tool frame (origin at (3, 2.5), rotated 60°)
plot_frame([3, 2.5], 60, length=0.7, label="Tool")

plt.xlim(-1, 5)
plt.ylim(-1, 5)
plt.axis('equal')
plt.grid(True, alpha=0.3)
plt.legend()
plt.title('Multiple Coordinate Frames')
plt.xlabel('X (meters)')
plt.ylabel('Y (meters)')
plt.show()
`}
  hints={[
    "R = rotation_matrix_2d(rotation_degrees)",
    "x_axis = R @ np.array([length, 0])",
    "y_axis = R @ np.array([0, length])",
    "plt.arrow(origin[0], origin[1], x_axis[0], x_axis[1], color='r', head_width=0.1)",
    "plt.plot(origin[0], origin[1], 'ko', markersize=8)",
    "plt.text(origin[0], origin[1], label)"
  ]}
/>

---

## 6. Try With AI

### TryWithAI 2.1.1: Rotation Matrix Derivation

<TryWithAI
  id="tryai-2-1-1"
  title="Understand Rotation Matrix Derivation"
  role="Teacher"
  scenario="You want to understand WHY the rotation matrix has the form [[cos θ, -sin θ], [sin θ, cos θ]]."
  yourTask="Think about what happens when you rotate the unit vectors (1,0) and (0,1) by angle θ. Try to derive the rotation matrix formula yourself before asking AI."
  aiPromptTemplate="I'm trying to understand how the 2D rotation matrix formula is derived. I know that rotating (1, 0) by θ should give (cos θ, sin θ), and rotating (0, 1) by θ should give (-sin θ, cos θ). Can you explain step-by-step how this leads to the rotation matrix [[cos θ, -sin θ], [sin θ, cos θ]]? Also explain the geometric meaning of each entry."
  successCriteria={[
    "You understand that matrix columns are rotated basis vectors",
    "You can explain each entry of the rotation matrix geometrically",
    "You know why the top-right entry is -sin θ (not +sin θ)"
  ]}
  reflectionQuestions={[
    "What happens if you use the rotation matrix for clockwise rotation instead?",
    "How would you derive the 3D rotation matrix around the z-axis?",
    "Why is R^T = R^(-1) for rotation matrices?"
  ]}
/>

---

## Summary

**Key Takeaways**:

1. **Coordinate Frames**:
   - Define reference points and axes for position/orientation
   - Robots use multiple frames: world, base, tool, object
   - Frames have origin + axes + orientation

2. **2D Rotation Matrices**:
   - Rotate points counterclockwise: $R(\theta) = \begin{bmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{bmatrix}$
   - Properties: orthogonal, determinant = 1, preserves distance
   - Applied via matrix multiplication: $\vec{v}' = R\vec{v}$

3. **Right-Hand Rule**:
   - Thumb points along rotation axis
   - Fingers curl in positive rotation direction
   - Positive rotation = counterclockwise

4. **Implementation in NumPy**:
   - `np.deg2rad()` to convert degrees to radians
   - Create rotation matrix: `np.array([[c, -s], [s, c]])`
   - Apply rotation: `R @ point`
   - Visualize with Matplotlib arrows

**What's Next**: [Lesson 2.2: Homogeneous Transformations](./lesson-02-homogeneous-transforms.md) extends rotation matrices to 3D and adds translation (combined rotation + movement).

---

**Estimated completion time**: 50 minutes | **Prerequisites**: Lesson 1.4 | **Difficulty**: B1 (Intermediate)
