---
title: "Lesson 2.3: Forward Kinematics"
description: "Compute robot end-effector position from joint angles using transformation chains"
chapter: 2
lesson: 3
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
prerequisites:
  - "chapter-02-lesson-02"
has_interactive_python: true
interactive_python_count: 4
has_try_with_ai: true
try_with_ai_count: 2
tags: ["forward-kinematics", "robot-arms", "joint-space", "task-space"]
---

# Lesson 2.3: Forward Kinematics

## 🎯 Learning Objectives

<LearningObjectives
  cefr_level="B1+"
  objectives={[
    {
      text: "Implement forward kinematics for 2-DOF and 3-DOF planar robot arms",
      blooms_level: "Apply",
      assessment_method: "Python exercises with test cases"
    },
    {
      text: "Visualize robot arm configurations using Matplotlib",
      blooms_level: "Apply",
      assessment_method: "Visualization exercise"
    },
    {
      text: "Understand workspace and joint limits",
      blooms_level: "Understand",
      assessment_method: "Quiz and Try With AI"
    }
  ]}
/>

## 📚 Prerequisites

<Prerequisites
  prerequisites={[
    {
      lessonId: "chapter-02-lesson-02",
      title: "Lesson 2.2: Homogeneous Transformations",
      link: "/docs/chapter-02/lesson-02-homogeneous-transforms"
    }
  ]}
/>

## Introduction

**Forward Kinematics (FK)** answers the question:

> **"Given the joint angles, where is the end-effector?"**

For a robot arm:
- **Input**: Joint angles $\theta_1, \theta_2, \ldots, \theta_n$
- **Output**: End-effector position $(x, y, z)$ and orientation

FK is the foundation of robot control. Once you know where the robot is, you can plan how to move it.

**Time**: 70 minutes

---

## 1. Joint Space vs. Task Space

### Joint Space
**Joint space** describes the robot using joint angles/positions.

**Example** (2-DOF arm):
- Joint 1: $\theta_1 = 30°$
- Joint 2: $\theta_2 = 45°$

### Task Space
**Task space** (or **Cartesian space**) describes the end-effector position in world coordinates.

**Example**:
- End-effector at $(x, y) = (1.2, 0.8)$ meters

### Forward Kinematics Maps Joint → Task

$$
\text{FK}: \text{Joint Space} \to \text{Task Space}
$$

$$
(x, y, z) = \text{FK}(\theta_1, \theta_2, \ldots, \theta_n)
$$

---

## 2. 2-DOF Planar Arm

### Arm Geometry

Consider a 2-DOF planar robot arm:
- **Link 1**: Length $L_1$, rotates by $\theta_1$
- **Link 2**: Length $L_2$, rotates by $\theta_2$

### Forward Kinematics Formula

Using trigonometry:

$$
x = L_1 \cos(\theta_1) + L_2 \cos(\theta_1 + \theta_2)
$$

$$
y = L_1 \sin(\theta_1) + L_2 \sin(\theta_1 + \theta_2)
$$

**Derivation**:
1. Joint 1 moves end of link 1 to $(L_1 \cos\theta_1, L_1 \sin\theta_1)$
2. Joint 2 rotates link 2 by total angle $\theta_1 + \theta_2$
3. Add link 2's contribution: $L_2 \cos(\theta_1 + \theta_2), L_2 \sin(\theta_1 + \theta_2)$

### Example Calculation

**Given**: $L_1 = 1.0$m, $L_2 = 0.8$m, $\theta_1 = 30°$, $\theta_2 = 45°$

$$
x = 1.0 \cos(30°) + 0.8 \cos(75°) = 0.866 + 0.207 = 1.073 \text{ m}
$$

$$
y = 1.0 \sin(30°) + 0.8 \sin(75°) = 0.5 + 0.773 = 1.273 \text{ m}
$$

---

## 3. Using Transformation Matrices

### Transformation Chain Approach

We can also compute FK using homogeneous transforms:

1. **Transform 1** ($T_0^1$): Rotate by $\theta_1$, translate by $L_1$
2. **Transform 2** ($T_1^2$): Rotate by $\theta_2$, translate by $L_2$
3. **Chain**: $T_0^2 = T_0^1 \cdot T_1^2$

Extract end-effector position from $T_0^2$.

---

## 4. 3-DOF Arm

For a 3-DOF planar arm:

$$
x = L_1 \cos\theta_1 + L_2 \cos(\theta_1 + \theta_2) + L_3 \cos(\theta_1 + \theta_2 + \theta_3)
$$

$$
y = L_1 \sin\theta_1 + L_2 \sin(\theta_1 + \theta_2) + L_3 \sin(\theta_1 + \theta_2 + \theta_3)
$$

**Pattern**: Each link adds its contribution rotated by the cumulative angle.

---

## 5. Workspace

The **workspace** is the set of all positions the end-effector can reach.

**For 2-DOF planar arm**:
- **Outer radius**: $L_1 + L_2$ (arm fully extended)
- **Inner radius**: $|L_1 - L_2|$ (arm folded)
- **Workspace**: Annulus (ring shape)

**Edge cases**:
- Cannot reach inside inner circle
- Cannot reach beyond outer circle

---

## 6. Exercises

### Exercise 2.3.1: 2-DOF Planar Arm FK

Implement forward kinematics for a 2-link planar arm.

<InteractivePython
  id="ex-2-3-1"
  title="2-DOF Arm Forward Kinematics"
  starterCode={`import numpy as np

def fk_2dof(theta1_deg, theta2_deg, L1=1.0, L2=0.8):
    """
    Forward kinematics for 2-DOF planar arm.

    Args:
        theta1_deg: Joint 1 angle (degrees)
        theta2_deg: Joint 2 angle (degrees)
        L1: Link 1 length (meters)
        L2: Link 2 length (meters)

    Returns:
        (x, y) end-effector position
    """
    # TODO: Implement FK
    # 1. Convert angles to radians
    # 2. Calculate x using formula
    # 3. Calculate y using formula
    # 4. Return (x, y)
    pass

# Test case 1: Arm straight (0°, 0°)
x, y = fk_2dof(0, 0)
print(f"Joints (0°, 0°): ({x:.3f}, {y:.3f})")
print(f"Expected: ({1.8:.3f}, {0:.3f})")
print()

# Test case 2: L-shape (90°, 0°)
x, y = fk_2dof(90, 0)
print(f"Joints (90°, 0°): ({x:.3f}, {y:.3f})")
print(f"Expected: ({0:.3f}, {1.8:.3f})")
print()

# Test case 3: Folded (-90°, 90°) - should give (L1-L2, 0)
x, y = fk_2dof(0, 180)
print(f"Joints (0°, 180°): ({x:.3f}, {y:.3f})")
print(f"Expected: ({0.2:.3f}, {0:.3f})")
`}
  hints={[
    "theta1_rad = np.deg2rad(theta1_deg)",
    "theta2_rad = np.deg2rad(theta2_deg)",
    "x = L1 * np.cos(theta1_rad) + L2 * np.cos(theta1_rad + theta2_rad)",
    "y = L1 * np.sin(theta1_rad) + L2 * np.sin(theta1_rad + theta2_rad)"
  ]}
/>

---

### Exercise 2.3.2: 3-DOF Planar Arm FK

Extend FK to 3 degrees of freedom.

<InteractivePython
  id="ex-2-3-2"
  title="3-DOF Arm Forward Kinematics"
  starterCode={`import numpy as np

def fk_3dof(theta1_deg, theta2_deg, theta3_deg, L1=1.0, L2=0.8, L3=0.6):
    """
    Forward kinematics for 3-DOF planar arm.

    Args:
        theta1_deg, theta2_deg, theta3_deg: Joint angles (degrees)
        L1, L2, L3: Link lengths (meters)

    Returns:
        (x, y) end-effector position
    """
    # TODO: Implement 3-DOF FK
    # 1. Convert all angles to radians
    # 2. Compute cumulative angles:
    #    - Link 1: theta1
    #    - Link 2: theta1 + theta2
    #    - Link 3: theta1 + theta2 + theta3
    # 3. Sum contributions from all 3 links
    pass

# Test: All joints at 0° (straight arm)
x, y = fk_3dof(0, 0, 0)
print(f"All joints 0°: ({x:.3f}, {y:.3f})")
print(f"Expected: ({2.4:.3f}, {0:.3f})")  # L1 + L2 + L3
print()

# Test: Create S-shape (45°, -90°, 45°)
x, y = fk_3dof(45, -90, 45)
print(f"S-shape (45°, -90°, 45°): ({x:.3f}, {y:.3f})")
`}
  hints={[
    "Convert all angles: theta1_rad, theta2_rad, theta3_rad",
    "cumulative_1 = theta1_rad",
    "cumulative_2 = theta1_rad + theta2_rad",
    "cumulative_3 = theta1_rad + theta2_rad + theta3_rad",
    "x = L1*cos(cumulative_1) + L2*cos(cumulative_2) + L3*cos(cumulative_3)"
  ]}
/>

---

### Exercise 2.3.3: Visualize Robot Arm

Visualize the robot arm configuration.

<InteractivePython
  id="ex-2-3-3"
  title="Visualize 2-DOF Robot Arm"
  starterCode={`import numpy as np
import matplotlib.pyplot as plt

def fk_2dof(theta1_deg, theta2_deg, L1=1.0, L2=0.8):
    """2-DOF FK (from previous exercise)."""
    theta1 = np.deg2rad(theta1_deg)
    theta2 = np.deg2rad(theta2_deg)
    x = L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2)
    y = L1 * np.sin(theta1) + L2 * np.sin(theta1 + theta2)
    return x, y

def plot_arm(theta1_deg, theta2_deg, L1=1.0, L2=0.8):
    """
    Plot 2-DOF robot arm configuration.

    Args:
        theta1_deg, theta2_deg: Joint angles (degrees)
        L1, L2: Link lengths
    """
    # TODO: Implement arm visualization
    # 1. Compute joint positions:
    #    - Base: (0, 0)
    #    - Joint 1: (L1*cos(theta1), L1*sin(theta1))
    #    - End-effector: fk_2dof(theta1, theta2)
    # 2. Plot links as lines connecting joints
    # 3. Plot joints as circles
    # 4. Add workspace circle (outer radius = L1 + L2)
    pass

# Visualize different configurations
fig, axes = plt.subplots(1, 3, figsize=(15, 5))

configs = [
    (0, 0, "Straight"),
    (45, 45, "Bent forward"),
    (90, -90, "Folded")
]

for ax, (theta1, theta2, title) in zip(axes, configs):
    plt.sca(ax)
    plot_arm(theta1, theta2)
    ax.set_title(f"{title}: θ1={theta1}°, θ2={theta2}°")
    ax.set_xlim(-2, 2)
    ax.set_ylim(-2, 2)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.show()
`}
  hints={[
    "theta1_rad = np.deg2rad(theta1_deg)",
    "joint1_pos = [L1*np.cos(theta1_rad), L1*np.sin(theta1_rad)]",
    "end_pos = list(fk_2dof(theta1_deg, theta2_deg, L1, L2))",
    "plt.plot([0, joint1_pos[0]], [0, joint1_pos[1]], 'b-', linewidth=3)",
    "plt.plot([joint1_pos[0], end_pos[0]], [joint1_pos[1], end_pos[1]], 'r-', linewidth=3)",
    "Draw workspace circle: theta = np.linspace(0, 2*np.pi, 100); plt.plot((L1+L2)*np.cos(theta), (L1+L2)*np.sin(theta), 'g--')"
  ]}
/>

---

### Exercise 2.3.4: FK Using Transformation Matrices

Implement FK using homogeneous transformation matrices (3D version).

<InteractivePython
  id="ex-2-3-4"
  title="FK with Transformation Matrices"
  starterCode={`import numpy as np

def transform_z(theta_deg, tx, ty=0, tz=0):
    """
    Create 4x4 transformation: rotate around z, then translate.

    Args:
        theta_deg: Rotation around z-axis (degrees)
        tx, ty, tz: Translation

    Returns:
        4x4 transformation matrix
    """
    theta = np.deg2rad(theta_deg)
    c = np.cos(theta)
    s = np.sin(theta)

    T = np.eye(4)
    T[:3, :3] = [[c, -s, 0],
                  [s,  c, 0],
                  [0,  0, 1]]
    T[:3, 3] = [tx, ty, tz]
    return T

def fk_2dof_matrix(theta1_deg, theta2_deg, L1=1.0, L2=0.8):
    """
    2-DOF FK using transformation matrices.

    Args:
        theta1_deg, theta2_deg: Joint angles
        L1, L2: Link lengths

    Returns:
        (x, y, z) end-effector position
    """
    # TODO: Implement FK using matrices
    # 1. Create T1: rotate theta1, translate L1 along new x-axis
    # 2. Create T2: rotate theta2, translate L2 along new x-axis
    # 3. Chain: T_total = T1 @ T2
    # 4. Extract position from T_total[:3, 3]
    pass

# Test
x, y, z = fk_2dof_matrix(45, 45)
print(f"FK via matrices (45°, 45°): ({x:.3f}, {y:.3f}, {z:.3f})")

# Compare with direct FK
def fk_2dof_direct(theta1_deg, theta2_deg, L1=1.0, L2=0.8):
    theta1 = np.deg2rad(theta1_deg)
    theta2 = np.deg2rad(theta2_deg)
    x = L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2)
    y = L1 * np.sin(theta1) + L2 * np.sin(theta1 + theta2)
    return x, y

x_direct, y_direct = fk_2dof_direct(45, 45)
print(f"FK via formula (45°, 45°): ({x_direct:.3f}, {y_direct:.3f})")
print(f"Match: {np.allclose([x, y], [x_direct, y_direct])}")
`}
  hints={[
    "T1 = transform_z(theta1_deg, L1, 0, 0)",
    "T2 = transform_z(theta2_deg, L2, 0, 0)",
    "T_total = T1 @ T2",
    "position = T_total[:3, 3]",
    "return position[0], position[1], position[2]"
  ]}
/>

---

## 7. Try With AI

### TryWithAI 2.3.1: Generalize FK to N-DOF

<TryWithAI
  id="tryai-2-3-1"
  title="Generalize FK to N Joints"
  role="Copilot"
  scenario="You want to write a general FK function that works for any number of joints (N-DOF), not just 2 or 3."
  yourTask="Complete exercises 2.3.1 and 2.3.2. Notice the pattern. Try to write a function fk_ndof(thetas, lengths) that takes lists of joint angles and link lengths."
  aiPromptTemplate="I've implemented FK for 2-DOF and 3-DOF arms. I see a pattern: each link contributes L_i * cos(cumulative_angle) to x and L_i * sin(cumulative_angle) to y. Can you help me write a general N-DOF FK function that takes: (1) A list of joint angles [θ1, θ2, ..., θn], (2) A list of link lengths [L1, L2, ..., Ln], and returns (x, y)? Also explain how to compute cumulative angles efficiently."
  successCriteria={[
    "You can implement FK for arbitrary number of joints",
    "You use NumPy cumsum() for efficient cumulative angle computation",
    "Your function handles edge cases (empty list, single joint)"
  ]}
  reflectionQuestions={[
    "How would you extend this to 3D (spatial) arms?",
    "What's the computational complexity of N-DOF FK?",
    "How would you vectorize this for multiple configurations?"
  ]}
/>

---

### TryWithAI 2.3.2: Workspace Validation

<TryWithAI
  id="tryai-2-3-2"
  title="Test FK with Edge Cases"
  role="Evaluator"
  scenario="You want comprehensive test cases for your FK implementation to ensure correctness."
  yourTask="Write test cases for FK that cover: (1) Straight configuration, (2) Fully extended, (3) Fully folded, (4) Workspace boundaries. Run your FK and verify the results."
  aiPromptTemplate="I've implemented 2-DOF FK. Here's my code: [paste]. Can you review my test cases and suggest additional edge cases I should test? I want to ensure FK is correct for all configurations, especially: workspace boundaries, singularities, and extreme joint angles (±180°). Also suggest how to verify FK correctness mathematically."
  successCriteria={[
    "You have test cases for at least 5 different configurations",
    "You verify FK against known geometric solutions",
    "You understand workspace limits (min/max reach)"
  ]}
  reflectionQuestions={[
    "How can you visualize the entire workspace?",
    "What happens at joint limits (e.g., ±180°)?",
    "How would you unit-test FK in a real robotics project?"
  ]}
/>

---

## Summary

**Key Takeaways**:

1. **Forward Kinematics**:
   - Input: Joint angles $(\theta_1, \theta_2, \ldots)$
   - Output: End-effector position $(x, y, z)$
   - Maps joint space → task space

2. **2-DOF Planar Arm**:
   $$
   x = L_1 \cos\theta_1 + L_2 \cos(\theta_1 + \theta_2)
   $$
   $$
   y = L_1 \sin\theta_1 + L_2 \sin(\theta_1 + \theta_2)
   $$

3. **Transformation Matrix Approach**:
   - Build transformation for each joint
   - Chain transformations: $T_{total} = T_1 T_2 \cdots T_n$
   - Extract position from final matrix

4. **Workspace**:
   - Set of all reachable positions
   - 2-DOF arm: annulus with inner radius $|L_1 - L_2|$, outer radius $L_1 + L_2$
   - Cannot reach outside workspace

5. **Implementation**:
   - Convert angles to radians
   - Compute cumulative angles for each link
   - Sum trigonometric contributions
   - Visualize with Matplotlib

**What's Next**: [Lesson 2.4: Inverse Kinematics](./lesson-04-inverse-kinematics.md) solves the reverse problem: given a target position, find the joint angles needed to reach it.

---

**Estimated completion time**: 70 minutes | **Prerequisites**: Lesson 2.2 | **Difficulty**: B1+ (Upper Intermediate)
