---
title: "Lesson 2.5: Jacobians and Velocities"
description: "Relate joint velocities to end-effector velocities using the Jacobian matrix"
chapter: 2
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
prerequisites:
  - "chapter-02-lesson-03"
  - "chapter-02-lesson-04"
has_interactive_python: true
interactive_python_count: 3
has_try_with_ai: true
try_with_ai_count: 1
tags: ["jacobian", "velocity-kinematics", "singularities", "differential-kinematics"]
---

# Lesson 2.5: Jacobians and Velocities

## 🎯 Learning Objectives

<LearningObjectives
  cefr_level="B2"
  objectives={[
    {
      text: "Compute the Jacobian matrix for a 2-DOF planar arm",
      blooms_level: "Apply",
      assessment_method: "Python exercise with numerical verification"
    },
    {
      text: "Use the Jacobian to compute end-effector velocity from joint velocities",
      blooms_level: "Apply",
      assessment_method: "Velocity calculation exercise"
    },
    {
      text: "Detect singularities by analyzing Jacobian determinant",
      blooms_level: "Analyze",
      assessment_method: "Singularity detection exercise"
    }
  ]}
/>

## 📚 Prerequisites

<Prerequisites
  prerequisites={[
    {
      lessonId: "chapter-02-lesson-03",
      title: "Lesson 2.3: Forward Kinematics",
      link: "/docs/chapter-02/lesson-03-forward-kinematics"
    },
    {
      lessonId: "chapter-02-lesson-04",
      title: "Lesson 2.4: Inverse Kinematics",
      link: "/docs/chapter-02/lesson-04-inverse-kinematics"
    }
  ]}
/>

## Introduction

So far, we've worked with **positions**:
- FK: joint angles → end-effector position
- IK: target position → joint angles

But robots move continuously! We need to understand **velocities**:

> **"If joints move at certain speeds, how fast does the end-effector move?"**

The **Jacobian matrix** ($J$) provides this relationship:

$$
\vec{v}_{end} = J \cdot \vec{\dot{\theta}}
$$

Where:
- $\vec{\dot{\theta}}$ = joint velocities $[\dot{\theta}_1, \dot{\theta}_2, \ldots]$
- $\vec{v}_{end}$ = end-effector velocity $[\dot{x}, \dot{y}, \ldots]$

**Applications**:
- **Velocity control**: Move end-effector at desired speed
- **Trajectory planning**: Smooth motion along paths
- **Singularity detection**: Identify configurations where robot loses DOF

**Time**: 60 minutes

---

## 1. What is the Jacobian?

### Differential Kinematics

The Jacobian relates **small changes** in joint angles to **small changes** in end-effector position:

$$
\Delta \vec{x} = J \cdot \Delta \vec{\theta}
$$

Taking the time derivative:

$$
\frac{d\vec{x}}{dt} = J \cdot \frac{d\vec{\theta}}{dt}
$$

$$
\vec{v}_{end} = J \cdot \vec{\dot{\theta}}
$$

### Jacobian Matrix Structure

For a 2-DOF planar arm:

$$
J = \begin{bmatrix}
\frac{\partial x}{\partial \theta_1} & \frac{\partial x}{\partial \theta_2} \\
\frac{\partial y}{\partial \theta_1} & \frac{\partial y}{\partial \theta_2}
\end{bmatrix}
$$

Each entry is a **partial derivative** of position with respect to joint angle.

---

## 2. Computing the Jacobian for 2-DOF Arm

### Forward Kinematics (Reminder)

$$
x = L_1 \cos\theta_1 + L_2 \cos(\theta_1 + \theta_2)
$$

$$
y = L_1 \sin\theta_1 + L_2 \sin(\theta_1 + \theta_2)
$$

### Partial Derivatives

$$
\frac{\partial x}{\partial \theta_1} = -L_1 \sin\theta_1 - L_2 \sin(\theta_1 + \theta_2)
$$

$$
\frac{\partial x}{\partial \theta_2} = -L_2 \sin(\theta_1 + \theta_2)
$$

$$
\frac{\partial y}{\partial \theta_1} = L_1 \cos\theta_1 + L_2 \cos(\theta_1 + \theta_2)
$$

$$
\frac{\partial y}{\partial \theta_2} = L_2 \cos(\theta_1 + \theta_2)
$$

### Jacobian Matrix

$$
J = \begin{bmatrix}
-L_1 \sin\theta_1 - L_2 \sin(\theta_1 + \theta_2) & -L_2 \sin(\theta_1 + \theta_2) \\
L_1 \cos\theta_1 + L_2 \cos(\theta_1 + \theta_2) & L_2 \cos(\theta_1 + \theta_2)
\end{bmatrix}
$$

---

## 3. Using the Jacobian

### Example: Compute End-Effector Velocity

**Given**:
- Joint velocities: $\dot{\theta}_1 = 10°/s$, $\dot{\theta}_2 = 5°/s$
- Current configuration: $\theta_1 = 30°$, $\theta_2 = 45°$
- Link lengths: $L_1 = 1.0$m, $L_2 = 0.8$m

**Steps**:
1. Compute Jacobian $J$ at current configuration
2. Apply: $\vec{v}_{end} = J \cdot \vec{\dot{\theta}}$

---

## 4. Singularities

### What is a Singularity?

A **singularity** is a configuration where the robot **loses a degree of freedom**.

**Mathematically**: Jacobian becomes **non-invertible** (determinant = 0).

**Physically**: Robot cannot move in certain directions, no matter how joints move.

### Detecting Singularities

Compute the **determinant** of the Jacobian:

$$
\det(J) = 0 \implies \text{Singularity}
$$

### 2-DOF Arm Singularities

For our 2-DOF planar arm:

$$
\det(J) = L_1 L_2 \sin(\theta_2)
$$

**Singularities occur when**:
- $\theta_2 = 0°$ (arm fully extended)
- $\theta_2 = 180°$ (arm fully folded)

At these configurations, the robot cannot move perpendicular to the arm!

---

## 5. Exercises

### Exercise 2.5.1: Compute Jacobian for 2-DOF Arm

Implement Jacobian computation.

<InteractivePython
  id="ex-2-5-1"
  title="Jacobian Matrix Calculation"
  starterCode={`import numpy as np

def jacobian_2dof(theta1_deg, theta2_deg, L1=1.0, L2=0.8):
    """
    Compute Jacobian matrix for 2-DOF planar arm.

    Args:
        theta1_deg, theta2_deg: Joint angles (degrees)
        L1, L2: Link lengths

    Returns:
        2x2 Jacobian matrix
    """
    # TODO: Implement Jacobian computation
    # 1. Convert angles to radians
    # 2. Compute partial derivatives:
    #    dx/dtheta1 = -L1*sin(theta1) - L2*sin(theta1+theta2)
    #    dx/dtheta2 = -L2*sin(theta1+theta2)
    #    dy/dtheta1 = L1*cos(theta1) + L2*cos(theta1+theta2)
    #    dy/dtheta2 = L2*cos(theta1+theta2)
    # 3. Build 2x2 matrix
    pass

# Test
J = jacobian_2dof(30, 45)
print("Jacobian at θ1=30°, θ2=45°:")
print(J)
print()

# Verify with numerical differentiation
def numerical_jacobian(theta1_deg, theta2_deg, L1=1.0, L2=0.8, eps=0.01):
    """Compute Jacobian numerically for verification."""
    def fk(t1, t2):
        t1_rad = np.deg2rad(t1)
        t2_rad = np.deg2rad(t2)
        x = L1*np.cos(t1_rad) + L2*np.cos(t1_rad+t2_rad)
        y = L1*np.sin(t1_rad) + L2*np.sin(t1_rad+t2_rad)
        return np.array([x, y])

    pos = fk(theta1_deg, theta2_deg)
    pos_t1 = fk(theta1_deg + eps, theta2_deg)
    pos_t2 = fk(theta1_deg, theta2_deg + eps)

    J_num = np.zeros((2, 2))
    J_num[:, 0] = (pos_t1 - pos) / np.deg2rad(eps)
    J_num[:, 1] = (pos_t2 - pos) / np.deg2rad(eps)
    return J_num

J_numerical = numerical_jacobian(30, 45)
print("Numerical Jacobian (for verification):")
print(J_numerical)
print()
print(f"Match: {np.allclose(J, J_numerical, atol=0.01)}")
`}
  hints={[
    "theta1 = np.deg2rad(theta1_deg)",
    "theta2 = np.deg2rad(theta2_deg)",
    "sum_angle = theta1 + theta2",
    "J[0,0] = -L1*np.sin(theta1) - L2*np.sin(sum_angle)",
    "J[0,1] = -L2*np.sin(sum_angle)",
    "J[1,0] = L1*np.cos(theta1) + L2*np.cos(sum_angle)",
    "J[1,1] = L2*np.cos(sum_angle)"
  ]}
/>

---

### Exercise 2.5.2: End-Effector Velocity from Joint Velocities

Use the Jacobian to compute end-effector velocity.

<InteractivePython
  id="ex-2-5-2"
  title="Compute End-Effector Velocity"
  starterCode={`import numpy as np

def jacobian_2dof(theta1_deg, theta2_deg, L1=1.0, L2=0.8):
    """Jacobian (from previous exercise)."""
    theta1 = np.deg2rad(theta1_deg)
    theta2 = np.deg2rad(theta2_deg)
    sum_angle = theta1 + theta2

    J = np.zeros((2, 2))
    J[0, 0] = -L1*np.sin(theta1) - L2*np.sin(sum_angle)
    J[0, 1] = -L2*np.sin(sum_angle)
    J[1, 0] = L1*np.cos(theta1) + L2*np.cos(sum_angle)
    J[1, 1] = L2*np.cos(sum_angle)
    return J

def end_effector_velocity(theta1_deg, theta2_deg,
                          joint_vel1_deg_s, joint_vel2_deg_s,
                          L1=1.0, L2=0.8):
    """
    Compute end-effector velocity from joint velocities.

    Args:
        theta1_deg, theta2_deg: Current joint angles
        joint_vel1_deg_s, joint_vel2_deg_s: Joint velocities (deg/s)
        L1, L2: Link lengths

    Returns:
        (vx, vy) end-effector velocity (m/s)
    """
    # TODO: Implement velocity calculation
    # 1. Get Jacobian J at current configuration
    # 2. Convert joint velocities to radians/s
    # 3. Compute: v_end = J @ joint_velocities
    # 4. Return (vx, vy)
    pass

# Test case 1: Joint 1 moving, joint 2 stationary
vx, vy = end_effector_velocity(0, 0, joint_vel1_deg_s=10, joint_vel2_deg_s=0)
print("Configuration: θ1=0°, θ2=0° (straight)")
print(f"Joint velocities: dθ1=10°/s, dθ2=0°/s")
print(f"End-effector velocity: ({vx:.3f}, {vy:.3f}) m/s")
print()

# Test case 2: Both joints moving
vx, vy = end_effector_velocity(45, 45, joint_vel1_deg_s=10, joint_vel2_deg_s=5)
print("Configuration: θ1=45°, θ2=45°")
print(f"Joint velocities: dθ1=10°/s, dθ2=5°/s")
print(f"End-effector velocity: ({vx:.3f}, {vy:.3f}) m/s")
`}
  hints={[
    "J = jacobian_2dof(theta1_deg, theta2_deg, L1, L2)",
    "joint_vel_rad = np.array([np.deg2rad(joint_vel1_deg_s), np.deg2rad(joint_vel2_deg_s)])",
    "v_end = J @ joint_vel_rad",
    "return v_end[0], v_end[1]"
  ]}
/>

---

### Exercise 2.5.3: Singularity Detection

Detect singular configurations by analyzing Jacobian determinant.

<InteractivePython
  id="ex-2-5-3"
  title="Detect Singularities"
  starterCode={`import numpy as np

def jacobian_2dof(theta1_deg, theta2_deg, L1=1.0, L2=0.8):
    """Jacobian (from previous exercise)."""
    theta1 = np.deg2rad(theta1_deg)
    theta2 = np.deg2rad(theta2_deg)
    sum_angle = theta1 + theta2

    J = np.zeros((2, 2))
    J[0, 0] = -L1*np.sin(theta1) - L2*np.sin(sum_angle)
    J[0, 1] = -L2*np.sin(sum_angle)
    J[1, 0] = L1*np.cos(theta1) + L2*np.cos(sum_angle)
    J[1, 1] = L2*np.cos(sum_angle)
    return J

def check_singularity(theta1_deg, theta2_deg, L1=1.0, L2=0.8, threshold=1e-3):
    """
    Check if configuration is singular.

    Args:
        theta1_deg, theta2_deg: Joint angles
        L1, L2: Link lengths
        threshold: Determinant threshold for singularity

    Returns:
        {
          'is_singular': bool,
          'det': float (Jacobian determinant),
          'description': str
        }
    """
    # TODO: Implement singularity check
    # 1. Compute Jacobian J
    # 2. Compute determinant: det(J)
    # 3. If |det(J)| < threshold, it's singular
    # 4. Describe the singularity type (extended, folded)
    pass

# Test configurations
configs = [
    (0, 0, "Straight (extended)"),
    (45, 45, "Normal configuration"),
    (0, 180, "Folded"),
    (90, 0, "Extended at 90°"),
    (30, 0.1, "Near-singular (almost extended)")
]

for theta1, theta2, description in configs:
    result = check_singularity(theta1, theta2)
    print(f"{description}: θ1={theta1}°, θ2={theta2}°")
    print(f"  Determinant: {result['det']:.6f}")
    print(f"  Singular: {result['is_singular']}")
    if result['is_singular']:
        print(f"  Type: {result['description']}")
    print()
`}
  hints={[
    "J = jacobian_2dof(theta1_deg, theta2_deg, L1, L2)",
    "det = np.linalg.det(J)",
    "is_singular = abs(det) < threshold",
    "Check theta2: if near 0°, it's extended; if near 180°, it's folded",
    "Return dictionary with det, is_singular, and description"
  ]}
/>

---

## 6. Try With AI

### TryWithAI 2.5.1: Jacobian Validation

<TryWithAI
  id="tryai-2-5-1"
  title="Validate Jacobian Implementation"
  role="Evaluator"
  scenario="You've implemented the Jacobian and want to thoroughly validate it's correct."
  yourTask="Complete Exercise 2.5.1. Then create additional test cases: (1) Compare analytical Jacobian to numerical differentiation for multiple configurations, (2) Verify Jacobian properties (dimensions, rank), (3) Test edge cases (singularities)."
  aiPromptTemplate="I've implemented the Jacobian for a 2-DOF arm. Here's my code: [paste]. Can you review it and suggest comprehensive test cases to validate correctness? I want to test: (1) Multiple configurations (0°, 45°, 90°, 180°), (2) Comparison with numerical differentiation, (3) Singularity detection, (4) Physical meaning of each Jacobian entry. Also explain how to interpret each element of the Jacobian geometrically."
  successCriteria={[
    "Your Jacobian matches numerical differentiation for all test configurations",
    "You understand the physical meaning of each Jacobian entry",
    "You can identify singularities by analyzing the determinant"
  ]}
  reflectionQuestions={[
    "What does each column of the Jacobian represent physically?",
    "What does each row represent?",
    "How would you use the Jacobian for inverse velocity kinematics?"
  ]}
/>

---

## Summary

**Key Takeaways**:

1. **Jacobian Matrix**:
   - Maps joint velocities to end-effector velocities: $\vec{v}_{end} = J \cdot \vec{\dot{\theta}}$
   - Contains partial derivatives: $J_{ij} = \frac{\partial x_i}{\partial \theta_j}$
   - Dimensions: (task space DOF) × (joint space DOF)

2. **2-DOF Arm Jacobian**:
   $$
   J = \begin{bmatrix}
   -L_1 \sin\theta_1 - L_2 \sin(\theta_1+\theta_2) & -L_2 \sin(\theta_1+\theta_2) \\
   L_1 \cos\theta_1 + L_2 \cos(\theta_1+\theta_2) & L_2 \cos(\theta_1+\theta_2)
   \end{bmatrix}
   $$

3. **Applications**:
   - **Velocity control**: Compute end-effector speed from joint speeds
   - **Inverse velocity kinematics**: $\vec{\dot{\theta}} = J^{-1} \vec{v}_{end}$ (when invertible)
   - **Trajectory following**: Maintain desired velocity along path

4. **Singularities**:
   - Occur when $\det(J) = 0$
   - Robot loses DOF at singularity
   - For 2-DOF arm: $\theta_2 = 0°$ (extended) or $\theta_2 = 180°$ (folded)
   - Avoid singularities during motion planning

5. **Numerical Verification**:
   - Compare analytical Jacobian to numerical differentiation
   - Small perturbations: $J_{ij} \approx \frac{x_i(\theta_j + \epsilon) - x_i(\theta_j)}{\epsilon}$

**🎉 Chapter 2 Complete!** You now have the mathematical tools for robot arm programming:
- Coordinate frames and transformations
- Forward and inverse kinematics
- Velocity control with Jacobians

**What's Next**: Take the [Chapter 2 Quiz](./quiz.md) to test your understanding!

---

**Estimated completion time**: 60 minutes | **Prerequisites**: Lessons 2.3, 2.4 | **Difficulty**: B2 (Advanced)
