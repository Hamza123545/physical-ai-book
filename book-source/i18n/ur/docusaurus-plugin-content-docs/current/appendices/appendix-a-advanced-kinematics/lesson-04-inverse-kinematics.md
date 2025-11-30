---
title: "Lesson 2.4: Inverse Kinematics"
description: "Find joint angles that achieve a target end-effector position - the inverse problem"
chapter: 2
lesson: 4
estimated_time: 70
cefr_level: "B2"
blooms_level: "Analyze"
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
has_interactive_python: true
interactive_python_count: 4
has_try_with_ai: true
try_with_ai_count: 2
tags: ["inverse-kinematics", "geometric-ik", "reachability", "multiple-solutions"]
---

# Lesson 2.4: Inverse Kinematics

## 🎯 Learning Objectives

<LearningObjectives
  cefr_level="B2"
  objectives={[
    {
      text: "Derive and implement geometric inverse kinematics for 2-DOF arms",
      blooms_level: "Analyze",
      assessment_method: "Python exercises with verification"
    },
    {
      text: "Handle multiple IK solutions (elbow up/down configurations)",
      blooms_level: "Analyze",
      assessment_method: "Multiple solution exercise"
    },
    {
      text: "Implement workspace reachability checks",
      blooms_level: "Apply",
      assessment_method: "Reachability validation exercise"
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
    }
  ]}
/>

## Introduction

**Inverse Kinematics (IK)** solves the opposite problem from FK:

> **"Given a target position, what joint angles will reach it?"**

- **Input**: Target position $(x_{target}, y_{target})$
- **Output**: Joint angles $(\theta_1, \theta_2)$

IK is **much harder** than FK because:
1. **Multiple solutions** may exist (elbow up vs. elbow down)
2. **No solution** may exist (target outside workspace)
3. **Closed-form** solutions aren't always available

This lesson focuses on **geometric IK** for planar arms—analytical solutions using geometry and trigonometry.

**Time**: 70 minutes

---

## 1. Why IK is Harder than FK

### Forward Kinematics
- **One input → One output**
- Given $(\theta_1, \theta_2)$ → Always get $(x, y)$
- Direct computation, no ambiguity

### Inverse Kinematics
- **One input → Multiple outputs** (or zero!)
- Given $(x, y)$ → May have 0, 1, 2, or more $(\theta_1, \theta_2)$ solutions
- Requires solving nonlinear equations

**Example** (2-DOF arm):
- Target: $(1.0, 1.0)$
- Solution 1: Elbow up → $(\theta_1, \theta_2) = (60°, -30°)$
- Solution 2: Elbow down → $(\theta_1, \theta_2) = (30°, 60°)$

Both configurations reach the same target!

---

## 2. Geometric IK for 2-DOF Planar Arm

### Problem Setup

Given:
- Link lengths: $L_1, L_2$
- Target position: $(x, y)$

Find: $(\theta_1, \theta_2)$

### Step 1: Check Reachability

Distance from origin to target:
$$
d = \sqrt{x^2 + y^2}
$$

**Reachability conditions**:
- **Minimum reach**: $d \geq |L_1 - L_2|$ (arm can't fold inward beyond this)
- **Maximum reach**: $d \leq L_1 + L_2$ (arm fully extended)

If $d$ violates these, **no solution exists**.

### Step 2: Solve for $\theta_2$ (Elbow Angle)

Using the law of cosines:

$$
\cos\theta_2 = \frac{x^2 + y^2 - L_1^2 - L_2^2}{2 L_1 L_2}
$$

$$
\theta_2 = \pm \arccos\left(\frac{x^2 + y^2 - L_1^2 - L_2^2}{2 L_1 L_2}\right)
$$

**Two solutions**:
- $+\arccos(\cdot)$: Elbow down
- $-\arccos(\cdot)$: Elbow up

### Step 3: Solve for $\theta_1$ (Shoulder Angle)

$$
\theta_1 = \text{atan2}(y, x) - \text{atan2}(L_2 \sin\theta_2, L_1 + L_2 \cos\theta_2)
$$

**Note**: Use `atan2(y, x)` instead of `atan(y/x)` to handle all quadrants correctly.

---

## 3. Handling Multiple Solutions

For a 2-DOF arm, there are typically **2 solutions**:

1. **Elbow up**: $\theta_2 < 0$ (negative elbow angle)
2. **Elbow down**: $\theta_2 > 0$ (positive elbow angle)

**Which to choose?**
- Depends on application (obstacle avoidance, joint limits, smoothness)
- Often pick the solution closest to current configuration
- Or pick based on constraints (e.g., "always elbow up")

---

## 4. Exercises

### Exercise 2.4.1: 2-DOF Geometric IK

Implement inverse kinematics for a 2-link planar arm.

<InteractivePython
  id="ex-2-4-1"
  title="2-DOF Inverse Kinematics"
  starterCode={`import numpy as np

def ik_2dof(x_target, y_target, L1=1.0, L2=0.8, elbow_up=True):
    """
    Geometric inverse kinematics for 2-DOF planar arm.

    Args:
        x_target, y_target: Target end-effector position
        L1, L2: Link lengths
        elbow_up: If True, return elbow-up solution; else elbow-down

    Returns:
        (theta1_deg, theta2_deg) or None if unreachable
    """
    # TODO: Implement IK
    # 1. Check reachability: d = sqrt(x^2 + y^2)
    #    If d > L1+L2 or d < |L1-L2|, return None
    # 2. Solve for theta2 using law of cosines
    # 3. Solve for theta1 using atan2
    # 4. Convert to degrees
    pass

# Test 1: Reachable point
theta1, theta2 = ik_2dof(1.0, 1.0)
print(f"IK for (1.0, 1.0): θ1={theta1:.1f}°, θ2={theta2:.1f}°")

# Verify with FK
def fk_2dof(theta1_deg, theta2_deg, L1=1.0, L2=0.8):
    theta1 = np.deg2rad(theta1_deg)
    theta2 = np.deg2rad(theta2_deg)
    x = L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2)
    y = L1 * np.sin(theta1) + L2 * np.sin(theta1 + theta2)
    return x, y

x_fk, y_fk = fk_2dof(theta1, theta2)
print(f"FK verification: ({x_fk:.3f}, {y_fk:.3f})")
print(f"Target: (1.0, 1.0)")
print(f"Match: {np.allclose([x_fk, y_fk], [1.0, 1.0], atol=0.001)}")
print()

# Test 2: Unreachable point
result = ik_2dof(5.0, 5.0)
print(f"IK for (5.0, 5.0): {result}")
print("Expected: None (out of reach)")
`}
  hints={[
    "d = np.sqrt(x_target**2 + y_target**2)",
    "if d > L1 + L2 or d < abs(L1 - L2): return None",
    "cos_theta2 = (x_target**2 + y_target**2 - L1**2 - L2**2) / (2*L1*L2)",
    "theta2_rad = np.arccos(np.clip(cos_theta2, -1, 1))",
    "If elbow_up: theta2_rad = -theta2_rad",
    "theta1_rad = np.arctan2(y_target, x_target) - np.arctan2(L2*np.sin(theta2_rad), L1 + L2*np.cos(theta2_rad))"
  ]}
/>

---

### Exercise 2.4.2: Multiple IK Solutions

Compute both elbow-up and elbow-down solutions.

<InteractivePython
  id="ex-2-4-2"
  title="Multiple IK Solutions"
  starterCode={`import numpy as np

def ik_2dof_both(x_target, y_target, L1=1.0, L2=0.8):
    """
    Return BOTH IK solutions (elbow up and elbow down).

    Returns:
        {
          'elbow_up': (theta1, theta2) or None,
          'elbow_down': (theta1, theta2) or None
        }
    """
    # TODO: Implement both solutions
    # 1. Check reachability
    # 2. Compute theta2 for elbow up (negative arccos)
    # 3. Compute corresponding theta1
    # 4. Compute theta2 for elbow down (positive arccos)
    # 5. Compute corresponding theta1
    # 6. Return both solutions
    pass

# Test
target = (1.2, 0.8)
solutions = ik_2dof_both(*target)

print(f"Target: {target}")
print()

if solutions['elbow_up']:
    theta1, theta2 = solutions['elbow_up']
    print(f"Elbow up: θ1={theta1:.1f}°, θ2={theta2:.1f}°")
else:
    print("Elbow up: No solution")

if solutions['elbow_down']:
    theta1, theta2 = solutions['elbow_down']
    print(f"Elbow down: θ1={theta1:.1f}°, θ2={theta2:.1f}°")
else:
    print("Elbow down: No solution")

# Verify both solutions reach the target
def fk_2dof(theta1_deg, theta2_deg, L1=1.0, L2=0.8):
    theta1 = np.deg2rad(theta1_deg)
    theta2 = np.deg2rad(theta2_deg)
    x = L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2)
    y = L1 * np.sin(theta1) + L2 * np.sin(theta1 + theta2)
    return x, y

print()
for config in ['elbow_up', 'elbow_down']:
    if solutions[config]:
        x, y = fk_2dof(*solutions[config])
        print(f"{config} FK: ({x:.3f}, {y:.3f}) - Match: {np.allclose([x,y], target, atol=0.001)}")
`}
  hints={[
    "Check reachability first",
    "cos_theta2 = (x^2 + y^2 - L1^2 - L2^2) / (2*L1*L2)",
    "theta2_up = -arccos(cos_theta2)",
    "theta2_down = arccos(cos_theta2)",
    "Compute theta1 for each theta2 using atan2 formula",
    "Return dictionary with both solutions"
  ]}
/>

---

### Exercise 2.4.3: Reachability Check

Validate whether a target is within the robot's workspace.

<InteractivePython
  id="ex-2-4-3"
  title="Workspace Reachability"
  starterCode={`import numpy as np

def is_reachable(x_target, y_target, L1=1.0, L2=0.8):
    """
    Check if target is within robot workspace.

    Returns:
        {
          'reachable': bool,
          'distance': float (from origin),
          'min_reach': float (inner workspace radius),
          'max_reach': float (outer workspace radius),
          'reason': str (if not reachable)
        }
    """
    # TODO: Implement reachability check
    # 1. Compute distance d to target
    # 2. Compute min_reach = |L1 - L2|
    # 3. Compute max_reach = L1 + L2
    # 4. Check if min_reach <= d <= max_reach
    # 5. Return result dictionary with explanation
    pass

# Test cases
test_points = [
    (1.0, 1.0, "Normal point"),
    (1.8, 0.0, "Fully extended"),
    (0.2, 0.0, "Folded inward"),
    (0.05, 0.0, "Too close (unreachable)"),
    (3.0, 0.0, "Too far (unreachable)")
]

for x, y, description in test_points:
    result = is_reachable(x, y)
    print(f"{description}: ({x}, {y})")
    print(f"  Distance: {result['distance']:.3f}")
    print(f"  Reachable: {result['reachable']}")
    if not result['reachable']:
        print(f"  Reason: {result['reason']}")
    print()
`}
  hints={[
    "distance = np.sqrt(x_target**2 + y_target**2)",
    "min_reach = abs(L1 - L2)",
    "max_reach = L1 + L2",
    "reachable = (min_reach <= distance <= max_reach)",
    "Set reason based on which condition failed"
  ]}
/>

---

### Exercise 2.4.4: 3-DOF IK (Simplified)

Implement IK for a 3-DOF arm (position only, ignore orientation).

<InteractivePython
  id="ex-2-4-4"
  title="3-DOF IK (Position Only)"
  starterCode={`import numpy as np

def ik_3dof_position(x_target, y_target, L1=1.0, L2=0.8, L3=0.6):
    """
    3-DOF IK for position only (ignore end-effector orientation).

    Strategy:
    1. Use first 2 joints to reach a "wrist" position
    2. Use joint 3 to extend to final target

    Args:
        x_target, y_target: Target position
        L1, L2, L3: Link lengths

    Returns:
        (theta1, theta2, theta3) in degrees, or None if unreachable
    """
    # TODO: Implement simplified 3-DOF IK
    # Strategy 1: Set theta3=0, solve 2-DOF IK for joints 1 and 2
    # This treats links 2 and 3 as a single combined link of length L2+L3
    #
    # More advanced: Solve for "wrist position" that's L3 away from target,
    # then solve 2-DOF IK to reach wrist position

    pass

# Test: Target at (1.5, 1.0)
result = ik_3dof_position(1.5, 1.0)
if result:
    theta1, theta2, theta3 = result
    print(f"3-DOF IK for (1.5, 1.0):")
    print(f"  θ1={theta1:.1f}°, θ2={theta2:.1f}°, θ3={theta3:.1f}°")

    # Verify with FK
    def fk_3dof(theta1_deg, theta2_deg, theta3_deg, L1=1.0, L2=0.8, L3=0.6):
        theta1 = np.deg2rad(theta1_deg)
        theta2 = np.deg2rad(theta2_deg)
        theta3 = np.deg2rad(theta3_deg)
        x = L1*np.cos(theta1) + L2*np.cos(theta1+theta2) + L3*np.cos(theta1+theta2+theta3)
        y = L1*np.sin(theta1) + L2*np.sin(theta1+theta2) + L3*np.sin(theta1+theta2+theta3)
        return x, y

    x_fk, y_fk = fk_3dof(theta1, theta2, theta3)
    print(f"  FK verification: ({x_fk:.3f}, {y_fk:.3f})")
    print(f"  Match: {np.allclose([x_fk, y_fk], [1.5, 1.0], atol=0.01)}")
else:
    print("Target unreachable")
`}
  hints={[
    "Simplest approach: Set theta3 = 0",
    "Treat L2 and L3 as combined link: L_combined = L2 + L3",
    "Call 2-DOF IK with (x_target, y_target, L1, L_combined)",
    "Return (theta1, theta2, 0)",
    "Advanced: Compute wrist position = target - L3 * (direction vector)"
  ]}
/>

---

## 5. Try With AI

### TryWithAI 2.4.1: Geometric IK Derivation

<TryWithAI
  id="tryai-2-4-1"
  title="Understand IK Derivation"
  role="Teacher"
  scenario="You want to deeply understand how the geometric IK formulas are derived, not just use them."
  yourTask="Before asking AI, try to derive the IK solution yourself using: (1) Law of cosines to find θ2, (2) Geometry to find θ1. Draw a diagram of the robot arm reaching for a target."
  aiPromptTemplate="I'm trying to understand the geometric derivation of 2-DOF planar arm IK. I know the formulas but want to understand WHY. Can you explain step-by-step: (1) How to use the law of cosines to find θ2 from the target position and link lengths, (2) Why θ2 has two solutions (± arccos), (3) How to derive θ1 using atan2 and geometry? Please include diagrams or detailed geometric reasoning."
  successCriteria={[
    "You understand the law of cosines application to the triangle formed by L1, L2, and target distance",
    "You can explain geometrically why there are two solutions",
    "You understand why atan2 is needed (not just atan)"
  ]}
  reflectionQuestions={[
    "What happens geometrically when the target is at max reach (L1 + L2)?",
    "What happens at min reach (|L1 - L2|)?",
    "How would you handle a 3-DOF arm differently?"
  ]}
/>

---

### TryWithAI 2.4.2: IK Solution Selection

<TryWithAI
  id="tryai-2-4-2"
  title="Choose Best IK Solution"
  role="Copilot"
  scenario="Your robot has multiple IK solutions for a target. You need to choose the best one based on: (1) Avoiding obstacles, (2) Minimizing joint movement, (3) Staying within joint limits."
  yourTask="Complete Exercise 2.4.2 to get both solutions. Think about criteria for choosing between them. Consider a scenario where the robot is currently at θ1=30°, θ2=30° and needs to reach a new target."
  aiPromptTemplate="I have a 2-DOF arm with two IK solutions (elbow up and elbow down) for a target. The current joint configuration is (θ1=30°, θ2=30°). I need to choose the best solution that: (1) Minimizes total joint movement, (2) Respects joint limits (e.g., -180° to 180°), (3) Avoids obstacles. Can you help me implement a solution selection algorithm? Here are my two IK solutions: [paste both solutions]."
  successCriteria={[
    "You can compute joint displacement (distance in joint space)",
    "You can filter solutions that violate joint limits",
    "You understand trade-offs in solution selection"
  ]}
  reflectionQuestions={[
    "How would you incorporate obstacle avoidance into IK selection?",
    "What if all solutions violate constraints?",
    "How would you handle redundant robots (more DOF than needed)?"
  ]}
/>

---

## Summary

**Key Takeaways**:

1. **Inverse Kinematics**:
   - Input: Target position $(x, y)$
   - Output: Joint angles $(\theta_1, \theta_2)$
   - Harder than FK: multiple/zero solutions possible

2. **Geometric IK for 2-DOF**:
   - Check reachability: $|L_1 - L_2| \leq d \leq L_1 + L_2$
   - Solve $\theta_2$ using law of cosines
   - Solve $\theta_1$ using $\text{atan2}$
   - Two solutions: elbow up ($-\arccos$) and elbow down ($+\arccos$)

3. **Multiple Solutions**:
   - 2-DOF arm typically has 2 solutions
   - Choose based on: current config, joint limits, obstacles
   - Both solutions verified by FK

4. **Reachability**:
   - **Workspace**: annulus from $|L_1 - L_2|$ to $L_1 + L_2$
   - **Inside inner circle**: unreachable (too close)
   - **Outside outer circle**: unreachable (too far)

5. **3-DOF IK**:
   - Redundant: infinite solutions (3 DOF for 2D position)
   - Simplified: fix one joint, solve 2-DOF IK
   - Full solution: optimize for additional criteria

**What's Next**: [Lesson 2.5: Jacobians and Velocities](./lesson-05-jacobians-velocities.md) introduces the Jacobian matrix to relate joint velocities to end-effector velocities.

---

**Estimated completion time**: 70 minutes | **Prerequisites**: Lesson 2.3 | **Difficulty**: B2 (Advanced)
