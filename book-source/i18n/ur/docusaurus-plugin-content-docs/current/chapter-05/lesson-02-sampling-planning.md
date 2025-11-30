---
title: "Lesson 5.2: Sampling-Based Planning"
description: "Learn RRT and PRM algorithms for high-dimensional planning"
chapter: 5
lesson: 2
estimated_time: 60
cefr_level: "B2"
blooms_level: "Apply"
digcomp_level: 6
generated_by: "claude-sonnet-4-5-20250929"
source_spec: "specs/002-physical-ai-textbook/spec.md"
created: "2025-11-29"
last_modified: "2025-11-29"
git_author: "claude"
workflow: "/sp.implement"
version: "1.0"
prerequisites: ["chapter-05/lesson-01-path-planning"]
has_interactive_python: true
interactive_python_count: 3
has_try_with_ai: true
try_with_ai_count: 1
tags: ["rrt", "prm", "sampling-based", "motion-planning"]
---

# Lesson 5.2: Sampling-Based Planning

## 🎯 Learning Objectives

<LearningObjectives
  cefr_level="B2"
  objectives={[
    {
      text: "Understand why sampling-based methods scale to high dimensions",
      blooms_level: "Understand",
      assessment_method: "Quiz questions"
    },
    {
      text: "Implement RRT (Rapidly-exploring Random Tree) algorithm",
      blooms_level: "Apply",
      assessment_method: "Interactive Python exercises"
    },
    {
      text: "Compare RRT vs PRM trade-offs for different planning problems",
      blooms_level: "Analyze",
      assessment_method: "TryWithAI exercises"
    }
  ]}
/>

## Introduction

**Grid-based A* fails** in high dimensions (robot arms, humanoids): a 6-DOF arm with 100 cells/dimension needs 100^6 = 1 trillion cells!

**Sampling-based methods** (RRT, PRM) randomly sample the configuration space, avoiding explicit discretization.

**Time**: 60 minutes

---

## 1. The Curse of Dimensionality

### Grid Explosion

For n-DOF robot with r resolution per dimension:
- **Grid cells**: r^n
- 6-DOF, 100 res/dim: 10^12 cells (infeasible!)

### Sampling Solution

Instead of discretizing:
1. **Sample random configurations** (e.g., 1000 samples)
2. **Connect nearby samples** (collision-free edges)
3. **Search the roadmap** (A* on samples)

---

## 2. PRM: Probabilistic Roadmap

### Algorithm

**Learning Phase** (build roadmap once):
1. Sample N random configurations
2. For each sample, try to connect to k nearest neighbors
3. Keep collision-free edges

**Query Phase** (reuse roadmap):
1. Connect start and goal to roadmap
2. Search roadmap with A*

### Properties

- **Multi-query**: Build once, use many times
- **Pros**: Efficient for repeated queries
- **Cons**: Slow preprocessing, may miss narrow passages

---

## 3. RRT: Rapidly-exploring Random Tree

### Algorithm

1. **Initialize**: Tree contains only start node
2. **Loop** until goal reached:
   - Sample random configuration q_rand
   - Find nearest node q_near in tree
   - Extend from q_near toward q_rand by step_size
   - If collision-free, add q_new to tree
3. **Return**: Path from start to goal

### Properties

- **Single-query**: Fast planning, no preprocessing
- **Pros**: Good for narrow passages, dynamic environments
- **Cons**: Paths not optimal (can post-process)

### Variants

- **RRT***: Converges to optimal path
- **Bi-directional RRT**: Grow trees from start and goal
- **RRT-Connect**: Greedy variant for faster connection

---

## 4. Collision Checking

Both PRM and RRT need:

```python
def is_collision_free(config, obstacles):
    """Check if robot configuration collides."""
    # For 2D point robot: check if in obstacle
    # For arm: check all links against obstacles
    return not in_obstacle(config)
```

**Expensive operation**: Dominate planning time. Optimizations matter!

---

## 5. Exercises

### Exercise 5.2.1: Random Sampling

Sample random configurations in 2D workspace.

<InteractivePython
  id="ex-5-2-1"
  title="Random Configuration Sampling"
  starterCode={`import numpy as np

def sample_random_config(bounds, n_samples=100):
    """Sample random 2D configurations within bounds.

    Args:
        bounds: ((x_min, x_max), (y_min, y_max))
        n_samples: Number of samples

    Returns:
        Array of shape (n_samples, 2)
    """
    # TODO: Generate random (x, y) within bounds
    # Hint: np.random.uniform(low, high, size)
    pass

# Test
bounds = ((0, 10), (0, 10))
samples = sample_random_config(bounds, 100)
print(f"Samples shape: {samples.shape}")
print(f"First 3: {samples[:3]}")
`}
  hints={[
    "x = np.random.uniform(bounds[0][0], bounds[0][1], n_samples)",
    "y = np.random.uniform(bounds[1][0], bounds[1][1], n_samples)",
    "return np.column_stack([x, y])"
  ]}
/>

---

### Exercise 5.2.2: Nearest Neighbor

Find nearest node in tree to a target configuration.

<InteractivePython
  id="ex-5-2-2"
  title="Nearest Neighbor Search"
  starterCode={`import numpy as np

def find_nearest(tree, target):
    """Find nearest node in tree to target.

    Args:
        tree: List of (x, y) configurations
        target: (x, y) target configuration

    Returns:
        Index of nearest node, nearest node
    """
    # TODO: Compute distances to all tree nodes, return nearest
    # Hint: np.linalg.norm for Euclidean distance
    pass

# Test
tree = [(0, 0), (1, 1), (2, 0), (1, 2)]
target = (1.5, 0.5)
idx, nearest = find_nearest(tree, target)
print(f"Nearest: {nearest} at index {idx}")
`}
  hints={[
    "Convert tree to numpy array: tree_arr = np.array(tree)",
    "Compute distances: dists = np.linalg.norm(tree_arr - target, axis=1)",
    "Find min: idx = np.argmin(dists)",
    "return idx, tree[idx]"
  ]}
/>

---

### Exercise 5.2.3: Simple RRT

Implement basic RRT for 2D point robot.

<InteractivePython
  id="ex-5-2-3"
  title="RRT Planner"
  starterCode={`import numpy as np

def simple_rrt(start, goal, bounds, max_iters=500, step_size=0.5):
    """Build RRT from start toward goal.

    Args:
        start: (x, y) start configuration
        goal: (x, y) goal configuration
        bounds: ((x_min, x_max), (y_min, y_max))
        max_iters: Maximum iterations
        step_size: Extension step size

    Returns:
        List of tree nodes [(x, y), ...]
    """
    tree = [start]

    for _ in range(max_iters):
        # TODO:
        # 1. Sample random config (or goal with 10% probability)
        # 2. Find nearest node in tree
        # 3. Extend toward sample by step_size
        # 4. Add to tree if within bounds
        # 5. If close to goal, return tree
        pass

    return tree

# Test
start = (0.0, 0.0)
goal = (9.0, 9.0)
bounds = ((0, 10), (0, 10))
tree = simple_rrt(start, goal, bounds)
print(f"Tree size: {len(tree)}")
print(f"Final node: {tree[-1]}")
`}
  hints={[
    "Sample: q_rand = (np.random.uniform(bounds[0][0], bounds[0][1]), ...)",
    "Goal bias: if np.random.rand() < 0.1: q_rand = goal",
    "Extend: direction = (q_rand - q_near) / distance; q_new = q_near + direction * step_size",
    "Check goal: if distance(q_new, goal) < step_size: return tree"
  ]}
/>

---

## 6. Try With AI

### TryWithAI 5.2.1: RRT vs PRM Trade-offs

<TryWithAI
  id="tryai-5-2-1"
  title="When to Use RRT vs PRM"
  role="Teacher"
  scenario="You need to choose between RRT and PRM for a robot planning task"
  yourTask="Describe a scenario and pick RRT or PRM, then ask AI for feedback"
  aiPromptTemplate="I have a [describe robot and environment, e.g., '6-DOF arm in static warehouse']. I chose [RRT/PRM] because [your reasoning]. Is this the right choice? What are the key trade-offs I should consider? Are there scenarios where my choice would be wrong?"
  successCriteria={[
    "You understand PRM is better for static, multi-query problems",
    "You understand RRT is better for dynamic, single-query problems",
    "You can identify when preprocessing cost is worthwhile"
  ]}
  reflectionQuestions={[
    "When would PRM fail to find narrow passages?",
    "How does obstacle density affect RRT performance?"
  ]}
/>

---

## Summary

**Key Takeaways**:

1. **Sampling scales**: Avoid r^n grid explosion with random sampling
2. **PRM**: Multi-query, preprocessing roadmap, best for static environments
3. **RRT**: Single-query, fast exploration, best for dynamic environments
4. **Collision checking**: Dominates runtime, needs optimization
5. **Trade-offs**: Optimality vs speed, single vs multi-query

**What's Next**: [Lesson 5.3: Trajectory Generation](./lesson-03-trajectory-generation.md) smooths paths into executable trajectories.

---

**Estimated completion time**: 60 minutes | **Prerequisites**: Lesson 5.1 | **Difficulty**: B2 (Upper Intermediate)
