---
title: "Lesson 5.1: Path Planning Basics"
description: "Learn grid-based path planning with A* algorithm"
chapter: 5
lesson: 1
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
prerequisites: ["chapter-01/lesson-04-python-robotics-intro"]
has_interactive_python: true
interactive_python_count: 3
has_try_with_ai: true
try_with_ai_count: 2
tags: ["path-planning", "a-star", "grid-search", "dijkstra"]
---

# Lesson 5.1: Path Planning Basics

## 🎯 Learning Objectives

<LearningObjectives
  cefr_level="B2"
  objectives={[
    {
      text: "Understand grid-based path planning and search algorithms",
      blooms_level: "Understand",
      assessment_method: "Quiz questions"
    },
    {
      text: "Implement A* pathfinding algorithm in Python",
      blooms_level: "Apply",
      assessment_method: "Interactive Python exercises"
    },
    {
      text: "Compare Dijkstra, Greedy Best-First, and A* search strategies",
      blooms_level: "Analyze",
      assessment_method: "TryWithAI exercises"
    }
  ]}
/>

## Introduction

**Path Planning** answers: *How do we get from A to B without hitting obstacles?*

Grid-based methods discretize space into cells (free or occupied), then search for collision-free paths using graph algorithms.

**Time**: 60 minutes

---

## 1. Grid Representation

### Occupancy Grid

Represent 2D space as a grid where:
- **0** = free cell
- **1** = obstacle

```python
grid = [
    [0, 0, 0, 1, 0],
    [0, 1, 0, 1, 0],
    [0, 1, 0, 0, 0],
    [0, 0, 0, 1, 0],
    [0, 0, 0, 0, 0]
]
```

**Robot moves**: 4-connected (up/down/left/right) or 8-connected (+ diagonals).

---

## 2. Search Algorithms

### Dijkstra's Algorithm
- Explores uniformly in all directions
- Guarantees shortest path
- **Cost**: Distance from start

### Greedy Best-First Search
- Heads straight toward goal (uses heuristic)
- Fast but NOT optimal
- **Cost**: Estimated distance to goal

### A* Algorithm
- **Best of both**: Uses cost-so-far + heuristic
- **Formula**: `f(n) = g(n) + h(n)`
  - `g(n)` = cost from start to node n
  - `h(n)` = heuristic (estimated cost from n to goal)
- **Optimal** if heuristic is admissible (never overestimates)

**Common heuristics**:
- Manhattan distance: `|x1 - x2| + |y1 - y2|` (4-connected)
- Euclidean distance: `√((x1-x2)² + (y1-y2)²)` (8-connected)

---

## 3. A* Algorithm Steps

1. **Initialize**: Add start to open_set with f=0
2. **Loop**:
   - Pop node with lowest f-score
   - If goal reached, reconstruct path and return
   - For each neighbor:
     - If obstacle, skip
     - Compute g_new = g_current + cost(current, neighbor)
     - If g_new better than previous, update and add to open_set
3. **Return**: Path from start to goal

---

## 4. Exercises

### Exercise 5.1.1: Manhattan Heuristic

Implement Manhattan distance heuristic for A*.

<InteractivePython
  id="ex-5-1-1"
  title="Manhattan Distance Heuristic"
  starterCode={`def manhattan_heuristic(pos, goal):
    """Compute Manhattan distance from pos to goal."""
    # TODO: Return |x1 - x2| + |y1 - y2|
    # Hint: pos and goal are tuples (x, y)
    pass

# Test
print(manhattan_heuristic((0, 0), (3, 4)))  # Should be 7
print(manhattan_heuristic((2, 3), (2, 3)))  # Should be 0
`}
  hints={[
    "Extract coordinates: x1, y1 = pos; x2, y2 = goal",
    "Return abs(x1 - x2) + abs(y1 - y2)"
  ]}
/>

---

### Exercise 5.1.2: Get Neighbors

Get valid neighbors (not obstacles, not out-of-bounds).

<InteractivePython
  id="ex-5-1-2"
  title="Get Valid Neighbors"
  starterCode={`def get_neighbors(grid, pos):
    """Return valid 4-connected neighbors."""
    rows, cols = len(grid), len(grid[0])
    x, y = pos
    neighbors = []

    # TODO: Check up, down, left, right
    # Add neighbor if: in bounds AND grid[nx][ny] == 0
    # Hint: directions = [(0,1), (0,-1), (1,0), (-1,0)]
    pass

# Test
grid = [[0, 0, 1], [0, 1, 0], [0, 0, 0]]
print(get_neighbors(grid, (0, 0)))  # Should be [(0, 1), (1, 0)]
`}
  hints={[
    "Loop over directions: for dx, dy in [(0,1), (0,-1), (1,0), (-1,0)]",
    "Check bounds: 0 <= nx < rows and 0 <= ny < cols",
    "Check free: grid[nx][ny] == 0"
  ]}
/>

---

### Exercise 5.1.3: Simple A* Implementation

Implement A* pathfinding (simplified version).

<InteractivePython
  id="ex-5-1-3"
  title="A* Pathfinding"
  starterCode={`import heapq

def astar(grid, start, goal):
    """Find shortest path using A*."""
    rows, cols = len(grid), len(grid[0])

    # Priority queue: (f_score, position)
    open_set = [(0, start)]
    came_from = {}
    g_score = {start: 0}

    def heuristic(pos):
        return abs(pos[0] - goal[0]) + abs(pos[1] - goal[1])

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            # Reconstruct path
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return path[::-1]

        # TODO: Explore neighbors
        # For each valid neighbor:
        #   - Compute g_new = g_score[current] + 1
        #   - If better than previous, update and push to open_set
        # Hint: Use get_neighbors from previous exercise
        pass

    return None  # No path found

# Test
grid = [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
path = astar(grid, (0, 0), (2, 2))
print(f"Path: {path}")
`}
  hints={[
    "Get neighbors: for neighbor in get_neighbors(grid, current)",
    "Update g: g_new = g_score[current] + 1",
    "If neighbor not in g_score or g_new < g_score[neighbor]: update",
    "Push to heap: heapq.heappush(open_set, (g_new + heuristic(neighbor), neighbor))"
  ]}
/>

---

## 5. Try With AI

### TryWithAI 5.1.1: Compare Search Algorithms

<TryWithAI
  id="tryai-5-1-1"
  title="Search Algorithm Comparison"
  role="Teacher"
  scenario="You want to understand when to use different search algorithms"
  yourTask="List scenarios where Dijkstra, Greedy Best-First, or A* would be preferred"
  aiPromptTemplate="I think [algorithm] is best for [scenario]. Can you explain the trade-offs between Dijkstra, Greedy Best-First, and A* in terms of optimality, speed, and use cases? Give me 2 examples where each algorithm excels."
  successCriteria={[
    "You understand A* balances optimality and speed",
    "You can identify when heuristics help or hurt performance"
  ]}
  reflectionQuestions={[
    "When would Greedy Best-First fail to find a path?",
    "Why does A* need an admissible heuristic for optimality?"
  ]}
/>

---

### TryWithAI 5.1.2: Debug A* Implementation

<TryWithAI
  id="tryai-5-1-2"
  title="A* Code Review"
  role="Evaluator"
  scenario="You implemented A* but want to ensure correctness and efficiency"
  yourTask="Complete Exercise 5.1.3 with your own implementation"
  aiPromptTemplate="Here's my A* implementation: [paste code]. Review it for: (1) correctness (does it always find optimal path?), (2) edge cases (what if start=goal? no path exists?), (3) efficiency improvements. What am I missing?"
  successCriteria={[
    "Your code handles start=goal case",
    "Your code returns None when no path exists",
    "You avoid re-exploring visited nodes"
  ]}
  reflectionQuestions={[
    "How would you modify A* for 8-connected grids?",
    "What data structure makes A* efficient?"
  ]}
/>

---

## Summary

**Key Takeaways**:

1. **Grid-based planning**: Discretize space into cells, search graph
2. **A* = Dijkstra + Heuristic**: Optimal and efficient with admissible h(n)
3. **Heuristics**: Manhattan (4-conn), Euclidean (8-conn)
4. **Priority queue**: heapq for efficient f-score ordering

**What's Next**: [Lesson 5.2: Sampling-Based Planning](./lesson-02-sampling-planning.md) scales to high-dimensional configuration spaces.

---

**Estimated completion time**: 60 minutes | **Prerequisites**: Chapter 1, Python basics | **Difficulty**: B2 (Upper Intermediate)
