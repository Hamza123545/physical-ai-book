---
title: "Lesson 7.3: Natural Language to ROS 2 Actions"
description: "Bridge natural language plans to executable ROS 2 actions and services"
chapter: 7
lesson: 3
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
prerequisites: ["chapter-07-lesson-02", "chapter-02-lesson-03"]
has_interactive_python: true
interactive_python_count: 1
has_try_with_ai: false
try_with_ai_count: 0
tags: ["ros2", "actions", "nl-to-code", "vla", "integration"]
---

# Lesson 7.3: Natural Language to ROS 2 Actions

## 🎯 Learning Objectives

- Convert LLM-generated plans to ROS 2 actions
- Implement action execution with error handling
- Validate commands before execution
- Monitor action progress and provide feedback

**Time**: 60 minutes

---

## Introduction

This lesson bridges the gap between LLM-generated plans and actual robot execution. You'll learn to translate structured action sequences into ROS 2 action calls, handle errors, and provide feedback.

**Key Challenge**: LLMs generate high-level plans, but robots need low-level ROS 2 commands.

---

## 1. Action Translation

### Plan to ROS 2 Actions

```python
class ActionExecutor:
    def __init__(self, node):
        self.node = node
        self.nav_action_client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
        self.pick_action_client = ActionClient(node, PickObject, 'pick_object')
        
    def execute_plan(self, plan):
        """Execute LLM-generated plan."""
        for action in plan["actions"]:
            result = self.execute_action(action)
            if not result.success:
                return False
        return True
    
    def execute_action(self, action):
        """Execute single action."""
        action_type = action["type"]
        
        if action_type == "navigate":
            return self.navigate(action["target"])
        elif action_type == "pick":
            return self.pick_object(action["object"])
        elif action_type == "place":
            return self.place_object(action["object"], action["location"])
```

---

## 2. ROS 2 Action Clients

### Navigation Action

```python
from nav2_msgs.action import NavigateToPose

def navigate_to_pose(self, target_pose):
    """Execute navigation action."""
    goal_msg = NavigateToPose.Goal()
    goal_msg.pose = target_pose
    
    self.nav_action_client.wait_for_server()
    send_goal_future = self.nav_action_client.send_goal_async(goal_msg)
    
    # Wait for result
    rclpy.spin_until_future_complete(self.node, send_goal_future)
    goal_handle = send_goal_future.result()
    
    if not goal_handle.accepted:
        return False
    
    # Get result
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(self.node, result_future)
    return result_future.result().result
```

---

## 3. Error Handling

### Robust Execution

```python
def safe_execute_action(self, action, max_retries=3):
    """Execute action with retry logic."""
    for attempt in range(max_retries):
        try:
            result = self.execute_action(action)
            if result.success:
                return True
            else:
                self.node.get_logger().warn(
                    f"Action failed: {result.error_message}"
                )
        except Exception as e:
            self.node.get_logger().error(f"Exception: {e}")
        
        if attempt < max_retries - 1:
            time.sleep(1)  # Wait before retry
    
    return False
```

---

## 4. Progress Monitoring

### Action Feedback

```python
def execute_with_feedback(self, action):
    """Execute action and provide feedback."""
    goal_handle = self.send_action_goal(action)
    
    # Monitor feedback
    while not goal_handle.is_done():
        feedback = goal_handle.get_feedback()
        self.node.get_logger().info(
            f"Progress: {feedback.current_pose}"
        )
        time.sleep(0.1)
    
    return goal_handle.result()
```

---

## 5. Exercises

### Exercise 7.3.1: Action Translator

Convert LLM plan to ROS 2 action calls.

<InteractivePython
  id="ex-7-3-1"
  title="Plan to ROS 2 Actions"
  starterCode={`def translate_plan_to_ros2(plan):
    """
    Convert LLM plan to ROS 2 action calls.
    
    Args:
        plan: Dict with "actions" list
    
    Returns:
        List of ROS 2 action call descriptions
    """
    # TODO: Translate each action to ROS 2 format
    pass

# Test
plan = {
    "actions": [
        {"type": "navigate", "target": "kitchen"},
        {"type": "pick", "object": "cup"},
        {"type": "place", "object": "cup", "location": "sink"}
    ]
}

ros2_actions = translate_plan_to_ros2(plan)
print(ros2_actions)
`}
  hints={[
    "Map action types to ROS 2 action names",
    "Convert parameters to ROS 2 message format",
    "Include error handling structure"
  ]}
/>

---

## Summary

**Key Takeaways**:
1. LLM plans must be translated to ROS 2 actions
2. Action clients execute goals asynchronously
3. Error handling and retries improve robustness
4. Progress monitoring provides user feedback

**What's Next**: [Lesson 7.4: Multi-Modal Interaction](./lesson-04-multimodal.md) combines speech, vision, and gesture.

---

**Estimated completion time**: 60 minutes | **Prerequisites**: Lesson 7.2, Chapter 2 (ROS 2) | **Difficulty**: B2 (Advanced)

