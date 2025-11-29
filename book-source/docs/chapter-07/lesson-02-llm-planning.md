---
title: "Lesson 7.2: LLM-Based Cognitive Planning"
description: "Use GPT models to translate natural language into structured robot action plans"
chapter: 7
lesson: 2
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
prerequisites: ["chapter-07-lesson-01"]
has_interactive_python: true
interactive_python_count: 1
has_try_with_ai: false
try_with_ai_count: 0
tags: ["llm", "gpt", "cognitive-planning", "task-decomposition", "vla"]
---

# Lesson 7.2: LLM-Based Cognitive Planning

## 🎯 Learning Objectives

- Understand how LLMs can plan robot actions
- Design prompts for task decomposition
- Generate structured action sequences from natural language
- Validate and refine LLM-generated plans

**Time**: 70 minutes

---

## Introduction

**Cognitive Planning** uses Large Language Models (LLMs) like GPT to translate high-level natural language commands into detailed action sequences. Instead of hard-coding every possible command, LLMs can understand intent and generate appropriate plans.

**Example**:
- **Input**: "Clean the room"
- **LLM Output**: 
  1. Navigate to room
  2. Identify objects to clean
  3. Pick up each object
  4. Place in appropriate location
  5. Return to starting position

---

## 1. LLM Prompt Engineering

### Basic Prompt Structure

```python
prompt = f"""
You are a robot planning assistant. Convert the following command into a sequence of actions.

Command: {user_command}

Available actions:
- move(direction, distance)
- turn(angle)
- pick(object_name)
- place(object_name, location)
- navigate_to(location)

Generate a JSON list of actions:
"""

response = openai.ChatCompletion.create(
    model="gpt-4",
    messages=[{"role": "user", "content": prompt}]
)
```

### Advanced Prompting

Include context about:
- Robot capabilities
- Current environment state
- Safety constraints
- Previous actions

---

## 2. Task Decomposition

### Hierarchical Planning

Complex tasks need decomposition:

```python
def plan_task(command, context):
    """Decompose high-level task into sub-tasks."""
    prompt = f"""
    Task: {command}
    Context: {context}
    
    Break this into 3-5 sub-tasks. For each sub-task, list:
    1. Sub-task description
    2. Required actions
    3. Prerequisites
    """
    
    # Call LLM
    plan = call_llm(prompt)
    return parse_plan(plan)
```

---

## 3. Action Sequence Generation

### Structured Output

Generate JSON-formatted action sequences:

```python
def generate_action_sequence(task_description):
    """Generate ROS 2 action sequence from task description."""
    prompt = f"""
    Convert this task into ROS 2 actions:
    Task: {task_description}
    
    Output JSON format:
    {{
        "actions": [
            {{"type": "navigate", "target": "location"}},
            {{"type": "pick", "object": "name"}},
            {{"type": "place", "object": "name", "location": "target"}}
        ]
    }}
    """
    
    response = call_llm(prompt)
    return json.loads(response)
```

---

## 4. Plan Validation

### Safety Checks

Always validate LLM-generated plans:

```python
def validate_plan(plan):
    """Validate generated plan for safety and feasibility."""
    errors = []
    
    # Check action types are valid
    valid_actions = ["move", "turn", "pick", "place", "navigate"]
    for action in plan["actions"]:
        if action["type"] not in valid_actions:
            errors.append(f"Invalid action: {action['type']}")
    
    # Check required parameters
    # Check safety constraints
    # Check resource availability
    
    return len(errors) == 0, errors
```

---

## 5. Exercises

### Exercise 7.2.1: Task Decomposition

Design a prompt that decomposes "Clean the room" into sub-tasks.

<InteractivePython
  id="ex-7-2-1"
  title="LLM Task Decomposition"
  starterCode={`def create_planning_prompt(task):
    """
    Create a prompt for LLM to decompose a task.
    
    Args:
        task: Natural language task description
    
    Returns:
        Formatted prompt string
    """
    # TODO: Create prompt with task, available actions, and output format
    pass

# Test
task = "Clean the room"
prompt = create_planning_prompt(task)
print(prompt)
`}
  hints={[
    "Include task description",
    "List available robot actions",
    "Specify desired output format (JSON)",
    "Include safety constraints"
  ]}
/>

---

## Summary

**Key Takeaways**:
1. LLMs can decompose high-level tasks into action sequences
2. Prompt engineering is crucial for reliable planning
3. Always validate LLM-generated plans for safety
4. Structured output (JSON) enables easy parsing

**What's Next**: [Lesson 7.3: Natural Language to ROS 2 Actions](./lesson-03-nl-to-ros2.md) bridges LLM plans to executable ROS 2 code.

---

**Estimated completion time**: 70 minutes | **Prerequisites**: Lesson 7.1 | **Difficulty**: B2 (Advanced)

