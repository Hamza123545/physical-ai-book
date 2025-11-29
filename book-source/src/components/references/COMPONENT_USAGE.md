# Component Usage Guide

This guide explains how to use the custom MDX components in lesson content.

## Table of Contents

1. [InteractivePython](#interactivepython)
2. [Quiz](#quiz)
3. [TryWithAI](#trywith ai)
4. [LearningObjectives](#learningobjectives)
5. [Prerequisites](#prerequisites)

---

## InteractivePython

Interactive Python code editor with Pyodide execution and automated testing.

### Props

| Prop | Type | Required | Default | Description |
|------|------|----------|---------|-------------|
| `id` | string | Yes | - | Unique exercise identifier for progress tracking |
| `title` | string | Yes | - | Exercise title displayed to students |
| `starterCode` | string | Yes | - | Initial code template with TODO comments |
| `testCases` | array | No | [] | Automated test cases for validation |
| `hints` | array | No | [] | Progressive hints revealed on request |
| `maxRuntime` | number | No | 30 | Maximum execution time in seconds |

### Example

```jsx
<InteractivePython
  id="exercise-ch02-fk-3dof"
  title="3-DOF Robot Arm Forward Kinematics"
  starterCode={`import numpy as np

def forward_kinematics(theta1, theta2, theta3, L1, L2, L3):
    # TODO: Implement forward kinematics
    pass
`}
  testCases={[
    {
      input: {theta1: 0, theta2: 0, theta3: 0, L1: 1, L2: 1, L3: 1},
      expectedOutput: {x: 3.0, y: 0.0},
      tolerance: 0.01,
      description: "All joints at zero should extend straight"
    }
  ]}
  hints={[
    "Start by computing the first joint position",
    "Use trigonometry to add each link's contribution"
  ]}
/>
```

---

## Quiz

Interactive quiz component with 50 questions, randomized batching, and immediate feedback.

### Props

| Prop | Type | Required | Default | Description |
|------|------|----------|---------|-------------|
| `chapterId` | number | Yes | - | Chapter ID for progress tracking |
| `questions` | array | Yes | - | Array of 50 quiz questions |
| `questionsPerBatch` | number | No | 17 | Number of questions shown per session |

### Example

```jsx
<Quiz
  chapterId={2}
  questions={[
    {
      id: "q-ch02-01",
      question: "What is forward kinematics?",
      type: "multiple-choice",
      options: [
        "Compute joint angles from position",
        "Compute position from joint angles",
        "Compute dynamics",
        "Compute trajectories"
      ],
      correctAnswer: 1,
      explanation: "Forward kinematics computes end-effector position from joint angles.",
      wrongAnswerExplanations: [
        "This is inverse kinematics",
        "This is dynamics",
        "This is trajectory planning"
      ]
    },
    // ... 49 more questions
  ]}
  questionsPerBatch={17}
/>
```

---

## TryWithAI

AI co-learning exercise component following the AI Three Roles Framework.

### Props

| Prop | Type | Required | Description |
|------|------|----------|-------------|
| `id` | string | Yes | Unique exercise ID |
| `title` | string | Yes | Exercise title |
| `role` | 'Teacher' \| 'Copilot' \| 'Evaluator' | Yes | AI role in this exercise |
| `scenario` | string | Yes | Real-world context for the problem |
| `yourTask` | string | Yes | What the student should attempt first |
| `aiPromptTemplate` | string | Yes | Prompt template for students to use with AI |
| `successCriteria` | array | Yes | How to verify the solution works |
| `reflectionQuestions` | array | Yes | Questions for reflection on AI collaboration |

### Example

```jsx
<TryWithAI
  id="tryai-ch02-ik"
  title="Derive Inverse Kinematics with AI Copilot"
  role="Copilot"
  scenario="You need to implement inverse kinematics for a 3-DOF robot arm"
  yourTask="First implement forward kinematics and test it"
  aiPromptTemplate="I have this FK function: [paste code]. Help me derive IK. Explain each step."
  successCriteria={[
    "You can explain the geometric reasoning",
    "Your code correctly computes joint angles"
  ]}
  reflectionQuestions={[
    "What did AI help you understand?",
    "How did you verify the solution?"
  ]}
/>
```

---

## LearningObjectives

Displays lesson objectives with CEFR and Bloom's taxonomy alignment.

### Props

| Prop | Type | Required | Description |
|------|------|----------|-------------|
| `cefr_level` | string | Yes | CEFR proficiency level (e.g., "B1", "B2") |
| `objectives` | array | Yes | Array of learning objectives |

### Objective Structure

Each objective must have:
- `text`: string - The objective description
- `blooms_level`: 'Remember' \| 'Understand' \| 'Apply' \| 'Analyze' \| 'Evaluate' \| 'Create'
- `assessment_method`: string - How this objective is assessed

### Example

```jsx
<LearningObjectives
  cefr_level="B1"
  objectives={[
    {
      text: "Implement forward kinematics for a 3-DOF planar robot arm",
      blooms_level: "Apply",
      assessment_method: "Interactive Python exercise"
    },
    {
      text: "Explain the difference between forward and inverse kinematics",
      blooms_level: "Understand",
      assessment_method: "Quiz question"
    }
  ]}
/>
```

---

## Prerequisites

Displays a checklist of prerequisite lessons with completion tracking.

### Props

| Prop | Type | Required | Default | Description |
|------|------|----------|---------|-------------|
| `prerequisites` | array | Yes | - | Array of prerequisite lessons |
| `showCompletionStatus` | boolean | No | true | Show checkmarks for completed lessons |

### Prerequisite Structure

Each prerequisite must have:
- `lessonId`: string - Lesson identifier (e.g., "chapter-01-lesson-02")
- `title`: string - Lesson title
- `link`: string - Relative link to the lesson

### Example

```jsx
<Prerequisites
  prerequisites={[
    {
      lessonId: "chapter-01-lesson-02",
      title: "Introduction to Robot Coordinate Frames",
      link: "/docs/chapter-01/lesson-02"
    },
    {
      lessonId: "chapter-02-lesson-01",
      title: "2D Rotation Matrices",
      link: "/docs/chapter-02/lesson-01"
    }
  ]}
/>
```

---

## Best Practices

### Component Order in Lessons

Follow this order for consistency:

1. **LearningObjectives** - Start with clear goals
2. **Prerequisites** - Ensure students have required knowledge
3. Main lesson content
4. **InteractivePython** exercises - Practice with code
5. **TryWithAI** exercises - Apply with AI assistance
6. Summary section

### YAML Frontmatter

Always include all 7 generation fields plus proficiency levels:

```yaml
---
title: "Lesson Title"
chapter: 2
lesson: 3
duration_minutes: 90

# Skills Proficiency
cefr_level: "B1"
blooms_level: "Apply"
digcomp_level: 3

# Generation metadata
generated_by: "claude-sonnet-4-5-20250929"
source_spec: "specs/002-physical-ai-textbook/spec.md"
created: "2025-11-29"
last_modified: "2025-11-29"
git_author: "hswat"
workflow: "/sp.implement"
version: "1.0"

# Learning metadata
prerequisites:
  - "chapter-02-lesson-01"
learning_objectives:
  - "Implement forward kinematics for 3-DOF robot"
success_criteria:
  - "Code correctly computes end-effector position"

# Component counts
has_interactive_python: true
interactive_python_count: 3
has_try_with_ai: true
try_with_ai_count: 2
---
```

### Accessibility

- Always provide `description` text for interactive components
- Use semantic headings (h1, h2, h3)
- Ensure color contrast meets WCAG 2.1 AA standards
- Test keyboard navigation

### Performance

- Keep `starterCode` concise (< 50 lines)
- Ensure Python code executes in < 30 seconds
- Limit test cases to 3-5 per exercise
- Use lazy loading for images and diagrams

---

## Testing Components

### Local Testing

```bash
cd book-source
npm start
# Navigate to lesson and test components
```

### Validation Checklist

- [ ] All components render without errors
- [ ] InteractivePython executes code in Pyodide
- [ ] Quiz displays questions and validates answers
- [ ] TryWithAI shows all sections correctly
- [ ] LearningObjectives displays Bloom's badges
- [ ] Prerequisites tracks completion status
- [ ] Components are responsive on mobile
- [ ] Dark mode works correctly
