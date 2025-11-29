# Data Model: Physical AI & Humanoid Robotics Textbook

**Date**: 2025-11-29
**Feature**: 002-physical-ai-textbook
**Phase**: Phase 1 - Data Model Design

## Overview

This textbook uses **client-side data models** stored in browser localStorage. No server-side database required (static deployment per spec requirement FR-016).

## Core Entities

### 1. Chapter

Represents a major learning module in the textbook.

**Attributes**:
- `id`: number (1-9)
- `title`: string
- `description`: string (one-sentence summary)
- `estimatedTime`: number (minutes, 120-240 range per constraint TC-007)
- `cefr_range`: string (e.g., "B1 to B1+")
- `lessons`: Lesson[] (3-5 lessons per chapter)
- `prerequisites`: number[] (chapter IDs that must be completed first)
- `topics`: string[] (main topics covered)

**Relationships**:
- Contains 3-5 Lessons
- May depend on previous Chapters (prerequisites)
- Has one Quiz (50 questions)

**Validation Rules**:
- Exactly 9 chapters total (FR-001)
- 3-5 lessons per chapter
- Chapter IDs sequential (1-9)
- Prerequisites form acyclic dependency graph

**Example**:
```json
{
  "id": 2,
  "title": "Robot Kinematics and Dynamics",
  "description": "Learn to model robot motion and compute joint configurations",
  "estimatedTime": 180,
  "cefr_range": "B1 to B1+",
  "lessons": [ /* Lesson objects */ ],
  "prerequisites": [1],
  "topics": ["forward-kinematics", "inverse-kinematics", "jacobians", "dynamics"]
}
```

### 2. Lesson

Represents a single learning unit within a chapter.

**Attributes**:
- `id`: string (format: `chapter-{n}-lesson-{m}`)
- `title`: string
- `description`: string
- `chapter`: number (parent chapter ID)
- `lessonNumber`: number (1-5 within chapter)
- `estimatedTime`: number (60-120 minutes per requirement #5)
- `cefr_level`: string (A1|A2|B1|B1+|B2|B2+|C1|C2)
- `blooms_level`: string (Remember|Understand|Apply|Analyze|Evaluate|Create)
- `digcomp_level`: number (1-8)
- `filePath`: string (path to markdown file)
- `prerequisites`: string[] (lesson IDs)
- `learningObjectives`: LearningObjective[]
- `successCriteria`: string[]
- `components`: ComponentCounts
- `tags`: string[]

**Relationships**:
- Belongs to one Chapter
- May depend on previous Lessons (prerequisites)
- Contains 3+ Interactive Python exercises (FR-006)
- Contains 1-2 "Try With AI" exercises

**Validation Rules**:
- Lesson ID format: `chapter-{n}-lesson-{m}`
- Estimated time: 60-120 minutes
- CEFR level matches content complexity
- At least 3 InteractivePython components (FR-006)
- 1-2 TryWithAI components

**Example**:
```json
{
  "id": "chapter-02-lesson-03",
  "title": "Forward Kinematics of Robot Arms",
  "description": "Compute end-effector position from joint angles",
  "chapter": 2,
  "lessonNumber": 3,
  "estimatedTime": 90,
  "cefr_level": "B1",
  "blooms_level": "Apply",
  "digcomp_level": 3,
  "filePath": "docs/chapter-02/lesson-03-forward-kinematics.md",
  "prerequisites": ["chapter-02-lesson-01", "chapter-02-lesson-02"],
  "learningObjectives": [ /* LearningObjective objects */ ],
  "successCriteria": [
    "Student implements FK for 3-DOF arm correctly",
    "Visualization shows accurate robot poses"
  ],
  "components": {
    "interactivePython": 3,
    "tryWithAI": 2,
    "quizQuestions": 0
  },
  "tags": ["kinematics", "forward-kinematics", "3DOF", "visualization"]
}
```

### 3. LearningObjective

Specific skill or knowledge outcome aligned with CEFR and Bloom's taxonomy.

**Attributes**:
- `id`: string
- `text`: string (measurable objective statement)
- `blooms_level`: string
- `assessment_method`: string (how to verify achievement)
- `aligned_with_spec`: string (reference to spec success criterion)

**Validation Rules**:
- Uses action verbs (implement, design, explain, analyze)
- Measurable and testable
- Aligns with lesson CEFR and Bloom's level

**Example**:
```json
{
  "id": "lo-chapter-02-lesson-03-01",
  "text": "Implement forward kinematics for a 3-DOF planar robot arm using NumPy",
  "blooms_level": "Apply",
  "assessment_method": "Interactive Python exercise with automated test cases",
  "aligned_with_spec": "SC-002"
}
```

### 4. InteractivePython

Executable Python code exercise with immediate feedback.

**Attributes**:
- `id`: string (unique exercise identifier)
- `title`: string
- `description`: string (exercise instructions)
- `starterCode`: string (initial code template)
- `solution`: string (reference solution, not shown to students)
- `testCases`: TestCase[]
- `hints`: string[] (progressive hints)
- `expectedOutput`: string (example of correct output)
- `maxRuntime`: number (seconds, max 30 per CC-003)
- `requiredLibraries`: string[] (NumPy, Matplotlib, etc.)

**Relationships**:
- Belongs to one Lesson
- Has multiple TestCases for validation

**Validation Rules**:
- Starter code must be syntactically valid Python
- Solution must execute in < 30 seconds
- All required libraries available in Pyodide
- Test cases cover edge cases

**Example**:
```json
{
  "id": "exercise-ch02-fk-3dof",
  "title": "3-DOF Robot Arm Forward Kinematics",
  "description": "Complete the forward_kinematics function to compute end-effector position",
  "starterCode": "import numpy as np\n\ndef forward_kinematics(theta1, theta2, theta3, L1, L2, L3):\n    # TODO: Implement forward kinematics\n    pass",
  "solution": "import numpy as np\n\ndef forward_kinematics(theta1, theta2, theta3, L1, L2, L3):\n    x = L1*np.cos(theta1) + L2*np.cos(theta1+theta2) + L3*np.cos(theta1+theta2+theta3)\n    y = L1*np.sin(theta1) + L2*np.sin(theta1+theta2) + L3*np.sin(theta1+theta2+theta3)\n    return x, y",
  "testCases": [ /* TestCase objects */ ],
  "hints": [
    "Start by computing the position of the first joint",
    "Use trigonometry to add each link's contribution",
    "Remember to accumulate angles for each joint"
  ],
  "expectedOutput": "x = 2.5, y = 1.8",
  "maxRuntime": 5,
  "requiredLibraries": ["numpy"]
}
```

### 5. TestCase

Automated validation for InteractivePython exercises.

**Attributes**:
- `id`: string
- `input`: object (function arguments)
- `expectedOutput`: any
- `tolerance`: number (for floating-point comparisons)
- `description`: string (what this test validates)

**Example**:
```json
{
  "id": "test-ch02-fk-01",
  "input": {
    "theta1": 0,
    "theta2": 0,
    "theta3": 0,
    "L1": 1,
    "L2": 1,
    "L3": 1
  },
  "expectedOutput": {"x": 3.0, "y": 0.0},
  "tolerance": 0.01,
  "description": "All joints at zero should extend straight along x-axis"
}
```

### 6. Quiz

Chapter assessment with 50 questions using Quiz component.

**Attributes**:
- `id`: string (format: `quiz-chapter-{n}`)
- `chapter`: number
- `totalQuestions`: number (always 50 per FR-011)
- `questionsPerBatch`: number (15-20, randomized)
- `questions`: QuizQuestion[]
- `passingScore`: null (no passing threshold per research decision)

**Relationships**:
- Belongs to one Chapter
- Contains exactly 50 QuizQuestions

**Validation Rules**:
- Exactly 50 questions (FR-011)
- Questions distributed across chapter lessons
- Mix of Bloom's levels (Remember: 20%, Understand: 30%, Apply: 30%, Analyze: 20%)
- All questions have explanations

**Example**:
```json
{
  "id": "quiz-chapter-02",
  "chapter": 2,
  "totalQuestions": 50,
  "questionsPerBatch": 17,
  "questions": [ /* 50 QuizQuestion objects */ ],
  "passingScore": null
}
```

### 7. QuizQuestion

Individual quiz question with multiple choice or true/false format.

**Attributes**:
- `id`: string
- `question`: string
- `type`: string (multiple-choice | true-false)
- `options`: string[] (2 for true-false, 4 for multiple-choice)
- `correctAnswer`: number (index of correct option)
- `explanation`: string (why answer is correct)
- `wrongAnswerExplanations`: string[] (why each wrong answer is incorrect)
- `blooms_level`: string
- `relatedLessons`: string[] (lesson IDs)
- `tags`: string[]

**Validation Rules**:
- True-false: 2 options
- Multiple choice: 4 options
- All options have explanations
- Related lessons exist

**Example**:
```json
{
  "id": "q-ch02-fk-01",
  "question": "What is the purpose of forward kinematics in robotics?",
  "type": "multiple-choice",
  "options": [
    "Compute joint angles from end-effector position",
    "Compute end-effector position from joint angles",
    "Compute robot mass properties",
    "Compute optimal trajectories"
  ],
  "correctAnswer": 1,
  "explanation": "Forward kinematics computes the end-effector position given joint angles, using geometric transformations.",
  "wrongAnswerExplanations": [
    "This describes inverse kinematics, not forward kinematics",
    "Mass properties are computed from dynamics models, not kinematics",
    "Trajectory optimization is a separate motion planning problem"
  ],
  "blooms_level": "Understand",
  "relatedLessons": ["chapter-02-lesson-03"],
  "tags": ["kinematics", "definitions"]
}
```

### 8. TryWithAI

AI co-learning exercise following AI Three Roles Framework.

**Attributes**:
- `id`: string
- `title`: string
- `role`: string (Teacher | Copilot | Evaluator)
- `scenario`: string (real-world context)
- `yourTask`: string (what student attempts first)
- `aiPromptTemplate`: string (prompt for AI tool)
- `successCriteria`: string[]
- `reflectionQuestions`: string[]

**Relationships**:
- Belongs to one Lesson
- 1-2 per lesson

**Validation Rules**:
- Role must be one of: Teacher, Copilot, Evaluator
- Prompt template clear and specific
- Success criteria verifiable

**Example**:
```json
{
  "id": "tryai-ch02-ik",
  "title": "Derive Inverse Kinematics with AI Copilot",
  "role": "Copilot",
  "scenario": "You need to implement inverse kinematics for a 3-DOF robot arm to reach target positions",
  "yourTask": "First, implement forward kinematics and test it. Then use AI to help derive inverse kinematics equations.",
  "aiPromptTemplate": "I have this forward kinematics function for a 3-DOF planar robot arm: [paste code]. Help me derive the inverse kinematics equations. Explain each step using geometric reasoning so I understand the approach.",
  "successCriteria": [
    "You can explain the geometric reasoning behind IK solution",
    "Your IK code correctly computes joint angles for target positions",
    "You understand when IK has multiple solutions or no solution"
  ],
  "reflectionQuestions": [
    "What geometric insight did the AI provide that helped you understand IK?",
    "How did you verify the AI's derivation was correct?",
    "What limitations of the IK solution did you discover?"
  ]
}
```

### 9. StudentProgress

Client-side progress tracking stored in localStorage.

**Attributes**:
- `userId`: string (generated client-side, UUID)
- `chapters`: ChapterProgress[]
- `exercises`: ExerciseProgress[]
- `lastSynced`: string (ISO timestamp)
- `exportedAt`: string | null (last export timestamp)

**Storage Location**: `localStorage['physical-ai-textbook-progress']`

**Validation Rules**:
- Data serializable to JSON
- Total size < 5MB (constraint TC-003)
- Auto-save on progress updates

**Example**:
```json
{
  "userId": "550e8400-e29b-41d4-a716-446655440000",
  "chapters": [
    {
      "chapterId": 1,
      "completed": true,
      "quizScore": 85,
      "quizAttempts": 2,
      "lastAccess": "2025-11-29T12:00:00Z"
    },
    {
      "chapterId": 2,
      "completed": false,
      "lessonsCompleted": ["chapter-02-lesson-01", "chapter-02-lesson-02"],
      "lastAccess": "2025-11-29T13:00:00Z"
    }
  ],
  "exercises": [
    {
      "exerciseId": "exercise-ch02-fk-3dof",
      "savedCode": "import numpy as np\n\n# Student's work in progress",
      "lastSaved": "2025-11-29T13:15:00Z",
      "testsPassed": 2,
      "testsTotal": 5
    }
  ],
  "lastSynced": "2025-11-29T13:15:00Z",
  "exportedAt": null
}
```

### 10. ChapterProgress

Progress tracking for a single chapter.

**Attributes**:
- `chapterId`: number
- `completed`: boolean
- `lessonsCompleted`: string[] (lesson IDs)
- `quizScore`: number | null (percentage, 0-100)
- `quizAttempts`: number
- `lastAccess`: string (ISO timestamp)

### 11. ExerciseProgress

Progress tracking for InteractivePython exercises.

**Attributes**:
- `exerciseId`: string
- `savedCode`: string (student's current code)
- `lastSaved`: string (ISO timestamp)
- `testsPassed`: number
- `testsTotal`: number
- `hintsUsed`: number
- `completed`: boolean

## Data Relationships Diagram

```
Textbook (root)
├── Chapter [1-9]
│   ├── Lesson [3-5]
│   │   ├── LearningObjective [multiple]
│   │   ├── InteractivePython [3+]
│   │   │   └── TestCase [multiple]
│   │   └── TryWithAI [1-2]
│   └── Quiz [1]
│       └── QuizQuestion [50]
└── StudentProgress
    ├── ChapterProgress [per chapter]
    └── ExerciseProgress [per exercise]
```

## Storage Strategy

### Static Content (Build-time)

Stored as:
- Markdown files with YAML frontmatter (lessons)
- JSON manifests (chapter/lesson metadata)
- React components (InteractivePython, Quiz, TryWithAI)

**Location**: `book-source/docs/` and `book-source/src/components/`

### Dynamic Content (Runtime)

Stored in:
- `localStorage['physical-ai-textbook-progress']`: StudentProgress object
- `sessionStorage['current-exercise']`: Temporary exercise state

**Size Management**:
- Monitor storage usage with `navigator.storage.estimate()`
- Warn user at 80% of 5MB quota
- Offer export before clearing old data

### Export/Import Format

Students can export progress as JSON file:

**Filename**: `physical-ai-textbook-progress-{date}.json`

**Contents**: Complete StudentProgress object

**Import**: Parse JSON and validate structure before restoring to localStorage

## Validation & Consistency

### Content Validation

**Chapter-Level**:
- 9 chapters total
- Sequential IDs (1-9)
- Acyclic prerequisite graph
- 3-5 lessons each
- Total time 2-4 hours per chapter

**Lesson-Level**:
- Valid CEFR level (B1-B2 range)
- Bloom's level appropriate for exercises
- At least 3 InteractivePython components
- 1-2 TryWithAI components
- 60-120 minute duration
- Prerequisites exist

**Quiz-Level**:
- Exactly 50 questions per chapter
- Questions distributed across lessons
- Mix of Bloom's levels
- All explanations present

**Exercise-Level**:
- Code executes in Pyodide < 30 seconds
- Required libraries available
- Test cases comprehensive
- Starter code syntactically valid

### Progress Validation

**Consistency Checks**:
- Lesson completion implies prerequisite lessons completed
- Chapter completion implies all lessons and quiz completed
- Exercise progress doesn't exceed available exercises

**Data Integrity**:
- JSON structure matches schema
- Timestamps in valid ISO format
- IDs reference existing content
- Scores in valid ranges (0-100)

## Performance Considerations

### Data Size Estimates

**Per Lesson**:
- Markdown: ~20KB
- Metadata: ~2KB
- Total: ~22KB

**Per Chapter** (5 lessons + quiz):
- Lessons: 5 × 22KB = 110KB
- Quiz (50 questions): ~30KB
- Total: ~140KB

**Entire Textbook**:
- 9 chapters: 9 × 140KB = 1.26MB
- Components: ~500KB
- Images/diagrams: ~3MB
- Total: **~5MB** (within constraint TC-005)

**Progress Data**:
- Chapter progress: ~200 bytes
- Exercise progress: ~500 bytes per exercise
- Total: ~50KB for full progress

## API Contracts

See `contracts/` directory for:
- `student-progress-api.json`: localStorage interface
- `content-metadata-api.json`: Chapter/lesson metadata structure
- `component-props.json`: React component interfaces

## Summary

Data model supports:
- ✅ 9 chapters with 3-5 lessons each
- ✅ CEFR B1-B2 proficiency mapping
- ✅ 50-question quizzes per chapter
- ✅ Interactive Python exercises with test cases
- ✅ "Try With AI" co-learning exercises
- ✅ Client-side progress tracking
- ✅ Export/import functionality
- ✅ Validation at all levels
- ✅ Performance within constraints
