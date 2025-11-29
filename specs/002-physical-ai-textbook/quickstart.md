# Quickstart Guide: Physical AI & Humanoid Robotics Textbook

**Feature**: 002-physical-ai-textbook
**Purpose**: Get contributors started with textbook content development

## Prerequisites

- Node.js 18+ and npm/yarn installed
- Git configured
- Text editor (VS Code recommended)
- Basic familiarity with Markdown and React/TypeScript

## Project Structure

```
physical-ai-book/
├── book-source/                    # Docusaurus site
│   ├── docs/                       # Lesson content (Markdown)
│   │   ├── chapter-01/            # Chapter directories
│   │   │   ├── index.md           # Chapter intro
│   │   │   ├── lesson-01.md       # Lesson files
│   │   │   ├── lesson-02.md
│   │   │   └── quiz.md            # Chapter quiz (50 questions)
│   │   └── intro.md               # Textbook homepage
│   ├── src/
│   │   ├── components/            # Custom MDX components
│   │   │   ├── InteractivePython.tsx
│   │   │   ├── Quiz.tsx
│   │   │   ├── TryWithAI.tsx
│   │   │   ├── LearningObjectives.tsx
│   │   │   └── Prerequisites.tsx
│   │   └── css/                   # Custom styles
│   ├── static/                    # Images, diagrams
│   ├── docusaurus.config.js       # Site configuration
│   ├── sidebars.js                # Navigation structure
│   └── package.json
├── specs/002-physical-ai-textbook/ # Planning artifacts
│   ├── spec.md                    # Requirements
│   ├── plan.md                    # This implementation plan
│   ├── research.md                # Technology decisions
│   ├── data-model.md              # Data structures
│   └── contracts/                 # API specs
└── .claude/skills/                # Reusable content creation skills

```

## Installation

### 1. Clone and Install Dependencies

```bash
cd book-source
npm install
```

### 2. Start Development Server

```bash
npm start
```

Opens browser at `http://localhost:3000`

### 3. Build for Production

```bash
npm run build
```

Output in `book-source/build/`

## Creating New Content

### Create a New Lesson

1. **Create lesson file**: `book-source/docs/chapter-XX/lesson-YY-slug.md`

2. **Add YAML frontmatter**:

```yaml
---
title: "Lesson Title"
description: "One-sentence summary"
chapter: 2
lesson: 3
estimated_time: 90

# Skills Proficiency
cefr_level: "B1"
blooms_level: "Apply"
digcomp_level: 3

# Generation metadata
generation_type: "lesson"
content_stage: "draft"
validation_status: "pending"
last_generated: "2025-11-29T12:00:00Z"
generator_version: "v2.1.0"
constitution_version: "v4.0.1"
quality_score: null

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

tags:
  - "kinematics"
  - "robot-arm"
---
```

3. **Write lesson content** following 4-Layer structure:
   - Learning Objectives
   - Prerequisites
   - Concept Introduction (Layer 1)
   - Worked Examples (Layer 2)
   - Interactive Exercises (Layer 3)
   - Challenge Problems (Layer 4)
   - Summary

4. **Add interactive components**:

**InteractivePython Exercise**:
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

**Try With AI Exercise**:
```jsx
<TryWithAI
  id="tryai-ch02-ik"
  title="Derive Inverse Kinematics with AI"
  role="Copilot"
  scenario="You need inverse kinematics for a 3-DOF arm"
  yourTask="First implement forward kinematics"
  aiPromptTemplate="I have this FK function: [paste]. Help me derive IK. Explain each step."
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

### Create a Chapter Quiz

1. **Create quiz file**: `book-source/docs/chapter-XX/quiz.md`

2. **Use Quiz component with 50 questions**:

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
      explanation: "Forward kinematics computes end-effector position from joint angles using geometric transformations.",
      wrongAnswerExplanations: [
        "This describes inverse kinematics, not forward kinematics",
        "Dynamics involves forces and torques, not just positions",
        "Trajectory planning is a separate motion planning problem"
      ]
    },
    // ... 49 more questions
  ]}
  questionsPerBatch={17}
/>
```

## Using Skills

### Generate Learning Objectives

```bash
# From .claude/skills/learning-objectives
# Invoke skill in conversation: "Generate learning objectives for Chapter 2 Lesson 3 on forward kinematics"
```

### Generate Quiz Questions

```bash
# From .claude/skills/quiz-generator
# Invoke skill: "Generate 50 quiz questions for Chapter 2 covering kinematics"
```

### Validate Content Quality

```bash
# From .claude/skills/content-evaluation-framework
# Invoke skill: "Validate Chapter 2 Lesson 3 for technical accuracy and pedagogical effectiveness"
```

## Development Workflow

### Standard Workflow

1. **Plan lesson** using `learning-objectives` skill
2. **Write draft** with YAML frontmatter and 4-Layer structure
3. **Add InteractivePython** exercises (3+ per lesson)
4. **Add TryWithAI** exercises (1-2 per lesson)
5. **Test code** in browser (all Python must run in Pyodide)
6. **Validate** using `content-evaluation-framework` skill
7. **Generate quiz** after all chapter lessons complete
8. **Final chapter validation**

### Quality Gates

**Before Marking Lesson Complete**:
- [ ] All YAML frontmatter fields filled
- [ ] CEFR level appropriate for content complexity
- [ ] At least 3 InteractivePython components
- [ ] 1-2 TryWithAI components
- [ ] All code executes in Pyodide < 30 seconds
- [ ] Learning objectives measurable and testable
- [ ] Prerequisites exist and form acyclic graph
- [ ] 60-120 minute estimated time
- [ ] Alt text for all images

**Before Marking Chapter Complete**:
- [ ] All lessons validated
- [ ] 50-question quiz created and tested
- [ ] Questions distributed across Bloom's levels
- [ ] Total chapter time 2-4 hours
- [ ] All prerequisites satisfied

## Testing

### Test Python Code Locally

```python
# Install Pyodide locally for testing
pip install pyodide-build
pyodide build  # Builds WASM packages

# Or test in browser dev tools
```

### Test Interactive Components

```bash
npm start
# Navigate to lesson and test each InteractivePython component
# Verify:
# - Code editor loads
# - Run button executes code
# - Output displays correctly
# - Test cases validate
# - Hints reveal progressively
# - Reset restores starter code
# - Save persists to localStorage
```

### Validate Quiz

```bash
npm start
# Navigate to chapter quiz
# Verify:
# - 50 questions present
# - Questions randomized each retake
# - 15-20 questions per batch
# - Immediate feedback after each answer
# - Explanations for correct and incorrect answers
# - Progress tracked
# - Score saved to localStorage
```

## Deployment

### Deploy to GitHub Pages

```bash
npm run build
npm run deploy
```

Requires `gh-pages` configured in `package.json`:

```json
{
  "scripts": {
    "deploy": "docusaurus deploy"
  },
  "homepage": "https://yourusername.github.io/physical-ai-book/"
}
```

### Deploy to Netlify/Vercel

Connect GitHub repo to Netlify/Vercel:
- Build command: `npm run build`
- Publish directory: `build`
- Node version: 18+

## Common Issues

### Python Code Doesn't Execute

- **Check Pyodide availability**: Components load Pyodide from CDN
- **Verify libraries**: Only NumPy, Matplotlib, SciPy available
- **Check syntax**: Python code must be valid Python 3.11+
- **Timeout**: Code must complete in < 30 seconds

### Quiz Not Displaying

- **Verify count**: Must have exactly 50 questions
- **Check format**: Each question must have all required fields
- **Component registration**: Quiz component globally registered in `src/theme/MDXComponents`

### Progress Not Saving

- **localStorage quota**: Check browser storage (max ~5-10MB)
- **Private browsing**: localStorage disabled in incognito mode
- **Browser support**: Modern browsers required (Chrome 57+, Firefox 52+, Safari 11+)

## Resources

### Documentation

- Docusaurus: https://docusaurus.io/docs
- Pyodide: https://pyodide.org/en/stable/
- KaTeX: https://katex.org/docs/supported.html

### Skills Reference

- `.claude/skills/learning-objectives/skill.md`
- `.claude/skills/quiz-generator/skill.md`
- `.claude/skills/content-evaluation-framework/skill.md`
- `.claude/skills/code-example-generator/skill.md`

### Templates

- `specs/002-physical-ai-textbook/plan.md` (Chapter/lesson breakdown)
- `book-source/src/components/references/example-quiz.md` (Quiz template)
- `book-source/src/components/QUIZ_USAGE.md` (Quiz usage guide)

## Getting Help

- Review `specs/002-physical-ai-textbook/spec.md` for requirements
- Check `specs/002-physical-ai-textbook/research.md` for technology decisions
- See `specs/002-physical-ai-textbook/data-model.md` for data structures
- Invoke skills for content generation assistance

## Next Steps

1. Review the full implementation plan: `specs/002-physical-ai-textbook/plan.md`
2. Start with Chapter 1 content creation
3. Follow the 4-Layer Teaching Method
4. Use skills for learning objectives, quizzes, and validation
5. Test all interactive components before marking lessons complete
