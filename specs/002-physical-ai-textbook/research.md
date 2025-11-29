# Research: Physical AI & Humanoid Robotics Textbook

**Date**: 2025-11-29
**Feature**: 002-physical-ai-textbook
**Phase**: Phase 0 - Research & Technology Decisions

## Research Questions & Resolutions

### 1. Pyodide Integration for Browser-Based Python

**Question**: How do we execute Python 3.11+ code exercises in the browser without server-side execution?

**Decision**: Use Pyodide (WebAssembly Python runtime)

**Rationale**:
- Official Python port to WebAssembly maintained by Python core developers
- Supports Python 3.11+ with full standard library
- Compatible with scientific computing libraries (NumPy, Matplotlib, SciPy)
- Excellent browser support (Chrome 57+, Firefox 52+, Safari 11+, Edge 79+)
- Active development and good documentation
- Used successfully in JupyterLite and other educational platforms

**Alternatives Considered**:
- **Skulpt**: Rejected - Python 2.x syntax only, limited library support
- **Brython**: Rejected - Incomplete standard library, poor scientific computing support
- **Server-side execution**: Rejected - Violates spec requirement for static deployment, adds complexity and hosting costs

**Implementation Details**:
- Load Pyodide from CDN (https://cdn.jsdelivr.net/pyodide/)
- Bundle NumPy, Matplotlib, SciPy packages
- Initialize once per page load, reuse runtime for multiple exercises
- Implement code execution timeout (30 seconds max per spec constraint CC-003)
- Capture stdout/stderr for user feedback

### 2. InteractivePython Component Architecture

**Question**: How do we structure interactive Python code exercises with immediate feedback?

**Decision**: Create custom React component `InteractivePython` with CodeMirror editor

**Rationale**:
- CodeMirror provides excellent Python syntax highlighting and editing experience
- React component can manage Pyodide runtime lifecycle
- Supports keyboard shortcuts (Ctrl+C, Ctrl+V, Ctrl+Z per FR-027)
- Easy integration with Docusaurus MDX
- Can persist code to localStorage for progress tracking (FR-014)

**Component Features**:
- **Editor**: CodeMirror 6 with Python mode
- **Execution**: Run button triggers Pyodide execution
- **Output**: Console display for stdout/stderr
- **Reset**: Restore starter code
- **Save**: Persist to localStorage with exercise ID as key
- **Loading**: Spinner during execution
- **Error handling**: User-friendly Python error messages with line numbers

**Alternatives Considered**:
- **Monaco Editor**: Rejected - Heavier bundle size, overkill for textbook exercises
- **Ace Editor**: Rejected - Less active development than CodeMirror
- **Plain textarea**: Rejected - Poor developer experience, no syntax highlighting

### 3. Quiz Component System

**Question**: How do we implement 50-question quizzes with randomized batching and immediate feedback?

**Decision**: Use existing Quiz component from book-source/src/components/Quiz.tsx (already globally registered)

**Rationale**:
- Component already exists and is globally registered in Docusaurus
- Supports 50 comprehensive questions with 15-20 displayed per batch
- Implements randomized shuffling on retake
- Provides immediate feedback per question with explanations
- No imports needed in lesson files (global registration)
- Color-coded feedback and progress tracking built-in

**Question Bank Strategy**:
- 50 questions per chapter distributed across lessons
- Mix of question types: multiple choice, true/false, concept application
- Align with Bloom's taxonomy levels appropriate for B1-B2 CEFR
- Include explanations for correct AND incorrect answers
- Reference specific lesson sections for review

**Alternatives Considered**:
- **Custom quiz component**: Rejected - Existing component meets all requirements
- **Third-party quiz libraries**: Rejected - Adds dependencies, existing component sufficient
- **Static markdown quizzes**: Rejected - No interactivity, no progress tracking

### 4. Progress Tracking & Persistence

**Question**: How do we track student progress (completed chapters, quiz scores, saved code) without server-side storage?

**Decision**: Use browser localStorage with structured JSON

**Rationale**:
- No server required (aligns with static deployment)
- 5-10MB storage sufficient for progress data
- Synchronous API easy to use
- Persists across sessions
- Privacy-friendly (no user data sent to servers)

**Storage Structure**:
```javascript
{
  "physical-ai-textbook": {
    "chapters": {
      "01": {
        "completed": true,
        "quizScore": 85,
        "quizAttempts": 2,
        "lastAccess": "2025-11-29T12:00:00Z"
      }
    },
    "exercises": {
      "chapter-02-forward-kinematics": {
        "code": "# Student's saved code",
        "lastSaved": "2025-11-29T12:00:00Z"
      }
    }
  }
}
```

**Limitations & Mitigations**:
- **No cross-device sync**: Accepted as reasonable constraint (Assumption #8 in spec)
- **Browser clearing data**: Provide export/import functionality (FR-030)
- **Storage quota**: Monitor usage, warn if approaching 5MB

**Alternatives Considered**:
- **IndexedDB**: Rejected - Overkill for simple key-value storage
- **Cookies**: Rejected - 4KB limit insufficient
- **Server-side storage**: Rejected - Requires authentication, violates static deployment

### 5. Mathematical Notation Rendering

**Question**: How do we render mathematical expressions (kinematics equations, control theory formulas)?

**Decision**: Use KaTeX for LaTeX math rendering in Docusaurus

**Rationale**:
- Fastest math rendering library (significantly faster than MathJax)
- Excellent Docusaurus integration via remark-math plugin
- Supports all common mathematical notation needed (matrices, vectors, derivatives, integrals)
- No JavaScript required for rendering (CSS-based)
- Accessibility features (aria-labels, semantic HTML)

**Configuration**:
```javascript
// docusaurus.config.js
remarkPlugins: [require('remark-math')],
rehypePlugins: [require('rehype-katex')],
stylesheets: [
  {
    href: 'https://cdn.jsdelivr.net/npm/katex@0.16.9/dist/katex.min.css',
    type: 'text/css',
  },
],
```

**Alternatives Considered**:
- **MathJax**: Rejected - Slower rendering, heavier bundle
- **AsciiMath**: Rejected - Less expressive than LaTeX, unfamiliar to students
- **Image-based formulas**: Rejected - Poor accessibility, not searchable

### 6. Robotics Visualization & Simulation

**Question**: How do we visualize robot kinematics, motion planning, and control without full physics engines?

**Decision**: Use lightweight 2D/3D visualizations with Matplotlib and custom Canvas rendering

**Rationale**:
- Matplotlib in Pyodide supports 2D plotting (trajectories, sensor data, learning curves)
- HTML5 Canvas for lightweight 2D robot arm visualization
- Three.js for optional 3D humanoid visualization (lazy loaded)
- Sufficient fidelity for learning concepts without full physics simulation
- Stays within browser performance constraints

**Visualization Types**:
- **Robot arm**: 2D stick figure with Canvas (forward kinematics, inverse kinematics)
- **Mobile robot navigation**: 2D top-down view with obstacles (path planning)
- **RL training**: Matplotlib line plots (reward curves, episode performance)
- **Humanoid balance**: Simple 2D or 3D stick figure with COM visualization
- **Sensor data**: Matplotlib plots (camera feeds as images, depth data as heatmaps)

**Performance Considerations**:
- Render at 10-30 FPS (sufficient for educational visualizations)
- Use requestAnimationFrame for smooth animations
- Limit simulation complexity (max 100 timesteps for RL demos)
- Precompute heavy calculations, display results incrementally

**Alternatives Considered**:
- **PyBullet/MuJoCo**: Rejected - Cannot run in browser, requires native compilation
- **Gazebo**: Rejected - Server-side only, too complex for educational use
- **p5.js**: Rejected - JavaScript-based, harder to integrate with Python exercises
- **Raw WebGL**: Rejected - Too low-level, development time prohibitive

### 7. Content Structure & Cognitive Load Management

**Question**: How do we structure 9 chapters with 3-5 lessons each to manage cognitive load at B1-B2 CEFR level?

**Decision**: Use 4-Layer Teaching Method with progressive complexity and explicit prerequisite mapping

**Rationale**:
- **Layer 1 (Foundation)**: Introduce concepts with definitions, analogies, visual aids
- **Layer 2 (Examples)**: Show worked examples with step-by-step explanations
- **Layer 3 (Practice)**: Guided exercises with hints and immediate feedback
- **Layer 4 (Application)**: Independent challenges applying multiple concepts

**Lesson Structure Pattern**:
1. **Learning Objectives** (CEFR-aligned, Bloom's taxonomy)
2. **Prerequisites** (explicit links to prior lessons)
3. **Concept Introduction** (Layer 1: 5-10 minutes)
4. **Worked Examples** (Layer 2: 10-15 minutes)
5. **Interactive Exercises** (Layer 3: 20-30 minutes)
6. **Challenge Problems** (Layer 4: 15-20 minutes)
7. **Summary & Next Steps**

**CEFR Mapping Strategy**:
- **B1 (Intermediate)**: Early chapters, basic concepts, familiar scenarios
- **B1+**: Mid-difficulty lessons, some technical terminology, structured tasks
- **B2 (Upper-Intermediate)**: Advanced chapters, complex concepts, independent problem-solving
- **B2+**: Final chapters, integration tasks, open-ended challenges

**Chapter Progression**:
- **Chapters 1-3**: Foundation (B1) - Introduction, kinematics, basic vision
- **Chapters 4-6**: Building (B1+ to B2) - Sensors, motion planning, RL intro
- **Chapters 7-9**: Advanced (B2 to B2+) - Humanoid control, multi-agent, real-world applications

**Alternatives Considered**:
- **Flat structure**: Rejected - No cognitive load management, students overwhelmed
- **Project-based**: Rejected - Difficult to assess prerequisite knowledge
- **Modular chunks**: Rejected - Loses narrative continuity

### 8. Skills Proficiency Mapping

**Question**: Which skills from .claude/skills/ apply to textbook content creation?

**Decision**: Use the following skills for content development workflow

**Applicable Skills**:

1. **book-scaffolding**: Overall textbook structure planning and part organization
   - Use for: Chapter-level planning, cognitive load management, prerequisite mapping

2. **learning-objectives**: Generate measurable learning outcomes aligned with CEFR and Bloom's taxonomy
   - Use for: Each lesson's learning objectives section

3. **quiz-generator**: Create 50-question interactive quizzes per chapter
   - Use for: End-of-chapter assessments with Quiz component

4. **code-example-generator**: Generate well-structured, testable Python code examples
   - Use for: InteractivePython exercises, worked examples

5. **exercise-designer**: Create progressive practice exercises with hints and solutions
   - Use for: Layer 3 practice problems

6. **concept-scaffolding**: Break complex concepts into digestible sub-concepts
   - Use for: Technical topics (inverse kinematics, RL algorithms, ZMP control)

7. **content-evaluation-framework**: Validate lesson quality before publication
   - Use for: Quality gates after each lesson/chapter completion

8. **skills-proficiency-mapper**: Map content to CEFR proficiency levels
   - Use for: Ensuring B1-B2 alignment across all lessons

9. **summary-generator**: Create lesson summaries extracting key concepts
   - Use for: End-of-lesson summaries, chapter reviews

**Workflow Integration**:
- Invoke `learning-objectives` at start of each lesson
- Use `code-example-generator` for InteractivePython components
- Apply `exercise-designer` for practice sections
- Run `quiz-generator` after completing all chapter lessons
- Execute `content-evaluation-framework` before marking chapter complete
- Use `skills-proficiency-mapper` to validate CEFR alignment

### 9. "Try With AI" Exercise Pattern

**Question**: How do we implement AI co-learning exercises where students work WITH AI tools?

**Decision**: Create "Try With AI" sections using prompt templates and AI tool integration guidance

**Rationale**:
- Constitution v4.0.1 Section IIb emphasizes AI Three Roles Framework
- Students learn to use AI as Teacher, Copilot, and Evaluator
- Prepares students for real-world AI-assisted development
- Balances independent coding with AI collaboration

**Try With AI Format**:
```markdown
## Try With AI

**Role**: [Teacher | Copilot | Evaluator]

**Scenario**: [Real-world problem context]

**Your Task**: [What student should attempt first]

**AI Prompt Template**:
> [Specific prompt for students to use with AI tools like ChatGPT, Claude, Copilot]

**Success Criteria**: [How to verify the AI-assisted solution works]

**Reflection Questions**:
1. What did the AI help you understand that was unclear before?
2. What part did you figure out independently?
3. How did you verify the AI's suggestions were correct?
```

**Example (from Chapter 2 - Robot Kinematics)**:
- **Role**: Copilot
- **Scenario**: Implementing inverse kinematics for a 3-DOF robot arm
- **Task**: Write the forward kinematics first, then ask AI to help derive inverse kinematics
- **AI Prompt**: "I have this forward kinematics function for a 3-DOF planar robot arm: [paste code]. Help me derive the inverse kinematics equations. Explain each step so I understand the geometric approach."
- **Success**: Student can explain the geometric reasoning, not just copy-paste code

**Frequency**: 1-2 "Try With AI" exercises per lesson (as specified in requirements)

### 10. YAML Frontmatter Structure for Lessons

**Question**: What metadata should each lesson file include for content management and skill tracking?

**Decision**: Use comprehensive YAML frontmatter with 7 generation fields + skills proficiency

**Structure**:
```yaml
---
# Core Metadata
title: "Lesson Title"
description: "One-sentence lesson summary"
chapter: 2
lesson: 3
estimated_time: 90  # minutes

# Skills Proficiency (CEFR + Bloom's + DigComp)
cefr_level: "B1+"  # A1, A2, B1, B1+, B2, B2+, C1, C2
blooms_level: "Apply"  # Remember, Understand, Apply, Analyze, Evaluate, Create
digcomp_level: 3  # 1-8 (Foundation to Highly Specialized)

# 7 Generation Fields (Constitution v4.0.1)
generation_type: "lesson"  # lesson | quiz | exercise | summary
content_stage: "draft"  # draft | review | validated | published
validation_status: "pending"  # pending | passed | failed
last_generated: "2025-11-29T12:00:00Z"
generator_version: "v2.1.0"
constitution_version: "v4.0.1"
quality_score: null  # 0-100, filled after validation

# Learning Metadata
prerequisites:
  - "chapter-01-lesson-02"
  - "chapter-02-lesson-01"
learning_objectives:
  - "Implement forward kinematics for 3-DOF robot arm"
  - "Visualize robot configurations in 2D workspace"
success_criteria:
  - "Student code correctly computes end-effector position"
  - "Visualization shows accurate robot poses"

# Content Components
has_interactive_python: true
interactive_python_count: 3
has_quiz_section: false  # Only true for chapter quizzes
has_try_with_ai: true
try_with_ai_count: 2

# Tags for Search/Filtering
tags:
  - "kinematics"
  - "forward-kinematics"
  - "robot-arm"
  - "2D-visualization"
---
```

**Rationale**:
- **7 Generation Fields**: Track content lifecycle for quality management
- **Skills Proficiency**: CEFR for language complexity, Bloom's for cognitive level, DigComp for digital competence
- **Prerequisites**: Enables dependency validation
- **Component Tracking**: Know what resources each lesson needs
- **Quality Score**: Populated by content-evaluation-framework skill

### 11. Validation Checkpoints

**Question**: How do we ensure quality at lesson and chapter boundaries?

**Decision**: Implement automated and manual validation gates

**Lesson-Level Validation** (after each lesson):
1. **Technical Validation**:
   - All InteractivePython code executes in Pyodide without errors
   - Code runtime < 30 seconds (constraint CC-003)
   - No external dependencies beyond NumPy, Matplotlib, SciPy

2. **Content Validation** (using content-evaluation-framework skill):
   - Learning objectives measurable and CEFR-aligned
   - All prerequisites listed and valid
   - Examples progress from simple to complex
   - Exercises have clear success criteria

3. **Accessibility Validation**:
   - All images have alt text (FR-028)
   - Code examples have descriptive labels
   - Mathematical notation renders correctly

4. **YAML Validation**:
   - All required frontmatter fields present
   - CEFR level matches content complexity
   - Bloom's level matches assessment type

**Chapter-Level Validation** (after all chapter lessons):
1. **Quiz Validation**:
   - Exactly 50 questions covering all chapter lessons
   - Questions distributed across Bloom's levels
   - All questions have explanations for correct AND incorrect answers

2. **Coherence Validation**:
   - Chapter narrative flows logically
   - Prerequisites form valid dependency graph (no cycles)
   - Complexity increases appropriately across lessons

3. **Skills Alignment**:
   - Success evals from spec achieved
   - Practical skills testable with provided exercises

4. **Duration Validation**:
   - Total chapter time 2-4 hours (constraint TC-007)
   - Individual lessons 60-120 minutes

**Validation Workflow**:
```bash
# Lesson validation
npm run validate:lesson book-source/docs/chapter-02/lesson-03.md

# Chapter validation
npm run validate:chapter book-source/docs/chapter-02/

# Full textbook validation
npm run validate:all
```

**Blocking vs Non-Blocking**:
- **Blocking**: Technical errors (code doesn't execute), missing required fields, accessibility violations
- **Non-Blocking**: Content suggestions, style improvements, optional enhancements

### 12. Docusaurus Configuration & Deployment

**Question**: What Docusaurus setup supports all interactive components and deployment requirements?

**Decision**: Docusaurus v3.x with custom plugins and components

**Configuration Decisions**:

**Plugins Required**:
- `@docusaurus/plugin-content-docs`: Main documentation
- `@docusaurus/plugin-ideal-image`: Optimized images
- `remark-math`: LaTeX math support
- `rehype-katex`: KaTeX rendering
- Custom plugin: `pyodide-loader` (preload Pyodide on homepage)

**Theme Configuration**:
- **Sidebar**: Auto-generated from chapter/lesson structure
- **Navbar**: Chapters dropdown, progress indicator, export button
- **Footer**: License, GitHub repo, feedback link
- **Color Mode**: Light/dark theme toggle

**MDX Components** (globally registered):
- `InteractivePython`: Code exercises
- `Quiz`: Chapter assessments
- `TryWithAI`: AI co-learning sections
- `LearningObjectives`: Formatted objectives lists
- `Prerequisites`: Prerequisite checklist with links

**Build Optimization**:
- Code splitting by chapter (lazy load chapters)
- Pyodide loaded once, cached
- Images optimized with ideal-image plugin
- Math rendered at build time (KaTeX)
- Bundle size target: < 100MB (constraint TC-005)

**Deployment**:
- Platform: GitHub Pages (or Netlify/Vercel)
- CI/CD: GitHub Actions for automatic deployment
- Custom domain: Optional
- HTTPS: Required for Pyodide (security requirement)

**Example docusaurus.config.js**:
```javascript
module.exports = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'Learn to build intelligent robots',
  url: 'https://yourusername.github.io',
  baseUrl: '/physical-ai-book/',

  presets: [
    [
      '@docusaurus/preset-classic',
      {
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          remarkPlugins: [require('remark-math')],
          rehypePlugins: [require('rehype-katex')],
        },
      },
    ],
  ],

  stylesheets: [
    {
      href: 'https://cdn.jsdelivr.net/npm/katex@0.16.9/dist/katex.min.css',
      type: 'text/css',
    },
  ],

  themes: ['@docusaurus/theme-live-codeblock'],

  plugins: [
    require.resolve('./plugins/pyodide-loader'),
  ],

  themeConfig: {
    navbar: {
      title: 'Physical AI Textbook',
      items: [
        {
          type: 'doc',
          docId: 'intro',
          position: 'left',
          label: 'Chapters',
        },
        {
          href: 'https://github.com/yourusername/physical-ai-book',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
  },
};
```

## Research Summary

All technology choices resolved:
- ✅ Pyodide for browser Python execution
- ✅ InteractivePython component with CodeMirror
- ✅ Existing Quiz component for assessments
- ✅ localStorage for progress tracking
- ✅ KaTeX for mathematical notation
- ✅ Matplotlib + Canvas for visualizations
- ✅ 4-Layer Teaching Method for content structure
- ✅ 9 applicable skills identified from .claude/skills/
- ✅ "Try With AI" pattern defined
- ✅ YAML frontmatter structure with 7 generation fields + proficiency
- ✅ Validation checkpoints at lesson and chapter boundaries
- ✅ Docusaurus v3.x configuration decided

**Ready for Phase 1**: Data model and contracts design can now proceed with all technical uncertainties resolved.
