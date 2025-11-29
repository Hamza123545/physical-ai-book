# Physical AI & Humanoid Robotics Textbook Constitution

**Created Using**: SpecKit Plus + Claude Code  
**Methodology**: Spec-Driven Development (spec→plan→tasks→implement)  
**Platform**: Docusaurus 3.9.2 + Pyodide 0.24.1

---

## Course Content Structure

### Required Chapters (13 Weeks)

This textbook follows a 13-week course structure covering Physical AI & Humanoid Robotics:

**Module 1: The Robotic Nervous System (ROS 2)** - Weeks 3-5
- `docs/ros2/nodes-topics-services.md`
- `docs/ros2/rclpy-python-agents.md`
- `docs/ros2/urdf-humanoid-description.md`
- `docs/ros2/launch-files-parameters.md`
- `docs/ros2/ros2-packages.md`

**Module 2: The Digital Twin (Gazebo & Unity)** - Weeks 6-7
- `docs/simulation/gazebo-physics-collisions.md`
- `docs/simulation/sensor-simulation-lidar-imu.md`
- `docs/simulation/unity-human-robot-interaction.md`
- `docs/simulation/urdf-sdf-formats.md`

**Module 3: The AI-Robot Brain (NVIDIA Isaac™)** - Weeks 8-10
- `docs/isaac/isaac-sim-setup.md`
- `docs/isaac/isaac-ros-vslam.md`
- `docs/isaac/nav2-path-planning.md`
- `docs/isaac/reinforcement-learning-robots.md`
- `docs/isaac/sim-to-real-transfer.md`

**Module 4: Vision-Language-Action (VLA)** - Week 13
- `docs/vla/voice-to-action-whisper.md`
- `docs/vla/cognitive-planning-llms.md`
- `docs/vla/multimodal-interaction.md`
- `docs/vla/capstone-autonomous-humanoid.md`

**Foundation Weeks 1-2: Introduction to Physical AI**
- `docs/intro/physical-ai-foundations.md`
- `docs/intro/embodied-intelligence.md`
- `docs/intro/humanoid-robotics-landscape.md`
- `docs/intro/sensor-systems-overview.md`

**Weeks 11-12: Humanoid Robot Development**
- `docs/humanoid/kinematics-dynamics.md`
- `docs/humanoid/bipedal-locomotion.md`
- `docs/humanoid/manipulation-grasping.md`
- `docs/humanoid/human-robot-interaction.md`

---

## Core Principles

### I. Phase-Driven Architecture

The project is split into **two distinct phases** to prevent scope creep and enable iterative delivery:

**PHASE 1 (Book Only)**:
- Docusaurus site with 7 chapters, 39 lessons, MDX pages, diagrams, quizzes, exercises
- Interactive Python components (Pyodide)
- Try With AI exercises
- Deploy to GitHub Pages
- **Points**: 100 (Base requirement)

**PHASE 2 (RAG + Features)**:
- FastAPI backend
- Qdrant vectors (RAG chatbot)
- Neon Postgres database
- Better Auth integration
- Content personalization
- Urdu translation
- Subagents (already complete ✅)
- **Points**: Up to 200 bonus points

**Rule**: Complete Phase 1 first (100 points). Phase 2 features are optional bonus (up to 200 bonus points).

---

### II. Spec-Driven Development (SDD)

Every deliverable (chapter, feature, API) starts with a **spec**.

**Chapter Spec Format**: `specs/chapters/<module>/<chapter-id>.spec.md`

**Feature Spec Format**: `specs/features/<feature-name>/spec.md`

**Process**: `Spec → Plan → Tasks → Code → Test → Verify`

**Backend Development Workflow**:
- **Skip**: `/sp.specify` and `/sp.clarify` (constitution already defines principles)
- **Use**: `/sp.plan` → `/sp.implement` (phase by phase)
- **Reference**: Backend Development Principles section in this constitution
- **Prompts**: Use `backend/PLAN-PROMPTS.md` and `backend/IMPLEMENT-PROMPTS.md`

---

### III. 4-Layer Teaching Method (FOUNDATION)

Every lesson MUST follow the 4-layer progressive structure:

- **Layer 1: Foundation** - Introduce concepts with definitions, intuition, and "why it matters"
- **Layer 2: Application** - Worked examples using show-then-explain pedagogy (concrete before abstract)
- **Layer 3: Integration** - Guided practice with InteractivePython exercises and scaffolded hints
- **Layer 4: Innovation** - Independent application through TryWithAI exercises (Teacher/Copilot/Evaluator roles)

**Rationale**: Cognitive science shows learners need concrete examples before abstract principles. Progressive complexity prevents cognitive overload.

---

### IV. AI Three Roles Framework (CO-LEARNING)

AI MUST be presented in three distinct pedagogical roles:

- **Teacher Role**: Explains concepts, answers "why" questions, provides mental models
- **Copilot Role**: Helps implement solutions, suggests approaches, debugs alongside student
- **Evaluator Role**: Reviews code, validates understanding, suggests improvements

**Implementation**: Every chapter includes TryWithAI exercises using at least 2 of the 3 roles.

**Rationale**: Bidirectional learning where AI teaches AND learns from student feedback creates deeper understanding.

---

### V. CEFR Cognitive Load Limits (NON-NEGOTIABLE)

Content complexity MUST respect cognitive load management:

- **B1 (Intermediate)**: 7-10 new concepts per lesson maximum
- **B1+ (Upper Intermediate)**: 8-12 new concepts per lesson maximum
- **B2 (Advanced)**: 10-15 new concepts per lesson maximum

**Scaffolding Requirements**:
- Heavy scaffolding: Step-by-step hints, fully worked examples, multiple test cases
- Moderate scaffolding: Partial hints, structure provided, 1-2 test cases
- Light scaffolding: Minimal hints, specification only, students design solution

**Rationale**: Exceeding cognitive load thresholds causes learner overwhelm and dropout.

---

### VI. Specification-First Development (SPECKIT PLUS)

All content MUST follow SpecKit Plus workflow using Claude Code:

**Workflow**: `/sp.specify` → `/sp.plan` → `/sp.tasks` → `/sp.implement`

1. **Spec Phase** (`/sp.specify`): Clear learning objectives, success criteria, prerequisite knowledge, module alignment
2. **Plan Phase** (`/sp.plan`): Lesson structure, exercise design, assessment strategy, file organization
3. **Tasks Phase** (`/sp.tasks`): Granular implementation checklist with dependencies, validation gates
4. **Implement Phase** (`/sp.implement`): Content creation following the plan, using domain-specific subagents and skills

**Tooling Requirements**:
- **SpecKit Plus**: https://github.com/panaversity/spec-kit-plus/
- **Claude Code**: https://www.claude.com/product/claude-code
- **Subagents**: `physical-ai-content-writer` for all content creation
- **Skills**: `robot-simulation-setup`, `sensor-integration`, `motion-planning`

**Non-Negotiable**: No implementation without approved spec. Specs are living documents. All content must be traceable to spec.md files.

---

### VII. Code Quality Standards (PRODUCTION-READY)

All code examples MUST meet production standards:

**Language**: Python 3.11+ with type hints
```python
def forward_kinematics(theta: np.ndarray, lengths: list[float]) -> tuple[float, float]:
    """Compute end-effector position from joint angles."""
    pass
```

**Pyodide Compatibility** (NON-NEGOTIABLE):
- ✅ Allowed: `numpy`, `scipy`, `matplotlib`, `sympy`
- ❌ Forbidden: File I/O, subprocess, threading, network calls, OpenCV, PyTorch, ROS 2, Gazebo, Isaac Sim (use simplified simulations)
- ⏱️ Execution: < 30 seconds per exercise
- 💾 Memory: Arrays < 10MB

**Note**: Full ROS 2, Gazebo, and Isaac Sim cannot run in Pyodide. Use educational simulations and simplified examples that demonstrate concepts without requiring full simulation environments.

**Testing**: Every code example includes test cases with expected output

**Documentation**: Comprehensive docstrings (Google style) for all functions

---

### VIII. Educational Excellence (PEDAGOGY + ACCURACY)

Content MUST balance pedagogical effectiveness with technical accuracy:

**Show-Then-Explain Pattern** (MANDATORY):
1. Show: Working example or visualization
2. Explain: Principles behind it
3. Practice: Guided exercise
4. Assess: Independent challenge

**Zero Gatekeeping Language**:
- ❌ FORBIDDEN: "Simply...", "Obviously...", "Just...", "Trivially...", "Anyone can..."
- ✅ REQUIRED: Define every technical term, explain why not just what, acknowledge difficulty

**Connection Mapping**:
- Prerequisites clearly stated at lesson start
- "Builds Toward" connections to future lessons
- Cross-chapter integration explicitly noted

---

## Quality Gates

### Validation-Auditor (MANDATORY)

All completed content MUST pass validation-auditor before publication:

- **Technical Correctness**: Formulas, algorithms, code accuracy verified
- **Code Execution**: All examples tested in Pyodide, no runtime errors
- **Pedagogical Effectiveness**: 4-Layer Method compliance, cognitive load checks
- **Constitution Alignment**: All 8 core principles verified

**Gate**: No content proceeds to publication without PASS status.

---

### Educational-Validator (REQUIRED)

Pedagogical review MUST validate:

- Learning objectives are measurable and aligned with Bloom's taxonomy
- Progressive complexity follows CEFR cognitive load limits
- Scaffolding appropriate for target proficiency level
- Exercises map to specific learning outcomes

---

### Factual-Verifier (REQUIRED)

Technical accuracy review MUST verify:

- Mathematical formulas are correct and properly notated
- Physical parameters are realistic for real robots
- Algorithm implementations match theoretical descriptions
- Citations provided for non-trivial claims

---

## Additional Standards

### Metadata Requirements

Every lesson file MUST include complete YAML frontmatter:

```yaml
---
title: "Lesson X.Y: Topic Name"
chapter: X
lesson: Y
estimated_time: 50-70
cefr_level: "B1" | "B1+" | "B2"
blooms_level: "Understand" | "Apply" | "Analyze" | "Evaluate"
digcomp_level: 3-6
generated_by: "claude-sonnet-4-5-20250929"
source_spec: "specs/002-physical-ai-textbook/spec.md"
created: "YYYY-MM-DD"
last_modified: "YYYY-MM-DD"
git_author: "hswat"
workflow: "/sp.implement"
version: "1.0"
prerequisites: ["chapter-XX-lesson-YY"]
has_interactive_python: true
interactive_python_count: 3-4
has_try_with_ai: true
try_with_ai_count: 1-2
tags: ["tag1", "tag2"]
---
```

---

### Assessment Standards

**Per-Chapter Quiz Requirements**:
- 10 questions (modified from original 50-question requirement)
- Distribution: 3 easy, 5 medium, 2 hard
- Coverage: All 5 lessons (2 questions per lesson)
- Explanations for correct AND incorrect answers
- Bloom's taxonomy alignment

---

### Accessibility Requirements

All content MUST be accessible:

- Alt text for all images and diagrams
- High contrast code examples
- Clear, jargon-free language at appropriate CEFR level
- Multiple learning modalities (text, code, visualization)
- Keyboard-navigable interactive components

---

## Governance

### Amendment Process

1. **Propose**: Document rationale, impact analysis, migration plan
2. **Review**: Validate with validation-auditor and educational-validator
3. **Approve**: Requires justification for complexity increase
4. **Migrate**: Update all existing content to comply

### Compliance Verification

**Pre-Commit**: All new content verified against constitution
**Post-Creation**: validation-auditor checks run automatically
**Quarterly Audit**: Full content review for constitution alignment

### Supremacy Clause

**This constitution supersedes all other practices, guidelines, and documentation.**

In case of conflict:
1. Constitution principles take precedence
2. Spec.md requirements second
3. Plan.md guidance third
4. All other documents fourth

---

## Reusable Intelligence Integration

### Subagents (Created via Claude Code)

**physical-ai-content-writer** (`.claude/agents/physical-ai-content-writer.md`):
- Domain expertise: ROS 2, Gazebo, NVIDIA Isaac, VLA, Humanoid Robotics, Kinematics, Control, Vision, RL
- Usage: All content creation, technical review, code validation
- Alignment: Full constitution compliance enforced
- Created: Using SpecKit Plus workflow via Claude Code

### Skills (Created via Claude Code)

**robot-simulation-setup** (`.claude/skills/robot-simulation-setup/SKILL.md`):
- Pyodide-compatible robot simulation code
- Educational kinematics/dynamics visualizations
- Created: Using SpecKit Plus workflow

**sensor-integration** (`.claude/skills/sensor-integration/SKILL.md`):
- Sensor fusion, IMU, LiDAR simulation examples
- Pyodide-compatible sensor processing code
- Created: Using SpecKit Plus workflow

**motion-planning** (`.claude/skills/motion-planning/SKILL.md`):
- Path planning (A*, RRT), PID control, trajectory optimization
- Pyodide-compatible planning algorithms
- Created: Using SpecKit Plus workflow

**book-scaffolding** (`.claude/skills/book-scaffolding`):
- Progressive complexity planning
- Cognitive load management
- Connection mapping across chapters
- Show-then-explain pattern enforcement

---

---

## Backend Development Principles

### Backend Architecture Standards

**Framework**: FastAPI (Python 3.11+) with async/await for all I/O operations

**Database Strategy**:
- **Neon Postgres**: Relational data (users, chat history, translations, user backgrounds)
- **Qdrant Cloud**: Vector embeddings for RAG chatbot (semantic search)
- Connection pooling for Postgres
- Proper migrations using Alembic
- Index frequently queried columns

**Security Requirements** (NON-NEGOTIABLE):
- Environment variables for all secrets (never hardcode)
- CORS properly configured for Docusaurus frontend
- Rate limiting on all public endpoints
- Input validation on all requests (Pydantic models)
- SQL injection prevention (parameterized queries only)

**Error Handling Standards**:
- Consistent error response format: `{"success": false, "error": {"code": "...", "message": "..."}}`
- Proper HTTP status codes (200, 201, 400, 401, 404, 500)
- Structured logging for debugging
- Graceful degradation with fallbacks where possible

**Code Organization**:
```
backend/
├── app/
│   ├── main.py          # FastAPI app
│   ├── config.py        # Configuration (env vars)
│   ├── models/          # Database models (SQLAlchemy/Pydantic)
│   ├── api/             # API routes
│   ├── services/        # Business logic
│   └── utils/           # Utilities
├── requirements.txt
└── .env.example
```

**API Design Standards**:
- RESTful conventions: `/api/{resource}`, `/api/{resource}/{id}`, `/api/{resource}/{id}/{action}`
- Success response: `{"success": true, "data": {...}, "message": "..."}`
- Error response: `{"success": false, "error": {"code": "...", "message": "...", "details": {...}}}`
- Status codes: 200 (Success), 201 (Created), 400 (Bad Request), 401 (Unauthorized), 404 (Not Found), 500 (Internal Error)

**Performance Requirements**:
- Response time < 3 seconds for typical queries
- Cache frequently accessed data (translations, personalized content)
- Batch operations where possible
- Optimize database queries (avoid N+1 problems)

**Testing Requirements**:
- Unit tests for services (mock external APIs)
- Integration tests for API endpoints
- Test coverage > 70%
- Mock external services (OpenAI, Qdrant, Better Auth)

**Documentation Requirements**:
- Docstrings for all functions/classes (Google style)
- API documentation via FastAPI (OpenAPI/Swagger)
- README with setup instructions
- Environment variables documented in `.env.example`

**File Naming Conventions**:
- Python files: `snake_case.py`
- API routes: `{resource}_routes.py`
- Services: `{feature}_service.py`
- Models: `{table}_model.py`
- Tests: `test_{module}.py`

**Backend Features** (Hackathon Requirements):
1. **RAG Chatbot** (Base - 100 points): FastAPI + OpenAI + Qdrant + Neon Postgres
2. **Better Auth** (Bonus - 50 points): Better Auth integration with user background collection
3. **Content Personalization** (Bonus - 50 points): LLM-based content adaptation based on user background
4. **Urdu Translation** (Bonus - 50 points): On-demand translation with caching

**Note**: Feature 2 (Subagents) already complete ✅ - Existing `.claude/agents/` and `.claude/skills/` fulfill requirement.

---

## Development Methodology

### SpecKit Plus Workflow

This entire textbook was created using **SpecKit Plus** (https://github.com/panaversity/spec-kit-plus/) and **Claude Code** (https://www.claude.com/product/claude-code).

**Process**:
1. Each module/chapter started with `/sp.specify` command
2. Planning done via `/sp.plan` with module structure
3. Tasks broken down via `/sp.tasks` with dependencies
4. Implementation via `/sp.implement` using domain-specific subagents

**Backend Development Workflow**:
- **Skip**: `/sp.specify` and `/sp.clarify` (constitution already defines principles)
- **Use**: `/sp.plan` → `/sp.implement` (phase by phase)
- **Reference**: Backend Development Principles section in this constitution
- **Prompts**: Use `backend/PLAN-PROMPTS.md` and `backend/IMPLEMENT-PROMPTS.md`

**Evidence**:
- All specs stored in `specs/` directory
- All plans in `specs/[module-name]/plan.md`
- All tasks in `specs/[module-name]/tasks.md`
- Implementation history in `history/prompts/`
- Backend prompts in `backend/PLAN-PROMPTS.md` and `backend/IMPLEMENT-PROMPTS.md`

**Reusable Intelligence**:
- Domain-specific subagent: `physical-ai-content-writer`
- Domain-specific skills: `robot-simulation-setup`, `sensor-integration`, `motion-planning`
- All created using SpecKit Plus methodology

---

**Version**: 6.1.0  
**Ratified**: 2025-11-29  
**Last Amended**: 2025-11-29  
**Created Using**: SpecKit Plus + Claude Code  
**Project**: Physical AI & Humanoid Robotics Interactive Textbook  
**Platform**: Docusaurus 3.9.2 + Pyodide 0.24.1  
**Course Duration**: 13 Weeks (Quarter System)
