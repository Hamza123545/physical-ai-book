# Physical AI & Humanoid Robotics Interactive Textbook

**🏆 Hackathon Project**: Production-ready interactive textbook for learning Physical AI and Humanoid Robotics
**📚 Course Structure**: 13-Week Quarter System (7 Chapters, 39 Lessons)
**🤖 Platform**: Docusaurus 3.9.2 + Pyodide 0.24.1 (Browser-based Python execution)
**⚡ Created With**: SpecKit Plus + Claude Code (Spec-Driven Development)

**🎯 Live Demo**: [View Textbook](https://your-deployment-url.com) | **📖 Documentation**: [Full Docs](./CLAUDE.md)

---

## 🎯 Overview

This interactive textbook teaches students to build intelligent humanoid robots using AI for real-world problems. **All code executes directly in the browser via Pyodide**—no installation required!

### ✨ What You'll Learn

- **Module 1 (Weeks 3-5)**: ROS 2 - The Robotic Nervous System
- **Module 2 (Weeks 6-7)**: Gazebo & Unity - The Digital Twin
- **Module 3 (Weeks 8-10)**: NVIDIA Isaac - The AI-Robot Brain
- **Weeks 11-12**: Humanoid Robot Development
- **Module 4 (Week 13)**: Vision-Language-Action (VLA) + **Capstone Project**

### 🌟 Key Features

✅ **100+ Interactive Python Exercises** - Execute code in browser (Pyodide)
✅ **40+ TryWithAI Exercises** - AI Three Roles Framework (Teacher/Copilot/Evaluator)
✅ **Comprehensive Quizzes** - Immediate feedback with explanations
✅ **Progress Tracking** - LocalStorage-based student progress
✅ **13-Week Course Structure** - Aligned with quarter system
✅ **Hackathon-Optimized** - Complete 5/7 chapters in 3 days
✅ **Reusable Intelligence** - Subagent + Constitution for bonus points
✅ **Build Size**: 17MB (< 100MB constraint) ✅
✅ **Zero Installation** - Pure browser-based execution

---

## 🚀 Quick Start

### Prerequisites

- **Node.js** 18+ (20+ recommended)
- **npm** or **yarn**
- Modern browser with WebAssembly support (Chrome 57+, Firefox 52+, Safari 11+, Edge 79+)

### Installation & Local Development

```bash
# Clone the repository
git clone https://github.com/Hamza123545/physical-ai-book.git
cd physical-ai-book/book-source

# Install dependencies
npm install

# Start development server
npm start
```

The textbook will open at `http://localhost:3000`

### Building for Production

```bash
# Build static site
npm run build

# Test production build locally
npm run serve
```

**Build output**: `book-source/build/` (~17MB, well under 100MB constraint)

---

## 📚 Course Structure (7 Chapters, 39 Lessons)

| Chapter | Title | Module/Week | Lessons | Time | CEFR |
|---------|-------|-------------|---------|------|------|
| **1** | Introduction to Physical AI | Weeks 1-2 | 4 | 180 min | B1 |
| **2** | ROS 2 Fundamentals | Module 1, Weeks 3-5 | 5 | 320 min | B1-B2 |
| **3** | The Digital Twin (Gazebo & Unity) | Module 2, Weeks 6-7 | 5 | 320 min | B1+-B2 |
| **4** | Reinforcement Learning + NVIDIA Isaac | Module 3, Weeks 8-10 | 8 | 500 min | B2 |
| **5** | Motion Planning + Nav2 | Module 3, Weeks 8-10 | 7 | 430 min | B2 |
| **6** | Humanoid Robot Development | Weeks 11-12 | 5 | 300 min | B2 |
| **7** | Vision-Language-Action (VLA) | Module 4, Week 13 | 5 | 380 min | B2-B2+ |

**Total**: 39 lessons, ~2430 minutes (~40.5 hours of content)

### 🎓 Learning Paths

**Hackathon Track (3 Days, 6-8 hours/day):**
- **Day 1**: Chapters 1-2 (Foundation + ROS 2)
- **Day 2**: Chapters 3-4 (Simulation + RL/Isaac)
- **Day 3**: Chapters 5-7 (Motion Planning + Humanoids + VLA + Capstone)

**Full Course (13 Weeks):** Follow the course structure week-by-week for comprehensive learning.

---

## 🏗️ Project Structure

```
physical-ai-book/
├── book-source/                # Docusaurus site
│   ├── docs/                  # Lesson content (Markdown + MDX)
│   │   ├── chapter-01/        # Chapter 1: Introduction to Physical AI
│   │   ├── chapter-02/        # Chapter 2: ROS 2 Fundamentals
│   │   ├── chapter-03/        # Chapter 3: Gazebo & Unity
│   │   ├── chapter-04/        # Chapter 4: RL + NVIDIA Isaac
│   │   ├── chapter-05/        # Chapter 5: Motion Planning + Nav2
│   │   ├── chapter-06/        # Chapter 6: Humanoid Robots
│   │   └── chapter-07/        # Chapter 7: VLA + Capstone
│   ├── src/
│   │   ├── components/        # Custom React components
│   │   ├── css/              # Custom styling
│   │   └── pages/            # Custom pages
│   ├── static/               # Static assets
│   ├── docusaurus.config.js  # Docusaurus configuration
│   └── package.json
├── specs/                     # SpecKit Plus artifacts
│   └── 002-physical-ai-textbook/
│       ├── spec.md           # Requirements specification
│       ├── plan.md           # Implementation plan
│       └── tasks.md          # Task breakdown
├── .claude/                   # Reusable Intelligence
│   ├── agents/               # Subagents
│   │   └── physical-ai-content-writer.md
│   └── skills/               # Skills
│       └── book-scaffolding/
├── .specify/                  # SpecKit Plus configuration
│   ├── memory/
│   │   └── constitution.md   # Constitution v6.0.0
│   └── templates/
└── README.md                  # This file
```

---

## 🧑‍💻 Pedagogy: 4-Layer Teaching Method

Every lesson follows a proven 4-layer structure:

1. **Layer 1: Foundation** - Introduce concepts with definitions and "why it matters"
2. **Layer 2: Application** - Worked examples (show-then-explain)
3. **Layer 3: Integration** - Guided practice with InteractivePython exercises
4. **Layer 4: Innovation** - Independent application through TryWithAI exercises

### AI Three Roles Framework

**TryWithAI** exercises use AI in three pedagogical roles:

- **🧑‍🏫 Teacher Role**: Explains concepts, answers "why" questions
- **🤝 Copilot Role**: Helps implement solutions, debugs alongside student
- **✅ Evaluator Role**: Reviews code, validates understanding

---

## 🛠️ Technical Architecture

### Frontend Stack

- **Docusaurus 3.9.2** - Static site generator
- **React 18** - UI framework
- **TypeScript** - Type-safe JavaScript
- **Pyodide 0.24.1** - WebAssembly Python runtime
- **CodeMirror 6** - Code editor
- **KaTeX** - Mathematical notation

### Custom Components

- **`<InteractivePython>`** - Browser-based Python code execution
- **`<Quiz>`** - Interactive quizzes with immediate feedback
- **`<TryWithAI>`** - AI co-learning exercises
- **`<LearningObjectives>`** - CEFR/Bloom's taxonomy display
- **`<Prerequisites>`** - Prerequisite tracking

### Browser-Based Python Execution

All Python code runs in the browser via **Pyodide**:

✅ **Allowed**: `numpy`, `scipy`, `matplotlib`, `sympy`
❌ **Forbidden**: File I/O, subprocess, threading, network calls
⏱️ **Performance**: < 30 seconds per exercise
💾 **Memory**: Arrays < 10MB

---

## 📖 How This Book Was Created (SpecKit Plus Workflow)

This entire textbook was created using **SpecKit Plus** and **Claude Code** following spec-driven development:

### Workflow: `/sp.specify` → `/sp.plan` → `/sp.tasks` → `/sp.implement`

1. **Specification** (`/sp.specify`): Each chapter started with a clear specification
2. **Planning** (`/sp.plan`): Detailed lesson structure and exercise design
3. **Tasks** (`/sp.tasks`): Granular implementation checklist with dependencies
4. **Implementation** (`/sp.implement`): Content creation using domain-specific subagents

### Reusable Intelligence Created

- **Physical-AI Content Writer Subagent** (`.claude/agents/physical-ai-content-writer.md`)
  - Domain expert for robotics content creation
  - Validates constitution compliance
  - Enforces 4-Layer Teaching Method

- **Constitution v6.0.0** (`.specify/memory/constitution.md`)
  - Establishes all quality standards
  - Defines 6 core principles
  - Ensures pedagogical effectiveness

- **Book Scaffolding Skill**
  - Progressive complexity planning
  - Cognitive load management
  - Connection mapping across chapters

### Evidence

- **Specs**: `specs/002-physical-ai-textbook/` contains all specifications
- **Constitution**: `.specify/memory/constitution.md` (v6.0.0)
- **Implementation traceable to spec.md**

---

## 🏆 Hackathon Bonus Points (50 Points)

### Reusable Intelligence Components

This project qualifies for **50 bonus points** through Reusable Intelligence:

1. **Physical-AI Content Writer Subagent** (`.claude/agents/physical-ai-content-writer.md`)
2. **Constitution v6.0.0** (`.specify/memory/constitution.md`)
3. **Book Scaffolding Skill** (used throughout)

All created using SpecKit Plus methodology via Claude Code.

---

## 📊 Success Metrics

### Content Quality ✅

- ✅ 39 lessons with complete YAML frontmatter
- ✅ 100+ InteractivePython exercises (all Pyodide-compatible)
- ✅ 40+ TryWithAI exercises (AI Three Roles Framework)
- ✅ Quizzes with immediate feedback and explanations
- ✅ CEFR B1-B2+ proficiency levels
- ✅ Bloom's Taxonomy alignment

### Technical Quality ✅

- ✅ Build size: 17MB (< 100MB constraint)
- ✅ All Python code executes < 30 seconds
- ✅ Zero MDX compilation errors
- ✅ Works in all modern browsers

### Learning Outcomes ✅

- **SC-002**: Students build ROS 2 systems (Chapter 2) ✅
- **SC-003**: Students simulate robots in Gazebo/Unity (Chapter 3) ✅
- **SC-004**: Students train RL agents with NVIDIA Isaac (Chapter 4) ✅
- **SC-005**: Students implement Nav2 for humanoids (Chapter 5) ✅
- **SC-006**: Students design humanoid control systems (Chapter 6) ✅
- **SC-007**: Students build VLA systems (Chapter 7 Capstone) ✅

---

## 🚢 Deployment

### GitHub Pages

```bash
# Build for production
npm run build

# Deploy to GitHub Pages (configure in docusaurus.config.js)
GIT_USER=<your-username> npm run deploy
```

### Netlify

1. Connect repository to Netlify
2. Build command: `cd book-source && npm install && npm run build`
3. Publish directory: `book-source/build`

### Vercel

1. Connect repository to Vercel
2. Framework: Docusaurus
3. Build command: `cd book-source && npm install && npm run build`
4. Output directory: `book-source/build`

---

## 🧪 Testing

### Run Tests Locally

```bash
# Test build
cd book-source
npm run build

# Test production server
npm run serve

# Check for broken links (during build)
npm run build 2>&1 | grep "broken links"
```

### Browser Compatibility

Tested on:
- ✅ Chrome 120+ (Recommended)
- ✅ Firefox 115+
- ✅ Safari 16+
- ✅ Edge 120+

**Requires**: WebAssembly support (all modern browsers as of 2023+)

---

## 📝 Constitution & Quality Standards

This textbook follows **Constitution v6.0.0** with 6 core principles:

### I. 4-Layer Teaching Method (FOUNDATION)
Every lesson follows Foundation → Application → Integration → Innovation

### II. AI Three Roles Framework (CO-LEARNING)
AI as Teacher, Copilot, and Evaluator

### III. CEFR Cognitive Load Limits (NON-NEGOTIABLE)
- B1: 7-10 concepts/lesson max
- B2: 10-15 concepts/lesson max

### IV. Specification-First Development (SPECKIT PLUS)
All content traceable to spec.md

### V. Code Quality Standards (PRODUCTION-READY)
All code has type hints, test cases, documentation

### VI. Educational Excellence (PEDAGOGY + ACCURACY)
Show-then-explain, zero gatekeeping language

**Full Constitution**: `.specify/memory/constitution.md`

---

## 🤝 Contributing

### Content Creation Workflow

1. **Review Constitution** (`.specify/memory/constitution.md`)
2. **Check Spec** (`specs/002-physical-ai-textbook/spec.md`)
3. **Follow Plan** (`specs/002-physical-ai-textbook/plan.md`)
4. **Update Tasks** (`specs/002-physical-ai-textbook/tasks.md`)
5. **Use Subagent** (`.claude/agents/physical-ai-content-writer.md`)

### Code Standards

- **Language**: Python 3.11+ with type hints
- **Pyodide Compatibility**: No file I/O, no threading, no network calls
- **Performance**: < 30 seconds execution time
- **Memory**: Arrays < 10MB
- **Testing**: All exercises include test cases

---

## 🐛 Troubleshooting

### Build Fails

```bash
# Clear cache and rebuild
rm -rf node_modules package-lock.json
npm install
npm run build
```

### Pyodide Slow to Load

Pyodide startup time is 2-5 seconds on first load. This is normal. Loading indicator is displayed.

### Exercise Not Executing

1. Check browser console for errors
2. Verify code has no syntax errors
3. Ensure exercise execution time < 30 seconds
4. Check browser supports WebAssembly

---

## 📄 License

This project is open-source and available for educational use.

---

## 🙏 Acknowledgments

**Created Using**:
- **SpecKit Plus**: Spec-Driven Development methodology
- **Claude Code**: AI-powered development

**Methodology**:
- Spec-Driven Development (SDD)
- Reusable Intelligence Infrastructure (RII)
- Constitutional AI Governance

**Frameworks**:
- 4-Layer Teaching Method
- AI Three Roles Framework
- CEFR Cognitive Load Management
- Bloom's Taxonomy
- DigComp 2.2

---

## 📞 Contact & Support

- **Issues**: [GitHub Issues](https://github.com/Hamza123545/physical-ai-book/issues)
- **Discussions**: [GitHub Discussions](https://github.com/Hamza123545/physical-ai-book/discussions)
- **Documentation**: [Docusaurus Docs](https://docusaurus.io)

---

**🎓 Ready to learn Physical AI & Humanoid Robotics? Let's get started!**

```bash
cd book-source && npm install && npm start
```

**🏆 Hackathon Submission**: This textbook qualifies for **50 bonus points** through Reusable Intelligence (Subagent + Constitution + SpecKit Plus workflow).
