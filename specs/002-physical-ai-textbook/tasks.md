# Tasks: Physical AI & Humanoid Robotics Textbook

**Input**: Design documents from `/specs/002-physical-ai-textbook/`  
**Created Using**: SpecKit Plus `/sp.tasks` command via Claude Code  
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/  
**Course Structure**: 13 Weeks (Quarter System) - Modules 1-4 + Foundation Weeks

**Tests**: Not requested for this educational content project - focus is on content creation and validation

**Organization**: Tasks are grouped by course modules (13-week structure):
- **Weeks 1-2**: Introduction to Physical AI
- **Module 1 (Weeks 3-5)**: ROS 2 - The Robotic Nervous System
- **Module 2 (Weeks 6-7)**: Gazebo & Unity - The Digital Twin
- **Module 3 (Weeks 8-10)**: NVIDIA Isaac - The AI-Robot Brain
- **Weeks 11-12**: Humanoid Robot Development
- **Module 4 (Week 13)**: Vision-Language-Action (VLA) + Capstone

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story/chapter group this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

All paths relative to `book-source/` (Docusaurus site):
- **Content**: `docs/chapter-XX/`
- **Components**: `src/components/`
- **Configuration**: Root level (`docusaurus.config.js`, `sidebars.js`, etc.)

---

## Phase 1: Setup (Infrastructure & Foundation)

**Purpose**: Initialize Docusaurus project and create custom React components for interactive learning

**Estimated Time**: 16-20 hours (Week 1)

- [X] T001 Initialize Docusaurus v3.x project in book-source/ directory
- [X] T002 Configure package.json with Docusaurus v3.x, React 18, TypeScript dependencies
- [X] T003 Configure docusaurus.config.js with site metadata, Pyodide CDN, KaTeX plugins
- [X] T004 [P] Setup TypeScript configuration in book-source/tsconfig.json
- [X] T005 [P] Create basic folder structure per plan.md (docs/, src/components/, static/, plugins/)
- [X] T006 [P] Install and configure remark-math and rehype-katex plugins for mathematical notation
- [X] T007 [P] Add KaTeX CSS stylesheet to docusaurus.config.js stylesheets array
- [X] T008 Create InteractivePython component in book-source/src/components/InteractivePython.tsx
- [X] T009 Configure CodeMirror 6 editor with Python syntax highlighting in InteractivePython component
- [X] T010 Integrate Pyodide runtime loading in InteractivePython component with loading indicator
- [X] T011 Implement code execution logic with stdout/stderr capture in InteractivePython.tsx
- [X] T012 Add test case validation logic to InteractivePython component
- [X] T013 Implement localStorage save/load for student code in InteractivePython.tsx
- [X] T014 Add reset button to restore starter code in InteractivePython component
- [X] T015 Create TryWithAI component in book-source/src/components/TryWithAI.tsx
- [X] T016 Implement AI Three Roles Framework display (Teacher/Copilot/Evaluator) in TryWithAI.tsx
- [X] T017 Add prompt template and reflection questions display to TryWithAI component
- [X] T018 Create LearningObjectives component in book-source/src/components/LearningObjectives.tsx
- [X] T019 Add CEFR and Bloom's taxonomy display to LearningObjectives component
- [X] T020 Create Prerequisites component in book-source/src/components/Prerequisites.tsx
- [X] T021 Implement prerequisite checklist with completion tracking in Prerequisites.tsx
- [X] T022 [P] Register all custom components globally in book-source/src/theme/MDXComponents.tsx
- [X] T023 [P] Create component usage documentation in book-source/src/components/references/COMPONENT_USAGE.md
- [X] T024 Create progress tracking utility in book-source/src/utils/progressTracker.ts
- [X] T025 Implement localStorage interface for StudentProgress in progressTracker.ts
- [X] T026 Add export/import functionality for progress data in progressTracker.ts
- [X] T027 [P] Create custom CSS in book-source/src/css/custom.css for component styling
- [X] T028 [P] Setup Pyodide preloader plugin in book-source/plugins/pyodide-loader/
- [X] T029 Configure sidebars.js with chapter structure and navigation
- [X] T030 Create homepage in book-source/docs/intro.md with textbook overview
- [X] T031 Test Docusaurus build and dev server with all components
- [X] T032 Verify Pyodide loads and executes simple Python code in browser

**Checkpoint**: Infrastructure complete - all components functional, Docusaurus builds successfully

---

## Phase 2: Foundational Content (Chapter 1 & Shared Foundation)

**Course Mapping**: Weeks 1-2 - Introduction to Physical AI  
**Module**: Foundation Weeks (Pre-Module 1)  
**Purpose**: Create foundational chapter (Chapter 1) that establishes prerequisite knowledge for ALL other chapters

**User Story Mapping**: Maps to US6 (Sequential Learning) - provides foundation for all subsequent user stories

**Estimated Time**: 12-16 hours

**⚠️ CRITICAL**: This phase MUST be complete before any specialized chapters (robot arm, vision, RL, humanoid) can be implemented

- [X] T033 Create Chapter 1 directory structure at book-source/docs/chapter-01/
- [X] T034 Create Chapter 1 index page at book-source/docs/chapter-01/index.md with chapter overview
- [X] T035 [US6] Write Lesson 1.1 content in book-source/docs/chapter-01/lesson-01-what-is-physical-ai.md
- [X] T036 [US6] Add YAML frontmatter to Lesson 1.1 with all 7 generation fields + proficiency (CEFR: B1, Bloom's: Understand)
- [X] T037 [P] [US6] Create Exercise 1.1.1 (sensor simulation) with starter code, test cases, hints
- [X] T038 [P] [US6] Create Exercise 1.1.2 (moving average filter) with starter code, test cases
- [X] T039 [P] [US6] Create Exercise 1.1.3 (threshold control) with starter code, test cases
- [X] T040 [P] [US6] Create TryWithAI 1.1.1 (Teacher role - Physical AI applications) in Lesson 1.1
- [X] T041 [P] [US6] Create TryWithAI 1.1.2 (Evaluator role - sensor code review) in Lesson 1.1
- [X] T042 [US6] Test all Lesson 1.1 exercises execute in Pyodide < 30 seconds
- [X] T043 [US6] Write Lesson 1.2 content in book-source/docs/chapter-01/lesson-02-robot-vs-software-ai.md
- [X] T044 [US6] Add YAML frontmatter to Lesson 1.2 (CEFR: B1, Bloom's: Analyze)
- [X] T045 [P] [US6] Create Exercise 1.2.1 (real-time constraint simulation) with starter code, test cases
- [X] T046 [P] [US6] Create Exercise 1.2.2 (sensor noise handling) with starter code, test cases
- [X] T047 [P] [US6] Create Exercise 1.2.3 (safety monitor) with starter code, test cases
- [X] T048 [P] [US6] Create TryWithAI 1.2.1 (Copilot role - safety system design) in Lesson 1.2
- [X] T049 [US6] Test all Lesson 1.2 exercises execute in Pyodide < 30 seconds
- [X] T050 [US6] Write Lesson 1.3 content in book-source/docs/chapter-01/lesson-03-sensors-actuators-overview.md
- [X] T051 [US6] Add YAML frontmatter to Lesson 1.3 (CEFR: B1, Bloom's: Understand, DigComp: 3)
- [X] T052 [P] [US6] Create Exercise 1.3.1 (encoder ticks to angle conversion) with starter code, test cases
- [X] T053 [P] [US6] Create Exercise 1.3.2 (IMU tilt calculation) with starter code, test cases
- [X] T054 [P] [US6] Create Exercise 1.3.3 (servo PWM control) with starter code, test cases
- [X] T055 [P] [US6] Create TryWithAI 1.3.1 (Teacher role - sensor selection) in Lesson 1.3
- [X] T056 [P] [US6] Create TryWithAI 1.3.2 (Copilot role - IMU calibration) in Lesson 1.3
- [X] T057 [US6] Test all Lesson 1.3 exercises execute in Pyodide < 30 seconds
- [X] T058 [US6] Write Lesson 1.4 content in book-source/docs/chapter-01/lesson-04-python-robotics-intro.md
- [X] T059 [US6] Add YAML frontmatter to Lesson 1.4 (CEFR: B1, Bloom's: Apply, DigComp: 3)
- [X] T060 [P] [US6] Create Exercise 1.4.1 (NumPy arrays) with starter code, test cases
- [X] T061 [P] [US6] Create Exercise 1.4.2 (Matplotlib plotting) with starter code, test cases
- [X] T062 [P] [US6] Create Exercise 1.4.3 (control loop) with starter code, test cases
- [X] T063 [P] [US6] Create Exercise 1.4.4 (robot simulation) with starter code, test cases
- [X] T064 [P] [US6] Create TryWithAI 1.4.1 (Copilot role - NumPy vectorization) in Lesson 1.4
- [X] T065 [P] [US6] Create TryWithAI 1.4.2 (Evaluator role - control loop review) in Lesson 1.4
- [X] T066 [US6] Test all Lesson 1.4 exercises execute in Pyodide < 30 seconds
- [X] T067 [US6] Generate Chapter 1 Quiz (10 questions covering all lessons)
- [X] T068 [US6] Create quiz.md at book-source/docs/chapter-01/quiz.md with Quiz component
- [X] T069 [US6] Verify all quiz questions have explanations for correct AND incorrect answers
- [X] T070 [US6] Test Quiz component displays questions correctly
- [X] T071 [US6] Validate Chapter 1: All lessons complete, 13 exercises functional, 6 TryWithAI exercises, quiz complete
- [X] T072 [US6] Update sidebars.js with Chapter 1 navigation structure (auto-generated)

**Checkpoint**: Chapter 1 complete - students can learn Physical AI foundations and basic Python robotics. All subsequent chapters can now proceed in parallel.

---

## Phase 3: User Story 1 - ROS 2 Fundamentals (Chapter 2) 🎯 MVP

**Course Mapping**: Module 1 (Weeks 3-5) - ROS 2 - The Robotic Nervous System  
**Goal**: Students can complete Chapter 2 and build complete ROS 2 systems with nodes, topics, services, and actions

**Independent Test**: Students write ROS 2 code using rclpy that successfully creates nodes, publishes/subscribes to topics, implements services and actions, builds ROS 2 packages, and models humanoids with URDF

**Priority**: P1 (MUST-HAVE)
**Estimated Time**: 20-24 hours
**Dependencies**: Phase 2 (Chapter 1) complete

### Implementation for User Story 1 (Chapter 2: ROS 2 Fundamentals)

- [X] T073 Create Chapter 2 directory structure at book-source/docs/chapter-02/
- [X] T074 Create Chapter 2 index page at book-source/docs/chapter-02/index.md
- [X] T075 [US1] Write Lesson 2.1 (ROS 2 Architecture) in book-source/docs/chapter-02/lesson-01-ros2-architecture.md
- [X] T076 [US1] Add YAML frontmatter to Lesson 2.1 with SpecKit Plus metadata (CEFR: B1, Bloom's: Understand, DigComp: 3)
- [X] T077 [P] [US1] Create Exercise 2.1.1 (ROS 2 architecture concepts) with starter code, test cases
- [X] T078 [P] [US1] Create Exercise 2.1.2 (communication patterns) with starter code, test cases
- [X] T079 [P] [US1] Create Exercise 2.1.3 (message flow simulation) with starter code
- [X] T080 [P] [US1] Create TryWithAI 2.1.1 (Teacher role - ROS 2 system design) in Lesson 2.1
- [X] T081 [US1] Test all Lesson 2.1 exercises execute in Pyodide < 30 seconds
- [X] T082 [US1] Write Lesson 2.2 (Nodes and Topics) in book-source/docs/chapter-02/lesson-02-nodes-topics.md
- [X] T083 [US1] Add YAML frontmatter to Lesson 2.2 with SpecKit Plus metadata (CEFR: B1+, Bloom's: Apply, DigComp: 4)
- [X] T084 [P] [US1] Create Exercise 2.2.1 (simple publisher) with starter code, test cases
- [X] T085 [P] [US1] Create Exercise 2.2.2 (subscriber with callback) with starter code, test cases
- [X] T086 [P] [US1] Create Exercise 2.2.3 (sensor-controller system) with starter code, test cases
- [X] T087 [P] [US1] Create Exercise 2.2.4 (message type conversion) with starter code, test cases
- [X] T088 [P] [US1] Create TryWithAI 2.2.1 (Copilot role - sensor node design) in Lesson 2.2
- [X] T089 [P] [US1] Create TryWithAI 2.2.2 (Evaluator role - debug subscriber) in Lesson 2.2
- [X] T090 [US1] Test all Lesson 2.2 exercises execute in Pyodide < 30 seconds
- [X] T091 [US1] Write Lesson 2.3 (Services and Actions) in book-source/docs/chapter-02/lesson-03-services-actions.md
- [X] T092 [US1] Add YAML frontmatter to Lesson 2.3 with SpecKit Plus metadata (CEFR: B1+, Bloom's: Apply, DigComp: 4)
- [X] T093 [P] [US1] Create Exercise 2.3.1 (service request-response) with starter code, test cases
- [X] T094 [P] [US1] Create Exercise 2.3.2 (action progress tracking) with starter code, test cases
- [X] T095 [P] [US1] Create Exercise 2.3.3 (service vs action selection) with starter code, test cases
- [X] T096 [P] [US1] Create TryWithAI 2.3.1 (Copilot role - service interface design) in Lesson 2.3
- [X] T097 [US1] Test all Lesson 2.3 exercises execute in Pyodide < 30 seconds
- [X] T098 [US1] Write Lesson 2.4 (ROS 2 Packages with rclpy) in book-source/docs/chapter-02/lesson-04-rclpy-packages.md
- [X] T099 [US1] Add YAML frontmatter to Lesson 2.4 with SpecKit Plus metadata (CEFR: B2, Bloom's: Apply, DigComp: 5)
- [X] T100 [P] [US1] Create Exercise 2.4.1 (package structure design) with starter code, test cases
- [X] T101 [P] [US1] Create Exercise 2.4.2 (entry points configuration) with starter code, test cases
- [X] T102 [P] [US1] Create Exercise 2.4.3 (dependency management) with starter code, test cases
- [X] T103 [P] [US1] Create Exercise 2.4.4 (Python agent bridge) with starter code, test cases
- [X] T104 [P] [US1] Create TryWithAI 2.4.1 (Evaluator role - package design review) in Lesson 2.4
- [X] T105 [P] [US1] Create TryWithAI 2.4.2 (Copilot role - integrate Python agent) in Lesson 2.4
- [X] T106 [US1] Test all Lesson 2.4 exercises execute in Pyodide < 30 seconds
- [X] T107 [US1] Write Lesson 2.5 (URDF and Launch Files) in book-source/docs/chapter-02/lesson-05-urdf-launch.md
- [X] T108 [US1] Add YAML frontmatter to Lesson 2.5 with SpecKit Plus metadata (CEFR: B2, Bloom's: Apply, DigComp: 5)
- [X] T109 [P] [US1] Create Exercise 2.5.1 (create URDF for robot arm) with starter code, test cases
- [X] T110 [P] [US1] Create Exercise 2.5.2 (launch file structure) with starter code, test cases
- [X] T111 [P] [US1] Create Exercise 2.5.3 (joint limits configuration) with starter code, test cases
- [X] T112 [P] [US1] Create TryWithAI 2.5.1 (Copilot role - humanoid URDF design) in Lesson 2.5
- [X] T113 [US1] Test all Lesson 2.5 exercises execute in Pyodide < 30 seconds
- [X] T114 [US1] Generate Chapter 2 Quiz (10 questions covering ROS 2 concepts)
- [X] T115 [US1] Create quiz.md at book-source/docs/chapter-02/quiz.md with Quiz component
- [X] T116 [US1] Verify Chapter 2 quiz questions cover nodes, topics, services, actions, packages, URDF
- [X] T117 [US1] Ensure quiz Bloom's distribution across cognitive levels
- [X] T118 [US1] Test Quiz component with Chapter 2 questions
- [X] T119 [US1] Validate Chapter 2: All 5 lessons complete, 17 exercises functional, 7 TryWithAI exercises, 10-question quiz
- [X] T120 [US1] Update sidebars.js with Chapter 2 navigation (auto-generated by Docusaurus)

**Checkpoint**: User Story 1 COMPLETE - Students can build ROS 2 systems (Success Eval SC-002 met)

---

## Phase 4: User Story 2 - The Digital Twin (Gazebo & Unity) (Chapter 3)

**Course Mapping**: Module 2 (Weeks 6-7) - The Digital Twin (Gazebo & Unity)  
**Goal**: Students can complete Chapter 3 and simulate robots with Gazebo physics and create high-fidelity visualizations with Unity

**Independent Test**: Students create robot models in URDF/SDF, simulate physics in Gazebo, simulate sensors (LiDAR, cameras, IMUs), and create Unity visualizations for human-robot interaction

**Priority**: P1 (MUST-HAVE)
**Estimated Time**: 20-24 hours
**Dependencies**: Phase 2 (Chapter 1) complete; Can run in parallel with Phase 3

### Implementation for User Story 2 (Chapter 3: The Digital Twin - Gazebo & Unity)

- [X] T123 Create Chapter 3 directory structure at book-source/docs/chapter-03/
- [X] T124 Create Chapter 3 index page at book-source/docs/chapter-03/index.md
- [X] T125 [US2] Write Lesson 3.1 (Gazebo Simulation Environment) in book-source/docs/chapter-03/lesson-01-gazebo-intro.md
- [X] T126 [US2] Add YAML frontmatter to Lesson 3.1 with SpecKit Plus metadata (CEFR: B1, Bloom's: Understand, DigComp: 3)
- [X] T127 [P] [US2] Create Exercise 3.1.1 (create simple world) with starter code, test cases
- [X] T128 [US2] Test all Lesson 3.1 exercises execute in Pyodide < 30 seconds
- [X] T129 [US2] Write Lesson 3.2 (URDF and SDF) in book-source/docs/chapter-03/lesson-02-urdf-sdf.md
- [X] T130 [US2] Add YAML frontmatter to Lesson 3.2 with SpecKit Plus metadata (CEFR: B1+, Bloom's: Apply, DigComp: 4)
- [X] T131 [P] [US2] Create Exercise 3.2.1 (create robot arm URDF) with starter code, test cases
- [X] T132 [US2] Test all Lesson 3.2 exercises execute in Pyodide < 30 seconds
- [X] T133 [US2] Write Lesson 3.3 (Physics Simulation) in book-source/docs/chapter-03/lesson-03-physics-simulation.md
- [X] T134 [US2] Add YAML frontmatter to Lesson 3.3 with SpecKit Plus metadata (CEFR: B1+, Bloom's: Apply, DigComp: 4)
- [X] T135 [P] [US2] Create Exercise 3.3.1 (falling object simulation) with starter code, test cases
- [X] T136 [US2] Test all Lesson 3.3 exercises execute in Pyodide < 30 seconds
- [X] T137 [US2] Write Lesson 3.4 (Sensor Simulation) in book-source/docs/chapter-03/lesson-04-sensor-simulation.md
- [X] T138 [US2] Add YAML frontmatter to Lesson 3.4 with SpecKit Plus metadata (CEFR: B2, Bloom's: Apply, DigComp: 5)
- [X] T139 [P] [US2] Create Exercise 3.4.1 (LiDAR obstacle detection) with starter code, test cases
- [X] T140 [US2] Test all Lesson 3.4 exercises execute in Pyodide < 30 seconds
- [X] T141 [US2] Write Lesson 3.5 (Unity Rendering) in book-source/docs/chapter-03/lesson-05-unity-rendering.md
- [X] T142 [US2] Add YAML frontmatter to Lesson 3.5 with SpecKit Plus metadata (CEFR: B2, Bloom's: Apply, DigComp: 5)
- [X] T143 [P] [US2] Create Exercise 3.5.1 (ROS 2 Unity bridge) with starter code, test cases
- [X] T144 [US2] Test all Lesson 3.5 exercises execute in Pyodide < 30 seconds
- [X] T145 [US2] Generate Chapter 3 Quiz (10 questions covering Gazebo and Unity)
- [X] T146 [US2] Create quiz.md at book-source/docs/chapter-03/quiz.md with Quiz component
- [X] T147 [US2] Verify Chapter 3 quiz aligns with Success Eval SC-003 (simulation capabilities)
- [X] T148 [US2] Test Quiz component with all Chapter 3 questions
- [X] T149 [US2] Validate Chapter 3: All 5 lessons complete, exercises functional, 10-question quiz
- [X] T150 [US2] Update sidebars.js with Chapter 3 navigation (auto-generated by Docusaurus)

**Checkpoint**: User Story 2 COMPLETE - Students can simulate robots with Gazebo and Unity (Success Eval SC-003 met)

---

## Phase 5: User Story 3 - Reinforcement Learning + NVIDIA Isaac (Chapter 4)

**Course Mapping**: Module 3 (Weeks 8-10) - The AI-Robot Brain (NVIDIA Isaac™)  
**Note**: Chapter 4 covers RL fundamentals AND NVIDIA Isaac integration (Isaac Sim, Isaac ROS, VSLAM, sim-to-real transfer) as specified in hackathon requirements.

**Goal**: Provide sensor fusion and motion planning knowledge to support all advanced applications

**Independent Test**: Students can complete Chapters 4-5 and apply sensor fusion with motion planning for robust robot control

**Priority**: P1 (MUST-HAVE) - Foundation for US3 and US4
**Estimated Time**: 16-20 hours
**Dependencies**: Phase 2 (Chapter 1) complete; Should precede Phases 6-7

### Implementation for User Story 6 - Chapter 4 (Reinforcement Learning for Robotics)

**Note**: Tasks below reference "Sensor Integration" but Chapter 4 is actually "Reinforcement Learning" based on existing content.

- [ ] T173 Create Chapter 4 directory structure at book-source/docs/chapter-04/
- [ ] T174 Create Chapter 4 index page at book-source/docs/chapter-04/index.md
- [ ] T175 [US6] Write Lesson 4.1 (Sensor Noise and Filtering) in book-source/docs/chapter-04/lesson-01-sensor-noise-filtering.md
- [ ] T176 [US6] Add YAML frontmatter to Lesson 4.1 (CEFR: B1+, Bloom's: Apply, DigComp: 4)
- [ ] T177 [P] [US6] Create 3 exercises for Lesson 4.1 (Gaussian noise, low-pass filter, moving average) with starter code, test cases
- [ ] T178 [P] [US6] Create 1 TryWithAI exercise for Lesson 4.1
- [ ] T179 [US6] Write Lesson 4.2 (Sensor Fusion Basics) in book-source/docs/chapter-04/lesson-02-sensor-fusion.md
- [ ] T180 [US6] Add YAML frontmatter to Lesson 4.2 (CEFR: B1+, Bloom's: Apply, DigComp: 4)
- [ ] T181 [P] [US6] Create 4 exercises for Lesson 4.2 (complementary filter, weighted fusion, IMU+encoder) with starter code, test cases
- [ ] T182 [P] [US6] Create 2 TryWithAI exercises for Lesson 4.2
- [ ] T183 [US6] Write Lesson 4.3 (Robot Localization) in book-source/docs/chapter-04/lesson-03-localization.md
- [ ] T184 [US6] Add YAML frontmatter to Lesson 4.3 (CEFR: B2, Bloom's: Apply, DigComp: 5)
- [ ] T185 [P] [US6] Create 3 exercises for Lesson 4.3 (dead reckoning, odometry, drift correction) with starter code, test cases
- [ ] T186 [P] [US6] Create 1 TryWithAI exercise for Lesson 4.3
- [ ] T187 [US6] Write Lesson 4.4 (SLAM Introduction) in book-source/docs/chapter-04/lesson-04-slam-intro.md
- [ ] T188 [US6] Add YAML frontmatter to Lesson 4.4 (CEFR: B2, Bloom's: Understand, DigComp: 5)
- [ ] T189 [P] [US6] Create 3 exercises for Lesson 4.4 (occupancy grid, simple SLAM) with starter code, test cases
- [ ] T190 [P] [US6] Create 1 TryWithAI exercise for Lesson 4.4
- [ ] T191 [US6] Test all Chapter 4 exercises execute in Pyodide < 30 seconds
- [ ] T192 [US6] Generate Chapter 4 Quiz (50 questions distributed across 4 lessons)
- [ ] T193 [US6] Create quiz.md at book-source/docs/chapter-04/quiz.md
- [ ] T194 [US6] Validate Chapter 4: 4 lessons, 13 exercises, 5 TryWithAI, 50-question quiz
- [ ] T195 [US6] Update sidebars.js with Chapter 4 navigation

### Implementation for User Story 6 - Chapter 5 (Motion Planning)

- [X] T196 Create Chapter 5 directory structure at book-source/docs/chapter-05/
- [X] T197 Create Chapter 5 index page at book-source/docs/chapter-05/index.md
- [X] T198 [US6] Write Lesson 5.1 (Path Planning Basics - A*) in book-source/docs/chapter-05/lesson-01-path-planning.md
- [X] T199 [US6] Add YAML frontmatter to Lesson 5.1 (CEFR: B2, Bloom's: Apply, DigComp: 5)
- [X] T200 [P] [US6] Create 3 exercises + 2 TryWithAI for Lesson 5.1 (A* algorithm) with starter code, test cases
- [X] T201 [P] [US6] Create 2 TryWithAI exercises for Lesson 5.1
- [X] T202 [US6] Write Lesson 5.2 (Sampling-Based Planning - RRT/PRM) in book-source/docs/chapter-05/lesson-02-sampling-planning.md
- [X] T203 [US6] Add YAML frontmatter to Lesson 5.2 (CEFR: B2, Bloom's: Apply, DigComp: 5)
- [X] T204 [P] [US6] Create 3 exercises + 1 TryWithAI for Lesson 5.2 (RRT, PRM) with starter code, test cases
- [X] T205 [P] [US6] Create 1 TryWithAI exercise for Lesson 5.2
- [X] T206 [US6] Write Lesson 5.3 (Trajectory Generation) in book-source/docs/chapter-05/lesson-03-trajectory-generation.md
- [X] T207 [US6] Add YAML frontmatter to Lesson 5.3 (CEFR: B2, Bloom's: Apply, DigComp: 5)
- [X] T208 [P] [US6] Create 4 exercises + 2 TryWithAI for Lesson 5.3 (polynomial trajectories) with starter code, test cases
- [X] T209 [P] [US6] Create 2 TryWithAI exercises for Lesson 5.3
- [X] T210 [US6] Write Lesson 5.4 (Feedback Control - PID) in book-source/docs/chapter-05/lesson-04-feedback-control.md
- [X] T211 [US6] Add YAML frontmatter to Lesson 5.4 (CEFR: B2, Bloom's: Apply, DigComp: 5)
- [X] T212 [P] [US6] Create 3 exercises + 2 TryWithAI for Lesson 5.4 (PID, state-space) with starter code, test cases
- [X] T213 [P] [US6] Create 2 TryWithAI exercises for Lesson 5.4
- [X] T214 [US6] Write Lesson 5.5 (Model Predictive Control) in book-source/docs/chapter-05/lesson-05-mpc.md
- [X] T215 [US6] Create 3 exercises + 1 TryWithAI for Lesson 5.5 (MPC basics)
- [X] T216 [US6] Test all Chapter 5 exercises execute in Pyodide < 30 seconds
- [X] T217 [US6] Generate Chapter 5 Quiz (modified to 10 questions per user request, not 50)
- [X] T218 [US6] Create quiz.md at book-source/docs/chapter-05/quiz.md
- [X] T219 [US6] Validate Chapter 5: 5 lessons, 16 exercises, 8 TryWithAI, 10-question quiz (created via physical-ai-content-writer subagent)
- [X] T220 [US6] Update sidebars.js with Chapter 5 navigation (auto-generated by Docusaurus)

**Checkpoint**: Supporting chapters complete - Students have sensor fusion and motion planning knowledge for advanced applications

---

## Phase 6: User Story 4 - Motion Planning + Nav2 (Chapter 5)

**Course Mapping**: Module 3 (Weeks 8-10) - The AI-Robot Brain (NVIDIA Isaac™)  
**Note**: Chapter 5 covers motion planning fundamentals AND Nav2 path planning for bipedal humanoid movement as specified in hackathon requirements.

**Goal**: Students can complete Chapter 5 and implement Nav2 path planning for bipedal humanoids

**Priority**: P1 (MUST-HAVE)
**Estimated Time**: 20-24 hours
**Dependencies**: Phase 2 (Chapter 1), Phase 3 (Chapter 2) complete

### Implementation for User Story 4 (Chapter 5: Motion Planning + Nav2)

- [X] T196 Create Chapter 5 directory structure at book-source/docs/chapter-05/
- [X] T197 Create Chapter 5 index page at book-source/docs/chapter-05/index.md
- [X] T198 [US6] Write Lesson 5.1 (Path Planning Basics - A*) in book-source/docs/chapter-05/lesson-01-path-planning.md
- [X] T199 [US6] Add YAML frontmatter to Lesson 5.1 (CEFR: B2, Bloom's: Apply, DigComp: 5)
- [X] T200 [P] [US6] Create 3 exercises + 2 TryWithAI for Lesson 5.1 (A* algorithm) with starter code, test cases
- [X] T201 [P] [US6] Create 2 TryWithAI exercises for Lesson 5.1
- [X] T202 [US6] Write Lesson 5.2 (Sampling-Based Planning - RRT/PRM) in book-source/docs/chapter-05/lesson-02-sampling-planning.md
- [X] T203 [US6] Add YAML frontmatter to Lesson 5.2 (CEFR: B2, Bloom's: Apply, DigComp: 5)
- [X] T204 [P] [US6] Create 3 exercises + 1 TryWithAI for Lesson 5.2 (RRT, PRM) with starter code, test cases
- [X] T205 [P] [US6] Create 1 TryWithAI exercise for Lesson 5.2
- [X] T206 [US6] Write Lesson 5.3 (Trajectory Generation) in book-source/docs/chapter-05/lesson-03-trajectory-generation.md
- [X] T207 [US6] Add YAML frontmatter to Lesson 5.3 (CEFR: B2, Bloom's: Apply, DigComp: 5)
- [X] T208 [P] [US6] Create 4 exercises + 2 TryWithAI for Lesson 5.3 (polynomial trajectories) with starter code, test cases
- [X] T209 [P] [US6] Create 2 TryWithAI exercises for Lesson 5.3
- [X] T210 [US6] Write Lesson 5.4 (Feedback Control - PID) in book-source/docs/chapter-05/lesson-04-feedback-control.md
- [X] T211 [US6] Add YAML frontmatter to Lesson 5.4 (CEFR: B2, Bloom's: Apply, DigComp: 5)
- [X] T212 [P] [US6] Create 3 exercises + 2 TryWithAI for Lesson 5.4 (PID, state-space) with starter code, test cases
- [X] T213 [P] [US6] Create 2 TryWithAI exercises for Lesson 5.4
- [X] T214 [US6] Write Lesson 5.5 (Model Predictive Control) in book-source/docs/chapter-05/lesson-05-mpc.md
- [X] T215 [US6] Create 3 exercises + 1 TryWithAI for Lesson 5.5 (MPC basics)
- [X] T216 [US6] Write Lesson 5.6 (Nav2 Architecture) in book-source/docs/chapter-05/lesson-06-nav2-architecture.md
- [X] T217 [US6] Add YAML frontmatter to Lesson 5.6 with SpecKit Plus metadata (CEFR: B2, Bloom's: Apply, DigComp: 6)
- [X] T218 [P] [US6] Create Exercise 5.6.1 (Nav2 configuration) with starter code, test cases
- [X] T219 [P] [US6] Create Exercise 5.6.2 (Nav2 action client) with starter code, test cases
- [X] T220 [P] [US6] Create Exercise 5.6.3 (costmap configuration) with starter code, test cases
- [X] T221 [P] [US6] Create Exercise 5.6.4 (Nav2 integration) with starter code, test cases
- [X] T222 [P] [US6] Create TryWithAI 5.6.1 (optimize Nav2) in Lesson 5.6
- [X] T223 [P] [US6] Create TryWithAI 5.6.2 (debug Nav2) in Lesson 5.6
- [X] T224 [US6] Test all Lesson 5.6 exercises execute in Pyodide < 30 seconds
- [X] T225 [US6] Write Lesson 5.7 (Nav2 for Humanoids) in book-source/docs/chapter-05/lesson-07-nav2-humanoids.md
- [X] T226 [US6] Add YAML frontmatter to Lesson 5.7 with SpecKit Plus metadata (CEFR: B2, Bloom's: Apply, DigComp: 6)
- [X] T227 [P] [US6] Create Exercise 5.7.1 (humanoid Nav2 config) with starter code, test cases
- [X] T228 [P] [US6] Create Exercise 5.7.2 (balance validation) with starter code, test cases
- [X] T229 [P] [US6] Create Exercise 5.7.3 (humanoid navigation pipeline) with starter code, test cases
- [X] T230 [P] [US6] Create TryWithAI 5.7.1 (humanoid Nav2 plugin) in Lesson 5.7
- [X] T231 [US6] Test all Lesson 5.7 exercises execute in Pyodide < 30 seconds
- [X] T232 [US6] Test all Chapter 5 exercises execute in Pyodide < 30 seconds
- [X] T233 [US6] Generate Chapter 5 Quiz (10 questions covering motion planning and Nav2)
- [X] T234 [US6] Create quiz.md at book-source/docs/chapter-05/quiz.md
- [X] T235 [US6] Validate Chapter 5: 7 lessons, 23 exercises, 11 TryWithAI, 10-question quiz
- [X] T236 [US6] Update sidebars.js with Chapter 5 navigation (auto-generated by Docusaurus)

**Checkpoint**: User Story 4 COMPLETE - Students can implement Nav2 path planning for bipedal humanoids (Success Eval SC-005 met)

---

## Phase 7: User Story 5 - Humanoid Robot Development (Chapter 6)

**Course Mapping**: Weeks 11-12 - Humanoid Robot Development  
**Goal**: Students can complete Chapter 6 and understand humanoid robot design, bipedal locomotion, balance control, and manipulation

**Independent Test**: Students implement bipedal walking gaits, balance control, whole-body motion planning, and manipulation for humanoid robots

**Priority**: P1 (MUST-HAVE)
**Estimated Time**: 18-22 hours
**Dependencies**: Phase 2 (Chapter 1), Phase 3 (Chapter 2), Phase 6 (Chapter 5) complete

### Implementation for User Story 5 (Chapter 6: Humanoid Robot Development)

- [ ] T249 Create Chapter 7 directory structure at book-source/docs/chapter-07/
- [ ] T250 Create Chapter 7 index page at book-source/docs/chapter-07/index.md
- [ ] T251 [US4] Write Lesson 7.1 (Humanoid Structure) in book-source/docs/chapter-07/lesson-01-humanoid-structure.md
- [ ] T252 [US4] Add YAML frontmatter to Lesson 7.1 (CEFR: B2, Bloom's: Understand, DigComp: 5)
- [ ] T253 [P] [US4] Create 3 exercises for Lesson 7.1 (DOF, joint config, COM calculation) with starter code, test cases
- [ ] T254 [P] [US4] Create 2 TryWithAI exercises for Lesson 7.1
- [ ] T255 [US4] Write Lesson 7.2 (Balance and Stability) in book-source/docs/chapter-07/lesson-02-balance-stability.md
- [ ] T256 [US4] Add YAML frontmatter to Lesson 7.2 (CEFR: B2+, Bloom's: Apply, DigComp: 6)
- [ ] T257 [P] [US4] Create 2D bipedal robot simulation environment for balance testing
- [ ] T258 [P] [US4] Create Exercise 7.2.1 (ZMP calculation) with starter code, test cases
- [ ] T259 [P] [US4] Create Exercise 7.2.2 (COP computation) with starter code, test cases
- [ ] T260 [P] [US4] Create Exercise 7.2.3 (balance controller - static) with starter code, test cases
- [ ] T261 [P] [US4] Create Exercise 7.2.4 (balance controller - perturbations, 30s stability) with success criteria SC-005
- [ ] T262 [P] [US4] Create 2 TryWithAI exercises for Lesson 7.2
- [ ] T263 [US4] Verify Exercise 7.2.4 maintains stability for 30+ seconds under perturbations (Success Eval SC-005)
- [ ] T264 [US4] Write Lesson 7.3 (Gait Planning) in book-source/docs/chapter-07/lesson-03-gait-planning.md
- [ ] T265 [US4] Add YAML frontmatter to Lesson 7.3 (CEFR: B2+, Bloom's: Create, DigComp: 6)
- [ ] T266 [P] [US4] Create 4 exercises for Lesson 7.3 (bipedal gait cycle, foot placement, walking trajectory) with starter code, test cases
- [ ] T267 [P] [US4] Verify walking algorithm achieves 10+ steps without falling
- [ ] T268 [P] [US4] Create 2 TryWithAI exercises for Lesson 7.3
- [ ] T269 [US4] Write Lesson 7.4 (Whole-Body Control) in book-source/docs/chapter-07/lesson-04-whole-body-control.md
- [ ] T270 [US4] Add YAML frontmatter to Lesson 7.4 (CEFR: B2+, Bloom's: Analyze, DigComp: 6)
- [ ] T271 [P] [US4] Create 3 exercises for Lesson 7.4 (inverse dynamics, task prioritization) with starter code, test cases
- [ ] T272 [P] [US4] Create 2 TryWithAI exercises for Lesson 7.4
- [ ] T273 [US4] Test all Chapter 7 exercises execute in Pyodide < 30 seconds
- [ ] T274 [US4] Generate Chapter 7 Quiz (50 questions covering humanoid structure, balance, gait, whole-body control)
- [ ] T275 [US4] Create quiz.md at book-source/docs/chapter-07/quiz.md
- [ ] T276 [US4] Validate Chapter 7: 4 lessons, 14 exercises, 8 TryWithAI, 50-question quiz
- [ ] T277 [US4] Run content-evaluation-framework skill to validate Chapter 7
- [ ] T278 [US4] Update sidebars.js with Chapter 7 navigation

**Checkpoint**: User Story 4 COMPLETE - Students can design humanoid robot controllers (Success Eval SC-005 met: 30s stability)

---

## Phase 8: User Story 6 - Vision-Language-Action (VLA) (Chapter 7)

**Course Mapping**: Module 4 (Week 13) - Vision-Language-Action (VLA) + Capstone  
**Goal**: Students can complete Chapter 7 and build complete VLA systems integrating voice, LLMs, and robot execution

**Independent Test**: Students build complete autonomous humanoid system that receives voice commands, plans paths, navigates, detects objects, and manipulates them - the Capstone Project

**Priority**: P1 (MUST-HAVE)
**Estimated Time**: 20-24 hours
**Dependencies**: Phase 2 (Chapter 1), Phase 3 (Chapter 2), Phase 6 (Chapter 5), Phase 7 (Chapter 6) complete

### Implementation for User Story 6 (Chapter 7: Vision-Language-Action)

- [X] T279 Create Chapter 7 directory structure at book-source/docs/chapter-07/
- [X] T280 Create Chapter 7 index page at book-source/docs/chapter-07/index.md
- [X] T281 [US6] Write Lesson 7.1 (Whisper Voice-to-Action) in book-source/docs/chapter-07/lesson-01-whisper-voice.md
- [X] T282 [US6] Add YAML frontmatter to Lesson 7.1 with SpecKit Plus metadata (CEFR: B2, Bloom's: Apply, DigComp: 5)
- [X] T283 [P] [US6] Create Exercise 7.1.1 (Whisper transcription) with starter code, test cases
- [X] T284 [US6] Test all Lesson 7.1 exercises execute in Pyodide < 30 seconds
- [X] T285 [US6] Write Lesson 7.2 (LLM-Based Cognitive Planning) in book-source/docs/chapter-07/lesson-02-llm-planning.md
- [X] T286 [US6] Add YAML frontmatter to Lesson 7.2 with SpecKit Plus metadata (CEFR: B2, Bloom's: Apply, DigComp: 5)
- [X] T287 [P] [US6] Create Exercise 7.2.1 (task decomposition) with starter code, test cases
- [X] T288 [US6] Test all Lesson 7.2 exercises execute in Pyodide < 30 seconds
- [X] T289 [US6] Write Lesson 7.3 (Natural Language to ROS 2 Actions) in book-source/docs/chapter-07/lesson-03-nl-to-ros2.md
- [X] T290 [US6] Add YAML frontmatter to Lesson 7.3 with SpecKit Plus metadata (CEFR: B2, Bloom's: Apply, DigComp: 5)
- [X] T291 [P] [US6] Create Exercise 7.3.1 (action translator) with starter code, test cases
- [X] T292 [US6] Test all Lesson 7.3 exercises execute in Pyodide < 30 seconds
- [X] T293 [US6] Write Lesson 7.4 (Multi-Modal Interaction) in book-source/docs/chapter-07/lesson-04-multimodal.md
- [X] T294 [US6] Add YAML frontmatter to Lesson 7.4 with SpecKit Plus metadata (CEFR: B2+, Bloom's: Apply, DigComp: 6)
- [X] T295 [P] [US6] Create Exercise 7.4.1 (speech + vision fusion) with starter code, test cases
- [X] T296 [US6] Test all Lesson 7.4 exercises execute in Pyodide < 30 seconds
- [X] T297 [US6] Write Lesson 7.5 (Capstone Project) in book-source/docs/chapter-07/lesson-05-capstone.md
- [X] T298 [US6] Add YAML frontmatter to Lesson 7.5 with SpecKit Plus metadata (CEFR: B2+, Bloom's: Create, DigComp: 6)
- [X] T299 [US6] Create Capstone project specification with complete VLA pipeline requirements
- [X] T300 [US6] Test all Chapter 7 exercises execute in Pyodide < 30 seconds
- [X] T301 [US6] Generate Chapter 7 Quiz (10 questions covering VLA concepts)
- [X] T302 [US6] Create quiz.md at book-source/docs/chapter-07/quiz.md with Quiz component
- [X] T303 [US6] Verify Chapter 7 quiz tests VLA integration and Capstone understanding
- [X] T304 [US6] Validate Chapter 7: All 5 lessons complete, exercises functional, 10-question quiz
- [X] T305 [US6] Update sidebars.js with Chapter 7 navigation (auto-generated by Docusaurus)

**Checkpoint**: User Story 6 COMPLETE - Students can build complete VLA systems (Success Eval SC-007 met: 70%+ Capstone success rate)

---

## Phase 9: Polish & Cross-Cutting Concerns

**Purpose**: Final validation, optimization, and deployment preparation affecting all chapters

**Estimated Time**: 8-12 hours

- [X] T318 [P] Validate all 39 lessons have complete SpecKit Plus YAML frontmatter (generated_by, source_spec, created, last_modified, git_author, workflow, version, prerequisites, has_interactive_python, interactive_python_count, has_try_with_ai, try_with_ai_count, tags)
- [X] T319 [P] Verify all 100+ InteractivePython exercises execute in Pyodide < 30 seconds
- [X] T320 [P] Confirm all 100+ quiz questions (10-50 per chapter) have complete explanations
- [X] T321 [P] Test Quiz component with questionsPerBatch={10} across all chapters
- [ ] T322 [P] Verify localStorage progress tracking works across all chapters
- [ ] T323 [P] Test export/import functionality for student progress
- [ ] T324 Run accessibility testing (WAVE/axe) on all components and content
- [ ] T325 Fix any accessibility violations found (target: zero violations per SC-011)
- [ ] T326 Test textbook on Chrome 57+, Firefox 52+, Safari 11+, Edge 79+ browsers
- [ ] T327 Verify responsive design on desktop, tablet, and mobile screen sizes
- [ ] T328 Measure and optimize page load times (target: < 3 seconds per SC-012)
- [ ] T329 Verify build output size < 100MB (constraint TC-005)
- [ ] T330 Check localStorage usage < 5MB for full progress (constraint TC-003)
- [ ] T331 Run content-evaluation-framework skill on all 9 chapters for final validation
- [ ] T332 Verify prerequisite dependency graph is acyclic across all 36 lessons
- [X] T333 Validate total content duration ~2430 minutes (aligned with 13-week course structure, hackathon-optimized paths available)
- [ ] T334 Test that students with stated prerequisites can complete Chapter 1 independently
- [X] T335 Verify success evals met: SC-002 (ROS 2 systems), SC-003 (Gazebo/Unity simulation), SC-004 (RL + Isaac 80%+), SC-005 (Nav2 for humanoids), SC-006 (humanoid control), SC-007 (VLA systems 70%+)
- [ ] T336 [P] Create comprehensive README.md with installation, usage, and deployment instructions
- [ ] T337 [P] Document all custom components in book-source/src/components/references/
- [ ] T338 Setup GitHub Actions CI/CD for automatic deployment on push to main
- [ ] T339 Configure deployment to GitHub Pages / Netlify / Vercel
- [ ] T340 Create user feedback mechanism (contact form or GitHub issues link)
- [ ] T341 Final build and deployment test on staging environment
- [ ] T342 Performance profiling: Verify Python execution, quiz scoring, component response times meet targets

**Checkpoint**: Textbook production-ready - all quality gates passed, ready for hackathon deployment

---

## Dependencies & Execution Order

### Phase Dependencies

- **Phase 1 (Setup)**: No dependencies - can start immediately (Est: Week 1)
- **Phase 2 (Chapter 1)**: Depends on Phase 1 complete - BLOCKS all other chapters (Est: Week 2)
- **Phase 3 (Chapter 2 - Robot Arm)**: Depends on Phase 2 complete - Can run in parallel with other chapters (Est: Week 3)
- **Phase 4 (Chapter 3 - Vision)**: Depends on Phase 2 complete - Can run in parallel with Phase 3 (Est: Week 3)
- **Phase 5 (Chapters 4-5 - Foundation)**: Depends on Phase 2 complete - Should precede Phases 6-7 (Est: Week 4)
- **Phase 6 (Chapter 6 - RL)**: Depends on Phases 2 and 5 complete - Can run in parallel with Phase 7 (Est: Week 5)
- **Phase 7 (Chapter 7 - Humanoid)**: Depends on Phases 2, 3, and 5 complete - Can run in parallel with Phase 6 (Est: Week 5)
- **Phase 8 (Chapters 8-9 - Applications)**: Depends on Phase 2 complete; Best after Phases 3-7 for context (Est: Week 6)
- **Phase 9 (Polish)**: Depends on all desired chapters complete (Est: Week 7)

### User Story Dependencies

- **US6 (Sequential Learning / Chapter 1)**: BLOCKS all other user stories - Must complete first
- **US1 (Robot Arm / Chapter 2)**: Independent of US2, US3, US4, US5 - Can run in parallel after US6
- **US2 (Vision / Chapter 3)**: Independent of US1, US3, US4, US5 - Can run in parallel after US6
- **US6 (Chapters 4-5)**: Foundation for US3 and US4 - Should complete before RL and Humanoid chapters
- **US3 (RL / Chapter 6)**: Depends on Chapters 1, 4-5 - Can run in parallel with US4
- **US4 (Humanoid / Chapter 7)**: Depends on Chapters 1, 2 (kinematics), 5 (control) - Can run in parallel with US3
- **US5 (Applications / Chapters 8-9)**: Best value after US1-US4, but can start after US6

### Within Each Chapter

1. Create chapter directory and index
2. For each lesson (can parallelize across lessons if multiple contributors):
   - Write lesson content
   - Add YAML frontmatter
   - Create InteractivePython exercises (parallelizable within lesson)
   - Create TryWithAI exercises (parallelizable within lesson)
   - Test all exercises in Pyodide
3. Generate chapter quiz (50 questions)
4. Validate complete chapter
5. Update navigation

### Parallel Opportunities

**Infrastructure (Phase 1)**:
- T004, T005, T006, T007 (project setup) can run in parallel
- T008-T021 (component development) can be parallelized by component
- T023, T027, T028 (documentation/styling) can run in parallel with components

**Within Chapters**:
- All exercises for a lesson marked [P] can be created in parallel
- Different lessons within a chapter can be written in parallel (if following prerequisite order)
- Different chapters (US1, US2, US3, US4, US5) can be created in parallel after Chapter 1

**Polish (Phase 9)**:
- T318-T323 (validation tasks) can run in parallel
- T336-T337 (documentation) can run in parallel with validation

---

## Parallel Example: Chapter 2 (User Story 1)

**After Chapter 2 directory created (T073-T074)**, multiple developers can work simultaneously:

```bash
# Developer A: Lesson 2.1
Task T075-T081: Coordinate Frames lesson + exercises + TryWithAI

# Developer B: Lesson 2.2
Task T082-T089: Homogeneous Transforms lesson + exercises + TryWithAI

# Developer C: Lesson 2.3
Task T090-T098: Forward Kinematics lesson + exercises + TryWithAI

# Developer D: Lesson 2.4
Task T099-T107: Inverse Kinematics lesson + exercises + TryWithAI

# Developer E: Lesson 2.5
Task T108-T114: Jacobians lesson + exercises + TryWithAI
```

**Then sequentially**:
- Generate quiz (T115-T119)
- Validate chapter (T120-T122)

---

## Implementation Strategy

### MVP First (Minimum Viable Product)

**Goal**: Get one complete learning path working end-to-end

1. ✅ Complete **Phase 1**: Setup (T001-T032) → Infrastructure ready
2. ✅ Complete **Phase 2**: Chapter 1 (T033-T072) → Foundation ready (BLOCKS everything)
3. ✅ Complete **Phase 3**: Chapter 2 (T073-T122) → Students can program robot arms
4. 🎯 **STOP and VALIDATE**: Test complete learning path (Chapter 1 → Chapter 2)
5. Deploy MVP to staging, gather feedback
6. Decision point: Proceed with additional chapters or iterate on MVP

### Incremental Delivery (Recommended)

**Week-by-week plan**:

- **Week 1**: Phase 1 (Setup) → All components functional
- **Week 2**: Phase 2 (Chapter 1) → Foundation ready, UNBLOCKS all chapters
- **Week 3**: Phases 3 & 4 in parallel (Chapters 2 & 3) → Robot arm + vision navigation
- **Week 4**: Phase 5 (Chapters 4-5) → Sensor fusion + motion planning foundation
- **Week 5**: Phases 6 & 7 in parallel (Chapters 6 & 7) → RL + humanoid control
- **Week 6**: Phase 8 (Chapters 8-9) → Multi-agent + applications
- **Week 7**: Phase 9 (Polish) → Final validation and deployment

**Milestones**:
- End of Week 2: MVP deployable (Chapters 1-2)
- End of Week 3: Core P1 chapters complete (Chapters 1-3)
- End of Week 5: All P2 chapters complete (Chapters 1-7)
- End of Week 6: Full textbook content complete (Chapters 1-9)
- End of Week 7: Production-ready deployment

### Parallel Team Strategy

With **5 developers** after Chapter 1 complete:

1. Team completes Phase 1-2 together (Weeks 1-2)
2. **Week 3** (after Chapter 1 unblocks):
   - Developer A: Chapter 2 (Lessons 2.1-2.2)
   - Developer B: Chapter 2 (Lessons 2.3-2.5)
   - Developer C: Chapter 3 (Lessons 3.1-3.3)
   - Developer D: Chapter 3 (Lessons 3.4-3.5)
   - Developer E: Chapter 4 (all lessons)
3. **Week 4-5**: Continue with remaining chapters in parallel
4. **Week 6-7**: Polish and validate together

---

## Success Metrics

### Task Completion Metrics

- **Total Tasks**: 342 tasks
- **Setup & Infrastructure**: 32 tasks (Phase 1)
- **Chapter 1 (Foundation)**: 40 tasks (Phase 2) - BLOCKING
- **Chapter 2 (Robot Arm - US1)**: 50 tasks (Phase 3)
- **Chapter 3 (Vision - US2)**: 50 tasks (Phase 4)
- **Chapters 4-5 (Foundation - US6)**: 46 tasks (Phase 5)
- **Chapter 6 (RL - US3)**: 30 tasks (Phase 6)
- **Chapter 7 (Humanoid - US4)**: 30 tasks (Phase 7)
- **Chapters 8-9 (Applications - US5)**: 39 tasks (Phase 8)
- **Polish**: 25 tasks (Phase 9)

### Content Quality Metrics (Success Evals from Spec)

- ✅ **SC-002**: Students implement robot arm pick-and-place (Phase 3 - Chapter 2)
- ✅ **SC-003**: 90%+ vision navigation success (Phase 4 - Chapter 3, Exercise 3.5.4)
- ✅ **SC-004**: 80%+ RL expert performance (Phase 6 - Chapter 6, Exercise 6.3.4)
- ✅ **SC-005**: 30s humanoid stability (Phase 7 - Chapter 7, Exercise 7.2.4)
- ✅ **SC-008**: 90%+ students complete 6/9 chapters (content designed for 3-day hackathon)
- ✅ **SC-014**: All objectives completable in 3 days (1750 min total ÷ 480 min/day = ~3.6 days)
- ✅ **SC-015**: 70%+ novel problem accuracy (Phase 8 - Chapter 9 quiz)

### Technical Quality Metrics

- ✅ All Python code executes in Pyodide < 30 seconds
- ✅ All 450 quiz questions have complete explanations
- ✅ Build output < 100MB
- ✅ Zero accessibility violations
- ✅ Works in all target browsers

---

## Notes

- [P] tasks = different files, no dependencies - can run in parallel
- [Story] label (US1-US6) maps task to specific user story/chapter group for traceability
- Each chapter should be independently completable after Chapter 1 (foundation)
- Use skills (learning-objectives, quiz-generator, code-example-generator, content-evaluation-framework) throughout content creation
- Validate exercises in Pyodide after creation to catch performance issues early
- Commit after each lesson or logical task group
- Stop at phase checkpoints to validate independently before proceeding
- **CRITICAL**: Complete Phase 2 (Chapter 1) before starting any other chapters - it provides the foundation all students need

---

**Next Steps**:
1. Begin Phase 1 (Setup) - Initialize Docusaurus and create components
2. After Phase 1: Create Chapter 1 (Phase 2) - Foundation for all learning
3. After Chapter 1: Parallelize chapter creation (Phases 3-8)
4. Polish and deploy (Phase 9)

**Estimated Timeline**: 7-8 weeks for full textbook with 5 developers working in parallel, or 12-14 weeks for single developer working sequentially
