# Implementation Plan: Physical AI & Humanoid Robotics Textbook

**Branch**: `002-physical-ai-textbook` | **Date**: 2025-11-29 | **Spec**: [spec.md](./spec.md)  
**Created Using**: SpecKit Plus `/sp.plan` command via Claude Code  
**Input**: Feature specification from `/specs/002-physical-ai-textbook/spec.md`

## Summary

Create a production-ready, browser-based interactive textbook teaching Physical AI and Humanoid Robotics following a **13-week course structure**. The textbook covers:

- **Weeks 1-2**: Introduction to Physical AI
- **Module 1 (Weeks 3-5)**: ROS 2 - The Robotic Nervous System
- **Module 2 (Weeks 6-7)**: Gazebo & Unity - The Digital Twin
- **Module 3 (Weeks 8-10)**: NVIDIA Isaac - The AI-Robot Brain
- **Weeks 11-12**: Humanoid Robot Development
- **Module 4 (Week 13)**: Vision-Language-Action (VLA) + Capstone

Each module contains 3-5 lessons with interactive Python exercises executed via Pyodide, quizzes, and AI co-learning exercises. Content targets intermediate learners (B1-B2 CEFR) and enables students to master ROS 2, simulate robots, develop with NVIDIA Isaac, and build conversational robotics systems.

**Technical Approach**: Docusaurus static site with custom React components (InteractivePython, Quiz, TryWithAI), Pyodide for browser-based Python execution, KaTeX for mathematical notation, and localStorage for progress tracking. No server-side execution required (fully static deployment).

## Technical Context

**Language/Version**: JavaScript/TypeScript (Node 18+), Python 3.11+ (Pyodide in browser)
**Primary Dependencies**: Docusaurus v3.x, Pyodide 0.24+, React 18, CodeMirror 6, KaTeX 0.16+
**Storage**: Browser localStorage (client-side progress tracking), static Markdown files (content)
**Testing**: Jest (component tests), Playwright (E2E), Pyodide runtime (Python code validation)
**Target Platform**: Modern web browsers (Chrome 57+, Firefox 52+, Safari 11+, Edge 79+) with WebAssembly support
**Project Type**: Static web application (Docusaurus site)
**Performance Goals**:
- Python code execution < 30 seconds per exercise
- Quiz scoring < 2 seconds
- Page load < 3 seconds
- Interactive component response < 100ms

**Constraints**:
- All Python execution in-browser (no server-side)
- Build output < 100MB
- localStorage < 5MB for progress
- Pyodide startup 2-5 seconds (provide loading indicator)
- Mathematical content limited to high school level
- Browser-based visualizations only (no native physics engines)

**Scale/Scope**:
- 7 chapters with 4-8 lessons each (39 lessons total)
- 100+ quiz questions (10-50 per chapter based on complexity)
- 100+ InteractivePython exercises (3+ per lesson minimum)
- 40+ TryWithAI exercises (1-2 per lesson)
- Target: 90%+ students complete 5/7 chapters in 3 days

## Constitution Check

*Note: The constitution file is currently a template. Applying general software development best practices.*

**Quality Gates**:
- ✅ All code testable (Python code has automated test cases)
- ✅ Content validated (using content-evaluation-framework skill)
- ✅ Accessibility compliant (WCAG 2.1 AA for all components)
- ✅ Performance measured (page load, execution times tracked)
- ✅ Progressive enhancement (static content if JavaScript disabled)

**Pedagogical Gates** (Constitution v4.0.1):
- ✅ Specification primacy ("Specs Are the New Syntax" - Chapter specs define learning outcomes first)
- ✅ 4-Layer Teaching Method (Foundation → Examples → Practice → Application)
- ✅ AI Three Roles Framework (Teacher, Copilot, Evaluator in TryWithAI exercises)
- ✅ Progressive complexity (B1 → B1+ → B2 → B2+ across chapters)
- ✅ Evals-first design (Success criteria defined before content creation)

## Project Structure

### Documentation (this feature)

```text
specs/002-physical-ai-textbook/
├── spec.md               # Requirements specification
├── plan.md               # This file - implementation plan
├── research.md           # Technology decisions and research findings
├── data-model.md         # Entity definitions and data structures
├── quickstart.md         # Contributor onboarding guide
├── contracts/            # API and component interfaces
│   └── component-props.json
├── checklists/          # Validation checklists
│   └── requirements.md
└── tasks.md             # FUTURE: Detailed implementation tasks (/sp.tasks)
```

### Source Code (repository root)

```text
physical-ai-book/
├── book-source/                # Docusaurus site
│   ├── docs/                  # Lesson content (Markdown + MDX)
│   │   ├── intro.md          # Textbook homepage
│   │   ├── chapter-01/       # Chapter 1: Introduction to Physical AI
│   │   │   ├── index.md     # Chapter overview
│   │   │   ├── lesson-01-what-is-physical-ai.md
│   │   │   ├── lesson-02-robot-vs-software-ai.md
│   │   │   ├── lesson-03-sensors-actuators-overview.md
│   │   │   ├── lesson-04-python-robotics-intro.md
│   │   │   └── quiz.md      # 50-question quiz
│   │   ├── chapter-02/       # Chapter 2: Robot Kinematics and Dynamics
│   │   │   ├── index.md
│   │   │   ├── lesson-01-coordinate-frames.md
│   │   │   ├── lesson-02-homogeneous-transforms.md
│   │   │   ├── lesson-03-forward-kinematics.md
│   │   │   ├── lesson-04-inverse-kinematics.md
│   │   │   ├── lesson-05-jacobians-velocities.md
│   │   │   └── quiz.md
│   │   ├── chapter-03/       # Chapter 3: Computer Vision for Robotics
│   │   ├── chapter-04/       # Chapter 4: Sensor Integration and Perception
│   │   ├── chapter-05/       # Chapter 5: Motion Planning and Control
│   │   ├── chapter-06/       # Chapter 6: Reinforcement Learning for Robotics
│   │   ├── chapter-07/       # Chapter 7: Humanoid Robot Design and Control
│   │   ├── chapter-08/       # Chapter 8: AI Agents for Robot Coordination
│   │   └── chapter-09/       # Chapter 9: Real-world Applications and Case Studies
│   ├── src/
│   │   ├── components/       # Custom MDX components
│   │   │   ├── InteractivePython.tsx
│   │   │   ├── Quiz.tsx (existing)
│   │   │   ├── TryWithAI.tsx
│   │   │   ├── LearningObjectives.tsx
│   │   │   ├── Prerequisites.tsx
│   │   │   └── references/  # Usage examples
│   │   │       ├── example-quiz.md
│   │   │       └── QUIZ_USAGE.md
│   │   ├── css/
│   │   │   └── custom.css
│   │   ├── pages/           # Custom pages (about, feedback)
│   │   └── theme/           # Theme customizations
│   ├── static/              # Static assets
│   │   ├── img/            # Diagrams, robot illustrations
│   │   └── data/           # Sample datasets for exercises
│   ├── plugins/            # Custom Docusaurus plugins
│   │   └── pyodide-loader/ # Preload Pyodide
│   ├── docusaurus.config.js
│   ├── sidebars.js
│   ├── package.json
│   └── tsconfig.json
├── specs/                  # Feature planning (this directory)
├── .claude/
│   └── skills/            # Content generation skills
│       ├── book-scaffolding/
│       ├── learning-objectives/
│       ├── quiz-generator/
│       ├── code-example-generator/
│       ├── content-evaluation-framework/
│       └── ... (9 total applicable skills)
└── README.md
```

**Structure Decision**: Static web application using Docusaurus. All content in Markdown/MDX files under `book-source/docs/`. Custom components in `book-source/src/components/`. No backend required - entirely client-side execution with Pyodide.

## Complexity Tracking

No constitution violations - this is a straightforward educational content project with well-defined structure.

---

## Chapter Breakdown & Implementation Plan

### Overview: 7 Chapters, 35 Lessons Total (Aligned with Hackathon Requirements)

| Chapter | Title | Module/Week | Lessons | CEFR Range | Est. Time | Success Evals |
|---------|-------|-------------|---------|------------|-----------|---------------|
| 1 | Introduction to Physical AI | Weeks 1-2 | 4 | B1 | 180 min | Foundation understanding |
| 2 | ROS 2 Fundamentals | Module 1, Weeks 3-5 | 5 | B1 to B2 | 320 min | Build ROS 2 systems |
| 3 | The Digital Twin (Gazebo & Unity) | Module 2, Weeks 6-7 | 5 | B1+ to B2 | 320 min | Simulate robots |
| 4 | Reinforcement Learning + NVIDIA Isaac | Module 3, Weeks 8-10 | 8 | B2 | 500 min | Train RL agents, Isaac Sim |
| 5 | Motion Planning + Nav2 | Module 3, Weeks 8-10 | 7 | B2 | 430 min | Nav2 for humanoids |
| 6 | Humanoid Robot Development | Weeks 11-12 | 5 | B2 | 300 min | Humanoid control systems |
| 7 | Vision-Language-Action (VLA) | Module 4, Week 13 | 5 | B2 to B2+ | 380 min | Complete VLA systems |

**Total**: 39 lessons, ~2430 minutes (~40.5 hours of content)

---

## Chapter 1: Introduction to Physical AI and Robotics

**Goal**: Establish foundational understanding of Physical AI, differentiate from software AI, introduce robot components and Python robotics programming.

**CEFR Level**: B1
**Estimated Time**: 180 minutes (3 hours)
**Prerequisites**: Basic Python, basic AI/ML concepts, high school math
**Success Criteria**: Students understand Physical AI concepts and can write basic Python code for robot control.

### Lessons (4)

#### Lesson 1.1: What is Physical AI?

**Title**: What is Physical AI and Why It Matters
**Duration**: 45 minutes
**CEFR**: B1 | **Bloom's**: Understand | **DigComp**: 2

**Learning Objectives**:
1. Define Physical AI and explain how it differs from traditional software AI
2. Identify real-world applications of Physical AI (manufacturing, healthcare, service robotics)
3. Explain the three key components: sensing, reasoning, actuation

**Interactive Python** (3 exercises):
1. **Exercise 1.1.1**: Simulate a simple sensor reading (generate random temperature data)
   - Starter code: `def read_sensor():`
   - Test: Verify output is in valid range

2. **Exercise 1.1.2**: Process sensor data with basic filtering (moving average)
   - Starter code: `def moving_average(data, window_size):`
   - Test: Verify smoothing reduces noise

3. **Exercise 1.1.3**: Make a simple decision based on sensor input (threshold-based control)
   - Starter code: `def control_decision(sensor_value, threshold):`
   - Test: Verify correct ON/OFF decision

**Try With AI** (2 exercises):
1. **Role**: Teacher
   - **Scenario**: Understanding Physical AI applications
   - **Task**: Research one Physical AI application (warehouse robots, surgical robots, delivery drones)
   - **AI Prompt**: "Explain how [chosen application] uses sensing, reasoning, and actuation. What are the key challenges?"
   - **Success**: Can describe the application's Physical AI components and challenges

2. **Role**: Evaluator
   - **Scenario**: Validating sensor processing code
   - **Task**: Write code to process sensor data
   - **AI Prompt**: "Review my sensor processing code: [paste code]. Does it handle edge cases like empty arrays, negative values, or sensor failures?"
   - **Success**: Code handles all edge cases identified by AI

**Quiz Distribution**: 12 questions (out of 50 for chapter)

---

#### Lesson 1.2: Robots vs. Software AI Systems

**Title**: Key Differences Between Robots and Pure Software AI
**Duration**: 40 minutes
**CEFR**: B1 | **Bloom's**: Analyze | **DigComp**: 2

**Learning Objectives**:
1. Compare and contrast robots with software-only AI systems
2. Explain unique challenges in Physical AI (real-time constraints, sensor noise, physical safety)
3. Identify when Physical AI is necessary vs. software AI sufficient

**Interactive Python** (3 exercises):
1. **Exercise 1.2.1**: Simulate real-time constraint (process data within time budget)
   - Starter code: Timed loop processing sensor data
   - Test: Verify processing completes within 100ms

2. **Exercise 1.2.2**: Handle sensor noise (add Gaussian noise, then filter)
   - Starter code: `def add_noise(signal, noise_level):`
   - Test: Verify noisy signal approximates clean signal after filtering

3. **Exercise 1.2.3**: Implement safety check (emergency stop on dangerous condition)
   - Starter code: `def safety_monitor(robot_state):`
   - Test: Verify emergency stop triggers correctly

**Try With AI** (1 exercise):
1. **Role**: Copilot
   - **Scenario**: Designing a safety system
   - **Task**: Sketch safety requirements for a robot
   - **AI Prompt**: "I'm designing a robot for [scenario]. What safety checks should I implement? Help me list edge cases."
   - **Success**: Comprehensive safety checklist with AI-identified edge cases

**Quiz Distribution**: 13 questions

---

#### Lesson 1.3: Sensors and Actuators Overview

**Title**: Introduction to Robot Sensing and Actuation
**Duration**: 50 minutes
**CEFR**: B1 | **Bloom's**: Understand | **DigComp**: 3

**Learning Objectives**:
1. Classify sensor types (proprioceptive vs. exteroceptive)
2. Explain common sensors (encoders, IMU, cameras, LIDAR)
3. Describe actuator types (DC motors, servos, stepper motors)
4. Convert sensor readings to useful information (encoder ticks → angle)

**Interactive Python** (3 exercises):
1. **Exercise 1.3.1**: Convert encoder ticks to joint angle
   - Starter code: `def ticks_to_angle(ticks, ticks_per_revolution):`
   - Test: Verify correct angle calculation

2. **Exercise 1.3.2**: Process IMU data (accelerometer to tilt angle)
   - Starter code: `def accelerometer_to_tilt(ax, ay, az):`
   - Test: Verify tilt angle using arctangent

3. **Exercise 1.3.3**: Simulate servo control (PWM duty cycle to angle)
   - Starter code: `def pwm_to_angle(duty_cycle, min_duty, max_duty):`
   - Test: Verify linear mapping

**Try With AI** (2 exercises):
1. **Role**: Teacher
   - **Scenario**: Choosing sensors for a project
   - **Task**: Design sensor suite for a mobile robot
   - **AI Prompt**: "I want to build a mobile robot that navigates indoors and avoids obstacles. What sensors do I need and why?"
   - **Success**: Can justify sensor choices based on robot requirements

2. **Role**: Copilot
   - **Scenario**: Debugging sensor calibration
   - **Task**: Calibrate an IMU sensor
   - **AI Prompt**: "My IMU accelerometer readings seem offset. Help me write a calibration function that computes bias from stationary readings."
   - **Success**: Calibration function correctly removes sensor bias

**Quiz Distribution**: 13 questions

---

#### Lesson 1.4: Getting Started with Python for Robotics

**Title**: Python Programming for Robot Control
**Duration**: 45 minutes
**CEFR**: B1 | **Bloom's**: Apply | **DigComp**: 3

**Learning Objectives**:
1. Use NumPy for numerical computations (arrays, matrix operations)
2. Visualize robot data with Matplotlib (plots, animations)
3. Structure robot control code (main loop, sensor reading, control decision, actuation)
4. Test robot code in simulation before deployment

**Interactive Python** (4 exercises):
1. **Exercise 1.4.1**: Create and manipulate NumPy arrays (joint angles)
   - Starter code: `import numpy as np` with TODO for array operations
   - Test: Verify correct array manipulation

2. **Exercise 1.4.2**: Plot sensor data over time with Matplotlib
   - Starter code: `import matplotlib.pyplot as plt` with sample data
   - Test: Verify plot displays correctly

3. **Exercise 1.4.3**: Implement a control loop (read sensor → decide → actuate)
   - Starter code: Control loop skeleton
   - Test: Verify loop runs for specified iterations

4. **Exercise 1.4.4**: Simulate a simple robot (position update from velocity)
   - Starter code: `def simulate_robot(pos, vel, dt):`
   - Test: Verify position integrates velocity correctly

**Try With AI** (2 exercises):
1. **Role**: Copilot
   - **Scenario**: Optimizing NumPy code
   - **Task**: Write vectorized NumPy code
   - **AI Prompt**: "I have this for-loop processing sensor data: [paste code]. Help me vectorize it with NumPy for better performance."
   - **Success**: Vectorized code produces same results, runs faster

2. **Role**: Evaluator
   - **Scenario**: Reviewing control loop structure
   - **Task**: Write a robot control loop
   - **AI Prompt**: "Review my control loop: [paste code]. Is the structure correct for real-time robot control? What could go wrong?"
   - **Success**: Loop handles timing, exceptions, and safety correctly

**Quiz Distribution**: 12 questions

---

### Chapter 1 Quiz

**File**: `chapter-01/quiz.md`
**Component**: `<Quiz chapterId={1} />`
**Total Questions**: 50
**Distribution**:
- Lesson 1.1: 12 questions (definitions, concepts, applications)
- Lesson 1.2: 13 questions (comparisons, challenges, when to use Physical AI)
- Lesson 1.3: 13 questions (sensors, actuators, conversions)
- Lesson 1.4: 12 questions (Python, NumPy, control loops)

**Bloom's Distribution**:
- Remember: 10 questions (20%) - definitions, terminology
- Understand: 15 questions (30%) - explain concepts, classify
- Apply: 15 questions (30%) - use Python, compute conversions
- Analyze: 10 questions (20%) - compare, debug, identify problems

**Question Types**:
- Multiple choice: 40 questions (4 options each)
- True/False: 10 questions

---

## Chapter 2: Robot Kinematics and Dynamics

**Goal**: Master robot arm modeling, compute forward/inverse kinematics, understand Jacobians. Enable students to program robot arm for pick-and-place tasks (Success Eval SC-002).

**CEFR Level**: B1 to B1+
**Estimated Time**: 240 minutes (4 hours)
**Prerequisites**: Chapter 1
**Success Criteria**: Students can implement 3-DOF robot arm pick-and-place algorithm that passes automated test cases.

### Lessons (5)

#### Lesson 2.1: Robot Coordinate Frames and Transformations

**Title**: Understanding Coordinate Frames in Robotics
**Duration**: 50 minutes
**CEFR**: B1 | **Bloom's**: Understand | **DigComp**: 3

**Learning Objectives**:
1. Define coordinate frames (base frame, end-effector frame, world frame)
2. Represent 2D rotations using rotation matrices
3. Apply transformations to convert points between frames
4. Visualize coordinate frames and transformations

**Interactive Python** (3 exercises):
1. **Exercise 2.1.1**: Create 2D rotation matrix from angle
   - Starter code: `def rotation_matrix_2d(theta):`
   - Test: Verify rotation by 90° produces correct matrix

2. **Exercise 2.1.2**: Apply rotation to a point
   - Starter code: `def rotate_point(point, theta):`
   - Test: Verify rotated point coordinates

3. **Exercise 2.1.3**: Visualize coordinate frames (plot origin and axes)
   - Starter code: Matplotlib setup to draw frames
   - Test: Visual verification of frame orientation

**Try With AI** (1 exercise):
1. **Role**: Teacher
   - **Scenario**: Understanding rotation matrices
   - **Task**: Derive 2D rotation matrix
   - **AI Prompt**: "Explain how a 2D rotation matrix works. Why does rotating by θ then φ equal rotating by θ+φ?"
   - **Success**: Can explain rotation composition and matrix multiplication

**Quiz Distribution**: 10 questions

---

#### Lesson 2.2: Homogeneous Transformations (3D)

**Title**: 3D Coordinate Transformations for Robot Arms
**Duration**: 50 minutes
**CEFR**: B1+ | **Bloom's**: Apply | **DigComp**: 4

**Learning Objectives**:
1. Represent 3D rotations (rotation matrices, Euler angles)
2. Use homogeneous coordinates for combined rotation and translation
3. Compose multiple transformations using matrix multiplication
4. Apply Denavit-Hartenberg (DH) parameters for robot modeling

**Interactive Python** (3 exercises):
1. **Exercise 2.2.1**: Create 3D rotation matrix around z-axis
   - Starter code: `def rot_z(theta):`
   - Test: Verify rotation matrix properties (orthogonal, det=1)

2. **Exercise 2.2.2**: Build homogeneous transformation matrix
   - Starter code: `def transform_matrix(rotation, translation):`
   - Test: Verify 4x4 matrix structure

3. **Exercise 2.2.3**: Compose transformations (chain multiple frames)
   - Starter code: `def chain_transforms(T_list):`
   - Test: Verify composed transformation equals manual calculation

**Try With AI** (2 exercises):
1. **Role**: Copilot
   - **Scenario**: Implementing DH parameters
   - **Task**: Build transformation from DH parameters
   - **AI Prompt**: "I need to implement the DH transformation matrix from parameters (a, alpha, d, theta). Help me write the function step-by-step."
   - **Success**: Correct DH transformation function

2. **Role**: Evaluator
   - **Scenario**: Debugging transformation chain
   - **Task**: Debug transformation composition
   - **AI Prompt**: "My transformation chain gives wrong results: [paste code]. Help me debug. Are the matrices multiplied in the right order?"
   - **Success**: Identifies and fixes matrix multiplication order

**Quiz Distribution**: 10 questions

---

#### Lesson 2.3: Forward Kinematics of Robot Arms

**Title**: Computing End-Effector Position from Joint Angles
**Duration**: 60 minutes
**CEFR**: B1+ | **Bloom's**: Apply | **DigComp**: 4

**Learning Objectives**:
1. Implement forward kinematics for 2-DOF planar arm
2. Extend to 3-DOF planar arm
3. Visualize robot configurations in 2D workspace
4. Verify FK implementation with test cases

**Interactive Python** (4 exercises):
1. **Exercise 2.3.1**: 2-DOF planar arm FK
   - Starter code: `def forward_kinematics_2dof(theta1, theta2, L1, L2):`
   - Test: Verify end-effector position for known configurations

2. **Exercise 2.3.2**: 3-DOF planar arm FK (extends 2-DOF)
   - Starter code: `def forward_kinematics_3dof(theta1, theta2, theta3, L1, L2, L3):`
   - Test: Multiple test cases including edge cases (fully extended, folded)

3. **Exercise 2.3.3**: Visualize robot arm configuration
   - Starter code: Matplotlib plot skeleton
   - Test: Visual verification of arm pose

4. **Exercise 2.3.4**: FK for spatial arm (3D, 3-DOF)
   - Starter code: Use homogeneous transforms
   - Test: 3D position computation

**Try With AI** (2 exercises):
1. **Role**: Copilot
   - **Scenario**: Extending FK to more DOF
   - **Task**: Generalize FK for N-DOF arm
   - **AI Prompt**: "I have FK for 3-DOF arm: [paste code]. Help me generalize this to work for any number of joints (N-DOF)."
   - **Success**: Generic FK function working for variable DOF

2. **Role**: Evaluator
   - **Scenario**: Testing FK implementation
   - **Task**: Write comprehensive test cases
   - **AI Prompt**: "I implemented FK for 3-DOF arm: [paste code]. Help me design test cases to verify it's correct. What edge cases should I check?"
   - **Success**: Test suite covers nominal cases and edge cases

**Quiz Distribution**: 10 questions

---

#### Lesson 2.4: Inverse Kinematics and Solutions

**Title**: Computing Joint Angles from Desired End-Effector Position
**Duration**: 60 minutes
**CEFR**: B2 | **Bloom's**: Analyze | **DigComp**: 5

**Learning Objectives**:
1. Explain the inverse kinematics problem
2. Derive analytical IK solution for 2-DOF planar arm (geometric approach)
3. Implement IK with elbow-up and elbow-down solutions
4. Handle IK failure cases (target out of reach)

**Interactive Python** (4 exercises):
1. **Exercise 2.4.1**: Geometric IK for 2-DOF planar arm
   - Starter code: `def inverse_kinematics_2dof(x, y, L1, L2):`
   - Test: Verify IK then FK recovers original position

2. **Exercise 2.4.2**: Handle multiple IK solutions (elbow-up/down)
   - Starter code: Return both solutions
   - Test: Both solutions reach target, differ in elbow configuration

3. **Exercise 2.4.3**: Check reachability (workspace limits)
   - Starter code: `def is_reachable(x, y, L1, L2):`
   - Test: Correctly identify in-reach vs. out-of-reach targets

4. **Exercise 2.4.4**: IK for 3-DOF planar arm (numerical/geometric hybrid)
   - Starter code: Solve for first 2 joints geometrically, third joint analytically
   - Test: Verify solution accuracy

**Try With AI** (2 exercises):
1. **Role**: Teacher
   - **Scenario**: Understanding IK geometric derivation
   - **Task**: Study IK derivation
   - **AI Prompt**: "Explain the geometric approach to IK for a 2-DOF planar arm. Walk me through the law of cosines derivation step-by-step."
   - **Success**: Can derive IK equations with geometric reasoning

2. **Role**: Copilot
   - **Scenario**: Implementing IK for 3-DOF
   - **Task**: Extend IK to 3-DOF
   - **AI Prompt**: "I have 2-DOF IK: [paste code]. Help me extend to 3-DOF. How do I handle the extra redundancy?"
   - **Success**: Working 3-DOF IK with redundancy resolution

**Quiz Distribution**: 10 questions

---

#### Lesson 2.5: Jacobians and Velocity Kinematics

**Title**: Relating Joint Velocities to End-Effector Velocities
**Duration**: 50 minutes
**CEFR**: B2 | **Bloom's**: Apply | **DigComp**: 5

**Learning Objectives**:
1. Define the robot Jacobian matrix
2. Compute Jacobian for 2-DOF planar arm (analytical differentiation)
3. Use Jacobian to convert joint velocities to end-effector velocity
4. Identify singularities (Jacobian rank deficiency)

**Interactive Python** (3 exercises):
1. **Exercise 2.5.1**: Compute Jacobian for 2-DOF planar arm
   - Starter code: `def jacobian_2dof(theta1, theta2, L1, L2):`
   - Test: Verify Jacobian using numerical differentiation

2. **Exercise 2.5.2**: Compute end-effector velocity from joint velocities
   - Starter code: `v_ee = jacobian @ joint_velocities`
   - Test: Verify velocity calculation

3. **Exercise 2.5.3**: Detect singularities (compute determinant)
   - Starter code: `def is_singular(jacobian, tol=1e-6):`
   - Test: Identify fully extended configuration as singular

**Try With AI** (1 exercise):
1. **Role**: Evaluator
   - **Scenario**: Validating Jacobian computation
   - **Task**: Verify Jacobian implementation
   - **AI Prompt**: "I computed the Jacobian: [paste code]. Verify this against the analytical derivation. Are the partial derivatives correct?"
   - **Success**: Jacobian matches analytical derivation

**Quiz Distribution**: 10 questions

---

### Chapter 2 Quiz

**Total Questions**: 50
**Distribution** across lessons: 10 per lesson
**Bloom's Distribution**: Remember 20%, Understand 25%, Apply 35%, Analyze 20%
**Covers**: Coordinate frames, transformations, FK, IK, Jacobians, singularities

---

## Chapter 3: Computer Vision for Robotics

**Goal**: Enable robots to perceive environment using cameras. Implement vision-based navigation (Success Eval SC-003: 90%+ success rate navigating simulated environment).

**CEFR Level**: B1+ to B2
**Estimated Time**: 240 minutes (4 hours)
**Prerequisites**: Chapter 1, basic linear algebra
**Success Criteria**: Students implement obstacle detection and path planning using simulated camera input.

### Lessons (5)

#### Lesson 3.1: Image Representation and Basic Operations

**Duration**: 45 minutes | **CEFR**: B1+ | **Bloom's**: Apply | **DigComp**: 4

**Learning Objectives**:
1. Represent images as NumPy arrays
2. Perform basic operations (crop, resize, rotate, threshold)
3. Convert color spaces (RGB to grayscale)
4. Display images with Matplotlib

**Interactive Python** (3 exercises):
1. Load and display synthetic image
2. Convert RGB to grayscale
3. Apply binary threshold for segmentation

**Try With AI** (1 exercise):
- Role: Copilot - Optimize image processing pipeline

**Quiz Distribution**: 10 questions

---

#### Lesson 3.2: Edge Detection and Feature Extraction

**Duration**: 50 minutes | **CEFR**: B2 | **Bloom's**: Apply | **DigComp**: 5

**Learning Objectives**:
1. Apply Sobel/Canny edge detection
2. Extract features (corners, blobs)
3. Filter noise before edge detection

**Interactive Python** (3 exercises):
1. Implement Sobel edge detector
2. Apply Canny edge detection
3. Detect corners using gradient analysis

**Try With AI** (2 exercises):
- Role: Teacher - Understand edge detection algorithms
- Role: Evaluator - Debug edge detection parameters

**Quiz Distribution**: 10 questions

---

#### Lesson 3.3: Object Detection and Recognition

**Duration**: 55 minutes | **CEFR**: B2 | **Bloom's**: Apply | **DigComp**: 5

**Learning Objectives**:
1. Segment objects by color/shape
2. Compute object centroid and orientation
3. Classify objects based on features
4. Track objects across frames

**Interactive Python** (4 exercises):
1. Segment objects by color threshold
2. Compute object centroid
3. Calculate object orientation
4. Track object position over time

**Try With AI** (2 exercises):
- Role: Copilot - Implement color-based object detection
- Role: Evaluator - Improve tracking robustness

**Quiz Distribution**: 10 questions

---

#### Lesson 3.4: Depth Perception and Obstacle Detection

**Duration**: 50 minutes | **CEFR**: B2 | **Bloom's**: Apply | **DigComp**: 5

**Learning Objectives**:
1. Simulate depth from camera (simulated depth image)
2. Detect obstacles from depth data
3. Compute free space for navigation
4. Represent environment as occupancy grid

**Interactive Python** (3 exercises):
1. Process depth image (threshold for obstacles)
2. Create occupancy grid from depth
3. Find free space for robot motion

**Try With AI** (1 exercise):
- Role: Copilot - Optimize obstacle detection

**Quiz Distribution**: 10 questions

---

#### Lesson 3.5: Vision-Based Navigation

**Duration**: 60 minutes | **CEFR**: B2 | **Bloom's**: Evaluate | **DigComp**: 6

**Learning Objectives**:
1. Implement visual servoing (move toward target)
2. Avoid obstacles using vision
3. Plan path through free space
4. Integrate vision with motion control

**Interactive Python** (4 exercises):
1. Compute heading to target from image
2. Detect obstacles in path
3. Implement simple path planner (bug algorithm)
4. Complete vision-based navigation challenge

**Try With AI** (2 exercises):
- Role: Copilot - Design navigation strategy
- Role: Evaluator - Test navigation in edge cases

**Quiz Distribution**: 10 questions

---

### Chapter 3 Quiz

**Total Questions**: 50
**Success Eval Alignment**: Questions test ability to implement vision-based navigation (SC-003)

---

## Chapter 4: Sensor Integration and Perception

**Goal**: Fuse multiple sensors (camera, IMU, encoders) for robust perception.

**CEFR Level**: B1+
**Estimated Time**: 180 minutes
**Prerequisites**: Chapters 1-3
**Success Criteria**: Students process sensor data and fuse information for robot state estimation.

### Lessons (4)

#### Lesson 4.1: Sensor Noise and Filtering (45 min, B1+)
- Gaussian noise models
- Low-pass filters
- Moving average, exponential smoothing

#### Lesson 4.2: Sensor Fusion Basics (50 min, B1+)
- Complementary filter
- Weighted average fusion
- IMU + encoder fusion

#### Lesson 4.3: Robot Localization (50 min, B2)
- Dead reckoning
- Odometry from encoders
- Drift correction

#### Lesson 4.4: Mapping and SLAM Introduction (35 min, B2)
- Occupancy grid mapping
- Simultaneous localization and mapping (conceptual)
- Simple map representation

**Interactive Python**: 3-4 exercises per lesson (total 13)
**Try With AI**: 1-2 per lesson (total 5)
**Quiz**: 50 questions

---

## Chapter 5: Motion Planning and Control

**Goal**: Plan collision-free paths and track trajectories with feedback control.

**CEFR Level**: B2
**Estimated Time**: 430 minutes
**Prerequisites**: Chapters 2, 3, 4
**Success Criteria**: Students implement path planning, trajectory tracking controllers, and Nav2 navigation for bipedal humanoids.

### Lessons (7)

#### Lesson 5.1: Configuration Space and Path Planning (55 min, B2)
- C-space representation
- Graph search (A*, Dijkstra)
- RRT basics

#### Lesson 5.2: Trajectory Generation (50 min, B2)
- Point-to-point trajectories
- Polynomial interpolation
- Velocity/acceleration limits

#### Lesson 5.3: Feedback Control (PID) (60 min, B2)
- PID controller design
- Tuning (Kp, Ki, Kd)
- Trajectory tracking

#### Lesson 5.4: Feedback Control (PID, State-Space) (60 min, B2)
- PID controller design
- State-space control
- Trajectory tracking

#### Lesson 5.5: Model Predictive Control (MPC) (60 min, B2)
- Receding horizon optimization
- Constraint handling
- MPC for trajectory tracking

#### Lesson 5.6: Nav2 Architecture (70 min, B2)
- Nav2 framework overview
- Planners, controllers, recovery behaviors
- Costmap configuration
- ROS 2 action integration

#### Lesson 5.7: Nav2 for Bipedal Humanoids (60 min, B2)
- Humanoid-specific Nav2 configuration
- Balance constraints in path planning
- Footstep planning integration
- Gait coordination

**Interactive Python**: 3-4 exercises per lesson (total 23)
**Try With AI**: 1-2 per lesson (total 11)
**Quiz**: 10 questions

---

## Chapter 6: Reinforcement Learning for Robotics

**Goal**: Train RL agents for robot control tasks (Success Eval SC-004: 80%+ expert performance).

**CEFR Level**: B2
**Estimated Time**: 220 minutes
**Prerequisites**: Chapters 1-5, basic probability
**Success Criteria**: Students train RL agent that learns pole balancing and achieves 80%+ expert performance.

### Lessons (4)

#### Lesson 6.1: RL Fundamentals (50 min, B2)
- MDP formulation
- Rewards, states, actions
- Value functions

#### Lesson 6.2: Q-Learning (60 min, B2)
- Q-table
- Bellman equation
- Epsilon-greedy exploration

#### Lesson 6.3: Training RL Agents for Robot Control (70 min, B2+)
- Pole balancing task
- Reward shaping
- Training loop

#### Lesson 6.4: Policy Gradients Introduction (40 min, B2+)
- Policy-based methods
- Simple policy gradient
- Comparison with Q-learning

**Interactive Python**: 3-4 exercises per lesson (total 15)
**Try With AI**: 2 per lesson (total 8)
**Quiz**: 50 questions

---

## Chapter 7: Humanoid Robot Design and Control

**Goal**: Design humanoid robot controllers for bipedal locomotion and balance (Success Eval SC-005: 30s stability).

**CEFR Level**: B2+
**Estimated Time**: 220 minutes
**Prerequisites**: Chapters 2, 5
**Success Criteria**: Students implement balance controller maintaining stability for 30+ seconds under perturbations.

### Lessons (4)

#### Lesson 7.1: Humanoid Robot Structure (50 min, B2)
- Degrees of freedom
- Joint configuration
- COM calculation

#### Lesson 7.2: Balance and Stability (60 min, B2+)
- Zero Moment Point (ZMP)
- Center of Pressure (COP)
- Static vs. dynamic balance

#### Lesson 7.3: Gait Planning and Walking (70 min, B2+)
- Bipedal gait cycle
- Foot placement
- Walking trajectory

#### Lesson 7.4: Whole-Body Control (40 min, B2+)
- Inverse dynamics
- Task prioritization
- Redundancy resolution

**Interactive Python**: 3-4 exercises per lesson (total 14)
**Try With AI**: 2 per lesson (total 8)
**Quiz**: 50 questions

---

## Chapter 8: AI Agents for Robot Coordination

**Goal**: Multi-agent systems and coordination strategies.

**CEFR Level**: B2+
**Estimated Time**: 150 minutes
**Prerequisites**: Chapters 1-7
**Success Criteria**: Students understand multi-agent coordination and implement basic swarm behavior.

### Lessons (3)

#### Lesson 8.1: Multi-Agent Systems (50 min, B2+)
- Agent communication
- Distributed decision-making
- Consensus algorithms

#### Lesson 8.2: Task Allocation and Coordination (60 min, B2+)
- Task assignment
- Auction-based allocation
- Formation control

#### Lesson 8.3: Swarm Robotics (40 min, B2+)
- Emergent behavior
- Flocking algorithms
- Collective intelligence

**Interactive Python**: 3 exercises per lesson (total 9)
**Try With AI**: 2 per lesson (total 6)
**Quiz**: 50 questions

---

## Chapter 9: Real-world Applications and Case Studies

**Goal**: Apply Physical AI concepts to real-world scenarios.

**CEFR Level**: B2+
**Estimated Time**: 120 minutes
**Prerequisites**: All previous chapters
**Success Criteria**: Students analyze case studies and apply learned concepts to novel problems (SC-015: 70%+ accuracy).

### Lessons (3)

#### Lesson 9.1: Warehouse Automation (40 min, B2+)
- Case study: Amazon Kiva robots
- System architecture
- Challenges and solutions

#### Lesson 9.2: Healthcare Robotics (40 min, B2+)
- Case study: Surgical robots
- Safety requirements
- Human-robot interaction

#### Lesson 9.3: Autonomous Vehicles (40 min, B2+)
- Case study: Self-driving cars
- Sensor fusion
- Decision-making under uncertainty

**Interactive Python**: 2 exercises per lesson (total 6)
**Try With AI**: 1 per lesson (total 3)
**Quiz**: 50 questions

---

## Implementation Workflow

### Phase 1: Infrastructure Setup (Week 1)

**Tasks**:
1. Initialize Docusaurus project
2. Configure Pyodide integration
3. Develop custom components:
   - InteractivePython (CodeMirror + Pyodide execution)
   - TryWithAI (prompt template display + reflection)
   - LearningObjectives (CEFR/Bloom's display)
   - Prerequisites (checklist with links)
4. Configure KaTeX for math rendering
5. Setup localStorage progress tracking
6. Create component usage documentation

**Deliverables**:
- Working Docusaurus site with all components
- Component test suite
- Developer quickstart guide

### Phase 2: Content Creation (Weeks 2-6)

**Approach**: Create content chapter-by-chapter using skills

**For Each Chapter**:
1. **Plan** (using book-scaffolding skill):
   - Define learning objectives for each lesson
   - Map CEFR levels and Bloom's taxonomy
   - Identify code examples needed
   - Plan quiz question distribution

2. **Write Lessons** (using multiple skills):
   - Invoke learning-objectives skill for each lesson
   - Write Layer 1-4 content (Foundation → Examples → Practice → Application)
   - Use code-example-generator for InteractivePython exercises
   - Create TryWithAI exercises with AI Three Roles Framework

3. **Generate Quiz** (using quiz-generator skill):
   - Create 50 questions distributed across lessons
   - Align with Bloom's taxonomy
   - Include explanations for all answers

4. **Validate** (using content-evaluation-framework skill):
   - Check technical accuracy
   - Verify pedagogical effectiveness
   - Validate CEFR alignment
   - Test all Python code in Pyodide

5. **Chapter Integration**:
   - Update navigation (sidebars.js)
   - Link prerequisites
   - Test full chapter flow

**Timeline**:
- Chapters 1-3 (Foundation): Week 2-3
- Chapters 4-6 (Building): Week 4-5
- Chapters 7-9 (Advanced): Week 6

### Phase 3: Testing & Validation (Week 7)

**Tasks**:
1. **Technical Testing**:
   - All InteractivePython exercises execute in Pyodide
   - Code runtime < 30 seconds
   - Quiz components function correctly
   - Progress tracking saves/loads
   - Export/import works

2. **Content Validation**:
   - CEFR levels appropriate
   - Bloom's levels match assessments
   - Prerequisites form acyclic graph
   - Duration estimates accurate

3. **Accessibility Testing**:
   - Keyboard navigation works
   - Screen reader compatibility
   - WCAG 2.1 AA compliance
   - Alt text for all images

4. **Performance Testing**:
   - Page load < 3 seconds
   - Build size < 100MB
   - localStorage < 5MB

5. **User Testing** (if possible):
   - Pilot with sample students
   - Gather feedback
   - Iterate on clarity

### Phase 4: Deployment (Week 8)

**Tasks**:
1. Final build optimization
2. Setup GitHub Pages / Netlify / Vercel
3. Configure custom domain (if applicable)
4. Setup CI/CD for automatic deployment
5. Create user documentation (README)
6. Announce and launch

---

## Success Metrics & Validation

### Content Quality Metrics

- ✅ All lessons have YAML frontmatter with SpecKit Plus metadata (generated_by, source_spec, created, last_modified, git_author, workflow, version, prerequisites, has_interactive_python, interactive_python_count, has_try_with_ai, try_with_ai_count, tags)
- ✅ CEFR levels: B1-B2+ range across all lessons
- ✅ 39 lessons total (7 chapters with 4-8 lessons each)
- ✅ 100+ InteractivePython exercises (3+ per lesson minimum)
- ✅ 40+ TryWithAI exercises (1-2 per lesson)
- ✅ 100+ quiz questions (10-50 per chapter based on complexity)
- ✅ All prerequisites form acyclic dependency graph
- ✅ Total content duration: ~2430 minutes (aligned with 13-week course structure)

### Technical Quality Metrics

- ✅ All Python code executes in Pyodide < 30 seconds
- ✅ Quiz scoring < 2 seconds
- ✅ Build output < 100MB
- ✅ Zero accessibility violations (WAVE/axe)
- ✅ Works in Chrome 57+, Firefox 52+, Safari 11+, Edge 79+

### Learning Outcome Metrics (Success Evals from Spec)

- **SC-002**: Students build ROS 2 systems with nodes, topics, services, actions (Chapter 2)
- **SC-003**: Students simulate robots in Gazebo and create Unity visualizations (Chapter 3)
- **SC-004**: Students train RL agents using NVIDIA Isaac Sim achieving 80%+ expert performance (Chapter 4)
- **SC-005**: Students implement Nav2 path planning for bipedal humanoids (Chapter 5)
- **SC-006**: Students design humanoid control systems with locomotion and manipulation (Chapter 6)
- **SC-007**: Students build complete VLA systems integrating voice, LLMs, and ROS 2 (Chapter 7)
- **SC-010**: 90%+ students complete 5/7 chapters in 3-day hackathon
- **SC-016**: Students complete all objectives within 3 days (6-8 hours/day)
- **SC-017**: Students build complete VLA systems with 70%+ success rate (Chapter 7 Capstone)

---

## Skills Utilization

| Skill | Purpose | Used In |
|-------|---------|---------|
| book-scaffolding | Overall structure planning, cognitive load management | Chapter planning |
| learning-objectives | Generate CEFR/Bloom's aligned objectives | Every lesson |
| quiz-generator | Create 50-question quizzes | Every chapter |
| code-example-generator | Generate testable Python code | InteractivePython exercises |
| exercise-designer | Design progressive practice exercises | Layer 3 content |
| concept-scaffolding | Break complex concepts into digestible parts | Advanced topics (IK, RL, ZMP) |
| content-evaluation-framework | Validate lesson quality | After each lesson |
| skills-proficiency-mapper | Ensure CEFR B1-B2 alignment | Content review |
| summary-generator | Create lesson/chapter summaries | End-of-lesson summaries |

---

## Risk Mitigation

### Risk 1: Pyodide Performance Limitations
**Mitigation**: Pre-test all exercises, optimize algorithms, provide pre-trained models for expensive tasks, include loading indicators.

### Risk 2: Content Complexity Mismatch
**Mitigation**: Pilot test with representative students, adjust CEFR levels based on feedback, provide optional challenge exercises.

### Risk 3: Scope Creep Beyond 3-Day Timeline
**Mitigation**: Mark chapters by priority (P1 core: Chapters 1-3, P2: Chapters 4-6, P3: Chapters 7-9), provide recommended learning paths (1-day, 2-day, 3-day tracks).

### Risk 4: Quiz Quality Issues
**Mitigation**: Use quiz-generator skill with Bloom's taxonomy, pilot test questions, review analytics to identify problematic questions.

### Risk 5: Browser Compatibility Issues
**Mitigation**: Test on all major browsers, provide clear compatibility requirements, implement feature detection with helpful error messages.

---

## Next Steps

1. **Review this plan** with stakeholders
2. **Run `/sp.tasks`** to generate detailed task breakdown
3. **Begin Phase 1**: Infrastructure setup (Docusaurus + components)
4. **Start content creation** with Chapter 1 using skills workflow
5. **Iterate** based on validation feedback

---

## Appendix: Lesson File Template

```markdown
---
title: "Lesson Title"
description: "One-sentence summary"
chapter: X
lesson: Y
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
  - "chapter-X-lesson-Y"
learning_objectives:
  - "Objective 1"
  - "Objective 2"
success_criteria:
  - "Criterion 1"
  - "Criterion 2"

# Component counts
has_interactive_python: true
interactive_python_count: 3
has_try_with_ai: true
try_with_ai_count: 2

tags:
  - "tag1"
  - "tag2"
---

# Lesson Title

## Learning Objectives

<LearningObjectives
  cefr_level="B1"
  objectives={[
    {
      text: "Objective 1",
      blooms_level: "Apply",
      assessment_method: "Interactive Python exercise"
    }
  ]}
/>

## Prerequisites

<Prerequisites
  prerequisites={[
    {
      lessonId: "chapter-X-lesson-Y",
      title: "Previous Lesson",
      link: "/docs/chapter-X/lesson-Y"
    }
  ]}
/>

## Layer 1: Concept Introduction

[Content explaining core concepts with definitions, analogies, visual aids]

## Layer 2: Worked Examples

[Step-by-step examples demonstrating concepts]

## Layer 3: Interactive Practice

### Exercise X.Y.1: [Title]

<InteractivePython
  id="exercise-chX-topic"
  title="Exercise Title"
  starterCode={`# Python code here`}
  testCases={[...]}
  hints={["Hint 1", "Hint 2"]}
/>

## Layer 4: Application Challenges

### Try With AI: [Title]

<TryWithAI
  id="tryai-chX-topic"
  title="Challenge Title"
  role="Copilot"
  scenario="..."
  yourTask="..."
  aiPromptTemplate="..."
  successCriteria={["...", "..."]}
  reflectionQuestions={["...", "..."]}
/>

## Summary

[Key takeaways, connections to next lesson]
```

---

**Plan Status**: COMPLETE
**Ready for**: `/sp.tasks` to generate task breakdown
**Files Created**:
- ✅ `research.md` (technology decisions)
- ✅ `data-model.md` (entity definitions)
- ✅ `quickstart.md` (contributor guide)
- ✅ `contracts/component-props.json` (component interfaces)
- ✅ `plan.md` (this file - comprehensive implementation plan)

**Total Implementation Scope**:
- 9 chapters
- 36 lessons
- 450 quiz questions
- 108+ InteractivePython exercises
- 36-72 TryWithAI exercises
- ~1750 minutes of content
- Static Docusaurus site with Pyodide
