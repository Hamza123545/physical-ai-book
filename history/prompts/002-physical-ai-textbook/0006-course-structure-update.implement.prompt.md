---
id: 0006
title: Course Structure Update - 13 Week Module Alignment
stage: implement
date: 2025-11-29
surface: agent
model: claude-sonnet-4-5-20250929
feature: 002-physical-ai-textbook
branch: 002-physical-ai-textbook
user: hswat
command: /sp.implement
labels: ["course-structure", "modules", "13-weeks", "ros2", "gazebo", "isaac", "vla"]
links:
  spec: specs/002-physical-ai-textbook/spec.md
  plan: specs/002-physical-ai-textbook/plan.md
  tasks: specs/002-physical-ai-textbook/tasks.md
files:
  - book-source/docs/intro.md
  - specs/002-physical-ai-textbook/spec.md
  - specs/002-physical-ai-textbook/plan.md
  - .specify/memory/constitution.md
  - README.md
tests:
  - Course structure alignment verification
  - Module mapping validation
---

## Prompt

Update all existing files to reflect proper 13-week course structure for Physical AI & Humanoid Robotics textbook.

**Course Structure**:
- Weeks 1-2: Introduction to Physical AI
- Module 1 (Weeks 3-5): ROS 2 - The Robotic Nervous System
- Module 2 (Weeks 6-7): Gazebo & Unity - The Digital Twin
- Module 3 (Weeks 8-10): NVIDIA Isaac - The AI-Robot Brain
- Weeks 11-12: Humanoid Robot Development
- Module 4 (Week 13): Vision-Language-Action (VLA) + Capstone

**Files to Update**:
1. `book-source/docs/intro.md` - Update course overview with 13-week structure
2. `specs/002-physical-ai-textbook/spec.md` - Add course structure section
3. `specs/002-physical-ai-textbook/plan.md` - Update summary with module structure
4. `.specify/memory/constitution.md` - Already updated with course structure
5. `README.md` - Already updated with course content

**Requirements**:
- All files must reflect 13-week course structure
- Modules clearly mapped to weeks
- Capstone project mentioned in Module 4
- SpecKit Plus methodology documented in all files

**Created Using**: SpecKit Plus `/sp.implement` via Claude Code

