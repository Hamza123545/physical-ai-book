---
id: 0007
title: Chapter Indexes Update - Course Structure Alignment
stage: implement
date: 2025-11-29
surface: agent
model: claude-sonnet-4-5-20250929
feature: 002-physical-ai-textbook
branch: 002-physical-ai-textbook
user: hswat
command: /sp.implement
labels: ["course-structure", "chapter-mapping", "module-alignment", "index-update"]
links:
  spec: specs/002-physical-ai-textbook/spec.md
  plan: specs/002-physical-ai-textbook/plan.md
  tasks: specs/002-physical-ai-textbook/tasks.md
files:
  - book-source/docs/chapter-01/index.md
  - book-source/docs/chapter-02/index.md
  - book-source/docs/chapter-03/index.md
  - book-source/docs/chapter-04/index.md
  - book-source/docs/chapter-05/index.md
  - book-source/docs/chapter-06/index.md
tests:
  - All chapter indexes updated with module/week mapping
  - Course structure properly documented
---

## Prompt

Update all existing chapter indexes to reflect proper 13-week course structure mapping.

**Course Structure**:
- Weeks 1-2: Introduction to Physical AI → Chapter 1
- Module 1 (Weeks 3-5): ROS 2 → Chapter 2 (kinematics) + ROS 2 content needed
- Module 2 (Weeks 6-7): Gazebo & Unity → Chapter 3 (vision) + simulation content needed
- Module 3 (Weeks 8-10): NVIDIA Isaac → Chapters 4 & 5 (RL + motion planning) + Isaac content needed
- Weeks 11-12: Humanoid Development → Chapter 6 (incomplete)
- Module 4 (Week 13): VLA → Missing, needs creation

**Files Updated**:
1. chapter-01/index.md - Added "Part of: Weeks 1-2"
2. chapter-02/index.md - Added "Part of: Module 1 (Weeks 3-5)" + ROS 2 note
3. chapter-03/index.md - Added "Part of: Module 2 (Weeks 6-7)" + simulation note
4. chapter-04/index.md - Added "Part of: Module 3 (Weeks 8-10)" + Isaac note
5. chapter-05/index.md - Added "Part of: Module 3 (Weeks 8-10)" + Nav2 note
6. chapter-06/index.md - Added "Part of: Weeks 11-12" + status note

**Created Using**: SpecKit Plus `/sp.implement` via Claude Code

