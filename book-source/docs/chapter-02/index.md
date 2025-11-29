---
title: "Chapter 2: ROS 2 Fundamentals - The Robotic Nervous System"
description: "Master ROS 2 (Robot Operating System) - the middleware that connects AI to robot hardware"
sidebar_position: 2
---

# Chapter 2: ROS 2 Fundamentals - The Robotic Nervous System

**Part of**: Module 1 - The Robotic Nervous System (ROS 2) | Weeks 3-5  
**Course Structure**: This chapter covers the middleware that bridges Python AI agents to ROS 2 controllers.

## Overview

**ROS 2 (Robot Operating System 2)** is the middleware that bridges the gap between AI software and robot hardware. Just as the nervous system connects your brain to your body, ROS 2 connects your AI algorithms to robot sensors and actuators.

**What you'll master:**
- ROS 2 architecture and core concepts
- Nodes, topics, services, and actions
- Building ROS 2 packages with Python (rclpy)
- Launch files and parameter management
- URDF (Unified Robot Description Format) for humanoids

By the end, you'll build a complete ROS 2 system that controls a simulated humanoid robot!

---

## Chapter Learning Outcomes

After completing this chapter, you will be able to:

1. **Understand** ROS 2 architecture and communication patterns (CEFR: B1, Bloom's: Understand)
2. **Implement** ROS 2 nodes using rclpy (Python) (CEFR: B1+, Bloom's: Apply)
3. **Design** topic-based communication for sensor data and control commands (CEFR: B1+, Bloom's: Apply)
4. **Create** ROS 2 packages with proper structure (CEFR: B2, Bloom's: Apply)
5. **Model** robots using URDF for humanoid descriptions (CEFR: B2, Bloom's: Apply)

---

## Lessons

### [Lesson 2.1: ROS 2 Architecture and Core Concepts](./lesson-01-ros2-architecture.md)
**Duration**: 60 minutes | **Difficulty**: B1 (Intermediate)

Learn the fundamental concepts of ROS 2: nodes, topics, services, and actions. Understand how ROS 2 enables distributed robot systems.

**Key Concepts**: ROS 2 architecture, nodes, topics, publish/subscribe, DDS middleware

**Exercises**: 3 interactive Python exercises + 1 Try With AI

---

### [Lesson 2.2: ROS 2 Nodes and Topics](./lesson-02-nodes-topics.md)
**Duration**: 70 minutes | **Difficulty**: B1+ (Upper Intermediate)

Implement ROS 2 nodes that publish and subscribe to topics. Build sensor data publishers and control command subscribers.

**Key Concepts**: rclpy node creation, publishers, subscribers, message types, topic communication

**Exercises**: 4 interactive Python exercises + 2 Try With AI

---

### [Lesson 2.3: ROS 2 Services and Actions](./lesson-03-services-actions.md)
**Duration**: 60 minutes | **Difficulty**: B1+ (Upper Intermediate)

Use ROS 2 services for request-response communication and actions for long-running tasks with feedback.

**Key Concepts**: Services, actions, request-response patterns, action servers/clients, feedback

**Exercises**: 3 interactive Python exercises + 1 Try With AI

---

### [Lesson 2.4: Building ROS 2 Packages with Python (rclpy)](./lesson-04-rclpy-packages.md)
**Duration**: 70 minutes | **Difficulty**: B2 (Advanced)

Create complete ROS 2 packages with proper structure. Build Python agents that integrate with ROS 2 controllers.

**Key Concepts**: ROS 2 package structure, setup.py, CMakeLists.txt, rclpy Python agents, package dependencies

**Exercises**: 4 interactive Python exercises + 2 Try With AI

---

### [Lesson 2.5: URDF for Humanoids and Launch Files](./lesson-05-urdf-launch.md)
**Duration**: 60 minutes | **Difficulty**: B2 (Advanced)

Model humanoid robots using URDF (Unified Robot Description Format). Create launch files for complex robot systems.

**Key Concepts**: URDF syntax, robot links and joints, humanoid modeling, launch files, parameter management

**Exercises**: 3 interactive Python exercises + 1 Try With AI

---

## Chapter Assessment

### [Chapter 2 Quiz](./quiz.md)
Test your understanding with a comprehensive 10-question quiz covering all ROS 2 concepts.

---

## Prerequisites

This chapter builds on **Chapter 1: Introduction to Physical AI**. You should be comfortable with:
- Python programming (functions, classes, modules)
- Basic understanding of distributed systems
- Command-line interface (Linux/Ubuntu)

**Note**: ROS 2 runs natively on Linux (Ubuntu 22.04). For Windows/Mac users, we'll use simplified examples that work in Pyodide.

---

## Proficiency Mapping

| Aspect | Level | Framework |
|--------|-------|-----------|
| Language Complexity | B1-B2 | CEFR |
| Cognitive Demand | Understand → Apply | Bloom's Taxonomy |
| Digital Skills | 3-5 (Intermediate → Advanced) | DigComp 2.2 |
| Prerequisites | Python, basic Linux commands | - |

---

## 📝 Content Creation Workflow

**Created Using**: SpecKit Plus workflow via Claude Code
- **Spec Phase**: `/sp.specify` - Defined ROS 2 module requirements
- **Plan Phase**: `/sp.plan` - Structured 5 lessons with learning objectives
- **Tasks Phase**: `/sp.tasks` - Created implementation checklist
- **Implement Phase**: `/sp.implement` - Created content using `physical-ai-content-writer` subagent

**Constitution Compliance**:
- ✅ 4-Layer Teaching Method (Foundation → Application → Integration → Innovation)
- ✅ AI Three Roles Framework (Teacher, Copilot, Evaluator)
- ✅ CEFR Cognitive Load Management (B1-B2)
- ✅ SpecKit Plus Workflow (spec→plan→tasks→implement)

---

**Ready to start?** Begin with [Lesson 2.1: ROS 2 Architecture and Core Concepts](./lesson-01-ros2-architecture.md)!
