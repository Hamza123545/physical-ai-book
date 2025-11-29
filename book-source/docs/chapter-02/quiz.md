---
title: "Chapter 2 Quiz"
description: "Test your understanding of ROS 2 fundamentals"
sidebar_position: 6
---

# Chapter 2 Quiz: ROS 2 Fundamentals

Test your understanding of the concepts covered in Chapter 2. This quiz covers all five lessons.

**Instructions**:
- 10 questions total
- Multiple choice format
- Immediate feedback provided
- You can retake the quiz as many times as you want

<Quiz
  chapterId={2}
  questions={[
    {
      id: "q-ch02-01",
      question: "What is a ROS 2 node?",
      type: "multiple-choice",
      options: [
        "A hardware component",
        "An independent process that performs a specific task",
        "A message type",
        "A launch file"
      ],
      correctAnswer: 1,
      explanation: "A ROS 2 node is an independent executable process that performs a specific function, such as reading sensor data, controlling motors, or running AI algorithms. Nodes communicate with each other via topics, services, and actions.",
      wrongAnswerExplanations: [
        "Nodes are software processes, not hardware components",
        "Message types define data structures, not executable processes",
        "Launch files orchestrate nodes but are not nodes themselves"
      ]
    },
    {
      id: "q-ch02-02",
      question: "What is the primary purpose of DDS in ROS 2?",
      type: "multiple-choice",
      options: [
        "To define robot structure",
        "To provide the middleware layer for message routing and discovery",
        "To create launch files",
        "To write Python code"
      ],
      correctAnswer: 1,
      explanation: "DDS (Data Distribution Service) is the middleware layer that handles low-level communication in ROS 2, including message routing, Quality of Service (QoS), node discovery, and reliability guarantees.",
      wrongAnswerExplanations: [
        "Robot structure is defined by URDF, not DDS",
        "Launch files are Python/XML files, not related to DDS",
        "DDS is middleware, not a programming language"
      ]
    },
    {
      id: "q-ch02-03",
      question: "When should you use a service instead of a topic?",
      type: "multiple-choice",
      options: [
        "For streaming sensor data continuously",
        "When you need immediate request-response communication",
        "For one-to-many communication",
        "For long-running tasks with progress feedback"
      ],
      correctAnswer: 1,
      explanation: "Services are used for synchronous request-response communication when you need an immediate answer, such as querying robot status or requesting map data. Topics are better for streaming data.",
      wrongAnswerExplanations: [
        "Streaming data should use topics, not services",
        "One-to-many communication is better suited for topics",
        "Long-running tasks with feedback should use actions, not services"
      ]
    },
    {
      id: "q-ch02-04",
      question: "What are ROS 2 actions primarily used for?",
      type: "multiple-choice",
      options: [
        "Quick status queries",
        "Long-running tasks with progress feedback",
        "Streaming sensor data",
        "Synchronous request-response"
      ],
      correctAnswer: 1,
      explanation: "Actions are designed for long-running tasks that take time to complete, need progress feedback, and can be canceled. Examples include navigation to a location or picking up an object.",
      wrongAnswerExplanations: [
        "Quick queries should use services, not actions",
        "Streaming data should use topics",
        "Synchronous communication should use services"
      ]
    },
    {
      id: "q-ch02-05",
      question: "What is the purpose of URDF (Unified Robot Description Format)?",
      type: "multiple-choice",
      options: [
        "To define robot communication protocols",
        "To describe robot structure, geometry, and physical properties",
        "To configure ROS 2 package dependencies",
        "To write launch files"
      ],
      correctAnswer: 1,
      explanation: "URDF is an XML format used to describe a robot's physical structure, including links (rigid bodies), joints (connections), visual appearance, collision geometry, and inertial properties.",
      wrongAnswerExplanations: [
        "Communication protocols are defined by ROS 2 topics/services, not URDF",
        "Package dependencies are defined in package.xml, not URDF",
        "Launch files are Python files, not URDF"
      ]
    },
    {
      id: "q-ch02-06",
      question: "In ROS 2, what is the publish/subscribe pattern used for?",
      type: "multiple-choice",
      options: [
        "Synchronous request-response communication",
        "Asynchronous one-to-many message passing",
        "Long-running task execution",
        "Package dependency management"
      ],
      correctAnswer: 1,
      explanation: "The publish/subscribe pattern enables asynchronous, decoupled communication where publishers send messages to topics and subscribers receive them. Multiple subscribers can listen to the same topic.",
      wrongAnswerExplanations: [
        "Synchronous request-response uses services, not topics",
        "Long-running tasks use actions, not topics",
        "Dependency management is handled by package.xml, not topics"
      ]
    },
    {
      id: "q-ch02-07",
      question: "What file defines the entry points for executable nodes in a ROS 2 Python package?",
      type: "multiple-choice",
      options: [
        "package.xml",
        "setup.py",
        "CMakeLists.txt",
        "launch.py"
      ],
      correctAnswer: 1,
      explanation: "The setup.py file contains entry_points configuration that defines console scripts, making Python nodes executable via 'ros2 run package_name node_name'.",
      wrongAnswerExplanations: [
        "package.xml defines metadata and dependencies, not entry points",
        "CMakeLists.txt is for C++ packages, not Python entry points",
        "launch.py files orchestrate nodes but don't define entry points"
      ]
    },
    {
      id: "q-ch02-08",
      question: "What is the correct sequence for creating and running a ROS 2 node in Python?",
      type: "multiple-choice",
      options: [
        "Create node → Spin → Initialize → Shutdown",
        "Initialize → Create node → Spin → Shutdown",
        "Spin → Initialize → Create node → Shutdown",
        "Shutdown → Initialize → Create node → Spin"
      ],
      correctAnswer: 1,
      explanation: "The correct sequence is: 1) rclpy.init() to initialize ROS 2, 2) Create your node instance, 3) rclpy.spin() to keep the node alive and process callbacks, 4) rclpy.shutdown() to clean up.",
      wrongAnswerExplanations: [
        "You must initialize before creating nodes",
        "You must initialize and create the node before spinning",
        "Shutdown should be last, not first"
      ]
    },
    {
      id: "q-ch02-09",
      question: "In a URDF file, what is the difference between visual and collision geometry?",
      type: "multiple-choice",
      options: [
        "There is no difference; they must be identical",
        "Visual is for appearance, collision is for physics simulation (can be simplified)",
        "Collision is for appearance, visual is for physics",
        "Visual defines joints, collision defines links"
      ],
      correctAnswer: 1,
      explanation: "Visual geometry defines how the robot looks in visualization tools. Collision geometry defines the shape used for physics simulation and can be simplified (e.g., using boxes instead of complex meshes) for better performance.",
      wrongAnswerExplanations: [
        "Visual and collision geometries can differ for performance",
        "Visual is for appearance, not physics",
        "Joints and links are separate URDF elements, not visual/collision"
      ]
    },
    {
      id: "q-ch02-10",
      question: "What is the primary purpose of a ROS 2 launch file?",
      type: "multiple-choice",
      options: [
        "To define message types",
        "To orchestrate multiple nodes and configure the robot system",
        "To write Python code",
        "To manage package dependencies"
      ],
      correctAnswer: 1,
      explanation: "Launch files allow you to start multiple nodes, load URDF files, set parameters, and configure entire robot systems with a single command: 'ros2 launch package_name launch_file.py'.",
      wrongAnswerExplanations: [
        "Message types are defined in .msg files, not launch files",
        "Launch files orchestrate nodes but don't contain the node code itself",
        "Dependencies are managed in package.xml, not launch files"
      ]
    }
  ]}
  questionsPerBatch={10}
/>

---

## Quiz Results Interpretation

- **8-10 correct**: Excellent! You have a strong grasp of ROS 2 fundamentals
- **6-7 correct**: Good work! Review the lessons for concepts you missed
- **4-5 correct**: Fair understanding. Revisit the lesson content and exercises
- **0-3 correct**: Review all five lessons carefully before proceeding to Chapter 3

**Next Steps**: Once you're comfortable with ROS 2 concepts, proceed to **[Chapter 3: The Digital Twin (Gazebo & Unity)](/docs/chapter-03/index.md)** to learn robot simulation!

