---
title: "Lesson 2.1: ROS 2 Architecture and Core Concepts"
description: "Learn the fundamental concepts of ROS 2: nodes, topics, services, and actions"
chapter: 2
lesson: 1
estimated_time: 60
cefr_level: "B1"
blooms_level: "Understand"
digcomp_level: 3
generated_by: "claude-sonnet-4-5-20250929"
source_spec: "specs/002-physical-ai-textbook/spec.md"
created: "2025-11-29"
last_modified: "2025-11-29"
git_author: "hswat"
workflow: "/sp.implement"
version: "1.0"
prerequisites: []
has_interactive_python: true
interactive_python_count: 3
has_try_with_ai: true
try_with_ai_count: 1
tags: ["ros2", "nodes", "topics", "dds", "middleware"]
---

# Lesson 2.1: ROS 2 Architecture and Core Concepts

## 🎯 Learning Objectives

- Understand ROS 2 architecture and design principles
- Learn about nodes, topics, services, and actions
- Understand publish/subscribe communication pattern
- Explore DDS middleware and distributed systems

**Time**: 60 minutes

---

## Introduction

**ROS 2 (Robot Operating System 2)** is middleware that enables communication between different parts of a robot system. Think of it as the "nervous system" that connects sensors, actuators, and AI algorithms.

**Key Benefits**:
- **Modularity**: Break complex systems into independent components
- **Distributed**: Run nodes on different computers
- **Language-agnostic**: Python, C++, Java, and more
- **Real-time capable**: Supports real-time robot control

---

## 1. ROS 2 Architecture Overview

### Core Components

```
┌─────────────┐
│   Node A    │  Publishes to topic
└──────┬──────┘
       │
       ▼
┌─────────────┐
│    Topic    │  Message bus
└──────┬──────┘
       │
       ▼
┌─────────────┐
│   Node B    │  Subscribes to topic
└─────────────┘
```

### Key Concepts

1. **Nodes**: Independent processes that perform specific tasks
2. **Topics**: Named channels for message passing
3. **Messages**: Data structures sent between nodes
4. **Services**: Request-response communication
5. **Actions**: Long-running tasks with feedback

---

## 2. Nodes

### What is a Node?

A **node** is an executable process that performs a specific function:
- Sensor node: Reads camera data
- Control node: Sends motor commands
- Planning node: Computes paths
- AI node: Runs machine learning models

### Node Characteristics

- **Independent**: Each node runs as separate process
- **Communicative**: Nodes exchange data via topics/services
- **Reusable**: Same node can work with different robots
- **Distributed**: Nodes can run on different machines

---

## 3. Topics and Publish/Subscribe

### Topic Communication

**Topics** are named buses where nodes exchange messages:

```python
# Publisher (Node A)
publisher.publish(message)  # Sends data

# Subscriber (Node B)
def callback(message):      # Receives data
    process(message)
```

### Publish/Subscribe Pattern

- **Publisher**: Node that sends messages
- **Subscriber**: Node that receives messages
- **Topic**: Named channel (e.g., `/camera/image`, `/cmd_vel`)
- **Message**: Data structure (e.g., Image, Twist)

**Benefits**:
- Decoupled: Publishers don't know subscribers
- Scalable: Multiple subscribers per topic
- Flexible: Easy to add/remove nodes

---

## 4. DDS Middleware

### What is DDS?

**DDS (Data Distribution Service)** is the middleware layer that handles:
- Message routing
- Quality of Service (QoS)
- Discovery (finding other nodes)
- Reliability guarantees

### ROS 2 and DDS

ROS 2 uses DDS implementations:
- **Fast DDS** (default)
- **Cyclone DDS**
- **RTI Connext** (commercial)

DDS provides the low-level communication, while ROS 2 provides the high-level API.

---

## 5. Services vs Topics

### When to Use Topics

- **Streaming data**: Sensor readings, video feeds
- **One-to-many**: One publisher, multiple subscribers
- **Fire-and-forget**: No response needed

### When to Use Services

- **Request-response**: Need immediate answer
- **One-to-one**: Direct communication
- **Synchronous**: Wait for response

**Example**: 
- Topic: `/camera/image` (continuous video stream)
- Service: `/get_map` (request map data, get response)

---

## 6. Exercises

### Exercise 2.1.1: Identify Communication Patterns

Determine whether to use topics or services for different scenarios.

<InteractivePython
  id="ex-2-1-1"
  title="Topics vs Services"
  starterCode={`def choose_communication_pattern(scenario):
    """
    Choose 'topic' or 'service' for each scenario.
    
    Scenarios:
    - "continuous_sensor_data": LiDAR scan every 100ms
    - "request_robot_status": Get current battery level
    - "video_stream": Camera feed at 30 FPS
    - "calculate_path": Request path from A to B
    - "motor_commands": Send velocity commands continuously
    """
    # TODO: Return 'topic' or 'service' for each scenario
    pass

# Test
scenarios = [
    "continuous_sensor_data",
    "request_robot_status",
    "video_stream",
    "calculate_path",
    "motor_commands"
]

for scenario in scenarios:
    pattern = choose_communication_pattern(scenario)
    print(f"{scenario}: {pattern}")
`}
  hints={[
    "Topics: continuous/streaming data, one-to-many",
    "Services: request-response, need immediate answer",
    "Sensor data → topic, status request → service"
  ]}
/>

---

### Exercise 2.1.2: Node Architecture Design

Design a simple robot system with nodes and topics.

<InteractivePython
  id="ex-2-1-2"
  title="Design Robot Architecture"
  starterCode={`def design_robot_system():
    """
    Design a robot system with nodes and topics.
    
    Robot has:
    - Camera sensor
    - LiDAR sensor
    - Motor controller
    - Navigation planner
    
    Return dict with nodes and their topics.
    """
    # TODO: Design system architecture
    system = {
        "nodes": [],
        "topics": []
    }
    return system

# Test
system = design_robot_system()
print("Nodes:", system["nodes"])
print("Topics:", system["topics"])
`}
  hints={[
    "Create nodes: camera_node, lidar_node, motor_node, planner_node",
    "Create topics: /camera/image, /lidar/scan, /cmd_vel, /plan",
    "Connect nodes via topics"
  ]}
/>

---

### Exercise 2.1.3: Message Flow Simulation

Simulate message flow through a ROS 2 system.

<InteractivePython
  id="ex-2-1-3"
  title="Message Flow Simulation"
  starterCode={`def simulate_message_flow():
    """
    Simulate messages flowing through topics.
    
    Publisher sends messages, subscriber receives them.
    """
    messages = ["msg1", "msg2", "msg3"]
    received = []
    
    # TODO: Simulate publish/subscribe
    # Publisher sends messages
    # Subscriber receives and stores them
    
    return received

# Test
received = simulate_message_flow()
print(f"Received {len(received)} messages")
`}
  hints={[
    "Publisher: iterate through messages and 'send' them",
    "Subscriber: 'receive' messages and append to received list",
    "Simulate topic as a queue"
  ]}
/>

---

## 7. Try With AI

### TryWithAI 2.1.1: Design ROS 2 System

<TryWithAI
  id="tryai-2-1-1"
  title="Design a ROS 2 Robot System"
  role="Teacher"
  scenario="You're designing a warehouse robot system with multiple sensors and actuators"
  yourTask="List 5-7 nodes your robot would need and what topics/services they would use"
  aiPromptTemplate="I'm designing a warehouse robot with these nodes: [your list]. Can you help me identify the topics and services needed, and suggest any nodes I might be missing? Also explain the communication flow."
  successCriteria={[
    "You identified at least 5 nodes",
    "You understand topic vs service selection",
    "You can explain the communication flow"
  ]}
  reflectionQuestions={[
    "Which nodes need to communicate with each other?",
    "What data needs to be shared continuously vs on-demand?",
    "How would you add a new sensor to this system?"
  ]}
/>

---

## Summary

**Key Takeaways**:

1. **ROS 2 Architecture**
   - Nodes are independent processes
   - Topics enable publish/subscribe communication
   - Services provide request-response patterns
   - DDS middleware handles low-level communication

2. **Communication Patterns**
   - Topics: Streaming data, one-to-many
   - Services: Request-response, one-to-one
   - Actions: Long-running tasks with feedback

3. **Design Principles**
   - Modular: Break system into nodes
   - Decoupled: Nodes communicate via topics
   - Distributed: Run on different machines
   - Reusable: Same nodes work with different robots

**What's Next**: [Lesson 2.2: ROS 2 Nodes and Topics](./lesson-02-nodes-topics.md) teaches you to implement actual ROS 2 nodes using Python.

---

**Estimated completion time**: 60 minutes | **Prerequisites**: Chapter 1 (Python basics) | **Difficulty**: B1 (Intermediate)

