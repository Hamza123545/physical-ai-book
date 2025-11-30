---
title: "Lesson 2.2: ROS 2 Nodes and Topics"
description: "Implement ROS 2 nodes that publish and subscribe to topics using rclpy"
chapter: 2
lesson: 2
estimated_time: 70
cefr_level: "B1+"
blooms_level: "Apply"
digcomp_level: 4
generated_by: "claude-sonnet-4-5-20250929"
source_spec: "specs/002-physical-ai-textbook/spec.md"
created: "2025-11-29"
last_modified: "2025-11-29"
git_author: "hswat"
workflow: "/sp.implement"
version: "1.0"
prerequisites: ["chapter-02-lesson-01"]
has_interactive_python: true
interactive_python_count: 4
has_try_with_ai: true
try_with_ai_count: 2
tags: ["ros2", "rclpy", "publishers", "subscribers", "topics"]
---

# Lesson 2.2: ROS 2 Nodes and Topics

## 🎯 Learning Objectives

- Create ROS 2 nodes using rclpy (Python)
- Implement publishers to send messages
- Implement subscribers to receive messages
- Work with standard ROS 2 message types
- Build sensor data publishers and control subscribers

**Time**: 70 minutes

---

## Prerequisites

<Prerequisites
  prerequisites={[
    {
      lessonId: "chapter-02-lesson-01",
      title: "Lesson 2.1: ROS 2 Architecture and Core Concepts",
      link: "/docs/chapter-02/lesson-01-ros2-architecture"
    }
  ]}
/>

## Introduction

In this lesson, you'll implement actual ROS 2 nodes using **rclpy** (ROS 2 Client Library for Python). You'll build nodes that publish sensor data and subscribe to control commands—the foundation of all ROS 2 robot systems.

**Key Skills**:
- Node creation and initialization
- Publisher/subscriber setup
- Message creation and publishing
- Callback functions for subscribers

---

## 1. Creating a ROS 2 Node

### Basic Node Structure

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')
        self.get_logger().info('Node started!')

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Node Lifecycle

1. **Initialize**: `rclpy.init()`
2. **Create Node**: Instantiate your node class
3. **Spin**: `rclpy.spin()` keeps node alive
4. **Shutdown**: `rclpy.shutdown()` cleans up

---

## 2. Creating a Publisher

### Publisher Example

```python
from std_msgs.msg import String

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')
        # Create publisher
        self.publisher = self.create_publisher(
            String,           # Message type
            '/sensor_data',   # Topic name
            10                # Queue size
        )
        # Create timer to publish periodically
        self.timer = self.create_timer(1.0, self.publish_data)
    
    def publish_data(self):
        msg = String()
        msg.data = "Sensor reading: 42.5"
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')
```

### Key Components

- **Message Type**: `String`, `Int32`, `Float64`, etc.
- **Topic Name**: `/sensor_data` (use forward slash)
- **Queue Size**: Buffer for messages (typically 10)

---

## 3. Creating a Subscriber

### Subscriber Example

```python
from std_msgs.msg import String

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        # Create subscriber
        self.subscription = self.create_subscription(
            String,              # Message type
            '/sensor_data',      # Topic name
            self.listener_callback,  # Callback function
            10                   # Queue size
        )
    
    def listener_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')
        # Process the message
        self.process_command(msg.data)
    
    def process_command(self, data):
        # Your processing logic here
        pass
```

### Callback Functions

- Called automatically when message arrives
- Receives message as parameter
- Should be fast (don't block)
- Can publish new messages if needed

---

## 4. Standard Message Types

### Common Message Types

```python
from std_msgs.msg import String, Int32, Float64, Bool
from geometry_msgs.msg import Twist, Point, Pose
from sensor_msgs.msg import Image, LaserScan, Imu
```

### Example: Publishing Twist (Velocity)

```python
from geometry_msgs.msg import Twist

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.publisher = self.create_publisher(
            Twist, '/cmd_vel', 10
        )
    
    def send_velocity(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.publisher.publish(msg)
```

---

## 5. Complete Example: Sensor Publisher + Control Subscriber

### Sensor Node (Publisher)

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import random

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')
        self.publisher = self.create_publisher(
            Float64, '/distance_sensor', 10
        )
        self.timer = self.create_timer(0.1, self.publish_reading)
        self.get_logger().info('Sensor node started')
    
    def publish_reading(self):
        # Simulate sensor reading
        reading = random.uniform(0.1, 5.0)
        msg = Float64()
        msg.data = reading
        self.publisher.publish(msg)
        self.get_logger().info(f'Published distance: {reading:.2f} m')
```

### Control Node (Subscriber)

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.subscription = self.create_subscription(
            Float64, '/distance_sensor',
            self.distance_callback, 10
        )
        self.get_logger().info('Control node started')
    
    def distance_callback(self, msg):
        distance = msg.data
        if distance < 0.5:
            self.get_logger().warn('Too close! Stop!')
        elif distance < 1.0:
            self.get_logger().info('Approaching obstacle')
        else:
            self.get_logger().info('Path clear')
```

---

## 6. Exercises

### Exercise 2.2.1: Create a Simple Publisher

Create a node that publishes a counter value.

<InteractivePython
  id="ex-2-2-1"
  title="Simple Publisher Node"
  starterCode={`class CounterNode:
    """Simulate a ROS 2 node that publishes counter values."""
    
    def __init__(self):
        self.counter = 0
        self.messages_published = []
    
    def publish(self):
        """Publish current counter value."""
        # TODO: Increment counter and store message
        pass
    
    def get_published_messages(self):
        return self.messages_published

# Test
node = CounterNode()
for _ in range(5):
    node.publish()

print("Published messages:", node.get_published_messages())
`}
  hints={[
    "Increment self.counter each time",
    "Append counter value to self.messages_published",
    "Simulate publishing by storing the value"
  ]}
/>

---

### Exercise 2.2.2: Create a Subscriber with Callback

Create a subscriber that processes incoming messages.

<InteractivePython
  id="ex-2-2-2"
  title="Subscriber with Callback"
  starterCode={`class MessageSubscriber:
    """Simulate a ROS 2 subscriber node."""
    
    def __init__(self):
        self.received_messages = []
        self.processed_count = 0
    
    def callback(self, message):
        """Process incoming message."""
        # TODO: Store message and increment count
        pass
    
    def receive_messages(self, messages):
        """Simulate receiving messages."""
        for msg in messages:
            self.callback(msg)

# Test
subscriber = MessageSubscriber()
incoming = ["msg1", "msg2", "msg3", "msg4"]
subscriber.receive_messages(incoming)

print(f"Received: {subscriber.received_messages}")
print(f"Processed: {subscriber.processed_count}")
`}
  hints={[
    "Append message to self.received_messages in callback",
    "Increment self.processed_count",
    "Callback is called for each message"
  ]}
/>

---

### Exercise 2.2.3: Build Sensor-Controller System

Create a complete system with sensor publisher and controller subscriber.

<InteractivePython
  id="ex-2-2-3"
  title="Sensor-Controller System"
  starterCode={`class SensorControllerSystem:
    """Complete sensor-controller system."""
    
    def __init__(self):
        self.sensor_data = []
        self.control_actions = []
    
    def sensor_publish(self, value):
        """Sensor publishes data."""
        # TODO: Store sensor reading
        pass
    
    def controller_subscribe(self, sensor_value):
        """Controller receives and processes sensor data."""
        # TODO: Process sensor value and generate control action
        # If value < 0.5, return "STOP"
        # If value < 1.0, return "SLOW"
        # Otherwise, return "GO"
        pass
    
    def run_simulation(self, sensor_readings):
        """Simulate sensor-controller interaction."""
        for reading in sensor_readings:
            self.sensor_publish(reading)
            action = self.controller_subscribe(reading)
            self.control_actions.append(action)

# Test
system = SensorControllerSystem()
readings = [2.0, 1.5, 0.8, 0.3, 1.2]
system.run_simulation(readings)

print("Sensor data:", system.sensor_data)
print("Control actions:", system.control_actions)
`}
  hints={[
    "Store sensor reading in sensor_publish",
    "Controller logic: < 0.5 = STOP, < 1.0 = SLOW, else = GO",
    "Append action to control_actions"
  ]}
/>

---

### Exercise 2.2.4: Message Type Conversion

Convert between different message representations.

<InteractivePython
  id="ex-2-2-4"
  title="Message Type Handling"
  starterCode={`def create_twist_message(linear_x, angular_z):
    """
    Create a Twist message (velocity command).
    
    Args:
        linear_x: Forward velocity (m/s)
        angular_z: Angular velocity (rad/s)
    
    Returns:
        Dict representing Twist message
    """
    # TODO: Create message structure
    # Twist has linear (x, y, z) and angular (x, y, z)
    pass

# Test
msg = create_twist_message(0.5, 0.2)
print("Twist message:", msg)
print(f"Linear X: {msg['linear']['x']}")
print(f"Angular Z: {msg['angular']['z']}")
`}
  hints={[
    "Return dict with 'linear' and 'angular' keys",
    "Each has 'x', 'y', 'z' components",
    "Set linear.x and angular.z, others to 0"
  ]}
/>

---

## 7. Try With AI

### TryWithAI 2.2.1: Design Sensor Node

<TryWithAI
  id="tryai-2-2-1"
  title="Design a Camera Sensor Node"
  role="Copilot"
  scenario="You need to create a ROS 2 node that publishes camera images"
  yourTask="Write pseudocode for a camera node that publishes images at 30 FPS. Include node setup, publisher creation, and publishing logic."
  aiPromptTemplate="I'm creating a camera sensor node in ROS 2. Here's my pseudocode: [your code]. Can you review it for correctness, suggest improvements, and help me understand what message type I should use for images?"
  successCriteria={[
    "You understand node structure",
    "You know how to create publishers",
    "You understand timing/publishing frequency"
  ]}
  reflectionQuestions={[
    "What message type should be used for images?",
    "How do you control publishing frequency?",
    "What happens if publishing is too fast?"
  ]}
/>

---

### TryWithAI 2.2.2: Debug Subscriber Code

<TryWithAI
  id="tryai-2-2-2"
  title="Debug Subscriber Implementation"
  role="Evaluator"
  scenario="Your subscriber isn't receiving messages"
  yourTask="Write a subscriber node that should receive Float64 messages on /sensor_data topic, but it's not working. Include your code and describe the problem."
  aiPromptTemplate="I wrote this subscriber code: [your code]. It's not receiving messages. Can you help me debug it? Check for common mistakes like wrong topic names, message types, or missing spin() calls."
  successCriteria={[
    "You understand common subscriber mistakes",
    "You can identify missing components",
    "You understand the callback mechanism"
  ]}
  reflectionQuestions={[
    "What are common reasons subscribers don't receive messages?",
    "How do you verify a topic has publishers?",
    "What's the purpose of rclpy.spin()?"
  ]}
/>

---

## Summary

**Key Takeaways**:

1. **Node Creation**
   - Inherit from `Node` class
   - Initialize with `super().__init__()`
   - Use `rclpy.spin()` to keep node alive

2. **Publishers**
   - Create with `create_publisher()`
   - Specify message type, topic name, queue size
   - Use timers for periodic publishing

3. **Subscribers**
   - Create with `create_subscription()`
   - Provide callback function
   - Callback receives message automatically

4. **Message Types**
   - Standard types: `String`, `Int32`, `Float64`
   - Geometry: `Twist`, `Point`, `Pose`
   - Sensors: `Image`, `LaserScan`, `Imu`

**What's Next**: [Lesson 2.3: ROS 2 Services and Actions](./lesson-03-services-actions.md) covers request-response communication and long-running tasks.

---

**Estimated completion time**: 70 minutes | **Prerequisites**: Lesson 2.1 | **Difficulty**: B1+ (Upper Intermediate)

