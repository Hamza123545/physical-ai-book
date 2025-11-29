---
title: "Lesson 2.4: Building ROS 2 Packages with Python (rclpy)"
description: "Create complete ROS 2 packages with proper structure and Python agents"
chapter: 2
lesson: 4
estimated_time: 70
cefr_level: "B2"
blooms_level: "Apply"
digcomp_level: 5
generated_by: "claude-sonnet-4-5-20250929"
source_spec: "specs/002-physical-ai-textbook/spec.md"
created: "2025-11-29"
last_modified: "2025-11-29"
git_author: "hswat"
workflow: "/sp.implement"
version: "1.0"
prerequisites: ["chapter-02-lesson-03"]
has_interactive_python: true
interactive_python_count: 4
has_try_with_ai: true
try_with_ai_count: 2
tags: ["ros2", "packages", "rclpy", "python-agents", "setup.py"]
---

# Lesson 2.4: Building ROS 2 Packages with Python (rclpy)

## 🎯 Learning Objectives

- Create ROS 2 package structure
- Set up package dependencies
- Build Python agents that integrate with ROS 2
- Organize code into proper ROS 2 packages
- Understand package.xml and setup.py

**Time**: 70 minutes

---

## Prerequisites

<Prerequisites
  prerequisites={[
    {
      lessonId: "chapter-02-lesson-03",
      title: "Lesson 2.3: ROS 2 Services and Actions",
      link: "/docs/chapter-02/lesson-03-services-actions"
    }
  ]}
/>

## Introduction

A **ROS 2 package** is the fundamental unit for organizing and distributing ROS 2 code. Packages contain nodes, launch files, configuration, and dependencies. This lesson teaches you to create professional ROS 2 packages with proper structure.

**Key Concepts**:
- Package structure and organization
- Dependencies management
- Python package setup
- Building and installing packages

---

## 1. ROS 2 Package Structure

### Standard Package Layout

```
my_robot_package/
├── package.xml          # Package metadata and dependencies
├── setup.py             # Python package setup
├── setup.cfg             # Package configuration
├── CMakeLists.txt        # Build configuration (for C++ or mixed)
├── my_robot_package/    # Python package directory
│   ├── __init__.py
│   └── nodes/           # Node scripts
│       ├── __init__.py
│       ├── sensor_node.py
│       └── control_node.py
└── launch/              # Launch files
    └── robot.launch.py
```

---

## 2. Creating a ROS 2 Package

### Using ros2 pkg create

```bash
# Create Python package
ros2 pkg create --build-type ament_python my_robot_package

# Create with dependencies
ros2 pkg create --build-type ament_python \
    --dependencies rclpy std_msgs geometry_msgs \
    my_robot_package
```

### Package Structure

```bash
cd my_robot_package
mkdir -p my_robot_package/nodes
mkdir -p launch
touch my_robot_package/__init__.py
touch my_robot_package/nodes/__init__.py
```

---

## 3. package.xml

### Package Metadata

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_package</name>
  <version>1.0.0</version>
  <description>My robot control package</description>
  <maintainer email="developer@example.com">Developer Name</maintainer>
  <license>MIT</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### Key Elements

- **name**: Package name
- **version**: Package version
- **depend**: Runtime dependencies
- **test_depend**: Testing dependencies

---

## 4. setup.py

### Python Package Setup

```python
from setuptools import setup
import os
from glob import glob

package_name = 'my_robot_package'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name, f'{package_name}.nodes'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Developer Name',
    maintainer_email='developer@example.com',
    description='My robot control package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_node = my_robot_package.nodes.sensor_node:main',
            'control_node = my_robot_package.nodes.control_node:main',
        ],
    },
)
```

### Entry Points

Entry points make nodes executable:
```python
entry_points={
    'console_scripts': [
        'sensor_node = my_robot_package.nodes.sensor_node:main',
    ],
}
```

This allows running: `ros2 run my_robot_package sensor_node`

---

## 5. Python Agent Integration

### Bridging Python Agents to ROS 2

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PythonAgentNode(Node):
    """Bridge between Python AI agent and ROS 2."""
    
    def __init__(self):
        super().__init__('python_agent_node')
        
        # ROS 2 publishers/subscribers
        self.command_pub = self.create_publisher(
            String, '/robot_commands', 10
        )
        self.sensor_sub = self.create_subscription(
            String, '/sensor_data',
            self.sensor_callback, 10
        )
        
        # Python AI agent (your custom code)
        self.ai_agent = MyAIAgent()
        
        self.get_logger().info('Python agent node started')
    
    def sensor_callback(self, msg):
        """Process sensor data with AI agent."""
        # Get sensor data
        sensor_data = msg.data
        
        # Process with AI agent
        decision = self.ai_agent.process(sensor_data)
        
        # Publish command
        cmd_msg = String()
        cmd_msg.data = decision
        self.command_pub.publish(cmd_msg)

class MyAIAgent:
    """Example Python AI agent."""
    
    def process(self, data):
        # Your AI logic here
        if "obstacle" in data.lower():
            return "stop"
        return "continue"

def main(args=None):
    rclpy.init(args=args)
    node = PythonAgentNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## 6. Building and Installing

### Build Package

```bash
# From workspace root
cd ~/ros2_ws
colcon build --packages-select my_robot_package
```

### Source Workspace

```bash
source install/setup.bash
```

### Run Node

```bash
ros2 run my_robot_package sensor_node
```

---

## 7. Launch Files

### Creating Launch Files

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_package',
            executable='sensor_node',
            name='sensor_node',
            output='screen'
        ),
        Node(
            package='my_robot_package',
            executable='control_node',
            name='control_node',
            output='screen'
        ),
    ])
```

### Running Launch Files

```bash
ros2 launch my_robot_package robot.launch.py
```

---

## 8. Exercises

### Exercise 2.4.1: Package Structure

Design the structure for a robot package with multiple nodes.

<InteractivePython
  id="ex-2-4-1"
  title="Package Structure Design"
  starterCode={`def design_package_structure(package_name):
    """
    Design ROS 2 package structure.
    
    Package should have:
    - Sensor node
    - Control node
    - Planning node
    - Launch file
    
    Return dict with structure.
    """
    structure = {
        "package_name": package_name,
        "nodes": [],
        "launch_files": [],
        "dependencies": []
    }
    # TODO: Fill in structure
    return structure

# Test
structure = design_package_structure("my_robot")
print("Package structure:", structure)
`}
  hints={[
    "Nodes: sensor_node, control_node, planning_node",
    "Launch file: robot.launch.py",
    "Dependencies: rclpy, std_msgs, geometry_msgs"
  ]}
/>

---

### Exercise 2.4.2: Entry Points Configuration

Create entry points for multiple nodes.

<InteractivePython
  id="ex-2-4-2"
  title="Entry Points Configuration"
  starterCode={`def create_entry_points(nodes):
    """
    Create entry points configuration for nodes.
    
    Args:
        nodes: List of (node_name, module_path) tuples
    
    Returns:
        Dict with entry_points structure
    """
    # TODO: Create entry_points dict
    # Format: 'node_name = package.module:main'
    pass

# Test
nodes = [
    ("sensor_node", "my_robot.nodes.sensor_node"),
    ("control_node", "my_robot.nodes.control_node"),
    ("planner_node", "my_robot.nodes.planner_node")
]

entry_points = create_entry_points(nodes)
print("Entry points:", entry_points)
`}
  hints={[
    "Entry points format: 'executable_name = module.path:function'",
    "Use 'console_scripts' key",
    "Each node needs executable name and module path"
  ]}
/>

---

### Exercise 2.4.3: Dependency Management

Identify dependencies for a robot package.

<InteractivePython
  id="ex-2-4-3"
  title="Package Dependencies"
  starterCode={`def identify_dependencies(features):
    """
    Identify ROS 2 dependencies based on features.
    
    Features:
    - "camera": Needs sensor_msgs
    - "navigation": Needs nav_msgs, geometry_msgs
    - "manipulation": Needs moveit_msgs
    - "ai_planning": Needs rclpy, std_msgs
    """
    dependencies = []
    # TODO: Map features to dependencies
    return dependencies

# Test
features = ["camera", "navigation", "ai_planning"]
deps = identify_dependencies(features)
print("Dependencies:", deps)
`}
  hints={[
    "Camera → sensor_msgs",
    "Navigation → nav_msgs, geometry_msgs",
    "AI planning → rclpy, std_msgs",
    "All need rclpy"
  ]}
/>

---

### Exercise 2.4.4: Python Agent Bridge

Create a bridge between Python agent and ROS 2.

<InteractivePython
  id="ex-2-4-4"
  title="Python Agent to ROS 2 Bridge"
  starterCode={`class AgentBridge:
    """Bridge between Python agent and ROS 2."""
    
    def __init__(self):
        self.received_messages = []
        self.published_commands = []
    
    def receive_sensor_data(self, sensor_value):
        """Receive sensor data (simulating ROS 2 subscriber)."""
        # TODO: Store sensor data
        pass
    
    def process_with_agent(self, sensor_value):
        """Process with AI agent logic."""
        # Simple agent: if value > 5, return "stop", else "go"
        # TODO: Implement agent logic
        pass
    
    def publish_command(self, command):
        """Publish command (simulating ROS 2 publisher)."""
        # TODO: Store published command
        pass
    
    def process_pipeline(self, sensor_readings):
        """Complete pipeline: receive → process → publish."""
        for reading in sensor_readings:
            self.receive_sensor_data(reading)
            command = self.process_with_agent(reading)
            self.publish_command(command)

# Test
bridge = AgentBridge()
readings = [3.0, 6.5, 2.1, 7.8, 1.5]
bridge.process_pipeline(readings)

print("Received:", bridge.received_messages)
print("Published:", bridge.published_commands)
`}
  hints={[
    "Store sensor values in received_messages",
    "Agent logic: value > 5 → 'stop', else → 'go'",
    "Store commands in published_commands"
  ]}
/>

---

## 9. Try With AI

### TryWithAI 2.4.1: Package Design Review

<TryWithAI
  id="tryai-2-4-1"
  title="Review Package Structure"
  role="Evaluator"
  scenario="You've created a ROS 2 package but want to ensure it follows best practices"
  yourTask="Describe your package structure: what nodes you have, how you organized files, and what dependencies you included. Ask for review."
  aiPromptTemplate="I created a ROS 2 package with this structure: [your description]. Can you review it for best practices? Check package organization, dependency management, and suggest improvements."
  successCriteria={[
    "You understand proper package structure",
    "You know how to organize nodes",
    "You understand dependency management"
  ]}
  reflectionQuestions={[
    "What's the purpose of setup.py?",
    "How do entry points work?",
    "What dependencies does your package need?"
  ]}
/>

---

### TryWithAI 2.4.2: Integrate Custom Python Code

<TryWithAI
  id="tryai-2-4-2"
  title="Integrate Python Agent with ROS 2"
  role="Copilot"
  scenario="You have a Python AI agent and want to integrate it with ROS 2"
  yourTask="Describe your Python agent (what it does, what inputs/outputs it has) and ask how to integrate it with ROS 2 nodes."
  aiPromptTemplate="I have a Python AI agent that [description]. It takes [inputs] and returns [outputs]. How should I integrate this with ROS 2? Should I create a node that wraps it, or integrate it directly into existing nodes?"
  successCriteria={[
    "You understand how to bridge Python code to ROS 2",
    "You know how to structure the integration",
    "You understand node architecture"
  ]}
  reflectionQuestions={[
    "Where should your agent logic live?",
    "How do you handle ROS 2 callbacks with agent processing?",
    "What's the best way to structure this?"
  ]}
/>

---

## Summary

**Key Takeaways**:

1. **Package Structure**
   - Standard layout: nodes/, launch/, package.xml, setup.py
   - Organize code into logical modules
   - Use proper Python package structure

2. **Dependencies**
   - Declare in package.xml
   - Include runtime and test dependencies
   - Use appropriate message types

3. **Entry Points**
   - Define in setup.py
   - Make nodes executable via `ros2 run`
   - Format: `executable_name = package.module:function`

4. **Python Agent Integration**
   - Create ROS 2 nodes that wrap Python agents
   - Use publishers/subscribers for communication
   - Bridge between AI code and robot hardware

**What's Next**: [Lesson 2.5: URDF for Humanoids and Launch Files](./lesson-05-urdf-launch.md) teaches you to model humanoid robots and create launch files.

---

**Estimated completion time**: 70 minutes | **Prerequisites**: Lesson 2.3 | **Difficulty**: B2 (Advanced)

