---
title: "Lesson 2.5: URDF for Humanoids and Launch Files"
description: "Model humanoid robots using URDF and create launch files for complex systems"
chapter: 2
lesson: 5
estimated_time: 60
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
prerequisites: ["chapter-02-lesson-04"]
has_interactive_python: true
interactive_python_count: 3
has_try_with_ai: true
try_with_ai_count: 1
tags: ["ros2", "urdf", "launch-files", "humanoids", "robot-modeling"]
---

# Lesson 2.5: URDF for Humanoids and Launch Files

## 🎯 Learning Objectives

- Understand URDF (Unified Robot Description Format) syntax
- Model humanoid robots with links and joints
- Create launch files for complex robot systems
- Manage parameters and configurations
- Integrate URDF with ROS 2 nodes

**Time**: 60 minutes

---

## Prerequisites

<Prerequisites
  prerequisites={[
    {
      lessonId: "chapter-02-lesson-04",
      title: "Lesson 2.4: Building ROS 2 Packages with Python (rclpy)",
      link: "/docs/chapter-02/lesson-04-rclpy-packages"
    }
  ]}
/>

## Introduction

**URDF (Unified Robot Description Format)** is an XML format for describing robot structure. Combined with **launch files**, you can model complex humanoid robots and start entire robot systems with a single command.

**Key Concepts**:
- Robot links (rigid bodies)
- Joints (connections between links)
- Humanoid modeling
- Launch file orchestration

---

## 1. URDF Basics

### Simple URDF Structure

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  
  <!-- Base Link (Torso) -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <origin xyz="0 0 0.25"/>
      <inertia ixx="0.5" ixy="0" ixz="0" 
               iyy="0.3" iyz="0" izz="0.5"/>
    </inertial>
  </link>
  
  <!-- Head Joint -->
  <joint name="head_joint" type="revolute">
    <parent link="base_link"/>
    <child link="head_link"/>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" 
           effort="10" velocity="1"/>
  </joint>
  
  <!-- Head Link -->
  <link name="head_link">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </visual>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0"
               iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  
</robot>
```

### Key Elements

- **`<robot>`**: Root element, defines robot name
- **`<link>`**: Rigid body (torso, head, arm, etc.)
- **`<joint>`**: Connection between links
- **`<visual>`**: Appearance for visualization
- **`<collision>`**: Collision geometry (can differ from visual)
- **`<inertial>`**: Mass and inertia properties

---

## 2. Humanoid Robot Modeling

### Humanoid Structure

Typical humanoid has:
- **Torso**: Central body
- **Head**: Vision and sensing
- **Arms**: Shoulder, elbow, wrist joints
- **Legs**: Hip, knee, ankle joints
- **Hands/Feet**: End effectors

### Example: Humanoid Arm

```xml
<!-- Left Shoulder Joint -->
<joint name="left_shoulder_pan" type="revolute">
  <parent link="torso"/>
  <child link="left_upper_arm"/>
  <origin xyz="0.15 0.1 0.2" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-3.14" upper="3.14" effort="50" velocity="2"/>
</joint>

<!-- Left Upper Arm -->
<link name="left_upper_arm">
  <visual>
    <geometry>
      <cylinder radius="0.03" length="0.25"/>
    </geometry>
  </visual>
  <inertial>
    <mass value="1.5"/>
    <origin xyz="0 0 0.125"/>
    <inertia ixx="0.01" ixy="0" ixz="0"
             iyy="0.01" iyz="0" izz="0.01"/>
  </inertial>
</link>

<!-- Left Elbow Joint -->
<joint name="left_elbow" type="revolute">
  <parent link="left_upper_arm"/>
  <child link="left_forearm"/>
  <origin xyz="0 0 0.25" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="0" upper="3.14" effort="30" velocity="2"/>
</joint>
```

---

## 3. Joint Types

### Common Joint Types

1. **Fixed**: No movement (welded connection)
2. **Revolute**: Rotation around axis (shoulder, elbow)
3. **Prismatic**: Linear movement (sliding)
4. **Continuous**: Unlimited rotation (wheels)

### Joint Limits

```xml
<limit 
    lower="-1.57"    <!-- Minimum angle (radians) -->
    upper="1.57"     <!-- Maximum angle (radians) -->
    effort="10"       <!-- Maximum torque (N⋅m) -->
    velocity="1"     <!-- Maximum velocity (rad/s) -->
/>
```

---

## 4. Launch Files

### Basic Launch File

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        
        # Robot state publisher (publishes URDF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': get_robot_description(),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),
        
        # Your custom nodes
        Node(
            package='my_robot_package',
            executable='sensor_node',
            name='sensor_node'
        ),
        Node(
            package='my_robot_package',
            executable='control_node',
            name='control_node'
        ),
    ])

def get_robot_description():
    # Load URDF file
    with open('path/to/robot.urdf', 'r') as f:
        return f.read()
```

---

## 5. Parameter Management

### Using Parameters

```python
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

DeclareLaunchArgument(
    'robot_name',
    default_value='humanoid_robot',
    description='Name of the robot'
),

Node(
    package='my_robot_package',
    executable='control_node',
    parameters=[{
        'robot_name': LaunchConfiguration('robot_name'),
        'max_velocity': 1.0,
        'safety_enabled': True
    }]
)
```

### YAML Parameter Files

```yaml
# config/params.yaml
control_node:
  ros__parameters:
    max_velocity: 1.0
    safety_enabled: true
    joint_limits:
      shoulder: 3.14
      elbow: 3.14
```

Load in launch file:
```python
Node(
    package='my_robot_package',
    executable='control_node',
    parameters=['config/params.yaml']
)
```

---

## 6. Complete Example: Humanoid Launch

### Full Launch File

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('my_humanoid_package')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'humanoid.urdf')
    
    return LaunchDescription([
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': open(urdf_file).read()}]
        ),
        
        # Joint state publisher (for simulation/testing)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher'
        ),
        
        # Sensor nodes
        Node(
            package='my_humanoid_package',
            executable='camera_node',
            name='camera_node'
        ),
        Node(
            package='my_humanoid_package',
            executable='imu_node',
            name='imu_node'
        ),
        
        # Control nodes
        Node(
            package='my_humanoid_package',
            executable='balance_controller',
            name='balance_controller'
        ),
        Node(
            package='my_humanoid_package',
            executable='motion_planner',
            name='motion_planner'
        ),
    ])
```

---

## 7. Exercises

### Exercise 2.5.1: Create Simple URDF

Create URDF for a 2-link robot arm.

<InteractivePython
  id="ex-2-5-1"
  title="Create URDF for Robot Arm"
  starterCode={`def create_arm_urdf():
    """
    Generate URDF XML for a 2-link robot arm.
    
    Links: base_link, link1, link2
    Joints: joint1 (base to link1), joint2 (link1 to link2)
    """
    # TODO: Create URDF XML string
    urdf = """<?xml version="1.0"?>
<robot name="simple_arm">
  <!-- TODO: Add links and joints -->
</robot>"""
    return urdf

# Test
urdf = create_arm_urdf()
print(urdf)
`}
  hints={[
    "Define base_link with box geometry",
    "Add joint1 (revolute) connecting base to link1",
    "Add link1 (cylinder)",
    "Add joint2 and link2"
  ]}
/>

---

### Exercise 2.5.2: Launch File Structure

Design launch file structure for a robot system.

<InteractivePython
  id="ex-2-5-2"
  title="Launch File Design"
  starterCode={`def design_launch_file(nodes):
    """
    Design launch file structure.
    
    Args:
        nodes: List of (package, executable, name) tuples
    
    Returns:
        Dict representing launch file structure
    """
    launch_structure = {
        "robot_state_publisher": True,
        "nodes": []
    }
    # TODO: Add nodes to structure
    return launch_structure

# Test
nodes = [
    ("my_pkg", "sensor_node", "sensor"),
    ("my_pkg", "control_node", "control"),
    ("my_pkg", "planner_node", "planner")
]

structure = design_launch_file(nodes)
print("Launch structure:", structure)
`}
  hints={[
    "Include robot_state_publisher",
    "Add each node with package, executable, name",
    "Structure as list of node configs"
  ]}
/>

---

### Exercise 2.5.3: Joint Configuration

Calculate joint limits for humanoid joints.

<InteractivePython
  id="ex-2-5-3"
  title="Joint Limits Configuration"
  starterCode={`def configure_joint_limits(joint_name):
    """
    Configure joint limits for humanoid joints.
    
    Joints:
    - shoulder: -180 to +180 degrees
    - elbow: 0 to 180 degrees
    - hip: -90 to +90 degrees
    - knee: 0 to 180 degrees
    
    Returns dict with lower, upper (in radians), effort, velocity
    """
    # TODO: Return appropriate limits for each joint
    pass

# Test
joints = ["shoulder", "elbow", "hip", "knee"]
for joint in joints:
    limits = configure_joint_limits(joint)
    print(f"{joint}: {limits}")
`}
  hints={[
    "Convert degrees to radians (π radians = 180 degrees)",
    "Shoulder: -π to π",
    "Elbow: 0 to π",
    "Set reasonable effort and velocity values"
  ]}
/>

---

## 8. Try With AI

### TryWithAI 2.5.1: Design Humanoid URDF

<TryWithAI
  id="tryai-2-5-1"
  title="Design Humanoid Robot URDF"
  role="Copilot"
  scenario="You need to create a URDF for a humanoid robot"
  yourTask="Describe the structure of your humanoid (what links and joints you need) and ask for help designing the URDF. Include torso, head, arms, and legs."
  aiPromptTemplate="I'm creating a URDF for a humanoid robot with [your structure]. Can you help me design the URDF? I need help with link geometries, joint configurations, and proper hierarchy. Also suggest appropriate inertial properties."
  successCriteria={[
    "You understand URDF structure",
    "You know how to model links and joints",
    "You understand humanoid kinematics"
  ]}
  reflectionQuestions={[
    "What's the parent-child relationship in humanoid joints?",
    "What joint types are appropriate for each connection?",
    "How do you set realistic joint limits?"
  ]}
/>

---

## Summary

**Key Takeaways**:

1. **URDF Structure**
   - Links define rigid bodies
   - Joints connect links
   - Visual, collision, and inertial properties
   - Tree structure (one root link)

2. **Humanoid Modeling**
   - Torso as base
   - Head, arms, legs as branches
   - Appropriate joint types and limits
   - Realistic inertial properties

3. **Launch Files**
   - Orchestrate multiple nodes
   - Load URDF and parameters
   - Manage robot state publisher
   - Enable complex system startup

4. **Best Practices**
   - Separate visual and collision geometries
   - Use realistic inertial properties
   - Organize launch files modularly
   - Use parameters for configuration

**What's Next**: You've completed Module 1! Next, [Chapter 3: The Digital Twin (Gazebo & Unity)](./../chapter-03/index.md) teaches you to simulate robots.

---

**Estimated completion time**: 60 minutes | **Prerequisites**: Lesson 2.4 | **Difficulty**: B2 (Advanced)

