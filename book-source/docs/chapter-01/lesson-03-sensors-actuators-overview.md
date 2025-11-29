---
title: "Lesson 1.3: Sensors and Actuators Overview"
description: "Learn about the sensors robots use to perceive the world and actuators they use to act"
chapter: 1
lesson: 3
estimated_time: 45
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
prerequisites:
  - "chapter-01-lesson-01"
  - "chapter-01-lesson-02"
has_interactive_python: true
interactive_python_count: 3
has_try_with_ai: true
try_with_ai_count: 2
tags: ["sensors", "actuators", "encoders", "imu", "motors"]
---

# Lesson 1.3: Sensors and Actuators Overview

## 🎯 Learning Objectives

<LearningObjectives
  cefr_level="B1"
  objectives={[
    {
      text: "Identify common sensors used in robotics and explain their purposes",
      blooms_level: "Understand",
      assessment_method: "Quiz and Try With AI"
    },
    {
      text: "Identify common actuators and their characteristics",
      blooms_level: "Understand",
      assessment_method: "Quiz"
    },
    {
      text: "Implement sensor data processing for encoders, IMUs, and servos",
      blooms_level: "Apply",
      assessment_method: "Interactive Python exercises"
    }
  ]}
/>

## 📚 Prerequisites

<Prerequisites
  prerequisites={[
    {
      lessonId: "chapter-01-lesson-01",
      title: "Lesson 1.1: What is Physical AI?",
      link: "/docs/chapter-01/lesson-01-what-is-physical-ai"
    },
    {
      lessonId: "chapter-01-lesson-02",
      title: "Lesson 1.2: Robot vs Software AI",
      link: "/docs/chapter-01/lesson-02-robot-vs-software-ai"
    }
  ]}
/>

## Introduction

Robots interact with the physical world through two types of hardware:

- **Sensors**: How robots **perceive** (eyes, ears, touch)
- **Actuators**: How robots **act** (muscles, movements)

Understanding sensors and actuators is fundamental to Physical AI. In this lesson, you'll learn about the most common types and how to process their data.

**Time**: 45 minutes

---

## 1. Sensors: Robot Perception

Sensors convert physical quantities into electrical signals that computers can process.

### 1.1 Position & Motion Sensors

#### Encoders
**What**: Measure rotation angle or position
**How**: Count ticks as a wheel/motor rotates
**Used for**: Wheel odometry, joint angles in robot arms
**Example**: 360 ticks per revolution → 1 tick = 1 degree

#### Inertial Measurement Unit (IMU)
**What**: Measures acceleration and rotation
**Components**:
- **Accelerometer**: Linear acceleration (3 axes: x, y, z)
- **Gyroscope**: Angular velocity (rotation rate)
- **Magnetometer** (optional): Magnetic field (compass)

**Used for**: Balance, orientation, detecting falls
**Example**: Smartphone knows if you're holding it portrait or landscape

### 1.2 Distance & Proximity Sensors

#### Ultrasonic Sensors
**What**: Measure distance using sound waves
**Range**: 2cm - 4m
**Used for**: Obstacle avoidance, parking assistance
**Limitation**: Struggles with soft/angled surfaces

#### LIDAR (Light Detection and Ranging)
**What**: Laser-based distance measurement
**Range**: 0.1m - 100m+
**Used for**: 3D mapping, autonomous vehicles
**Advantage**: Precise, works in any lighting

#### Infrared (IR) Sensors
**What**: Measure distance using infrared light
**Range**: 10cm - 80cm
**Used for**: Cliff detection, proximity sensing
**Limitation**: Affected by surface color/material

### 1.3 Vision Sensors

#### Cameras
**What**: Capture images (RGB or grayscale)
**Used for**: Object detection, navigation, human interaction
**Challenges**: Requires significant processing, lighting-dependent

#### Depth Cameras (RGB-D)
**What**: Camera + depth sensor
**Used for**: 3D object recognition, gesture control
**Examples**: Microsoft Kinect, Intel RealSense

### 1.4 Force & Touch Sensors

#### Force/Torque Sensors
**What**: Measure applied forces
**Used for**: Delicate manipulation, assembly tasks
**Example**: Robot hand knows how hard it's gripping

#### Tactile Sensors
**What**: Detect contact and pressure
**Used for**: Object grasping, collision detection
**Example**: Robot skin that feels touch

---

## 2. Actuators: Robot Action

Actuators convert electrical energy into motion.

### 2.1 Motors

#### DC Motors
**What**: Continuous rotation motors
**Control**: Voltage controls speed
**Used for**: Wheels, fans, pumps
**Pros**: Simple, cheap
**Cons**: Hard to control position precisely

#### Servo Motors
**What**: Motors with position feedback
**Control**: Command target angle
**Range**: Usually 0-180° or 0-360°
**Used for**: Robot arms, camera gimbals
**Pros**: Precise position control

#### Stepper Motors
**What**: Move in discrete steps
**Control**: Pulse commands for each step
**Used for**: 3D printers, CNC machines
**Pros**: Precise, no feedback needed
**Cons**: Lower torque at high speed

### 2.2 Other Actuators

#### Pneumatic/Hydraulic Actuators
**What**: Use air or fluid pressure
**Used for**: Heavy lifting, industrial robots
**Pros**: Very high force
**Cons**: Complex, needs compressor/pump

#### Linear Actuators
**What**: Create straight-line motion
**Used for**: Extending/retracting mechanisms
**Example**: Electric window openers

---

## 3. Sensor-Actuator Feedback Loops

Robots use **feedback control**:

1. **Sense**: Read sensor data
2. **Think**: Compute desired action
3. **Act**: Command actuator
4. **Repeat**: Continuously update

**Example - Line Following Robot**:
1. **Sense**: IR sensor detects line position
2. **Think**: Calculate steering correction
3. **Act**: Adjust motor speeds
4. **Repeat**: 100 times per second

---

## 4. Exercises

### Exercise 1.3.1: Encoder Tick to Angle Conversion

Convert encoder ticks to rotation angles.

<InteractivePython
  id="ex-1-3-1"
  title="Encoder Tick to Angle Conversion"
  starterCode={`import numpy as np

def ticks_to_angle(ticks, ticks_per_revolution=360):
    """
    Convert encoder ticks to angle in degrees.

    Args:
        ticks: Number of encoder ticks
        ticks_per_revolution: Encoder resolution

    Returns:
        Angle in degrees (0-360)
    """
    # TODO: Calculate angle from ticks
    # Hint: angle = (ticks / ticks_per_revolution) * 360
    # Use modulo (%) to keep angle in 0-360 range
    pass

# Test
print(f"90 ticks = {ticks_to_angle(90, 360):.1f}°")
print(f"180 ticks = {ticks_to_angle(180, 360):.1f}°")
print(f"450 ticks = {ticks_to_angle(450, 360):.1f}°")  # Should wrap around

# Wheel distance traveled
def ticks_to_distance(ticks, wheel_diameter=0.1, ticks_per_rev=360):
    """
    Calculate distance traveled from encoder ticks.

    Args:
        ticks: Encoder ticks
        wheel_diameter: Wheel diameter in meters
        ticks_per_rev: Encoder resolution

    Returns:
        Distance in meters
    """
    # TODO: Calculate distance
    # Hint: circumference = π * diameter
    # distance = (ticks / ticks_per_rev) * circumference
    pass

print(f"\\n360 ticks = {ticks_to_distance(360, 0.1, 360):.3f} m")
`}
  hints={[
    "Angle = (ticks % ticks_per_revolution) * 360 / ticks_per_revolution",
    "Circumference = np.pi * diameter",
    "Distance = revolutions * circumference"
  ]}
/>

---

### Exercise 1.3.2: IMU Tilt Calculation

Calculate tilt angle from accelerometer data.

<InteractivePython
  id="ex-1-3-2"
  title="IMU Tilt Angle from Accelerometer"
  starterCode={`import numpy as np

def calculate_tilt(accel_x, accel_y, accel_z):
    """
    Calculate tilt angles (roll, pitch) from accelerometer.

    Args:
        accel_x, accel_y, accel_z: Acceleration in g's

    Returns:
        (roll, pitch) in degrees
    """
    # TODO: Calculate roll and pitch
    # Roll = arctan2(accel_y, accel_z)
    # Pitch = arctan2(-accel_x, sqrt(accel_y^2 + accel_z^2))
    # Convert radians to degrees
    pass

# Test cases
print("Robot level:")
roll, pitch = calculate_tilt(0, 0, 1.0)  # Upright (gravity only in z)
print(f"  Roll: {roll:.1f}°, Pitch: {pitch:.1f}°")

print("\\nTilted 45° forward:")
roll, pitch = calculate_tilt(0.707, 0, 0.707)
print(f"  Roll: {roll:.1f}°, Pitch: {pitch:.1f}°")

print("\\nTilted 90° to the side:")
roll, pitch = calculate_tilt(0, 1.0, 0)
print(f"  Roll: {roll:.1f}°, Pitch: {pitch:.1f}°")
`}
  hints={[
    "Use np.arctan2(y, x) for arctangent",
    "Convert radians to degrees: angle_deg = angle_rad * 180 / np.pi",
    "np.sqrt(x**2 + y**2) for magnitude"
  ]}
/>

---

### Exercise 1.3.3: Servo PWM Control

Calculate PWM (Pulse Width Modulation) signal for servo control.

<InteractivePython
  id="ex-1-3-3"
  title="Servo PWM Signal Generation"
  starterCode={`def angle_to_pwm(angle, min_pulse=1000, max_pulse=2000, min_angle=0, max_angle=180):
    """
    Convert desired servo angle to PWM pulse width.

    Args:
        angle: Desired angle (degrees)
        min_pulse: PWM pulse for min_angle (microseconds)
        max_pulse: PWM pulse for max_angle (microseconds)
        min_angle: Minimum servo angle
        max_angle: Maximum servo angle

    Returns:
        PWM pulse width in microseconds
    """
    # TODO: Linear interpolation from angle to pulse width
    # Formula: pulse = min_pulse + (angle - min_angle) * (max_pulse - min_pulse) / (max_angle - min_angle)
    pass

# Test
print(f"0° = {angle_to_pwm(0)} μs")
print(f"90° = {angle_to_pwm(90)} μs")
print(f"180° = {angle_to_pwm(180)} μs")

# Safety check
def safe_servo_command(angle, prev_angle=None, max_change=30):
    """
    Validate servo command is safe.

    Returns:
        (is_safe, clamped_angle)
    """
    # TODO:
    # 1. Clamp angle to 0-180 range
    # 2. If prev_angle exists, limit change to max_change
    # 3. Return (True/False, final_angle)
    pass

print("\\nSafety tests:")
safe, cmd = safe_servo_command(200)
print(f"Command 200° -> Safe: {safe}, Clamped: {cmd}°")

safe, cmd = safe_servo_command(120, prev_angle=50, max_change=30)
print(f"Jump 50°->120° -> Safe: {safe}, Limited: {cmd}°")
`}
  hints={[
    "Linear interpolation: out = out_min + (in - in_min) * (out_max - out_min) / (in_max - in_min)",
    "Clamp: np.clip(angle, 0, 180)",
    "Limit change: if abs(angle - prev_angle) > max_change: use prev_angle + max_change"
  ]}
/>

---

## 5. Try With AI

### TryWithAI 1.3.1: Sensor Selection

<TryWithAI
  id="tryai-1-3-1"
  title="Choose Sensors for a Robot Project"
  role="Teacher"
  scenario="You're building a small indoor delivery robot for an office building. It needs to navigate hallways, avoid obstacles, and find charging stations."
  yourTask="List which sensors you think this robot needs and why. Consider navigation, safety, and battery management."
  aiPromptTemplate="I'm designing an indoor delivery robot. Here are the sensors I chose: [your list with reasons]. Can you explain if these are good choices and suggest any sensors I'm missing? Also explain trade-offs (cost, complexity, performance)."
  successCriteria={[
    "You selected appropriate sensors for navigation and obstacle avoidance",
    "You considered both cost and capability",
    "You understand trade-offs between different sensor types"
  ]}
  reflectionQuestions={[
    "Why might you use multiple sensors for the same task (e.g., both ultrasonic and LIDAR)?",
    "What's the minimum sensor set to make this robot functional?",
    "How would sensor choices change for an outdoor robot?"
  ]}
/>

---

### TryWithAI 1.3.2: IMU Calibration

<TryWithAI
  id="tryai-1-3-2"
  title="IMU Calibration Procedure"
  role="Copilot"
  scenario="Your robot's IMU is giving incorrect tilt readings. You suspect it needs calibration."
  yourTask="Research how IMU calibration works. Try implementing Exercise 1.3.2, then think about what could cause errors."
  aiPromptTemplate="I'm trying to calibrate an IMU for my robot. Here's what I know about the problem: [describe the issue]. Can you walk me through a calibration procedure step-by-step? Also explain what could cause systematic errors in tilt calculations."
  successCriteria={[
    "You understand the concept of sensor calibration",
    "You can identify sources of IMU error (bias, drift, noise)",
    "You know how to collect calibration data"
  ]}
  reflectionQuestions={[
    "Why do sensors need calibration?",
    "How often should you recalibrate?",
    "What happens if you use an uncalibrated sensor?"
  ]}
/>

---

## Summary

**Key Takeaways**:

1. **Sensors** (Perception):
   - **Position**: Encoders, IMU (accelerometer, gyroscope)
   - **Distance**: Ultrasonic, LIDAR, IR
   - **Vision**: Cameras, depth sensors
   - **Touch**: Force sensors, tactile sensors

2. **Actuators** (Action):
   - **Motors**: DC (speed), Servo (position), Stepper (precision)
   - **Other**: Pneumatic, hydraulic, linear actuators

3. **Feedback Control Loop**:
   - Sense → Think → Act → Repeat
   - Continuous updating at high frequency

4. **Practical Skills**:
   - Converting encoder ticks to angles/distances
   - Calculating tilt from IMU data
   - Controlling servos with PWM signals

**What's Next**: [Lesson 1.4: Python for Robotics Introduction](./lesson-04-python-robotics-intro.md) teaches you to use NumPy and Matplotlib for robot simulation and visualization.

---

**Estimated completion time**: 45 minutes | **Prerequisites**: Lessons 1.1, 1.2 | **Difficulty**: B1 (Intermediate)
