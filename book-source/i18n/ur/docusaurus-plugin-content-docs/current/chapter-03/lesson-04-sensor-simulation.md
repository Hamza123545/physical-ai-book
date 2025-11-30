---
title: "Lesson 3.4: Sensor Simulation in Gazebo"
description: "Simulate realistic sensor data: LiDAR, cameras, IMUs for perception algorithms"
chapter: 3
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
prerequisites: ["chapter-03-lesson-03"]
has_interactive_python: true
interactive_python_count: 1
has_try_with_ai: false
try_with_ai_count: 0
tags: ["gazebo", "sensors", "lidar", "cameras", "imu", "simulation"]
---

# Lesson 3.4: Sensor Simulation in Gazebo

## 🎯 Learning Objectives

- Simulate LiDAR sensors for 3D scanning
- Generate depth camera (RGB-D) data
- Model IMU sensors for orientation and acceleration
- Add sensor noise for realistic simulation

**Time**: 70 minutes

---

## Introduction

Gazebo can simulate realistic sensor data, enabling you to develop and test perception algorithms without real hardware. This is crucial for training computer vision and SLAM systems.

**Key Sensors**:
- **LiDAR**: 3D point clouds for mapping
- **Depth Cameras**: RGB-D data for object detection
- **IMUs**: Orientation and acceleration
- **Cameras**: Visual perception

---

## 1. LiDAR Simulation

### LiDAR Plugin

```xml
<sensor name="lidar" type="ray">
  <pose>0 0 0.2 0 0 0</pose>
  <visualize>true</visualize>
  <update_rate>40</update_rate>
  <ray>
    <scan>
      <horizontal>
        <samples>720</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
      <vertical>
        <samples>1</samples>
        <min_angle>0</min_angle>
        <max_angle>0</max_angle>
      </vertical>
    </scan>
    <range>
      <min>0.1</min>
      <max>30.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
  <plugin name="gazebo_ros_laser_controller" filename="libgazebo_ros_laser.so">
    <topicName>/scan</topicName>
    <frameName>laser_frame</frameName>
  </plugin>
</sensor>
```

---

## 2. Depth Camera (RGB-D)

### Camera Plugin

```xml
<sensor name="depth_camera" type="depth">
  <pose>0 0 0.1 0 0 0</pose>
  <visualize>true</visualize>
  <update_rate>30</update_rate>
  <camera name="camera1">
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.05</near>
      <far>3.0</far>
    </clip>
  </camera>
  <plugin name="gazebo_ros_camera" filename="libgazebo_ros_camera.so">
    <ros>
      <namespace>/camera</namespace>
    </ros>
    <camera_name>camera1</camera_name>
    <frame_name>camera_frame</frame_name>
    <hack_baseline>0.07</hack_baseline>
    <min_depth>0.05</min_depth>
    <max_depth>3.0</max_depth>
  </plugin>
</sensor>
```

---

## 3. IMU Sensor

### IMU Plugin

```xml
<sensor name="imu" type="imu">
  <pose>0 0 0 0 0 0</pose>
  <update_rate>100</update_rate>
  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.0002</stddev>
        </noise>
      </x>
      <!-- Similar for y, z -->
    </angular_velocity>
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>
        </noise>
      </x>
      <!-- Similar for y, z -->
    </linear_acceleration>
  </imu>
  <plugin name="gazebo_ros_imu" filename="libgazebo_ros_imu.so">
    <ros>
      <namespace>/imu</namespace>
    </ros>
    <topicName>/imu/data</topicName>
    <frameName>imu_frame</frameName>
  </plugin>
</sensor>
```

---

## 4. Sensor Noise

Real sensors have noise. Gazebo allows you to model this:

```xml
<noise type="gaussian">
  <mean>0.0</mean>
  <stddev>0.01</stddev>
  <bias_mean>0.0</bias_mean>
  <bias_stddev>0.0</bias_stddev>
</noise>
```

---

## 5. Exercises

### Exercise 3.4.1: Process LiDAR Data

Simulate processing LiDAR scan data to detect obstacles.

<InteractivePython
  id="ex-3-4-1"
  title="LiDAR Obstacle Detection"
  starterCode={`import numpy as np

def detect_obstacles(ranges, min_range=0.1, max_range=30.0, threshold=1.0):
    """
    Detect obstacles from LiDAR ranges.
    
    Args:
        ranges: Array of distance measurements
        min_range: Minimum valid range
        max_range: Maximum valid range
        threshold: Distance threshold for obstacle
    
    Returns:
        List of obstacle angles (indices)
    """
    # TODO: Filter valid ranges, find obstacles closer than threshold
    pass

# Test with sample data
ranges = np.array([2.5, 1.8, 0.5, 0.3, 0.8, 2.1, 3.0])
obstacles = detect_obstacles(ranges, threshold=1.0)
print(f"Obstacles detected at angles: {obstacles}")
`}
  hints={[
    "Filter ranges: valid = (ranges >= min_range) & (ranges <= max_range)",
    "Find obstacles: obstacles = np.where(valid_ranges < threshold)[0]",
    "Return list of indices where obstacles are detected"
  ]}
/>

---

## Summary

**Key Takeaways**:
1. Gazebo simulates realistic sensor data
2. LiDAR provides 3D point clouds for mapping
3. Depth cameras give RGB-D data for perception
4. IMUs measure orientation and acceleration
5. Sensor noise should be modeled for realism

**What's Next**: [Lesson 3.5: Unity for High-Fidelity Rendering](./lesson-05-unity-rendering.md) explores photorealistic visualization.

---

**Estimated completion time**: 70 minutes | **Prerequisites**: Lesson 3.3 | **Difficulty**: B2 (Advanced)

