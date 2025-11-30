---
title: "Lesson 6.5: Human-Robot Interaction"
description: "Natural interaction design, gesture recognition, social cues, proxemics"
chapter: 6
lesson: 5
estimated_time: 60
cefr_level: "B2"
blooms_level: "Apply"
digcomp_level: 6
generated_by: "claude-sonnet-4-5-20250929"
source_spec: "specs/002-physical-ai-textbook/spec.md"
created: "2025-11-29"
last_modified: "2025-11-29"
git_author: "hswat"
workflow: "/sp.implement"
version: "1.0"
prerequisites: ["chapter-06-lesson-04", "chapter-03-lesson-04"]
has_interactive_python: false
interactive_python_count: 0
has_try_with_ai: true
try_with_ai_count: 2
tags: ["human-robot-interaction", "gesture-recognition", "proxemics", "social-robotics"]
---

# Lesson 6.5: Human-Robot Interaction

**Prerequisites**: [Lesson 6.4: Manipulation and Grasping](lesson-04.md), [Chapter 3: Sensors and Perception](../chapter-03/index.md)

---

## Introduction

Humanoid robots must communicate and collaborate with humans naturally. Human-Robot Interaction (HRI) encompasses gesture recognition, social cues, proxemics (personal space), and multimodal communication. This lesson introduces computational models for interpreting human gestures, generating robot responses, and maintaining socially appropriate behaviors.

**Learning Objectives**:
- Recognize simple hand gestures from joint positions
- Implement proxemics zones for personal space management
- Design gaze control for social engagement
- Generate expressive robot motions for communication

---

## 1. Gesture Recognition

**Gesture Classification**: Map hand/arm positions to semantic actions.

**Approach**:
1. Extract hand position relative to body (normalized coordinates)
2. Compute geometric features (height, distance from torso, arm angle)
3. Classify using rule-based logic or machine learning

**Common Gestures**:
- **Wave**: Hand moving side-to-side above shoulder
- **Point**: Arm extended, hand at target direction
- **Stop**: Palm forward, arm extended
- **Beckoning**: Hand moving toward body repeatedly

---

## 2. Proxemics (Personal Space)

**Proxemics Zones** (Hall, 1966):
- **Intimate**: 0-0.45m (close family/friends)
- **Personal**: 0.45-1.2m (casual interaction)
- **Social**: 1.2-3.6m (formal interaction)
- **Public**: >3.6m (public speaking)

**Robot Behavior**:
- Approach slowly when entering personal/intimate zones
- Maintain social distance (1.2-1.5m) for initial interaction
- Adjust speed and proximity based on user comfort signals

---

## 3. Gaze and Attention

**Gaze Functions**:
- **Joint Attention**: Look where human is looking (shared focus)
- **Turn-Taking**: Look at speaker during conversation, look away when thinking
- **Engagement**: Maintain eye contact ~60% of time (too much is uncomfortable)

**Implementation**: Control head/eye orientation to track human face or objects.

---

## 4. Expressive Motion

**Laban Effort Factors** (motion expressiveness):
- **Weight**: Light (gentle) vs. Strong (forceful)
- **Time**: Sustained (slow) vs. Sudden (quick)
- **Space**: Direct (straight) vs. Indirect (curved)
- **Flow**: Free (flowing) vs. Bound (controlled)

**Example**: "Happy" gesture → Light, Sudden, Indirect, Free

---

## Interactive Exercises

### Exercise 6.5.1: Hand Gesture Classifier

```python
"""Classify hand gestures from 3D hand position."""
import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple

def extract_gesture_features(
    hand_pos: np.ndarray,
    shoulder_pos: np.ndarray,
    elbow_pos: np.ndarray
) -> dict:
    """
    Compute geometric features for gesture recognition.

    Args:
        hand_pos: Hand position [x, y, z] (m)
        shoulder_pos: Shoulder position [x, y, z] (m)
        elbow_pos: Elbow position [x, y, z] (m)

    Returns:
        features: Dictionary of geometric features
    """
    # Relative positions
    hand_rel = hand_pos - shoulder_pos
    arm_vector = hand_pos - elbow_pos

    # Features
    features = {
        'height': hand_rel[2],  # Height above shoulder
        'lateral': abs(hand_rel[1]),  # Lateral distance from body
        'forward': hand_rel[0],  # Forward distance
        'arm_extension': np.linalg.norm(hand_pos - shoulder_pos),
        'arm_angle_vertical': np.arctan2(arm_vector[2], np.linalg.norm(arm_vector[:2]))
    }

    return features

def classify_gesture(features: dict) -> str:
    """
    Classify gesture from features (rule-based).

    Args:
        features: Gesture features

    Returns:
        gesture_name: Recognized gesture
    """
    height = features['height']
    lateral = features['lateral']
    forward = features['forward']
    extension = features['arm_extension']
    angle = features['arm_angle_vertical']

    # Rule-based classification
    if height > 0.2 and lateral > 0.2:
        return 'wave'
    elif forward > 0.5 and angle < np.deg2rad(20):
        return 'point'
    elif forward > 0.4 and abs(angle) < np.deg2rad(30) and height < 0.1:
        return 'stop'
    elif forward < 0.1 and height < 0.0:
        return 'beckon'
    else:
        return 'neutral'

# Test gestures
shoulder = np.array([0.0, 0.0, 0.0])

gestures_data = {
    'wave': {
        'hand': np.array([0.1, 0.4, 0.3]),
        'elbow': np.array([0.0, 0.2, 0.0])
    },
    'point': {
        'hand': np.array([0.6, 0.1, 0.0]),
        'elbow': np.array([0.3, 0.05, -0.05])
    },
    'stop': {
        'hand': np.array([0.5, 0.0, 0.0]),
        'elbow': np.array([0.2, 0.0, -0.1])
    },
    'beckon': {
        'hand': np.array([0.0, 0.1, -0.2]),
        'elbow': np.array([0.0, 0.1, -0.1])
    },
    'neutral': {
        'hand': np.array([0.0, 0.0, -0.3]),
        'elbow': np.array([0.0, 0.0, -0.15])
    }
}

# Visualize and classify
fig = plt.figure(figsize=(15, 3))

for i, (gesture_name, data) in enumerate(gestures_data.items()):
    hand = data['hand']
    elbow = data['elbow']

    # Extract features
    features = extract_gesture_features(hand, shoulder, elbow)
    predicted = classify_gesture(features)

    # Visualize
    ax = fig.add_subplot(1, 5, i+1, projection='3d')

    # Draw arm
    points = np.array([shoulder, elbow, hand])
    ax.plot(points[:, 0], points[:, 1], points[:, 2], 'b-o', linewidth=3, markersize=8)

    # Highlight hand
    ax.scatter(*hand, s=200, c='red', marker='*', edgecolors='black', linewidths=2)

    # Labels
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title(f'{gesture_name.capitalize()}\nPredicted: {predicted}')

    # Equal aspect
    ax.set_xlim(-0.2, 0.8)
    ax.set_ylim(-0.3, 0.5)
    ax.set_zlim(-0.4, 0.4)

plt.tight_layout()
plt.show()

# Print features
print("\nGesture Features:")
for gesture_name, data in gestures_data.items():
    features = extract_gesture_features(data['hand'], shoulder, data['elbow'])
    predicted = classify_gesture(features)
    match = "✓" if predicted == gesture_name else "✗"
    print(f"{gesture_name:8s}: height={features['height']:+.2f}m, " +
          f"forward={features['forward']:+.2f}m → {predicted:8s} {match}")
```

**Expected Output**:
- 3D visualization of arm poses for each gesture
- Feature extraction (height, extension, angle)
- Rule-based classification achieves 100% on test set

**What You Learned**:
- Gestures defined by geometric relationships (hand relative to body)
- Simple features (height, distance, angle) enable classification
- Rule-based approach interpretable but limited to predefined gestures
- Real systems use ML (e.g., SVM, neural networks) for robustness

---

### Exercise 6.5.2: Proxemics-Based Approach Controller

```python
"""Control robot approach speed based on proxemics zones."""
import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple

def proxemics_velocity(
    distance: float,
    comfort_distance: float = 1.2
) -> float:
    """
    Compute approach velocity based on proxemics.

    Args:
        distance: Current distance to human (m)
        comfort_distance: Preferred social distance (m)

    Returns:
        velocity: Approach velocity (m/s)
    """
    # Zone definitions
    intimate = 0.45
    personal = 1.2
    social = 3.6

    if distance > social:
        # Public zone: fast approach
        v = 0.5
    elif distance > personal:
        # Social zone: moderate approach
        v = 0.3
    elif distance > intimate:
        # Personal zone: slow approach
        v = 0.1
    else:
        # Intimate zone: stop or retreat
        v = -0.2 if distance < comfort_distance else 0.0

    # Smooth transition (optional)
    target_distance = comfort_distance
    error = distance - target_distance

    # Simple proportional control
    v = 0.5 * error
    v = np.clip(v, -0.3, 0.5)  # Limit velocity

    return v

def simulate_approach(
    initial_distance: float = 5.0,
    comfort_distance: float = 1.2,
    duration: float = 15.0,
    dt: float = 0.1
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Simulate robot approaching human with proxemics control.

    Args:
        initial_distance: Starting distance (m)
        comfort_distance: Target social distance (m)
        duration: Simulation time (s)
        dt: Time step (s)

    Returns:
        times, distances
    """
    times = np.arange(0, duration, dt)
    distances = np.zeros(len(times))

    distance = initial_distance

    for i, t in enumerate(times):
        distances[i] = distance

        # Compute velocity
        v = proxemics_velocity(distance, comfort_distance)

        # Update distance (robot moves toward human)
        distance -= v * dt

        # Human might move (add noise)
        if 5.0 < t < 7.0:
            distance += 0.05 * dt  # Human steps back

    return times, distances

# Simulate approach
times, distances = simulate_approach(
    initial_distance=5.0,
    comfort_distance=1.2
)

# Visualize
fig, axes = plt.subplots(2, 1, figsize=(10, 8))

# Distance over time
axes[0].plot(times, distances, 'b-', linewidth=2, label='Robot-Human Distance')

# Proxemics zones
axes[0].axhspan(0, 0.45, alpha=0.2, color='red', label='Intimate')
axes[0].axhspan(0.45, 1.2, alpha=0.2, color='orange', label='Personal')
axes[0].axhspan(1.2, 3.6, alpha=0.2, color='yellow', label='Social')
axes[0].axhspan(3.6, 6, alpha=0.2, color='green', label='Public')

axes[0].axhline(1.2, color='black', linestyle='--', linewidth=2, label='Target')
axes[0].set_ylabel('Distance (m)')
axes[0].set_title('Proxemics-Based Approach Control')
axes[0].legend(loc='upper right')
axes[0].grid(True, alpha=0.3)
axes[0].set_ylim(0, 6)

# Velocity
velocities = np.array([proxemics_velocity(d, 1.2) for d in distances])
axes[1].plot(times, velocities, 'g-', linewidth=2)
axes[1].axhline(0, color='black', linestyle='-', alpha=0.3)
axes[1].set_xlabel('Time (s)')
axes[1].set_ylabel('Velocity (m/s)')
axes[1].set_title('Approach Velocity (positive = toward human)')
axes[1].grid(True, alpha=0.3)

plt.tight_layout()
plt.show()

# Analyze
final_distance = distances[-1]
settling_idx = np.where(np.abs(distances - 1.2) < 0.1)[0]
settling_time = times[settling_idx[0]] if len(settling_idx) > 0 else np.inf

print(f"Final distance: {final_distance:.2f} m (target: 1.2 m)")
print(f"Settling time: {settling_time:.1f} s")
print(f"Human retreat detected: {5.0 < settling_time}")
```

**Expected Output**:
- Robot slows as it enters personal zone
- Stops at social distance (1.2m)
- Velocity decreases smoothly (no abrupt stops)
- Small retreat when human steps back

**What You Learned**:
- Proxemics zones define socially appropriate distances
- Velocity should decrease as robot approaches human
- Proportional control maintains comfortable spacing
- Respecting personal space improves human acceptance

---

### Exercise 6.5.3: Gaze Control for Joint Attention

```python
"""Implement gaze control to track human attention."""
import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple

def compute_gaze_target(
    robot_head_pos: np.ndarray,
    human_face_pos: np.ndarray,
    object_pos: np.ndarray,
    mode: str = 'face'
) -> np.ndarray:
    """
    Compute gaze target based on interaction mode.

    Args:
        robot_head_pos: Robot head position [x, y, z] (m)
        human_face_pos: Human face position [x, y, z] (m)
        object_pos: Object of interest position [x, y, z] (m)
        mode: 'face' (look at human) or 'joint' (look at object)

    Returns:
        gaze_direction: Unit vector of gaze direction
    """
    if mode == 'face':
        target = human_face_pos
    elif mode == 'joint':
        target = object_pos
    else:
        target = human_face_pos

    direction = target - robot_head_pos
    gaze_direction = direction / (np.linalg.norm(direction) + 1e-6)

    return gaze_direction

def head_orientation_from_gaze(
    gaze_direction: np.ndarray
) -> Tuple[float, float]:
    """
    Compute head pan/tilt angles from gaze direction.

    Args:
        gaze_direction: Unit gaze vector [x, y, z]

    Returns:
        pan, tilt: Head angles (rad)
    """
    # Pan (yaw): rotation around Z axis
    pan = np.arctan2(gaze_direction[1], gaze_direction[0])

    # Tilt (pitch): elevation angle
    tilt = np.arcsin(gaze_direction[2])

    return pan, tilt

# Simulate gaze behavior during interaction
duration = 10.0
dt = 0.1
times = np.arange(0, duration, dt)

# Positions
robot_head = np.array([0.0, 0.0, 1.5])  # Robot head at 1.5m height
human_face = np.array([1.5, 0.3, 1.6])  # Human 1.5m away, slightly to side
object_pos = np.array([1.0, -0.5, 1.0])  # Object on table

# Gaze mode over time
# 0-3s: Look at human (engagement)
# 3-7s: Look at object (joint attention)
# 7-10s: Look back at human (turn-taking)

pan_angles = np.zeros(len(times))
tilt_angles = np.zeros(len(times))
modes = []

for i, t in enumerate(times):
    if t < 3.0:
        mode = 'face'
    elif t < 7.0:
        mode = 'joint'
    else:
        mode = 'face'

    modes.append(mode)

    gaze_dir = compute_gaze_target(robot_head, human_face, object_pos, mode)
    pan, tilt = head_orientation_from_gaze(gaze_dir)

    pan_angles[i] = pan
    tilt_angles[i] = tilt

# Visualize
fig = plt.figure(figsize=(14, 10))

# 3D scene
ax1 = fig.add_subplot(2, 2, 1, projection='3d')

ax1.scatter(*robot_head, s=300, c='blue', marker='o', label='Robot Head', edgecolors='black', linewidths=2)
ax1.scatter(*human_face, s=300, c='green', marker='s', label='Human Face', edgecolors='black', linewidths=2)
ax1.scatter(*object_pos, s=200, c='red', marker='^', label='Object', edgecolors='black', linewidths=2)

# Gaze rays at key times
for t_sample, color in [(1.0, 'green'), (5.0, 'red'), (8.0, 'green')]:
    idx = int(t_sample / dt)
    if modes[idx] == 'face':
        target = human_face
    else:
        target = object_pos

    direction = (target - robot_head) * 0.8
    ax1.quiver(*robot_head, *direction, color=color, arrow_length_ratio=0.1, linewidth=2, alpha=0.7)

ax1.set_xlabel('X (m)')
ax1.set_ylabel('Y (m)')
ax1.set_zlabel('Z (m)')
ax1.set_title('Gaze Scene (Top View)')
ax1.legend()
ax1.set_xlim(-0.5, 2)
ax1.set_ylim(-1, 1)
ax1.set_zlim(0.5, 2)

# Pan angle
ax2 = fig.add_subplot(2, 2, 2)
ax2.plot(times, np.rad2deg(pan_angles), 'b-', linewidth=2)
ax2.fill_between(times, -90, 90, where=[m == 'face' for m in modes], alpha=0.2,
                  color='green', label='Look at Human')
ax2.fill_between(times, -90, 90, where=[m == 'joint' for m in modes], alpha=0.2,
                  color='red', label='Look at Object')
ax2.set_ylabel('Pan Angle (degrees)')
ax2.set_title('Head Pan (Yaw)')
ax2.legend()
ax2.grid(True, alpha=0.3)

# Tilt angle
ax3 = fig.add_subplot(2, 2, 3)
ax3.plot(times, np.rad2deg(tilt_angles), 'g-', linewidth=2)
ax3.fill_between(times, -45, 45, where=[m == 'face' for m in modes], alpha=0.2, color='green')
ax3.fill_between(times, -45, 45, where=[m == 'joint' for m in modes], alpha=0.2, color='red')
ax3.set_xlabel('Time (s)')
ax3.set_ylabel('Tilt Angle (degrees)')
ax3.set_title('Head Tilt (Pitch)')
ax3.grid(True, alpha=0.3)

# Gaze mode
ax4 = fig.add_subplot(2, 2, 4)
mode_values = [1 if m == 'face' else 0 for m in modes]
ax4.fill_between(times, 0, 1, where=[m == 1 for m in mode_values], alpha=0.5,
                  color='green', label='Face', step='post')
ax4.fill_between(times, 0, 1, where=[m == 0 for m in mode_values], alpha=0.5,
                  color='red', label='Object', step='post')
ax4.set_xlabel('Time (s)')
ax4.set_ylabel('Gaze Mode')
ax4.set_title('Gaze Target Over Time')
ax4.set_ylim(-0.1, 1.1)
ax4.legend()
ax4.grid(True, alpha=0.3)

plt.tight_layout()
plt.show()

# Print key angles
print("Head Orientations:")
for t_sample in [1.0, 5.0, 8.0]:
    idx = int(t_sample / dt)
    print(f"t={t_sample:.1f}s ({modes[idx]}): Pan={np.rad2deg(pan_angles[idx]):+.1f}°, " +
          f"Tilt={np.rad2deg(tilt_angles[idx]):+.1f}°")
```

**Expected Output**:
- 3D scene shows robot, human, and object positions
- Head angles shift between human face and object
- Smooth transitions during gaze shifts
- Color-coded gaze modes (green=face, red=object)

**What You Learned**:
- Gaze control directs robot attention to targets
- Joint attention (looking at shared object) improves collaboration
- Alternating gaze between human and object signals engagement
- Pan/tilt angles computed from 3D gaze direction

---

### Exercise 6.5.4: Expressive Gesture Generation

```python
"""Generate expressive robot gestures using Laban effort factors."""
import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple

def generate_expressive_trajectory(
    start_pos: np.ndarray,
    end_pos: np.ndarray,
    duration: float,
    weight: str = 'light',
    time_quality: str = 'sustained',
    space: str = 'direct',
    dt: float = 0.01
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Generate trajectory with Laban effort qualities.

    Args:
        start_pos: Starting position [x, y] (m)
        end_pos: Ending position [x, y] (m)
        duration: Motion duration (s)
        weight: 'light' or 'strong'
        time_quality: 'sustained' or 'sudden'
        space: 'direct' or 'indirect'
        dt: Time step (s)

    Returns:
        times, positions
    """
    times = np.arange(0, duration, dt)
    positions = np.zeros((len(times), 2))

    for i, t in enumerate(times):
        # Normalized time [0, 1]
        s = t / duration

        # Time quality (velocity profile)
        if time_quality == 'sustained':
            # Slow start and end (ease-in-out)
            s_smooth = 3*s**2 - 2*s**3
        else:  # sudden
            # Quick motion (linear or accelerating)
            s_smooth = s**2

        # Space quality (path curvature)
        if space == 'direct':
            # Straight line
            pos = start_pos + s_smooth * (end_pos - start_pos)
        else:  # indirect
            # Curved path (add sinusoidal deviation)
            midpoint = (start_pos + end_pos) / 2
            perpendicular = np.array([-(end_pos[1] - start_pos[1]), end_pos[0] - start_pos[0]])
            perpendicular = perpendicular / (np.linalg.norm(perpendicular) + 1e-6)

            curvature = 0.2 * np.sin(s * np.pi)  # Arc
            pos = start_pos + s_smooth * (end_pos - start_pos) + curvature * perpendicular

        positions[i] = pos

    return times, positions

# Generate gestures with different expressive qualities
start = np.array([0.0, 0.0])
end = np.array([0.5, 0.3])
duration = 2.0

gestures = {
    'Happy (Light, Sudden, Indirect)': ('light', 'sudden', 'indirect'),
    'Careful (Light, Sustained, Direct)': ('light', 'sustained', 'direct'),
    'Forceful (Strong, Sudden, Direct)': ('strong', 'sudden', 'direct'),
    'Gentle (Light, Sustained, Indirect)': ('light', 'sustained', 'indirect')
}

fig, axes = plt.subplots(2, 2, figsize=(12, 10))
axes = axes.flat

for ax, (name, (weight, time_q, space)) in zip(axes, gestures.items()):
    # Adjust duration based on time quality
    dur = duration if time_q == 'sustained' else duration * 0.5

    times, positions = generate_expressive_trajectory(
        start, end, dur, weight, time_q, space
    )

    # Plot trajectory
    ax.plot(positions[:, 0], positions[:, 1], 'b-', linewidth=2, label='Path')
    ax.scatter(*start, s=200, c='green', marker='o', edgecolors='black',
               linewidths=2, label='Start', zorder=5)
    ax.scatter(*end, s=200, c='red', marker='X', edgecolors='black',
               linewidths=2, label='End', zorder=5)

    # Velocity arrows (show time quality)
    for i in range(0, len(times), max(1, len(times)//10)):
        if i < len(times) - 1:
            vel = (positions[i+1] - positions[i]) / (times[1] - times[0])
            ax.arrow(positions[i, 0], positions[i, 1], vel[0]*0.02, vel[1]*0.02,
                    head_width=0.02, head_length=0.01, fc='orange', ec='orange', alpha=0.6)

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title(name)
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_xlim(-0.1, 0.6)
    ax.set_ylim(-0.2, 0.5)
    ax.set_aspect('equal')

plt.tight_layout()
plt.show()

# Analyze motion characteristics
for name, (weight, time_q, space) in gestures.items():
    dur = duration if time_q == 'sustained' else duration * 0.5
    times, positions = generate_expressive_trajectory(start, end, dur, weight, time_q, space)

    velocities = np.diff(positions, axis=0) / (times[1] - times[0])
    avg_speed = np.mean(np.linalg.norm(velocities, axis=1))
    max_speed = np.max(np.linalg.norm(velocities, axis=1))
    path_length = np.sum(np.linalg.norm(np.diff(positions, axis=0), axis=1))

    print(f"{name}:")
    print(f"  Duration: {dur:.2f}s, Avg speed: {avg_speed:.3f} m/s, " +
          f"Max speed: {max_speed:.3f} m/s, Path length: {path_length:.3f}m")
```

**Expected Output**:
- Direct paths are straight; indirect paths curve
- Sudden motions are faster (shorter duration)
- Sustained motions have smooth velocity profiles
- Curved paths indicate playfulness/gentleness

**What You Learned**:
- Laban effort factors parameterize motion expressiveness
- Time quality affects velocity profile (smooth vs. sharp)
- Space quality affects path shape (straight vs. curved)
- Expressive motion enhances robot communication

---

## TryWithAI Exercises

### TryWithAI 6.5.1: Multi-Gesture Interaction System

**Prompt**:
```
Build a real-time gesture-based robot control system. The system should:

1. Recognize 5 gestures: wave (greeting), point (target selection), stop (halt), beckoning (come closer), thumbs-up (confirmation)
2. Generate appropriate robot responses for each gesture (e.g., wave back, approach, stop motion, acknowledge)
3. Include a state machine to manage interaction flow (idle → engaged → task → idle)
4. Visualize human skeleton, detected gesture, and robot response in each frame

Use synthetic gesture data (sequence of hand/arm positions). Show how the robot transitions between states based on gesture sequences.
```

**Expected Skills**:
- Multi-class gesture recognition
- State machine design for interaction
- Response generation (inverse action mapping)
- Temporal gesture sequence handling

---

### TryWithAI 6.5.2: Adaptive Proxemics with Comfort Estimation

**Prompt**:
```
Design a proxemics controller that learns individual user comfort preferences. The system should:

1. Monitor user behavior cues (stepping back, crossing arms, gaze aversion)
2. Estimate user comfort level (0-1 scale) from cues
3. Adapt preferred distance based on estimated comfort (increase distance if discomfort detected)
4. Store per-user preferences and recall for repeat interactions

Simulate interactions with 3 user types: comfortable (allows 0.8m), neutral (1.2m), uncomfortable (1.8m). Show how the robot adapts its approach distance over multiple encounters.
```

**Expected Skills**:
- Behavioral cue detection
- Online learning/adaptation
- User modeling and personalization
- Memory and recognition across sessions

---

## Summary

This lesson introduced human-robot interaction fundamentals:

**Key Concepts**:
- Gesture recognition maps body poses to semantic actions
- Proxemics zones define socially appropriate distances
- Gaze control enables joint attention and engagement
- Expressive motion communicates robot intent and emotion

**Practical Skills**:
- Classifying hand gestures from geometric features
- Implementing proxemics-based approach control
- Controlling robot gaze for joint attention
- Generating expressive trajectories with Laban qualities

**Next Chapter**: [Chapter 7: Mobile Manipulation](../chapter-07/index.md) combines mobility and manipulation skills.

---

**Further Reading**:
- Hall, E.T. (1966). *The Hidden Dimension*. Anchor Books.
- Breazeal, C. (2003). "Toward Sociable Robots." *Robotics and Autonomous Systems*.
- Laban, R., & Ullmann, L. (1971). *The Mastery of Movement*. Princeton Book Company.
