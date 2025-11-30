---
title: "Lesson 7.1: Voice-to-Action with OpenAI Whisper"
description: "Learn to use OpenAI Whisper for speech recognition and voice command processing"
chapter: 7
lesson: 1
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
prerequisites: ["chapter-02-lesson-02"]
has_interactive_python: true
interactive_python_count: 1
has_try_with_ai: false
try_with_ai_count: 0
tags: ["whisper", "speech-recognition", "voice-commands", "vla"]
---

# Lesson 7.1: Voice-to-Action with OpenAI Whisper

## 🎯 Learning Objectives

- Understand OpenAI Whisper architecture and capabilities
- Integrate Whisper API for speech-to-text conversion
- Process voice commands for robot control
- Implement real-time voice command recognition
- Publish voice commands to ROS 2 topics

**Time**: 70 minutes

---

## Introduction

**OpenAI Whisper** is a state-of-the-art speech recognition system that can transcribe audio in multiple languages with high accuracy. For robotics, Whisper enables natural voice control—users can speak commands instead of typing or using complex interfaces.

**Key Features**:
- High accuracy across languages
- Handles accents and background noise
- Real-time transcription capability
- Free API tier available

---

## 1. OpenAI Whisper Overview

### Architecture

Whisper uses a transformer-based architecture:
- **Encoder**: Processes audio into features
- **Decoder**: Generates text transcription
- **Multilingual**: Supports 99+ languages

### Use Cases in Robotics

- Voice commands: "Move forward", "Pick up the cup"
- Natural language: "Can you help me clean the room?"
- Multi-modal interaction: Combine voice with gestures
- Accessibility: Enable voice control for users

---

## 2. Whisper API Integration

### Basic Usage

```python
import openai

# Transcribe audio file
audio_file = open("voice_command.wav", "rb")
transcript = openai.Audio.transcribe(
    model="whisper-1",
    file=audio_file,
    language="en"
)
print(transcript["text"])
```

### Real-Time Transcription

For real-time voice commands, you'll need to:
1. Capture audio from microphone
2. Send chunks to Whisper API
3. Process transcriptions
4. Extract commands

---

## 3. ROS 2 Integration

### Voice Command Publisher

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import openai

class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command_node')
        self.publisher = self.create_publisher(String, '/voice_commands', 10)
        
    def transcribe_audio(self, audio_file):
        """Transcribe audio using Whisper."""
        transcript = openai.Audio.transcribe(
            model="whisper-1",
            file=audio_file
        )
        return transcript["text"]
    
    def publish_command(self, text):
        """Publish voice command to ROS 2 topic."""
        msg = String()
        msg.data = text
        self.publisher.publish(msg)
        self.get_logger().info(f'Published command: {text}')
```

---

## 4. Command Processing

### Extracting Robot Commands

Voice commands need to be parsed and validated:

```python
def parse_voice_command(text):
    """Extract robot action from natural language."""
    text_lower = text.lower()
    
    # Movement commands
    if "move forward" in text_lower or "go forward" in text_lower:
        return {"action": "move", "direction": "forward"}
    elif "turn left" in text_lower:
        return {"action": "turn", "direction": "left"}
    elif "pick up" in text_lower:
        # Extract object name
        return {"action": "pick", "object": extract_object(text_lower)}
    
    return None
```

---

## 5. Exercises

### Exercise 7.1.1: Basic Whisper Transcription

Transcribe a sample audio file using Whisper API.

<InteractivePython
  id="ex-7-1-1"
  title="Whisper Transcription"
  starterCode={`# Note: This exercise requires OpenAI API key
# In real implementation, use: openai.Audio.transcribe()

def simulate_whisper_transcription(audio_text):
    """
    Simulate Whisper transcription (for browser-based exercise).
    In real implementation, this would call OpenAI API.
    """
    # TODO: Simulate transcription process
    # Return transcribed text
    pass

# Test
audio = "Hello robot, please move forward"
transcript = simulate_whisper_transcription(audio)
print(f"Transcribed: {transcript}")
`}
  hints={[
    "For simulation, return the input text (real API would process audio)",
    "In real code, use openai.Audio.transcribe()",
    "Handle errors and edge cases"
  ]}
/>

---

## Summary

**Key Takeaways**:
1. Whisper provides high-accuracy speech recognition
2. Voice commands enable natural robot interaction
3. ROS 2 integration allows voice control of robots
4. Command parsing extracts actionable instructions

**What's Next**: [Lesson 7.2: LLM-Based Cognitive Planning](./lesson-02-llm-planning.md) teaches you to use GPT models for task planning.

---

**Estimated completion time**: 70 minutes | **Prerequisites**: Chapter 2 (ROS 2) | **Difficulty**: B2 (Advanced)

