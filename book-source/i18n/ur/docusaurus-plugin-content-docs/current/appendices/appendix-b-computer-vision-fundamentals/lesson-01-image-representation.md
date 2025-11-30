---
title: "Lesson 3.1: Image Representation and Processing"
description: "Learn how images are represented as arrays and perform fundamental image processing operations"
chapter: 3
lesson: 1
estimated_time: 50
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
prerequisites:
  - "chapter-01-lesson-04"
has_interactive_python: true
interactive_python_count: 3
has_try_with_ai: true
try_with_ai_count: 1
tags: ["image-processing", "numpy", "rgb", "grayscale", "thresholding"]
---

# Lesson 3.1: Image Representation and Processing

## 🎯 Learning Objectives

<LearningObjectives
  cefr_level="B1+"
  objectives={[
    {
      text: "Understand how images are represented as NumPy arrays",
      blooms_level: "Understand",
      assessment_method: "Quiz and exercises"
    },
    {
      text: "Convert RGB images to grayscale using weighted averaging",
      blooms_level: "Apply",
      assessment_method: "Python exercise"
    },
    {
      text: "Apply binary thresholding for image segmentation",
      blooms_level: "Apply",
      assessment_method: "Segmentation exercise"
    }
  ]}
/>

## 📚 Prerequisites

<Prerequisites
  prerequisites={[
    {
      lessonId: "chapter-01-lesson-04",
      title: "Lesson 1.4: Python for Robotics Introduction",
      link: "/docs/chapter-01/lesson-04-python-robotics-intro"
    }
  ]}
/>

## Introduction

**Computer vision** starts with understanding how computers represent images. Unlike humans who see continuous scenes, computers store images as **grids of numbers**.

In this lesson, you'll learn:
- How images are stored as NumPy arrays
- RGB color representation
- Converting color images to grayscale
- Binary thresholding for segmentation

**Time**: 50 minutes

---

## 1. Images as NumPy Arrays

### Grayscale Images

A grayscale image is a 2D array where each pixel has a brightness value (0 = black, 255 = white):

```python
import numpy as np

# 5x5 grayscale image
image = np.array([
    [0,   50,  100, 150, 200],
    [25,  75,  125, 175, 225],
    [50,  100, 150, 200, 250],
    [75,  125, 175, 225, 255],
    [100, 150, 200, 250, 255]
], dtype=np.uint8)

print(f"Shape: {image.shape}")  # (5, 5)
print(f"Pixel at (2, 3): {image[2, 3]}")  # 200
```

### RGB Color Images

Color images have **3 channels** (Red, Green, Blue), forming a 3D array:

```python
# Shape: (height, width, 3)
rgb_image = np.zeros((100, 100, 3), dtype=np.uint8)

# Red pixel at (50, 50)
rgb_image[50, 50] = [255, 0, 0]  # [R, G, B]

# Green pixel at (60, 60)
rgb_image[60, 60] = [0, 255, 0]

# Blue pixel at (70, 70)
rgb_image[70, 70] = [0, 0, 255]
```

**Key Insight**: `rgb_image[row, col]` returns a 3-element array `[R, G, B]`.

---

## 2. RGB to Grayscale Conversion

### Why Grayscale?

Many vision algorithms work on grayscale images because:
- **Faster**: 1/3 the data to process
- **Simpler**: No color channel management
- **Sufficient**: Many features don't need color

### Conversion Formula

Human eyes don't perceive R, G, B equally. Standard conversion:

$$
\text{Gray} = 0.299 \times R + 0.587 \times G + 0.114 \times B
$$

**Why these weights?**
- Green contributes most to perceived brightness (0.587)
- Blue contributes least (0.114)

```python
def rgb_to_gray(rgb_image):
    """Convert RGB to grayscale using standard weights."""
    return (0.299 * rgb_image[:,:,0] +  # Red channel
            0.587 * rgb_image[:,:,1] +  # Green channel
            0.114 * rgb_image[:,:,2])   # Blue channel
```

---

## 3. Binary Thresholding

**Thresholding** converts a grayscale image to **binary** (black and white only):

$$
\text{Binary}(x, y) = \begin{cases}
255 & \text{if } \text{Gray}(x, y) > \text{threshold} \\
0 & \text{otherwise}
\end{cases}
$$

**Use cases**:
- Segmenting objects from background
- Detecting bright objects
- Preprocessing for edge detection

```python
def threshold(image, threshold_value):
    """Apply binary threshold."""
    binary = np.zeros_like(image)
    binary[image > threshold_value] = 255
    return binary
```

---

## 4. Exercises

### Exercise 3.1.1: Create and Display Synthetic Image

Create simple synthetic images for testing.

<InteractivePython
  id="ex-3-1-1"
  title="Create Synthetic Images"
  starterCode={`import numpy as np
import matplotlib.pyplot as plt

def create_checkerboard(size=100, squares=10):
    """
    Create a checkerboard pattern.

    Args:
        size: Image size (size x size pixels)
        squares: Number of squares per row/column

    Returns:
        Binary checkerboard image (0s and 255s)
    """
    # TODO: Implement checkerboard generation
    # 1. Create square_size = size // squares
    # 2. Create empty array
    # 3. Fill alternating squares with 255
    # Hint: Use (row // square_size + col // square_size) % 2
    pass

# Test
board = create_checkerboard(100, 10)
plt.imshow(board, cmap='gray')
plt.title('Checkerboard Pattern')
plt.axis('off')
plt.show()

print(f"Shape: {board.shape}")
print(f"Unique values: {np.unique(board)}")
`}
  hints={[
    "square_size = size // squares",
    "image = np.zeros((size, size), dtype=np.uint8)",
    "For each pixel (i,j): color = (i//square_size + j//square_size) % 2",
    "image[i,j] = 255 if color == 0 else 0"
  ]}
/>

---

### Exercise 3.1.2: RGB to Grayscale Conversion

Implement the standard RGB to grayscale conversion.

<InteractivePython
  id="ex-3-1-2"
  title="RGB to Grayscale"
  starterCode={`import numpy as np
import matplotlib.pyplot as plt

def rgb_to_grayscale(rgb_image):
    """
    Convert RGB image to grayscale using weighted average.

    Args:
        rgb_image: (H, W, 3) NumPy array

    Returns:
        (H, W) grayscale image
    """
    # TODO: Implement RGB to grayscale
    # Formula: 0.299*R + 0.587*G + 0.114*B
    pass

# Test with synthetic RGB image
rgb = np.zeros((100, 100, 3), dtype=np.uint8)
rgb[:, :33, 0] = 255  # Red third
rgb[:, 33:66, 1] = 255  # Green third
rgb[:, 66:, 2] = 255  # Blue third

gray = rgb_to_grayscale(rgb)

# Display
fig, axes = plt.subplots(1, 2, figsize=(10, 4))
axes[0].imshow(rgb)
axes[0].set_title('RGB Image')
axes[0].axis('off')

axes[1].imshow(gray, cmap='gray')
axes[1].set_title('Grayscale')
axes[1].axis('off')

plt.show()

print(f"RGB shape: {rgb.shape}")
print(f"Gray shape: {gray.shape}")
print(f"Red column avg: {gray[50, 16]:.1f}")   # Should be ~76 (0.299*255)
print(f"Green column avg: {gray[50, 50]:.1f}") # Should be ~150 (0.587*255)
print(f"Blue column avg: {gray[50, 83]:.1f}")  # Should be ~29 (0.114*255)
`}
  hints={[
    "Extract channels: R = rgb_image[:,:,0], G = rgb_image[:,:,1], B = rgb_image[:,:,2]",
    "gray = 0.299 * R + 0.587 * G + 0.114 * B",
    "Convert to uint8: gray.astype(np.uint8)",
    "Return grayscale image"
  ]}
/>

---

### Exercise 3.1.3: Binary Threshold Segmentation

Apply thresholding to segment objects from background.

<InteractivePython
  id="ex-3-1-3"
  title="Binary Thresholding"
  starterCode={`import numpy as np
import matplotlib.pyplot as plt

def apply_threshold(image, threshold_value):
    """
    Apply binary threshold to image.

    Args:
        image: Grayscale image
        threshold_value: Threshold (0-255)

    Returns:
        Binary image (0 or 255)
    """
    # TODO: Implement thresholding
    # 1. Create binary image (zeros)
    # 2. Set pixels > threshold to 255
    # 3. Return binary image
    pass

# Create test image with gradients
test_img = np.zeros((100, 100), dtype=np.uint8)
for i in range(100):
    test_img[i, :] = i * 255 // 100  # Vertical gradient

# Apply different thresholds
thresholds = [50, 127, 200]

fig, axes = plt.subplots(1, 4, figsize=(16, 4))
axes[0].imshow(test_img, cmap='gray')
axes[0].set_title('Original')
axes[0].axis('off')

for idx, thresh in enumerate(thresholds):
    binary = apply_threshold(test_img, thresh)
    axes[idx+1].imshow(binary, cmap='gray')
    axes[idx+1].set_title(f'Threshold = {thresh}')
    axes[idx+1].axis('off')

plt.show()

# Verify
binary_127 = apply_threshold(test_img, 127)
print(f"Pixels above threshold: {np.sum(binary_127 == 255)}")
print(f"Expected: ~5000 (half the image)")
`}
  hints={[
    "binary = np.zeros_like(image)",
    "binary[image > threshold_value] = 255",
    "Return binary",
    "Alternatively: binary = (image > threshold_value).astype(np.uint8) * 255"
  ]}
/>

---

## 5. Try With AI

### TryWithAI 3.1.1: Image Processing Optimization

<TryWithAI
  id="tryai-3-1-1"
  title="Optimize Image Processing Code"
  role="Copilot"
  scenario="Your image processing code works but is slow. You want to optimize it for real-time robot vision (30 FPS = 33ms per frame)."
  yourTask="Complete exercises 3.1.2 and 3.1.3. Time your code with a 640x480 image. Identify bottlenecks."
  aiPromptTemplate="I've implemented RGB to grayscale and thresholding. Here's my code: [paste]. For a 640x480 image, it takes [X] ms. I need to process 30 frames per second (< 33ms per frame). Can you help me optimize this code? Specifically: (1) Are there faster NumPy operations I can use? (2) Should I vectorize differently? (3) Can I avoid loops? Also explain the performance differences."
  successCriteria={[
    "You understand vectorization vs. loops for image processing",
    "You can profile code to find bottlenecks",
    "Your optimized code runs in < 10ms for 640x480 images"
  ]}
  reflectionQuestions={[
    "What's the computational complexity of your operations?",
    "How does memory layout affect performance?",
    "When should you use NumPy vs. specialized libraries (OpenCV)?"
  ]}
/>

---

## Summary

**Key Takeaways**:

1. **Image Representation**:
   - Grayscale: 2D array `(height, width)`, values 0-255
   - RGB: 3D array `(height, width, 3)`, each channel 0-255
   - Access: `image[row, col]` or `image[row, col, channel]`

2. **RGB to Grayscale**:
   - Formula: `0.299*R + 0.587*G + 0.114*B`
   - Green weighted highest (human vision perception)
   - Reduces data by 3x for faster processing

3. **Binary Thresholding**:
   - Converts grayscale to pure black/white
   - `pixel = 255 if gray > threshold else 0`
   - Used for segmentation and preprocessing

4. **NumPy Operations**:
   - Vectorized operations (no loops!)
   - Boolean indexing: `binary[image > thresh] = 255`
   - Broadcasting for efficient array operations

**What's Next**: [Lesson 3.2: Edge and Corner Detection](./lesson-02-edge-detection.md) uses image gradients to find object boundaries and features.

---

**Estimated completion time**: 50 minutes | **Prerequisites**: Lesson 1.4 | **Difficulty**: B1+ (Upper Intermediate)
