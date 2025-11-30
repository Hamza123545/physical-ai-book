---
title: "Lesson 3.2: Edge and Corner Detection"
description: "Detect edges and corners in images using gradient-based operators"
chapter: 3
lesson: 2
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
prerequisites:
  - "chapter-03-lesson-01"
has_interactive_python: true
interactive_python_count: 3
has_try_with_ai: true
try_with_ai_count: 2
tags: ["edge-detection", "sobel", "canny", "corners", "gradients"]
---

# Lesson 3.2: Edge and Corner Detection

## 🎯 Learning Objectives

<LearningObjectives
  cefr_level="B2"
  objectives={[
    {
      text: "Implement Sobel edge detection using image gradients",
      blooms_level: "Apply",
      assessment_method: "Python exercise with edge visualization"
    },
    {
      text: "Apply Canny edge detection algorithm",
      blooms_level: "Apply",
      assessment_method: "Multi-stage edge detection exercise"
    },
    {
      text: "Detect corners for feature extraction",
      blooms_level: "Apply",
      assessment_method: "Corner detection exercise"
    }
  ]}
/>

## 📚 Prerequisites

<Prerequisites
  prerequisites={[
    {
      lessonId: "chapter-03-lesson-01",
      title: "Lesson 3.1: Image Representation",
      link: "/docs/chapter-03/lesson-01-image-representation"
    }
  ]}
/>

## Introduction

**Edges** are sharp changes in image intensity—they mark object boundaries, shadows, and surface orientation changes. **Corners** are distinctive points where edges meet, useful for tracking and matching.

**Why detect edges and corners?**
- Object recognition and segmentation
- Feature tracking across frames
- 3D reconstruction
- Navigation and obstacle detection

**Time**: 60 minutes

---

## 1. Image Gradients

### What is a Gradient?

The **gradient** measures how quickly pixel intensity changes:

$$
\nabla I = \begin{bmatrix} \frac{\partial I}{\partial x} \\ \frac{\partial I}{\partial y} \end{bmatrix}
$$

**Magnitude**: How strong the edge is
$$
|\nabla I| = \sqrt{G_x^2 + G_y^2}
$$

**Direction**: Edge orientation
$$
\theta = \text{atan2}(G_y, G_x)
$$

---

## 2. Sobel Edge Detector

### Sobel Operators

Sobel uses two 3×3 kernels to compute gradients:

**Horizontal edges** (vertical gradient):
$$
G_x = \begin{bmatrix} -1 & 0 & 1 \\ -2 & 0 & 2 \\ -1 & 0 & 1 \end{bmatrix}
$$

**Vertical edges** (horizontal gradient):
$$
G_y = \begin{bmatrix} -1 & -2 & -1 \\ 0 & 0 & 0 \\ 1 & 2 & 1 \end{bmatrix}
$$

**How it works**:
1. Convolve image with $G_x$ and $G_y$
2. Compute magnitude: $\sqrt{G_x^2 + G_y^2}$
3. Threshold to get binary edges

---

## 3. Canny Edge Detection

**Canny** is a multi-stage algorithm producing clean, thin edges:

**Steps**:
1. **Gaussian blur**: Reduce noise
2. **Gradient**: Compute magnitude and direction (Sobel)
3. **Non-maximum suppression**: Thin edges to 1-pixel width
4. **Double threshold**: Classify strong/weak edges
5. **Edge tracking**: Connect weak edges to strong edges

**Advantages**: Clean edges, less noise, single-pixel width

---

## 4. Corner Detection

**Corners** are points where gradients change in multiple directions.

**Harris Corner Detector**:
- Computes image gradients in x and y
- Finds points where both $G_x$ and $G_y$ are large
- Corners have high response in Harris matrix

**Uses**: Feature tracking, image matching, 3D reconstruction

---

## 5. Exercises

### Exercise 3.2.1: Sobel Edge Detector

Implement Sobel edge detection.

<InteractivePython
  id="ex-3-2-1"
  title="Sobel Edge Detection"
  starterCode={`import numpy as np
import matplotlib.pyplot as plt
from scipy import signal

def sobel_edges(image):
    """
    Detect edges using Sobel operator.

    Args:
        image: Grayscale image (H, W)

    Returns:
        Edge magnitude image
    """
    # Sobel kernels
    sobel_x = np.array([[-1, 0, 1],
                        [-2, 0, 2],
                        [-1, 0, 1]])

    sobel_y = np.array([[-1, -2, -1],
                        [ 0,  0,  0],
                        [ 1,  2,  1]])

    # TODO: Implement Sobel edge detection
    # 1. Convolve image with sobel_x and sobel_y
    # 2. Compute gradient magnitude: sqrt(Gx^2 + Gy^2)
    # 3. Return edge magnitude
    pass

# Test with synthetic image
test_img = np.zeros((100, 100))
test_img[30:70, 30:70] = 255  # White square on black

edges = sobel_edges(test_img)

fig, axes = plt.subplots(1, 2, figsize=(10, 4))
axes[0].imshow(test_img, cmap='gray')
axes[0].set_title('Original')
axes[0].axis('off')

axes[1].imshow(edges, cmap='gray')
axes[1].set_title('Sobel Edges')
axes[1].axis('off')

plt.show()

print(f"Edge magnitude range: {edges.min():.1f} - {edges.max():.1f}")
`}
  hints={[
    "Use scipy.signal.convolve2d(image, kernel, mode='same')",
    "Gx = signal.convolve2d(image, sobel_x, mode='same')",
    "Gy = signal.convolve2d(image, sobel_y, mode='same')",
    "magnitude = np.sqrt(Gx**2 + Gy**2)",
    "Return magnitude"
  ]}
/>

---

### Exercise 3.2.2: Simplified Canny Edge Detection

Implement key stages of Canny edge detection.

<InteractivePython
  id="ex-3-2-2"
  title="Canny Edge Detection (Simplified)"
  starterCode={`import numpy as np
import matplotlib.pyplot as plt
from scipy import ndimage, signal

def canny_edges(image, low_threshold=50, high_threshold=150):
    """
    Simplified Canny edge detection.

    Args:
        image: Grayscale image
        low_threshold: Low threshold for edge tracking
        high_threshold: High threshold for strong edges

    Returns:
        Binary edge image
    """
    # TODO: Implement Canny edge detection
    # 1. Gaussian blur (use ndimage.gaussian_filter)
    # 2. Compute gradients (Sobel from previous exercise)
    # 3. Threshold: strong edges (> high), weak edges (low < x < high)
    # 4. Return binary edges
    pass

# Test
test_img = np.zeros((100, 100))
# Draw circle
y, x = np.ogrid[:100, :100]
circle = (x - 50)**2 + (y - 50)**2 < 30**2
test_img[circle] = 255

edges = canny_edges(test_img, low_threshold=50, high_threshold=150)

fig, axes = plt.subplots(1, 2, figsize=(10, 4))
axes[0].imshow(test_img, cmap='gray')
axes[0].set_title('Original')
axes[0].axis('off')

axes[1].imshow(edges, cmap='gray')
axes[1].set_title('Canny Edges')
axes[1].axis('off')

plt.show()
`}
  hints={[
    "blurred = ndimage.gaussian_filter(image, sigma=1.4)",
    "Compute Sobel gradients on blurred image",
    "strong_edges = (gradient_mag > high_threshold)",
    "edges = strong_edges.astype(np.uint8) * 255",
    "Return edges"
  ]}
/>

---

### Exercise 3.2.3: Simple Corner Detection

Detect corners using gradient analysis.

<InteractivePython
  id="ex-3-2-3"
  title="Corner Detection"
  starterCode={`import numpy as np
import matplotlib.pyplot as plt
from scipy import signal, ndimage

def detect_corners(image, threshold=0.01):
    """
    Detect corners using simplified Harris-like approach.

    Args:
        image: Grayscale image
        threshold: Corner response threshold

    Returns:
        Binary image with corners marked
    """
    # Sobel gradients
    sobel_x = np.array([[-1, 0, 1], [-2, 0, 2], [-1, 0, 1]])
    sobel_y = np.array([[-1, -2, -1], [0, 0, 0], [1, 2, 1]])

    # TODO: Implement corner detection
    # 1. Compute gradients Ix, Iy
    # 2. Compute products: Ix^2, Iy^2, Ix*Iy
    # 3. Apply Gaussian to products (smooth)
    # 4. Corner response: det(M) - k*trace(M)^2
    #    where M = [[Ix^2, Ix*Iy], [Ix*Iy, Iy^2]]
    # 5. Threshold response to find corners
    pass

# Test with square
test_img = np.zeros((100, 100))
test_img[20:80, 20:80] = 255

corners = detect_corners(test_img, threshold=0.01)

plt.figure(figsize=(12, 4))
plt.subplot(131)
plt.imshow(test_img, cmap='gray')
plt.title('Original')
plt.axis('off')

plt.subplot(132)
plt.imshow(corners, cmap='gray')
plt.title('Detected Corners')
plt.axis('off')

# Overlay corners on original
plt.subplot(133)
plt.imshow(test_img, cmap='gray')
y_coords, x_coords = np.where(corners > 0)
plt.plot(x_coords, y_coords, 'ro', markersize=8)
plt.title('Corners Overlaid')
plt.axis('off')

plt.show()

print(f"Number of corners detected: {np.sum(corners > 0)}")
`}
  hints={[
    "Ix = signal.convolve2d(image, sobel_x, mode='same')",
    "Iy = signal.convolve2d(image, sobel_y, mode='same')",
    "Ix2 = ndimage.gaussian_filter(Ix * Ix, sigma=1)",
    "Iy2 = ndimage.gaussian_filter(Iy * Iy, sigma=1)",
    "Ixy = ndimage.gaussian_filter(Ix * Iy, sigma=1)",
    "det = Ix2 * Iy2 - Ixy**2",
    "trace = Ix2 + Iy2",
    "response = det - 0.04 * trace**2",
    "corners = (response > threshold * response.max()).astype(np.uint8) * 255"
  ]}
/>

---

## 6. Try With AI

### TryWithAI 3.2.1: Edge Detection Algorithms

<TryWithAI
  id="tryai-3-2-1"
  title="Compare Edge Detection Methods"
  role="Teacher"
  scenario="You want to understand the differences between Sobel, Canny, and other edge detectors."
  yourTask="Complete exercises 3.2.1 and 3.2.2. Compare the outputs. Notice that Canny produces cleaner, thinner edges."
  aiPromptTemplate="I've implemented both Sobel and Canny edge detection. Here are my observations: [describe differences]. Can you explain: (1) Why does Canny produce thinner edges? (2) What is non-maximum suppression? (3) When would I prefer Sobel over Canny (or vice versa)? (4) Are there other edge detection methods I should know about (Prewitt, Laplacian of Gaussian)?"
  successCriteria={[
    "You understand why Canny edges are thinner (non-max suppression)",
    "You know the trade-offs: Sobel (fast) vs Canny (clean)",
    "You can choose appropriate detector for your application"
  ]}
  reflectionQuestions={[
    "What happens if you set Canny thresholds too high or too low?",
    "How does noise affect edge detection?",
    "Can you detect edges in color images?"
  ]}
/>

---

### TryWithAI 3.2.2: Parameter Tuning

<TryWithAI
  id="tryai-3-2-2"
  title="Tune Edge Detection Parameters"
  role="Evaluator"
  scenario="Your edge detector finds too many false edges (noise) or misses important edges."
  yourTask="Run exercise 3.2.2 with different threshold values. Observe how changing thresholds affects results."
  aiPromptTemplate="I'm tuning Canny edge detection thresholds. With low=50, high=150, I get [describe results]. Here's my edge image: [describe or paste statistics]. Can you review my parameters and suggest: (1) Are my thresholds too high/low? (2) How should I choose thresholds systematically? (3) What's a good ratio between high and low thresholds? (4) How can I make edge detection robust to different lighting?"
  successCriteria={[
    "You understand how thresholds affect edge detection quality",
    "You can systematically tune parameters",
    "You know typical threshold ratios (high:low ≈ 2:1 or 3:1)"
  ]}
  reflectionQuestions={[
    "Can you auto-tune thresholds based on image statistics?",
    "What's the relationship between blur (sigma) and edge thickness?",
    "How would you validate edge detection quality?"
  ]}
/>

---

## Summary

**Key Takeaways**:

1. **Image Gradients**:
   - Measure intensity change: $\nabla I = [G_x, G_y]$
   - Magnitude: $\sqrt{G_x^2 + G_y^2}$
   - Direction: $\text{atan2}(G_y, G_x)$

2. **Sobel Edge Detection**:
   - Convolve with Sobel kernels
   - Fast, simple, but noisy edges
   - Good for real-time applications

3. **Canny Edge Detection**:
   - Multi-stage: blur → gradient → non-max suppression → thresholding
   - Clean, thin, connected edges
   - More computationally expensive

4. **Corner Detection**:
   - Finds points where edges meet
   - Harris detector: high gradient variance in both directions
   - Useful for feature tracking

**What's Next**: [Lesson 3.3: Object Detection and Tracking](./lesson-03-object-detection.md) uses color segmentation to find and track objects.

---

**Estimated completion time**: 60 minutes | **Prerequisites**: Lesson 3.1 | **Difficulty**: B2 (Advanced)
