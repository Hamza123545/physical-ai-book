---
title: "Chapter 3 Quiz: Computer Vision for Robotics"
description: "Test your understanding of computer vision concepts for robotics"
chapter: 3
quiz_type: "chapter"
total_questions: 10
passing_score: 70
estimated_time: 20
generated_by: "claude-sonnet-4-5-20250929"
source_spec: "specs/002-physical-ai-textbook/spec.md"
created: "2025-11-29"
last_modified: "2025-11-29"
git_author: "hswat"
workflow: "/sp.implement"
version: "1.0"
---

# Chapter 3 Quiz: Computer Vision for Robotics

<Quiz
  chapterId={3}
  questions={[
    {
      id: "q-ch03-01",
      question: "What is the standard formula for converting an RGB image to grayscale?",
      type: "multiple-choice",
      options: [
        "Gray = (R + G + B) / 3",
        "Gray = 0.299*R + 0.587*G + 0.114*B",
        "Gray = 0.33*R + 0.33*G + 0.33*B",
        "Gray = max(R, G, B)"
      ],
      correctAnswer: 1,
      explanation: "The standard grayscale conversion uses weighted averaging (0.299*R + 0.587*G + 0.114*B) because human eyes perceive green more strongly than red or blue. This is the ITU-R BT.601 standard.",
      wrongAnswerExplanations: [
        "Simple averaging (R+G+B)/3 doesn't account for human perception differences between color channels.",
        "Equal weights (0.33 each) ignore the fact that human eyes are more sensitive to green light.",
        "Taking the maximum value would result in very bright, unrealistic grayscale images."
      ],
      difficulty: "medium",
      blooms_level: "Understand",
      tags: ["image-processing", "grayscale", "rgb"]
    },
    {
      id: "q-ch03-02",
      question: "What is the purpose of the Sobel operator in edge detection?",
      type: "multiple-choice",
      options: [
        "To blur the image and reduce noise",
        "To compute image gradients in x and y directions",
        "To convert color images to binary",
        "To detect corners in the image"
      ],
      correctAnswer: 1,
      explanation: "The Sobel operator uses two 3×3 convolution kernels to compute horizontal and vertical gradients (Gx and Gy). The gradient magnitude √(Gx² + Gy²) indicates edge strength.",
      wrongAnswerExplanations: [
        "Gaussian blur is used for noise reduction, not Sobel. Sobel actually emphasizes edges, not blurs them.",
        "Binary thresholding converts to binary, not the Sobel operator. Sobel produces gradient magnitude images.",
        "Harris corner detector or similar methods detect corners, not Sobel. Sobel detects edges (boundaries)."
      ],
      difficulty: "medium",
      blooms_level: "Understand",
      tags: ["edge-detection", "sobel", "gradients"]
    },
    {
      id: "q-ch03-03",
      question: "In the Canny edge detection algorithm, what is the purpose of non-maximum suppression?",
      type: "multiple-choice",
      options: [
        "To remove noise from the image",
        "To thin edges to single-pixel width",
        "To connect weak edges to strong edges",
        "To compute gradient magnitude"
      ],
      correctAnswer: 1,
      explanation: "Non-maximum suppression thins edges to single-pixel width by keeping only local maxima in the gradient direction. This produces clean, precise edge maps.",
      wrongAnswerExplanations: [
        "Gaussian blur (the first step of Canny) removes noise, not non-maximum suppression.",
        "Edge tracking (the final step) connects weak to strong edges, not non-maximum suppression.",
        "Sobel or similar operators compute gradient magnitude before non-maximum suppression is applied."
      ],
      difficulty: "hard",
      blooms_level: "Understand",
      tags: ["canny", "edge-detection", "non-maximum-suppression"]
    },
    {
      id: "q-ch03-04",
      question: "How do you compute the centroid (center of mass) of a binary object in an image?",
      type: "multiple-choice",
      options: [
        "Find the brightest pixel in the object",
        "Take the average of all pixel coordinates in the object",
        "Find the median x and y coordinates",
        "Use the top-left corner of the bounding box"
      ],
      correctAnswer: 1,
      explanation: "The centroid is computed as (x_c, y_c) = (mean(x_coords), mean(y_coords)) where x_coords and y_coords are all pixel positions belonging to the object. This gives the center of mass.",
      wrongAnswerExplanations: [
        "The brightest pixel may not be at the center of the object; centroid requires averaging all object pixels.",
        "Median would give a median-center, not the true center of mass (centroid).",
        "The bounding box corner is not representative of the object's actual center of mass."
      ],
      difficulty: "medium",
      blooms_level: "Apply",
      tags: ["object-detection", "centroids", "image-processing"]
    },
    {
      id: "q-ch03-05",
      question: "Why is HSV color space often preferred over RGB for color-based object segmentation?",
      type: "multiple-choice",
      options: [
        "HSV is faster to compute than RGB",
        "HSV separates color (hue) from brightness (value), making it more robust to lighting changes",
        "HSV uses less memory than RGB",
        "HSV images are always higher quality"
      ],
      correctAnswer: 1,
      explanation: "HSV separates hue (color), saturation (color intensity), and value (brightness). This separation makes color segmentation more robust to lighting variations because you can filter on hue while tolerating brightness changes.",
      wrongAnswerExplanations: [
        "RGB to HSV conversion actually adds computational cost, it's not faster.",
        "HSV and RGB both use 3 channels with the same memory footprint.",
        "Image quality is independent of color space representation; HSV doesn't improve quality, just robustness."
      ],
      difficulty: "hard",
      blooms_level: "Understand",
      tags: ["color-segmentation", "hsv", "rgb", "object-detection"]
    },
    {
      id: "q-ch03-06",
      question: "What does a depth image represent?",
      type: "multiple-choice",
      options: [
        "The color intensity at each pixel",
        "The distance from the camera to each pixel",
        "The gradient magnitude at each pixel",
        "The object class at each pixel"
      ],
      correctAnswer: 1,
      explanation: "A depth image is a 2D array where each pixel value represents the distance (in meters or millimeters) from the camera to that point in the scene. This enables 3D perception for obstacle detection and mapping.",
      wrongAnswerExplanations: [
        "Color intensity is stored in RGB or grayscale images, not depth images.",
        "Gradient magnitude comes from edge detection operators like Sobel, not depth cameras.",
        "Object class is the output of semantic segmentation, not depth sensing."
      ],
      difficulty: "easy",
      blooms_level: "Understand",
      tags: ["depth-perception", "3d-vision", "depth-cameras"]
    },
    {
      id: "q-ch03-07",
      question: "What is an occupancy grid used for in robotics?",
      type: "multiple-choice",
      options: [
        "Storing RGB color information",
        "Representing which cells in a top-down map are occupied vs. free",
        "Computing image gradients",
        "Tracking object motion across frames"
      ],
      correctAnswer: 1,
      explanation: "An occupancy grid is a top-down 2D map where each cell indicates whether that spatial region is occupied (obstacle) or free. It's essential for navigation and path planning.",
      wrongAnswerExplanations: [
        "RGB images store color information, not occupancy grids.",
        "Gradient operators (Sobel, etc.) compute image gradients, not occupancy grids.",
        "Object tracking uses centroid matching or Kalman filters, not occupancy grids."
      ],
      difficulty: "medium",
      blooms_level: "Understand",
      tags: ["occupancy-grid", "mapping", "navigation"]
    },
    {
      id: "q-ch03-08",
      question: "In nearest-neighbor object tracking, how do you match objects between two consecutive frames?",
      type: "multiple-choice",
      options: [
        "Match by color similarity",
        "Match each object in frame 1 to the nearest object in frame 2 by distance",
        "Match by object size",
        "Match by alphabetical object ID"
      ],
      correctAnswer: 1,
      explanation: "Nearest-neighbor tracking computes the Euclidean distance between each object centroid in frame 1 and all centroids in frame 2, then matches to the nearest one. This assumes objects move small distances between frames.",
      wrongAnswerExplanations: [
        "While color can help tracking, nearest-neighbor specifically refers to spatial distance matching.",
        "Size can change between frames (perspective, occlusion), so it's not reliable for nearest-neighbor tracking.",
        "Object IDs are assigned after matching, not used for matching."
      ],
      difficulty: "medium",
      blooms_level: "Apply",
      tags: ["object-tracking", "nearest-neighbor", "computer-vision"]
    },
    {
      id: "q-ch03-09",
      question: "What is visual servoing?",
      type: "multiple-choice",
      options: [
        "Using vision to detect objects",
        "Controlling a robot using visual feedback from cameras",
        "Processing images faster for real-time applications",
        "Training neural networks on visual data"
      ],
      correctAnswer: 1,
      explanation: "Visual servoing is a control technique where the robot uses real-time visual feedback (camera images) to guide its motion toward a target or along a desired path. It closes the loop between perception and action.",
      wrongAnswerExplanations: [
        "Object detection is a component of visual servoing but not the control strategy itself.",
        "Processing speed optimization is important but doesn't define visual servoing.",
        "Training neural networks is machine learning, not visual servoing (which can use classical or learning methods)."
      ],
      difficulty: "medium",
      blooms_level: "Understand",
      tags: ["visual-servoing", "control", "navigation"]
    },
    {
      id: "q-ch03-10",
      question: "In the bug algorithm for navigation, what does the robot do when it encounters an obstacle?",
      type: "multiple-choice",
      options: [
        "Stop and wait for human intervention",
        "Move toward the goal in a straight line",
        "Follow the obstacle boundary until a clear path to the goal is found",
        "Turn around and go back to the start"
      ],
      correctAnswer: 2,
      explanation: "The bug algorithm is a reactive navigation strategy: (1) move toward goal, (2) if obstacle encountered, follow its boundary, (3) when a clear path to the goal exists, resume direct approach. This simple algorithm guarantees goal reaching in most environments.",
      wrongAnswerExplanations: [
        "The bug algorithm is autonomous and doesn't require human intervention.",
        "Moving straight toward the goal only works when there are no obstacles; bug algorithm switches to boundary following when blocked.",
        "Going back to start defeats the purpose; bug algorithm makes progress by following boundaries."
      ],
      difficulty: "medium",
      blooms_level: "Understand",
      tags: ["bug-algorithm", "path-planning", "navigation"]
    }
  ]}
  questionsPerBatch={10}
/>

---

## Next Steps

Congratulations on completing Chapter 3! 🎉

**You've learned**:
- Image representation and processing
- Edge and corner detection
- Object detection and tracking
- Depth perception and obstacle detection
- Vision-based navigation

**What's Next**: [Chapter 4: Reinforcement Learning for Robotics](../chapter-04/index.md)

---

**Estimated completion time**: 20 minutes | **Passing score**: 70% (7/10 correct)
