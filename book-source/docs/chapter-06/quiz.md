---
title: "Chapter 6 Quiz"
description: "Test your understanding of Humanoid Robot Design and Control"
sidebar_position: 6
---

# Chapter 6 Quiz: Humanoid Robot Design and Control

Test your understanding of the concepts covered in Chapter 6. This quiz covers all five lessons.

**Instructions**:
- 10 questions total
- Multiple choice format
- Immediate feedback provided
- You can retake the quiz as many times as you want

<Quiz
  chapterId={6}
  questions={[
    {
      id: "q-ch06-01",
      question: "What is the main challenge in humanoid robot kinematics compared to fixed-base robots?",
      type: "multiple-choice",
      options: [
        "Humanoids have fewer degrees of freedom",
        "Humanoids are underactuated and must maintain balance while moving",
        "Humanoids can't use inverse kinematics",
        "Humanoids don't have joints"
      ],
      correctAnswer: 1,
      explanation: "Humanoid robots are underactuated (fewer actuators than degrees of freedom) and must maintain balance while moving. This requires whole-body coordination and center-of-mass control, unlike fixed-base robots that are fully actuated.",
      wrongAnswerExplanations: [
        "Humanoids typically have many degrees of freedom (20+)",
        "Inverse kinematics is essential for humanoids",
        "Humanoids have many joints (shoulders, elbows, hips, knees, etc.)"
      ]
    },
    {
      id: "q-ch06-02",
      question: "What is the Zero Moment Point (ZMP) in bipedal locomotion?",
      type: "multiple-choice",
      options: [
        "The point where all forces sum to zero",
        "The point on the ground where the net moment is zero, indicating stability",
        "The center of mass of the robot",
        "The point where the robot's feet touch the ground"
      ],
      correctAnswer: 1,
      explanation: "The Zero Moment Point (ZMP) is the point on the ground plane where the net moment of all forces (gravity, inertia, ground reaction) is zero. If the ZMP stays within the support polygon (foot contact area), the robot is stable.",
      wrongAnswerExplanations: [
        "ZMP is about moments, not just forces",
        "ZMP is on the ground, not at the center of mass",
        "ZMP is a calculated point, not just foot contact"
      ]
    },
    {
      id: "q-ch06-03",
      question: "What is the inverted pendulum model used for in humanoid balance control?",
      type: "multiple-choice",
      options: [
        "To model the robot's arms",
        "To simplify balance dynamics by modeling the body as a point mass on a massless leg",
        "To calculate joint torques directly",
        "To design the robot's appearance"
      ],
      correctAnswer: 1,
      explanation: "The inverted pendulum model simplifies humanoid balance by treating the robot as a point mass (center of mass) on a massless leg. This captures the essential balance dynamics while being computationally tractable for real-time control.",
      wrongAnswerExplanations: [
        "The model is for balance, not arm modeling",
        "Joint torques are calculated from the model, not directly",
        "The model is for dynamics, not appearance"
      ]
    },
    {
      id: "q-ch06-04",
      question: "What is whole-body motion planning for humanoids?",
      type: "multiple-choice",
      options: [
        "Planning only arm movements",
        "Coordinating all body parts (arms, legs, torso) while respecting balance and center-of-mass constraints",
        "Planning only leg movements",
        "Planning without considering balance"
      ],
      correctAnswer: 1,
      explanation: "Whole-body motion planning coordinates all body parts (arms, legs, torso, head) simultaneously while ensuring the center of mass stays within the support polygon for balance. This is essential for complex tasks like reaching while walking.",
      wrongAnswerExplanations: [
        "Whole-body includes all parts, not just arms",
        "Whole-body includes all parts, not just legs",
        "Balance constraints are critical for humanoids"
      ]
    },
    {
      id: "q-ch06-05",
      question: "What is impedance control used for in humanoid manipulation?",
      type: "multiple-choice",
      options: [
        "To increase robot stiffness",
        "To make the robot compliant and safe for physical interaction with humans and objects",
        "To reduce power consumption",
        "To simplify control"
      ],
      correctAnswer: 1,
      explanation: "Impedance control regulates the relationship between force and motion, making the robot compliant (soft) rather than rigid. This enables safe physical interaction, natural object manipulation, and reduces injury risk in human-robot contact.",
      wrongAnswerExplanations: [
        "Impedance control can make robots more or less stiff depending on parameters",
        "Compliance may increase power usage, not reduce it",
        "Impedance control adds complexity but improves safety"
      ]
    },
    {
      id: "q-ch06-06",
      question: "What makes grasping with humanoid hands challenging?",
      type: "multiple-choice",
      options: [
        "Humanoid hands are too simple",
        "High degrees of freedom, object shape uncertainty, and force control requirements",
        "Humanoid hands can't grasp anything",
        "Grasping is always easy"
      ],
      correctAnswer: 1,
      explanation: "Humanoid hands have many degrees of freedom (20+ joints), objects have unknown/complex shapes, and grasping requires precise force control to avoid crushing or dropping objects. This makes grasping a complex manipulation problem.",
      wrongAnswerExplanations: [
        "Humanoid hands are complex, not simple",
        "Humanoid hands can grasp, but it's challenging",
        "Grasping is a complex problem requiring careful control"
      ]
    },
    {
      id: "q-ch06-07",
      question: "What is proxemics in human-robot interaction?",
      type: "multiple-choice",
      options: [
        "Robot speed control",
        "The study of personal space and appropriate distances in social interaction",
        "Robot power management",
        "Joint angle limits"
      ],
      correctAnswer: 1,
      explanation: "Proxemics studies how people use space in social interactions (intimate, personal, social, public distances). For humanoid robots, respecting appropriate distances makes interactions feel natural and comfortable for humans.",
      wrongAnswerExplanations: [
        "Proxemics is about space, not speed",
        "Proxemics is about social space, not power",
        "Proxemics is about interaction distance, not joint limits"
      ]
    },
    {
      id: "q-ch06-08",
      question: "Why is center-of-mass control critical for humanoid robots?",
      type: "multiple-choice",
      options: [
        "It's not important",
        "The center of mass must stay within the support polygon to maintain balance",
        "It only matters for arm movements",
        "It only matters when standing still"
      ],
      correctAnswer: 1,
      explanation: "For a humanoid to remain balanced, its center of mass projection must stay within the support polygon (area of foot contact with ground). This is critical during all movements - walking, reaching, turning - not just when stationary.",
      wrongAnswerExplanations: [
        "Center-of-mass control is essential for balance",
        "It affects the entire body, not just arms",
        "It's critical during all movements, not just when still"
      ]
    },
    {
      id: "q-ch06-09",
      question: "What is a walking gait in bipedal locomotion?",
      type: "multiple-choice",
      options: [
        "The robot's walking speed",
        "A repeating pattern of leg movements (stance and swing phases) that creates forward motion",
        "The robot's height",
        "The number of joints in the legs"
      ],
      correctAnswer: 1,
      explanation: "A walking gait is a cyclic pattern of leg movements: stance phase (foot on ground, supporting weight) and swing phase (foot lifted, moving forward). The pattern repeats to create continuous forward motion while maintaining balance.",
      wrongAnswerExplanations: [
        "Gait is the pattern, not the speed",
        "Gait is the movement pattern, not height",
        "Gait is the movement sequence, not joint count"
      ]
    },
    {
      id: "q-ch06-10",
      question: "What makes human-robot interaction 'natural'?",
      type: "multiple-choice",
      options: [
        "Using only voice commands",
        "Combining multiple modalities (speech, gesture, vision) and respecting social norms like proxemics",
        "Always moving fast",
        "Never making eye contact"
      ],
      correctAnswer: 1,
      explanation: "Natural human-robot interaction uses multiple communication channels (speech, gestures, facial expressions, body language) and respects social norms (appropriate distances, turn-taking, gaze). This mimics human-human interaction patterns.",
      wrongAnswerExplanations: [
        "Natural interaction uses multiple modalities, not just voice",
        "Natural interaction adapts speed to context, not always fast",
        "Eye contact (gaze) is important for natural interaction"
      ]
    }
  ]}
  questionsPerBatch={10}
/>

---

## Quiz Results Interpretation

- **8-10 correct**: Excellent! You have a strong grasp of humanoid robotics
- **6-7 correct**: Good work! Review the lessons for concepts you missed
- **4-5 correct**: Fair understanding. Revisit the lesson content and exercises
- **0-3 correct**: Review all five lessons carefully before proceeding to Chapter 7

**Next Steps**: Once you're comfortable with humanoid concepts, proceed to **[Chapter 7: Vision-Language-Action (VLA)](/docs/chapter-07/index.md)** to learn how to integrate LLMs with humanoid robots!

