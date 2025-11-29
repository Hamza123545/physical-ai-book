---
title: "Chapter 5 Quiz"
description: "Test your understanding of motion planning and control"
sidebar_position: 6
---

# Chapter 5 Quiz: Motion Planning and Control

Test your understanding of the concepts covered in Chapter 5. This quiz covers all five lessons.

**Instructions**:
- 10 questions total (2 per lesson)
- Multiple choice format
- Immediate feedback provided
- You can retake the quiz as many times as you want

<Quiz
  chapterId={5}
  questions={[
    {
      id: "q-ch05-01",
      question: "What is the main advantage of A* over Dijkstra's algorithm for pathfinding?",
      type: "multiple-choice",
      options: [
        "A* always finds paths faster by using a heuristic to guide search",
        "A* uses less memory than Dijkstra",
        "A* can find paths in 3D spaces while Dijkstra cannot",
        "A* doesn't require a goal position"
      ],
      correctAnswer: 0,
      explanation: "A* combines cost-so-far (like Dijkstra) with a heuristic estimate of remaining cost, guiding search toward the goal more efficiently. With an admissible heuristic, A* remains optimal while exploring fewer nodes.",
      wrongAnswerExplanations: [
        "Memory usage is similar between A* and Dijkstra",
        "Both algorithms can work in any dimensionality",
        "A* requires a goal to compute the heuristic"
      ]
    },
    {
      id: "q-ch05-02",
      question: "What does the Manhattan distance heuristic compute?",
      type: "multiple-choice",
      options: [
        "The straight-line (Euclidean) distance between two points",
        "The sum of absolute differences in coordinates: |x1-x2| + |y1-y2|",
        "The number of diagonal moves needed",
        "The shortest obstacle-free path length"
      ],
      correctAnswer: 1,
      explanation: "Manhattan distance (also called L1 distance or taxicab distance) sums the absolute differences in each coordinate. It's admissible for 4-connected grids and represents the minimum moves needed without diagonals.",
      wrongAnswerExplanations: [
        "Straight-line distance is Euclidean distance, not Manhattan",
        "Diagonal moves use Euclidean or Chebyshev distance",
        "Manhattan distance is a heuristic estimate, not the actual path length"
      ]
    },
    {
      id: "q-ch05-03",
      question: "Why do sampling-based methods (RRT, PRM) avoid the curse of dimensionality?",
      type: "multiple-choice",
      options: [
        "They use compression algorithms to reduce dimensionality",
        "They randomly sample configurations instead of discretizing the entire space",
        "They only work in 2D where dimensionality isn't a problem",
        "They use neural networks to learn low-dimensional representations"
      ],
      correctAnswer: 1,
      explanation: "Sampling-based methods randomly sample configurations from the configuration space, avoiding the need to discretize all dimensions into a grid. This prevents the exponential growth in cells (r^n) that makes grid-based methods infeasible for high-dimensional robots.",
      wrongAnswerExplanations: [
        "No compression is used; they simply avoid full discretization",
        "Sampling methods work in any dimensionality, especially high dimensions",
        "They don't use neural networks (though learning can enhance sampling)"
      ]
    },
    {
      id: "q-ch05-04",
      question: "What is the key difference between PRM and RRT?",
      type: "multiple-choice",
      options: [
        "PRM builds a roadmap once for multiple queries; RRT plans for a single query",
        "PRM only works in 2D; RRT works in any dimension",
        "PRM is faster but less accurate than RRT",
        "PRM requires obstacles to be known; RRT doesn't"
      ],
      correctAnswer: 0,
      explanation: "PRM (Probabilistic Roadmap) has a learning phase that builds a reusable roadmap for multiple start/goal queries. RRT (Rapidly-exploring Random Tree) is a single-query planner that builds a tree from start to goal each time. This makes PRM better for static environments with repeated queries, and RRT better for dynamic environments.",
      wrongAnswerExplanations: [
        "Both work in any dimensionality",
        "Both have similar accuracy; optimality depends on variants (PRM*, RRT*)",
        "Both require obstacle information for collision checking"
      ]
    },
    {
      id: "q-ch05-05",
      question: "What is the key difference between a path and a trajectory?",
      type: "multiple-choice",
      options: [
        "A path is 3D while a trajectory is 2D",
        "A path is geometric waypoints; a trajectory adds time parameterization",
        "A path is shorter than a trajectory",
        "A path avoids obstacles; a trajectory doesn't"
      ],
      correctAnswer: 1,
      explanation: "A path is a geometric sequence of waypoints (positions) without timing information. A trajectory is time-parameterized: q(t) specifies position as a function of time, including velocity and acceleration profiles. Robots execute trajectories, not just paths.",
      wrongAnswerExplanations: [
        "Both can exist in any spatial dimension",
        "Length is not the distinguishing factor; time parameterization is",
        "Both should avoid obstacles; the difference is time information"
      ]
    },
    {
      id: "q-ch05-06",
      question: "Why is a quintic polynomial trajectory smoother than a cubic polynomial?",
      type: "multiple-choice",
      options: [
        "Quintic uses 5 waypoints while cubic uses 3",
        "Quintic constrains acceleration at boundaries (zero jerk); cubic doesn't",
        "Quintic is faster to compute",
        "Quintic takes less time to execute"
      ],
      correctAnswer: 1,
      explanation: "Quintic (5th degree) polynomials have 6 coefficients, allowing constraints on position, velocity, AND acceleration at start/end. This results in zero jerk at boundaries. Cubic polynomials only constrain position and velocity, leading to discontinuous acceleration (infinite jerk) at boundaries.",
      wrongAnswerExplanations: [
        "The degree refers to polynomial order, not number of waypoints",
        "Quintic requires more computation due to higher degree",
        "Execution time depends on the time duration chosen, not polynomial degree"
      ]
    },
    {
      id: "q-ch05-07",
      question: "In a PID controller, what does the integral term (I) primarily address?",
      type: "multiple-choice",
      options: [
        "Reduces overshoot by dampening oscillations",
        "Eliminates steady-state error by accumulating past errors",
        "Speeds up initial response to errors",
        "Filters sensor noise"
      ],
      correctAnswer: 1,
      explanation: "The integral term accumulates error over time and applies a correction proportional to this accumulated error. This eliminates steady-state offset that the proportional term alone cannot remove (e.g., constant disturbance like friction or gravity).",
      wrongAnswerExplanations: [
        "Reducing overshoot is the derivative (D) term's role",
        "Fast initial response comes from the proportional (P) term",
        "Noise filtering is typically done separately; derivative term is sensitive to noise"
      ]
    },
    {
      id: "q-ch05-08",
      question: "What happens if the proportional gain (Kp) in a PID controller is too high?",
      type: "multiple-choice",
      options: [
        "The system responds too slowly",
        "The system oscillates or becomes unstable",
        "Steady-state error increases",
        "The controller stops working"
      ],
      correctAnswer: 1,
      explanation: "Excessive proportional gain causes the system to overreact to errors, leading to overshoot. The system then overcorrects in the opposite direction, causing oscillations. If Kp is high enough, the system becomes unstable.",
      wrongAnswerExplanations: [
        "High Kp makes the system respond faster, not slower",
        "High Kp reduces (not increases) steady-state error for the P term alone",
        "High Kp causes instability but doesn't stop the controller from functioning"
      ]
    },
    {
      id: "q-ch05-09",
      question: "What is the 'receding horizon' principle in MPC?",
      type: "multiple-choice",
      options: [
        "The robot's view of the environment gradually shrinks",
        "Optimize over N future steps, apply first control, then re-optimize at next time step",
        "The optimization horizon increases as the robot gets closer to the goal",
        "Past control inputs are forgotten after N time steps"
      ],
      correctAnswer: 1,
      explanation: "MPC optimizes a control sequence over a fixed future horizon (N steps), but only applies the first control action. At the next time step, the horizon 'recedes' (shifts forward in time) and optimization is performed again. This allows MPC to continuously adapt to new information.",
      wrongAnswerExplanations: [
        "The receding horizon refers to the optimization window, not sensor range",
        "The horizon length N typically remains constant, not increasing",
        "While MPC focuses on the future, the principle is about re-optimization, not forgetting the past"
      ]
    },
    {
      id: "q-ch05-10",
      question: "What is the main advantage of MPC over PID control?",
      type: "multiple-choice",
      options: [
        "MPC is always faster to compute",
        "MPC doesn't require tuning",
        "MPC naturally handles constraints (e.g., max velocity, actuator limits)",
        "MPC works without a system model"
      ],
      correctAnswer: 2,
      explanation: "MPC formulates control as an optimization problem, where constraints (on states, controls, outputs) are naturally incorporated. PID has no built-in mechanism for constraint handling. This makes MPC ideal for systems with hard limits on actuators, safety boundaries, or obstacle avoidance.",
      wrongAnswerExplanations: [
        "MPC requires solving an optimization problem, making it computationally slower than PID",
        "MPC requires tuning cost weights (Q, R) and horizon length N",
        "MPC explicitly requires a system model for prediction; PID doesn't"
      ]
    }
  ]}
  questionsPerBatch={10}
/>

---

## Quiz Results Interpretation

- **9-10 correct**: Excellent! You have mastered motion planning and control concepts
- **7-8 correct**: Very good! Review the lessons for concepts you missed
- **5-6 correct**: Fair understanding. Revisit lesson content and exercises
- **0-4 correct**: Review all five lessons carefully before proceeding

**Next Steps**: Once comfortable with Chapter 5, proceed to **[Chapter 6: Computer Vision for Robotics](/docs/chapter-06/index.md)** to learn perception for navigation and manipulation!
