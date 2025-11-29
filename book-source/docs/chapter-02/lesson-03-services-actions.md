---
title: "Lesson 2.3: ROS 2 Services and Actions"
description: "Use ROS 2 services for request-response and actions for long-running tasks"
chapter: 2
lesson: 3
estimated_time: 60
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
prerequisites: ["chapter-02-lesson-02"]
has_interactive_python: true
interactive_python_count: 3
has_try_with_ai: true
try_with_ai_count: 1
tags: ["ros2", "services", "actions", "request-response"]
---

# Lesson 2.3: ROS 2 Services and Actions

## 🎯 Learning Objectives

- Understand when to use services vs topics
- Implement ROS 2 service servers and clients
- Use actions for long-running tasks with feedback
- Handle service requests and action goals
- Build request-response communication patterns

**Time**: 60 minutes

---

## Prerequisites

<Prerequisites
  prerequisites={[
    {
      lessonId: "chapter-02-lesson-02",
      title: "Lesson 2.2: ROS 2 Nodes and Topics",
      link: "/docs/chapter-02/lesson-02-nodes-topics"
    }
  ]}
/>

## Introduction

While **topics** are great for streaming data, sometimes you need **synchronous request-response** communication. ROS 2 provides:
- **Services**: Synchronous request-response (like function calls)
- **Actions**: Asynchronous long-running tasks with feedback

**Use Cases**:
- Service: "Get current battery level" → immediate response
- Action: "Navigate to location" → progress updates, final result

---

## 1. ROS 2 Services

### Service vs Topic

| Aspect | Topic | Service |
|--------|-------|---------|
| Communication | Asynchronous | Synchronous |
| Pattern | Publish/Subscribe | Request/Response |
| Use Case | Streaming data | On-demand requests |
| Response | None | Immediate result |

### When to Use Services

- Need immediate response
- One-time requests
- State queries ("What's my position?")
- Configuration changes

---

## 2. Creating a Service Server

### Service Definition

First, define the service type (in `.srv` file):

```
# GetBatteryLevel.srv
float32 voltage
---
float32 percentage
bool success
```

### Service Server Implementation

```python
from example_interfaces.srv import AddTwoInts

class BatteryService(Node):
    def __init__(self):
        super().__init__('battery_service')
        self.srv = self.create_service(
            AddTwoInts,              # Service type
            'get_battery_level',     # Service name
            self.get_battery_callback
        )
        self.get_logger().info('Battery service ready')
    
    def get_battery_callback(self, request, response):
        # Process request
        voltage = request.a  # Example: voltage from request
        
        # Calculate response
        percentage = (voltage / 12.0) * 100.0
        success = voltage > 10.0
        
        # Set response
        response.sum = percentage
        response.success = success
        
        self.get_logger().info(f'Battery: {percentage:.1f}%')
        return response
```

---

## 3. Creating a Service Client

### Client Implementation

```python
from example_interfaces.srv import AddTwoInts

class BatteryClient(Node):
    def __init__(self):
        super().__init__('battery_client')
        self.client = self.create_client(
            AddTwoInts,
            'get_battery_level'
        )
    
    def get_battery(self):
        # Wait for service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        
        # Create request
        request = AddTwoInts.Request()
        request.a = 11.5  # Example voltage
        
        # Send request
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        # Get response
        response = future.result()
        if response:
            self.get_logger().info(f'Battery: {response.sum}%')
            return response.sum
        return None
```

---

## 4. ROS 2 Actions

### What are Actions?

**Actions** are for long-running tasks that:
- Take time to complete
- Need progress feedback
- Can be canceled
- May fail

**Examples**:
- Navigate to location (takes 30 seconds, show progress)
- Pick up object (show arm movement)
- Complex computation (show percentage complete)

### Action Structure

```
Goal → Feedback → Result
```

- **Goal**: What to do (e.g., "Navigate to (5, 3)")
- **Feedback**: Progress updates (e.g., "50% complete, at (2.5, 1.5)")
- **Result**: Final outcome (e.g., "Success" or "Failed")

---

## 5. Creating an Action Server

### Action Server Example

```python
from example_interfaces.action import Fibonacci

class NavigationActionServer(Node):
    def __init__(self):
        super().__init__('navigation_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,              # Action type
            'navigate_to_goal',    # Action name
            self.execute_callback
        )
    
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing navigation goal...')
        
        # Get goal
        goal = goal_handle.request
        
        # Execute task with feedback
        for i in range(goal.order):
            # Do work
            time.sleep(1)
            
            # Send feedback
            feedback_msg = Fibonacci.Feedback()
            feedback_msg.sequence = [0, 1, 1, 2, 3, 5][:i+1]
            goal_handle.publish_feedback(feedback_msg)
            
            # Check if canceled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return Fibonacci.Result()
        
        # Complete
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = [0, 1, 1, 2, 3, 5]
        return result
```

---

## 6. Creating an Action Client

### Action Client Example

```python
from example_interfaces.action import Fibonacci
from rclpy.action import ActionClient

class NavigationActionClient(Node):
    def __init__(self):
        super().__init__('navigation_action_client')
        self._action_client = ActionClient(
            self,
            Fibonacci,
            'navigate_to_goal'
        )
    
    def send_goal(self, target_order):
        # Wait for server
        self._action_client.wait_for_server()
        
        # Create goal
        goal_msg = Fibonacci.Goal()
        goal_msg.order = target_order
        
        # Send goal
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: {feedback.sequence}')
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        
        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')
        rclpy.shutdown()
```

---

## 7. Exercises

### Exercise 2.3.1: Service Request-Response

Implement a simple service that adds two numbers.

<InteractivePython
  id="ex-2-3-1"
  title="Service Request-Response"
  starterCode={`class AddService:
    """Simulate a ROS 2 service that adds two numbers."""
    
    def __init__(self):
        self.service_name = "add_two_ints"
    
    def handle_request(self, a, b):
        """
        Handle service request.
        
        Args:
            a: First number
            b: Second number
        
        Returns:
            Sum of a and b
        """
        # TODO: Return sum
        pass

# Test
service = AddService()
result = service.handle_request(5, 3)
print(f"Service result: {result}")
assert result == 8, "Service should return 8"
`}
  hints={[
    "Simply return a + b",
    "Service processes request and returns response"
  ]}
/>

---

### Exercise 2.3.2: Action Progress Tracking

Simulate an action that provides progress feedback.

<InteractivePython
  id="ex-2-3-2"
  title="Action with Feedback"
  starterCode={`class NavigationAction:
    """Simulate a navigation action with progress feedback."""
    
    def __init__(self):
        self.progress_updates = []
    
    def execute_navigation(self, target_distance, steps=10):
        """
        Execute navigation with progress updates.
        
        Args:
            target_distance: Total distance to travel
            steps: Number of progress updates
        
        Returns:
            List of progress percentages
        """
        # TODO: Generate progress updates from 0% to 100%
        # Store each progress percentage in self.progress_updates
        pass

# Test
action = NavigationAction()
action.execute_navigation(10.0, steps=5)
print("Progress updates:", action.progress_updates)
# Should show: [0, 25, 50, 75, 100] or similar
`}
  hints={[
    "Calculate progress percentage for each step",
    "Progress goes from 0% to 100%",
    "Store percentages in self.progress_updates"
  ]}
/>

---

### Exercise 2.3.3: Service vs Action Selection

Choose between service and action for different scenarios.

<InteractivePython
  id="ex-2-3-3"
  title="Service vs Action Selection"
  starterCode={`def choose_communication_type(scenario):
    """
    Choose 'service' or 'action' for each scenario.
    
    Scenarios:
    - "get_battery_level": Quick status check
    - "navigate_to_location": Takes 30 seconds, need progress
    - "calculate_path": Quick computation, immediate result
    - "pick_up_object": Takes 10 seconds, show arm movement
    - "get_robot_pose": Quick position query
    """
    # TODO: Return 'service' or 'action' for each
    pass

# Test
scenarios = [
    "get_battery_level",
    "navigate_to_location",
    "calculate_path",
    "pick_up_object",
    "get_robot_pose"
]

for scenario in scenarios:
    comm_type = choose_communication_type(scenario)
    print(f"{scenario}: {comm_type}")
`}
  hints={[
    "Service: Quick, immediate response",
    "Action: Long-running, needs feedback",
    "Navigation and manipulation usually need actions"
  ]}
/>

---

## 8. Try With AI

### TryWithAI 2.3.1: Design Service Interface

<TryWithAI
  id="tryai-2-3-1"
  title="Design a Map Service"
  role="Copilot"
  scenario="You need a service that returns map data when requested"
  yourTask="Design a service interface for getting map data. What should the request contain? What should the response contain? Write the service definition."
  aiPromptTemplate="I'm designing a map service. Here's my service definition: [your definition]. Can you review it and suggest improvements? Also help me understand when I should use a service vs publishing map data on a topic."
  successCriteria={[
    "You understand service request/response structure",
    "You can design appropriate service interface",
    "You understand when to use services vs topics"
  ]}
  reflectionQuestions={[
    "What information does the client need to request?",
    "What data should the service return?",
    "Why use a service instead of a topic for map data?"
  ]}
/>

---

## Summary

**Key Takeaways**:

1. **Services**
   - Synchronous request-response
   - Use for quick queries and one-time requests
   - Server handles requests, client sends them
   - Immediate response required

2. **Actions**
   - Asynchronous long-running tasks
   - Provide progress feedback
   - Can be canceled
   - Use for navigation, manipulation, complex tasks

3. **When to Use What**
   - **Topic**: Streaming data, continuous updates
   - **Service**: Quick queries, immediate response needed
   - **Action**: Long tasks, progress feedback needed

**What's Next**: [Lesson 2.4: Building ROS 2 Packages with Python (rclpy)](./lesson-04-rclpy-packages.md) teaches you to create complete ROS 2 packages.

---

**Estimated completion time**: 60 minutes | **Prerequisites**: Lesson 2.2 | **Difficulty**: B1+ (Upper Intermediate)

