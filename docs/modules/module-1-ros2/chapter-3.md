---
title: "Chapter 3: Communication Patterns (Topics, Services, Actions)"
description: "Understanding the three primary communication patterns in ROS 2: topics, services, and actions"
keywords: [ROS 2, topics, services, actions, communication patterns, message passing]
sidebar_position: 4
---

# Chapter 3: Communication Patterns (Topics, Services, Actions)

## Learning Objectives
By the end of this chapter, you will be able to:
1. Implement publishers and subscribers using topics for asynchronous communication
2. Create and use services for synchronous request-response communication
3. Design and implement actions for long-running tasks with feedback
4. Apply Quality of Service (QoS) settings appropriately for different communication needs
5. Choose the appropriate communication pattern for different robotic scenarios

## Prerequisites
Before starting this chapter, you should have:
- Understanding of ROS 2 fundamentals and nodes (Chapter 1)
- Knowledge of creating and managing ROS 2 packages (Chapter 2)
- Basic understanding of asynchronous vs synchronous communication

## Core Concepts

### Topics - Publish/Subscribe Pattern
Topics enable asynchronous, one-way communication between nodes. Publishers send messages to topics, and subscribers receive messages from topics. This pattern is ideal for continuous data streams like sensor readings, robot states, or control commands.

**Key Features:**
- Asynchronous communication
- Multiple publishers/subscribers per topic
- Message buffering with configurable history
- Quality of Service (QoS) settings for reliability

### Services - Request/Response Pattern
Services provide synchronous, two-way communication. A client sends a request to a service server, which processes the request and returns a response. This pattern is suitable for operations that require immediate responses, like configuration changes or simple computations.

**Key Features:**
- Synchronous communication
- Request-response model
- One client per service call
- Error handling and timeouts

### Actions - Long-Running Tasks with Feedback
Actions combine features of topics and services to handle long-running tasks. They allow clients to send goals to action servers, receive feedback during execution, and get results when complete. This pattern is perfect for navigation, manipulation, or calibration tasks.

**Key Features:**
- Goal-feedback-result pattern
- Cancelable operations
- Asynchronous execution
- Continuous feedback

## Implementation

### Using Topics with Custom QoS Settings
Implementing a publisher with specific QoS requirements:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

class QoSPublisher(Node):

    def __init__(self):
        super().__init__('qos_publisher')
        
        # Create a QoS profile with specific settings
        qos_profile = QoSProfile(
            depth=10,  # Keep the 10 most recent messages
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,  # Ensure all messages are received
            durability=QoSDurabilityPolicy.VOLATILE  # Transient messages
        )
        
        self.publisher = self.create_publisher(String, 'qos_topic', qos_profile)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello QoS: {self.i}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    qos_publisher = QoSPublisher()
    rclpy.spin(qos_publisher)
    qos_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Creating a Service Server
Implementing a service for simple calculations:

```python
from rclpy.node import Node
from rclpy.qos import QoSProfile
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    minimal_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Examples

### Example: Action Server for Navigation
Implementing a navigation action that provides feedback during execution:

```python
import time
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rcl_interfaces.action import Fibonacci

class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_callback,
            callback_group=rclpy.callback_groups.ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]
        
        for i in range(1, goal_handle.request.order):
            # Check if there's a cancel request
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()
            
            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1])
            
            self.get_logger().info(f'Publishing feedback: {feedback_msg.sequence}')
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)
        
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        self.get_logger().info(f'Returning result: {result.sequence}')
        return result

def main(args=None):
    rclpy.init(args=args)
    action_server = FibonacciActionServer()
    
    try:
        rclpy.spin(action_server)
    except KeyboardInterrupt:
        pass
    finally:
        action_server.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example: Action Client for Navigation
A client that sends navigation goals and receives feedback:

```python
import time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rcl_interfaces.action import Fibonacci

class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(
            self,
            Fibonacci,
            'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()
        self.get_logger().info('Sending goal request...')

        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.sequence}')

def main(args=None):
    rclpy.init(args=args)
    action_client = FibonacciActionClient()
    action_client.send_goal(10)
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
```

## Summary
In this chapter, we explored the three primary communication patterns in ROS 2: topics, services, and actions. Each pattern serves different communication needs:

- **Topics** for continuous, asynchronous data streams
- **Services** for simple request-response interactions
- **Actions** for long-running operations with feedback

Quality of Service (QoS) settings allow fine-tuning communication behavior to meet specific requirements for reliability, durability, and history. Understanding when to use each communication pattern is crucial for designing robust and efficient robotic systems.

## Exercises

### Logical Analysis Exercise
1. Analyze when you would use each communication pattern for different robotic components (e.g., sensors, controllers, navigation).
2. Explain how QoS settings impact system reliability and performance in safety-critical robotic applications.

### Conceptual Exploration Exercise
1. Research and compare the QoS policies in DDS and their impact on robotic communication.
2. Investigate how actions handle network failures and reconnections compared to services.

### Implementation Practice Exercise
1. Create a sensor node that publishes readings via topics with appropriate QoS settings for real-time performance.
2. Implement a configuration service that allows other nodes to update operating parameters.
3. Design an action server for robot navigation that includes progress feedback and cancellation capability.
4. Create a scenario where a single node uses all three communication patterns to interact with different systems.

## References
1. ROS 2 Communication Patterns: https://docs.ros.org/en/rolling/Concepts/About-Topics-Services-Actions.html
2. QoS in ROS 2: https://docs.ros.org/en/rolling/Concepts/About-Quality-of-Service-Settings.html
3. Actions Tutorial: https://docs.ros.org/en/rolling/Tutorials/Actions/Understanding-ROS2-Actions.html