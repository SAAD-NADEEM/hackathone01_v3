---
title: "Chapter 2: Nodes, Topics, and Services"
description: "Deep dive into ROS 2 communication patterns: nodes, topics, and services"
keywords: [ROS 2, nodes, topics, services, communication patterns, messaging]
sidebar_position: 2
---

# Chapter 2: Nodes, Topics, and Services

## Learning Objectives

By the end of this chapter, students will be able to:
1. Implement nodes that communicate via topics (publishers and subscribers)
2. Create services for request/response communication
3. Design appropriate communication patterns for different scenarios
4. Debug communication issues between nodes

## Prerequisites

Before starting this chapter, students should have:
- Understanding of ROS 2 architecture fundamentals (Chapter 1)
- Programming experience in Python and/or C++
- Basic command-line proficiency

## Core Concepts

### Nodes

Nodes are the fundamental building blocks of ROS 2 applications. Each node is an executable that uses ROS 2 to communicate with other nodes. Nodes can publish or subscribe to topics, provide or use services, and interact with parameters.

### Topics and Publish-Subscribe Pattern

Topics enable asynchronous communication between nodes through a publish-subscribe pattern. Publishers send messages to a topic, and subscribers receive messages from topics. This pattern decouples publishers and subscribers in time and space.

### Services and Request-Response Pattern

Services provide synchronous request-response communication. A client sends a request to a service, and the service processes it and returns a response. This is useful for operations that require immediate feedback.

## Implementation

Example of publisher-subscriber pattern:

```python
# Publisher
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher = self.create_publisher(String, 'chatter', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World'
        self.publisher.publish(msg)

# Subscriber
class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String, 'chatter', self.listener_callback, 10)

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: {msg.data}')
```

**What this code does**: Establishes a communication channel between two nodes using the publish-subscribe pattern.
**Why this works**: The publisher sends messages to the 'chatter' topic, and the subscriber receives them without direct coupling.

## Examples

### Example 1: Simple Publisher-Subscriber
- Creates a publisher that sends sensor data
- Implements a subscriber that processes the data
- Demonstrates message types and callbacks

### Example 2: Service Client and Server
- Shows request-response communication
- Implements a service that performs calculations
- Illustrates synchronous communication patterns

## Summary

Understanding nodes, topics, and services is crucial for designing effective ROS 2 applications. The publish-subscribe pattern works well for streaming data, while services are better for request-response interactions. Choosing the right pattern for your use case is key to developing robust robotic systems.

## Exercises

### Logical Exercise
Analyze when you would choose topics over services for communication between two robot components.

### Conceptual Exercise
Design a communication architecture for a robot with sensors, actuators, and planning components using appropriate patterns.

### Implementation Exercise
Create a service that takes two numbers as input and returns their sum, then create a client to test it.