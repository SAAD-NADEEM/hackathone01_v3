---
title: "Chapter 1: Introduction to ROS 2 Architecture"
description: "Understanding the fundamental architecture of Robot Operating System 2"
keywords: [ROS 2, architecture, nodes, topics, services, middleware]
sidebar_position: 1
---

# Chapter 1: Introduction to ROS 2 Architecture

## Learning Objectives

By the end of this chapter, students will be able to:
1. Explain the fundamental components of ROS 2 architecture
2. Identify the differences between ROS 1 and ROS 2
3. Describe the role of DDS (Data Distribution Service) in ROS 2
4. Understand the concept of nodes, packages, and workspaces

## Prerequisites

Before starting this chapter, students should have:
- Basic understanding of robotics concepts
- Programming experience in Python or C++
- Familiarity with command-line interfaces

## Core Concepts

### What is ROS 2?

Robot Operating System 2 (ROS 2) is an open-source framework for writing robot software. It's not an operating system, but rather a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.

### Architecture Overview

ROS 2 is built on the Data Distribution Service (DDS) standard, which provides a middleware for enabling scalable, real-time, dependable, distributed data exchanges. This architecture allows ROS 2 to support:

- Multi-robot systems
- Real-time systems
- Systems requiring security
- Cross-platform compatibility

### Key Architecture Components

1. **Nodes**: Basic compute units that perform processing
2. **Topics**: Named buses over which nodes exchange messages
3. **Services**: Synchronous request/response communication
4. **Actions**: Asynchronous goal-oriented communication
5. **Parameters**: Configuration values accessible to nodes

## Implementation

Let's look at a basic ROS 2 node implementation:

```python
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
```

**What this code does**: Creates a simple publisher node that publishes "Hello World" messages to a topic.
**Why this works**: The node uses the ROS 2 client library to handle communication with the ROS 2 middleware.

## Examples

### Example 1: Creating a Simple Node
- Demonstrates basic node structure
- Shows how to initialize a ROS 2 node
- Illustrates the event loop

### Example 2: Publisher and Subscriber
- Shows bidirectional communication
- Demonstrates the publisher-subscriber pattern
- Explains message synchronization

## Summary

ROS 2 provides a robust architecture for building complex robotic systems. Its DDS-based design enables scalability, real-time performance, and security features that were lacking in ROS 1. Understanding this architecture is fundamental to developing advanced robotic applications.

## Exercises

### Logical Exercise
Explain why separating the concept of nodes, topics, and services improves the modularity of robotic systems.

### Conceptual Exercise
Compare and contrast the architectural differences between ROS 1 and ROS 2, focusing on communication patterns.

### Implementation Exercise
Create a simple ROS 2 node that publishes the current timestamp to a topic every second.