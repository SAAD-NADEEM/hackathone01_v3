---
title: "Chapter 4: Advanced Communication Patterns"
description: "Advanced ROS 2 communication patterns: actions, parameters, and latching"
keywords: [ROS 2, actions, parameters, latching, advanced communication]
sidebar_position: 4
---

# Chapter 4: Advanced Communication Patterns

## Learning Objectives

By the end of this chapter, students will be able to:
1. Implement actions for goal-oriented communication
2. Use parameters effectively for system configuration
3. Understand and apply latching and transient local QoS
4. Design complex communication architectures

## Prerequisites

Before starting this chapter, students should have:
- Understanding of basic ROS 2 communication (Chapters 1-2)
- Experience with ROS 2 tools (Chapter 3)
- Intermediate programming skills

## Core Concepts

### Actions

Actions are used for long-running tasks that have feedback and can be canceled. They follow a three-part pattern: goal, feedback, and result. Actions are ideal for operations like navigation, where you need to know the progress and be able to interrupt.

### Parameters

Parameters provide a way to configure nodes at runtime. They're especially useful for settings that might need to change without restarting the node. Parameters can be set at launch time or during runtime.

### Quality of Service (QoS)

QoS settings allow fine-tuning of communication behavior. For example:
- Reliability: Can messages be dropped or must they be guaranteed?
- Durability: Do late-joining subscribers receive old messages?
- History: How many messages should be kept?

## Implementation

Example of an action server implementation:

```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]
        
        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()
                
            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1])
            
            goal_handle.publish_feedback(feedback_msg)
        
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        return result
```

**What this code does**: Implements a Fibonacci sequence generator as an action that provides feedback during execution and can be canceled.
**Why this works**: The action server follows the ROS 2 action protocol, handling goals, providing feedback, and returning results.

## Examples

### Example 1: Navigation Action
- Implements robot navigation with progress feedback
- Demonstrates goal cancellation capability
- Shows how actions are better than services for long-running tasks

### Example 2: Parameter Server
- Creates a node that serves as a configuration manager
- Shows dynamic parameter updates
- Illustrates parameter validation

## Summary

Advanced communication patterns in ROS 2 provide the tools needed for complex robotic applications. Actions are important for long-running tasks with feedback, parameters enable runtime configuration, and QoS settings allow for performance optimization. Understanding when and how to use these patterns is key to designing robust robotic systems.

## Exercises

### Logical Exercise
Analyze when to use actions vs services vs topics for a given robotic task.

### Conceptual Exercise
Design a parameter management system for a complex robot with multiple subsystems.

### Implementation Exercise
Create an action server that simulates a robot arm movement with progress feedback.