---
title: "Chapter 3: ROS 2 Tools and Debugging"
description: "Using ROS 2 tools for development, debugging, and visualization"
keywords: [ROS 2, tools, debugging, visualization, rqt, ros2cli]
sidebar_position: 3
---

# Chapter 3: ROS 2 Tools and Debugging

## Learning Objectives

By the end of this chapter, students will be able to:
1. Use ROS 2 command-line tools for system introspection
2. Utilize visualization tools like rqt for debugging
3. Monitor and analyze node communication
4. Troubleshoot common ROS 2 issues

## Prerequisites

Before starting this chapter, students should have:
- ROS 2 architecture knowledge (Chapter 1)
- Node, topic, and service understanding (Chapter 2)
- Command-line interface proficiency

## Core Concepts

### ROS 2 Command-Line Interface (CLI)

ROS 2 provides a comprehensive set of command-line tools for interacting with and debugging ROS systems:

- `ros2 run`: Run executables in packages
- `ros2 node`: List and info about nodes
- `ros2 topic`: List and info about topics
- `ros2 service`: List and info about services
- `ros2 param`: Get, set, and list parameters
- `ros2 action`: List and info about actions

### Visualization Tools

rqt is a Qt-based framework for GUI plugins in ROS. It provides various tools for visualizing and debugging ROS systems including:
- Topic monitoring
- Node graph visualization
- Message inspection
- Plotting capabilities

## Implementation

Using ROS 2 command-line tools:

```bash
# List all active nodes
ros2 node list

# Get info about a specific node
ros2 node info /node_name

# Echo messages on a topic
ros2 topic echo /topic_name std_msgs/msg/String

# Call a service
ros2 service call /service_name std_srvs/srv/Empty

# List all parameters of a node
ros2 param list /node_name

# Get parameter value
ros2 param get /node_name parameter_name
```

**What these commands do**: Provide system introspection capabilities to understand the state of your ROS network.
**Why this works**: These tools interact with the ROS 2 middleware to retrieve information about the runtime system.

## Examples

### Example 1: Node Communication Analysis
- Use `ros2 node info` to see what topics a node publishes/subscribes to
- Monitor topic data with `ros2 topic echo`
- Analyze message rates and types

### Example 2: Parameter Tuning
- List available parameters with `ros2 param list`
- Adjust parameters during runtime with `ros2 param set`
- Monitor parameter effects on system behavior

## Summary

ROS 2 tools are essential for developing, debugging, and maintaining robot systems. The command-line tools provide programmatic access to system information, while visualization tools like rqt offer intuitive interfaces for monitoring system behavior. Mastering these tools significantly improves development efficiency and system reliability.

## Exercises

### Logical Exercise
Explain how you would debug a situation where nodes are not communicating as expected.

### Conceptual Exercise
Design a debugging workflow for a multi-node ROS 2 system with sensor data processing.

### Implementation Exercise
Use ROS 2 command-line tools to monitor a running system and document the communication patterns.