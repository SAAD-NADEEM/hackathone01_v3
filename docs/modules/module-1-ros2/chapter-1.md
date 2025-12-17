---
title: "Chapter 1: ROS 2 Fundamentals and Architecture"
description: "Understanding the core concepts, architecture, and differences from ROS 1"
keywords: [ROS 2, architecture, DDS, nodes, topics, services, actions]
sidebar_position: 2
---

# Chapter 1: ROS 2 Fundamentals and Architecture

## Learning Objectives
By the end of this chapter, you will be able to:
1. Explain the fundamental architecture of ROS 2
2. Identify the key differences between ROS 1 and ROS 2
3. Understand the role of DDS in ROS 2 communication
4. Describe the basic building blocks of a ROS 2 system

## Prerequisites
Before starting this chapter, you should have:
- Basic understanding of robotics concepts
- Familiarity with software architecture concepts
- Understanding of distributed systems (helpful but not required)

## Core Concepts

### What is ROS 2?
ROS 2 (Robot Operating System 2) is the next generation of the Robot Operating System, designed to overcome the limitations of ROS 1 and provide a more robust, secure, and scalable framework for robotic applications. Unlike ROS 1, which was primarily designed for research environments, ROS 2 is built with production and commercial deployment in mind.

### DDS - The Foundation of ROS 2
The most significant architectural change in ROS 2 is the adoption of Data Distribution Service (DDS) as the underlying communication middleware. DDS is a vendor-neutral, open international standard that provides a publish-subscribe communication pattern with quality of service (QoS) controls.

**Key DDS Benefits:**
- **Distributed**: No central master node that can become a single point of failure
- **Robust QoS Controls**: Fine-grained control over communication behavior (reliability, durability, etc.)
- **Security**: Built-in security features for authentication, encryption, and access control
- **Real-time Support**: Better support for real-time systems with deterministic behavior

### ROS 2 vs ROS 1: Key Differences
1. **Communication Layer**: ROS 1 used a custom TCPROS/UDPROS protocol, while ROS 2 uses DDS
2. **Master Node**: ROS 1 required a master node; ROS 2 has no central master
3. **Quality of Service**: ROS 2 provides QoS settings for reliable communication
4. **Security**: ROS 2 has built-in security features from the ground up
5. **Cross-platform Support**: Better support for non-Linux platforms in ROS 2
6. **Real-time Support**: Improved real-time system support in ROS 2

## Implementation

### Setting up your first ROS 2 workspace
Let's start by creating a ROS 2 workspace and exploring its structure:

```bash
# Create a new workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

### Understanding ROS 2 Packages
In ROS 2, functionality is organized into packages. A package contains nodes, libraries, and other resources needed to perform specific tasks. Each package has a specific structure:

```
my_package/
├── CMakeLists.txt      # Build configuration for C++
├── package.xml         # Package metadata and dependencies
├── src/                # Source code files
├── include/            # Header files (C++)
├── launch/             # Launch files
├── config/             # Configuration files
└── test/               # Test files
```

## Examples

### Example: Basic Publisher Node
Here's a simple ROS 2 publisher node in Python:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

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

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example: Basic Subscriber Node
The corresponding subscriber node would look like:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Summary
In this chapter, we explored the fundamental concepts and architecture of ROS 2. We learned about the key differences from ROS 1, particularly the adoption of DDS as the underlying communication middleware. We also looked at the basic structure of a ROS 2 package and implemented simple publisher and subscriber nodes.

ROS 2's architecture addresses many of the limitations of ROS 1, making it more suitable for production environments. The use of DDS provides better support for distributed systems, real-time applications, and security, which are essential for physical AI and humanoid robotics applications.

## Exercises

### Logical Analysis Exercise
1. Explain why the removal of the master node in ROS 2 makes the system more robust compared to ROS 1.
2. Discuss the implications of DDS's QoS controls for safety-critical robotic applications.

### Conceptual Exploration Exercise
1. Research and compare different DDS implementations (Fast DDS, Cyclone DDS, RTI Connext).
2. Explain the advantages and disadvantages of the publish-subscribe communication pattern for robotic systems.

### Implementation Practice Exercise
1. Create a ROS 2 workspace and build the publisher/subscriber nodes shown in the examples above.
2. Modify the publisher to publish a custom message type containing a timestamp, string, and integer.
3. Create a launch file to start both the publisher and subscriber nodes together.

## References
1. ROS 2 Documentation: https://docs.ros.org/en/rolling/
2. DDS Specification: https://www.omg.org/spec/DDS/1.4/
3. ROS 2 Design: https://design.ros2.org/