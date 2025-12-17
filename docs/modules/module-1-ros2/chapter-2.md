---
title: "Chapter 2: Nodes, Packages, and Workspaces"
description: "Understanding the building blocks of ROS 2 systems: nodes, packages, and workspace organization"
keywords: [ROS 2, nodes, packages, workspaces, build system, colcon]
sidebar_position: 3
---

# Chapter 2: Nodes, Packages, and Workspaces

## Learning Objectives
By the end of this chapter, you will be able to:
1. Create and structure ROS 2 packages according to best practices
2. Understand the node lifecycle and management in ROS 2
3. Configure and build workspaces using colcon
4. Implement parameter management and node composition
5. Use launch files to manage complex robotic systems

## Prerequisites
Before starting this chapter, you should have:
- Chapter 1 knowledge of ROS 2 fundamentals
- Basic understanding of Python and/or C++
- Familiarity with command-line tools

## Core Concepts

### ROS 2 Nodes
A node in ROS 2 is an executable process that performs computation. Nodes are the fundamental building blocks of ROS applications. Unlike ROS 1, ROS 2 nodes can be:

- **Standalone**: Each node runs in its own process
- **Composed**: Multiple nodes can be combined into a single process for efficiency
- **Managed**: Nodes can be managed by lifecycle managers for complex startup/shutdown sequences

### ROS 2 Packages
Packages organize ROS 2 code and resources. A package contains:
- Source code (in C++ or Python)
- Launch files
- Configuration files
- Message/service/action definitions
- Documentation
- Tests

### Workspaces
A workspace is a directory that contains packages and build artifacts. ROS 2 uses colcon as the build system to compile packages within a workspace.

## Implementation

### Creating a ROS 2 Package
Use the `ros2 pkg create` command to create a new package:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_robot_package
```

This creates a Python package structure with the following files:

```
my_robot_package/
├── package.xml          # Package metadata
├── setup.py             # Python setup configuration
├── setup.cfg            # Installation configuration
├── resource/            # Resource files
├── test/                # Test files
├── my_robot_package/    # Python module
│   ├── __init__.py
│   └── my_node.py
└── README.md
```

### Creating a Parameter-Based Node
Let's create a node that uses parameters:

```python
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

class ParameterNode(Node):

    def __init__(self):
        super().__init__('parameter_node')
        
        # Declare parameters with defaults
        self.declare_parameter('robot_name', 'turtlebot')
        self.declare_parameter('max_velocity', 1.0)
        self.declare_parameter('safety_mode', True)
        
        # Get parameter values
        self.robot_name = self.get_parameter('robot_name').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.safety_mode = self.get_parameter('safety_mode').value
        
        self.get_logger().info(f'Robot: {self.robot_name}, Max velocity: {self.max_velocity}, Safety: {self.safety_mode}')
        
        # Create a callback for parameter changes
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # Create a timer to periodically publish info
        self.timer = self.create_timer(1.0, self.timer_callback)
    
    def parameter_callback(self, params):
        for param in params:
            if param.name == 'robot_name':
                self.robot_name = param.value
            elif param.name == 'max_velocity':
                self.max_velocity = param.value
            elif param.name == 'safety_mode':
                self.safety_mode = param.value
        return SetParametersResult(successful=True)
    
    def timer_callback(self):
        self.get_logger().info(f'Robot {self.robot_name} running at {self.max_velocity} m/s')

def main(args=None):
    rclpy.init(args=args)
    node = ParameterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Examples

### Example: Launch File for Multiple Nodes
Create a launch file to start multiple nodes together:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='turtlebot',
        description='Name of the robot'
    )
    
    # Create nodes
    sensor_node = Node(
        package='my_robot_package',
        executable='sensor_node',
        name='sensor_node',
        parameters=[
            {'robot_name': LaunchConfiguration('robot_name')},
            {'sensor_range': 10.0}
        ]
    )
    
    controller_node = Node(
        package='my_robot_package',
        executable='controller_node',
        name='controller_node',
        parameters=[
            {'robot_name': LaunchConfiguration('robot_name')},
            {'max_velocity': 1.0}
        ]
    )
    
    return LaunchDescription([
        robot_name_arg,
        sensor_node,
        controller_node
    ])
```

### Example: Node Composition
For better performance, you can compose multiple nodes into a single process:

```python
from rclpy.node import Node
from rclpy import create_node
from rclpy.executors import SingleThreadedExecutor
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future
from example_interfaces.srv import AddTwoInts

class AdditionServer(Node):
    def __init__(self):
        super().__init__('addition_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {response.sum}')
        return response

class AdditionClient(Node):
    def __init__(self):
        super().__init__('addition_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        return self.cli.call_async(self.req)

def main():
    rclpy.init()
    
    # Create composed nodes
    addition_server = AdditionServer()
    addition_client = AdditionClient()
    
    # Use single-threaded executor to run both nodes
    executor = SingleThreadedExecutor()
    executor.add_node(addition_server)
    executor.add_node(addition_client)
    
    # Send a request from the client
    future = addition_client.send_request(1, 2)
    
    try:
        executor.spin_until_future_complete(future)
        response = future.result()
        addition_client.get_logger().info(f'Result: {response.sum}')
    except KeyboardInterrupt:
        pass
    finally:
        addition_server.destroy_node()
        addition_client.destroy_node()
        rclpy.shutdown()
```

## Summary
In this chapter, we explored the building blocks of ROS 2: nodes, packages, and workspaces. We learned how to create and structure packages, implement parameter-based nodes, and use launch files to manage complex robotic systems. The ability to compose nodes into a single process provides performance benefits for resource-constrained robotic applications.

Node composition and parameter management are essential for creating scalable and maintainable robotic systems. Launch files allow for complex system configurations that can be easily modified and reused across different robotic platforms.

## Exercises

### Logical Analysis Exercise
1. Compare the benefits and drawbacks of standalone vs. composed nodes in ROS 2.
2. Explain how parameter management in ROS 2 improves upon ROS 1's parameter system.

### Conceptual Exploration Exercise
1. Research the different build types available in ROS 2 (ament_cmake, ament_python, etc.).
2. Compare colcon with other build systems (catkin, cmake, etc.).

### Implementation Practice Exercise
1. Create a ROS 2 package with both Python and C++ nodes.
2. Implement a node that dynamically reconfigures parameters based on sensor input.
3. Create a launch file that starts multiple nodes with different parameter sets.
4. Implement a composed node setup and compare its performance with standalone nodes.

## References
1. ROS 2 Packages Documentation: https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html
2. ROS 2 Launch Documentation: https://docs.ros.org/en/rolling/Tutorials/Launch-system.html
3. Colcon Build System: https://colcon.readthedocs.io/