---
title: "Chapter 4: Launch Systems and Parameter Management"
description: "Advanced configuration of ROS 2 systems using launch files and parameter management"
keywords: [ROS 2, launch files, parameters, configuration, system management, YAML]
sidebar_position: 5
---

# Chapter 4: Launch Systems and Parameter Management

## Learning Objectives
By the end of this chapter, you will be able to:
1. Design complex launch files to start and configure multiple nodes
2. Implement parameter management strategies for different execution environments
3. Use composition and lifecycle management in launch files
4. Create reusable launch file components with arguments and conditions
5. Configure Quality of Service settings through launch files

## Prerequisites
Before starting this chapter, you should have:
- Understanding of ROS 2 nodes and packages (Chapters 1-2)
- Knowledge of communication patterns (Chapter 3)
- Basic familiarity with YAML format

## Core Concepts

### Launch Files in ROS 2
Launch files in ROS 2 are Python scripts that define how to start and configure a collection of nodes. They replace the XML-based launch files of ROS 1 with more flexible Python-based alternatives that allow for programmatic control and complex conditional logic.

**Key Features:**
- Composable node startup
- Parameter file loading
- Conditional node launching
- Environment variable substitution
- Command-line argument support

### Parameter Management
ROS 2 provides sophisticated parameter management capabilities that allow nodes to be configured for different environments and use cases without code changes. Parameters can be specified at startup, loaded from files, or accessed through services.

**Parameter Types:**
- Built-in types (int, float, string, bool, arrays)
- Dynamic reconfiguration
- Parameter validation
- Node-specific parameters

### Composition and Lifecycle Management
ROS 2 allows for running multiple nodes within a single process (composition) and managing node lifecycles through specialized managers, enabling more efficient resource usage and complex state management.

## Implementation

### Advanced Launch File with Parameters and Conditions
A comprehensive launch file demonstrating various ROS 2 launch capabilities:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def launch_setup(context, *args, **kwargs):
    # Get launch configuration values
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_namespace = LaunchConfiguration('robot_namespace')
    debug_mode = LaunchConfiguration('debug')
    
    # Create nodes with parameters
    sensor_node = Node(
        package='my_robot_package',
        executable='sensor_node',
        name='sensor_node',
        namespace=robot_namespace,
        parameters=[
            {'use_sim_time': use_sim_time},
            {'sensor_range': 10.0},
            {'detection_frequency': 10}
        ],
        condition=UnlessCondition(debug_mode)  # Only run if not in debug mode
    )
    
    controller_node = Node(
        package='my_robot_package',
        executable='controller_node',
        name='controller_node',
        namespace=robot_namespace,
        parameters=[
            {'use_sim_time': use_sim_time},
            {'max_velocity': 1.0},
            {'safety_distance': 0.5}
        ],
        output='screen'  # Direct output to screen for debugging
    )
    
    # Composable nodes container
    container = ComposableNodeContainer(
        name='my_container',
        namespace=robot_namespace,
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='my_robot_package',
                plugin='my_robot_package::ImageProcessor',
                name='image_processor',
                parameters=[
                    {'image_width': 640},
                    {'image_height': 480},
                    {'use_sim_time': use_sim_time}
                ]
            ),
            ComposableNode(
                package='my_robot_package',
                plugin='my_robot_package::LidarProcessor',
                name='lidar_processor',
                parameters=[
                    {'min_range': 0.1},
                    {'max_range': 30.0},
                    {'use_sim_time': use_sim_time}
                ]
            )
        ],
        output='screen'
    )
    
    return [sensor_node, controller_node, container]

def generate_launch_description():
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    robot_namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value='my_robot',
        description='Namespace for robot nodes'
    )
    
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable debug mode'
    )
    
    # Include the setup function
    ld = LaunchDescription()
    
    ld.add_action(use_sim_time_arg)
    ld.add_action(robot_namespace_arg)
    ld.add_action(debug_arg)
    
    ld.add_action(OpaqueFunction(function=launch_setup))
    
    return ld
```

### Parameter Files in YAML Format
Parameter files provide a way to configure nodes with values loaded from external files:

```yaml
# my_robot_params.yaml
/**:  # Apply to all nodes
  ros__parameters:
    use_sim_time: false
    global_frame: "map"
    robot_base_frame: "base_link"

my_robot:
  ros__parameters:
    # Controller parameters
    max_linear_velocity: 1.0
    max_angular_velocity: 1.5
    acceleration_limit: 0.5
    
    # Sensor parameters
    laser_scan_topic: "/scan"
    camera_topic: "/camera/image_raw"
    
    # Navigation parameters
    planner_frequency: 1.0
    controller_frequency: 5.0

sensor_node:
  ros__parameters:
    sensor_range: 10.0
    detection_frequency: 10
    noise_model: "gaussian"
    noise_std_dev: 0.01

controller_node:
  ros__parameters:
    kp: 1.0  # Proportional gain
    ki: 0.1  # Integral gain
    kd: 0.05  # Derivative gain
    max_output: 10.0
    min_output: -10.0
```

## Examples

### Example: Conditional Launch Based on Environment
Launch file that adapts to different environments:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Environment-specific parameters
    robot_model = LaunchConfiguration('robot_model', default='turtlebot4')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    
    # Set environment variable for robot model
    set_model_env = SetEnvironmentVariable(
        name='ROBOT_MODEL',
        value=robot_model
    )
    
    # Include different launch files based on robot model
    robot_launch_file = PathJoinSubstitution([
        get_package_share_directory('my_robot_bringup'),
        'launch',
        [robot_model, '_robot.launch.py']
    ])
    
    # Node that only runs if use_rviz is true
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([
            get_package_share_directory('my_robot_bringup'),
            'rviz',
            'default.rviz'
        ])],
        condition=IfCondition(use_rviz)
    )
    
    return LaunchDescription([
        set_model_env,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_launch_file)
        ),
        rviz_node
    ])
```

### Example: Parameter Validation and Dynamic Reconfiguration
Node that validates parameters and allows dynamic reconfiguration:

```python
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rcl_interfaces.srv import SetParameters
from rclpy.exceptions import ParameterNotDeclaredException
from rclpy.parameter import Parameter

class ParameterValidationNode(Node):
    
    def __init__(self):
        super().__init__('parameter_validation_node')
        
        # Declare parameters with descriptors
        self.declare_parameter(
            'control_frequency',
            10,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description='Frequency for control loop (Hz)',
                integer_range=[ParameterDescriptor().integer_range[0].from_value(1),
                              ParameterDescriptor().integer_range[0].to_value(100),
                              ParameterDescriptor().integer_range[0].step(1)]
            )
        )
        
        self.declare_parameter(
            'safety_distance',
            0.5,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='Minimum safety distance in meters',
                floating_point_range=[ParameterDescriptor().floating_point_range[0].from_value(0.0),
                                     ParameterDescriptor().floating_point_range[0].to_value(5.0),
                                     ParameterDescriptor().floating_point_range[0].step(0.1)]
            )
        )
        
        # Add callback for parameter changes
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        # Timer for control loop
        control_freq = self.get_parameter('control_frequency').value
        self.timer = self.create_timer(1.0/control_freq, self.control_loop)
        
        self.get_logger().info(f'Initialized with control_frequency: {control_freq}, safety_distance: {self.get_parameter("safety_distance").value}')
    
    def parameters_callback(self, params):
        """
        Validates parameters before setting them
        """
        for param in params:
            if param.name == 'control_frequency' and (param.value < 1 or param.value > 100):
                return SetParameters.Result(successful=False, reason='Control frequency must be between 1 and 100 Hz')
            
            if param.name == 'safety_distance' and (param.value < 0.0 or param.value > 5.0):
                return SetParameters.Result(successful=False, reason='Safety distance must be between 0.0 and 5.0 m')
        
        # If all parameters are valid, set them
        for param in params:
            if param.name in ['control_frequency', 'safety_distance']:
                # Update timer if control_frequency changed
                if param.name == 'control_frequency':
                    self.timer.timer_period_ns = int(1e9 / param.value)  # Update timer period
                    self.get_logger().info(f'Updated control frequency to {param.value} Hz')
        
        return SetParameters.Result(successful=True)
    
    def control_loop(self):
        # Main control loop implementation
        safety_distance = self.get_parameter('safety_distance').value
        self.get_logger().info(f'Control loop running, safety distance: {safety_distance}m')

def main(args=None):
    rclpy.init(args=args)
    node = ParameterValidationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Summary
In this chapter, we explored ROS 2's sophisticated launch system and parameter management capabilities. Launch files in ROS 2 are Python scripts that provide powerful programmatic control over node startup and configuration, including conditional logic, parameter loading, and composition.

Parameter management in ROS 2 enables nodes to be configured for different environments without code changes, with support for validation, dynamic reconfiguration, and hierarchical parameter organization. These capabilities are essential for creating flexible and maintainable robotic systems that can adapt to different robots, environments, and operational modes.

## Exercises

### Logical Analysis Exercise
1. Compare the advantages and disadvantages of launch files vs. manual node startup in production robotic systems.
2. Analyze how parameter management can improve the maintainability of robotic software across different hardware platforms.

### Conceptual Exploration Exercise
1. Research the differences between composition containers and lifecycle nodes in ROS 2.
2. Investigate how ROS 2 handles parameter conflicts when multiple parameter files are loaded.

### Implementation Practice Exercise
1. Create a launch file that starts a complete robotic system with multiple sensors and controllers.
2. Implement parameter validation for a robot controller node with bounds checking.
3. Create a parameter file for a mobile robot that can be used in both simulation and real-world environments.
4. Design a launch file system with reusable components that can be combined for different robot configurations.

## References
1. ROS 2 Launch System: https://docs.ros.org/en/rolling/Tutorials/Launch-system.html
2. Parameter Management: https://docs.ros.org/en/rolling/Tutorials/Parameters/Understanding-ROS2-Parameters.html
3. Composition in ROS 2: https://docs.ros.org/en/rolling/Tutorials/Composition.html