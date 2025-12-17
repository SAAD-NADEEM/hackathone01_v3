---
title: "Chapter 13: Action Planning and Execution with VLA"
description: "Implementing action planning systems that integrate vision, language, and physical action"
keywords: [action planning, VLA, task planning, robotic execution, multimodal control, embodied AI]
sidebar_position: 17
---

# Chapter 13: Action Planning and Execution with VLA

## Learning Objectives
By the end of this chapter, you will be able to:
1. Design multimodal action planning systems that integrate vision and language inputs
2. Implement hierarchical task planners for complex robotic behaviors
3. Create execution monitoring systems that adapt to environmental changes
4. Develop fallback strategies for handling plan failures
5. Evaluate the performance of VLA-based action systems

## Prerequisites
Before starting this chapter, you should have:
- Understanding of vision-language models (Chapter 11)
- Knowledge of natural language interfaces (Chapter 12)
- Familiarity with ROS 2 action servers and navigation systems
- Basic understanding of task planning and execution frameworks

## Core Concepts

### Multimodal Action Planning
Multimodal action planning integrates information from multiple sources (vision, language, proprioception) to generate sequences of actions that accomplish user goals. This approach enables robots to understand complex tasks described in natural language and execute them in real-world environments.

### Hierarchical Task Planning
Complex robotic tasks are decomposed into hierarchies of subtasks, where high-level goals are refined into sequences of low-level actions. This approach allows for more efficient planning and better handling of complex, multi-step tasks.

### Execution Monitoring and Adaptation
Robotic systems must continuously monitor the execution of their plans and adapt to changes in the environment or unexpected events. This requires real-time integration of sensor data with the action plan.

### Plan Representation and Reasoning
Effective action planning requires appropriate representations for tasks, states, and actions, along with reasoning mechanisms that can handle uncertainty and incomplete information.

## Implementation

### VLA-Based Task Planner Implementation
Here's an implementation of a VLA-based task planner:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.qos import QoSProfile
import torch
import clip
import numpy as np
from dataclasses import dataclass
from typing import List, Dict, Any, Optional
from enum import Enum
import time

class TaskStatus(Enum):
    IDLE = 0
    PLANNING = 1
    EXECUTING = 2
    COMPLETED = 3
    FAILED = 4
    CANCELLED = 5

@dataclass
class TaskStep:
    action: str  # "navigate", "grasp", "detect", etc.
    parameters: Dict[str, Any]
    description: str  # Natural language description
    
class VLAActionPlannerNode(Node):
    def __init__(self):
        super().__init__('vla_action_planner')
        
        # Initialize CLIP model for vision processing
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.clip_model, self.clip_preprocess = clip.load("ViT-B/32", device=self.device)
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Publishers
        self.nav_goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.cmd_vel_pub = self.create_publisher(Point, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/task_status', 10)
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.task_command_sub = self.create_subscription(
            String, '/task_command', self.task_command_callback, 10)
        
        # Task planning attributes
        self.current_task = None
        self.task_plan = []
        self.current_step_index = 0
        self.current_status = TaskStatus.IDLE
        self.latest_image = None
        self.latest_scan = None
        
        # Object locations database (simplified)
        self.known_objects = {
            'red cube': (1.0, 0.0, 0.0),
            'blue cylinder': (2.0, 1.0, 0.0),
            'green sphere': (0.5, -1.5, 0.0)
        }
        
        # Task execution parameters
        self.execution_threshold = 0.3  # Threshold for action acceptance
        self.max_execution_time = 30.0  # Max time to execute a single action
        self.current_task_start_time = None
        
        self.get_logger().info("VLA Action Planner Node initialized")
    
    def image_callback(self, msg):
        # Store latest image for vision processing
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.latest_image = cv_image
    
    def scan_callback(self, msg):
        # Store latest scan for navigation planning
        self.latest_scan = msg
    
    def task_command_callback(self, msg):
        command = msg.data.lower()
        self.get_logger().info(f"Received task command: {command}")
        
        # Parse and plan the task
        task_plan = self.parse_task_command(command)
        if task_plan:
            self.execute_task_plan(task_plan, command)
        else:
            self.get_logger().warn(f"Could not understand task command: {command}")
    
    def parse_task_command(self, command: str) -> List[TaskStep]:
        """
        Parse a natural language command and generate a task plan
        This is a simplified parser - real systems would use more sophisticated NLP
        """
        plan = []
        
        if "pick up" in command or "grasp" in command:
            # Extract object to grasp
            obj = self.extract_object(command)
            if obj and obj in self.known_objects:
                # Navigate to object
                obj_pos = self.known_objects[obj]
                plan.append(TaskStep(
                    action="navigate",
                    parameters={"x": obj_pos[0], "y": obj_pos[1], "theta": obj_pos[2]},
                    description=f"Navigate to {obj}"
                ))
                
                # Grasp the object
                plan.append(TaskStep(
                    action="grasp",
                    parameters={"object": obj},
                    description=f"Grasp the {obj}"
                ))
            else:
                self.get_logger().warn(f"Unknown object in command: {command}")
                return []
        
        elif "go to" in command or "navigate to" in command:
            # Extract location
            location = self.extract_location(command)
            if location:
                # For now, we'll use a simple location mapping
                location_map = {
                    'kitchen': (5.0, 5.0, 0.0),
                    'bedroom': (-2.0, -1.0, 0.0),
                    'office': (3.0, -2.0, 0.0)
                }
                
                if location in location_map:
                    pos = location_map[location]
                    plan.append(TaskStep(
                        action="navigate",
                        parameters={"x": pos[0], "y": pos[1], "theta": pos[2]},
                        description=f"Navigate to {location}"
                    ))
                else:
                    self.get_logger().warn(f"Unknown location: {location}")
                    return []
        
        elif "bring to" in command:
            # Extract object and destination
            obj = self.extract_object(command)
            destination = self.extract_location(command)
            
            if obj and destination and obj in self.known_objects:
                # Navigate to object
                obj_pos = self.known_objects[obj]
                plan.append(TaskStep(
                    action="navigate",
                    parameters={"x": obj_pos[0], "y": obj_pos[1], "theta": obj_pos[2]},
                    description=f"Navigate to {obj}"
                ))
                
                # Grasp the object
                plan.append(TaskStep(
                    action="grasp",
                    parameters={"object": obj},
                    description=f"Grasp the {obj}"
                ))
                
                # Navigate to destination
                location_map = {
                    'kitchen': (5.0, 5.0, 0.0),
                    'bedroom': (-2.0, -1.0, 0.0),
                    'office': (3.0, -2.0, 0.0)
                }
                
                if destination in location_map:
                    dest_pos = location_map[destination]
                    plan.append(TaskStep(
                        action="navigate",
                        parameters={"x": dest_pos[0], "y": dest_pos[1], "theta": dest_pos[2]},
                        description=f"Navigate to {destination}"
                    ))
                    
                    # Release the object
                    plan.append(TaskStep(
                        action="release",
                        parameters={"object": obj},
                        description=f"Release the {obj} at {destination}"
                    ))
    
        return plan
    
    def extract_object(self, command: str) -> Optional[str]:
        """Extract object name from command"""
        # Simple object extraction based on known objects
        for obj in self.known_objects.keys():
            if obj in command:
                return obj
        return None
    
    def extract_location(self, command: str) -> Optional[str]:
        """Extract location from command"""
        # Simple location extraction
        locations = ['kitchen', 'bedroom', 'office', 'living room', 'dining room']
        for loc in locations:
            if loc in command.lower():
                return loc
        return None
    
    def execute_task_plan(self, plan: List[TaskStep], original_command: str):
        """Start execution of a task plan"""
        if not plan:
            self.get_logger().warn("Empty task plan, nothing to execute")
            return
        
        self.task_plan = plan
        self.current_step_index = 0
        self.current_status = TaskStatus.PLANNING
        self.current_task = original_command
        self.current_task_start_time = time.time()
        
        self.get_logger().info(f"Starting execution of task: {original_command}")
        self.get_logger().info(f"Plan contains {len(plan)} steps")
        
        # Publish status
        status_msg = String()
        status_msg.data = f"PLANNING: {original_command}"
        self.status_pub.publish(status_msg)
        
        # Execute first step
        self.execute_current_step()
    
    def execute_current_step(self):
        """Execute the current step in the plan"""
        if self.current_step_index >= len(self.task_plan):
            # All steps completed
            self.current_status = TaskStatus.COMPLETED
            self.get_logger().info("Task completed successfully")
            
            status_msg = String()
            status_msg.data = "COMPLETED: Task execution finished"
            self.status_pub.publish(status_msg)
            return
        
        current_step = self.task_plan[self.current_step_index]
        self.get_logger().info(f"Executing step {self.current_step_index + 1}/{len(self.task_plan)}: {current_step.description}")
        
        self.current_status = TaskStatus.EXECUTING
        
        # Execute based on action type
        if current_step.action == "navigate":
            self.execute_navigation(current_step.parameters)
        elif current_step.action == "grasp":
            self.execute_grasp(current_step.parameters)
        elif current_step.action == "release":
            self.execute_release(current_step.parameters)
        elif current_step.action == "detect":
            self.execute_detection(current_step.parameters)
        else:
            self.get_logger().error(f"Unknown action type: {current_step.action}")
            self.current_status = TaskStatus.FAILED
    
    def execute_navigation(self, params: Dict[str, Any]):
        """Execute navigation action"""
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = "map"
        goal.pose.position.x = params["x"]
        goal.pose.position.y = params["y"]
        goal.pose.position.z = 0.0
        goal.pose.orientation.z = np.sin(params["theta"] / 2.0)
        goal.pose.orientation.w = np.cos(params["theta"] / 2.0)
        
        self.nav_goal_pub.publish(goal)
        self.get_logger().info(f"Published navigation goal to ({params['x']}, {params['y']})")
        
        # In a real system, we would monitor navigation progress
        # For simulation, we'll move to next step after delay
        self.current_status = TaskStatus.EXECUTING
        
        # Use a timer to advance to next step after simulated navigation
        self.nav_timer = self.create_timer(2.0, self.advance_to_next_step)
    
    def execute_grasp(self, params: Dict[str, Any]):
        """Execute grasp action"""
        self.get_logger().info(f"Attempting to grasp {params['object']}")
        
        # In a real system, this would activate a gripper
        # For simulation, we'll just log and advance
        self.get_logger().info(f"Grasped {params['object']} successfully")
        
        # Simulate grasp time
        self.grasp_timer = self.create_timer(1.0, self.advance_to_next_step)
    
    def execute_release(self, params: Dict[str, Any]):
        """Execute release action"""
        self.get_logger().info(f"Releasing {params['object']}")
        
        # In a real system, this would open a gripper
        # For simulation, we'll just log and advance
        self.get_logger().info(f"Released {params['object']} successfully")
        
        # Simulate release time
        self.release_timer = self.create_timer(1.0, self.advance_to_next_step)
    
    def execute_detection(self, params: Dict[str, Any]):
        """Execute object detection action"""
        self.get_logger().info(f"Detecting {params.get('object_type', 'object')}")
        
        # Use vision system to detect objects
        if self.latest_image is not None:
            # Process image to detect objects
            detected = self.detect_objects_in_image(self.latest_image, params.get('object_type', 'object'))
            self.get_logger().info(f"Detection result: {detected}")
        else:
            self.get_logger().warn("No image available for detection")
        
        # Simulate detection time
        self.detect_timer = self.create_timer(1.0, self.advance_to_next_step)
    
    def detect_objects_in_image(self, image, object_type):
        """Use CLIP to detect if specific object is in image"""
        if not self.clip_model:
            return False
        
        from PIL import Image as PILImage
        pil_image = PILImage.fromarray(image)
        image_input = self.clip_preprocess(pil_image).unsqueeze(0).to(self.device)
        
        # Create text description for the object type
        text_descriptions = clip.tokenize([f"a photo of a {object_type}", f"an image of {object_type}"]).to(self.device)
        
        # Encode image and text
        with torch.no_grad():
            image_features = self.clip_model.encode_image(image_input)
            text_features = self.clip_model.encode_text(text_descriptions)
            
            # Normalize features
            image_features /= image_features.norm(dim=-1, keepdim=True)
            text_features /= text_features.norm(dim=-1, keepdim=True)
            
            # Calculate similarity
            similarity = (100.0 * image_features @ text_features.T).softmax(dim=-1)
        
        # Check if the object type was detected
        values, indices = similarity[0].topk(1)
        confidence = values[0].item()
        
        return confidence > self.execution_threshold
    
    def advance_to_next_step(self):
        """Advance to the next step in the plan"""
        # Cancel any active timers
        if hasattr(self, 'nav_timer') and self.nav_timer:
            self.nav_timer.cancel()
        if hasattr(self, 'grasp_timer') and self.grasp_timer:
            self.grasp_timer.cancel()
        if hasattr(self, 'release_timer') and self.release_timer:
            self.release_timer.cancel()
        if hasattr(self, 'detect_timer') and self.detect_timer:
            self.detect_timer.cancel()
        
        # Move to next step
        self.current_step_index += 1
        self.execute_current_step()
    
    def check_task_timeout(self):
        """Check if the current task has timed out"""
        if self.current_task_start_time and time.time() - self.current_task_start_time > self.max_execution_time:
            self.get_logger().error("Task execution timed out")
            self.current_status = TaskStatus.FAILED
            
            status_msg = String()
            status_msg.data = "FAILED: Task execution timed out"
            self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VLAActionPlannerNode()
    
    # Create a timer to check for timeouts
    def timeout_check():
        node.check_task_timeout()
    
    timeout_timer = node.create_timer(1.0, timeout_check)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down VLA Action Planner Node")
    finally:
        # Stop the robot
        cmd_vel = Point()
        # In a real system, we'd publish a stop command
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Examples

### Example: Hierarchical Task Planner with VLA Integration
Implementation of a more sophisticated hierarchical task planner:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import torch
import clip
import numpy as np
from typing import Dict, List, Any, Optional, Callable
from dataclasses import dataclass
from enum import Enum
import time

class TaskPriority(Enum):
    LOW = 1
    NORMAL = 2
    HIGH = 3
    CRITICAL = 4

class ExecutionStatus(Enum):
    PENDING = 1
    IN_PROGRESS = 2
    COMPLETED = 3
    FAILED = 4
    CANCELLED = 5

@dataclass
class Task:
    id: str
    description: str
    priority: TaskPriority
    steps: List[Dict[str, Any]]
    created_at: float
    dependencies: List[str] = None  # IDs of tasks this task depends on
    
    def __post_init__(self):
        if self.dependencies is None:
            self.dependencies = []

@dataclass
class ExecutionStep:
    action: str
    parameters: Dict[str, Any]
    success_condition: Callable[[Node], bool]
    timeout: float = 10.0  # seconds
    retry_count: int = 3

class VLAHierarchicalPlannerNode(Node):
    def __init__(self):
        super().__init__('vla_hierarchical_planner')
        
        # Initialize components
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.clip_model, self.clip_preprocess = clip.load("ViT-B/32", device=self.device)
        self.bridge = CvBridge()
        
        # Publishers
        self.nav_goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.task_status_pub = self.create_publisher(String, '/task_status', 10)
        self.debug_pub = self.create_publisher(String, '/debug_info', 10)
        
        # Subscribers
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.task_queue_sub = self.create_subscription(String, '/task_queue', self.task_queue_callback, 10)
        
        # Task management
        self.task_queue: List[Task] = []
        self.active_tasks: Dict[str, Task] = {}
        self.task_history: List[Task] = []
        self.latest_image = None
        self.latest_scan = None
        
        # Environment model
        self.object_locations = {}
        self.obstacle_map = {}
        
        # Execution management
        self.current_execution = None
        self.execution_steps = []
        self.current_step_index = 0
        self.step_start_time = None
        self.step_retry_count = 0
        
        # Timer for executing tasks
        self.execution_timer = self.create_timer(0.1, self.execute_next_step)
        
        self.get_logger().info("VLA Hierarchical Planner Node initialized")
    
    def image_callback(self, msg):
        # Store the latest image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.latest_image = cv_image
    
    def scan_callback(self, msg):
        # Store the latest scan
        self.latest_scan = msg
    
    def task_queue_callback(self, msg):
        # Parse task from the message
        task_data = msg.data.split(':')
        if len(task_data) >= 2:
            task_id = task_data[0]
            task_description = task_data[1]
            
            # Create and add task to queue
            task = self.create_task_from_description(task_id, task_description)
            if task:
                self.add_task_to_queue(task)
    
    def create_task_from_description(self, task_id: str, description: str) -> Optional[Task]:
        """Create a task based on its natural language description"""
        description = description.lower()
        
        if "fetch" in description or "bring" in description or "get" in description:
            # Create a fetch task
            steps = []
            
            # Extract target object and destination
            target_obj = self.extract_object_from_description(description)
            destination = self.extract_location_from_description(description)
            
            if target_obj:
                # Navigate to object
                steps.append({
                    'action': 'navigate_to_object',
                    'parameters': {'object': target_obj},
                    'success_condition': self.check_navigation_success,
                    'timeout': 30.0
                })
                
                # Grasp object
                steps.append({
                    'action': 'grasp_object',
                    'parameters': {'object': target_obj},
                    'success_condition': self.check_grasp_success,
                    'timeout': 10.0
                })
                
                if destination:
                    # Navigate to destination
                    steps.append({
                        'action': 'navigate_to_location',
                        'parameters': {'location': destination},
                        'success_condition': self.check_navigation_success,
                        'timeout': 30.0
                    })
                    
                    # Release object
                    steps.append({
                        'action': 'release_object',
                        'parameters': {'location': destination},
                        'success_condition': self.check_release_success,
                        'timeout': 5.0
                    })
            
            return Task(
                id=task_id,
                description=description,
                priority=TaskPriority.NORMAL,
                steps=steps,
                created_at=time.time()
            )
        
        elif "navigate" in description or "go to" in description:
            # Create a navigation task
            location = self.extract_location_from_description(description)
            if location:
                steps = [{
                    'action': 'navigate_to_location',
                    'parameters': {'location': location},
                    'success_condition': self.check_navigation_success,
                    'timeout': 30.0
                }]
                
                return Task(
                    id=task_id,
                    description=description,
                    priority=TaskPriority.NORMAL,
                    steps=steps,
                    created_at=time.time()
                )
        
        # Add more task types as needed
        return None
    
    def extract_object_from_description(self, description: str) -> Optional[str]:
        """Extract object name from task description"""
        # Known objects in the environment
        known_objects = ['red cube', 'blue cylinder', 'green sphere', 'book', 'cup', 'box']
        
        for obj in known_objects:
            if obj in description:
                return obj
        
        return None
    
    def extract_location_from_description(self, description: str) -> Optional[str]:
        """Extract location from task description"""
        # Known locations in the environment
        known_locations = ['kitchen', 'bedroom', 'office', 'living room', 'dining room']
        
        for loc in known_locations:
            if loc in description:
                return loc
        
        return None
    
    def add_task_to_queue(self, task: Task):
        """Add a task to the execution queue"""
        self.task_queue.append(task)
        
        # Sort queue by priority
        self.task_queue.sort(key=lambda t: t.priority.value, reverse=True)
        
        self.get_logger().info(f"Added task {task.id} to queue: {task.description}")
        
        # Debug info
        self.publish_debug_info(f"Task {task.id} added. Queue size: {len(self.task_queue)}")
    
    def execute_next_step(self):
        """Execute the next step in the current task"""
        # If no active task, check the queue
        if not self.current_execution and self.task_queue:
            # Get next task based on priority
            next_task = self.task_queue.pop(0)
            self.start_task_execution(next_task)
        
        # If we have an active execution, continue
        if self.current_execution:
            self.continue_task_execution()
    
    def start_task_execution(self, task: Task):
        """Start executing a task"""
        self.current_execution = task
        self.current_step_index = 0
        self.execution_steps = task.steps
        self.step_retry_count = 0
        
        self.get_logger().info(f"Starting execution of task {task.id}: {task.description}")
        
        # Publish status
        status_msg = String()
        status_msg.data = f"EXECUTING: {task.description}"
        self.task_status_pub.publish(status_msg)
    
    def continue_task_execution(self):
        """Continue execution of the current task"""
        if self.current_step_index >= len(self.execution_steps):
            # Task completed
            self.complete_current_task(ExecutionStatus.COMPLETED)
            return
        
        current_step = self.execution_steps[self.current_step_index]
        
        # Check if step is taking too long
        if self.step_start_time:
            elapsed_time = time.time() - self.step_start_time
            if elapsed_time > current_step['timeout']:
                self.get_logger().warn(f"Step timed out after {elapsed_time:.2f}s")
                
                # Try to retry or fail the task
                if self.step_retry_count < current_step.get('retry_count', 3):
                    self.step_retry_count += 1
                    self.get_logger().info(f"Retrying step ({self.step_retry_count})")
                    self.execute_step_immediately(current_step)
                else:
                    self.get_logger().error("Step failed after retries")
                    self.complete_current_task(ExecutionStatus.FAILED)
                
                return
        
        # Execute the current step if not already started
        if self.step_start_time is None:
            self.execute_step_immediately(current_step)
    
    def execute_step_immediately(self, step):
        """Execute a step immediately"""
        self.get_logger().info(f"Executing step: {step['action']} with {step['parameters']}")
        
        action = step['action']
        params = step['parameters']
        
        # Start timing the step
        self.step_start_time = time.time()
        
        # Execute based on action type
        if action == 'navigate_to_object':
            self.execute_navigate_to_object(params)
        elif action == 'navigate_to_location':
            self.execute_navigate_to_location(params)
        elif action == 'grasp_object':
            self.execute_grasp_object(params)
        elif action == 'release_object':
            self.execute_release_object(params)
        else:
            self.get_logger().error(f"Unknown action: {action}")
            self.complete_current_task(ExecutionStatus.FAILED)
    
    def execute_navigate_to_object(self, params):
        """Navigate to a specific object using visual input"""
        target_object = params['object']
        
        # Look up object location using vision system
        object_pos = self.locate_object_in_environment(target_object)
        
        if object_pos:
            # Navigate to the object
            goal = PoseStamped()
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.header.frame_id = "map"
            goal.pose.position.x = object_pos[0]
            goal.pose.position.y = object_pos[1]
            goal.pose.position.z = 0.0
            goal.pose.orientation.w = 1.0  # No rotation
            
            self.nav_goal_pub.publish(goal)
            self.get_logger().info(f"Navigating to {target_object} at ({object_pos[0]}, {object_pos[1]})")
        else:
            self.get_logger().error(f"Could not locate {target_object}")
            self.complete_current_task(ExecutionStatus.FAILED)
    
    def execute_navigate_to_location(self, params):
        """Navigate to a specific location"""
        target_location = params['location']
        
        # Define location coordinates (in a real system, these would come from a map)
        location_coords = {
            'kitchen': (5.0, 5.0, 0.0),
            'bedroom': (-2.0, -1.0, 0.0),
            'office': (3.0, -2.0, 0.0),
            'living room': (0.0, 3.0, 0.0)
        }
        
        if target_location in location_coords:
            coords = location_coords[target_location]
            goal = PoseStamped()
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.header.frame_id = "map"
            goal.pose.position.x = coords[0]
            goal.pose.position.y = coords[1]
            goal.pose.position.z = 0.0
            goal.pose.orientation.z = np.sin(coords[2] / 2.0)
            goal.pose.orientation.w = np.cos(coords[2] / 2.0)
            
            self.nav_goal_pub.publish(goal)
            self.get_logger().info(f"Navigating to {target_location} at ({coords[0]}, {coords[1]})")
        else:
            self.get_logger().error(f"Unknown location: {target_location}")
            self.complete_current_task(ExecutionStatus.FAILED)
    
    def execute_grasp_object(self, params):
        """Grasp an object (simulated)"""
        target_object = params['object']
        self.get_logger().info(f"Attempting to grasp {target_object}")
        
        # In a real system, this would activate a gripper
        # Simulate success after a delay
        self.simulated_action_complete(2.0, self.advance_to_next_step)
    
    def execute_release_object(self, params):
        """Release an object (simulated)"""
        location = params.get('location', 'current')
        self.get_logger().info(f"Releasing object at {location}")
        
        # In a real system, this would open a gripper
        # Simulate success after a delay
        self.simulated_action_complete(1.5, self.advance_to_next_step)
    
    def locate_object_in_environment(self, object_name):
        """Locate an object in the environment using vision"""
        if not self.latest_image or not self.clip_model:
            return None
        
        # Use CLIP to detect the object in the current image
        from PIL import Image as PILImage
        pil_image = PILImage.fromarray(self.latest_image)
        image_input = self.clip_preprocess(pil_image).unsqueeze(0).to(self.device)
        
        # Create text description for the object
        text_descriptions = clip.tokenize([f"a photo of a {object_name}", f"an image containing a {object_name}"]).to(self.device)
        
        # Encode image and text
        with torch.no_grad():
            image_features = self.clip_model.encode_image(image_input)
            text_features = self.clip_model.encode_text(text_descriptions)
            
            # Normalize features
            image_features /= image_features.norm(dim=-1, keepdim=True)
            text_features /= text_features.norm(dim=-1, keepdim=True)
            
            # Calculate similarity
            similarity = (100.0 * image_features @ text_features.T).softmax(dim=-1)
            values, indices = similarity[0].topk(1)
        
        # If object detected with sufficient confidence, return a placeholder position
        confidence = values[0].item()
        if confidence > 0.5:  # Threshold for detection
            # In a real system, we'd need to determine actual coordinates
            # For now, return a simulated position
            return (1.5, 0.5, 0.0)  # Placeholder coordinates
        
        return None
    
    def check_navigation_success(self):
        """Check if navigation was successful"""
        # In a real system, this would check odometry and goal status
        # For simulation, we'll just return True after a delay
        return self.step_start_time and (time.time() - self.step_start_time) > 2.0
    
    def check_grasp_success(self):
        """Check if grasp was successful"""
        # In a real system, this would check gripper sensors
        # For simulation, we'll just return True after a delay
        return self.step_start_time and (time.time() - self.step_start_time) > 2.0
    
    def check_release_success(self):
        """Check if release was successful"""
        # In a real system, this would check gripper sensors
        # For simulation, we'll just return True after a delay
        return self.step_start_time and (time.time() - self.step_start_time) > 1.5
    
    def advance_to_next_step(self):
        """Advance to the next step in the current task"""
        if self.current_execution:
            self.current_step_index += 1
            self.step_start_time = None
            self.step_retry_count = 0
            
            self.get_logger().info(f"Advanced to step {self.current_step_index + 1}/{len(self.execution_steps)}")
    
    def complete_current_task(self, status: ExecutionStatus):
        """Complete the current task with the given status"""
        if self.current_execution:
            task = self.current_execution
            
            self.get_logger().info(f"Task {task.id} completed with status: {status}")
            
            # Update status
            status_msg = String()
            status_msg.data = f"COMPLETED: {task.description}" if status == ExecutionStatus.COMPLETED else f"FAILED: {task.description}"
            self.task_status_pub.publish(status_msg)
            
            # Add to history
            self.task_history.append(task)
            
            # Clear current execution
            self.current_execution = None
            self.execution_steps = []
            self.current_step_index = 0
            self.step_start_time = None
            self.step_retry_count = 0
    
    def simulated_action_complete(self, delay, callback):
        """Simulate an action completing after a delay"""
        # In a real implementation, we would monitor actual robot state
        # For this simulation, we'll just call the callback after the delay
        timer = self.create_timer(delay, callback)
    
    def publish_debug_info(self, info: str):
        """Publish debug information"""
        debug_msg = String()
        debug_msg.data = info
        self.debug_pub.publish(debug_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VLAHierarchicalPlannerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down VLA Hierarchical Planner Node")
    finally:
        # Stop the robot
        cmd_vel = Twist()
        node.cmd_vel_pub.publish(cmd_vel)
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example: Execution Monitoring and Recovery System
A system that monitors execution and provides recovery mechanisms:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile
from typing import Dict, List, Callable, Any
from dataclasses import dataclass
from datetime import datetime
import time

@dataclass
class ExecutionMonitor:
    name: str
    check_function: Callable
    frequency: float  # Hz
    last_check: float = 0
    error_threshold: float = 0.5

class VLAExecutionMonitor(Node):
    def __init__(self):
        super().__init__('vla_execution_monitor')
        
        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.emergency_stop_pub = self.create_publisher(Bool, '/emergency_stop', 10)
        self.monitor_status_pub = self.create_publisher(String, '/monitor_status', 10)
        
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # Monitor configuration
        self.monitors: List[ExecutionMonitor] = []
        self.scan_data = None
        self.obstacle_detected = False
        self.safety_violation = False
        
        # Add default monitors
        self.add_monitor(ExecutionMonitor(
            name="obstacle_monitor",
            check_function=self.check_for_obstacles,
            frequency=10.0,  # Check 10 times per second
            error_threshold=0.5  # Minimum distance to consider obstacle
        ))
        
        self.add_monitor(ExecutionMonitor(
            name="timeout_monitor",
            check_function=self.check_for_timeouts,
            frequency=1.0,  # Check once per second
            error_threshold=30.0  # Max execution time in seconds
        ))
        
        # Task tracking
        self.current_task_start_time = time.time()
        self.last_task_update = time.time()
        
        # Timer for monitoring
        self.monitor_timer = self.create_timer(0.1, self.run_monitors)
        
        self.get_logger().info("VLA Execution Monitor Node initialized")
    
    def add_monitor(self, monitor: ExecutionMonitor):
        """Add a monitor to the monitoring system"""
        self.monitors.append(monitor)
        self.get_logger().info(f"Added monitor: {monitor.name}")
    
    def scan_callback(self, msg):
        """Store the latest scan data"""
        self.scan_data = msg
    
    def run_monitors(self):
        """Run all active monitors"""
        current_time = time.time()
        
        for monitor in self.monitors:
            # Check if it's time to run this monitor
            if (current_time - monitor.last_check) >= (1.0 / monitor.frequency):
                try:
                    # Run the monitor's check function
                    result = monitor.check_function()
                    
                    if result and result['error']:
                        self.handle_monitor_error(monitor, result)
                    
                    monitor.last_check = current_time
                except Exception as e:
                    self.get_logger().error(f"Error in monitor {monitor.name}: {e}")
    
    def check_for_obstacles(self):
        """Check for obstacles in the robot's path"""
        if not self.scan_data:
            return {'error': False, 'message': 'No scan data available'}
        
        # Check forward-facing laser readings
        scan = self.scan_data
        center_idx = len(scan.ranges) // 2
        forward_range_indices = range(center_idx - 10, center_idx + 10)
        
        for idx in forward_range_indices:
            if 0 <= idx < len(scan.ranges):
                if scan.range_min <= scan.ranges[idx] <= 0.5:  # 0.5m threshold
                    self.obstacle_detected = True
                    return {
                        'error': True, 
                        'message': f'Obstacle detected at {scan.ranges[idx]:.2f}m',
                        'distance': scan.ranges[idx]
                    }
        
        self.obstacle_detected = False
        return {'error': False, 'message': 'No obstacles detected'}
    
    def check_for_timeouts(self):
        """Check if the current task is taking too long"""
        elapsed_time = time.time() - self.current_task_start_time
        
        if elapsed_time > 30.0:  # 30-second timeout
            return {
                'error': True, 
                'message': f'Task timeout: {elapsed_time:.2f}s elapsed',
                'elapsed': elapsed_time
            }
        
        return {'error': False, 'message': f'Task running normally: {elapsed_time:.2f}s elapsed'}
    
    def handle_monitor_error(self, monitor: ExecutionMonitor, error_result):
        """Handle an error detected by a monitor"""
        self.get_logger().warn(f"Monitor {monitor.name} detected error: {error_result['message']}")
        
        # Publish status
        status_msg = String()
        status_msg.data = f"MONITOR_ALERT: {monitor.name} - {error_result['message']}"
        self.monitor_status_pub.publish(status_msg)
        
        # Take appropriate action based on monitor type
        if monitor.name == "obstacle_monitor":
            self.handle_obstacle_detected(error_result)
        elif monitor.name == "timeout_monitor":
            self.handle_timeout_detected(error_result)
    
    def handle_obstacle_detected(self, error_result):
        """Handle obstacle detection"""
        self.get_logger().info("Obstacle detected, stopping robot")
        
        # Stop the robot
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        
        # If obstacle is very close, trigger emergency stop
        if error_result.get('distance', float('inf')) < 0.3:
            self.get_logger().error("Very close obstacle detected, triggering emergency stop")
            emergency_msg = Bool()
            emergency_msg.data = True
            self.emergency_stop_pub.publish(emergency_msg)
    
    def handle_timeout_detected(self, error_result):
        """Handle task timeout"""
        self.get_logger().error("Task timeout detected, stopping robot")
        
        # Stop the robot
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        
        # Publish emergency stop
        emergency_msg = Bool()
        emergency_msg.data = True
        self.emergency_stop_pub.publish(emergency_msg)
    
    def reset_task_timer(self):
        """Reset the task timer when a new task starts"""
        self.current_task_start_time = time.time()
        self.last_task_update = time.time()
        self.safety_violation = False

def main(args=None):
    rclpy.init(args=args)
    node = VLAExecutionMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down VLA Execution Monitor Node")
    finally:
        # Stop the robot
        stop_cmd = Twist()
        node.cmd_vel_pub.publish(stop_cmd)
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Summary
In this chapter, we explored action planning and execution systems that integrate vision, language, and physical action. We learned about:

- Multimodal action planning that combines visual perception and language understanding to generate robot behavior
- Hierarchical task planning approaches that decompose complex tasks into manageable subtasks
- Execution monitoring systems that ensure tasks are performed safely and correctly
- Recovery mechanisms for handling failures and unexpected situations

VLA-based action planning systems represent a significant advancement in robotics, enabling robots to understand complex, natural language commands and execute them in real-world environments. These systems integrate multiple sources of information to create more capable and intuitive robotic assistants.

The implementation of such systems requires careful attention to:
- Real-time processing requirements
- Safety and reliability considerations
- Error handling and recovery strategies
- Integration with existing robotic platforms

These systems form the foundation for advanced robotic capabilities that can understand and respond to human commands in complex, dynamic environments.

## Exercises

### Logical Analysis Exercise
1. Analyze the computational requirements of VLA-based action planning vs. traditional symbolic planning approaches.
2. Evaluate the safety considerations in autonomous execution of complex tasks based on natural language commands.

### Conceptual Exploration Exercise
1. Research how reinforcement learning can be integrated with VLA systems for adaptive task planning.
2. Investigate how large language models (like GPT) can enhance action planning in robotics.

### Implementation Practice Exercise
1. Implement a VLA-based system that can execute multi-step household tasks based on natural language commands.
2. Create a hierarchical planner that can handle dependencies between different actions.
3. Develop an execution monitoring system with multiple safety checks and recovery procedures.
4. Build a complete VLA system that integrates with a mobile manipulator robot.

## References
1. Task Planning in Robotics: https://ieeexplore.ieee.org/document/8982492
2. Vision-Language Models for Robotics: https://arxiv.org/abs/2302.12189
3. Execution Monitoring in Robotics: https://www.sciencedirect.com/science/article/pii/S0921889019305379