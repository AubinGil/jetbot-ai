#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import py_trees
from py_trees.trees import BehaviourTree
from py_trees.composites import Sequence, Selector, Parallel
from py_trees.behaviours import Running, Success, Failure
from py_trees import common, blackboard


class CheckObstacle(py_trees.behaviour.Behaviour):
    """Check if obstacle is detected from VLM"""
    def __init__(self, name, node):
        super().__init__(name)
        self.node = node
        self.blackboard = blackboard.Client()
        self.blackboard.register_key('obstacle_info', access=py_trees.common.Access.READ)
    
    def update(self):
        obstacle_info = self.blackboard.get('obstacle_info')
        if obstacle_info and 'YES' in obstacle_info.upper():
            self.node.get_logger().info('Obstacle detected!')
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class FindClearPath(py_trees.behaviour.Behaviour):
    """Determine clear path direction from VLM"""
    def __init__(self, name, node):
        super().__init__(name)
        self.node = node
        self.blackboard = blackboard.Client()
        self.blackboard.register_key('obstacle_info', access=py_trees.common.Access.READ)
        self.blackboard.register_key('turn_direction', access=py_trees.common.Access.WRITE)
    
    def update(self):
        obstacle_info = self.blackboard.get('obstacle_info')
        if not obstacle_info:
            return py_trees.common.Status.FAILURE
        
        # Parse VLM response for clear direction
        if 'LEFT' in obstacle_info.upper():
            self.blackboard.set('turn_direction', 'left')
        elif 'RIGHT' in obstacle_info.upper():
            self.blackboard.set('turn_direction', 'right')
        else:
            self.blackboard.set('turn_direction', 'right')  # default
        
        self.node.get_logger().info(f'Clear path: {self.blackboard.get("turn_direction")}')
        return py_trees.common.Status.SUCCESS


class MoveForward(py_trees.behaviour.Behaviour):
    """Move robot forward"""
    def __init__(self, name, node):
        super().__init__(name)
        self.node = node
    
    def update(self):
        twist = Twist()
        twist.linear.x = 0.2  # m/s
        twist.angular.z = 0.0
        self.node.cmd_vel_pub.publish(twist)
        self.node.get_logger().info('Moving forward')
        return py_trees.common.Status.RUNNING


class TurnToAvoid(py_trees.behaviour.Behaviour):
    """Turn robot to avoid obstacle"""
    def __init__(self, name, node):
        super().__init__(name)
        self.node = node
        self.blackboard = blackboard.Client()
        self.blackboard.register_key('turn_direction', access=py_trees.common.Access.READ)
        self.turn_time = 0
        self.max_turn_time = 20  # ticks
    
    def initialise(self):
        self.turn_time = 0
    
    def update(self):
        direction = self.blackboard.get('turn_direction')
        angular_z = 0.5 if direction == 'left' else -0.5
        
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = angular_z
        self.node.cmd_vel_pub.publish(twist)
        
        self.turn_time += 1
        if self.turn_time >= self.max_turn_time:
            self.node.get_logger().info(f'Turned {direction}')
            return py_trees.common.Status.SUCCESS
        
        return py_trees.common.Status.RUNNING


class StopRobot(py_trees.behaviour.Behaviour):
    """Stop the robot"""
    def __init__(self, name, node):
        super().__init__(name)
        self.node = node
    
    def update(self):
        twist = Twist()
        self.node.cmd_vel_pub.publish(twist)
        return py_trees.common.Status.SUCCESS


class BehaviorTreeNode(Node):
    def __init__(self):
        super().__init__('behavior_tree_node')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.obstacle_sub = self.create_subscription(
            String,
            '/vlm/obstacle_detection',
            self.obstacle_callback,
            10
        )
        
        self.scene_sub = self.create_subscription(
            String,
            '/vlm/scene_description',
            self.scene_callback,
            10
        )
        
        # Blackboard setup
        self.blackboard = blackboard.Client()
        self.blackboard.register_key('obstacle_info', access=py_trees.common.Access.WRITE)
        self.blackboard.register_key('scene_info', access=py_trees.common.Access.WRITE)
        self.blackboard.set('obstacle_info', '')
        self.blackboard.set('scene_info', '')
        
        # Build behavior tree
        self.tree = self.create_behavior_tree()
        
        # Timer for tree execution
        self.timer = self.create_timer(0.1, self.tick_tree)  # 10 Hz
        
        self.get_logger().info('Behavior Tree Node started')
    
    def obstacle_callback(self, msg):
        self.blackboard.set('obstacle_info', msg.data)
    
    def scene_callback(self, msg):
        self.blackboard.set('scene_info', msg.data)
    
    def create_behavior_tree(self):
        """Create the navigation behavior tree"""
        
        # Root: Selector (try obstacle avoidance, else move forward)
        root = py_trees.composites.Selector(name="Navigation", memory=False)
        
        # Branch 1: Obstacle avoidance sequence
        obstacle_sequence = py_trees.composites.Sequence(name="Avoid Obstacle", memory=True)
        obstacle_sequence.add_children([
            CheckObstacle("Check Obstacle", self),
            StopRobot("Stop", self),
            FindClearPath("Find Clear Path", self),
            TurnToAvoid("Turn to Avoid", self)
        ])
        
        # Branch 2: Move forward (default)
        move_forward = MoveForward("Move Forward", self)
        
        # Add branches to root
        root.add_children([obstacle_sequence, move_forward])
        
        # Create tree
        tree = py_trees.trees.BehaviourTree(root)
        
        return tree
    
    def tick_tree(self):
        """Tick the behavior tree"""
        self.tree.tick()


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorTreeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop robot
        twist = Twist()
        node.cmd_vel_pub.publish(twist)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
