#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from turtlesim.msg import Pose
from turtlesim_plus_interfaces.srv import GivePosition
from turtlesim.srv import Kill , Spawn

import numpy as np
import math

class controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.create_subscription(Point, '/target', self.target_callback, 10)
        self.spawn_pizza_client = self.create_client(GivePosition, 'spawn_pizza')
        
        self.robot_pose = np.array([0.0, 0.0, 0.0])  # [x, y, theta]
        self.target_queue = []  # Queue to hold targets
        self.target_threshold = 0.1  # Threshold to determine if target is reached
        self.declare_parameter('auto_move_active', False)
        self.auto_move_active = False

        self.create_timer(0.1, self.timer_callback)

    def pose_callback(self, msg):
        self.robot_pose[0] = msg.x
        self.robot_pose[1] = msg.y
        self.robot_pose[2] = msg.theta

    def target_callback(self, msg):
        self.target_queue.append(np.array([msg.x, msg.y]))  # Add new target to the queue
        if not self.auto_move_active:
            self.auto_move_active = True

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def publish_cmd_vel(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)  # Ensure the value is a float
        msg.angular.z = float(angular)  # Ensure the value is a float
        self.cmd_vel_pub.publish(msg)
        
    def spawn_pizza(self, x, y):
        position_request = GivePosition.Request()
        position_request.x = x
        position_request.y = y
        self.spawn_pizza_client.call_async(position_request)

    def timer_callback(self):
        if not self.auto_move_active or not self.target_queue:
            return  # No targets to move towards or auto-move is not active

        current_target = self.target_queue[0]  # Get the first target in the queue
        dx = current_target[0] - self.robot_pose[0]
        dy = current_target[1] - self.robot_pose[1]
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance < self.target_threshold:
            self.publish_cmd_vel(0.0, 0.0)  # Stop movement
            
            # Check if the current target is not the final position (10, 10)
            if current_target[0] != 10 or current_target[1] != 10:
                self.spawn_pizza(current_target[0], current_target[1])  # Spawn pizza at the reached target
            
            self.target_queue.pop(0)  # Remove the reached target from the queue
            
            if not self.target_queue:
                # If there are no more targets, set a new target to (10, 10)
                self.target_queue.append((10, 10))  # Append new target coordinates
                print("New target set at (10, 10).")
                self.auto_move_active = True  # Ensure auto-move is reactivated

            return

        target_angle = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(target_angle - self.robot_pose[2])

        # Adjust speed based on proximity
        linear_speed = min(5.0 * distance, 8.0)  # Ramp down speed as it approaches the target
        angular_speed = min(10.0 * angle_diff, 10.0)

        self.publish_cmd_vel(linear_speed, angular_speed)



def main(args=None):
    rclpy.init(args=args)
    node = controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
