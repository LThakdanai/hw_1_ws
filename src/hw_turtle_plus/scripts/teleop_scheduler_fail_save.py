#!/usr/bin/python3

from hw_turtle_plus.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim_plus_interfaces.srv import GivePosition
from std_srvs.srv import Empty

import numpy as np
from pynput import keyboard  # Import the pynput library for key presses
import yaml  # Library for saving/loading YAML files
import os  # For checking if file exists

class teleop_scheduler(Node):
    def __init__(self):
        super().__init__('teleop_scheduler')
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.spawn_pizza_client = self.create_client(GivePosition, 'spawn_pizza')
       
        # Declare the auto_move_active parameter
        self.declare_parameter('auto_move_active', False)

        # ROS 2 parameter for pizza limit
        self.declare_parameter('pizza_limit', 30)
        self.pizza_limit = self.get_parameter('pizza_limit').get_parameter_value().integer_value

        self.pizza_positions = []  # Track positions of spawned pizzas
        self.pizza_count = 0  # Track number of pizzas spawned
        self.robot_pose = np.array([0.0, 0.0, 0.0])  # x, y, theta
        self.velocity = 0.0
        self.angular = 0.0
        self.create_timer(0.01, self.timer_callback)
        self.eat_pizza_client = self.create_client(Empty, '/turtle1/eat')

        self.yaml_file = "pizza_positions.yaml"  # File to store pizza positions
        self.save_count = 0  # Track the number of saves
        self.max_saves = 4  # Limit the number of saves to 4
        self.last_saved_pizza_count = 0  # Track how many pizzas were saved last time

        # Set up a keyboard listener
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

        # Load previous pizza positions from YAML file if it exists
        self.load_pizza_positions()

        # Variables for automatic movement
        self.auto_move = False
        self.target_positions = []
        self.current_target_index = 0

    def spawn_pizza(self, x, y):
        if self.pizza_count < self.pizza_limit:
            if self.spawn_pizza_client.wait_for_service(timeout_sec=1.0):
                position_request = GivePosition.Request()
                position_request.x = x
                position_request.y = y
                self.spawn_pizza_client.call_async(position_request)
                self.pizza_count += 1  # Increment pizza count
                pizza_pos = {'x': x, 'y': y}
                if pizza_pos not in self.pizza_positions:
                    self.pizza_positions.append(pizza_pos)
                self.get_logger().info(f"Spawning pizza at ({x}, {y}). Pizza count: {self.pizza_count}")
            else:
                self.get_logger().warn("Spawn pizza service not available")
        else:
            self.get_logger().warn("Pizza spawn limit reached!")

    def save_pizza_positions(self):
        """Save pizza positions to a YAML file in a human-readable format."""
        # Check if save limit has been reached
        if self.save_count >= self.max_saves:
            self.get_logger().warn(f"Save limit of {self.max_saves} reached!")
            return

        # Only save pizzas that haven't been saved yet
        new_pizzas = self.pizza_positions[self.last_saved_pizza_count:]
        if not new_pizzas:
            self.get_logger().info("No new pizzas to save.")
            return

        # Convert numpy arrays to plain lists before saving
        pizza_positions_plain = [{'x': float(p['x']), 'y': float(p['y'])} for p in new_pizzas]

        # Create a dictionary where save_count is written first
        save_data = {
            'save_count': self.save_count + 1,  # Increment save count in file
            'new_pizzas': pizza_positions_plain  # Then write pizza positions
        }

        # Save data to the YAML file
        with open(self.yaml_file, 'a') as file:  # Append mode to keep track of multiple saves
            yaml.dump(save_data, file)

        self.last_saved_pizza_count = len(self.pizza_positions)  # Update last saved count
        self.save_count += 1  # Increment save count in memory

        self.get_logger().info(f"Pizza positions saved to {self.yaml_file}. Save count: {self.save_count}")

    def load_pizza_positions(self):
        """Load pizza positions from a YAML file if it exists."""
        if os.path.exists(self.yaml_file):
            with open(self.yaml_file, 'r') as file:
                data = yaml.safe_load(file) or []
                if isinstance(data, list):  # Handle case of a list-based YAML file
                    self.pizza_positions = data
                elif isinstance(data, dict):  # Handle new format with save count
                    self.pizza_positions = data.get('new_pizzas', [])
                    self.save_count = data.get('save_count', 0)
            self.last_saved_pizza_count = len(self.pizza_positions)  # Set the last saved pizza count
            self.get_logger().info(f"Loaded {len(self.pizza_positions)} pizza positions from file.")
        else:
            self.get_logger().info("No pizza positions file found. Starting fresh.")

    def pose_callback(self, msg):
        self.robot_pose = np.array([msg.x, msg.y, msg.theta])

    def cmdvel(self, v, w):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.cmd_vel_pub.publish(msg)

    def timer_callback(self):
        # Update pizza limit dynamically
        self.pizza_limit = self.get_parameter('pizza_limit').get_parameter_value().integer_value

        if self.auto_move:
            self.move_to_target()


    def eat_pizza(self):
        eat_request = Empty.Request()
        self.eat_pizza_client.call_async(eat_request)

    def start_eating_unsaved_pizzas(self):
        unsaved_pizzas = self.pizza_positions[self.last_saved_pizza_count:]
        if not unsaved_pizzas:
            self.get_logger().info("No unsaved pizzas to eat.")
            return
        self.target_positions = unsaved_pizzas.copy()
        self.current_target_index = 0
        self.auto_move = True
        # Set the parameter to True
        self.set_parameters([rclpy.parameter.Parameter('auto_move_active', rclpy.Parameter.Type.BOOL, True)])
        self.get_logger().info("Starting to eat unsaved pizzas.")

    def move_to_target(self):
        if self.current_target_index >= len(self.target_positions):
            self.get_logger().info("All unsaved pizzas have been eaten.")
            self.auto_move = False
            # Set the parameter back to False
            self.set_parameters([rclpy.parameter.Parameter('auto_move_active', rclpy.Parameter.Type.BOOL, False)])
            return
        target = self.target_positions[self.current_target_index]
        x_target = target['x']
        y_target = target['y']
        x_current = self.robot_pose[0]
        y_current = self.robot_pose[1]
        theta_current = self.robot_pose[2]

        # Compute the distance and angle to the target
        dx = x_target - x_current
        dy = y_target - y_current
        distance = np.sqrt(dx**2 + dy**2)
        angle_to_target = np.arctan2(dy, dx)
        angle_diff = self.normalize_angle(angle_to_target - theta_current)

        # If close enough to target, stop and eat pizza
        if distance < 0.1:
            self.cmdvel(0.0, 0.0)
            self.eat_pizza()
            self.get_logger().info(f"Ate pizza at ({x_target}, {y_target}).")

            # Decrement pizza count
            self.pizza_count -= 1

            # Remove the eaten pizza from pizza_positions
            self.pizza_positions.remove(target)

            self.current_target_index += 1
        else:
            # Simple proportional controller
            k_linear = 4.0
            k_angular = 12.0
            linear_speed = k_linear * distance
            angular_speed = k_angular * angle_diff
            # Limit speeds
            max_linear_speed = 5.0
            max_angular_speed = 8.0
            linear_speed = np.clip(linear_speed, -max_linear_speed, max_linear_speed)
            angular_speed = np.clip(angular_speed, -max_angular_speed, max_angular_speed)
            self.cmdvel(linear_speed, angular_speed)

    def normalize_angle(self, angle):
        while angle > np.pi:
            angle -= 2.0 * np.pi
        while angle < -np.pi:
            angle += 2.0 * np.pi
        return angle

    def on_press(self, key):
        try:
            if key.char == 'j':
                # Spawn pizza at the current position of the turtle
                x, y = self.robot_pose[0], self.robot_pose[1]
                self.spawn_pizza(x, y)
            elif key.char == 'k':
                if self.auto_move:
                    self.get_logger().warn("Cannot save pizzas while auto-move is active.")
                else:
                    # Save pizza positions when 'k' is pressed
                    self.save_pizza_positions()
            elif key.char == 'l':
                # Start moving to eat unsaved pizzas
                self.start_eating_unsaved_pizzas()
        except AttributeError:
            pass

    def on_release(self, key):
        try:
            # Stop the turtle when the key is released
            if key.char in ['w', 's']:
                self.velocity = 0.0
            if key.char in ['a', 'd']:
                self.angular = 0.0
        except AttributeError:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = teleop_scheduler()
    rclpy.spin(node)
    node.listener.stop()  # Stop the keyboard listener when the node shuts down
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
