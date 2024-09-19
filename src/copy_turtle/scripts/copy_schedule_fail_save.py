#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import yaml
import os
from geometry_msgs.msg import Point

class CopySchedule(Node):
    def __init__(self):
        super().__init__('copy_schedule')
        # Create a publisher for the topic /target
        self.target_publisher = self.create_publisher(Point, '/target', 10)
        
        # Load and process the YAML file
        self.load_and_publish_positions()

    def load_and_publish_positions(self):
        # Specify the path to your YAML file in the home directory
        yaml_file_path = os.path.expanduser('~/pizza_positions.yaml')  # This will resolve to the home directory

        if not os.path.exists(yaml_file_path):
            self.get_logger().error(f"YAML file '{yaml_file_path}' not found.")
            return

        with open(yaml_file_path, 'r') as file:
            try:
                data = yaml.safe_load(file)
            except yaml.YAMLError as exc:
                self.get_logger().error(f"Error parsing YAML file: {exc}")
                return

        # Process positions with save_count = 1
        for entry in data.get('positions', []):
            if entry.get('save_count') == 1:  # Change this to match the desired save_count
                new_pizzas = entry.get('new_pizzas', [])
                for pos in new_pizzas:
                    point_msg = Point()
                    point_msg.x = pos.get('x', 0.0)
                    point_msg.y = pos.get('y', 0.0)
                    point_msg.z = 0.0  # Assuming z is not used
                    self.target_publisher.publish(point_msg)
                    self.get_logger().info(f"Published position - x: {point_msg.x}, y: {point_msg.y}")

def main(args=None):
    rclpy.init(args=args)
    node = CopySchedule()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
