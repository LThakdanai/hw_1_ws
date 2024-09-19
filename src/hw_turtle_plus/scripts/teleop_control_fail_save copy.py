#!/usr/bin/python3

from hw_turtle_plus.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

from pynput import keyboard  # Import the pynput library for key presses

class teleop_control(Node):
    def __init__(self):
        super().__init__('teleop_control')
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.velocity = 0.0
        self.angular = 0.0

        self.create_timer(0.01, self.timer_callback)

        # Declare the auto_move_active parameter
        self.declare_parameter('auto_move_active', False)

        # Set up a keyboard listener
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

    def cmdvel(self, v, w):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.cmd_vel_pub.publish(msg)

    def timer_callback(self):
        # Check if auto_move is active
        auto_move_active = self.get_parameter('auto_move_active').get_parameter_value().bool_value

        if not auto_move_active:
            # Publish the velocity commands set by key presses
            self.cmdvel(self.velocity, self.angular)
        else:
            # Auto-move is active; do not publish velocity commands
            pass

    def on_press(self, key):
        try:
            if key.char == 'w':
                self.velocity = 1.0  # Move forward
            elif key.char == 's':
                self.velocity = -1.0  # Move backward
            elif key.char == 'a':
                self.angular = 1.0  # Turn left
            elif key.char == 'd':
                self.angular = -1.0  # Turn right
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
    node = teleop_control()
    rclpy.spin(node)
    node.listener.stop()  # Stop the keyboard listener when the node shuts down
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
