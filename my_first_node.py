#!/usr/bin/env python3 

# Import ROS 2 Python client library
import rclpy
from rclpy.node import Node

# Define a custom Node class
class MyNode(Node):

    def __init__(self):
        # Initialize the node with the name 'first_node'
        super().__init__("first_node")

        # Log an initial message when the node starts
        self.get_logger().info("hello word")

        # Create a timer that triggers the callback every 1.0 second
        self.create_timer(1.0, self.timer_callback)

    # Timer callback function that runs periodically
    def timer_callback(self):
        self.get_logger().info("hello")  # Print "hello" every second


# The main function that runs when the script is executed
def main(args=None):
    rclpy.init(args=args)     # Initialize the ROS 2 Python client library
    node = MyNode()           # Create an instance of MyNode
    rclpy.spin(node)          # Keep the node running, handling callbacks
    rclpy.shutdown()          # Clean up and shut down ROS when done


# Ensure the main function is called when the script is executed directly
if __name__ == '__main__':
    main()

