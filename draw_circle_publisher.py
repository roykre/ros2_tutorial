#!/usr/bin/env python3

# Import the core ROS 2 Python client library
import rclpy
from rclpy.node import Node

# Import the Twist message type for velocity commands
from geometry_msgs.msg import Twist

# Define a custom Node class to publish velocity commands that make the turtle draw a circle
class DrawCircleNode(Node):
    def __init__(self):
        # Initialize the node with the name 'draw_circle'
        super().__init__("draw_circle")

        # Create a publisher that sends Twist messages to the /turtle1/cmd_vel topic
        # The queue size is set to 10
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

        # Create a timer that calls sent_velocity_command every 0.5 seconds
        self.timer_ = self.create_timer(0.5, self.sent_velocity_command)

        # Log a message to indicate that the node has started
        self.get_logger().info("draw circle node has been started")

    # This function sends a velocity command to make the turtle move in a circle
    def sent_velocity_command(self):
        msg = Twist()
        msg.linear.x = 2.0    # Move forward with linear velocity
        msg.angular.z = 1.0   # Rotate with angular velocity
        self.cmd_vel_pub_.publish(msg)

# Entry point of the program
def main(args=None):
    rclpy.init(args=args)       # Initialize ROS 2 communication
    node = DrawCircleNode()     # Create an instance of the node
    rclpy.spin(node)            # Keep the node alive and responsive
    rclpy.shutdown()            # Cleanly shut down ROS 2 when done

