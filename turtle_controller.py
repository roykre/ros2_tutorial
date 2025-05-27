#!/usr/bin/env python3 

# Import ROS 2 Python client libraries
import rclpy
from rclpy.node import Node

# Import ROS message types
from turtlesim.msg import Pose           # For receiving turtle position updates
from geometry_msgs.msg import Twist      # For sending movement commands

# Define a Node class that controls the turtle
class TurtleControllerNode(Node):
    
    def __init__(self):
        # Initialize the node with the name "turtle_controller"
        super().__init__("turtle_controller")

        # Create a publisher to the /turtle1/cmd_vel topic (for controlling turtle velocity)
        self.cmd_vel_publisher = self.create_publisher(
            Twist, "/turtle1/cmd_vel", 10
        )

        # Create a subscriber to the /turtle1/pose topic (to receive turtle position)
        self.pose_subscriber_ = self.create_subscription(
            Pose, "/turtle1/pose", self.pose_callback, 10
        )

        # Log that the controller has started
        self.get_logger().info("Turtle controller has been started")

    # Callback function to respond to pose updates
    def pose_callback(self, pose: Pose):
        cmd = Twist()  # Create a new Twist message to hold velocity commands

        # If the turtle is near the border, make it turn
        if pose.x > 9.0 or pose.x < 2.0 or pose.y > 9.0 or pose.y < 2.0:
            cmd.linear.x = 1.0       # Move forward slowly
            cmd.angular.z = 0.9      # Start turning
        else:
            cmd.linear.x = 5.0       # Move forward fast
            cmd.angular.z = 0.0      # Go straight

        # Publish the command to the turtle
        self.cmd_vel_publisher.publish(cmd)
        
# Main function to run the ROS 2 node
def main(args=None):
    rclpy.init(args=args)                   # Initialize ROS 2
    node = TurtleControllerNode()           # Create an instance of the node
    rclpy.spin(node)                        # Keep the node alive and listening
    rclpy.shutdown()                        # Shutdown ROS gracefully when done
