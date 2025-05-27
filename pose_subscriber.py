#!/usr/bin/env python3 

# Import the ROS 2 Python client library
import rclpy
from rclpy.node import Node

# Import the Pose message from turtlesim
from turtlesim.msg import Pose

# Define a node class that subscribes to the /turtle1/pose topic
class PoseSubscriberNode(Node):

    def __init__(self):
        # Initialize the node with the name "pose_subscriber"
        super().__init__("pose_subscriber")

        # Create a subscription to the /turtle1/pose topic with queue size 10
        # When a message is received, self.pose_callback will be called
        self.pose_subscriber_ = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )

    # This callback is called whenever a new Pose message is received
    def pose_callback(self, msg: Pose):
        # Log the full message and y-coordinate to the console
        self.get_logger().info("(" + str(msg) + ", " + str(msg.y) + ")")

# Main function to initialize the ROS client library, start the node, and keep it spinning
def main(args=None):
    rclpy.init(args=args)               # Initialize ROS
    node = PoseSubscriberNode()         # Create the node
    rclpy.spin(node)                    # Keep the node running and listening
    rclpy.shutdown()                   # Clean shutdown after node is stopped
