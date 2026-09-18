"""
Task 1:
Publish one safe, zero-velocity TwistStamped message to /cmd_vel.

Manual verification:
1. In Terminal A, run:
   ros2 topic echo /cmd_vel --once

2. In Terminal B, run:
   ros2 run day2pkg node1

Expected result:
Terminal A prints one geometry_msgs/msg/TwistStamped message with
twist.linear.x == 0.0 and twist.angular.z == 0.0.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


class Node1(Node):
    def __init__(self):
        super().__init__('node1')  # inherit from Node class and give it a name 'node1'

        # create a publisher
        self.publisher_ = self.create_publisher(
            TwistStamped,  # message type
            '/cmd_vel',  # topic name
            10   # queue size
        )

        self.sent = False
        self.finished = False
        self.publish_timer = self.create_timer(2.0, self.publish_once)
        self.shutdown_timer = None

    def publish_once(self):
        '''
        Publish a single zero-velocity TwistStamped message to the cmd_vel topic.
        '''
        message = TwistStamped()  # create a new TwistStamped message

        # set the timestamp to the current time
        message.header.stamp = self.get_clock().now().to_msg()

        # set the linear and angular velocities to zero
        message.twist.linear.x = 0.0
        message.twist.angular.z = 0.0

        # publish the message
        self.publisher_.publish(message)

        # log the publication
        self.get_logger().info('published one zero-velocity TwistStamped message')

        self.sent = True
        self.publish_timer.cancel()
        self.shutdown_timer = self.create_timer(2.0, self.finish)

    def finish(self):
        """Allow time for delivery, then let main() stop the node."""
        self.finished = True
        self.shutdown_timer.cancel()


def main(args=None):
    rclpy.init(args=args)  # initialize the ROS 2 Python client library

    # create an instance of Node1
    node = Node1()

    while rclpy.ok() and not node.finished:
        rclpy.spin_once(node)

    # publish a single zero-velocity TwistStamped message
    # node.publish_once()
    # destroy the node explicitly
    node.destroy_node()

    rclpy.shutdown()  # shutdown the ROS 2 Python client library