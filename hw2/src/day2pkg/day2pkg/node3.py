"""
Task 3:
Make a function CreateLinearTwist(x) to create a TwistStamped with the given value as Linear x
Make a function CreateAngularTwist(z) to create a TwistStamped with the given value as Angular z
Now modify your callback to use these functions, and change the speed and/or angle as your Node executes
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


class Node3(Node):
    def __init__(self):
        super().__init__('node3')

        # create a publisher
        self.publisher_ = self.create_publisher(
            TwistStamped,  # message type
            '/cmd_vel',  # topic name
            10   # queue size
        )

        self.callback_count = 0
        self.finished = False
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        """Create a TwistStamped message with a low forward velocity and publish it."""
        self.callback_count += 1

        if self.callback_count <= 3:
            message = self.CreateLinearTwist(0.1)
            self.get_logger().info(f'Publishing linear twist: {message.twist.linear.x}')
        elif self.callback_count <= 6:
            message = self.CreateAngularTwist(0.3)
            self.get_logger().info(f'Publishing angular twist: {message.twist.angular.z}')
        else:
            message = self.CreateLinearTwist(0.0)
            self.get_logger().info('published stop command')

            self.timer.cancel()
            self.finished = True

        self.publisher_.publish(message)

    def CreateLinearTwist(self, x):
        """Create a TwistStamped message with a linear velocity in the x direction."""
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.twist.linear.x = x
        twist.twist.angular.z = 0.0
        return twist

    def CreateAngularTwist(self, z):
        """Create a TwistStamped message with an angular velocity in the z direction."""
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.twist.linear.x = 0.0
        twist.twist.angular.z = z
        return twist


def main(args=None):
    rclpy.init(args=args)  # initialize the ROS 2 Python client library

    node = Node3()

    while rclpy.ok() and not node.finished:
        rclpy.spin_once(node)

    # destroy the node explicitly
    node.destroy_node()

    rclpy.shutdown()  # shutdown the ROS 2 Python client library