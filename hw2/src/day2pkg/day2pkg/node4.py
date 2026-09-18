"""
Task 4:
Make your Node drive the Turtlebot 50 cm
Make your Node drive the Turtlebot 1m
Make your Node drive the Turtlebot in a circle with a 0.5m radius
Make your Node drive 90 degrees of a circle with a 0.5m radius
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


class Node4(Node):
    def __init__(self):
        super().__init__('node4')

        # create a publisher
        self.publisher_ = self.create_publisher(
            TwistStamped,  # message type
            '/cmd_vel',  # topic name
            10   # queue size
        )

        self.callback_count = 0
        self.finished = False
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        """Create a TwistStamped message with a low forward velocity and publish it."""
        self.callback_count += 1

        # if self.callback_count <= 50:
        # if self.callback_count <= 100:
            # message = self.CreateTwist(0.1, 0.0)
        # if self.callback_count <= 315:
        if self.callback_count <= 79:
            message = self.CreateTwist(0.1, 0.2)
        else:
            message = self.CreateTwist(0.0, 0.0)
            # self.get_logger().info('50 cm complete: published stop command')
            # self.get_logger().info('100 cm complete: published stop command')
            # self.get_logger().info('0.5m radius circle complete: published stop command')
            self.get_logger().info('90 degree 0.5m radius circle complete: published stop command')

            self.timer.cancel()
            self.finished = True

        self.publisher_.publish(message)
    
    def CreateTwist(self, linear_x, angular_z):
        """Create a TwistStamped with linear and angular velocity."""
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.twist.linear.x = linear_x
        twist.twist.angular.z = angular_z
        return twist


def main(args=None):
    rclpy.init(args=args)

    node = Node4()

    while rclpy.ok() and not node.finished:
        rclpy.spin_once(node)

    # destroy the node explicitly
    node.destroy_node()

    rclpy.shutdown()  # shutdown the ROS 2 Python client library