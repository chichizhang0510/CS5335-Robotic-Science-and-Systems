"""
Task 2:
Run a timer callback once per second three times.
Publish a low forward velocity each time, then publish a zero-velocity
TwistStamped message and destroy the node.

Problem:
Terminal logs indicate that the time interval between the three callbacks was approximately one second, and a velocity command of `linear.x = 0.1` was issued each time; following the third callback, a `TwistStamped` stop message (with all-zero values) was also published.

However, the actual TurtleBot sometimes appears to move only once or twice, rather than visibly moving three times.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


class Node2(Node):
    def __init__(self):
        super().__init__('node2')  # inherit from Node class and give it a name 'node2'

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

        self.publish_velocity(0.1, 0.0)
        self.get_logger().info(
            f'callback {self.callback_count}/3: moving forward at 0.1 m/s'
        )

        if self.callback_count == 3:
            self.publish_velocity(0.0, 0.0)
            self.get_logger().info('published stop command')

            self.timer.cancel()
            self.finished = True

    def publish_velocity(self, linear_x, angular_z):
        '''
        Publish a TwistStamped message with the specified linear and angular velocities.
        '''
        message = TwistStamped()  # create a new TwistStamped message

        # set the timestamp to the current time
        message.header.stamp = self.get_clock().now().to_msg()

        # set the linear and angular velocities
        message.twist.linear.x = linear_x
        message.twist.angular.z = angular_z

        # publish the message
        self.publisher_.publish(message)


def main(args=None):
    rclpy.init(args=args)  # initialize the ROS 2 Python client library

    # create an instance of Node2
    node = Node2()

    while rclpy.ok() and not node.finished:
        rclpy.spin_once(node)

    # destroy the node explicitly
    node.destroy_node()

    rclpy.shutdown()  # shutdown the ROS 2 Python client library