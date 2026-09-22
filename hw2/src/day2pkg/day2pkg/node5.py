"""
Task 5 and 6:
Make a function DriveStraight (d) that drives your Turtlebot for d meters in a straight line.
Make a function DriveCircle(r) that drives your Turtlebot in a circle of radius r (meters).
Make a function DriveArc(r,d) that drives your Turtlebot in a circular arc of d degrees and radius r (meters).

Drive a path like the letter D - straight for 1 m, then turn 90 degrees to the right, 
then drive in a half-circle with a 0.5 m radius, then turn 90 degrees to the right 
so your Turtlebot is back in its starting position and orientation.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

import math


class Node5(Node):
    def __init__(self):
        super().__init__('node5')

        # create a publisher
        self.publisher_ = self.create_publisher(
            TwistStamped,  # message type
            '/cmd_vel',  # topic name
            10   # queue size
        )

        self.stage = 0
        self.segment_count = 0

        self.finished = False
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        """Create a TwistStamped message with a low forward velocity and publish it."""
        self.segment_count += 1

        if self.stage == 0:
            message = self.DriveStraight(1.0)
        elif self.stage == 1:
            message = self.RotateInPlace(-90)
        elif self.stage == 2:
            message = self.DriveArc(0.5, -180)
        else:
            message = self.RotateInPlace(-90)

        if message is None:
            # Explicitly stop between path segments.
            self.publisher_.publish(self.CreateTwist(0.0, 0.0))

            self.stage += 1
            self.segment_count = 0

            if self.stage == 4:
                self.get_logger().info('D path complete: published stop command')
                self.timer.cancel()
                self.finished = True

            return

        self.publisher_.publish(message)
    
    def CreateTwist(self, linear_x, angular_z):
        """Create a TwistStamped with linear and angular velocity."""
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.twist.linear.x = linear_x
        twist.twist.angular.z = angular_z
        return twist

    def DriveStraight(self, distance):
        """Drive the Turtlebot straight for a specified distance in meters."""
        target_count = math.ceil(abs(distance) * 100 + 9)

        if self.segment_count <= target_count:
            linear_x = 0.1 if distance >= 0 else -0.1
            return self.CreateTwist(linear_x, 0.0)

        return None

    def DriveCircle(self, radius):
        """Drive the Turtlebot in a circle with a specified radius in meters."""
        '''if self.callback_count <= 2 * 3.14159 * radius * 100:
            message = self.CreateTwist(0.1, 0.1 / radius)
        else:
            message = self.CreateTwist(0.0, 0.0)
            self.get_logger().info(f'Circle with radius {radius} meters complete: published stop command')
            self.timer.cancel()
            self.finished = True'''
        return self.DriveArc(radius, 360)

    def DriveArc(self, radius, degrees):
        """Drive the Turtlebot in a circular arc with a specified radius and degrees."""
        if radius <= 0:
            raise ValueError('radius must be greater than zero')

        if degrees == 0:
            raise ValueError('degrees must not be zero')

        arc_length = abs(degrees) / 360.0 * 2 * math.pi * radius
        # target_count = math.ceil(arc_length * 100)
        target_count = math.ceil(arc_length / (0.15 * 0.1))

        if self.segment_count <= target_count:
            # angular_z = 0.1 / radius
            angular_z = 0.15 / radius

            if degrees < 0:
                angular_z = -angular_z

            # return self.CreateTwist(0.1, angular_z)
            return self.CreateTwist(0.15, angular_z)

        return None

    def RotateInPlace(self, degrees):
        """Return in-place rotation commands for degrees."""
        target_count = math.ceil(
            math.radians(abs(degrees)) / (0.3 * 0.1)
        )

        if self.segment_count <= target_count:
            angular_z = 0.3 if degrees >= 0 else -0.3
            return self.CreateTwist(0.0, angular_z)

        return None


def main(args=None):
    rclpy.init(args=args)

    node = Node5()

    while rclpy.ok() and not node.finished:
        rclpy.spin_once(node)

    # destroy the node explicitly
    node.destroy_node()

    rclpy.shutdown()  # shutdown the ROS 2 Python client library