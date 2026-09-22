import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import time
import math


class Node1(Node):
    def __init__(self):
        super().__init__('node1')
        self.publisherfield_ = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.speed = 0.15          # forward speed (m/s)
        self.dt = 0.1              # publish every 0.1 s

    def CreateLinearTwist(self, x):
        msg = TwistStamped()
        msg.twist.linear.x = x
        return msg

    def CreateAngularTwist(self, z):
        msg = TwistStamped()
        msg.twist.angular.z = z
        return msg

    def DriveStraight(self, d):
        duration = d / self.speed                 # seconds needed: distance / speed
        ticks = int(duration / self.dt)           # how many 0.1s publishes
        for i in range(ticks):
            self.publisherfield_.publish(self.CreateLinearTwist(self.speed))
            time.sleep(self.dt)
        self.publisherfield_.publish(self.CreateLinearTwist(0.0))   # brakes

    def DriveCircle(self, r):
        # radius = linear.x / angular.z  ->  angular.z = linear.x / r
        ang = self.speed / r                      # turn speed for this radius
        # a full circle = 360 deg = 2*pi radians of turning
        duration = (2 * math.pi) / ang            # time = total_angle / angular_speed
        ticks = int(duration / self.dt)
        for i in range(ticks):
            msg = self.CreateLinearTwist(self.speed)
            msg.twist.angular.z = ang           # BLANK 1: the turn speed you computed above
            self.publisherfield_.publish(msg)
            time.sleep(self.dt)
        self.publisherfield_.publish(self.CreateLinearTwist(0.0))

    def DriveArc(self, r, d):
        ang = self.speed / r
        radians = d * math.pi / 180                             # BLANK 2: convert d degrees -> radians
        duration = radians / ang
        ticks = int(duration / self.dt)
        for i in range(ticks):
            msg = self.CreateLinearTwist(self.speed)
            msg.twist.angular.z = ang
            self.publisherfield_.publish(msg)
            time.sleep(self.dt)
        self.publisherfield_.publish(self.CreateLinearTwist(0.0))

    def TurnInPlace(self, degrees):
        turn_speed = 0.5                          # rad/s, how fast to spin (pick a value)
        radians =  degrees * math.pi / 180                       # BLANK 1: convert degrees -> radians (same as DriveArc)
        duration = abs(radians) / turn_speed      # abs() so negative angles still give positive time
        ticks = int(duration / self.dt)
        # direction: positive degrees = left, negative = right
        direction = turn_speed if degrees > 0 else -turn_speed
        for i in range(ticks):
            msg = self.CreateLinearTwist(0.0)     # BLANK 2: forward speed for an IN-PLACE turn?
            msg.twist.angular.z = direction
            self.publisherfield_.publish(msg)
            time.sleep(self.dt)
        self.publisherfield_.publish(self.CreateLinearTwist(0.0))


def main(args=None):
    rclpy.init(args=args)
    node = Node1()
    #node.DriveStraight(1.0/0.90)   #1m
    #node.DriveStraight(0.67)     # 50cm
    #node.DriveCircle(0.5)        # BLANK 3: change this to test each Task 4 target
    #node.DriveArc(0.5, 90)

    
    #task6 below

    #node.DriveStraight(1.1)
    #node.TurnInPlace(-90)      # right = negative
    #node.DriveArc(0.5, 180)    # half circle
    #node.TurnInPlace(-90)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()