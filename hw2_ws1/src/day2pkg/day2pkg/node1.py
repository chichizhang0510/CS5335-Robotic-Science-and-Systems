import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

class Node1(Node):
    def __init__(self):
        super().__init__('node1')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('z', 0.0)

        self.publisher_ = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def publish_message(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = 0.2
        msg.twist.angular.z = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info('published message : {}'.format(msg))

    def timer_callback(self):
        x = self.get_parameter('x').get_parameter_value().double_value
        z = self.get_parameter('z').get_parameter_value().double_value
        linear_message = self.CreateLinearTwist(x)
        angular_message = self.CreateAngularTwist(z) 
        msg = linear_message
        msg.twist.angular.z = angular_message.twist.angular.z
        self.publisher_.publish(msg)
        self.get_logger().info('published message : {}'.format(msg))

    def CreateLinearTwist(self, x = 0.0):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = x
        return msg
    
    def CreateAngularTwist(self, z=0.0):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.angular.z = z
        return msg

# Modify it to accept x and z as arguments
def main(args=None):
    rclpy.init(args=args)
    node = Node1()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_message = TwistStamped()
        stop_message.header.stamp = node.get_clock().now().to_msg()
        stop_message.header.frame_id = 'base_link'
        node.publisher_.publish(stop_message)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()