"""Task 3: Action server that turns MoveTurn goals into /cmd_vel commands."""

import math
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from action_interfaces.action import MoveTurn


class DrivingNode(Node):
    def __init__(self):
        super().__init__("driving_node")

        self.declare_parameter("linear_speed", 0.10)  # metres / second
        self.declare_parameter("angular_speed", 0.30)  # radians / second
        self._linear_speed = self.get_parameter("linear_speed").value
        self._angular_speed = self.get_parameter("angular_speed").value

        if self._linear_speed <= 0.0 or self._angular_speed <= 0.0:
            raise ValueError("linear_speed and angular_speed must both be positive.")

        # send TwistStamped message to /cmd_vel topic
        self._publisher = self.create_publisher(TwistStamped, "/cmd_vel", 10)

        self._callbacks = ReentrantCallbackGroup()

        # start listener
        # this is a /move_turn Action server, listening to MoveTurn goal
        self._server = ActionServer(
            self,
            MoveTurn,
            "move_turn",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._callbacks,
        )

    def goal_callback(self, goal_request: MoveTurn.Goal) -> GoalResponse:
        """
        Do it when receiving every goal
        """
        # invalid situations
        if goal_request.distance < 0.0 or goal_request.angle < 0.0:
            self.get_logger().warn("Reject: distance and angle must be non-negative.")
            return GoalResponse.REJECT
        if goal_request.distance != 0.0 and goal_request.angle != 0.0:
            self.get_logger().warn(
                "Reject: request either movement or a turn, not both."
            )
            return GoalResponse.REJECT

        # accept
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle) -> CancelResponse:
        self.get_logger().info("Cancellation request accepted.")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        distance = goal_handle.request.distance
        angle_degrees = goal_handle.request.angle

        if distance != 0.0:
            duration = distance / self._linear_speed
            linear_x, angular_z = self._linear_speed, 0.0
            description = f"moving {distance:.2f} m"
        else:
            angle_radians = math.radians(angle_degrees)
            duration = angle_radians / self._angular_speed
            linear_x, angular_z = 0.0, self._angular_speed
            description = f"turning {angle_degrees:.1f} degrees"

        self.get_logger().info(
            f"Started {description}; target duration {duration:.2f} s."
        )
        start_time = time.monotonic()
        try:
            while time.monotonic() - start_time < duration:
                if goal_handle.is_cancel_requested:
                    self._publish_twist(0.0, 0.0)
                    goal_handle.canceled()
                    self.get_logger().warn("Goal cancelled; published a stop command.")
                    return MoveTurn.Result()

                # Only linear.x is non-zero during a move; only angular.z is
                # non-zero during a turn.  All other Twist fields stay at zero.
                self._publish_twist(linear_x, angular_z)
                time.sleep(0.05)
        finally:
            # Safe for normal completion, cancellation, Ctrl-C, or an exception.
            self._publish_twist(0.0, 0.0)

        goal_handle.succeed()
        self.get_logger().info("Goal completed; published a stop command.")
        return MoveTurn.Result()

    def _publish_twist(self, linear_x: float, angular_z: float) -> None:
        """Publish a timestamped command, leaving every unspecified field at zero."""
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.twist.linear.x = linear_x
        message.twist.angular.z = angular_z
        self._publisher.publish(message)

    def destroy_node(self):
        """Stop the robot before the node disappears."""
        self._publish_twist(0.0, 0.0)
        self._server.destroy()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DrivingNode()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
