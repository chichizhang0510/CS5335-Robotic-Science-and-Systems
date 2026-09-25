"""Task 2: command-line Action client for the TurtleBot Driving Node."""

import rclpy
from rclpy.action import ActionClient  # let class be /move_turn's client
from rclpy.node import Node  # let the class be ROS node

from action_interfaces.action import MoveTurn


class ExecutiveNode(Node):
    """Ask for one movement command at a time and wait for its result."""

    def __init__(self):
        # build and name a ROS node, its name is executive_node
        super().__init__("executive_node")
        # an attribute of this class: the node is the client of a Action which is MoveTurn type and /move_turn name.
        self._client = ActionClient(self, MoveTurn, "move_turn")

    def send_command(self, distance: float, angle: float) -> bool:
        """
        send one task and wait for completion
        """
        # build a goal
        goal = MoveTurn.Goal()
        goal.distance = distance
        goal.angle = angle

        # make sure the driving node has been started
        # launch file is resonsible for starting 2 nodes
        self.get_logger().info("Waiting for the Driving Node Action server...")
        self._client.wait_for_server()

        # send goal
        send_future = self._client.send_goal_async(goal)
        # ROS processing, until receive feedback
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        # check if driving node has accepted
        if not goal_handle.accepted:
            self.get_logger().error("Driving Node rejected.")
            return False

        # waiting for completion
        self.get_logger().info("Command accepted; waiting for completion.")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        status = result_future.result().status

        if status == 4:  # action_msgs/msg/GoalStatus.STATUS_SUCCEEDED
            self.get_logger().info("Command completed.")
            return True
        elif status == 5:  # action_msgs/msg/GoalStatus.STATUS_CANCELED
            self.get_logger().warn("Command was cancelled.")
        else:
            self.get_logger().error(f"Command ended with Action status {status}.")
        return False

    def run_figure_eight(self, side_length: float) -> None:
        # split into 15 action goal
        # move to lower right -> right diamond circle -> left diamond circle
        commands = [
            (0.0, 315.0),  # counter-clockwise = clockwise 45
            (side_length, 0.0),  # move to lower right
            (0.0, 90.0),  # turn to upper right
            (side_length, 0.0),  # move to right
            (0.0, 90.0),  # turn to upper left
            (side_length, 0.0),  # move to upper right
            (0.0, 90.0),  # turn to lower left
            (side_length, 0.0),  # move back to center
            (side_length, 0.0),  # move to lower left
            (0.0, 270.0),  # counter-clockwise 270 = clockwise 90, turn to upper left
            (side_length, 0.0),  # move to left
            (0.0, 270.0),  # counter-clockwise 270 = clockwise 90, turn to upper right
            (side_length, 0.0),  # move to upper left
            (0.0, 270.0),  # counter-clockwise 270 = clockwise 90, turn to lower right
            (side_length, 0.0),  # move back to center
        ]

        self.get_logger().info(f"Starting figure-eight with {side_length:.2f} m sides.")

        for distance, angle in commands:
            if not self.send_command(distance, angle):
                self.get_logger().warn("Figure-eight stopped before completion.")
                return

        self.get_logger().info("Figure-eight completed.")

    def run_letter_p(self, side_length: float) -> None:
        # height is 2 times side length
        # move up -> draw the top rectangle -> end at middle left
        commands = [
            (0.0, 90.0),  # turn to up
            (side_length * 2.0, 0.0),  # move to top left
            (0.0, 270.0),  # counter-clockwise 270 = clockwise 90, turn to right
            (side_length, 0.0),  # move to top right
            (0.0, 270.0),  # counter-clockwise 270 = clockwise 90, turn to down
            (side_length, 0.0),  # move to middle right
            (0.0, 270.0),  # counter-clockwise 270 = clockwise 90, turn to left
            (side_length, 0.0),  # move to middle left
        ]
        self.get_logger().info(f"Starting letter P with {side_length:.2f} m sides.")
        for distance, angle in commands:
            if not self.send_command(distance, angle):
                self.get_logger().warn("Letter P stopped before completion.")
                return
        self.get_logger().info("Letter P completed.")


def _parse_command(command: str):
    """Convert a user command to (distance, angle), or return None for quit."""
    words = command.strip().lower().split()
    if not words:
        raise ValueError("Enter a command.")
    if words[0] in ("quit", "q", "exit") and len(words) == 1:
        return None
    if len(words) != 2:
        raise ValueError("Use: move <metres>, turn <degrees>, or quit.")

    value = float(words[1])
    if words[0] in ("move", "m"):
        return value, 0.0
    if words[0] in ("turn", "t"):
        return 0.0, value
    raise ValueError("Use: move <metres>, turn <degrees>, or quit.")


def _parse_figure_eight(command: str):
    # let inputs be a word list
    words = command.strip().lower().split()

    # skip it
    if not words or words[0] not in ("figure8", "figure-8", "f8"):
        return None

    # default length
    if len(words) == 1:
        return 0.20

    # invalid paramemters
    if len(words) != 2:
        raise ValueError("Use: figure8 <side-metres>.")
    side_length = float(words[1])

    # invalid input
    if side_length <= 0.0:
        raise ValueError("Figure-eight side length must be positive.")
    return side_length


def _parse_letter_p(command: str):
    # let inputs be a word list
    words = command.strip().lower().split()

    # skip it
    if not words or words[0] not in ("p", "letterp", "letter-p"):
        return None

    # default length
    if len(words) == 1:
        return 0.20

    # invalid parameters
    if len(words) != 2:
        raise ValueError("Use: p <side-metres>.")
    side_length = float(words[1])

    # invalid input
    if side_length <= 0.0:
        raise ValueError("Letter P side length must be positive.")
    return side_length


def main(args=None):
    rclpy.init(args=args)
    node = ExecutiveNode()

    print("Project 3 Executive Node")
    print("Commands: move <metres>, turn <degrees>, figure8 <side-metres>, p <side-metres>, quit")

    # the loop
    try:
        while rclpy.ok():
            try:
                # read command
                command = input("> ")

                # Extra Credit A
                figure_eight_side = _parse_figure_eight(command)
                if figure_eight_side is not None:
                    node.run_figure_eight(figure_eight_side)
                    continue

                # Extra Credit A
                letter_p_side = _parse_letter_p(command)
                if letter_p_side is not None:
                    node.run_letter_p(letter_p_side)
                    continue

                # parse, get move/turn/quit
                parsed = _parse_command(command)
                if parsed is None:
                    break

                # send command
                node.send_command(*parsed)
            except ValueError as error:
                node.get_logger().error(str(error))
            except (EOFError, KeyboardInterrupt):
                break
    finally:
        # when receive quit, do this
        node.destroy_node()
        rclpy.shutdown()
