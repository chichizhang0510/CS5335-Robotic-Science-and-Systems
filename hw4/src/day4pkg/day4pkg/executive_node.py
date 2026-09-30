"""Task 2: command-line Action client for the TurtleBot Driving Node."""

import rclpy
from rclpy.action import ActionClient  # let class be /move_turn's client
from rclpy.node import Node  # let the class be ROS node

from hw4_interfaces.action import MoveTurn


class ExecutiveNode(Node):
    """Ask for one movement command at a time and wait for its result."""

    def __init__(self):
        # build and name a ROS node, its name is executive_node
        super().__init__("executive_node")
        # an attribute of this class: the node is the client of a Action which is MoveTurn type and /move_turn name.
        self._client = ActionClient(self, MoveTurn, "move_turn")

    def send_command(
        self,
        distance: float,
        angle: float,
        radius: float = 0.0,
        arc_left: bool = True,
        arc_degrees: float = 0.0,
    ) -> bool:
        """
        send one task and wait for completion

        radius=0.0 means "straight line" (distance only) or "turn in
        place" (angle only), matching the original behaviour. A
        non-zero radius tells the Driving Node to drive a circular arc
        of arc_degrees to the left or right instead — see
        send_arc_command() below, which builds these arguments for you.
        """
        # build a goal
        goal = MoveTurn.Goal()
        goal.distance = distance
        goal.angle = angle
        goal.radius = radius
        goal.arc_left = arc_left
        goal.arc_degrees = arc_degrees

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

    def send_arc_command(self, direction: str, radius: float, degrees: float) -> bool:
        """
        Drive the TurtleBot along a circular arc.

        direction: 'left' (counter-clockwise) or 'right' (clockwise)
        radius:    arc radius in metres, must be positive
        degrees:   how much of the circle to travel, 0 to 360

        Internally this just calls send_command() with the arc fields
        filled in, so it reuses the exact same goal-sending and
        result-waiting logic.
        """
        direction = direction.strip().lower()
        if direction not in ("left", "right"):
            raise ValueError("direction must be 'left' or 'right'.")
        if radius <= 0.0:
            raise ValueError("radius must be positive.")
        if not (0.0 <= degrees <= 360.0):
            raise ValueError("degrees must be between 0 and 360.")

        self.get_logger().info(
            f"Arc command: {direction}, radius={radius:.2f} m, {degrees:.1f} deg."
        )
        return self.send_command(
            distance=0.0,
            angle=0.0,
            radius=radius,
            arc_left=(direction == "left"),
            arc_degrees=degrees,
        )

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
            (side_length, 0.0),  # move to middle right
            (0.0, 90.0),  # turn to up
            (side_length, 0.0),  # move to top right
            (0.0, 90.0),  # turn to left
            (side_length, 0.0),  # move to top left
            (0.0, 90.0),  # turn to down
            (side_length * 2.0, 0.0),  # move to bottom left
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


def _parse_arc_command(command: str):
    """
    Convert a user command to (direction, radius, degrees), or None
    if this isn't an arc command.

    Expected form: arc <left|right> <radius-metres> <degrees>
    """
    words = command.strip().lower().split()

    if not words or words[0] not in ("arc", "a"):
        return None

    if len(words) != 4:
        raise ValueError("Use: arc <left|right> <radius-metres> <degrees>.")

    direction = words[1]
    if direction not in ("left", "right"):
        raise ValueError("Direction must be 'left' or 'right'.")

    radius = float(words[2])
    if radius <= 0.0:
        raise ValueError("Radius must be positive.")

    degrees = float(words[3])
    if not (0.0 <= degrees <= 360.0):
        raise ValueError("Degrees must be between 0 and 360.")

    return direction, radius, degrees


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

    print("Project 4 Executive Node")
    print(
        "Commands: move <metres>, turn <degrees>, "
        "arc <left|right> <radius-metres> <degrees>, "
        "figure8 <side-metres>, p <side-metres>, quit"
    )

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

                # arc command
                arc_params = _parse_arc_command(command)
                if arc_params is not None:
                    node.send_arc_command(*arc_params)
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
