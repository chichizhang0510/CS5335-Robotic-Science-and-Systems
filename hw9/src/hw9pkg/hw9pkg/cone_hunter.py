"""
HW9 Tasks 1 and 2 on the real TurtleBot 4.

  ros2 run hw9pkg cone_hunter --ros-args --params-file config/hw9_params.yaml

Task 1: turns 360 degrees and reports the colour, direction and distance of
each cone. Task 2: asks for a colour, drives to that cone and stops about an
inch short. It repeats until you type q. Ctrl-C stops the robot at any time.
"""

import threading

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions

from hw9pkg.mission import ConeHunter, MissionError
from hw9pkg.ros_robot import RosRobot


def main(args=None):
    # Ctrl-C must raise KeyboardInterrupt in the main thread (which then stops
    # the robot) instead of rclpy shutting the context down underneath us.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    robot = executor = None
    try:
        robot = RosRobot('cone_hunter')
        executor = SingleThreadedExecutor()
        executor.add_node(robot)
        threading.Thread(target=executor.spin, daemon=True).start()

        robot.wait_ready()
        hunter = ConeHunter(robot, robot.cfg)
        print(f'Cone colours: {", ".join(robot.cfg.colors)}; stopping at lidar range '
              f'{robot.cfg.contact_range + robot.cfg.target_gap:.3f} m '
              f'(contact_range {robot.cfg.contact_range:.3f} + gap {robot.cfg.target_gap:.3f}). '
              f'Images go to {robot.cfg.output_dir}/', flush=True)
        hunter.task1()
        hunter.task2()
    except KeyboardInterrupt:
        print('\nCtrl-C: stopping the robot.', flush=True)
    except MissionError as e:
        print(f'\nERROR: {e}', flush=True)
    finally:
        if robot is not None:
            robot.stop_hard()
        if executor is not None:
            executor.shutdown()
        if robot is not None:
            robot.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
