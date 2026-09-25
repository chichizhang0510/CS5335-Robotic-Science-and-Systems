"""Task 4: start the Executive Node and Driving Node together."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="day3pkg",
                executable="driving_node",
            ),
            Node(
                package="day3pkg",
                executable="executive_node",
            ),
        ]
    )
