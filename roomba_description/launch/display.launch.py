import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node


def generate_launch_description():
    roomba_description_share_path = get_package_share_directory("roomba_description")
    robot_description = {"robot_description": LaunchConfiguration("urdf_path")}
    rviz_config_path = os.path.join(
        roomba_description_share_path, "config", "roomba.rviz"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "urdf_path",
                default_value=Command(
                    [
                        PathJoinSubstitution([FindExecutable(name="xacro")]),
                        " ",
                        PathJoinSubstitution(
                            [
                                roomba_description_share_path,
                                "urdf",
                                "roomba.urdf.xacro",
                            ],
                        ),
                    ],
                ),
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                exec_name="joint_state_publisher_gui",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                exec_name="robot_state_publisher",
                parameters=[robot_description],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                exec_name="rviz2",
                arguments=["-d", rviz_config_path],
            ),
        ]
    )
