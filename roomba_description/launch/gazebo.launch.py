from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    roomba_description_share_path = get_package_share_directory("roomba_description")
    robot_description = {
        "robot_description": Command(
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
        )
    }
    controller_yaml_path = PathJoinSubstitution(
        [
            get_package_share_directory("roomba_description"),
            "config",
            "controllers.yaml",
        ]
    )

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                exec_name="robot_state_publisher",
                parameters=[robot_description],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            get_package_share_directory("gazebo_ros"),
                            "launch",
                            "gazebo.launch.py",
                        ]
                    )
                )
            ),
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                exec_name="urdf_spawner",
                arguments=[
                    "-entity",
                    "roomba",
                    "-topic",
                    "/robot_description",
                    "-x",
                    "0",
                    "-y",
                    "0",
                    "-z",
                    "0",
                ],
                parameters=[{"use_sim_time": True}],
            ),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                exec_name="controller_manager",
                parameters=[robot_description, controller_yaml_path],
            ),
        ]
    )
