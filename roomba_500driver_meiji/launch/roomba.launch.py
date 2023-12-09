from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'port',
                description='Roomba USB port',
                default_value=TextSubstitution(text='/dev/ttyUSB0'),
            ),
            Node(
                package='roomba_500driver_meiji',
                executable='main500',
                exec_name='roomba',
                parameters=[{'port': LaunchConfiguration('port')}],
            ),
            Node(
                package='roomba_500driver_meiji',
                executable='twist_to_roombactrl_converter',
                exec_name='twist_to_roombactrl_converter',
            ),
        ]
    )
