from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    linear_speed_limit_arg = DeclareLaunchArgument(
        'linear_speed_limit',
        default_value='1.0',
        description='Maximum linear speed'
    )
    angular_speed_limit_arg = DeclareLaunchArgument(
        'angular_speed_limit',
        default_value='5.0',
        description='Maximum angular speed'
    )

    linear_speed_limit = LaunchConfiguration('linear_speed_limit')
    angular_speed_limit = LaunchConfiguration('angular_speed_limit')

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node'
    )
    joy_teleop_node = Node(
        package='batbot_teleop',
        executable='joy_teleop',
        name='joy_teleop',
        parameters=[{
            'linear_speed_limit': linear_speed_limit,
            'angular_speed_limit': angular_speed_limit
        }]
    )

    return LaunchDescription([
        linear_speed_limit_arg,
        angular_speed_limit_arg,

        joy_node,
        joy_teleop_node
    ])