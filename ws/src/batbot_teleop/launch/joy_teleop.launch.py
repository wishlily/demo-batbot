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
    linear_accel_limit_arg = DeclareLaunchArgument(
        'linear_accel_limit',
        default_value='0.8',
        description='Maximum linear acceleration (m/s^2)'
    )
    angular_accel_limit_arg = DeclareLaunchArgument(
        'angular_accel_limit',
        default_value='2.0',
        description='Maximum angular acceleration (rad/s^2)'
    )
    axis_deadzone_arg = DeclareLaunchArgument(
        'axis_deadzone',
        default_value='0.2',
        description='Joystick axis deadzone'
    )
    cmd_publish_rate_arg = DeclareLaunchArgument(
        'cmd_publish_rate',
        default_value='30.0',
        description='Command publish rate (Hz)'
    )

    linear_speed_limit = LaunchConfiguration('linear_speed_limit')
    angular_speed_limit = LaunchConfiguration('angular_speed_limit')
    linear_accel_limit = LaunchConfiguration('linear_accel_limit')
    angular_accel_limit = LaunchConfiguration('angular_accel_limit')
    axis_deadzone = LaunchConfiguration('axis_deadzone')
    cmd_publish_rate = LaunchConfiguration('cmd_publish_rate')

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        namespace='batbot',
    )
    joy_teleop_node = Node(
        package='batbot_teleop',
        executable='joy_teleop',
        name='joy_teleop',
        namespace='batbot',
        parameters=[{
            'linear_speed_limit': linear_speed_limit,
            'angular_speed_limit': angular_speed_limit,
            'linear_accel_limit': linear_accel_limit,
            'angular_accel_limit': angular_accel_limit,
            'axis_deadzone': axis_deadzone,
            'cmd_publish_rate': cmd_publish_rate
        }]
    )

    return LaunchDescription([
        linear_speed_limit_arg,
        angular_speed_limit_arg,
        linear_accel_limit_arg,
        angular_accel_limit_arg,
        axis_deadzone_arg,
        cmd_publish_rate_arg,

        joy_node,
        joy_teleop_node
    ])
