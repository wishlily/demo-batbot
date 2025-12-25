from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("batbot_description")
    model_arg = DeclareLaunchArgument(
        "model",
        default_value=os.path.join(pkg_share, "urdf/batbot", "batbot.urdf.xacro"),
        description="Path to robot description file (URDF or XACRO)",
    )

    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('batbot_description'),
                'launch',
                'description.launch.py'])
        ]),
        launch_arguments=[
            ("model", LaunchConfiguration("model")),
            ("use_jsp", "true")
        ]
    )

    rviz_config_file = os.path.join(pkg_share, "config/rviz", "urdf.rviz")
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )

    return LaunchDescription([
        model_arg,
        robot_description_launch,
        rviz2
    ])
