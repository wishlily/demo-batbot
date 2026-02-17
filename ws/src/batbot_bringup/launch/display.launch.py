from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    description_pkg_share = get_package_share_directory("batbot_description")
    model_arg = DeclareLaunchArgument(
        "model",
        default_value=os.path.join(description_pkg_share, "urdf/batbot", "batbot.urdf.xacro"),
        description="Path to robot description file (URDF or XACRO)",
    )
    robot_ns_arg = DeclareLaunchArgument(
        "robot_ns",
        default_value="batbot",
        description="Robot namespace used by odometry and sensor topics",
    )
    bringup_pkg_share = get_package_share_directory("batbot_bringup")
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=os.path.join(bringup_pkg_share, "config/rviz", "urdf.rviz"),
        description="Path to RViz config file",
    )

    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("batbot_description"),
                "launch",
                "description.launch.py",
            ])
        ]),
        launch_arguments=[
            ("model", LaunchConfiguration("model")),
            ("use_jsp", "true"),
            ("use_sim_time", "false"),
        ],
    )

    localization_config_file = os.path.join(bringup_pkg_share, "config", "ekf.yaml")
    localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        namespace=LaunchConfiguration("robot_ns"),
        output="screen",
        parameters=[localization_config_file],
        remappings=[
            ("odometry/filtered", "odom"),
        ],
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
    )

    return LaunchDescription([
        model_arg,
        robot_ns_arg,
        rviz_config_arg,
        robot_description_launch,
        localization,
        rviz2,
    ])
