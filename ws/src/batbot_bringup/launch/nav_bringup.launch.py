from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    description_pkg_share = get_package_share_directory("batbot_description")
    navigation_pkg_share = get_package_share_directory("batbot_navigation")

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
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation (Gazebo) clock if true",
    )
    use_namespace_arg = DeclareLaunchArgument(
        "use_namespace",
        default_value="False",
        description="Apply namespace to Nav2 nodes",
    )
    map_arg = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(navigation_pkg_share, "maps", "batbot_map.yaml"),
        description="Full path to map yaml file",
    )
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(navigation_pkg_share, "config", "nav2.yaml"),
        description="Full path to Nav2 parameter file",
    )
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=os.path.join(get_package_share_directory("batbot_bringup"), "config/rviz", "mapping.rviz"),
        description="Path to RViz config file",
    )

    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("batbot_bringup"),
                "launch",
                "display.launch.py",
            ])
        ]),
        launch_arguments=[
            ("model", LaunchConfiguration("model")),
            ("robot_ns", LaunchConfiguration("robot_ns")),
            ("rviz_config", LaunchConfiguration("rviz_config")),
        ],
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("batbot_navigation"),
                "launch",
                "navigation.launch.py",
            ])
        ]),
        launch_arguments=[
            ("use_sim_time", LaunchConfiguration("use_sim_time")),
            ("map", LaunchConfiguration("map")),
            ("params_file", LaunchConfiguration("params_file")),
            ("use_composition", "False"),
            ("robot_ns", LaunchConfiguration("robot_ns")),
            ("use_namespace", LaunchConfiguration("use_namespace")),
        ],
    )

    return LaunchDescription([
        model_arg,
        robot_ns_arg,
        use_sim_time_arg,
        use_namespace_arg,
        map_arg,
        params_file_arg,
        rviz_config_arg,
        display_launch,
        navigation_launch,
    ])
