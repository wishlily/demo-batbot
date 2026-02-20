from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_share = get_package_share_directory("batbot_navigation")
    default_map = os.path.join(package_share, "maps", "batbot_map.yaml")
    default_params = os.path.join(package_share, "config", "nav2.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    use_composition = LaunchConfiguration("use_composition")
    robot_ns = LaunchConfiguration("robot_ns")
    use_namespace = LaunchConfiguration("use_namespace")
    nav2_namespace = PythonExpression(
        ["'", robot_ns, "' if '", use_namespace, "' == 'True' else ''"]
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation (Gazebo) clock if true",
    )
    declare_map_yaml = DeclareLaunchArgument(
        "map",
        default_value=default_map,
        description="Full path to map yaml file",
    )
    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Full path to Nav2 parameter file",
    )
    declare_autostart = DeclareLaunchArgument(
        "autostart",
        default_value="True",
        description="Automatically startup the Nav2 stack",
    )
    declare_use_composition = DeclareLaunchArgument(
        "use_composition",
        default_value="False",
        description="Use composed bringup (single container) when true",
    )
    declare_robot_ns = DeclareLaunchArgument(
        "robot_ns",
        default_value="batbot",
        description="Robot namespace used by odometry and sensor topics",
    )
    declare_use_namespace = DeclareLaunchArgument(
        "use_namespace",
        default_value="False",
        description="Apply a ROS namespace to the navigation stack",
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("nav2_bringup"),
                "launch",
                "bringup_launch.py",
            ])
        ]),
        launch_arguments=[
            ("slam", "False"),
            ("map", map_yaml),
            ("use_sim_time", use_sim_time),
            ("params_file", params_file),
            ("autostart", autostart),
            ("use_composition", use_composition),
            ("namespace", nav2_namespace),
            ("use_namespace", use_namespace),
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_map_yaml,
        declare_params_file,
        declare_autostart,
        declare_use_composition,
        declare_robot_ns,
        declare_use_namespace,
        nav2_bringup,
    ])
