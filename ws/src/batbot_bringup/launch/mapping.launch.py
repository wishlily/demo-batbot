from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from lifecycle_msgs.msg import Transition
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
        description="Robot namespace used by sensor topics (e.g. /<robot_ns>/scan)",
    )
    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="true",
        description="Start slam_toolbox for 2D mapping",
    )
    bringup_pkg_share = get_package_share_directory("batbot_bringup")
    mapping_rviz_config = os.path.join(bringup_pkg_share, "config/rviz", "mapping.rviz")

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
            ("rviz_config", mapping_rviz_config),
        ],
    )

    slam_config_file = os.path.join(bringup_pkg_share, "config", "slam.yaml")
    slam_toolbox = LifecycleNode(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox",
        namespace="",
        output="screen",
        parameters=[slam_config_file],
        remappings=[
            ("/scan", ["/", LaunchConfiguration("robot_ns"), "/scan"]),
        ],
        condition=IfCondition(LaunchConfiguration("use_slam")),
    )
    slam_configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_toolbox),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(LaunchConfiguration("use_slam")),
    )
    slam_activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam_toolbox),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        ),
        condition=IfCondition(LaunchConfiguration("use_slam")),
    )

    return LaunchDescription([
        model_arg,
        robot_ns_arg,
        use_slam_arg,
        display_launch,
        slam_toolbox,
        slam_configure,
        slam_activate,
    ])
