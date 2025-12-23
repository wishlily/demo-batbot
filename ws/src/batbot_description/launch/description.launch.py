from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("batbot_description")
    model_arg = DeclareLaunchArgument(
        "model",
        default_value=os.path.join(pkg_share, "urdf/batbot", "batbot.urdf.xacro"),
        description="Path to robot description file (URDF or XACRO)",
    )
    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]), value_type=str
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            {'use_sim_time': True},
            {"robot_description": robot_description}
        ],
    )

    use_jsp_arg = DeclareLaunchArgument(
        "use_jsp",
        default_value="false",
        description="Whether to start joint_state_publisher"
    )
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        condition=IfCondition(LaunchConfiguration("use_jsp"))
    )

    return LaunchDescription([
        model_arg,
        use_jsp_arg,
        robot_state_publisher,
        joint_state_publisher
    ])
