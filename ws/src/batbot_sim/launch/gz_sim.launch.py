from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    world_name = "empty_room"
    # generate world sdf
    world_pkg_share = get_package_share_directory('batbot_sim')
    world_gen_dir = '/tmp/batbot_sim_generated'
    length_arg = DeclareLaunchArgument('length', default_value='10.0',
                                       description='AUTO gen World length')
    width_arg = DeclareLaunchArgument('width', default_value='5.0',
                                      description='AUTO gen World width')
    generate_world_cmd = ExecuteProcess(
        cmd=[
            os.path.join(world_pkg_share, 'scripts', 'gen_world.sh'),
            "-o", world_gen_dir,
            "-p", world_pkg_share,
            "-l", LaunchConfiguration('length'),
            "-w", LaunchConfiguration('width')
        ],
        output='screen'
    )
    world_arg = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(world_gen_dir, "", "world.sdf"),
        description="Path to world SDF file",
    )
    # gazebo resource path
    gz_resource_env = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[world_gen_dir, ':', os.path.join(world_pkg_share, 'models')]
    )
    # robot description
    model_arg = DeclareLaunchArgument(
        "model",
        default_value=os.path.join(get_package_share_directory("batbot_description"),
                                   "urdf/batbot", "batbot.urdf.xacro"),
        description="Path to robot description file (URDF or XACRO)",
    )
    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]), value_type=str
    )
    # gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": ["-r ", LaunchConfiguration("world")]}.items(),
    )
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "/robot_description",
            "-name", "batbot",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.01"
        ],
        output="screen",
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            f'/world/{world_name}/model/batbot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
        ],
        output='screen'
    )
    # gazebo
    wait_for_generation = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=generate_world_cmd,
            on_exit=[gazebo]
        )
    )

    return LaunchDescription([
        world_arg,
        length_arg,
        width_arg,
        gz_resource_env,
        generate_world_cmd,

        model_arg,
        robot_state_publisher,
        spawn_entity,
        bridge,

        wait_for_generation,
    ])
