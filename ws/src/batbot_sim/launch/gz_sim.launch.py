from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
import os


def generate_launch_description():
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
    # robot description
    robot_description_arg = DeclareLaunchArgument(
        "robot",
        default_value="batbot_description",
        description="Robot description package name",
    )
    robot_description_launch_arg = DeclareLaunchArgument(
        'robot_description_launch',
        default_value='true',
        description='Launch robot description'
    )
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(LaunchConfiguration('robot')),
                'launch',
                'description.launch.py'])
        ]),
        condition=IfCondition(LaunchConfiguration('robot_description_launch'))
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
            "-name", LaunchConfiguration('robot'),
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.5"
        ],
        output="screen",
    )
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
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
        generate_world_cmd,

        robot_description_arg,
        robot_description_launch_arg,
        robot_description_launch,

        spawn_entity,
        bridge,

        wait_for_generation,
    ])
