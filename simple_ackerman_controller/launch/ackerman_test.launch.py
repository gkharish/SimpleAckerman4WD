from multiprocessing import context
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, PythonExpression, TextSubstitution
from launch.conditions import LaunchConfigurationEquals
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory

import xacro
from launch.actions import ExecuteProcess,  RegisterEventHandler
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    rover_model_sim = 'mybot'

    world_file_name = 'wall_world.sdf' 

    use_sim_time = True

    bot_sim_pkg = get_package_share_directory('osiris_bot')
    world_path = os.path.join(bot_sim_pkg, 'worlds', world_file_name)

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gazebo_default = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ' -r {}'.format(world_path)}.items(),
    )
  
    robot_description_path = os.path.join(bot_sim_pkg, "urdf", "{}.gazebo".format(rover_model_sim))

    doc = xacro.parse(open(robot_description_path))
    xacro.process_doc(doc)
    robot_description_content = doc.toxml()
    robot_description = {"robot_description": robot_description_content}

    spawn_bot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', doc.toxml(),
                   '-name', 'rover',
                   '-x', '0.0',
                   '-z', '4.5',
                   '-Y', str(0 * (3.14/180)),  # radians
                   '-allow_renaming', 'true'],
    )

    gazebo_bridge_param_file = os.path.join(bot_sim_pkg, 'config', 'ign_ros_bridge.yaml')
    gazebo_bridge = Node(
        package='ros_gz_bridge',
        executable='bridge_node',
        parameters=[{
            "config_file": gazebo_bridge_param_file,
            "qos_overrides./camera/aligned_depth_to_color/image_raw.publisher.reliability": "best_effort",
            "qos_overrides./camera/color/image_raw.publisher.reliability": "best_effort",
            "qos_overrides./camera/color/camera_info.publisher.reliability": "best_effort",
        }],
        output='screen'
    )


    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    # Replace this wtih your own controller (abstract ackerman)
    # load_swerve_controller_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
    #          'autonomous_controller'],
    #     output='screen'
    # )

    teleop_prefix = get_package_share_directory('rovers_bringup')
    inc_teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([teleop_prefix, '/launch/teleop-launch.py']),
    )

    nodes_list = [
        SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1'),
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        # ============ Rovers description ==============
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description]),

        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=spawn_bot,
        #         on_exit=[load_joint_state_controller],
        #     )
        # ),
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=load_joint_state_controller,
        #         on_exit=[load_swerve_controller_controller],
        #     )
        # ),
        gazebo_default,
        spawn_bot,
        gazebo_bridge,
        #inc_teleop_launch,

    ]

    ld = LaunchDescription(nodes_list)

    return ld
