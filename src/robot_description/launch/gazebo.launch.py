import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from os.path import join

def generate_launch_description():

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_robot = get_package_share_directory('robot_description')

    robot_description_file = os.path.join(pkg_robot, 'urdf', 'robot_description.xacro')
    ros_gz_bridge_config = os.path.join(pkg_robot, 'config', 'ros_gz_bridge_gazebo.yaml')
    world_file = os.path.join(pkg_robot, 'worlds', 'world.sdf')

    robot_description_xml = xacro.process_file(robot_description_file).toxml()
    robot_description = {'robot_description': robot_description_xml}

    # -----------------------------
    # Robot State Publisher (TF)
    # -----------------------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': True}
        ],
    )

    # -----------------------------
    # Gazebo Harmonic
    # -----------------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r -v 4 {world_file}'
        }.items()
    )

    # -----------------------------
    # Spawn Robot
    # -----------------------------
    spawn_robot = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-topic', '/robot_description',
                    '-name', 'robot_description',
                    '-allow_renaming', 'false',
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.4',
                    '-Y', '0.0'
                ],
                output='screen'
            )
        ]
    )

    # -----------------------------
    # Gazebo ↔ ROS Bridge
    # -----------------------------
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[
            {'config_file': ros_gz_bridge_config},
            {'use_sim_time': True}
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        ros_gz_bridge,
    ])
