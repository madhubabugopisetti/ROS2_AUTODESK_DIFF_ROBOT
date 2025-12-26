import os
import xacro
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from os.path import join


def generate_launch_description():

    pkg_robot = get_package_share_directory('robot_description')
    pkg_gz = get_package_share_directory('ros_gz_sim')
    world_file = os.path.join(pkg_robot, 'worlds', 'world.sdf')

    # --- Robot description ---
    xacro_file = os.path.join(pkg_robot, 'urdf', 'robot_description.xacro')
    robot_description_xml = xacro.process_file(xacro_file).toxml()
    robot_description = {'robot_description': robot_description_xml}

    # --- Robot State Publisher (TF) ---
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': True}],
        output='screen'
    )

    # --- Gazebo ---
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(pkg_gz, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args": f"-r -v 4 {world_file}"
        }.items()
    )

    # --- Spawn robot ---
    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-topic', '/robot_description',
                    '-name', 'robot_description',
                    '-z', '0.32'
                ],
                output='screen'
            )
        ]
    )
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            os.path.join(pkg_robot, "config", "ros2_controllers.yaml"),
            {"use_sim_time": True}
        ],
        output="screen"
    )

    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller"],
    )

    # --- ROS ↔ Gazebo bridge ---
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[
            {'config_file': os.path.join(pkg_robot, 'config', 'ros_gz_bridge_gazebo.yaml')}
        ],
        output='screen'
    )

    # --- RViz ---
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d', os.path.join(pkg_robot, 'config', 'display.rviz')
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        ros_gz_bridge,
        rviz
    ])
