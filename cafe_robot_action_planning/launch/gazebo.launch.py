import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    pkg_name = 'cafe_robot_action_planning'

    # Include rviz launch file
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(pkg_name), 'launch', 'rviz.launch.py'
        )])
    )

    # Default world file
    default_world = os.path.join(
        get_package_share_directory(pkg_name),
        'worlds',
        'empty.world'
    )

    # Declare world argument
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Specify the world in CLI or it opens empty_world'
    )

    
    world = LaunchConfiguration('world')

    # Include Gazebo launch file with proper argument substitution
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
        )]),
        launch_arguments={'gz_args': ['-r -v4', world], 'on_exit_shutdown': 'true'}.items()
    )

    # Spawn robot entity in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'cafe_robot',
            '-z', '0.5'
        ],
        output='screen',
        parameters=[
            {'use_sim_time': True},
        ]
    )

    bridge_params = os.path.join(get_package_share_directory(pkg_name), 'config', 'bridge_params.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        parameters=[
            {'use_sim_time': True},
        ]
    )

    # Return the LaunchDescription
    return LaunchDescription([
        rsp,
        gazebo,
        world_arg,
        spawn_entity,
        ros_gz_bridge,
    ])
