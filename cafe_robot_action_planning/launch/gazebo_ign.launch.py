import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    pkg_name = 'cafe_robot_action_planning'

    # Process the URDF file using xacro
    pkg_path = os.path.join(get_package_share_directory(pkg_name))
    xacro_file = os.path.join(pkg_path, 'urdf', 'ign_robot.urdf.xacro')
    
    robot_description = xacro.process_file(xacro_file).toxml()

    # Default world file
    default_world = os.path.join(
        get_package_share_directory(pkg_name),
        'worlds',
        'ign_gz.world'
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
        launch_arguments={'gz_args': ['-r -v4 ', default_world], 'on_exit_shutdown': 'true'}.items()
    )

    # Spawn robot entity in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'diff_drive',
            '-x', '-2.15',
            '-y', '-4.0',
            '-z', '0.5',
            '-Y', '1.57',
        ],
        output='screen',
        parameters=[
            {'use_sim_time': True},
        ]
    )

    bridge_params = os.path.join(get_package_share_directory(pkg_name), 'config', 'gazebo','bridge_params_ign.yaml')
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

    params = {'robot_description': robot_description, 'use_sim_time': True}
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    order_manager = Node(
        package=pkg_name,
        executable='order_manager',
        output='screen',
    )

    # Return the LaunchDescription
    return LaunchDescription([
        rsp,
        gazebo,
        world_arg,
        spawn_entity,
        ros_gz_bridge,
        order_manager,
    ])
