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
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file using xacro
    pkg_path = os.path.join(get_package_share_directory(pkg_name))
    xacro_file = os.path.join(pkg_path, 'urdf', 'cafe_robot.urdf.xacro')
    
    robot_description = xacro.process_file(xacro_file).toxml()


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
        launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    # Spawn robot entity in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'cafe_robot',
            '-z', '0.2'
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

    params = {'robot_description': robot_description, 'use_sim_time': use_sim_time}
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )


    # Return the LaunchDescription
    return LaunchDescription([
        rsp,
        gazebo,
        world_arg,
        spawn_entity,
        ros_gz_bridge,
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'
        ),
        
    ])
