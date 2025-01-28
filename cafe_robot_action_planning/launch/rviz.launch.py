import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # Package and file paths
    pkg_name = 'cafe_robot_action_planning'
    xacro_file = os.path.join(get_package_share_directory(pkg_name), 'urdf', 'cafe_robot.urdf.xacro')
    rviz_config_file = os.path.join(get_package_share_directory(pkg_name), 'config', 'cafe_robot_config.rviz')

    # Xacro file to generate URDF
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = robot_description_config.toxml()

    return LaunchDescription([
        # Robot State Publisher node
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        # RViz2 node 
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file], 
            output='screen'
        ),

        # Joint State Publisher GUI node
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),
    ])
