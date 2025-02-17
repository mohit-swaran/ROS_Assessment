import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():
  pkg_tb3_sim = get_package_share_directory('cafe_robot_action_planning')
  pkg_tb3_autonomy = get_package_share_directory('cafe_robot_bt')

  autonomy_node_cmd = Node(
      package="cafe_robot_bt",
      executable="bt_node",
      name="bt_node",
      parameters=[{
          "location_file": os.path.join(pkg_tb3_sim, "config", "nav2","locations.yaml")
      }]
  )

  ld = LaunchDescription()

  # Add the commands to the launch description
  ld.add_action(autonomy_node_cmd)

  return ld