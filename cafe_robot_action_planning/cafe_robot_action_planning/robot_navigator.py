#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from cafe_robot_interfaces.srv import Order  # Importing the Custom Order service

import time

class RobotNavigator(Node):
    def __init__(self):
        super().__init__('robot_navigator')
        self.navigator = BasicNavigator()

        # Service Server: Listens for order requests
        self.srv = self.create_service(Order, 'order_manager', self.order_callback)

        self.get_logger().info("Robot Navigator is ready to receive orders.")

        # Define poses 
        self.poses = {
            "home": self.set_pose(0.0, 0.0, 0.0),
            "kitchen": self.set_pose(4.0, 2.0, 0.0),
            "table1": self.set_pose(4.4, -4.0, 0.0),
            "table2": self.set_pose(4.8, -8.0, 0.0),
            "table3": self.set_pose(3.3, -8.0, 0.0)
        }

    def set_pose(self, x, y, yaw):
        """Helper function to create PoseStamped from Dict"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = yaw
        return pose

    def order_callback(self, request, response):
        """Handles incoming service requests from the Order Manager"""
        table_number = request.table_number
        request_type = request.request_type

        if request_type == "new":
            self.get_logger().info(f"Received new order for Table {table_number}")
            response.accepted = True
            response.response_message = f"Order for Table {table_number} is accepted."
            
            self.process_order(table_number)
        
        elif request_type == "cancel":
            self.get_logger().warn(f"Cancel request received for Table {table_number}")
            response.accepted = True
            response.response_message = f"Order for Table {table_number} is cancelled."

        elif request_type == "status":
            response.accepted = True
            response.response_message = "Robot status: Ready to take new orders."

        else:
            response.accepted = False
            response.response_message = "Invalid request type."

        return response

    def process_order(self, table_number):
        """Handles the actual delivery process"""
        table_key = f"table{table_number}"
        
        if table_key not in self.poses:
            self.get_logger().error(f"Invalid table number: {table_number}")
            return
        
        self.get_logger().info(f"Processing order for Table {table_number}...")

        self.navigator.waitUntilNav2Active()

        # Navigate to the kitchen
        if not self.navigate_to("kitchen"):
            return

        # Simulate food pickup
        self.get_logger().info("Picking up food...")
        time.sleep(2)

        # Navigate to the assigned table
        if not self.navigate_to(table_key):
            return

        # Simulate food delivery
        self.get_logger().info("Delivering food...")
        time.sleep(2)

        # Return to home
        self.navigate_to("home")

    def navigate_to(self, location):
        """Handles navigation with feedback and logging."""
        self.navigator.goToPose(self.poses[location])

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                self.get_logger().info(f'Navigating to {location}: {feedback.distance_remaining:.2f} meters remaining.')

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f"Arrived at {location}.")
            return True
        else:
            self.get_logger().error(f"Failed to reach {location}.")
            return False


def main(args=None):
    rclpy.init(args=args)
    robot_navigator = RobotNavigator()

    rclpy.spin(robot_navigator)  # Keep running to listen for service calls

    robot_navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
