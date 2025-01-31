#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from cafe_robot_interfaces.srv import Order
import time

class RobotNavigator(Node):
    def __init__(self):
        super().__init__('robot_navigator')
        self.navigator = BasicNavigator()

        # Create a service client for the 'order_manager' service
        self.client = self.create_client(Order, '/order_manager')

        # Wait for the service to be available before making requests
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for order_manager service to be available...')

        self.get_logger().info("Robot Navigator is ready to receive orders.")

        # Define poses
        self.poses = {
            "home": self.set_pose(0.0, 0.0, 0.0),
            "kitchen": self.set_pose(4.0, 2.0, 0.0),
            "table1": self.set_pose(4.4, -4.0, 0.0),
            "table2": self.set_pose(4.8, -8.0, 0.0),
            "table3": self.set_pose(3.3, -8.0, 0.0)
        }
        self.navigator.setInitialPose(self.poses['home'])

        # Timer for checking new orders
        self.create_timer(0.2, self.check_for_orders)

    def set_pose(self, x, y, yaw):
        """Helper function to create PoseStamped from coordinates."""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = yaw
        return pose

    def check_for_orders(self):
        """Periodically checks for new orders."""
        self.get_logger().info("Checking for new orders...")

        # Request the status of all orders
        request = Order.Request()
        request.request_type = "status"

        future = self.client.call_async(request)
        future.add_done_callback(self.handle_order_status)

    def handle_order_status(self, future):
        """Handles the response from the order_manager service."""
        try:
            response = future.result()
            if response.accepted:
                orders = eval(response.response_message)  # Convert string to dictionary
                for table_number, status in orders.items():
                    if status == "pending":
                        self.process_order(int(table_number))
            else:
                self.get_logger().warn('Failed to retrieve order status.')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')


    # Order of Sequence 
        #when ordere received 
            # Home -> Kitchen -> Confirmation from kitchen -> Ordered Table number -> Return to home

    def process_order(self, table_number):
        """Handles the delivery process for a specific table."""
        table_key = f"table{table_number}"

        if table_key not in self.poses:
            self.get_logger().error(f"Invalid table number: {table_number}")
            return

        self.get_logger().info(f"Processing order for Table {table_number}...")

        # Update the order status to 'in_progress'
        request = Order.Request()
        request.table_number = table_number
        request.request_type = "update"
        request.status = "in_progress"

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None and future.result().accepted:
            self.get_logger().info(f"Order for Table {table_number} is now in progress.")
        else:
            self.get_logger().warn(f"Failed to update order status for Table {table_number}.")
            return

        self.navigator.waitUntilNav2Active()

        # Navigate to the kitchen
        if not self.navigate_to("kitchen"):
            return
        
        # Check if order is confirmed or rejected 
        self.get_logger().info("Arrived at the kitchen. Waiting for confirmation...")
        # Simulating the confirmation in the kitchen
        
        if not self.check_order_confirmation(table_number):
            self.get_logger().info(f"Order for Table {table_number} was rejected. Cancelling delivery.")
            self.navigate_to("home")
            return 

        # Simulate food pickup at the kitchen
        self.get_logger().info("Picking up food...")
        time.sleep(2)

        # Navigate to the assigned table
        if not self.navigate_to(table_key):
            return

        # Simulate food delivery
        self.get_logger().info("Delivering food...")
        time.sleep(2)

        # Update the order status to 'completed'
        request.status = "completed"
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None and future.result().accepted:
            self.get_logger().info(f"Order for Table {table_number} has been completed.")
        else:
            self.get_logger().warn(f"Failed to update order status for Table {table_number}.")

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
    

    def check_order_confirmation(self, table_number, timeout_sec=5):
        """Waits for confirmation of the order before proceeding."""
        request = Order.Request()
        request.table_number = table_number
        request.request_type = "status"  

        start_time = self.get_clock().now()

        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < timeout_sec:
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self, future)

            if future.result() is not None:
                orders = eval(future.result().response_message)  # Convert string to dictionary
                status = orders.get(table_number, "")

                if status == "confirmed":
                    self.get_logger().info(f"Order for Table {table_number} is confirmed. Proceeding with delivery.")
                    return True
                elif status == "rejected":
                    self.get_logger().warn(f"Order for Table {table_number} was rejected. Cancelling delivery.")
                    return False  # Exit process if order is rejected

            self.get_logger().info(f"Waiting for confirmation... (Current status: {status})")
            time.sleep(1)  # Check again after 1 second

        self.get_logger().warn(f"Timeout reached! No confirmation received for Table {table_number}. Cancelling order.")
        return False


def main(args=None):
    rclpy.init(args=args)
    robot_navigator = RobotNavigator()
    rclpy.spin(robot_navigator)
    robot_navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
