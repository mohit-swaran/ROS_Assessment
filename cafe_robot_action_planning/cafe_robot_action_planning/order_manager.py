#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from cafe_robot_interfaces.srv import Order

class OrderManager(Node):
    def __init__(self):
        super().__init__('order_manager')
        self.orders = {}  # Dictionary to store table orders (e.g., {2: 'pending'})

        # Create a service to manage orders
        self.srv = self.create_service(Order, 'order_manager', self.handle_order_request)
        self.get_logger().info("Order Manager is ready to receive orders.")

    def handle_order_request(self, request, response):
        """
        Handles incoming service requests for managing orders.
        """
        if request.request_type == "place":
            return self.place_order(request, response)
        elif request.request_type == "status":
            return self.get_order_status(response)
        elif request.request_type == "update":
            return self.update_order_status(request, response)
        else:
            response.accepted = False
            response.response_message = "Invalid request type."
            self.get_logger().warn("Received invalid request type.")
            return response

    def place_order(self, request, response):
        """
        Places a new order if the table does not already have an active order.
        """
        table_number = request.table_number

        if table_number in self.orders and self.orders[table_number] != "completed":
            response.accepted = False
            response.response_message = f"Order for Table {table_number} already exists."
            self.get_logger().warn(response.response_message)
        else:
            self.orders[table_number] = "pending"
            response.accepted = True
            response.response_message = f"Order for Table {table_number} has been placed."
            self.get_logger().info(response.response_message)

        return response

    def get_order_status(self, response):
        """
        Returns the current order status as a dictionary string.
        """
        response.accepted = True
        response.response_message = str(self.orders)  # Convert dictionary to string
        return response

    def update_order_status(self, request, response):
        """
        Updates the status of an order if the table number exists.
        """
        table_number = request.table_number
        new_status = request.status

        if table_number in self.orders:
            self.orders[table_number] = new_status
            response.accepted = True
            response.response_message = f"Order for Table {table_number} updated to {new_status}."
            self.get_logger().info(response.response_message)
        else:
            response.accepted = False
            response.response_message = f"No existing order for Table {table_number}."
            self.get_logger().warn(response.response_message)

        return response
    
    def confirm_order(self, request, response):
        """
        Confirms or rejects the order for a given table.
        """
        table_number = request.table_number
        accepted = request.accepted

        if table_number in self.orders:
            if accepted:
                self.orders[table_number] = "confirmed"
                response.accepted = True
                response.response_message = f"Order for Table {table_number} confirmed."
                self.get_logger().info(f"Order for Table {table_number} confirmed.")
            else:
                self.orders[table_number] = "rejected"
                response.accepted = False
                response.response_message = f"Order for Table {table_number} rejected."
                self.get_logger().info(f"Order for Table {table_number} rejected.")
        else:
            response.accepted = False
            response.response_message = f"No order found for Table {table_number}."
            self.get_logger().warn(f"No order found for Table {table_number}.")

        return response

def main(args=None):
    rclpy.init(args=args)
    order_manager = OrderManager()
    rclpy.spin(order_manager)
    order_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
