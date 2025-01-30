#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from cafe_robot_interfaces.srv import Order

class OrderManager(Node):
    def __init__(self):
        super().__init__('order_manager_service')
        self.orders = {}  # Store active orders in dict
        self.srv = self.create_service(Order, '/order_manager', self.handle_order)
        self.get_logger().info("Order Manager Service Ready.")

    def handle_order(self, request, response):
        if request.request_type == "new":
            if request.table_number in self.orders:
                response.accepted = False
                response.response_message = f"Order for Table {request.table_number} already exists."
            else:
                self.orders[request.table_number] = "pending"
                response.accepted = True
                response.response_message = f"Order for Table {request.table_number} received."

        elif request.request_type == "cancel":
            if request.table_number in self.orders:
                del self.orders[request.table_number]
                response.accepted = True
                response.response_message = f"Order for Table {request.table_number} canceled."
            else:
                response.accepted = False
                response.response_message = f"No active order for Table {request.table_number}."

        elif request.request_type == "status":
            response.accepted = True
            response.response_message = str(self.orders)

        else:
            response.accepted = False
            response.response_message = "Invalid request type."

        return response

def main():
    rclpy.init()
    node = OrderManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
