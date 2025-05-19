#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyFirstNode(Node):
    def __init__(self):
        super().__init__('my_first_node')
        self.counter = 0
        self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("Hi " + str(self.counter))
        self.counter += 1
        if self.counter > 10:
            self.get_logger().info("Goodbye")
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    # Create a node
    node = MyFirstNode()
    node.get_logger().info("Node has been created")
    node.get_logger().info("Node is running")


    # Spin the node
    rclpy.spin(node)

    # Destroy the node
    rclpy.shutdown()

if __name__ == '__main__':
    main()