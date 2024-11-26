#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3

class PositionMonitorNode(Node):
    def __init__(self):
        super().__init__('position_monitor_node')
        self.subscription = self.create_subscription(
            Vector3,
            'current_angles',
            self.position_callback,
            10
        )

    def position_callback(self, msg):
        self.get_logger().info(f"Current position: Pan={msg.x}, Tilt={msg.y}")

def main(args=None):
    rclpy.init(args=args)
    node = PositionMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
