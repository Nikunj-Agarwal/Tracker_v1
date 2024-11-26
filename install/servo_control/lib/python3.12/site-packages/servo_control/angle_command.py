#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3

class AngleCommandNode(Node):
    def __init__(self):
        super().__init__('angle_command_node')
        self.publisher = self.create_publisher(Vector3, 'desired_angles', 10)
        self.timer = self.create_timer(1.0, self.publish_command)

    def publish_command(self):
        # Example command: Incremental movement
        command = Vector3()
        command.x = 90  # Target pan
        command.y = 90  # Target tilt
        command.z = 0   # Incremental flag (z=0 for absolute positioning)
        self.publisher.publish(command)
        self.get_logger().info(f"Published command: Pan={command.x}, Tilt={command.y}, Increment={command.z}")

def main(args=None):
    rclpy.init(args=args)
    node = AngleCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
