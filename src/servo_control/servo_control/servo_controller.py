
#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import serial
import time
import os

class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')

        # Declare parameters for serial port and speed limiting
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('step_size', 2)  # Degrees per step
        self.declare_parameter('step_delay', 0.05)  # Delay in seconds
        self.declare_parameter('log_file', 'servo_positions.log')  # Log file for positions

        # Read parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.step_size = self.get_parameter('step_size').value
        self.step_delay = self.get_parameter('step_delay').value
        self.log_file = self.get_parameter('log_file').value

        # Initialize serial connection
        self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
        time.sleep(2)  # Wait for the serial connection to stabilize

        # Load or initialize servo angles
        self.current_pan, self.current_tilt = self.initialize_positions()

        # Subscribe to the topic for desired angles
        self.create_subscription(Vector3, 'desired_angles', self.angle_callback, 10)
        
        # Publisher for current angles
        self.position_publisher = self.create_publisher(Vector3, 'current_angles', 10)

        # Smoothly move to initial position (90, 90)
        self.smooth_move(90, 90)

        self.get_logger().info("ServoController initialized")

    def initialize_positions(self):
        """Initialize servo positions from the log file or reset to 90 degrees."""
        if os.path.exists(self.log_file):
            try:
                with open(self.log_file, 'r') as file:
                    line = file.readline().strip()
                    if line:
                        pan, tilt = map(float, line.split(','))
                        self.get_logger().info(f"Loaded last known positions: Pan={pan}, Tilt={tilt}")
                        return int(pan), int(tilt)
            except Exception as e:
                self.get_logger().warn(f"Failed to read log file: {e}")

        self.get_logger().info("Resetting to default positions: Pan=90, Tilt=90")
        return 90, 90

    def log_positions(self):
        """Log current servo positions to a file."""
        try:
            with open(self.log_file, 'w') as file:
                file.write(f"{self.current_pan},{self.current_tilt}")
                self.get_logger().info(f"Logged positions: Pan={self.current_pan}, Tilt={self.current_tilt}")
        except Exception as e:
            self.get_logger().warn(f"Failed to write log file: {e}")

    def angle_callback(self, msg):
        target_pan = msg.x
        target_tilt = msg.y
        self.get_logger().info(f"Target angles received: Pan={target_pan}, Tilt={target_tilt}")
        self.move_to_angles(target_pan, target_tilt)

    def publish_current_position(self):
        msg = Vector3()
        msg.x = float(self.current_pan)  # Cast to float to ensure compatibility
        msg.y = float(self.current_tilt)  # Cast to float to ensure compatibility
        self.position_publisher.publish(msg)
        self.get_logger().info(f"Published current position: Pan={self.current_pan}, Tilt={self.current_tilt}")


    def smooth_move(self, target_pan, target_tilt):
        """Smoothly move the servo from the current position to the target position."""
        while self.current_pan != target_pan or self.current_tilt != target_tilt:
            if self.current_pan < target_pan:
                self.current_pan += min(self.step_size, target_pan - self.current_pan)
            elif self.current_pan > target_pan:
                self.current_pan -= min(self.step_size, self.current_pan - target_pan)

            if self.current_tilt < target_tilt:
                self.current_tilt += min(self.step_size, target_tilt - self.current_tilt)
            elif self.current_tilt > target_tilt:
                self.current_tilt -= min(self.step_size, self.current_tilt - target_tilt)

            # Send the updated angles to the ESP32
            self.send_to_esp(self.current_pan, self.current_tilt)

            # Log the updated positions
            self.log_positions()

            # Publish the current position
            self.publish_current_position()

            time.sleep(self.step_delay)

    def send_to_esp(self, pan, tilt):
        """Send the pan and tilt commands to the ESP32."""
        command = f"P:{int(pan)},T:{int(tilt)}\n"
        self.ser.write(command.encode())
        self.get_logger().info(f"Sent command: {command.strip()}")

    def move_left(self, step=5):
        """Move the pan to the left by a certain step."""
        target_pan = max(0, self.current_pan - step)  # Prevent pan < 0
        self.smooth_move(target_pan, self.current_tilt)

    def move_right(self, step=5):
        """Move the pan to the right by a certain step."""
        target_pan = min(180, self.current_pan + step)  # Prevent pan > 180
        self.smooth_move(target_pan, self.current_tilt)

    def move_up(self, step=5):
        """Move the tilt up by a certain step."""
        target_tilt = min(180, self.current_tilt + step)  # Prevent tilt > 180
        self.smooth_move(self.current_pan, target_tilt)

    def move_down(self, step=5):
        """Move the tilt down by a certain step."""
        target_tilt = max(0, self.current_tilt - step)  # Prevent tilt < 0
        self.smooth_move(self.current_pan, target_tilt)

    def move_to_angles(self, pan_angle, tilt_angle):
        """Move to specific angles for pan and tilt."""
        pan_angle = max(0, min(180, pan_angle))  # Clamp between 0 and 180
        tilt_angle = max(0, min(180, tilt_angle))  # Clamp between 0 and 180
        self.smooth_move(pan_angle, tilt_angle)


def main(args=None):
    rclpy.init(args=args)
    node = ServoController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
