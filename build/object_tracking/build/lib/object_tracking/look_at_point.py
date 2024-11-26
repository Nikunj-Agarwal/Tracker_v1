import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Vector3
from tf2_ros import TransformListener, Buffer
import tf2_geometry_msgs  # Import this package
import rclpy.duration
import collections

class LookAtPoint(Node):
    def __init__(self):
        super().__init__('look_at_point')
        # Parameters for camera intrinsics
        self.fx = 612.26
        self.fy = 630.73
        self.cx = 320
        self.cy = 240

        # Servo range limits (example: 0° to 180°)
        self.servo_pan_min = 0
        self.servo_pan_max = 180
        self.servo_tilt_min = 0
        self.servo_tilt_max = 180

        # Maximum change in angles per update
        self.max_angle_change = 2.0  # Adjust as needed

        # Minimum pixel change to trigger movement
        self.min_pixel_change = 30

        # Moving average window size
        self.window_size = 5
        self.ball_positions = collections.deque(maxlen=self.window_size)

        # Subscribe to ball position
        self.ball_subscription = self.create_subscription(
            Point,
            'ball_position',
            self.ball_callback,
            10
        )

        # Publish desired servo angles
        self.servo_publisher = self.create_publisher(Vector3, 'desired_angles', 10)

        # Timer to control the processing rate (10 Hz)
        self.timer = self.create_timer(0.1, self.process_ball_position)  # 0.1 seconds = 10 Hz

        self.ball_position = None
        self.previous_ball_position = None
        self.current_pan_angle = 90.0  # Initial pan angle
        self.current_tilt_angle = 90.0  # Initial tilt angle
        self.get_logger().info("LookAtPoint node initialized.")

    def ball_callback(self, msg):
        self.ball_positions.append((msg.x, msg.y))
        self.ball_position = msg

    def process_ball_position(self):
        if self.ball_position is None or len(self.ball_positions) < self.window_size:
            return

        # Calculate the average position
        avg_x = sum(pos[0] for pos in self.ball_positions) / self.window_size
        avg_y = sum(pos[1] for pos in self.ball_positions) / self.window_size

        if self.previous_ball_position is not None:
            pixel_change = abs(avg_x - self.previous_ball_position[0]) + abs(avg_y - self.previous_ball_position[1])
            if pixel_change <= self.min_pixel_change:
                return

        # Adjust for origin center
        u_center = avg_x - self.cx
        v_center = avg_y - self.cy
        z = 1.5  # Assume a fixed depth; replace with actual depth if available

        # Compute the 3D point in camera coordinates
        x = (u_center * z) / self.fx
        y = (v_center * z) / self.fy

        # Map the 3D point to servo angles directly
        pan_angle = self.map_to_servo_range(
            x,
            src_min=-1,
            src_max=1,
            dest_min=self.servo_pan_min,
            dest_max=self.servo_pan_max
        )
        tilt_angle = self.map_to_servo_range(
            y,
            src_min=-1,
            src_max=1,
            dest_min=self.servo_tilt_min,
            dest_max=self.servo_tilt_max
        )

        # Clamp the angle changes
        pan_angle = self.clamp_angle_change(self.current_pan_angle, pan_angle)
        tilt_angle = self.clamp_angle_change(self.current_tilt_angle, tilt_angle)

        # Round the angles to the nearest whole number and convert to floats
        pan_angle = float(round(pan_angle))
        tilt_angle = float(round(tilt_angle))

        # Update the current angles
        self.current_pan_angle = pan_angle
        self.current_tilt_angle = tilt_angle

        # Update the previous ball position
        self.previous_ball_position = (avg_x, avg_y)

        # Publish the servo angles
        angles = Vector3()
        angles.x = pan_angle
        angles.y = tilt_angle
        angles.z = 0.0  # No rotation around Z-axis
        self.servo_publisher.publish(angles)
        self.get_logger().info(f"Published servo angles: Pan = {pan_angle}, Tilt = {tilt_angle}")

    def clamp_angle_change(self, current_angle, new_angle):
        change = new_angle - current_angle
        if abs(change) > self.max_angle_change:
            change = self.max_angle_change if change > 0 else -self.max_angle_change
        return current_angle + change

    @staticmethod
    def map_to_servo_range(value, src_min, src_max, dest_min, dest_max):
        src_span = src_max - src_min
        dest_span = dest_max - dest_min
        scaled_value = (value - src_min) / src_span
        return dest_min + (scaled_value * dest_span)

def main(args=None):
    rclpy.init(args=args)
    node = LookAtPoint()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
