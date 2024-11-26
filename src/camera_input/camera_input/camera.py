#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # Declare parameters
        self.declare_parameter('video_device', '/dev/video2')
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('frame_rate', 60)

        # Read parameters
        video_device = self.get_parameter('video_device').value
        frame_width = self.get_parameter('frame_width').value
        frame_height = self.get_parameter('frame_height').value
        frame_rate = self.get_parameter('frame_rate').value

        # Initialize OpenCV video capture
        self.cap = cv2.VideoCapture(video_device)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)
        self.cap.set(cv2.CAP_PROP_FPS, frame_rate)

        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open camera: {video_device}")
            rclpy.shutdown()

        # Publisher for image frames
        self.image_publisher = self.create_publisher(Image, 'camera/image_raw', 10)
        self.bridge = CvBridge()

        # Timer to read and publish frames
        self.timer = self.create_timer(1.0 / frame_rate, self.capture_frame)
        self.get_logger().info(f"CameraNode initialized using device {video_device}")

    def capture_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to capture frame")
            return

        # Flip the frame vertically
        frame = cv2.flip(frame, 0)

        # Optional: Show the frame using OpenCV
        cv2.imshow("Camera Feed", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to quit the viewer
            rclpy.shutdown()

        # Convert frame to ROS2 Image message and publish
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.image_publisher.publish(msg)

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
